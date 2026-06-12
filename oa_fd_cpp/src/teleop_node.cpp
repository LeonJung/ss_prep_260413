// teleop_node.cpp — enactic-style bimanual force-feedback bilateral.
// See teleop_node.hpp.
//
// Per joint, MIT command  tau = Kp(q_ref-q) + Kd(dq_ref-dq) + g(q) + friction(q̇)
//   ACTIVE   : q_ref/dq_ref = mirrored peer state (leader<->follower cross-couple)
//   PAUSED   : q_ref = home, dq_ref = 0
//   HOMING   : q_ref = quintic ramp (start->home), dq_ref = 0
//   FREEDRIVE: Kp=Kd=0  -> tau = g(q) + friction (gravity/friction balanced)
// The Kp/Kd impedance is executed motor-side (sent inside the MIT packet);
// g(q)+friction go in the torque-feedforward field.

#include "oa_fd_cpp/teleop_node.hpp"

#include <algorithm>
#include <cmath>

namespace oa_fd {

namespace {
inline double quintic_ease(double a) {
  a = std::clamp(a, 0.0, 1.0);
  return 10 * a*a*a - 15 * a*a*a*a + 6 * a*a*a*a*a;
}
}  // namespace

TeleopNode::TeleopNode(const Options& opts)
    : rclcpp::Node("oa_fd_node"), opts_(opts) {
  rt_cfg_.enabled = opts.use_rt;
  rt_cfg_.priority = opts.rt_priority;
  rt_cfg_.cpu_affinity = opts.rt_cpu;

  if (!opts.config_path.empty()) {
    if (!load_config(opts.config_path, cfg_))
      RCLCPP_WARN(get_logger(), "config load failed; using defaults");
    else
      RCLCPP_INFO(get_logger(), "config: %s", opts.config_path.c_str());
  }
  if (!opts.urdf_override.empty()) {
    cfg_.gravity.urdf = opts.urdf_override;
    RCLCPP_INFO(get_logger(), "urdf (override): %s", cfg_.gravity.urdf.c_str());
  }

  rclcpp::QoS state_qos{10};
  rclcpp::QoS latched{1};
  latched.reliable().transient_local();

  mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/oa/mode", latched);
  mode_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/oa/mode", latched,
      std::bind(&TeleopNode::mode_cb, this, std::placeholders::_1));

  right_.name = "right";
  right_.mirror = cfg_.mirror_right;
  right_.leader   = std::make_unique<OaxArm>(cfg_.can_leader_right,   cfg_.can_fd, cfg_.recv_timeout_us);
  right_.follower = std::make_unique<OaxArm>(cfg_.can_follower_right, cfg_.can_fd, cfg_.recv_timeout_us);
  right_.leader_pub   = create_publisher<sensor_msgs::msg::JointState>("/oa/leader_right/joint_state", state_qos);
  right_.follower_pub = create_publisher<sensor_msgs::msg::JointState>("/oa/follower_right/joint_state", state_qos);

  left_.name = "left";
  left_.mirror = cfg_.mirror_left;
  left_.leader   = std::make_unique<OaxArm>(cfg_.can_leader_left,   cfg_.can_fd, cfg_.recv_timeout_us);
  left_.follower = std::make_unique<OaxArm>(cfg_.can_follower_left, cfg_.can_fd, cfg_.recv_timeout_us);
  left_.leader_pub   = create_publisher<sensor_msgs::msg::JointState>("/oa/leader_left/joint_state", state_qos);
  left_.follower_pub = create_publisher<sensor_msgs::msg::JointState>("/oa/follower_left/joint_state", state_qos);

  // Select active pair(s): --arms right|left|both
  if (opts_.arms == "right")      pairs_ = {&right_};
  else if (opts_.arms == "left")  pairs_ = {&left_};
  else                            pairs_ = {&right_, &left_};

  // Select role(s) within each pair: --role leader|follower|both.
  // With a single role, ACTIVE coupling is disabled for the driven arm
  // (its peer state is invalid) — it behaves like FREEDRIVE instead.
  drive_leader_   = (opts_.role != "follower");
  drive_follower_ = (opts_.role != "leader");
  RCLCPP_INFO(get_logger(), "active arms: %s (%zu pair), role: %s",
              opts_.arms.c_str(), pairs_.size(), opts_.role.c_str());

  // Safe startup: FREEDRIVE (gravity only). Operator verifies, then explicitly
  // commands HOMING/PAUSED/ACTIVE. (No auto-home: a stiff pull-to-home on launch
  // can slam the arm if it starts far from home.)
  publish_mode(MODE_FREEDRIVE);
}

TeleopNode::~TeleopNode() { stop(); }

bool TeleopNode::connect() {
  if (cfg_.gravity.enabled) {
    GravityCfg gr = cfg_.gravity;
    gr.vec = cfg_.grav_vec_right; gr.root_link = cfg_.root_link_right; gr.tip_link = cfg_.tip_link_right;
    GravityCfg gl = cfg_.gravity;
    gl.vec = cfg_.grav_vec_left;  gl.root_link = cfg_.root_link_left;  gl.tip_link = cfg_.tip_link_left;
    bool okr = grav_right_.load(gr);
    bool okl = grav_left_.load(gl);
    if (!okr || !okl)
      RCLCPP_WARN(get_logger(), "gravity comp OFF (URDF missing/invalid) — arm will sag!");
  }
  right_.grav = &grav_right_;
  left_.grav  = &grav_left_;

  for (Pair* p : pairs_) {
    std::vector<OaxArm*> arms;
    if (drive_leader_)   arms.push_back(p->leader.get());
    if (drive_follower_) arms.push_back(p->follower.get());
    for (OaxArm* a : arms) {
      if (!a->init())   { RCLCPP_ERROR(get_logger(), "init failed %s", a->iface().c_str()); return false; }
      if (!a->enable()) { RCLCPP_ERROR(get_logger(), "enable failed %s", a->iface().c_str()); return false; }
      RCLCPP_INFO(get_logger(), "arm up: %s", a->iface().c_str());
    }
  }
  return true;
}

void TeleopNode::run() {
  if (running_.exchange(true)) return;
  control_thread_ = std::thread(&TeleopNode::control_loop, this);
}

void TeleopNode::stop() {
  running_ = false;
  if (control_thread_.joinable()) control_thread_.join();
  for (Pair* p : pairs_) {
    if (drive_leader_ && p->leader)     p->leader->disable();
    if (drive_follower_ && p->follower) p->follower->disable();
  }
}

void TeleopNode::mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < 3) return;
  std::lock_guard<std::mutex> lk(mode_mtx_);
  mode_state_ = static_cast<int>(msg->data[0]);
  mode_t_start_ = msg->data[1];
  mode_duration_ = msg->data[2];
}

void TeleopNode::publish_mode(int mode, double t_start, double duration) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {static_cast<double>(mode), t_start, duration};
  mode_pub_->publish(msg);
}

void TeleopNode::friction(const Vec7& qd, Vec7& f) const {
  for (int i = 0; i < DOF; ++i)
    f[i] = cfg_.fric_Fc[i] * std::tanh(cfg_.fric_k[i] * qd[i])
         + cfg_.fric_Fv[i] * qd[i] + cfg_.fric_Fo[i];
}

void TeleopNode::compute_pair(Pair& p, int mode, double now_sec,
                              double h_t_start, double h_duration,
                              MitCmd& lc, MitCmd& fc) {
  Vec7 gl{}, gf{}, frl{}, frf{};
  if (p.grav) { p.grav->gravity(p.lq, gl); p.grav->gravity(p.fq, gf); }
  friction(p.lqd, frl);
  friction(p.fqd, frf);

  // mirror peer into local frame (delta about home)
  Vec7 f_in_l{}, fd_in_l{}, l_in_f{}, ld_in_f{};
  for (int i = 0; i < DOF; ++i) {
    f_in_l[i]  = cfg_.home[i] + p.mirror[i] * (p.fq[i] - cfg_.home[i]);
    fd_in_l[i] = p.mirror[i] * p.fqd[i];
    l_in_f[i]  = cfg_.home[i] + p.mirror[i] * (p.lq[i] - cfg_.home[i]);
    ld_in_f[i] = p.mirror[i] * p.lqd[i];
  }

  const double alpha = (h_duration > 0.0)
      ? std::clamp((now_sec - h_t_start) / h_duration, 0.0, 1.0) : 1.0;
  const double s = quintic_ease(alpha);

  for (int i = 0; i < DOF; ++i) {
    // feedforward (gravity + friction comp) always applied
    lc.tau[i] = gl[i] + frl[i];
    fc.tau[i] = gf[i] + frf[i];

    switch (mode) {
      case MODE_ACTIVE:
        // Cross-coupling needs BOTH sides' live state. With --role leader or
        // follower the peer state is stale -> coupling would slam; fall back
        // to gravity-only (freedrive-like) for the driven arm.
        if (drive_leader_ && drive_follower_) {
          lc.kp[i] = cfg_.Kp[i]; lc.kd[i] = cfg_.Kd[i];
          lc.pos[i] = f_in_l[i];  lc.vel[i] = fd_in_l[i];
          fc.kp[i] = cfg_.Kp[i]; fc.kd[i] = cfg_.Kd[i];
          fc.pos[i] = l_in_f[i];  fc.vel[i] = ld_in_f[i];
        } else {
          lc.kp[i] = 0.0; lc.kd[i] = 0.0; lc.pos[i] = 0.0; lc.vel[i] = 0.0;
          fc.kp[i] = 0.0; fc.kd[i] = 0.0; fc.pos[i] = 0.0; fc.vel[i] = 0.0;
        }
        break;
      case MODE_PAUSED:
        // hold the pose captured at PAUSED entry (NOT a fixed home) -> no slam
        lc.kp[i] = cfg_.Kp[i]; lc.kd[i] = cfg_.Kd[i]; lc.pos[i] = p.l_hold[i]; lc.vel[i] = 0.0;
        fc.kp[i] = cfg_.Kp[i]; fc.kd[i] = cfg_.Kd[i]; fc.pos[i] = p.f_hold[i]; fc.vel[i] = 0.0;
        break;
      case MODE_HOMING: {
        double lt = p.l_home_start[i] + s * (cfg_.home[i] - p.l_home_start[i]);
        double ft = p.f_home_start[i] + s * (cfg_.home[i] - p.f_home_start[i]);
        lc.kp[i] = cfg_.Kp[i]; lc.kd[i] = cfg_.Kd[i]; lc.pos[i] = lt; lc.vel[i] = 0.0;
        fc.kp[i] = cfg_.Kp[i]; fc.kd[i] = cfg_.Kd[i]; fc.pos[i] = ft; fc.vel[i] = 0.0;
        break;
      }
      case MODE_FREEDRIVE:
      default:
        lc.kp[i] = 0.0; lc.kd[i] = 0.0; lc.pos[i] = 0.0; lc.vel[i] = 0.0;
        fc.kp[i] = 0.0; fc.kd[i] = 0.0; fc.pos[i] = 0.0; fc.vel[i] = 0.0;
        break;
    }
    // clamp feedforward torque (Kp/Kd part is bounded motor-side by limits)
    lc.tau[i] = std::clamp(lc.tau[i], -cfg_.torque_limit[i], cfg_.torque_limit[i]);
    fc.tau[i] = std::clamp(fc.tau[i], -cfg_.torque_limit[i], cfg_.torque_limit[i]);
  }
}

void TeleopNode::publish_pair(Pair& p) {
  auto fill = [this](const Vec7& q, const Vec7& qd, const Vec7& tau,
                     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub) {
    sensor_msgs::msg::JointState m;
    m.header.stamp = this->now();
    m.name = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
    m.position.assign(q.begin(), q.end());
    m.velocity.assign(qd.begin(), qd.end());
    m.effort.assign(tau.begin(), tau.end());
    pub->publish(m);
  };
  fill(p.lq, p.lqd, p.ltau, p.leader_pub);
  fill(p.fq, p.fqd, p.ftau, p.follower_pub);
}

void TeleopNode::control_loop() {
  init_rt_thread(rt_cfg_);
  JitterTracker jitter(std::chrono::microseconds(
      static_cast<int64_t>(cfg_.timestep * 1e6)));
  auto deadline = now_monotonic();
  const auto period = jitter.target_period();
  RCLCPP_INFO(get_logger(), "control_loop  period=%ld us  rt=%s",
              (long)(period.count() / 1000), rt_cfg_.enabled ? "ON" : "OFF");

  int prev_mode = MODE_PAUSED;
  int log_counter = 0;

  if (cfg_.auto_home_on_start) {
    publish_mode(MODE_HOMING, this->now().seconds(), cfg_.homing_duration);
    RCLCPP_INFO(get_logger(), "auto_home_on_start -> HOMING %.1fs", cfg_.homing_duration);
  }

  while (running_) {
    jitter.tick(now_monotonic());
    const double now_sec = this->now().seconds();

    for (Pair* p : pairs_) {
      if (drive_leader_)   p->leader->read(p->lq, p->lqd, p->ltau);
      if (drive_follower_) p->follower->read(p->fq, p->fqd, p->ftau);
    }

    int mode; double h_t_start, h_duration;
    {
      std::lock_guard<std::mutex> lk(mode_mtx_);
      mode = mode_state_; h_t_start = mode_t_start_; h_duration = mode_duration_;
    }

    if (mode != prev_mode) {
      RCLCPP_INFO(get_logger(), "mode %d -> %d", prev_mode, mode);
      if (mode == MODE_HOMING)
        for (Pair* p : pairs_) { p->l_home_start = p->lq; p->f_home_start = p->fq; }
      else if (mode == MODE_PAUSED)   // capture current pose -> hold in place (no slam)
        for (Pair* p : pairs_) { p->l_hold = p->lq; p->f_hold = p->fq; }
    }

    double worst_err = 0.0;
    for (Pair* p : pairs_) {
      MitCmd lc, fc;
      compute_pair(*p, mode, now_sec, h_t_start, h_duration, lc, fc);
      if (drive_leader_)   p->leader->write_mit(lc.kp, lc.kd, lc.pos, lc.vel, lc.tau);
      if (drive_follower_) p->follower->write_mit(fc.kp, fc.kd, fc.pos, fc.vel, fc.tau);
      if (mode == MODE_HOMING)
        for (int i = 0; i < DOF; ++i) {
          if (drive_leader_)
            worst_err = std::max(worst_err, std::abs(p->lq[i] - cfg_.home[i]));
          if (drive_follower_)
            worst_err = std::max(worst_err, std::abs(p->fq[i] - cfg_.home[i]));
        }
      publish_pair(*p);
    }

    if (mode == MODE_HOMING) {
      const double alpha = (h_duration > 0.0)
          ? std::clamp((now_sec - h_t_start) / h_duration, 0.0, 1.0) : 1.0;
      if (alpha >= 1.0 && worst_err < 0.05) {
        publish_mode(MODE_PAUSED);
        RCLCPP_INFO(get_logger(), "HOMING complete (err=%.3f) -> PAUSED", worst_err);
      }
    }

    if (++log_counter >= static_cast<int>(1.0 / cfg_.timestep)) {
      log_counter = 0;
      // gravity sanity: print leader q and computed g(q) for each ACTIVE pair.
      // ~0 at a near-vertical pose is normal; horizontal poses load j1/j2/j4.
      for (Pair* p : pairs_) {
        // show the driven side (leader if active, else follower)
        const Vec7& q = drive_leader_ ? p->lq : p->fq;
        const char* side = drive_leader_ ? "leader" : "follower";
        Vec7 g{};
        const bool gon = (p->grav && p->grav->ok());
        if (p->grav) p->grav->gravity(q, g);
        const int rc = p->grav ? p->grav->last_rc() : -1;
        RCLCPP_INFO(get_logger(),
          "[DIAG] mode=%d %s/%s grav=%s rc=%d | q=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f] "
          "g=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
          mode, p->name.c_str(), side, gon ? "ON" : "OFF", rc,
          q[0], q[1], q[2], q[3], q[4], q[5], q[6],
          g[0], g[1], g[2], g[3], g[4], g[5], g[6]);
      }
      RCLCPP_INFO(get_logger(), "[DIAG] %s", jitter.log_line("").c_str());
    }

    prev_mode = mode;
    deadline += period;
    sleep_until(deadline);
  }
  RCLCPP_INFO(get_logger(), "control_loop exited");
}

}  // namespace oa_fd
