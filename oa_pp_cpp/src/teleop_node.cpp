// teleop_node.cpp — bimanual position-position bilateral. See teleop_node.hpp.
//
// Control law (per pair, ported from ur10e_teleop_real_cpp + gravity FF):
//   leader (operator arm):
//     ACTIVE : tau = g(q) + ramp * sign(raw)*max(0,|raw|-deadband)
//              raw = KP_BI*(fq_mirrored - lq) + KD_BI*(fqd_mirrored - lqd)
//     PAUSED : tau = g(q) + KP_HOLD*(home - q) - KD_HOLD*qd
//   follower (environment arm):
//     ACTIVE : tau = g(q) + KP_TRACK*(lq_mirrored - fq) + KD_TRACK*(lqd_mirrored - fqd)
//     PAUSED : tau = g(q) + KP_HOLD*(home - q) - KD_HOLD*qd
//   HOMING  : both arms PD to quintic-ramped target (start->home) + g(q)
//   FREEDRIVE: tau = g(q) only (gravity-balanced, backdrivable)
// All commands sent as pure torque (MIT kp=kd=0).

#include "oa_pp_cpp/teleop_node.hpp"

#include <algorithm>
#include <cmath>

namespace oa_pp {

namespace {
inline double quintic_ease(double a) {
  a = std::clamp(a, 0.0, 1.0);
  return 10 * a*a*a - 15 * a*a*a*a + 6 * a*a*a*a*a;
}
constexpr double ACTIVE_RAMP = 0.5;   // soft-start window [s]
}  // namespace

TeleopNode::TeleopNode(const Options& opts)
    : rclcpp::Node("oa_pp_node"), opts_(opts) {
  rt_cfg_.enabled = opts.use_rt;
  rt_cfg_.priority = opts.rt_priority;
  rt_cfg_.cpu_affinity = opts.rt_cpu;

  if (!opts.config_path.empty()) {
    if (!load_config(opts.config_path, cfg_))
      RCLCPP_WARN(get_logger(), "config load failed; using defaults");
    else
      RCLCPP_INFO(get_logger(), "config: %s", opts.config_path.c_str());
  }

  rclcpp::QoS state_qos{10};
  rclcpp::QoS latched{1};
  latched.reliable().transient_local();

  mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/oa/mode", latched);
  mode_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/oa/mode", latched,
      std::bind(&TeleopNode::mode_cb, this, std::placeholders::_1));

  // ---- build pairs ----
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

  publish_mode(MODE_PAUSED);
}

TeleopNode::~TeleopNode() { stop(); }

bool TeleopNode::connect() {
  if (cfg_.gravity.enabled && !grav_.load(cfg_.gravity))
    RCLCPP_WARN(get_logger(), "gravity comp OFF (URDF missing/invalid) — arm will sag!");

  for (Pair* p : {&right_, &left_}) {
    for (OaxArm* a : {p->leader.get(), p->follower.get()}) {
      if (!a->init()) {
        RCLCPP_ERROR(get_logger(), "init failed on %s", a->iface().c_str());
        return false;
      }
      if (!a->enable()) {
        RCLCPP_ERROR(get_logger(), "enable failed on %s", a->iface().c_str());
        return false;
      }
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
  for (Pair* p : {&right_, &left_}) {
    if (p->leader)   p->leader->disable();
    if (p->follower) p->follower->disable();
  }
}

void TeleopNode::mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < 3) return;
  std::lock_guard<std::mutex> lk(mode_mtx_);
  mode_state_    = static_cast<int>(msg->data[0]);
  mode_t_start_  = msg->data[1];
  mode_duration_ = msg->data[2];
}

void TeleopNode::publish_mode(int mode, double t_start, double duration) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {static_cast<double>(mode), t_start, duration};
  mode_pub_->publish(msg);
}

void TeleopNode::compute_pair(Pair& p, int mode, double now_sec,
                              double active_t_start, double h_t_start,
                              double h_duration, Vec7& tau_leader,
                              Vec7& tau_follower) {
  Vec7 gl{}, gf{};
  grav_.gravity(p.lq, gl);
  grav_.gravity(p.fq, gf);

  // mirror the peer into the local frame (delta about home).
  Vec7 f_in_l{}, fd_in_l{}, l_in_f{}, ld_in_f{};
  for (int i = 0; i < DOF; ++i) {
    f_in_l[i]  = cfg_.home[i] + p.mirror[i] * (p.fq[i]  - cfg_.home[i]);
    fd_in_l[i] = p.mirror[i] * p.fqd[i];
    l_in_f[i]  = cfg_.home[i] + p.mirror[i] * (p.lq[i]  - cfg_.home[i]);
    ld_in_f[i] = p.mirror[i] * p.lqd[i];
  }

  switch (mode) {
    case MODE_ACTIVE: {
      const double ramp = std::min(1.0, (now_sec - active_t_start) / ACTIVE_RAMP);
      for (int i = 0; i < DOF; ++i) {
        // leader: bilateral spring from follower, continuous deadband
        double raw = cfg_.leader_kp_bi[i] * (f_in_l[i] - p.lq[i])
                   + cfg_.leader_kd_bi[i] * (fd_in_l[i] - p.lqd[i]);
        double excess = std::max(0.0, std::abs(raw) - cfg_.leader_deadband[i]);
        tau_leader[i] = gl[i] + ramp * std::copysign(excess, raw);
        // follower: stiff PD tracking of leader
        tau_follower[i] = gf[i]
            + cfg_.follower_kp_track[i] * (l_in_f[i] - p.fq[i])
            + cfg_.follower_kd_track[i] * (ld_in_f[i] - p.fqd[i]);
      }
      break;
    }
    case MODE_PAUSED:
      for (int i = 0; i < DOF; ++i) {
        tau_leader[i]   = gl[i] + cfg_.leader_kp_hold[i]   * (cfg_.home[i] - p.lq[i]) - cfg_.leader_kd_hold[i]   * p.lqd[i];
        tau_follower[i] = gf[i] + cfg_.follower_kp_hold[i] * (cfg_.home[i] - p.fq[i]) - cfg_.follower_kd_hold[i] * p.fqd[i];
      }
      break;
    case MODE_HOMING: {
      const double alpha = (h_duration > 0.0)
          ? std::clamp((now_sec - h_t_start) / h_duration, 0.0, 1.0) : 1.0;
      const double s = quintic_ease(alpha);
      for (int i = 0; i < DOF; ++i) {
        double lt = p.l_home_start[i] + s * (cfg_.home[i] - p.l_home_start[i]);
        double ft = p.f_home_start[i] + s * (cfg_.home[i] - p.f_home_start[i]);
        tau_leader[i]   = gl[i] + cfg_.leader_kp_hold[i]   * (lt - p.lq[i]) - cfg_.leader_kd_hold[i]   * p.lqd[i];
        tau_follower[i] = gf[i] + cfg_.follower_kp_hold[i] * (ft - p.fq[i]) - cfg_.follower_kd_hold[i] * p.fqd[i];
      }
      break;
    }
    case MODE_FREEDRIVE:
    default:
      for (int i = 0; i < DOF; ++i) { tau_leader[i] = gl[i]; tau_follower[i] = gf[i]; }
      break;
  }

  // clamp
  for (int i = 0; i < DOF; ++i) {
    tau_leader[i]   = std::clamp(tau_leader[i],   -cfg_.torque_limit[i], cfg_.torque_limit[i]);
    tau_follower[i] = std::clamp(tau_follower[i], -cfg_.torque_limit[i], cfg_.torque_limit[i]);
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
  const double startup = this->now().seconds();
  double active_t_start = startup;

  if (cfg_.auto_home_on_start) {
    publish_mode(MODE_HOMING, this->now().seconds(), cfg_.homing_duration);
    RCLCPP_INFO(get_logger(), "auto_home_on_start -> HOMING %.1fs", cfg_.homing_duration);
  }

  while (running_) {
    jitter.tick(now_monotonic());
    const double now_sec = this->now().seconds();

    // read all 4 arms
    for (Pair* p : {&right_, &left_}) {
      p->leader->read(p->lq, p->lqd, p->ltau);
      p->follower->read(p->fq, p->fqd, p->ftau);
    }

    // snapshot mode
    int mode; double h_t_start, h_duration;
    {
      std::lock_guard<std::mutex> lk(mode_mtx_);
      mode = mode_state_; h_t_start = mode_t_start_; h_duration = mode_duration_;
    }

    // transitions
    if (mode != prev_mode) {
      RCLCPP_INFO(get_logger(), "mode %d -> %d", prev_mode, mode);
      if (mode == MODE_HOMING) {
        for (Pair* p : {&right_, &left_}) { p->l_home_start = p->lq; p->f_home_start = p->fq; }
      } else if (mode == MODE_ACTIVE) {
        active_t_start = now_sec;
      }
    }

    // compute + write per pair
    double worst_err = 0.0;
    for (Pair* p : {&right_, &left_}) {
      Vec7 tl{}, tf{};
      compute_pair(*p, mode, now_sec, active_t_start, h_t_start, h_duration, tl, tf);
      p->leader->write_torque(tl);
      p->follower->write_torque(tf);
      if (mode == MODE_HOMING)
        for (int i = 0; i < DOF; ++i) {
          worst_err = std::max(worst_err, std::abs(p->lq[i] - cfg_.home[i]));
          worst_err = std::max(worst_err, std::abs(p->fq[i] - cfg_.home[i]));
        }
      publish_pair(*p);
    }

    // homing completion
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
      RCLCPP_INFO(get_logger(), "[DIAG] mode=%d  %s", mode, jitter.log_line("").c_str());
    }

    prev_mode = mode;
    deadline += period;
    sleep_until(deadline);
  }
  RCLCPP_INFO(get_logger(), "control_loop exited");
}

}  // namespace oa_pp
