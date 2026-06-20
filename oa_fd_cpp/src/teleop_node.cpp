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
#include <cstdio>
#include <cstdlib>

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

  // Load the 4 per-arm config files (each self-contained). Globals (g_) are
  // refreshed from each; they carry identical loop params.
  auto load1 = [&](const std::string& path, ArmCfg& a, const char* tag) {
    if (path.empty()) { RCLCPP_WARN(get_logger(), "%s: no config path", tag); return; }
    if (load_arm_config(path, a, g_)) RCLCPP_INFO(get_logger(), "%s: %s", tag, path.c_str());
    else RCLCPP_WARN(get_logger(), "%s: load failed (%s) — defaults", tag, path.c_str());
  };
  load1(opts.cfg_leader_left,    cfg_lead_left_,  "leader_left");
  load1(opts.cfg_leader_right,   cfg_lead_right_, "leader_right");
  load1(opts.cfg_follower_left,  cfg_foll_left_,  "follower_left");
  load1(opts.cfg_follower_right, cfg_foll_right_, "follower_right");
  // per-arm gravity-URDF overrides (launch passes the cali file paths)
  if (!opts.urdf_leader_left.empty())     cfg_lead_left_.grav_urdf  = opts.urdf_leader_left;
  if (!opts.urdf_leader_right.empty())    cfg_lead_right_.grav_urdf = opts.urdf_leader_right;
  if (!opts.urdf_follower_left.empty())   cfg_foll_left_.grav_urdf  = opts.urdf_follower_left;
  if (!opts.urdf_follower_right.empty())  cfg_foll_right_.grav_urdf = opts.urdf_follower_right;

  rclcpp::QoS state_qos{10};
  rclcpp::QoS latched{1};
  latched.reliable().transient_local();

  mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/oa/mode", latched);
  mode_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/oa/mode", latched,
      std::bind(&TeleopNode::mode_cb, this, std::placeholders::_1));

  right_.name = "right";
  right_.lead_cfg = &cfg_lead_right_;  right_.foll_cfg = &cfg_foll_right_;
  right_.leader   = std::make_unique<OaxArm>(cfg_lead_right_.can, g_.can_fd, g_.recv_timeout_us);
  right_.follower = std::make_unique<OaxArm>(cfg_foll_right_.can, g_.can_fd, g_.recv_timeout_us);
  right_.leader_pub   = create_publisher<sensor_msgs::msg::JointState>("/oa/leader_right/joint_state", state_qos);
  right_.follower_pub = create_publisher<sensor_msgs::msg::JointState>("/oa/follower_right/joint_state", state_qos);

  left_.name = "left";
  left_.lead_cfg = &cfg_lead_left_;  left_.foll_cfg = &cfg_foll_left_;
  left_.leader   = std::make_unique<OaxArm>(cfg_lead_left_.can, g_.can_fd, g_.recv_timeout_us);
  left_.follower = std::make_unique<OaxArm>(cfg_foll_left_.can, g_.can_fd, g_.recv_timeout_us);
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
  // Build one gravity model per arm from its ArmCfg.
  auto mk = [](const ArmCfg& a) {
    GravityCfg c; c.enabled = a.grav_enabled; c.urdf = a.grav_urdf;
    c.root_link = a.root_link; c.tip_link = a.tip_link; c.vec = a.grav_vec;
    c.scale = a.grav_scale; c.scale_joints = a.grav_scale_joints; return c;
  };
  bool ok = true;
  ok &= grav_lead_left_.load (mk(cfg_lead_left_));
  ok &= grav_foll_left_.load (mk(cfg_foll_left_));
  ok &= grav_lead_right_.load(mk(cfg_lead_right_));
  ok &= grav_foll_right_.load(mk(cfg_foll_right_));
  RCLCPP_INFO(get_logger(), "gravity urdf  LL:%s LR:%s FL:%s FR:%s",
              cfg_lead_left_.grav_urdf.c_str(), cfg_lead_right_.grav_urdf.c_str(),
              cfg_foll_left_.grav_urdf.c_str(), cfg_foll_right_.grav_urdf.c_str());
  if (!ok)
    RCLCPP_WARN(get_logger(), "gravity comp OFF (URDF missing/invalid) — arm will sag!");
  left_.grav_lead  = &grav_lead_left_;   left_.grav_foll  = &grav_foll_left_;
  right_.grav_lead = &grav_lead_right_;  right_.grav_foll = &grav_foll_right_;

  // Hard MECHANICAL position clamp in the driver (physical range; WIDER than
  // the operator fences in joint_limits). Out-of-range cmd -> motor fault ->
  // torque drop -> limp arm, so clamp every command here. Left mechanical;
  // right = y-mirror (q1,q2,q3,q5,q7 flip).
  const Vec7 MECH_LO_L = {-3.34, -3.27, -1.57, 0.0, -1.5, -0.75, -1.4};
  const Vec7 MECH_HI_L = { 0.91,  0.13,  1.57, 1.8,  1.5,  0.75,  1.4};
  const double MMIR[DOF] = {-1, -1, -1, 1, -1, 1, -1};
  for (Pair* p : pairs_) {
    Vec7 mlo = MECH_LO_L, mhi = MECH_HI_L;
    if (p->name == "right")
      for (int i = 0; i < DOF; ++i)
        if (MMIR[i] < 0) { mlo[i] = -MECH_HI_L[i]; mhi[i] = -MECH_LO_L[i]; }

    std::vector<OaxArm*> arms;
    if (drive_leader_)   arms.push_back(p->leader.get());
    if (drive_follower_) arms.push_back(p->follower.get());
    for (OaxArm* a : arms) {
      if (!a->init())   { RCLCPP_ERROR(get_logger(), "init failed %s", a->iface().c_str()); return false; }
      if (!a->enable()) { RCLCPP_ERROR(get_logger(), "enable failed %s", a->iface().c_str()); return false; }
      a->set_pos_limits(mlo, mhi);
      // GATE: no torque until every motor proves live telemetry. Without this
      // a cold first launch can run with q=0 -> gravity comp silently dead
      // on q1/q2 ("first launch no torque, relaunch fixes it").
      if (!a->verify_state()) {
        RCLCPP_ERROR(get_logger(),
                     "%s: motors enabled but state telemetry missing — REFUSING "
                     "to start (would run gravity comp on q=0). Check CAN / "
                     "power, then relaunch.", a->iface().c_str());
        return false;
      }
      RCLCPP_INFO(get_logger(), "arm up: %s (7/7 motors reporting)", a->iface().c_str());
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

void TeleopNode::friction(const ArmCfg& a, const Vec7& qd, Vec7& f) const {
  const Vec7& Fc = a.fric_Fc; const Vec7& k  = a.fric_k;
  const Vec7& Fv = a.fric_Fv; const Vec7& Fo = a.fric_Fo;
  const Vec7& vs = a.fric_v_start; const Vec7& vf = a.fric_v_full;
  for (int i = 0; i < DOF; ++i) {
    double raw = Fc[i] * std::tanh(k[i] * qd[i]) + Fv[i] * qd[i] + Fo[i];
    // velocity gate: no comp at standstill (kills the negative-damping
    // runaway in FREEDRIVE), full comp once clearly moving.
    if (vf[i] > vs[i]) {
      const double g = std::clamp(
          (std::abs(qd[i]) - vs[i]) / (vf[i] - vs[i]), 0.0, 1.0);
      raw *= g;
    }
    f[i] = raw;
  }
}

void TeleopNode::compute_pair(Pair& p, int mode, double now_sec,
                              double h_t_start, double h_duration,
                              MitCmd& lc, MitCmd& fc) {
  // Gravity with per-joint axis mirror: g_side = m * gravity(m * q).
  // (Right arm is the left arm's mirror; m flips the mirrored joints' angle
  //  on the way in and their torque on the way out. m=all-1 on the left.)
  const ArmCfg& L = *p.lead_cfg;   // leader arm params
  const ArmCfg& F = *p.foll_cfg;   // follower arm params

  // Gravity with per-joint axis mirror: g = m * gravity(m * q). Each arm uses
  // ITS OWN mirror (leader and follower of a pair share the side's mounting,
  // so usually the same, but read per-arm).
  Vec7 gl{}, gf{}, frl{}, frf{};
  {
    Vec7 lqm, fqm;
    for (int i = 0; i < DOF; ++i) { lqm[i] = L.grav_mirror[i] * p.lq[i];
                                    fqm[i] = F.grav_mirror[i] * p.fq[i]; }
    if (p.grav_lead) { p.grav_lead->gravity(lqm, gl);
                       for (int i = 0; i < DOF; ++i) gl[i] *= L.grav_mirror[i]; }
    if (p.grav_foll) { p.grav_foll->gravity(fqm, gf);
                       for (int i = 0; i < DOF; ++i) gf[i] *= F.grav_mirror[i]; }
  }
  friction(L, p.lqd_f, frl);   // leader (filtered: raw qd noise chatters gate)
  friction(F, p.fqd_f, frf);   // follower (own friction block; 0=off to start)

  // mirror peer into local frame (delta about home). couple_mirror relates the
  // two arms of THIS pair (leader's coupling map).
  const Vec7& cmir = L.couple_mirror;
  Vec7 f_in_l{}, fd_in_l{}, l_in_f{}, ld_in_f{};
  for (int i = 0; i < DOF; ++i) {
    f_in_l[i]  = L.home[i] + cmir[i] * (p.fq[i] - L.home[i]);
    fd_in_l[i] = cmir[i] * p.fqd[i];
    l_in_f[i]  = F.home[i] + cmir[i] * (p.lq[i] - F.home[i]);
    ld_in_f[i] = cmir[i] * p.lqd[i];
  }

  const double alpha = (h_duration > 0.0)
      ? std::clamp((now_sec - h_t_start) / h_duration, 0.0, 1.0) : 1.0;
  const double s = quintic_ease(alpha);

  // ACTIVE soft-engage ramp: coupling stiffness scales 0->1 over active_engage_s_
  // from ACTIVE entry. Slamless engagement from any leader/follower mismatch.
  const double engage = (mode == MODE_ACTIVE && active_engage_s_ > 0.0)
      ? quintic_ease(std::clamp((now_sec - active_t_start_) / active_engage_s_, 0.0, 1.0))
      : 1.0;

  // FREEDRIVE shaping: joint-limit repulsion, MOTOR-SIDE, per ARM (each arm's
  // own joint_limits + fd_limit_* — leader and follower may differ).
  // Zone repulsion: executed MOTOR-SIDE as MIT impedance toward the zone
  // edge. The motor closes this loop on its LOCAL encoder with no CAN
  // delay, so it is passive regardless of arm configuration.
  //
  // History (do not repeat): three PC-side torque variants all failed on
  // hardware. (1) raw-qd asymmetric spring -> chattered on ±0.15 rad/s
  // velocity noise (1.2 Hz limit cycle at rest in zone). (2) EMA-filtered
  // damping -> ~20 ms lag turned the damper into an energy injector after
  // fast reversals (q4 fired out at 8 rad/s). (3) symmetric spring +
  // raw-qd PC-side damping -> the 1-3 ms CAN telemetry delay broke
  // discrete-time passivity in LOW-INERTIA configurations (j2 sustained
  // 13 Hz / 12 Nm when the arm was aligned with its axis). PC-computed
  // velocity feedback through a delayed channel is structurally unsafe
  // here; only the motor-local loop is delay-free.
  //
  // fmax cap: deep violations get kp_eff = fmax/depth so the commanded
  // torque at the CURRENT depth never exceeds fmax (re-evaluated at 1 kHz;
  // converges to the full kp as the joint nears the edge).
  //
  // The MIT target is set `limit_push` INSIDE the valid range (not exactly
  // on the edge) -> a small steady inward force even at the edge (operator
  // request). Damping is scaled by `limit_exit_kd` while EXITING (moving
  // back into range), so coming out isn't draggy. Exit direction is judged
  // from FILTERED velocity, and it only GATES the motor-side kd (a passive
  // damping term) — no PC-side velocity torque, so the delay trap doesn't
  // apply.
  auto zone_override = [](double q, double qdf, int i, MitCmd& c, const ArmCfg& a) {
    const double k = a.fd_limit_kp[i];
    if (k <= 0.0) return;
    const double lo = a.limit_lower[i] + a.fd_limit_margin[i];
    const double hi = a.limit_upper[i] - a.fd_limit_margin[i];
    double target, depth; bool exiting;
    if (q < lo)      { target = lo + a.fd_limit_push[i]; depth = lo - q; exiting = (qdf > 0.0); }
    else if (q > hi) { target = hi - a.fd_limit_push[i]; depth = q - hi; exiting = (qdf < 0.0); }
    else return;
    const double kp_eff =
        (depth > 1e-6) ? std::min(k, a.fd_limit_fmax[i] / depth) : k;
    const double kd_eff =
        a.fd_limit_kd[i] * (exiting ? a.fd_limit_exit_kd : 1.0);
    c.kp[i] = std::max(c.kp[i], kp_eff);          // posture spring may coexist
    c.kd[i] = std::max(c.kd[i], kd_eff);
    c.pos[i] = target;
    c.vel[i] = 0.0;
  };
  const bool freedrive_like =
      (mode == MODE_FREEDRIVE) ||
      (mode == MODE_ACTIVE && !(drive_leader_ && drive_follower_));

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
          // coupling stiffness + velocity FF ramp 0->full on engage (no slam);
          // gravity/friction FF (lc.tau/fc.tau, set above) stay full throughout.
          // Peer-velocity FF scaled by couple_vel_ff: 0 => vel_ref=0 => kd is
          // pure delay-free local damping (passivity-safe, no vibration); >0 =>
          // inject delayed peer velocity for sharper tracking (may vibrate).
          // couple_kp_scale lowers the coupling stiffness (vs the 4Hz tremor /
          // heavy drag from Kp=120 over a delayed channel); hold modes keep full Kp.
          const double vff = engage * g_.couple_vel_ff;
          const double ks  = engage * g_.couple_kp_scale;
          lc.kp[i] = ks * L.Kp[i]; lc.kd[i] = ks * L.Kd[i];
          lc.pos[i] = f_in_l[i];  lc.vel[i] = vff * fd_in_l[i];
          fc.kp[i] = ks * F.Kp[i]; fc.kd[i] = ks * F.Kd[i];
          fc.pos[i] = l_in_f[i];  fc.vel[i] = vff * ld_in_f[i];
        } else {
          // single-role fallback = freedrive-like -> same posture shaping
          lc.kp[i] = L.fd_posture_kp[i]; lc.kd[i] = L.fd_posture_kd[i];
          lc.pos[i] = L.fd_posture_q[i]; lc.vel[i] = 0.0;
          fc.kp[i] = F.fd_posture_kp[i]; fc.kd[i] = F.fd_posture_kd[i];
          fc.pos[i] = F.fd_posture_q[i]; fc.vel[i] = 0.0;
        }
        break;
      case MODE_PAUSED:
        // hold the pose captured at PAUSED entry (NOT a fixed home) -> no slam
        lc.kp[i] = L.Kp[i]; lc.kd[i] = L.Kd[i]; lc.pos[i] = p.l_hold[i]; lc.vel[i] = 0.0;
        fc.kp[i] = F.Kp[i]; fc.kd[i] = F.Kd[i]; fc.pos[i] = p.f_hold[i]; fc.vel[i] = 0.0;
        break;
      case MODE_HOMING: {
        double lt = p.l_home_start[i] + s * (L.home[i] - p.l_home_start[i]);
        double ft = p.f_home_start[i] + s * (F.home[i] - p.f_home_start[i]);
        lc.kp[i] = L.Kp[i]; lc.kd[i] = L.Kd[i]; lc.pos[i] = lt; lc.vel[i] = 0.0;
        fc.kp[i] = F.Kp[i]; fc.kd[i] = F.Kd[i]; fc.pos[i] = ft; fc.vel[i] = 0.0;
        break;
      }
      case MODE_FREEDRIVE:
      default:
        // weak posture spring (motor-side) — tames q3<->q5 self-motion while
        // staying easy to override by hand. Zeros = old pure freedrive.
        lc.kp[i] = L.fd_posture_kp[i]; lc.kd[i] = L.fd_posture_kd[i];
        lc.pos[i] = L.fd_posture_q[i]; lc.vel[i] = 0.0;
        fc.kp[i] = F.fd_posture_kp[i]; fc.kd[i] = F.fd_posture_kd[i];
        fc.pos[i] = F.fd_posture_q[i]; fc.vel[i] = 0.0;
        break;
    }
    if (freedrive_like) {
      zone_override(p.lq[i], p.lqd_f[i], i, lc, L);
      zone_override(p.fq[i], p.fqd_f[i], i, fc, F);
    }
    // clamp feedforward torque (Kp/Kd part is bounded motor-side by limits)
    lc.tau[i] = std::clamp(lc.tau[i], -L.torque_limit[i], L.torque_limit[i]);
    fc.tau[i] = std::clamp(fc.tau[i], -F.torque_limit[i], F.torque_limit[i]);
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
      static_cast<int64_t>(g_.timestep * 1e6)));
  auto deadline = now_monotonic();
  const auto period = jitter.target_period();
  RCLCPP_INFO(get_logger(), "control_loop  period=%ld us  rt=%s",
              (long)(period.count() / 1000), rt_cfg_.enabled ? "ON" : "OFF");

  int prev_mode = MODE_PAUSED;
  int log_counter = 0;

  // TEMP per-cycle CSV diagnostic (long format). Enable: OA_FD_LOG=/tmp/x.csv
  if (const char* lp = std::getenv("OA_FD_LOG")) {
    dbg_log_.open(lp);
    if (dbg_log_.is_open()) {
      dbg_log_ << "t,pair,mode,engage,joint,"
                  "Lq,Ldq,Ltau,Lpos,Lvel,Lkp,Lkd,Ltauff,"
                  "Fq,Fdq,Ftau,Fpos,Fvel,Fkp,Fkd,Ftauff\n";
      RCLCPP_WARN(get_logger(), "OA_FD_LOG -> logging per-cycle CSV to %s", lp);
    }
  }

  if (g_.auto_home_on_start) {
    publish_mode(MODE_HOMING, this->now().seconds(), g_.homing_duration);
    RCLCPP_INFO(get_logger(), "auto_home_on_start -> HOMING %.1fs", g_.homing_duration);
  }

  while (running_) {
    jitter.tick(now_monotonic());
    const double now_sec = this->now().seconds();

    for (Pair* p : pairs_) {
      if (drive_leader_)   p->leader->read(p->lq, p->lqd, p->ltau);
      if (drive_follower_) p->follower->read(p->fq, p->fqd, p->ftau);
      // low-pass velocities for friction gate / zone repulsion
      const double a = std::clamp(g_.vel_filter_alpha, 0.0, 1.0);
      for (int i = 0; i < DOF; ++i) {
        p->lqd_f[i] += a * (p->lqd[i] - p->lqd_f[i]);
        p->fqd_f[i] += a * (p->fqd[i] - p->fqd_f[i]);
      }
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
      else if (mode == MODE_ACTIVE)   // start the coupling-stiffness ramp (no slam)
        active_t_start_ = now_sec;
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
            worst_err = std::max(worst_err, std::abs(p->lq[i] - p->lead_cfg->home[i]));
          if (drive_follower_)
            worst_err = std::max(worst_err, std::abs(p->fq[i] - p->foll_cfg->home[i]));
        }
      if (dbg_log_.is_open()) {
        const double engage = (mode == MODE_ACTIVE && active_engage_s_ > 0.0)
            ? quintic_ease(std::clamp((now_sec - active_t_start_) / active_engage_s_, 0.0, 1.0))
            : 1.0;
        for (int i = 0; i < DOF; ++i) {
          char buf[320];
          std::snprintf(buf, sizeof(buf),
            "%.6f,%s,%d,%.4f,%d,"
            "%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.5f,"
            "%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.5f\n",
            now_sec, p->name.c_str(), mode, engage, i + 1,
            p->lq[i], p->lqd[i], p->ltau[i], lc.pos[i], lc.vel[i], lc.kp[i], lc.kd[i], lc.tau[i],
            p->fq[i], p->fqd[i], p->ftau[i], fc.pos[i], fc.vel[i], fc.kp[i], fc.kd[i], fc.tau[i]);
          dbg_log_ << buf;
        }
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

    if (++log_counter >= static_cast<int>(1.0 / g_.timestep)) {
      log_counter = 0;
      // gravity sanity: print leader q and computed g(q) for each ACTIVE pair.
      // ~0 at a near-vertical pose is normal; horizontal poses load j1/j2/j4.
      for (Pair* p : pairs_) {
        // show the driven side (leader if active, else follower) with ITS model
        const Vec7& q = drive_leader_ ? p->lq : p->fq;
        const char* side = drive_leader_ ? "leader" : "follower";
        GravityModel* gm = drive_leader_ ? p->grav_lead : p->grav_foll;
        Vec7 g{};
        const ArmCfg* ac = drive_leader_ ? p->lead_cfg : p->foll_cfg;
        Vec7 qm; for (int i = 0; i < DOF; ++i) qm[i] = ac->grav_mirror[i] * q[i];
        const bool gon = (gm && gm->ok());
        if (gm) { gm->gravity(qm, g); for (int i = 0; i < DOF; ++i) g[i] *= ac->grav_mirror[i]; }
        const int rc = gm ? gm->last_rc() : -1;
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
