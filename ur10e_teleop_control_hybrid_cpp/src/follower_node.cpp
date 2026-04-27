// follower_node.cpp — Follower ROS2 node implementation (Phase 5).
// Same structure as leader_node.cpp; control law in Phase 4.

#include "ur10e_teleop_control_hybrid_cpp/follower_node.hpp"

#include <array>
#include <chrono>

#include <ur_client_library/types.h>
#include <ur_client_library/ur/ur_driver.h>

#include "ur10e_teleop_control_hybrid_cpp/ur_jacobian.hpp"

namespace ur10e_teleop_control_hybrid_cpp {

namespace {
Eigen::VectorXd vec6_to_eigen(const Vec6& a) {
  Eigen::VectorXd v(6);
  for (int i = 0; i < 6; ++i) v(i) = a[i];
  return v;
}
}  // namespace

FollowerNode::FollowerNode(const Options& opts)
    : rclcpp::Node("follower_hybrid_node"), opts_(opts) {
  rt_cfg_.enabled      = opts.use_rt;
  rt_cfg_.priority     = opts.rt_priority;
  rt_cfg_.cpu_affinity = opts.rt_cpu;

  if (!opts.config_path.empty()) {
    if (!load_config(opts.config_path, cfg_)) {
      RCLCPP_WARN(get_logger(), "config load failed; using defaults");
    } else {
      RCLCPP_INFO(get_logger(), "config loaded: %s", opts.config_path.c_str());
    }
  }

  home_qpos_   = cfg_.follower_home;
  peer_home_   = cfg_.leader_home;
  mirror_sign_ = cfg_.mirror_sign;
  torque_limit_ = cfg_.has_follower_torque_limit
      ? cfg_.follower_torque_limit : cfg_.torque_limit;

  RCLCPP_INFO(get_logger(),
    "FollowerNode  robot=%s  ip=%s  rt=%s  prio=%d  cpu=%d",
    opts.robot_type.c_str(), opts.robot_ip.c_str(),
    opts.use_rt ? "ON" : "OFF", opts.rt_priority, opts.rt_cpu);

  rclcpp::QoS state_qos{10};
  rclcpp::QoS latched_qos{1};
  latched_qos.reliable().transient_local();

  state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/ur10e/follower/joint_state", state_qos);
  mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur10e/mode", latched_qos);

  peer_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/ur10e/leader/joint_state", state_qos,
      std::bind(&FollowerNode::peer_cb, this, std::placeholders::_1));
  mode_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ur10e/mode", latched_qos,
      std::bind(&FollowerNode::mode_cb, this, std::placeholders::_1));

  if (!connect_robot()) {
    throw std::runtime_error("FollowerNode: robot connection failed");
  }
}

FollowerNode::~FollowerNode() { stop(); }

bool FollowerNode::connect_robot() {
  // Optional: auto power-cycle via Dashboard (port 29999) before RTDE.
  if (cfg_.auto_power_cycle) {
    dashboard_ = std::make_unique<DashboardClient>(opts_.robot_ip);
    if (!dashboard_->power_up_sequence()) {
      RCLCPP_ERROR(get_logger(),
        "auto power-cycle (power-up) FAILED — aborting connect_robot");
      return false;
    }
  }

  urcl::UrDriverConfiguration cfg;
  cfg.robot_ip           = opts_.robot_ip;
  cfg.output_recipe_file = opts_.resources_dir + "/rtde_output_recipe.txt";
  cfg.input_recipe_file  = opts_.resources_dir + "/rtde_input_recipe.txt";
  cfg.script_file        = opts_.resources_dir + "/external_control.urscript";
  cfg.headless_mode      = true;
  cfg.reverse_port        = opts_.reverse_port_base + 0;
  cfg.script_sender_port  = opts_.reverse_port_base + 1;
  cfg.trajectory_port     = opts_.reverse_port_base + 2;
  cfg.script_command_port = opts_.reverse_port_base + 3;
  cfg.handle_program_state = [this](bool running) {
    program_running_ = running;
    RCLCPP_INFO(get_logger(), "program_running = %s", running ? "true" : "false");
  };

  RCLCPP_INFO(get_logger(),
    "UrDriver  ip=%s  ports=%u-%u  script=%s",
    cfg.robot_ip.c_str(), cfg.reverse_port, cfg.script_command_port,
    cfg.script_file.c_str());

  try {
    driver_ = std::make_unique<urcl::UrDriver>(cfg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "UrDriver ctor failed: %s", e.what());
    return false;
  }
  RCLCPP_INFO(get_logger(), "robot software version: %s",
              driver_->getVersion().toString().c_str());
  if (cfg_.friction_comp) {
    driver_->setFrictionCompensation(true);
  }

  // ---- hybrid (B1) modules ----
  const std::string urdf_path =
      opts_.resources_dir + "/" + opts_.robot_type + ".urdf";
  try {
    dyn_ = std::make_unique<DynamicsModel>(
        urdf_path, cfg_.hybrid_base_link, cfg_.hybrid_tip_link);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "DynamicsModel load failed (%s): %s",
                 urdf_path.c_str(), e.what());
    return false;
  }
  vel_est_ = std::make_unique<VelocityEstimator>(
      6, cfg_.hybrid_velocity_cutoff_hz, cfg_.timestep);

  DisturbanceObserver::Params dob_p;
  dob_p.cutoff_hz = cfg_.hybrid_dob_cutoff_hz;
  dob_p.accel_cutoff_hz = cfg_.hybrid_dob_accel_cutoff_hz;
  dob_p.viscous = vec6_to_eigen(cfg_.hybrid_d_viscous);
  dob_p.firmware_grav_comp = cfg_.gravity_comp_internal;
  dob_ = std::make_unique<DisturbanceObserver>(*dyn_, dob_p, cfg_.timestep);

  FourChannelController::Params cp;
  cp.Kp = vec6_to_eigen(cfg_.hybrid_follower_kp);
  cp.Kd = vec6_to_eigen(cfg_.hybrid_follower_kd);
  cp.Kf = vec6_to_eigen(cfg_.hybrid_follower_kf);
  cp.D  = vec6_to_eigen(cfg_.hybrid_d_viscous);
  cp.firmware_grav_comp = cfg_.gravity_comp_internal;
  cp.tau_ext_cancel_gain = cfg_.hybrid_tau_ext_cancel_gain;
  cp.use_diagonal_inertia = cfg_.hybrid_use_diagonal_inertia;
  ctrl_ = std::make_unique<FourChannelController>(*dyn_, cp);

  RCLCPP_INFO(get_logger(),
      "hybrid(B1): URDF=%s  vel_fc=%.0fHz  dob_fc=%.0f/%.0fHz  "
      "|Kp|=%.1f |Kd|=%.1f |Kf|=%.3f  fw_grav_comp=%s  τ̂_cancel=%.2f  "
      "M=%s",
      urdf_path.c_str(), cfg_.hybrid_velocity_cutoff_hz,
      cfg_.hybrid_dob_cutoff_hz, cfg_.hybrid_dob_accel_cutoff_hz,
      cp.Kp.norm(), cp.Kd.norm(), cp.Kf.norm(),
      cp.firmware_grav_comp ? "true" : "false",
      cp.tau_ext_cancel_gain,
      cp.use_diagonal_inertia ? "diag" : "full");

  if (cfg_.hybrid_tank_enabled) {
    EnergyTank::Params tp;
    tp.E_max = cfg_.hybrid_tank_e_max;
    tp.E_init = cfg_.hybrid_tank_e_init;
    tp.refill_ceiling = cfg_.hybrid_tank_refill_ceiling;
    tp.D_dissipation = vec6_to_eigen(cfg_.hybrid_tank_d_dissipation);
    tank_ = std::make_unique<EnergyTank>(tp, cfg_.timestep);
    RCLCPP_INFO(get_logger(),
        "hybrid(C1): EnergyTank  E_max=%.2fJ  E_init=%.2fJ  ceiling=%.2f",
        tp.E_max, tp.E_init, tp.refill_ceiling);
  } else {
    RCLCPP_INFO(get_logger(), "hybrid(C1): EnergyTank DISABLED (bare 4CH)");
  }

  return true;
}

void FollowerNode::run() {
  if (running_.exchange(true)) return;
  driver_->startRTDECommunication();
  control_thread_ = std::thread(&FollowerNode::control_loop, this);
}

void FollowerNode::stop() {
  running_ = false;
  if (control_thread_.joinable()) control_thread_.join();
  if (driver_) {
    try { driver_->stopControl(); } catch (...) {}
  }
  if (dashboard_) {
    try { dashboard_->power_down_sequence(); } catch (...) {}
    dashboard_->close();
    dashboard_.reset();
  }
}

void FollowerNode::peer_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < 6) return;
  std::lock_guard<std::mutex> lk(peer_mtx_);
  for (int i = 0; i < 6; ++i) peer_q_[i] = msg->position[i];
  if (msg->velocity.size() >= 6) {
    for (int i = 0; i < 6; ++i) peer_dq_[i] = msg->velocity[i];
  } else {
    peer_dq_.fill(0.0);
  }
  // hybrid: peer publishes τ̂_ext as effort for 4CH force reflection.
  if (msg->effort.size() >= 6) {
    for (int i = 0; i < 6; ++i) peer_tau_[i] = msg->effort[i];
  } else {
    peer_tau_.fill(0.0);
  }
  peer_q_valid_ = true;
}

void FollowerNode::mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < 3) return;
  std::lock_guard<std::mutex> lk(mode_mtx_);
  mode_state_    = static_cast<int>(msg->data[0]);
  mode_t_start_  = msg->data[1];
  mode_duration_ = msg->data[2];
}

void FollowerNode::publish_state(const std::array<double, 6>& q,
                                  const std::array<double, 6>& dq,
                                  const std::array<double, 6>& tau_contact) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  msg.position.assign(q.begin(), q.end());
  msg.velocity.assign(dq.begin(), dq.end());
  // _ff: effort carries J^T·F_TCP (joint-space contact torque) so the
  // leader can reflect it as haptic feedback with K_FT * peer_effort.
  msg.effort.assign(tau_contact.begin(), tau_contact.end());
  state_pub_->publish(msg);
}

void FollowerNode::publish_mode(int mode, double t_start, double duration) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {static_cast<double>(mode), t_start, duration};
  mode_pub_->publish(msg);
}

namespace {
inline double quintic_ease_f(double a) {
  a = std::clamp(a, 0.0, 1.0);
  return 10 * a*a*a - 15 * a*a*a*a + 6 * a*a*a*a*a;
}
}  // namespace

void FollowerNode::control_loop() {
  init_rt_thread(rt_cfg_);

  JitterTracker jitter(std::chrono::microseconds(
      static_cast<int64_t>(cfg_.timestep * 1e6)));
  auto deadline = now_monotonic();
  const auto period = jitter.target_period();

  RCLCPP_INFO(get_logger(), "control_loop entered  period=%ld us  rt=%s",
              (long)(period.count() / 1000),
              rt_cfg_.enabled ? "ON" : "OFF");

  std::array<double, 6> q{}, dq{};
  std::array<double, 6> q_hold{}, q_home_start{}, q_target_init{};
  q_hold = home_qpos_;
  q_home_start = home_qpos_;

  // initial read
  {
    auto data = driver_->getDataPackage();
    if (data) {
      urcl::vector6d_t _q, _qd;
      if (data->getData("actual_q",  _q) && data->getData("actual_qd", _qd)) {
        for (int i = 0; i < 6; ++i) { q[i] = _q[i]; dq[i] = _qd[i]; }
      }
    }
    q_target_init = q;
  }
  Eigen::VectorXd q_e(6), qd_zero = Eigen::VectorXd::Zero(6);
  for (int i = 0; i < 6; ++i) q_e(i) = q[i];
  vel_est_->reset(q_e);
  dob_->reset(qd_zero);
  Eigen::VectorXd tau_applied_prev = Eigen::VectorXd::Zero(6);
  double active_t_start = 0.0;
  RCLCPP_INFO(get_logger(), "q_target_init seeded; vel/DOB observers reset");

  // _ff: zero the wrist F/T sensor so the haptic path starts from a
  // calibrated baseline. Requires external_control.urscript to already be
  // running, which startRTDECommunication() has just kicked off.
  if (driver_->zeroFTSensor()) {
    RCLCPP_INFO(get_logger(), "zeroFTSensor() OK");
  } else {
    RCLCPP_WARN(get_logger(), "zeroFTSensor() FAILED — haptic baseline may drift");
  }

  int prev_state = /*MODE_ACTIVE*/ 0;
  int log_counter = 0;

  // Auto-homing on start
  if (cfg_.auto_home_on_start) {
    publish_mode(/*MODE_HOMING=*/2, this->now().seconds(), cfg_.homing_duration);
    RCLCPP_INFO(get_logger(), "auto_home_on_start → HOMING in %.1fs",
                cfg_.homing_duration);
  }

  urcl::vector6d_t tau_cmd = {0, 0, 0, 0, 0, 0};

  while (running_) {
    auto t_now_mono = now_monotonic();
    jitter.tick(t_now_mono);
    const double now_sec = this->now().seconds();

    // read state (q, dq, and TCP wrench for _ff haptic path)
    std::array<double, 6> F_tcp{};
    bool ft_valid = false;
    {
      auto data = driver_->getDataPackage();
      if (data) {
        urcl::vector6d_t _q, _qd, _f;
        if (data->getData("actual_q", _q) && data->getData("actual_qd", _qd)) {
          for (int i = 0; i < 6; ++i) { q[i] = _q[i]; dq[i] = _qd[i]; }
        }
        if (data->getData("actual_TCP_force", _f)) {
          for (int i = 0; i < 6; ++i) F_tcp[i] = _f[i];
          ft_valid = true;
        }
      }
    }

    // snapshot mode / peer
    int cur_state;
    double h_t_start, h_duration;
    {
      std::lock_guard<std::mutex> lk(mode_mtx_);
      cur_state = mode_state_;
      h_t_start = mode_t_start_;
      h_duration = mode_duration_;
    }
    std::array<double, 6> raw_peer_q{}, raw_peer_dq{}, raw_peer_tau{};
    bool bilateral_active = false;
    {
      std::lock_guard<std::mutex> lk(peer_mtx_);
      if (peer_q_valid_) {
        raw_peer_q = peer_q_;
        raw_peer_dq = peer_dq_;
        raw_peer_tau = peer_tau_;
        bilateral_active = true;
      }
    }

    // ---- update observers every cycle ----
    for (int i = 0; i < 6; ++i) q_e(i) = q[i];
    const Eigen::VectorXd qd_hat = vel_est_->update(q_e);
    const Eigen::VectorXd tau_ext_hat =
        dob_->update(q_e, qd_hat, tau_applied_prev);

    // state transition
    if (cur_state != prev_state) {
      RCLCPP_INFO(get_logger(), "state %d → %d", prev_state, cur_state);
      if (cur_state == /*MODE_PAUSED=*/1) {
        q_hold = (prev_state == /*MODE_HOMING=*/2) ? home_qpos_ : q;
      } else if (cur_state == /*MODE_HOMING=*/2) {
        q_home_start = q;
      } else if (cur_state == /*MODE_ACTIVE=*/0) {
        active_t_start = now_sec;
      }
      vel_est_->reset(q_e);
      dob_->reset(qd_zero);
      if (tank_) tank_->reset();
    }

    // mirror peer → local (q, q̇, τ̂_ext)
    Eigen::VectorXd q_peer_e  = q_e;
    Eigen::VectorXd qd_peer_e = qd_hat;
    Eigen::VectorXd tau_peer_e = Eigen::VectorXd::Zero(6);
    if (bilateral_active) {
      for (int i = 0; i < 6; ++i) {
        const double delta = raw_peer_q[i] - peer_home_[i];
        q_peer_e(i)   = home_qpos_[i] + mirror_sign_[i] * delta;
        qd_peer_e(i)  = mirror_sign_[i] * raw_peer_dq[i];
        tau_peer_e(i) = mirror_sign_[i] * raw_peer_tau[i];
      }
    }

    // control law
    std::array<double, 6> tau{};
    const double ACTIVE_RAMP = 0.5;  // soft-start window [s]
    bool skip_torque_write = false;  // HOMING uses MODE_SERVOJ instead
    switch (cur_state) {
      case /*MODE_ACTIVE=*/0: {
        // Hybrid B1: Sensorless 4CH with DOB, symmetric with leader.
        const double ramp = std::min(
            1.0, (now_sec - active_t_start) / ACTIVE_RAMP);
        const Eigen::Matrix<double, 6, 1> tau_4ch = ctrl_->compute(
            q_e, qd_hat, tau_ext_hat,
            q_peer_e, qd_peer_e, tau_peer_e, ramp);
        Eigen::Matrix<double, 6, 1> tau_final = tau_4ch;
        // C1: Two-Layer Energy Tank — modulate only the tracking bracket.
        if (tank_) {
          const Eigen::Matrix<double, 6, 1> tau_static = ctrl_->compute(
              q_e, qd_hat, tau_ext_hat,
              q_peer_e, qd_peer_e, tau_peer_e, 0.0);
          const Eigen::VectorXd tau_active = tau_4ch - tau_static;
          const double alpha = tank_->step(tau_active, qd_hat);
          tau_final = tau_static + alpha * tau_active;
        }
        for (int i = 0; i < 6; ++i) tau[i] = tau_final(i);
        break;
      }
      case /*MODE_PAUSED=*/1:
        for (int i = 0; i < 6; ++i) {
          tau[i] = cfg_.follower_kp_hold[i] * (q_hold[i] - q[i])
                 - cfg_.follower_kd_hold[i] * dq[i];
        }
        break;
      case /*MODE_HOMING=*/2: {
        // Position-control homing via MODE_SERVOJ with quintic ramp from
        // q_home_start to home_qpos_ over homing_duration.
        const double t_elapsed = now_sec - h_t_start;
        const double alpha = (h_duration > 0.0)
                           ? std::clamp(t_elapsed / h_duration, 0.0, 1.0)
                           : 1.0;
        const double s = quintic_ease_f(alpha);
        urcl::vector6d_t home_cmd;
        for (int i = 0; i < 6; ++i)
          home_cmd[i] = q_home_start[i]
                      + s * (home_qpos_[i] - q_home_start[i]);
        driver_->writeJointCommand(
            home_cmd,
            urcl::comm::ControlMode::MODE_SERVOJ,
            urcl::RobotReceiveTimeout::millisec(20));
        skip_torque_write = true;
        tau.fill(0.0);
        double err = 0.0;
        for (int i = 0; i < 6; ++i)
          err = std::max(err, std::abs(q[i] - home_qpos_[i]));
        if (alpha >= 1.0 && err < 0.01) {
          q_target_init = home_qpos_;
          publish_mode(/*MODE_PAUSED=*/1);
          RCLCPP_INFO(get_logger(),
            "HOMING complete (err=%.4f rad) → PAUSED", err);
        }
        break;
      }
      case /*MODE_FREEDRIVE=*/3:
      default:
        // tau already zero
        break;
    }

    // ---- TCP workspace safety (2-tier virtual wall) — ACTIVE only ----
    if (cfg_.ws_enabled && cur_state == /*MODE_ACTIVE=*/0) {
      auto p_tcp = tcp_position(q, opts_.robot_type);
      double pen_lo[3], pen_hi[3];
      double max_pen = 0.0;
      for (int i = 0; i < 3; ++i) {
        pen_lo[i] = std::max(0.0, cfg_.ws_xyz_min[i] - p_tcp(i));
        pen_hi[i] = std::max(0.0, p_tcp(i) - cfg_.ws_xyz_max[i]);
        max_pen = std::max({max_pen, pen_lo[i], pen_hi[i]});
      }
      if (max_pen > 0.0) {
        Vec6d F6; F6.setZero();
        for (int i = 0; i < 3; ++i)
          F6(i) = cfg_.ws_k_wall * (pen_lo[i] - pen_hi[i]);
        Mat66 J = ur_jacobian(q, opts_.robot_type);
        Vec6d dtau = J.transpose() * F6;
        for (int i = 0; i < 6; ++i) tau[i] += dtau(i);
        if (max_pen > cfg_.ws_soft_penetration) {
          RCLCPP_WARN(get_logger(),
            "workspace HARD limit (pen=%.3f m > %.3f m) → PAUSED",
            max_pen, cfg_.ws_soft_penetration);
          publish_mode(/*MODE_PAUSED=*/1);
        }
      }
    }

    // clip
    for (int i = 0; i < 6; ++i) {
      tau[i] = std::clamp(tau[i], -torque_limit_[i], torque_limit_[i]);
      tau_cmd[i] = tau[i];
      tau_applied_prev(i) = tau[i];
    }

    if (!skip_torque_write) {
      driver_->writeJointCommand(
          tau_cmd,
          urcl::comm::ControlMode::MODE_TORQUE,
          urcl::RobotReceiveTimeout::millisec(20));
    }

    // hybrid: publish τ̂_ext (DOB estimate) as effort. Leader uses it as
    // the τ_d term in its 4CH Kf·(τ̂_ext + τ̂_ext_peer) reflection.
    // The F/T sensor reading (F_tcp / tau_contact) is now purely for
    // diagnostics and can be surfaced elsewhere if needed.
    (void)ft_valid; (void)F_tcp;
    std::array<double, 6> tau_ext_arr{};
    for (int i = 0; i < 6; ++i) tau_ext_arr[i] = tau_ext_hat(i);
    publish_state(q, dq, tau_ext_arr);

    if (++log_counter >= static_cast<int>(1.0 / cfg_.timestep)) {
      log_counter = 0;
      double tau_max = 0.0;
      for (double v : tau) tau_max = std::max(tau_max, std::abs(v));
      if (tank_) {
        RCLCPP_INFO(get_logger(),
          "[DIAG] mode=%d |tau|max=%.2fN·m  tank E=%.2fJ α=%.2f  %s",
          cur_state, tau_max, tank_->energy(), tank_->last_alpha(),
          jitter.log_line("").c_str());
      } else {
        RCLCPP_INFO(get_logger(),
          "[DIAG] mode=%d |tau|max=%.2fN·m  %s",
          cur_state, tau_max, jitter.log_line("").c_str());
      }
    }

    prev_state = cur_state;
    deadline += period;
    sleep_until(deadline);
  }
  RCLCPP_INFO(get_logger(), "control_loop exited");
}

}  // namespace ur10e_teleop_control_hybrid_cpp
