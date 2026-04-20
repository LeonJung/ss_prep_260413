// leader_node.cpp — Leader ROS2 node implementation.
// Phase 5: config loaded, UrDriver connected, pubs/subs wired, control loop
// reads state + sends zero-torque placeholder. Bilateral PD + deadband is
// TODO Phase 4.

#include "ur10e_teleop_real_cpp/leader_node.hpp"

#include <array>
#include <chrono>

#include <ur_client_library/types.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/ur/version_information.h>

namespace ur10e_teleop_real_cpp {

LeaderNode::LeaderNode(const Options& opts)
    : rclcpp::Node("leader_real_node"), opts_(opts) {
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

  // Resolve per-robot values
  home_qpos_   = cfg_.leader_home;
  peer_home_   = cfg_.follower_home;
  mirror_sign_ = cfg_.mirror_sign;
  torque_limit_ = cfg_.has_leader_torque_limit
      ? cfg_.leader_torque_limit : cfg_.torque_limit;

  RCLCPP_INFO(get_logger(),
    "LeaderNode  robot=%s  ip=%s  rt=%s  prio=%d  cpu=%d",
    opts.robot_type.c_str(), opts.robot_ip.c_str(),
    opts.use_rt ? "ON" : "OFF", opts.rt_priority, opts.rt_cpu);

  // ---- QoS setup ----
  rclcpp::QoS state_qos{10};
  rclcpp::QoS latched_qos{1};
  latched_qos.reliable().transient_local();

  // ---- publishers ----
  state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/ur10e/leader/joint_state", state_qos);
  mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur10e/mode", latched_qos);

  // ---- subscribers ----
  peer_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/ur10e/follower/joint_state", state_qos,
      std::bind(&LeaderNode::peer_cb, this, std::placeholders::_1));
  mode_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ur10e/mode", latched_qos,
      std::bind(&LeaderNode::mode_cb, this, std::placeholders::_1));
  reset_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/ur10e/reset", latched_qos,
      std::bind(&LeaderNode::reset_cb, this, std::placeholders::_1));

  // ---- connect to UR via ur_client_library ----
  if (!connect_robot()) {
    throw std::runtime_error("LeaderNode: robot connection failed");
  }

  // initial mode publish — stay in PAUSED until something tells us otherwise
  publish_mode(MODE_PAUSED);
}

LeaderNode::~LeaderNode() { stop(); }

bool LeaderNode::connect_robot() {
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

  auto version = driver_->getVersion();
  RCLCPP_INFO(get_logger(), "robot software version: %s",
              version.toString().c_str());

  if (cfg_.friction_comp) {
    driver_->setFrictionCompensation(true);
    RCLCPP_INFO(get_logger(), "setFrictionCompensation(true)");
  }

  return true;
}

void LeaderNode::run() {
  if (running_.exchange(true)) return;
  driver_->startRTDECommunication();
  control_thread_ = std::thread(&LeaderNode::control_loop, this);
}

void LeaderNode::stop() {
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

void LeaderNode::peer_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < 6) return;
  std::lock_guard<std::mutex> lk(peer_mtx_);
  for (int i = 0; i < 6; ++i) peer_q_[i] = msg->position[i];
  if (msg->velocity.size() >= 6) {
    for (int i = 0; i < 6; ++i) peer_dq_[i] = msg->velocity[i];
  } else {
    peer_dq_.fill(0.0);
  }
  peer_q_valid_ = true;
}

void LeaderNode::mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < 3) return;
  std::lock_guard<std::mutex> lk(mode_mtx_);
  mode_state_    = static_cast<int>(msg->data[0]);
  mode_t_start_  = msg->data[1];
  mode_duration_ = msg->data[2];
}

void LeaderNode::reset_cb(const std_msgs::msg::Int32::SharedPtr msg) {
  reset_counter_ = msg->data;
}

void LeaderNode::publish_state(const std::array<double, 6>& q,
                                const std::array<double, 6>& dq) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  msg.position.assign(q.begin(), q.end());
  msg.velocity.assign(dq.begin(), dq.end());
  state_pub_->publish(msg);
}

void LeaderNode::publish_mode(int mode, double t_start, double duration) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {static_cast<double>(mode), t_start, duration};
  mode_pub_->publish(msg);
}

namespace {

inline double quintic_ease(double a) {
  a = std::clamp(a, 0.0, 1.0);
  return 10 * a*a*a - 15 * a*a*a*a + 6 * a*a*a*a*a;
}

}  // namespace

void LeaderNode::control_loop() {
  init_rt_thread(rt_cfg_);

  JitterTracker jitter(std::chrono::microseconds(
      static_cast<int64_t>(cfg_.timestep * 1e6)));
  auto deadline = now_monotonic();
  const auto period = jitter.target_period();

  RCLCPP_INFO(get_logger(), "control_loop entered  period=%ld us  rt=%s",
              (long)(period.count() / 1000),
              rt_cfg_.enabled ? "ON" : "OFF");

  // ---- state variables (mirror leader_real_node.py) ----
  std::array<double, 6> q{}, dq{};
  std::array<double, 6> q_user{}, q_hold{}, q_home_start{};
  q_hold       = home_qpos_;
  q_home_start = home_qpos_;

  // initial read to seed q_user
  {
    auto data = driver_->getDataPackage();
    if (data) {
      urcl::vector6d_t _q, _qd;
      if (data->getData("actual_q",  _q) && data->getData("actual_qd", _qd)) {
        for (int i = 0; i < 6; ++i) { q[i] = _q[i]; dq[i] = _qd[i]; }
      }
    }
    q_user = q;
  }
  RCLCPP_INFO(get_logger(), "q_user seeded to actual_q");

  int prev_state = MODE_ACTIVE;
  int log_counter = 0;

  const double startup_time = this->now().seconds();
  const double RESET_STARTUP_GRACE = 2.0;
  double active_t_start = startup_time;
  int last_reset_counter = reset_counter_.load();

  // Auto-homing on start
  if (cfg_.auto_home_on_start) {
    publish_mode(MODE_HOMING, this->now().seconds(), cfg_.homing_duration);
    RCLCPP_INFO(get_logger(), "auto_home_on_start → HOMING in %.1fs",
                cfg_.homing_duration);
  }

  urcl::vector6d_t tau_cmd = {0, 0, 0, 0, 0, 0};

  while (running_) {
    auto t_now_mono = now_monotonic();
    jitter.tick(t_now_mono);
    const double now_sec = this->now().seconds();

    // ---- read state ----
    {
      auto data = driver_->getDataPackage();
      if (data) {
        urcl::vector6d_t _q, _qd;
        if (data->getData("actual_q", _q) && data->getData("actual_qd", _qd)) {
          for (int i = 0; i < 6; ++i) { q[i] = _q[i]; dq[i] = _qd[i]; }
        }
      }
    }

    // ---- snapshot mode / peer state ----
    int cur_state;
    double h_t_start, h_duration;
    {
      std::lock_guard<std::mutex> lk(mode_mtx_);
      cur_state = mode_state_;
      h_t_start = mode_t_start_;
      h_duration = mode_duration_;
    }
    std::array<double, 6> raw_peer_q{}, raw_peer_dq{};
    bool bilateral_active = false;
    {
      std::lock_guard<std::mutex> lk(peer_mtx_);
      if (peer_q_valid_) {
        raw_peer_q = peer_q_;
        raw_peer_dq = peer_dq_;
        bilateral_active = true;
      }
    }

    // ---- reset handling (→ HOMING) ----
    int cur_reset = reset_counter_.load();
    if (cur_reset != last_reset_counter) {
      last_reset_counter = cur_reset;
      if (now_sec - startup_time < RESET_STARTUP_GRACE) {
        RCLCPP_INFO(get_logger(),
            "reset ignored during startup grace (counter=%d)", cur_reset);
      } else if (cur_state == MODE_ACTIVE) {
        publish_mode(MODE_HOMING, now_sec, cfg_.homing_duration);
        RCLCPP_INFO(get_logger(),
            "reset request → HOMING (%.1fs)", cfg_.homing_duration);
      }
    }

    // ---- state transition ----
    if (cur_state != prev_state) {
      RCLCPP_INFO(get_logger(), "state %d → %d", prev_state, cur_state);
      if (cur_state == MODE_PAUSED) {
        q_hold = (prev_state == MODE_HOMING) ? home_qpos_ : q;
      } else if (cur_state == MODE_HOMING) {
        q_home_start = q;
      } else if (cur_state == MODE_ACTIVE) {
        q_user = q;
        active_t_start = now_sec;
      }
    }

    // ---- mirror peer → local frame ----
    std::array<double, 6> peer_q_local{}, peer_dq_local{};
    if (bilateral_active) {
      for (int i = 0; i < 6; ++i) {
        const double delta = raw_peer_q[i] - peer_home_[i];
        peer_q_local[i]  = home_qpos_[i] + mirror_sign_[i] * delta;
        peer_dq_local[i] = mirror_sign_[i] * raw_peer_dq[i];
      }
    }

    // ---- control law ----
    std::array<double, 6> tau{};
    // gravity compensation: firmware handles it when gravity_comp_internal
    // is true (our default). So tau_grav is always zero here.
    const double ACTIVE_RAMP = 0.5;  // soft-start window [s]
    bool skip_torque_write = false;  // set by HOMING branch (uses MODE_SERVOJ)

    switch (cur_state) {
      case MODE_ACTIVE: {
        const double ramp = std::min(
            1.0, (now_sec - active_t_start) / ACTIVE_RAMP);
        for (int i = 0; i < 6; ++i) {
          // user-spring (zero when KP_USER=0 in current config)
          double tau_user = cfg_.leader_kp_user[i] * (q_user[i] - q[i])
                          - cfg_.leader_kd_user[i] * dq[i];
          // bilateral coupling with continuous deadband + soft-start ramp
          double tau_bi = 0.0;
          if (bilateral_active) {
            double raw = cfg_.leader_kp_bi[i] * (peer_q_local[i] - q[i])
                       + cfg_.leader_kd_bi[i] * (peer_dq_local[i] - dq[i]);
            double mag = std::abs(raw);
            double excess = std::max(0.0, mag - cfg_.leader_tau_bi_deadband[i]);
            tau_bi = ramp * std::copysign(excess, raw);
          }
          tau[i] = tau_user + tau_bi;
        }
        break;
      }
      case MODE_PAUSED:
        for (int i = 0; i < 6; ++i) {
          tau[i] = cfg_.leader_kp_hold[i] * (q_hold[i] - q[i])
                 - cfg_.leader_kd_hold[i] * dq[i];
        }
        break;
      case MODE_HOMING: {
        // Position-control homing via ur_client_library's MODE_SERVOJ.
        // Commanding home_qpos_ directly triggers UR's joint-velocity
        // limit ("345 rad/s required ... exceeding joint velocity limits")
        // because servoj expects an achievable next-step target, not a
        // final pose. Ramp from q_home_start (captured on transition) to
        // home_qpos_ over homing_duration using a quintic ease profile.
        const double t_elapsed = now_sec - h_t_start;
        const double alpha = (h_duration > 0.0)
                           ? std::clamp(t_elapsed / h_duration, 0.0, 1.0)
                           : 1.0;
        const double s = quintic_ease(alpha);
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
        if (alpha >= 1.0 && err < 0.01) {   // ~0.57° per joint
          q_user = home_qpos_;
          publish_mode(MODE_PAUSED);
          RCLCPP_INFO(get_logger(),
            "HOMING complete (err=%.4f rad) → PAUSED", err);
        }
        break;
      }
      case MODE_FREEDRIVE:
      default:
        // tau already zero
        break;
    }

    // ---- clip to torque_limit ----
    for (int i = 0; i < 6; ++i) {
      tau[i] = std::clamp(tau[i], -torque_limit_[i], torque_limit_[i]);
      tau_cmd[i] = tau[i];
    }

    // ---- write torque (skipped during HOMING, which uses MODE_SERVOJ) ----
    if (!skip_torque_write) {
      driver_->writeJointCommand(
          tau_cmd,
          urcl::comm::ControlMode::MODE_TORQUE,
          urcl::RobotReceiveTimeout::millisec(20));
    }

    // ---- publish state ----
    publish_state(q, dq);

    // ---- 1 Hz diag ----
    if (++log_counter >= static_cast<int>(1.0 / cfg_.timestep)) {
      log_counter = 0;
      double tau_max = 0.0;
      for (double v : tau) tau_max = std::max(tau_max, std::abs(v));
      RCLCPP_INFO(get_logger(),
        "[DIAG] mode=%d |tau|max=%.2fN·m  %s",
        cur_state, tau_max, jitter.log_line("").c_str());
    }

    prev_state = cur_state;
    deadline += period;
    sleep_until(deadline);
  }
  RCLCPP_INFO(get_logger(), "control_loop exited");
}

}  // namespace ur10e_teleop_real_cpp
