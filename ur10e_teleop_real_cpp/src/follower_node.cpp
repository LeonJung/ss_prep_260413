// follower_node.cpp — Follower ROS2 node implementation (Phase 5).
// Same structure as leader_node.cpp; control law in Phase 4.

#include "ur10e_teleop_real_cpp/follower_node.hpp"

#include <array>
#include <chrono>

#include <ur_client_library/types.h>
#include <ur_client_library/ur/ur_driver.h>

namespace ur10e_teleop_real_cpp {

FollowerNode::FollowerNode(const Options& opts)
    : rclcpp::Node("follower_real_node"), opts_(opts) {
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
                                  const std::array<double, 6>& dq) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  msg.position.assign(q.begin(), q.end());
  msg.velocity.assign(dq.begin(), dq.end());
  state_pub_->publish(msg);
}

void FollowerNode::publish_mode(int mode, double t_start, double duration) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {static_cast<double>(mode), t_start, duration};
  mode_pub_->publish(msg);
}

void FollowerNode::control_loop() {
  init_rt_thread(rt_cfg_);

  JitterTracker jitter(std::chrono::microseconds(
      static_cast<int64_t>(cfg_.timestep * 1e6)));
  auto deadline = now_monotonic();
  const auto period = jitter.target_period();

  RCLCPP_INFO(get_logger(), "control_loop entered  period=%ld us  rt=%s",
              (long)(period.count() / 1000),
              rt_cfg_.enabled ? "ON" : "OFF");

  urcl::vector6d_t tau_cmd = {0, 0, 0, 0, 0, 0};

  int log_counter = 0;
  while (running_) {
    auto t_now = now_monotonic();
    jitter.tick(t_now);

    auto data = driver_->getDataPackage();
    std::array<double, 6> q{}, dq{};
    if (data) {
      urcl::vector6d_t _q, _qd;
      if (data->getData("actual_q",  _q) && data->getData("actual_qd", _qd)) {
        for (int i = 0; i < 6; ++i) { q[i] = _q[i]; dq[i] = _qd[i]; }
      }
    }

    // TODO(phase4): KP_TRACK * (peer_q_local - q) + KD_TRACK * ... + tau_grav
    for (int i = 0; i < 6; ++i) tau_cmd[i] = 0.0;

    driver_->writeJointCommand(
        tau_cmd,
        urcl::comm::ControlMode::MODE_TORQUE,
        urcl::RobotReceiveTimeout::millisec(20));

    publish_state(q, dq);

    if (++log_counter >= static_cast<int>(1.0 / cfg_.timestep)) {
      log_counter = 0;
      RCLCPP_INFO(get_logger(), "%s", jitter.log_line("[JIT]").c_str());
    }

    deadline += period;
    sleep_until(deadline);
  }
  RCLCPP_INFO(get_logger(), "control_loop exited");
}

}  // namespace ur10e_teleop_real_cpp
