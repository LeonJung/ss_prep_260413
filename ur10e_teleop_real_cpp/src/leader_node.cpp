// leader_node.cpp — Leader ROS2 node (skeleton).
// TODO (Phase 4/5): load config, connect ur_client_library UrDriver,
// create pub/sub, implement bilateral PD + deadband control loop mirroring
// ur10e_teleop_real_py/src/leader_real_node.py.

#include "ur10e_teleop_real_cpp/leader_node.hpp"

#include <chrono>

namespace ur10e_teleop_real_cpp {

LeaderNode::LeaderNode(const Options& opts)
    : rclcpp::Node("leader_real_node"), opts_(opts) {
  rt_cfg_.enabled      = opts.use_rt;
  rt_cfg_.priority     = opts.rt_priority;
  rt_cfg_.cpu_affinity = opts.rt_cpu;

  RCLCPP_INFO(get_logger(),
    "LeaderNode created  robot=%s  ip=%s  rt=%s  prio=%d  cpu=%d  config=%s",
    opts.robot_type.c_str(), opts.robot_ip.c_str(),
    opts.use_rt ? "ON" : "OFF", opts.rt_priority, opts.rt_cpu,
    opts.config_path.c_str());
  // TODO: load yaml, connect UrDriver, set up pubs/subs
}

LeaderNode::~LeaderNode() { stop(); }

void LeaderNode::run() {
  if (running_.exchange(true)) return;  // already running
  control_thread_ = std::thread(&LeaderNode::control_loop, this);
}

void LeaderNode::stop() {
  running_ = false;
  if (control_thread_.joinable()) control_thread_.join();
}

void LeaderNode::control_loop() {
  init_rt_thread(rt_cfg_);

  JitterTracker jitter(std::chrono::microseconds(2000));  // 500 Hz target
  auto deadline = now_monotonic();
  const auto period = jitter.target_period();

  RCLCPP_INFO(get_logger(), "control_loop entered (rt=%s)",
              rt_cfg_.enabled ? "ON" : "OFF");

  int log_counter = 0;
  while (running_) {
    auto t0 = now_monotonic();
    jitter.tick(t0);

    // TODO: read robot state, compute tau (PD + deadband + mode), write torque.
    //       See ur10e_teleop_real_py/src/leader_real_node.py _control_loop.

    // Log jitter stats every 500 cycles (~1 s at 500 Hz).
    if (++log_counter >= 500) {
      log_counter = 0;
      RCLCPP_INFO(get_logger(), "%s",
                  jitter.log_line("[JIT]").c_str());
    }

    deadline += period;
    sleep_until(deadline);
  }

  RCLCPP_INFO(get_logger(), "control_loop exited");
}

}  // namespace ur10e_teleop_real_cpp
