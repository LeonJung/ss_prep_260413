// follower_node.cpp — Follower ROS2 node (skeleton).
// TODO (Phase 4/5): load config, connect UrDriver, create pub/sub,
// implement KP_TRACK * peer_q tracking control loop mirroring
// ur10e_teleop_real_py/src/follower_real_node.py.

#include "ur10e_teleop_real_cpp/follower_node.hpp"

#include <chrono>

namespace ur10e_teleop_real_cpp {

FollowerNode::FollowerNode(const Options& opts)
    : rclcpp::Node("follower_real_node"), opts_(opts) {
  rt_cfg_.enabled      = opts.use_rt;
  rt_cfg_.priority     = opts.rt_priority;
  rt_cfg_.cpu_affinity = opts.rt_cpu;

  RCLCPP_INFO(get_logger(),
    "FollowerNode created  robot=%s  ip=%s  rt=%s  prio=%d  cpu=%d  config=%s",
    opts.robot_type.c_str(), opts.robot_ip.c_str(),
    opts.use_rt ? "ON" : "OFF", opts.rt_priority, opts.rt_cpu,
    opts.config_path.c_str());
  // TODO: load yaml, connect UrDriver, set up pubs/subs
}

FollowerNode::~FollowerNode() { stop(); }

void FollowerNode::run() {
  if (running_.exchange(true)) return;
  control_thread_ = std::thread(&FollowerNode::control_loop, this);
}

void FollowerNode::stop() {
  running_ = false;
  if (control_thread_.joinable()) control_thread_.join();
}

void FollowerNode::control_loop() {
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

    // TODO: read robot state, compute KP_TRACK*(peer_q - q), write torque.

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
