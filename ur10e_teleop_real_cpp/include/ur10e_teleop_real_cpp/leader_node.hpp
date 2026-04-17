// leader_node.hpp — Leader (UR3e) ROS2 node with bilateral PD + deadband,
// mirroring ur10e_teleop_real_py/src/leader_real_node.py behavior.
//
// Skeleton only; control loop to be filled in Phase 4/5.

#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "ur10e_teleop_real_cpp/rt_thread.hpp"

namespace ur10e_teleop_real_cpp {

class LeaderNode : public rclcpp::Node {
public:
  struct Options {
    std::string robot_ip{"169.254.186.94"};
    std::string robot_type{"ur3e"};
    std::string config_path;
    bool use_rt = false;
    int  rt_priority = 80;
    int  rt_cpu = -1;
  };

  explicit LeaderNode(const Options& opts);
  ~LeaderNode() override;

  // Start the control thread (non-blocking).
  void run();
  // Request stop and join the control thread.
  void stop();

private:
  void control_loop();

  Options opts_;
  RTConfig rt_cfg_;
  std::atomic<bool> running_{false};
  std::thread control_thread_;
};

}  // namespace ur10e_teleop_real_cpp
