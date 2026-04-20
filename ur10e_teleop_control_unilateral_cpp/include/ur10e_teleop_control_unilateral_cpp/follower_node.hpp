// follower_node.hpp — Follower (UR10e) ROS2 node tracking leader via KP_TRACK.

#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ur_client_library/ur/ur_driver.h>

#include "ur10e_teleop_control_unilateral_cpp/config.hpp"
#include "ur10e_teleop_control_unilateral_cpp/dashboard_client.hpp"
#include "ur10e_teleop_control_unilateral_cpp/rt_thread.hpp"

namespace ur10e_teleop_control_unilateral_cpp {

class FollowerNode : public rclcpp::Node {
public:
  struct Options {
    std::string robot_ip{"169.254.186.92"};
    std::string robot_type{"ur10e"};
    std::string config_path;
    std::string resources_dir;
    bool use_rt = false;
    int  rt_priority = 80;
    int  rt_cpu = -1;
    // Follower's PC-side ports offset (from leader's 50001) so a same-PC
    // combined launch doesn't collide.
    uint32_t reverse_port_base = 50011;
  };

  explicit FollowerNode(const Options& opts);
  ~FollowerNode() override;

  void run();
  void stop();

private:
  bool connect_robot();
  void control_loop();

  void peer_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void publish_state(const std::array<double, 6>& q,
                     const std::array<double, 6>& dq);
  void publish_mode(int mode, double t_start = 0.0, double duration = 0.0);

  Options opts_;
  ControlConfig cfg_;
  RTConfig rt_cfg_;

  std::unique_ptr<urcl::UrDriver> driver_;
  std::unique_ptr<DashboardClient> dashboard_;
  std::atomic<bool> program_running_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr peer_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mode_sub_;

  std::mutex peer_mtx_;
  std::array<double, 6> peer_q_{};
  std::array<double, 6> peer_dq_{};
  bool peer_q_valid_ = false;

  std::mutex mode_mtx_;
  int mode_state_ = /*MODE_ACTIVE*/ 0;
  double mode_t_start_ = 0.0;
  double mode_duration_ = 0.0;

  // resolved once in ctor
  Vec6 home_qpos_{};         // follower_home
  Vec6 peer_home_{};         // leader_home
  Vec6 mirror_sign_{};
  Vec6 torque_limit_{};

  std::atomic<bool> running_{false};
  std::thread control_thread_;
};

}  // namespace ur10e_teleop_control_unilateral_cpp
