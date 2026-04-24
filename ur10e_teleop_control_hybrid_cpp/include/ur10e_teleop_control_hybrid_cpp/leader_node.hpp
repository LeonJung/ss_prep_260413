// leader_node.hpp — Leader (UR3e) ROS2 node with bilateral PD + deadband,
// mirroring ur10e_teleop_real_py/src/leader_real_node.py behavior.

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
#include <std_msgs/msg/int32.hpp>

#include <ur_client_library/ur/ur_driver.h>

#include "ur10e_teleop_control_hybrid_cpp/config.hpp"
#include "ur10e_teleop_control_hybrid_cpp/dashboard_client.hpp"
#include "ur10e_teleop_control_hybrid_cpp/rt_thread.hpp"

namespace ur10e_teleop_control_hybrid_cpp {

// Mode constants — match the Python nodes exactly.
enum Mode : int {
  MODE_ACTIVE = 0,
  MODE_PAUSED = 1,
  MODE_HOMING = 2,
  MODE_FREEDRIVE = 3,
};

class LeaderNode : public rclcpp::Node {
public:
  struct Options {
    std::string robot_ip{"169.254.186.94"};
    std::string robot_type{"ur3e"};
    std::string config_path;
    std::string resources_dir;   // contains rtde_*_recipe.txt + external_control.urscript
    bool use_rt = false;
    int  rt_priority = 80;
    int  rt_cpu = -1;
    // PC-side port base; each UrDriver uses 4 consecutive ports
    uint32_t reverse_port_base = 50001;
  };

  explicit LeaderNode(const Options& opts);
  ~LeaderNode() override;

  void run();
  void stop();

private:
  bool connect_robot();
  void control_loop();

  // --- callbacks ---
  void peer_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void reset_cb(const std_msgs::msg::Int32::SharedPtr msg);

  // --- publish helpers ---
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
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reset_sub_;

  // peer state (follower)
  std::mutex peer_mtx_;
  std::array<double, 6> peer_q_{};
  std::array<double, 6> peer_dq_{};
  // _ff: follower publishes tau_contact = J^T·F_TCP as joint effort.
  std::array<double, 6> peer_tau_{};
  bool peer_q_valid_ = false;

  // mode (subscribed — can be overridden by external publishers)
  std::mutex mode_mtx_;
  int mode_state_ = MODE_ACTIVE;
  double mode_t_start_ = 0.0;
  double mode_duration_ = 0.0;

  std::atomic<int> reset_counter_{0};

  // resolved once in ctor
  Vec6 home_qpos_{};         // this robot's home (leader_home)
  Vec6 peer_home_{};         // peer's home (follower_home)
  Vec6 mirror_sign_{};
  Vec6 torque_limit_{};      // leader_torque_limit or shared torque_limit

  std::atomic<bool> running_{false};
  std::thread control_thread_;
};

}  // namespace ur10e_teleop_control_hybrid_cpp
