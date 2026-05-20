// vive_leader_node.hpp — Leader node driven by a Vive tracker.
//
// Drop-in replacement for the UR3e-based leader in
// ur10e_teleop_control_unilateral_cpp. Publishes the same topics
//   /ur10e/leader/joint_state    (sensor_msgs/JointState)
//   /ur10e/mode                  (std_msgs/Float64MultiArray)
// so the existing follower (UR10e) needs no modification.
//
// Pipeline:
//   ViveTracker.poll()  →  Calibration.apply()  →  ur_ik_solve()
//                                                    →  JointState publish
//
// Mode semantics inherit from the UR3e leader's 4-mode state machine
// (ACTIVE / PAUSED / HOMING / FREEDRIVE). HOMING / PAUSED freeze the
// virtual joint output; ACTIVE / FREEDRIVE pass the IK result through.

#pragma once
#include <array>
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

#include "ur10e_teleop_unilateral_vive_cpp/calibration.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/config.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/rt_thread.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/vive_tracker.hpp"

namespace ur10e_teleop_unilateral_vive_cpp {

// Mode constants — must stay numerically identical to the UR3e leader
// so a Float64MultiArray published from anywhere works for both.
enum Mode : int {
  MODE_ACTIVE    = 0,
  MODE_PAUSED    = 1,
  MODE_HOMING    = 2,
  MODE_FREEDRIVE = 3,
};

class ViveLeaderNode : public rclcpp::Node {
 public:
  struct Options {
    std::string robot_type{"ur3e"};        // IK target frame (matches mirror_sign in follower)
    std::string config_path;
    std::string calib_path;                // YAML with T_ur_from_tracker
    std::string tracker_serial;            // empty = first tracker found
    double      control_rate_hz{500.0};
    bool        use_rt = false;
    int         rt_priority = 80;
    int         rt_cpu = -1;
    double      tracker_init_timeout_sec{10.0};
    // Low-pass on joint velocity (numerical diff is noisy at high rate):
    //   dq_filt[k] = alpha * dq_raw + (1 - alpha) * dq_filt[k-1]
    double      dq_filter_alpha{0.2};
  };

  explicit ViveLeaderNode(const Options& opts);
  ~ViveLeaderNode() override;

  // Start tracker + control thread. Returns false if tracker init failed.
  bool run();
  void stop();

 private:
  void control_loop();

  // ROS callbacks
  void peer_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void reset_cb(const std_msgs::msg::Int32::SharedPtr msg);

  // publishers
  void publish_state(const std::array<double, 6>& q,
                     const std::array<double, 6>& dq);
  void publish_mode(int mode, double t_start = 0.0, double duration = 0.0);

  Options opts_;
  ControlConfig cfg_;
  RTConfig rt_cfg_;

  std::unique_ptr<ViveTracker> tracker_;
  Calibration calib_;

  // ROS pubs / subs (same topic schema as the UR3e leader)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr peer_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reset_sub_;

  // mode state machine
  std::mutex mode_mtx_;
  int mode_state_ = MODE_PAUSED;
  double mode_t_start_ = 0.0;
  double mode_duration_ = 0.0;
  std::atomic<int> reset_counter_{0};

  // resolved once in ctor
  Vec6 home_qpos_{};

  std::atomic<bool> running_{false};
  std::thread control_thread_;
};

}  // namespace ur10e_teleop_unilateral_vive_cpp
