// vive_leader_node.hpp — Bimanual Vive-tracker leader.
//
// One process drives up to two trackers (left + right) simultaneously
// from a single OpenVR session. Each tracker is independent:
//   - own SteamVR serial
//   - own tracker→UR-base calibration
//   - own UR3e IK virtual joint state
//   - own /<prefix>/leader/joint_state topic
// but they share the mode state machine and the reset path.
//
// Bimanual layout (default):
//   left  → /ur10e/left/leader/joint_state
//   right → /ur10e/right/leader/joint_state
//   mode  → /ur10e/mode             (latched, shared)
//   reset → /ur10e/reset            (latched, shared)
//
// Single-arm fallback: leave one side's serial empty; only the
// configured side runs.

#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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

enum Mode : int {
  MODE_ACTIVE    = 0,
  MODE_PAUSED    = 1,
  MODE_HOMING    = 2,
  MODE_FREEDRIVE = 3,
};

// Per-arm configuration. tracker_serial="" disables this side.
struct ArmOptions {
  std::string tracker_serial;        // "" = disabled
  std::string calib_path;            // YAML; "" = identity transform
  std::string topic_prefix{"/ur10e/left"};  // publish to <prefix>/leader/joint_state
};

class ViveLeaderNode : public rclcpp::Node {
 public:
  struct Options {
    ArmOptions left;
    ArmOptions right;

    std::string robot_type{"ur3e"};
    std::string config_path;
    double      control_rate_hz{500.0};
    bool        use_rt = false;
    int         rt_priority = 80;
    int         rt_cpu = -1;
    double      tracker_init_timeout_sec{10.0};
    double      dq_filter_alpha{0.2};
  };

  explicit ViveLeaderNode(const Options& opts);
  ~ViveLeaderNode() override;

  // Init trackers (one per configured arm) + start control thread.
  // Returns false if any configured arm failed to bind its tracker.
  bool run();
  void stop();

 private:
  // Per-arm runtime state.
  struct Arm {
    ArmOptions opts;
    std::unique_ptr<ViveTracker> tracker;
    Calibration calib;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
    // current published joint state
    std::array<double, 6> q{};
    std::array<double, 6> dq{};
    std::array<double, 6> q_prev{};
    std::array<double, 6> q_home_start{};
  };

  void control_loop();
  void tick_arm(Arm& arm, int cur_state, double t_now,
                double m_t_start, double m_duration);

  // ROS callbacks
  void mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void reset_cb(const std_msgs::msg::Int32::SharedPtr msg);

  // pub helpers
  void publish_arm_state(Arm& arm);
  void publish_mode(int mode, double t_start = 0.0, double duration = 0.0);

  Options opts_;
  ControlConfig cfg_;
  RTConfig rt_cfg_;

  // shared sessions / state
  std::vector<Arm> arms_;          // 1 or 2 entries depending on Options

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reset_sub_;

  // mode state machine (shared across arms)
  std::mutex mode_mtx_;
  int mode_state_ = MODE_PAUSED;
  double mode_t_start_ = 0.0;
  double mode_duration_ = 0.0;
  std::atomic<int> reset_counter_{0};

  Vec6 home_qpos_{};       // shared default home (same for both arms for now)

  std::atomic<bool> running_{false};
  std::thread control_thread_;
};

}  // namespace ur10e_teleop_unilateral_vive_cpp
