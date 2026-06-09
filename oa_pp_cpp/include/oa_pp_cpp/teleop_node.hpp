// teleop_node.hpp — single-process bimanual position-position bilateral node.
//
// 2 bilateral pairs on one control PC, 4 USB2CAN interfaces:
//   RIGHT pair : leader can0  <-->  follower can2
//   LEFT  pair : leader can1  <-->  follower can3
// One RT control thread reads all 4 arms, computes per-pair pos-pos coupling
// + gravity feedforward, and writes pure-torque MIT commands to all 4.
//
// Mode state machine (shared across both pairs), via /oa/mode:
//   0=ACTIVE  1=PAUSED  2=HOMING  3=FREEDRIVE   (Float64MultiArray [m,t,dur])

#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "oa_pp_cpp/config.hpp"
#include "oa_pp_cpp/gravity.hpp"
#include "oa_pp_cpp/oax_driver.hpp"
#include "oa_pp_cpp/rt_thread.hpp"

namespace oa_pp {

enum Mode : int { MODE_ACTIVE = 0, MODE_PAUSED = 1, MODE_HOMING = 2, MODE_FREEDRIVE = 3 };

// One leader<->follower bilateral pair (one side of the body).
struct Pair {
  std::string name;                 // "right" / "left"
  std::unique_ptr<OaxArm> leader;
  std::unique_ptr<OaxArm> follower;
  Vec7 mirror{};                    // leader<->follower joint sign
  // live state
  Vec7 lq{}, lqd{}, ltau{};
  Vec7 fq{}, fqd{}, ftau{};
  Vec7 l_home_start{}, f_home_start{};   // captured at HOMING entry
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr leader_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr follower_pub;
};

class TeleopNode : public rclcpp::Node {
public:
  struct Options {
    std::string config_path;
    bool use_rt = false;
    int  rt_priority = 80;
    int  rt_cpu = -1;
  };

  explicit TeleopNode(const Options& opts);
  ~TeleopNode() override;

  bool connect();      // init + enable all 4 arms; build gravity model
  void run();          // start RT control thread
  void stop();

private:
  void mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_mode(int mode, double t_start = 0.0, double duration = 0.0);
  void control_loop();

  // Fill leader/follower torque for one pair given current mode/time.
  void compute_pair(Pair& p, int mode, double now_sec, double active_t_start,
                    double h_t_start, double h_duration,
                    Vec7& tau_leader, Vec7& tau_follower);

  void publish_pair(Pair& p);

  Options opts_;
  OaPpConfig cfg_;
  RTConfig rt_cfg_;
  GravityModel grav_;

  Pair right_, left_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mode_sub_;

  std::mutex mode_mtx_;
  int    mode_state_   = MODE_PAUSED;
  double mode_t_start_ = 0.0;
  double mode_duration_ = 0.0;

  std::atomic<bool> running_{false};
  std::thread control_thread_;
};

}  // namespace oa_pp
