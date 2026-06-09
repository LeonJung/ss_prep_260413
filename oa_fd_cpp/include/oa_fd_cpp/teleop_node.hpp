// teleop_node.hpp — bimanual enactic-style force-feedback bilateral node.
//
// 2 pairs on one PC, 4 USB2CAN (R: can0<->can2, L: can1<->can3).
// Per joint, each arm is commanded in MIT mode:
//   tau_motor = Kp(q_ref - q) + Kd(dq_ref - dq) + g(q) + friction(q̇)
// with cross-coupled references (leader_ref = follower state & vice-versa),
// reproducing enactic openarm_teleop's bilateral scheme.
//
// Mode (/oa/mode, Float64MultiArray [m,t,dur]): 0=ACTIVE 1=PAUSED 2=HOMING 3=FREEDRIVE

#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "oa_fd_cpp/config.hpp"
#include "oa_fd_cpp/gravity.hpp"
#include "oa_fd_cpp/oax_driver.hpp"
#include "oa_fd_cpp/rt_thread.hpp"

namespace oa_fd {

enum Mode : int { MODE_ACTIVE = 0, MODE_PAUSED = 1, MODE_HOMING = 2, MODE_FREEDRIVE = 3 };

// MIT command for one arm.
struct MitCmd { Vec7 kp{}, kd{}, pos{}, vel{}, tau{}; };

struct Pair {
  std::string name;
  std::unique_ptr<OaxArm> leader;
  std::unique_ptr<OaxArm> follower;
  Vec7 mirror{};
  Vec7 lq{}, lqd{}, ltau{};
  Vec7 fq{}, fqd{}, ftau{};
  Vec7 l_home_start{}, f_home_start{};
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

  bool connect();
  void run();
  void stop();

private:
  void mode_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_mode(int mode, double t_start = 0.0, double duration = 0.0);
  void control_loop();

  // Fill MIT commands for both arms of a pair.
  void compute_pair(Pair& p, int mode, double now_sec,
                    double h_t_start, double h_duration,
                    MitCmd& leader_cmd, MitCmd& follower_cmd);
  void friction(const Vec7& qd, Vec7& f) const;
  void publish_pair(Pair& p);

  Options opts_;
  OaFdConfig cfg_;
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

}  // namespace oa_fd
