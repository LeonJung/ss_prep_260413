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
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
  Vec7 lqd_f{}, fqd_f{};   // low-pass velocities (friction gate / zone logic;
                           // raw qd is ±0.15 rad/s noisy at standstill)
  Vec7 l_home_start{}, f_home_start{};   // captured at HOMING entry
  Vec7 l_hold{}, f_hold{};               // captured at PAUSED entry (hold-in-place)
  GravityModel* grav_lead = nullptr;  // leader gravity model (handle tip)
  GravityModel* grav_foll = nullptr;  // follower gravity model (gripper tip)
  const ArmCfg* lead_cfg = nullptr;   // this pair's leader arm params
  const ArmCfg* foll_cfg = nullptr;   // this pair's follower arm params
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr leader_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr follower_pub;
};

class TeleopNode : public rclcpp::Node {
public:
  struct Options {
    // one self-contained yaml per arm
    std::string cfg_leader_left, cfg_leader_right;
    std::string cfg_follower_left, cfg_follower_right;
    // optional gravity-URDF overrides per arm (else taken from the arm's yaml)
    std::string urdf_leader_left, urdf_leader_right;
    std::string urdf_follower_left, urdf_follower_right;
    std::string arms = "both";   // --arms right|left|both : which pair(s) to drive
    std::string role = "both";   // --role leader|follower|both : which side of each pair
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
  // friction for one arm (its own ArmCfg).
  // apply_gate: zero friction comp near standstill (needed only in FREEDRIVE to
  // kill the negative-damping runaway). In Kp-coupled modes (ACTIVE/PAUSED/
  // HOMING) the spring stabilizes, so full friction comp is used (no gate) —
  // matches enactic, removes the slow-motion drag/heaviness.
  void friction(const ArmCfg& a, const Vec7& qd, Vec7& f, bool apply_gate) const;
  void publish_pair(Pair& p);

  Options opts_;
  GlobalCfg g_;                         // loop-level globals
  // 4 per-arm configs (each from its own yaml).
  ArmCfg cfg_lead_left_, cfg_lead_right_, cfg_foll_left_, cfg_foll_right_;
  RTConfig rt_cfg_;
  // 4 gravity models, one per arm.
  GravityModel grav_lead_left_, grav_lead_right_;
  GravityModel grav_foll_left_, grav_foll_right_;

  Pair right_, left_;
  std::vector<Pair*> pairs_;   // active pairs per Options::arms (right/left/both)
  bool drive_leader_   = true; // per Options::role
  bool drive_follower_ = true;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mode_sub_;

  std::mutex mode_mtx_;
  int    mode_state_   = MODE_FREEDRIVE;   // safe startup: gravity only, no stiff pull
  double mode_t_start_ = 0.0;
  double mode_duration_ = 0.0;

  // ACTIVE soft-engage: on entry the leader/follower may be far apart; ramping
  // the COUPLING stiffness (Kp/Kd + velocity FF) 0->full over this window means
  // no slam regardless of the initial mismatch. Gravity/friction FF stay full
  // the whole time (always safe). Captured at the FREEDRIVE/PAUSED->ACTIVE edge.
  double active_t_start_   = 0.0;
  double active_engage_s_  = 1.5;   // ramp duration [s]

  std::atomic<bool> running_{false};
  std::thread control_thread_;

  // TEMP diagnostic: per-cycle CSV log (long format, one row per joint). Opened
  // only if env OA_FD_LOG is set to a path; off otherwise (zero overhead).
  std::ofstream dbg_log_;
};

}  // namespace oa_fd
