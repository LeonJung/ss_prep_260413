// config.hpp — configuration schema + YAML loader for oa_pp_cpp.
//
// Single control PC, 4 USB2CAN interfaces, bimanual bilateral:
//   leader_right  (can0)  <-->  follower_right (can2)
//   leader_left   (can1)  <-->  follower_left  (can3)
// Two independent position-position bilateral pairs.

#pragma once
#include <array>
#include <string>

namespace oa_pp {

constexpr int DOF = 7;                 // OpenArm A2 arm joints (gripper excluded)
using Vec7 = std::array<double, DOF>;

struct GravityCfg {
  bool        enabled  = true;
  std::string urdf;                    // path to single-arm URDF (REQUIRED if enabled)
  std::string root_link = "openarmx_link0";
  std::string tip_link  = "openarmx_link7";
  std::array<double, 3> vec = {0.0, 0.0, -9.81};  // gravity in root frame
  double      scale    = 1.0;
};

struct OaPpConfig {
  // ---- CAN role -> interface name ----
  std::string can_leader_right   = "can0";
  std::string can_leader_left    = "can1";
  std::string can_follower_right = "can2";
  std::string can_follower_left  = "can3";
  bool        can_fd             = false;
  int         recv_timeout_us    = 500;

  // ---- loop ----
  double timestep = 0.002;             // 500 Hz (lower if 4x USB2CAN saturates)

  // ---- homing ----
  bool   auto_home_on_start = true;
  double homing_duration    = 5.0;     // s

  // ---- limits / poses (per joint) ----
  // Robstride tMax: RS04=120, RS03=60, RS00=14 -> keep well under for safety.
  Vec7 torque_limit = {40, 40, 25, 25, 8, 8, 8};
  Vec7 home         = {0, 0, 0, 0, 0, 0, 0};

  // ---- leader arm law (operator side) ----
  Vec7 leader_kp_bi   = {60, 60, 40, 30, 10, 10, 8};
  Vec7 leader_kd_bi   = {0, 0, 0, 0, 0, 0, 0};
  Vec7 leader_deadband = {0.6, 0.6, 0.4, 0.3, 0.15, 0.15, 0.12};  // Nm
  Vec7 leader_kp_hold = {80, 80, 50, 40, 12, 12, 10};
  Vec7 leader_kd_hold = {6, 6, 4, 3, 1, 1, 1};

  // ---- follower arm law ----
  Vec7 follower_kp_track = {120, 120, 80, 60, 18, 18, 14};
  Vec7 follower_kd_track = {6, 6, 4, 3, 1, 1, 1};
  Vec7 follower_kp_hold  = {80, 80, 50, 40, 12, 12, 10};
  Vec7 follower_kd_hold  = {6, 6, 4, 3, 1, 1, 1};

  // ---- leader<->follower joint mirror sign, per side ----
  Vec7 mirror_right = {1, 1, 1, 1, 1, 1, 1};
  Vec7 mirror_left  = {1, 1, 1, 1, 1, 1, 1};

  GravityCfg gravity;
};

bool load_config(const std::string& path, OaPpConfig& out);

}  // namespace oa_pp
