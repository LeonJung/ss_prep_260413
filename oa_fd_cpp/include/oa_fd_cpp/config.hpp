// config.hpp — configuration schema + YAML loader for oa_fd_cpp.
//
// Enactic-style bilateral (force feedback) on OpenArm A2 (openarmx CAN):
//   per joint MIT command  tau = Kp(q_ref-q) + Kd(dq_ref-dq) + g(q) + friction(q̇)
//   cross-coupled references: leader_ref = follower state, follower_ref = leader.
// Single control PC, 4 USB2CAN:
//   leader_right(can0) <-> follower_right(can2);  leader_left(can1) <-> follower_left(can3)

#pragma once
#include <array>
#include <string>

namespace oa_fd {

constexpr int DOF = 7;                 // OpenArm A2 arm joints (gripper excluded)
using Vec7 = std::array<double, DOF>;

struct GravityCfg {
  bool        enabled  = true;
  std::string urdf;
  std::string root_link = "openarmx_link0";
  std::string tip_link  = "openarmx_link7";
  std::array<double, 3> vec = {0.0, 0.0, -9.81};
  double      scale    = 1.0;                       // global trim
  // per-joint trim, multiplied with `scale` (model error differs per joint —
  // e.g. j2 perfect at 0.95 global while j1 over-compensates).
  std::array<double, DOF> scale_joints = {1, 1, 1, 1, 1, 1, 1};
};

struct OaFdConfig {
  // ---- CAN role -> interface ----
  std::string can_leader_right   = "can0";
  std::string can_leader_left    = "can1";
  std::string can_follower_right = "can2";
  std::string can_follower_left  = "can3";
  bool        can_fd             = false;
  int         recv_timeout_us    = 500;

  // ---- loop ----
  double timestep = 0.001;             // enactic runs 1 kHz

  // ---- homing ----
  bool   auto_home_on_start = false;   // safety: no stiff pull-to-home on launch
  double homing_duration    = 5.0;

  // ---- per-joint ----
  Vec7 torque_limit = {40, 40, 25, 25, 8, 8, 8};
  Vec7 home         = {0, 0, 0, 0, 0, 0, 0};

  // ---- MIT impedance gains (motor-side) ----
  // Placeholders. enactic (Damiao) leader values: Kp[240,240,240,240,24,31,25],
  // Kd[3,3,3,3,0.2,0.2,0.2]. Robstride differs — RE-TUNE on hardware.
  Vec7 Kp = {120, 120, 120, 120, 18, 20, 16};
  Vec7 Kd = {2.0, 2.0, 2.0, 2.0, 0.2, 0.2, 0.2};

  // ---- tanh friction model: f = gate(|v|) * (Fc*tanh(k*v) + Fv*v + Fo) ----
  // Default OFF (zeros). Identify per joint on hardware before enabling.
  Vec7 fric_Fc = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fric_k  = {30, 30, 30, 30, 30, 30, 30};
  Vec7 fric_Fv = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fric_Fo = {0, 0, 0, 0, 0, 0, 0};
  // Velocity gate: 0 below v_start, ramps to 1 at v_full. Kills the
  // negative-damping slope at standstill that made FREEDRIVE (Kp=Kd=0)
  // run away when nudged. v_full <= v_start disables the gate (old behavior).
  double fric_v_start = 0.0;   // [rad/s]
  double fric_v_full  = 0.0;   // [rad/s]

  // ---- leader<->follower joint mirror, per side ----
  Vec7 mirror_right = {1, 1, 1, 1, 1, 1, 1};
  Vec7 mirror_left  = {1, 1, 1, 1, 1, 1, 1};

  // ---- FREEDRIVE shaping: weak posture spring + joint-limit repulsion ----
  // Spring (motor-side impedance toward posture_q) tames the 7-DOF q3<->q5
  // self-motion; repulsion (PC-side torque inside `limit_margin` of a
  // boundary) keeps joints off hard stops / out of forbidden ranges.
  // All zeros = the old pure gravity-only FREEDRIVE.
  Vec7 fd_posture_kp = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_posture_kd = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_posture_q  = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_limit_kp   = {0, 0, 0, 0, 0, 0, 0};  // Nm/rad inside the zone
  // damping applied ONLY inside the zone — absorbs the spring rebound
  // (acts in BOTH directions: hardens approach, slows the exit)
  Vec7 fd_limit_kd   = {0, 0, 0, 0, 0, 0, 0};  // Nm/(rad/s) inside the zone
  // spring fraction kept while exiting (1.0 = symmetric spring = catapult)
  double fd_limit_exit_scale = 0.25;
  Vec7 fd_limit_margin = {0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15};  // zone width [rad], per joint
  // Repulsion boundaries per side (defaults: effectively disabled).
  Vec7 limit_lower_left  = {-1e9, -1e9, -1e9, -1e9, -1e9, -1e9, -1e9};
  Vec7 limit_upper_left  = { 1e9,  1e9,  1e9,  1e9,  1e9,  1e9,  1e9};
  Vec7 limit_lower_right = {-1e9, -1e9, -1e9, -1e9, -1e9, -1e9, -1e9};
  Vec7 limit_upper_right = { 1e9,  1e9,  1e9,  1e9,  1e9,  1e9,  1e9};

  GravityCfg gravity;
  // Gravity vector expressed in each arm's URDF root frame. OpenArm mounting
  // usually makes this ±Y (NOT Z), opposite sign per side. Tune in FREEDRIVE.
  std::array<double, 3> grav_vec_right = {0.0, 0.0, -9.81};
  std::array<double, 3> grav_vec_left  = {0.0, 0.0, -9.81};
  // Per-side KDL chain endpoints. The bundled single-arm URDF
  // (urdf/openarmx_arm.urdf) uses generic names for both sides; only the
  // gravity vector differs left/right (set by mounting).
  std::string root_link_right = "openarmx_link0";
  std::string tip_link_right  = "openarmx_link7";
  std::string root_link_left  = "openarmx_link0";
  std::string tip_link_left   = "openarmx_link7";
};

bool load_config(const std::string& path, OaFdConfig& out);

}  // namespace oa_fd
