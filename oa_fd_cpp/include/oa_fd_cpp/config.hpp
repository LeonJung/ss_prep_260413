// config.hpp — configuration schema + YAML loader for oa_fd_cpp.
//
// PER-ARM config: each of the 4 arms (leader/follower x left/right) has its
// OWN self-contained yaml file (config/oa_fd_<role>_<side>.yaml) loaded into
// one ArmCfg. Friction, gravity model, gains, limits, freedrive shaping all
// differ per arm, so nothing is shared between arms except a few loop-level
// globals (GlobalCfg: timestep, filter, homing, CAN fd/timeout).
//
// Per joint MIT command  tau = Kp(q_ref-q) + Kd(dq_ref-dq) + g(q) + friction(q̇)
//   bilateral: leader_ref = follower state, follower_ref = leader state.

#pragma once
#include <array>
#include <string>

namespace oa_fd {

constexpr int DOF = 7;                 // OpenArm A2 arm joints (gripper excluded)
using Vec7 = std::array<double, DOF>;

// ---- gravity-model build params (interface to GravityModel::load) --------
struct GravityCfg {
  bool        enabled  = true;
  std::string urdf;
  std::string root_link = "openarmx_link0";
  std::string tip_link  = "openarmx_link7";
  std::array<double, 3> vec = {0.0, 0.0, 9.81};
  double      scale    = 1.0;
  std::array<double, DOF> scale_joints = {1, 1, 1, 1, 1, 1, 1};
};

// ---- one arm's complete parameter set (one yaml file) --------------------
struct ArmCfg {
  std::string can = "can0";            // CAN interface for this arm

  // gravity model (KDL JntToGravity on `urdf`). vec/mirror per this arm's
  // mounting; mirror flips the joint axes for the right arm (sagittal mirror).
  bool        grav_enabled = true;
  std::string grav_urdf;               // path; launch passes it
  std::string root_link = "openarmx_link0";
  std::string tip_link  = "openarmx_link7";
  std::array<double, 3> grav_vec = {0.0, 0.0, 9.81};
  double      grav_scale = 1.0;
  Vec7        grav_scale_joints = {1, 1, 1, 1, 1, 1, 1};
  Vec7        grav_mirror = {1, 1, 1, 1, 1, 1, 1};

  // MIT impedance + motion
  Vec7 Kp = {120, 120, 120, 120, 18, 20, 16};
  Vec7 Kd = {2.0, 2.0, 2.0, 2.0, 0.2, 0.2, 0.2};
  Vec7 home = {0, 0, 0, 0, 0, 0, 0};
  Vec7 torque_limit = {40, 40, 25, 25, 8, 8, 8};

  // leader<->follower joint-correspondence mirror (this arm's pair)
  Vec7 couple_mirror = {1, 1, 1, 1, 1, 1, 1};

  // tanh friction: f = gate(|v|) * (Fc*tanh(k*v) + Fv*v + Fo)
  Vec7 fric_Fc = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fric_k  = {8, 8, 4, 4, 4, 4, 4};
  Vec7 fric_Fv = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fric_Fo = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fric_v_start = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fric_v_full  = {0, 0, 0, 0, 0, 0, 0};

  // FREEDRIVE shaping: posture spring + motor-side joint-limit repulsion
  Vec7 fd_posture_kp = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_posture_kd = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_posture_q  = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_limit_kp   = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_limit_kd   = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_limit_fmax = {1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9};
  Vec7 fd_limit_push = {0, 0, 0, 0, 0, 0, 0};
  Vec7 fd_limit_margin = {0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15};
  double fd_limit_exit_kd = 1.0;
  // joint-limit boundaries [rad] (defaults effectively disabled)
  Vec7 limit_lower = {-1e9, -1e9, -1e9, -1e9, -1e9, -1e9, -1e9};
  Vec7 limit_upper = { 1e9,  1e9,  1e9,  1e9,  1e9,  1e9,  1e9};
};

// ---- loop-level globals (one per process; read from any arm file) --------
struct GlobalCfg {
  bool   can_fd = false;
  int    recv_timeout_us = 500;
  double timestep = 0.001;
  double vel_filter_alpha = 1.0;
  bool   auto_home_on_start = false;
  double homing_duration = 5.0;
  // ACTIVE coupling: scale on the PEER-velocity feedforward (dq_ref) put into
  // the MIT vel field. dq_ref is the peer velocity through the CAN delay, so
  // kd*(dq_ref-dq) is DELAYED velocity feedback -> non-passive -> vibration
  // (the same D2 trap we avoid elsewhere). 0.0 => vel_ref=0 => kd is pure
  // delay-free LOCAL damping (passivity-safe, default). 1.0 => full peer-vel FF
  // (sharper tracking, may vibrate). Tune in [0,1].
  double couple_vel_ff = 0.0;
};

// Load one arm's yaml file into `arm` (and refresh loop globals `g` from it;
// every file carries the same global block, last-loaded wins). Returns false
// on read failure.
bool load_arm_config(const std::string& path, ArmCfg& arm, GlobalCfg& g);

}  // namespace oa_fd
