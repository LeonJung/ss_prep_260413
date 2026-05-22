// config.hpp — YAML config loader for both leader and follower.
// Mirrors fields used in ur10e_teleop_real_py/config/real_ur.yaml.

#pragma once
#include <array>
#include <string>
#include <vector>

namespace ur10e_teleop_unilateral_vive_cpp {

using Vec6 = std::array<double, 6>;
using Vec3a = std::array<double, 3>;     // distinct from ur_jacobian's
                                          // Eigen-backed Vec3 (Vector3d)

struct ControlConfig {
  // ---- firmware-handled compensation ----
  bool friction_comp = true;         // direct_torque(tau, friction_comp=True)
  bool gravity_comp_internal = true; // firmware internal gravity comp

  // ---- per-robot torque limits (Nm) ----
  Vec6 torque_limit           = {10, 10, 10, 10, 10, 10};
  Vec6 leader_torque_limit    = {10, 10, 10, 10, 10, 10};
  Vec6 follower_torque_limit  = {10, 10, 10, 10, 10, 10};
  bool has_leader_torque_limit   = false;
  bool has_follower_torque_limit = false;

  // ---- per-robot home (rad) ----
  // Single-arm defaults (parent unilateral package style).
  Vec6 leader_home   = {0, 0, 0, 0, 0, 0};
  Vec6 follower_home = {0, 0, 0, 0, 0, 0};
  // Bimanual side-specific homes. Defaults equal the single-arm values
  // above so single-arm configs keep working unchanged. Override one
  // or both per side in YAML if a bimanual physical mounting needs them.
  Vec6 leader_home_left;
  Vec6 leader_home_right;
  Vec6 follower_home_left;
  Vec6 follower_home_right;
  bool has_leader_home_left{false};
  bool has_leader_home_right{false};
  bool has_follower_home_left{false};
  bool has_follower_home_right{false};

  // ---- joint mirroring ----
  Vec6 mirror_sign = {1, 1, 1, 1, 1, 1};

  // ---- auto-homing ----
  bool   auto_home_on_start       = false;
  double homing_duration          = 5.0;    // seconds
  double homing_contact_threshold = 0.3;    // fraction of OVERFORCE_CONSTRAINT

  // ---- auto power-cycle (Dashboard Server, port 29999) ----
  bool   auto_power_cycle         = false;

  // ---- workspace (TCP bounding box, 2-tier virtual wall) ----
  bool                    ws_enabled           = false;
  std::array<double, 3>   ws_xyz_min           = {-1e9, -1e9, -1e9};
  std::array<double, 3>   ws_xyz_max           = { 1e9,  1e9,  1e9};
  double                  ws_k_wall            = 2000.0;   // N/m
  double                  ws_soft_penetration  = 0.02;     // m (hard-escalation threshold)

  // ---- leader gains ----
  Vec6 leader_kp_user   = {0, 0, 0, 0, 0, 0};
  Vec6 leader_kd_user   = {0, 0, 0, 0, 0, 0};
  Vec6 leader_kp_hold   = {100, 100, 50, 25, 25, 25};
  Vec6 leader_kd_hold   = {20, 20, 12, 10, 10, 10};
  Vec6 leader_kp_bi     = {100, 100, 50, 25, 25, 25};
  Vec6 leader_kd_bi     = {0, 0, 0, 0, 0, 0};
  Vec6 leader_tau_bi_deadband          = {0, 0, 0, 0, 0, 0};
  Vec6 leader_overforce_user           = {1000, 1000, 1000, 1000, 1000, 1000};
  Vec6 leader_overforce_constraint     = {30, 30, 30, 30, 30, 30};

  // ---- follower gains ----
  Vec6 follower_kp_track               = {300, 300, 150, 100, 100, 80};
  Vec6 follower_kd_track               = {30, 30, 20, 10, 10, 10};
  Vec6 follower_kp_hold                = {100, 100, 50, 25, 25, 25};
  Vec6 follower_kd_hold                = {20, 20, 12, 10, 10, 10};
  Vec6 follower_overforce_constraint   = {60, 60, 60, 30, 30, 30};

  // ---- friction model (only used when friction_comp = false) ----
  Vec6 friction_fc = {0, 0, 0, 0, 0, 0};
  Vec6 friction_fv = {0, 0, 0, 0, 0, 0};
  Vec6 friction_k  = {50, 50, 50, 50, 50, 50};

  // ---- tool offset (Vive teleop) ----
  // The "EE" point that the operator perceives is offset from the UR
  // flange (the joint-6 frame that FK returns). With tool_mode = "arm"
  // the offset is zero — rotation of the tracker about its own center
  // ends up as a pure q5 spin (flange Z rotation). With tool_mode =
  // "hand" the offset moves the EE to the dg5f gripper palm center;
  // rotating the tracker then forces the arm to swing the flange
  // around the palm point, which is the natural feel.
  std::string tool_mode = "arm";  // "arm" | "hand"
  Vec3a tool_offset_arm  = {0.0, 0.0, 0.0};     // flange = EE
  Vec3a tool_offset_hand = {0.0, 0.02, 0.15};   // dg5f palm in flange frame
                                                 // (Z forward; tweak Y per real mount)

  // ---- control loop ----
  double timestep = 0.002;  // 500 Hz
};

// Load config from a YAML file. Returns true on success, fills `out`.
// On partial failure (missing fields), prints warning and keeps defaults.
bool load_config(const std::string& path, ControlConfig& out);

}  // namespace ur10e_teleop_unilateral_vive_cpp
