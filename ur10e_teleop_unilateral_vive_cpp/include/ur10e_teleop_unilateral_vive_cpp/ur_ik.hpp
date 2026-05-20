// ur_ik.hpp — iterative inverse kinematics for UR e-Series.
//
// Strategy: damped least squares (DLS) using the analytical Jacobian
// already in ur_jacobian.hpp. Warm-started from the current joint
// solution, so consecutive Vive-tracker samples (which are spatially
// close at 500 Hz) converge in 1–3 iterations.
//
// Not full closed-form 8-branch analytical IK — the iterative path is
// simpler, monotone (no branch flips), and matches the "joint-stay-
// close-to-previous" behavior that a human-driven leader naturally
// produces. Singularities are handled by the DLS damping term.

#pragma once

#include <array>
#include <string>

#include <Eigen/Dense>

namespace ur10e_teleop_unilateral_vive_cpp {

struct UrIkOptions {
  double pos_tol_m   = 1.0e-4;   // 0.1 mm
  double rot_tol_rad = 1.0e-3;   // ~0.057 deg
  double damping     = 1.0e-2;   // DLS damping factor (lambda)
  double step_clamp  = 0.3;      // max |delta q| per iter (rad)
  int    max_iter    = 30;
};

// Solve IK from current q.
//   target_T : desired T_base_ee (4×4)
//   q_in     : current joint angles, warm-start
//   robot    : "ur3e" | "ur10e" | "ur5e" (passed through to ur_jacobian)
//   q_out    : on success, the new joint solution
// Returns true on convergence (both pos & rot tolerances).
bool ur_ik_solve(const Eigen::Matrix4d& target_T,
                 const std::array<double, 6>& q_in,
                 const std::string& robot,
                 std::array<double, 6>& q_out,
                 const UrIkOptions& opts = {});

}  // namespace ur10e_teleop_unilateral_vive_cpp
