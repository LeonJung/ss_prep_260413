#include "ur10e_teleop_unilateral_vive_cpp/ur_ik.hpp"

#include <algorithm>
#include <cmath>

#include "ur10e_teleop_unilateral_vive_cpp/ur_jacobian.hpp"

namespace ur10e_teleop_unilateral_vive_cpp {

namespace {

// Pack pose error into a 6-vector [linear_error; angular_error] in
// base frame. Angular error from the rotation residual R_err = R_t * R_c^T
// taken as the axis-angle vector (roughly the small-angle approximation
// near convergence, but exact near zero).
Eigen::Matrix<double, 6, 1> pose_error(const Eigen::Matrix4d& target,
                                        const Eigen::Matrix4d& current) {
  Eigen::Matrix<double, 6, 1> e;
  // linear: target_pos − current_pos
  e.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);

  // angular: log(R_target * R_current^T) → axis * angle
  Eigen::Matrix3d R_err =
      target.block<3, 3>(0, 0) * current.block<3, 3>(0, 0).transpose();
  Eigen::AngleAxisd aa(R_err);
  e.tail<3>() = aa.axis() * aa.angle();
  return e;
}

}  // namespace

bool ur_ik_solve(const Eigen::Matrix4d& target_T,
                 const std::array<double, 6>& q_in,
                 const std::string& robot,
                 std::array<double, 6>& q_out,
                 const UrIkOptions& opts) {
  std::array<double, 6> q = q_in;

  for (int it = 0; it < opts.max_iter; ++it) {
    // FK from current q
    Eigen::Matrix4d T_cur;
    forward_kinematics(q, robot, T_cur);

    // Pose error vector
    Eigen::Matrix<double, 6, 1> e = pose_error(target_T, T_cur);
    const double pos_err = e.head<3>().norm();
    const double rot_err = e.tail<3>().norm();
    if (pos_err < opts.pos_tol_m && rot_err < opts.rot_tol_rad) {
      q_out = q;
      return true;
    }

    // Jacobian + damped least squares step:
    //   dq = (J^T J + λ² I)^-1 J^T e
    Mat66 J = ur_jacobian(q, robot);
    Mat66 JtJ = J.transpose() * J;
    Mat66 dls = JtJ + (opts.damping * opts.damping) * Mat66::Identity();
    Eigen::Matrix<double, 6, 1> dq = dls.ldlt().solve(J.transpose() * e);

    // Clamp step magnitude to avoid overshoot at singularities
    double dq_max = dq.cwiseAbs().maxCoeff();
    if (dq_max > opts.step_clamp) {
      dq *= (opts.step_clamp / dq_max);
    }

    for (int i = 0; i < 6; ++i) q[i] += dq[i];
  }

  // Did not converge — return the best-effort q anyway so callers can
  // continue (Vive tracker noise at 500 Hz means an occasional non-
  // converged frame is acceptable as long as we don't latch it forever).
  q_out = q;
  return false;
}

}  // namespace ur10e_teleop_unilateral_vive_cpp
