#pragma once

#include <Eigen/Dense>

#include "ur10e_teleop_control_hybrid_cpp/dynamics_model.hpp"

namespace ur10e_teleop_control_hybrid_cpp {

// Joint-space Disturbance Observer (Ohnishi / Sariyildiz style).
//
// Inverts the rigid-body model to compute the residual torque that
// cannot be explained by the commanded τ, then low-passes it to get a
// clean external-torque estimate τ̂_ext:
//
//   residual(t) = M(q) q̈̂ + C(q,q̇̂) q̇̂ + g(q) − τ_applied
//   τ̂_ext(t)   = Q(s) · residual(t)
//
// Q(s) is a first-order LPF with cutoff ω_c (rad/s). A viscous term
// D·q̇̂ may optionally be folded into the residual to decouple modelled
// damping from the "external" estimate.
//
// q̈̂ is obtained by differentiating q̇̂ and filtering with a separate
// cutoff ω_a (typically 1.5–2× ω_c to avoid phase compounding).
//
// Not thread-safe; owns no dynamics state other than filter memory.
// Holds a reference to the caller's DynamicsModel — the caller must
// outlive this object.
class DisturbanceObserver {
 public:
  struct Params {
    double cutoff_hz{60.0};         // Q-filter cutoff for τ̂_ext
    double accel_cutoff_hz{100.0};  // pre-filter on q̈̂
    Eigen::VectorXd viscous;        // per-joint D (size 6, zero-init if empty)
  };

  DisturbanceObserver(DynamicsModel& dyn, const Params& p, double dt);

  // One RT cycle. Returns τ̂_ext (6x1).
  //   q           : measured joint position
  //   q_dot_hat   : filtered velocity (from VelocityEstimator)
  //   tau_applied : torque command sent to the robot this cycle (before
  //                 any modulation from the energy tank; see note)
  const Eigen::VectorXd& update(const Eigen::VectorXd& q,
                                const Eigen::VectorXd& q_dot_hat,
                                const Eigen::VectorXd& tau_applied);

  // Reset internal state on mode transitions. After reset, τ̂_ext is
  // zero and acceleration estimate is zero.
  void reset(const Eigen::VectorXd& q_dot_hat);

  const Eigen::VectorXd& value() const { return tau_ext_hat_; }
  const Eigen::VectorXd& accel() const { return q_ddot_hat_; }

 private:
  DynamicsModel& dyn_;
  double dt_;
  double alpha_q_;    // α for Q-filter
  double alpha_a_;    // α for accel pre-filter
  Eigen::VectorXd D_; // viscous

  bool initialized_;
  Eigen::VectorXd qd_prev_;
  Eigen::VectorXd q_ddot_hat_;
  Eigen::VectorXd tau_ext_hat_;
};

}  // namespace ur10e_teleop_control_hybrid_cpp
