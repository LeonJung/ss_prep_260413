#pragma once

#include <Eigen/Dense>
#include <cstddef>

namespace ur10e_teleop_control_hybrid_cpp {

// First-order LPF on the numerical derivative of a joint-space position
// signal. One instance per robot side. Not thread-safe.
class VelocityEstimator {
 public:
  VelocityEstimator(std::size_t n_joints, double cutoff_hz, double dt);

  // Advance one step with a new position sample. Returns the filtered
  // velocity estimate.
  const Eigen::VectorXd& update(const Eigen::VectorXd& q);

  // Reinitialize with the current position; velocity is reset to zero.
  // Call on mode transitions (PAUSED → ACTIVE, HOMING end, etc.) to
  // avoid a spurious derivative spike.
  void reset(const Eigen::VectorXd& q);

  const Eigen::VectorXd& value() const { return q_dot_hat_; }
  std::size_t n_joints() const { return n_; }

 private:
  std::size_t n_;
  double dt_;
  double alpha_;
  Eigen::VectorXd q_prev_;
  Eigen::VectorXd q_dot_hat_;
  bool initialized_;
};

}  // namespace ur10e_teleop_control_hybrid_cpp
