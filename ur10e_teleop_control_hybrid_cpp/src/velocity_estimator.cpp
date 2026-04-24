#include "ur10e_teleop_control_hybrid_cpp/velocity_estimator.hpp"

#include <cmath>
#include <stdexcept>

namespace ur10e_teleop_control_hybrid_cpp {

VelocityEstimator::VelocityEstimator(std::size_t n_joints,
                                     double cutoff_hz,
                                     double dt)
    : n_(n_joints), dt_(dt), initialized_(false) {
  if (n_ == 0) throw std::invalid_argument("VelocityEstimator: n_joints == 0");
  if (dt_ <= 0.0) throw std::invalid_argument("VelocityEstimator: dt <= 0");
  if (cutoff_hz <= 0.0)
    throw std::invalid_argument("VelocityEstimator: cutoff_hz <= 0");

  const double wc_dt = 2.0 * M_PI * cutoff_hz * dt_;
  alpha_ = wc_dt / (1.0 + wc_dt);
  q_prev_ = Eigen::VectorXd::Zero(n_);
  q_dot_hat_ = Eigen::VectorXd::Zero(n_);
}

const Eigen::VectorXd& VelocityEstimator::update(const Eigen::VectorXd& q) {
  if (!initialized_) {
    q_prev_ = q;
    q_dot_hat_.setZero();
    initialized_ = true;
    return q_dot_hat_;
  }
  const Eigen::VectorXd raw = (q - q_prev_) / dt_;
  q_dot_hat_ = alpha_ * raw + (1.0 - alpha_) * q_dot_hat_;
  q_prev_ = q;
  return q_dot_hat_;
}

void VelocityEstimator::reset(const Eigen::VectorXd& q) {
  q_prev_ = q;
  q_dot_hat_.setZero();
  initialized_ = true;
}

}  // namespace ur10e_teleop_control_hybrid_cpp
