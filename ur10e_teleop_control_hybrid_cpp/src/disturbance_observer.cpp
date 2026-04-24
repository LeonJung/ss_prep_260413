#include "ur10e_teleop_control_hybrid_cpp/disturbance_observer.hpp"

#include <cmath>
#include <stdexcept>

namespace ur10e_teleop_control_hybrid_cpp {

namespace {
double lpf_alpha(double cutoff_hz, double dt) {
  const double wc_dt = 2.0 * M_PI * cutoff_hz * dt;
  return wc_dt / (1.0 + wc_dt);
}
}  // namespace

DisturbanceObserver::DisturbanceObserver(DynamicsModel& dyn,
                                         const Params& p,
                                         double dt)
    : dyn_(dyn), dt_(dt), initialized_(false) {
  if (dt_ <= 0.0)
    throw std::invalid_argument("DisturbanceObserver: dt <= 0");
  if (p.cutoff_hz <= 0.0 || p.accel_cutoff_hz <= 0.0)
    throw std::invalid_argument("DisturbanceObserver: cutoff <= 0");

  alpha_q_ = lpf_alpha(p.cutoff_hz, dt_);
  alpha_a_ = lpf_alpha(p.accel_cutoff_hz, dt_);

  const std::size_t n = dyn_.n_joints();
  if (p.viscous.size() == 0) {
    D_ = Eigen::VectorXd::Zero(n);
  } else if (static_cast<std::size_t>(p.viscous.size()) == n) {
    D_ = p.viscous;
  } else {
    throw std::invalid_argument(
        "DisturbanceObserver: viscous size mismatch vs n_joints");
  }
  qd_prev_ = Eigen::VectorXd::Zero(n);
  q_ddot_hat_ = Eigen::VectorXd::Zero(n);
  tau_ext_hat_ = Eigen::VectorXd::Zero(n);
}

const Eigen::VectorXd& DisturbanceObserver::update(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_dot_hat,
    const Eigen::VectorXd& tau_applied) {
  // On first call, just seed and return zero — avoid spurious spikes
  // from uninitialized qd_prev_.
  if (!initialized_) {
    qd_prev_ = q_dot_hat;
    q_ddot_hat_.setZero();
    tau_ext_hat_.setZero();
    initialized_ = true;
    return tau_ext_hat_;
  }

  // Acceleration pre-filter
  const Eigen::VectorXd qdd_raw = (q_dot_hat - qd_prev_) / dt_;
  q_ddot_hat_ = alpha_a_ * qdd_raw + (1.0 - alpha_a_) * q_ddot_hat_;
  qd_prev_ = q_dot_hat;

  // Residual: model-predicted torque minus commanded torque
  const auto& M = dyn_.mass(q);
  const auto& Cqd = dyn_.coriolis(q, q_dot_hat);
  const auto& g = dyn_.gravity(q);

  Eigen::VectorXd residual = M * q_ddot_hat_ + Cqd + g
                             + D_.cwiseProduct(q_dot_hat)
                             - tau_applied;

  // Q-filter
  tau_ext_hat_ = alpha_q_ * residual + (1.0 - alpha_q_) * tau_ext_hat_;
  return tau_ext_hat_;
}

void DisturbanceObserver::reset(const Eigen::VectorXd& q_dot_hat) {
  qd_prev_ = q_dot_hat;
  q_ddot_hat_.setZero();
  tau_ext_hat_.setZero();
  initialized_ = true;
}

}  // namespace ur10e_teleop_control_hybrid_cpp
