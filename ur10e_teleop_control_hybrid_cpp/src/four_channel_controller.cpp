#include "ur10e_teleop_control_hybrid_cpp/four_channel_controller.hpp"

#include <stdexcept>

namespace ur10e_teleop_control_hybrid_cpp {

namespace {
void require_size(const Eigen::VectorXd& v, std::size_t n, const char* name) {
  if (static_cast<std::size_t>(v.size()) != n) {
    throw std::invalid_argument(std::string(name) + ": size must be " +
                                std::to_string(n));
  }
}
}  // namespace

FourChannelController::FourChannelController(DynamicsModel& dyn,
                                             const Params& p)
    : dyn_(dyn), p_(p) {
  const std::size_t n = dyn_.n_joints();
  require_size(p_.Kp, n, "Kp");
  require_size(p_.Kd, n, "Kd");
  require_size(p_.Kf, n, "Kf");
  require_size(p_.D,  n, "D");
}

Eigen::Matrix<double, 6, 1> FourChannelController::compute(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_dot_hat,
    const Eigen::VectorXd& tau_ext_hat,
    const Eigen::VectorXd& q_peer,
    const Eigen::VectorXd& qd_peer,
    const Eigen::VectorXd& tau_ext_hat_peer,
    double ramp) {
  const auto& M  = dyn_.mass(q);
  const auto& C  = dyn_.coriolis(q, q_dot_hat);

  const Eigen::VectorXd e_pos = q_peer - q;
  const Eigen::VectorXd e_vel = qd_peer - q_dot_hat;
  const Eigen::VectorXd tau_sum = tau_ext_hat_peer + tau_ext_hat;

  Eigen::VectorXd u_inner = p_.Kp.cwiseProduct(e_pos)
                          + p_.Kd.cwiseProduct(e_vel)
                          + p_.Kf.cwiseProduct(tau_sum);
  u_inner *= ramp;

  Eigen::Matrix<double, 6, 1> tau =
        M * u_inner
      - tau_ext_hat
      + C
      + p_.D.cwiseProduct(q_dot_hat);
  if (!p_.firmware_grav_comp) {
    tau += dyn_.gravity(q);
  }
  return tau;
}

}  // namespace ur10e_teleop_control_hybrid_cpp
