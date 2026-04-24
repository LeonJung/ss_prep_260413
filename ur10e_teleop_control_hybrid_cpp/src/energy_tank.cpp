#include "ur10e_teleop_control_hybrid_cpp/energy_tank.hpp"

#include <algorithm>
#include <stdexcept>

namespace ur10e_teleop_control_hybrid_cpp {

EnergyTank::EnergyTank(const Params& p, double dt)
    : dt_(dt), E_max_(p.E_max), E_init_(p.E_init),
      refill_ceiling_(p.refill_ceiling) {
  if (dt_ <= 0.0) throw std::invalid_argument("EnergyTank: dt <= 0");
  if (E_max_ <= 0.0) throw std::invalid_argument("EnergyTank: E_max <= 0");
  if (E_init_ < 0.0 || E_init_ > E_max_)
    throw std::invalid_argument("EnergyTank: E_init outside [0, E_max]");
  if (refill_ceiling_ <= 0.0 || refill_ceiling_ > 1.0)
    throw std::invalid_argument("EnergyTank: refill_ceiling ∉ (0, 1]");

  if (p.D_dissipation.size() == 0) {
    D_ = Eigen::VectorXd::Zero(6);
  } else {
    D_ = p.D_dissipation;
  }
  E_ = E_init_;
}

double EnergyTank::step(const Eigen::VectorXd& tau_active,
                        const Eigen::VectorXd& q_dot_hat) {
  // Refill from virtual dissipation — always non-negative.
  const double P_diss = q_dot_hat.dot(D_.cwiseProduct(q_dot_hat));
  double dE_refill = P_diss * dt_;
  // Throttle refill near the ceiling so the tank can't grow unboundedly
  // from a sustained "jiggle" attack.
  const double ceiling = refill_ceiling_ * E_max_;
  if (E_ + dE_refill > ceiling) {
    dE_refill = std::max(0.0, ceiling - E_);
  }
  last_refill_ = dE_refill;
  double E_after_refill = std::min(E_ + dE_refill, E_max_);

  // Power going out via the active torque (controller → environment).
  const double P_active = tau_active.dot(q_dot_hat);
  last_P_active_ = P_active;

  double alpha = 1.0;
  double dE_out;
  if (P_active > 0.0) {
    const double dE_needed = P_active * dt_;
    if (E_after_refill >= dE_needed) {
      alpha = 1.0;
      dE_out = dE_needed;
    } else {
      alpha = (dE_needed > 0.0) ? std::max(0.0, E_after_refill / dE_needed) : 0.0;
      dE_out = alpha * dE_needed;
    }
  } else {
    // Regenerative: controller absorbing energy — tank grows.
    alpha = 1.0;
    dE_out = P_active * dt_;  // negative → E grows
  }

  E_ = std::clamp(E_after_refill - dE_out, 0.0, E_max_);
  last_alpha_ = alpha;
  return alpha;
}

void EnergyTank::reset(double E) {
  E_ = (E < 0.0) ? E_init_ : std::clamp(E, 0.0, E_max_);
  last_alpha_ = 1.0;
  last_P_active_ = 0.0;
  last_refill_ = 0.0;
}

}  // namespace ur10e_teleop_control_hybrid_cpp
