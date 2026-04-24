#pragma once

#include <Eigen/Dense>

namespace ur10e_teleop_control_hybrid_cpp {

// Two-Layer passivity wrapper — bottom layer for the hybrid controller
// (Franken/Stramigioli 2011; Minelli 2023 refinements).
//
//   • The tank stores energy E(k) ∈ [0, E_max].
//   • Each cycle it refills from a virtual dissipation term (D·q̇² · dt)
//     and dispenses the energy actually sent out by the controller's
//     "active" torque (τ_active · q̇ · dt).
//   • When the desired active power cannot be sustained by the tank, the
//     step returns a scalar α ∈ [0,1] that scales τ_active down enough
//     to preserve passivity: α = E_available / (P_active · dt).
//
// The upper-layer controller (FourChannelController) is unchanged —
// integration is via the `ramp` argument of its compute(), which
// already scales only the Kp/Kd/Kf bracket (the "active" part).
class EnergyTank {
 public:
  struct Params {
    double E_max{5.0};            // J — cap on stored energy
    double E_init{2.5};           // J — initial tank level
    double refill_ceiling{0.9};   // fraction of E_max above which refill is clipped
    Eigen::VectorXd D_dissipation;  // size 6, Nms/rad — virtual damping that refills tank
  };

  EnergyTank(const Params& p, double dt);

  // Given the predicted "active" torque (τ_active = M · inner_bracket,
  // not yet committed) and the current velocity estimate, update the
  // tank and return the scaling factor α ∈ [0, 1].
  double step(const Eigen::VectorXd& tau_active,
              const Eigen::VectorXd& q_dot_hat);

  // Reset the tank level. A negative argument resets to E_init.
  void reset(double E = -1.0);

  double energy() const { return E_; }
  double last_alpha() const { return last_alpha_; }
  double last_P_active() const { return last_P_active_; }
  double last_refill() const { return last_refill_; }

 private:
  double dt_;
  double E_max_;
  double E_init_;
  double refill_ceiling_;
  Eigen::VectorXd D_;

  double E_{0.0};
  double last_alpha_{1.0};
  double last_P_active_{0.0};
  double last_refill_{0.0};
};

}  // namespace ur10e_teleop_control_hybrid_cpp
