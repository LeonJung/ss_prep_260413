#pragma once

#include <Eigen/Dense>

#include "ur10e_teleop_control_hybrid_cpp/disturbance_observer.hpp"
#include "ur10e_teleop_control_hybrid_cpp/dynamics_model.hpp"
#include "ur10e_teleop_control_hybrid_cpp/velocity_estimator.hpp"

namespace ur10e_teleop_control_hybrid_cpp {

// 4-Channel Lawrence bilateral control with model-based feedforward
// and a Disturbance Observer for sensorless τ̂_ext estimation (Tier B1,
// "Fast Bilateral Teleoperation" style).
//
//   τ_cmd = M(q){ Kp ⊙ (q_peer − q)
//               + Kd ⊙ (q̇_peer − q̇̂)
//               + Kf ⊙ (τ̂_ext_peer + τ̂_ext) }
//         − τ̂_ext
//         + C(q,q̇̂)·q̇̂ + D ⊙ q̇̂ + g(q)
//
// ⊙ is element-wise (per-joint gains). Kf = 0 recovers inverse-dynamics
// bilateral PD.
//
// This class does not own the observer/dynamics modules; the caller
// holds them as members and passes references. All sizes must be 6.
class FourChannelController {
 public:
  struct Params {
    Eigen::VectorXd Kp;        // size 6
    Eigen::VectorXd Kd;        // size 6
    Eigen::VectorXd Kf;        // size 6
    Eigen::VectorXd D;         // modeled viscous damping, size 6
    // When the actuator path itself adds g(q) (UR firmware grav-comp),
    // omit +g(q) from the controller output — sending it again would
    // double-compensate.
    bool firmware_grav_comp{false};
  };

  FourChannelController(DynamicsModel& dyn, const Params& p);

  // Compute a torque command.
  //   q, q_dot_hat     : this side's state
  //   tau_ext_hat      : this side's DOB estimate (filtered)
  //   q_peer, qd_peer  : peer state (already mirrored into this frame)
  //   tau_ext_hat_peer : peer's DOB estimate (already mirrored)
  //   ramp             : 0…1 scaling of the Kp/Kd/Kf bracket, used for
  //                      soft-start on mode transitions. Gravity +
  //                      Coriolis + DOB cancellation are never ramped so
  //                      the arm keeps compensating its own dynamics.
  Eigen::Matrix<double, 6, 1> compute(
      const Eigen::VectorXd& q,
      const Eigen::VectorXd& q_dot_hat,
      const Eigen::VectorXd& tau_ext_hat,
      const Eigen::VectorXd& q_peer,
      const Eigen::VectorXd& qd_peer,
      const Eigen::VectorXd& tau_ext_hat_peer,
      double ramp = 1.0);

  const Params& params() const { return p_; }
  void set_kf(const Eigen::VectorXd& kf) { p_.Kf = kf; }

 private:
  DynamicsModel& dyn_;
  Params p_;
};

}  // namespace ur10e_teleop_control_hybrid_cpp
