// Minimal sanity check for DynamicsModel + VelocityEstimator.
// Usage: dynamics_selftest <urdf_path> [base_link] [tip_link]
//   defaults: base_link=base_link  tip_link=tool0

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>

#include "ur10e_teleop_control_hybrid_cpp/disturbance_observer.hpp"
#include "ur10e_teleop_control_hybrid_cpp/dynamics_model.hpp"
#include "ur10e_teleop_control_hybrid_cpp/velocity_estimator.hpp"

using ur10e_teleop_control_hybrid_cpp::DisturbanceObserver;
using ur10e_teleop_control_hybrid_cpp::DynamicsModel;
using ur10e_teleop_control_hybrid_cpp::VelocityEstimator;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <urdf_path> [base_link] [tip_link]\n", argv[0]);
    return 1;
  }
  const std::string urdf = argv[1];
  const std::string base = argc >= 3 ? argv[2] : "base_link";
  const std::string tip  = argc >= 4 ? argv[3] : "tool0";

  try {
    DynamicsModel dyn(urdf, base, tip);
    std::printf("DynamicsModel: loaded (%zu joints) from %s→%s\n",
                dyn.n_joints(), base.c_str(), tip.c_str());

    // Home-ish pose: arm horizontal, elbow up 90 deg — so gravity bites
    // at shoulder-lift and elbow.
    Eigen::VectorXd q(6);
    q << 0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0;
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(6);
    qd(1) = 0.2;  // shoulder-lift moving

    auto M = dyn.mass(q);
    auto Cqd = dyn.coriolis(q, qd);
    auto g = dyn.gravity(q);

    std::printf("\nM(q) diagonal: ");
    for (int i = 0; i < 6; ++i) std::printf("%.4f ", M(i, i));
    std::printf("\n");
    std::printf("g(q) vector:   ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", g(i));
    std::printf("\n");
    std::printf("C(q,qd)qd:     ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", Cqd(i));
    std::printf("\n");

    // Sanity: at q_zero (all zeros) with gravity along -z, shoulder_pan
    // should have g≈0 (vertical axis), and wrist_3 also ≈0.
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
    auto g0 = dyn.gravity(q0);
    std::printf("g(0) vector:   ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", g0(i));
    std::printf("\n");

    // Timing
    const int N = 10000;
    auto t0 = std::chrono::steady_clock::now();
    volatile double sink = 0.0;
    for (int i = 0; i < N; ++i) {
      const double s = 1e-4 * i;
      Eigen::VectorXd qi = q + s * Eigen::VectorXd::Ones(6);
      sink += dyn.mass(qi)(0, 0);
      sink += dyn.coriolis(qi, qd)(1);
      sink += dyn.gravity(qi)(2);
    }
    auto t1 = std::chrono::steady_clock::now();
    const double us =
        std::chrono::duration<double, std::micro>(t1 - t0).count() / N;
    std::printf("\nper-iter M+C+g wall: %.2f us (sink=%g)\n", us,
                static_cast<double>(sink));

    // VelocityEstimator smoke test
    VelocityEstimator ve(6, 80.0, 0.002);
    Eigen::VectorXd qprev = q;
    for (int i = 0; i < 100; ++i) {
      qprev(0) += 0.002 * 1.0;  // 1 rad/s ramp on joint 0
      ve.update(qprev);
    }
    std::printf("VelocityEstimator: final q̇̂(0)=%.4f (expected ≈1.0)\n",
                ve.value()(0));

    // DisturbanceObserver sanity: robot held still at q, with tau_applied
    // = g(q) (perfect gravity comp). Expect τ̂_ext → 0 as filter settles.
    DisturbanceObserver::Params dp;
    dp.cutoff_hz = 60.0;
    dp.accel_cutoff_hz = 100.0;
    DisturbanceObserver dob(dyn, dp, 0.002);
    Eigen::VectorXd qd_zero = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd g_q = dyn.gravity(q);
    for (int i = 0; i < 2000; ++i) dob.update(q, qd_zero, g_q);
    std::printf("\nDOB @ still, tau=g(q):   τ̂_ext = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", dob.value()(i));
    std::printf("(expect ≈0)\n");

    // Now: tau_applied = 0 (robot not fighting gravity), still held still
    // externally. τ̂_ext should converge to g(q) (external = what holds
    // it up).
    dob.reset(qd_zero);
    for (int i = 0; i < 2000; ++i) dob.update(q, qd_zero, qd_zero);
    std::printf("DOB @ still, tau=0:      τ̂_ext = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", dob.value()(i));
    std::printf("(expect ≈ g(q))\n");
    std::printf("                         g(q)  = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", g_q(i));
    std::printf("\n");

    // Inject a fake external torque on joint 3 (+5 Nm). tau_applied still
    // = g(q). Residual should ≈ 5 Nm on joint 3.
    dob.reset(qd_zero);
    Eigen::VectorXd tau_fake_ext(6);
    tau_fake_ext << 0, 0, 0, 5.0, 0, 0;
    Eigen::VectorXd tau_app = g_q - tau_fake_ext;  // robot must undershoot
    for (int i = 0; i < 2000; ++i) dob.update(q, qd_zero, tau_app);
    std::printf("DOB + external +5Nm j3:  τ̂_ext = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", dob.value()(i));
    std::printf("(expect ≈ +5 on j3)\n");

    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "selftest failed: %s\n", e.what());
    return 2;
  }
}
