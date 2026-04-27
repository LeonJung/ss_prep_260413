// Minimal sanity check for DynamicsModel + VelocityEstimator.
// Usage: dynamics_selftest <urdf_path> [base_link] [tip_link]
//   defaults: base_link=base_link  tip_link=tool0

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>

#include "ur10e_teleop_control_hybrid_cpp/disturbance_observer.hpp"
#include "ur10e_teleop_control_hybrid_cpp/dynamics_model.hpp"
#include "ur10e_teleop_control_hybrid_cpp/energy_tank.hpp"
#include "ur10e_teleop_control_hybrid_cpp/four_channel_controller.hpp"
#include "ur10e_teleop_control_hybrid_cpp/velocity_estimator.hpp"

using ur10e_teleop_control_hybrid_cpp::DisturbanceObserver;
using ur10e_teleop_control_hybrid_cpp::DynamicsModel;
using ur10e_teleop_control_hybrid_cpp::EnergyTank;
using ur10e_teleop_control_hybrid_cpp::FourChannelController;
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

    // ---- FourChannelController sanity ----
    FourChannelController::Params cp;
    cp.Kp = Eigen::VectorXd::Constant(6, 10.0);
    cp.Kd = Eigen::VectorXd::Constant(6, 1.0);
    cp.Kf = Eigen::VectorXd::Zero(6);   // Phase-4 baseline
    cp.D  = Eigen::VectorXd::Zero(6);
    FourChannelController ctrl(dyn, cp);

    Eigen::VectorXd tau_ext_zero = Eigen::VectorXd::Zero(6);

    // (A) q == q_peer, zero velocities, zero τ̂_ext → tau should == g(q)
    auto tau_hold = ctrl.compute(q, qd_zero, tau_ext_zero,
                                 q, qd_zero, tau_ext_zero, 1.0);
    std::printf("\n4CH @ matched (Kf=0): τ    = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", tau_hold(i));
    std::printf("\n                      g(q)  = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", g_q(i));
    std::printf("(expect τ ≈ g(q))\n");

    // (B) q_peer = q + 0.1·e₂ → Kp·M coupling term on joint 2 dominant
    Eigen::VectorXd q_peer_test = q;
    q_peer_test(2) += 0.1;
    auto M_q = dyn.mass(q);
    Eigen::VectorXd e_expect(6); e_expect.setZero(); e_expect(2) = 0.1;
    Eigen::VectorXd tau_pd_expected = M_q * (cp.Kp.cwiseProduct(e_expect)) + g_q;
    auto tau_pd = ctrl.compute(q, qd_zero, tau_ext_zero,
                               q_peer_test, qd_zero, tau_ext_zero, 1.0);
    std::printf("4CH @ q_peer+0.1·e₂:  τ    = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", tau_pd(i));
    std::printf("\n                      M·Kp·e+g = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", tau_pd_expected(i));
    std::printf("(expect match)\n");

    // (C) ramp=0 → only gravity+C+D (no Kp/Kd/Kf contribution)
    auto tau_ramp0 = ctrl.compute(q, qd_zero, tau_ext_zero,
                                  q_peer_test, qd_zero, tau_ext_zero, 0.0);
    std::printf("4CH @ ramp=0:         τ    = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", tau_ramp0(i));
    std::printf("(expect ≈ g(q))\n");

    // (D) firmware_grav_comp=true: 4CH should NOT add g(q) to its output
    FourChannelController::Params cp_fw = cp;
    cp_fw.firmware_grav_comp = true;
    FourChannelController ctrl_fw(dyn, cp_fw);
    auto tau_fw_match = ctrl_fw.compute(q, qd_zero, tau_ext_zero,
                                        q, qd_zero, tau_ext_zero, 1.0);
    std::printf("4CH(fw_grav) @ matched: τ  = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", tau_fw_match(i));
    std::printf("(expect ≈ 0)\n");

    // (E) firmware_grav_comp=true: DOB at rest, τ_applied=0 → τ̂_ext ≈ 0
    DisturbanceObserver::Params dp_fw = dp;
    dp_fw.firmware_grav_comp = true;
    DisturbanceObserver dob_fw(dyn, dp_fw, 0.002);
    for (int i = 0; i < 2000; ++i) dob_fw.update(q, qd_zero, qd_zero);
    std::printf("DOB(fw_grav) @ tau=0:   τ̂_ext = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", dob_fw.value()(i));
    std::printf("(expect ≈ 0 — fw provides g)\n");

    // (F) cancel_gain=0: τ̂_ext should NOT enter the controller output
    FourChannelController::Params cp_no_cancel = cp_fw;
    cp_no_cancel.tau_ext_cancel_gain = 0.0;
    FourChannelController ctrl_no_cancel(dyn, cp_no_cancel);
    Eigen::VectorXd tau_ext_inj(6);
    tau_ext_inj << 0, 0, 0, 5.0, 0, 0;  // pretend DOB sees +5 Nm on j3
    auto tau_no_cancel = ctrl_no_cancel.compute(q, qd_zero, tau_ext_inj,
                                                q, qd_zero, tau_ext_zero, 1.0);
    std::printf("4CH(cancel=0) @ τ̂=+5j3: τ  = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", tau_no_cancel(i));
    std::printf("(expect ≈ 0 — DOB ignored)\n");

    // (G) cancel_gain=1: same scenario, expect −5 on j3
    FourChannelController::Params cp_full = cp_fw;
    cp_full.tau_ext_cancel_gain = 1.0;
    FourChannelController ctrl_full(dyn, cp_full);
    auto tau_full = ctrl_full.compute(q, qd_zero, tau_ext_inj,
                                      q, qd_zero, tau_ext_zero, 1.0);
    std::printf("4CH(cancel=1) @ τ̂=+5j3: τ  = ");
    for (int i = 0; i < 6; ++i) std::printf("%+.4f ", tau_full(i));
    std::printf("(expect ≈ -5 on j3)\n");

    // ---- EnergyTank sanity ----
    EnergyTank::Params tp;
    tp.E_max = 5.0;
    tp.E_init = 2.5;
    tp.refill_ceiling = 0.9;
    tp.D_dissipation = Eigen::VectorXd::Constant(6, 1.0);
    EnergyTank tank(tp, 0.002);

    auto fmt_state = [&tank](const char* tag) {
      std::printf("%-42s E=%.3fJ  α=%.3f  P_act=%+.3fW  refill=%+.4fJ\n",
                  tag, tank.energy(), tank.last_alpha(),
                  tank.last_P_active(), tank.last_refill());
    };

    std::printf("\nTank init:                                 ");
    std::printf("E=%.3fJ (expect 2.5)\n", tank.energy());

    // (1) No motion, no tau_active: no change, α=1
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
    tank.step(zero, zero);
    fmt_state("Tank step(0,0):");

    // (2) Active 5W vs D-refill 1W (from v₀=1) → net drain 4W, 500ms → -2J
    // Start from ceiling so drain doesn't clip at 0.
    tank.reset(4.5);
    Eigen::VectorXd t_act(6); t_act << 5.0, 0, 0, 0, 0, 0;
    Eigen::VectorXd v(6);     v     << 1.0, 0, 0, 0, 0, 0;   // P_diss=1W
    for (int i = 0; i < 250; ++i) tank.step(t_act, v);
    fmt_state("Tank 5W-drain 1W-refill, 0.5s from 4.5J:");

    // (3) Sustained drain past empty → α clips to 0
    for (int i = 0; i < 2000; ++i) tank.step(t_act, v);
    fmt_state("Tank drain sustained 4s:");

    // (4) Regenerative power (τ·q̇ < 0) with refill motion → E grows
    tank.reset(0.0);
    Eigen::VectorXd t_regen(6); t_regen << -1.0, 0, 0, 0, 0, 0;
    for (int i = 0; i < 500; ++i) tank.step(t_regen, v);
    fmt_state("Tank regen -1W for 1s (from 0):");

    // (5) Damping refill only: q̇=[1,…], D=[1,…] → P_diss = 6 W refill
    tank.reset(0.0);
    Eigen::VectorXd v_all = Eigen::VectorXd::Ones(6);
    for (int i = 0; i < 500; ++i) tank.step(zero, v_all);  // 1s of refill
    fmt_state("Tank refill D·q̇² for 1s (from 0):");
    std::printf("   ceiling = %.3fJ, E_max = %.3fJ\n",
                tp.refill_ceiling * tp.E_max, tp.E_max);

    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "selftest failed: %s\n", e.what());
    return 2;
  }
}
