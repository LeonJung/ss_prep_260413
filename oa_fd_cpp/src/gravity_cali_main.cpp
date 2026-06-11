// gravity_cali_main.cpp — oa_gravity_cali: static gravity identification data
// collector for ONE arm.
//
// Moves the arm through a grid of poses (motor-side MIT impedance + current
// model gravity FF), holds each pose, and records the MEASURED motor torques.
// At static equilibrium the measured joint torque equals the TRUE gravity
// torque g_real(q) — independent of the (imperfect) model used to get there.
// Output CSV feeds script/fit_gravity.py, which solves per-link mass/COM
// corrections by least squares (gravity is linear in mass & mass*COM).
//
// Usage:
//   oa_gravity_cali --can can1 --urdf <model.urdf> --out /tmp/gravity_cali.csv
//                   [--side left|right] [--gz 9.81] [--dwell 2.0] [--fd]
//
// Safety: modest impedance, quintic interpolation, torque-FF clamped, starts
// from the CURRENT pose, returns to hang (q=0) at the end, then disables.
// Keep the e-stop in reach anyway.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "oa_fd_cpp/config.hpp"
#include "oa_fd_cpp/gravity.hpp"
#include "oa_fd_cpp/oax_driver.hpp"

using namespace oa_fd;

namespace {

double quintic(double a) {
  a = std::clamp(a, 0.0, 1.0);
  return 10 * a * a * a - 15 * a * a * a * a + 6 * a * a * a * a * a;
}

// Calibration poses (LEFT arm conventions; j1/j2 sign-flipped for right).
// Chosen to excite j1/j2/j3/j4 gravity terms over their (conservative) ranges.
const std::vector<Vec7> POSES_LEFT = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-2.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, -2.2, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0},
    {-0.8, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0},
    {-1.6, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0},
    {0.0, -1.2, 0.0, 0.8, 0.0, 0.0, 0.0},
    {0.0, -1.6, 0.0, 1.2, 0.0, 0.0, 0.0},
    {-0.8, -0.8, 0.0, 0.8, 0.0, 0.0, 0.0},
    {0.5, -0.8, 0.0, 0.8, 0.0, 0.0, 0.0},
    {-0.8, -0.8, 0.8, 0.8, 0.0, 0.0, 0.0},
    {-0.8, -0.8, -0.8, 0.8, 0.0, 0.0, 0.0},
};

}  // namespace

int main(int argc, char** argv) {
  std::string can = "can1", urdf, out = "/tmp/gravity_cali.csv", side = "left";
  double gz = 9.81, dwell = 2.0;
  bool fd = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--can") can = next();
    else if (a == "--urdf") urdf = next();
    else if (a == "--out") out = next();
    else if (a == "--side") side = next();
    else if (a == "--gz") gz = std::stod(next());
    else if (a == "--dwell") dwell = std::stod(next());
    else if (a == "--fd") fd = true;
  }
  if (urdf.empty()) {
    std::fprintf(stderr, "need --urdf <model.urdf>\n");
    return 1;
  }

  // gravity model (used only as feedforward DURING moves; measurement is raw)
  GravityCfg gc;
  gc.enabled = true;
  gc.urdf = urdf;
  gc.root_link = "openarmx_link0";
  gc.tip_link = "openarmx_link7";
  gc.vec = {0.0, 0.0, gz};
  GravityModel grav;
  if (!grav.load(gc)) {
    std::fprintf(stderr, "gravity model load failed (continuing: FF=0)\n");
  }

  // poses (mirror j1/j2 for the right side, matching mirrored joint ranges)
  std::vector<Vec7> poses = POSES_LEFT;
  if (side == "right")
    for (auto& p : poses) { p[0] = -p[0]; p[1] = -p[1]; }

  const Vec7 KP = {60, 60, 60, 60, 10, 10, 8};
  const Vec7 KD = {2, 2, 2, 2, 0.2, 0.2, 0.2};
  const Vec7 TAU_FF_MAX = {40, 40, 25, 25, 8, 8, 8};

  OaxArm arm(can, fd, 500);
  if (!arm.init() || !arm.enable()) {
    std::fprintf(stderr, "arm init/enable failed on %s\n", can.c_str());
    return 1;
  }

  std::ofstream csv(out);
  csv << "pose";
  for (int k = 1; k <= DOF; ++k) csv << ",q" << k;
  for (int k = 1; k <= DOF; ++k) csv << ",tau_meas" << k;
  for (int k = 1; k <= DOF; ++k) csv << ",tau_model" << k;
  csv << "\n";

  const double dt = 0.004;  // 250 Hz
  Vec7 q{}, dq{}, tau{};
  arm.read(q, dq, tau);
  Vec7 from = q;

  auto hold_cmd = [&](const Vec7& target) {
    Vec7 g{};
    arm.read(q, dq, tau);
    grav.gravity(q, g);
    for (int i = 0; i < DOF; ++i)
      g[i] = std::clamp(g[i], -TAU_FF_MAX[i], TAU_FF_MAX[i]);
    Vec7 zero{};
    arm.write_mit(KP, KD, target, zero, g);
  };

  std::printf("oa_gravity_cali: %zu poses on %s (%s arm), dwell %.1fs\n",
              poses.size(), can.c_str(), side.c_str(), dwell);

  for (size_t pi = 0; pi <= poses.size(); ++pi) {
    // last "pose" = return to hang q=0
    Vec7 target{};
    if (pi < poses.size()) target = poses[pi];

    // quintic move 4 s
    const double T = 4.0;
    int nmove = static_cast<int>(T / dt);
    for (int s = 0; s < nmove; ++s) {
      double a = quintic(static_cast<double>(s) / nmove);
      Vec7 ref;
      for (int i = 0; i < DOF; ++i) ref[i] = from[i] + a * (target[i] - from[i]);
      hold_cmd(ref);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
    from = target;
    if (pi == poses.size()) break;  // returned home — done

    // settle
    int nsettle = static_cast<int>(dwell / dt);
    for (int s = 0; s < nsettle; ++s) {
      hold_cmd(target);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }

    // measure: 1 s average of q and measured torque
    Vec7 qa{}, ta{};
    int nmeas = static_cast<int>(1.0 / dt);
    for (int s = 0; s < nmeas; ++s) {
      hold_cmd(target);
      for (int i = 0; i < DOF; ++i) { qa[i] += q[i]; ta[i] += tau[i]; }
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
    Vec7 gm{};
    for (int i = 0; i < DOF; ++i) { qa[i] /= nmeas; ta[i] /= nmeas; }
    grav.gravity(qa, gm);

    csv << pi;
    for (int i = 0; i < DOF; ++i) csv << ',' << qa[i];
    for (int i = 0; i < DOF; ++i) csv << ',' << ta[i];
    for (int i = 0; i < DOF; ++i) csv << ',' << gm[i];
    csv << "\n";
    csv.flush();
    std::printf("  pose %2zu/%zu  q=[%5.2f %5.2f %5.2f %5.2f]  "
                "tau_meas=[%6.2f %6.2f %6.2f %6.2f]  model=[%6.2f %6.2f %6.2f %6.2f]\n",
                pi + 1, poses.size(), qa[0], qa[1], qa[2], qa[3],
                ta[0], ta[1], ta[2], ta[3], gm[0], gm[1], gm[2], gm[3]);
  }

  arm.disable();
  std::printf("done -> %s\n", out.c_str());
  return 0;
}
