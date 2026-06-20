// friction_cali_main.cpp — oa_friction_cali: per-joint friction identification.
//
// One joint at a time: all other joints held at a safe hang-ish pose, the
// target joint sweeps back and forth at several CONSTANT velocities. During
// the constant-velocity mid-section,  tau_meas = g(q) + friction(qd)
// (inertial terms ~0 at constant speed), so
//     friction(qd) = tau_meas - g_model(q)
// Samples are logged continuously; script/fit_friction.py fits the per-joint
// tanh model  f(v) = Fc*tanh(k v) + Fv*v + Fo  for oa_fd.yaml.
//
// Usage:
//   oa_friction_cali --can can1 --urdf <model.urdf> --out /tmp/friction.csv
//                    [--side left|right] [--joint N (1..7, default all)]
//                    [--gz 9.81] [--fd]
//
// Safety: single-joint motion from a hang-ish base pose within conservative
// sub-ranges; impedance + gravity FF; e-stop in reach.

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

// Base = arm STRAIGHT (q4=0, elbow fully extended): a bent elbow (q4=0.3)
// swings the forearm forward so q3/q6 sweeps hit the torso; straight, the
// distal sweeps rotate ~in place and stay clear. (q4's OWN sweep still starts
// >=0.2 to keep that measurement off the q4=0 hard stop — see emit-sweep.)
const Vec7 BASE_LEFT = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const double SWEEP_LO_L[DOF] = {-1.8, -2.0, -1.2, 0.20, -1.2, -0.55, -1.2};
const double SWEEP_HI_L[DOF] = {0.55, 0.00, 1.2, 1.50, 1.2, 0.55, 1.2};
const double SPEEDS[] = {0.2, 0.5, 0.9};   // rad/s, each run both directions

}  // namespace

int main(int argc, char** argv) {
  std::string can = "can1", urdf, out = "/tmp/friction_cali.csv", side = "left";
  std::string sweep_file;
  int only_joint = 0;  // 0 = all
  double gz = 9.81;
  bool fd = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--can") can = next();
    else if (a == "--urdf") urdf = next();
    else if (a == "--out") out = next();
    else if (a == "--side") side = next();
    else if (a == "--joint") only_joint = std::stoi(next());
    else if (a == "--gz") gz = std::stod(next());
    else if (a == "--sweep-file") sweep_file = next();  // per-joint "lo hi" (collision-safe, gen_cali_poses --emit-sweep). AS-IS, no mirror.
    else if (a == "--fd") fd = true;
  }
  if (urdf.empty()) { std::fprintf(stderr, "need --urdf\n"); return 1; }

  GravityCfg gc;
  gc.enabled = true;
  gc.urdf = urdf;
  gc.root_link = "openarmx_link0";
  gc.tip_link = "openarmx_link7";
  gc.vec = {0.0, 0.0, gz};
  GravityModel grav;
  if (!grav.load(gc)) { std::fprintf(stderr, "gravity model load failed\n"); return 1; }

  // Right arm = y-mirror of left, full map (q1,q2,q3,q5,q7 flip; q4,q6 same) —
  // same m as gravity.mirror_right / cali_poses_right. The old code mirrored
  // only j1/j2, which sent the sweeps to the WRONG (inner/torso) side.
  static const double MIR[DOF] = {-1, -1, -1, 1, -1, 1, -1};
  Vec7 base = BASE_LEFT;
  double lo[DOF], hi[DOF];
  for (int i = 0; i < DOF; ++i) { lo[i] = SWEEP_LO_L[i]; hi[i] = SWEEP_HI_L[i]; }
  if (side == "right") {
    for (int i = 0; i < DOF; ++i) {
      base[i] *= MIR[i];
      if (MIR[i] < 0) { double l = lo[i], h = hi[i]; lo[i] = -h; hi[i] = -l; }
    }
  }
  // Collision-safe sweep ranges (gen_cali_poses --emit-sweep): per-joint
  // "lo hi", already in the target arm's frame -> taken AS-IS (overrides the
  // built-in ranges, no extra mirror). Needed for the FOLLOWER's gripper,
  // whose longer reach hits the torso on the wide q3/q6 sweeps.
  if (!sweep_file.empty()) {
    std::ifstream sf(sweep_file);
    if (!sf.good()) { std::fprintf(stderr, "cannot read --sweep-file %s\n", sweep_file.c_str()); return 1; }
    std::string line; int j = 0;
    while (std::getline(sf, line) && j < DOF) {
      if (line.empty() || line[0] == '#') continue;
      double l, h;
      if (std::sscanf(line.c_str(), "%lf %lf", &l, &h) == 2 ||
          std::sscanf(line.c_str(), "%lf,%lf", &l, &h) == 2) { lo[j] = l; hi[j] = h; ++j; }
    }
    std::printf("loaded %d collision-safe sweep ranges from %s\n", j, sweep_file.c_str());
  }

  const Vec7 KP = {110, 110, 100, 100, 16, 16, 12};
  const Vec7 KD = {3.5, 3.5, 3.0, 3.0, 0.4, 0.4, 0.3};
  const Vec7 TAU_FF_MAX = {40, 40, 25, 25, 8, 8, 8};

  OaxArm arm(can, fd, 500);
  if (!arm.init() || !arm.enable()) {
    std::fprintf(stderr, "arm init/enable failed on %s\n", can.c_str());
    return 1;
  }
  // SAFETY clamp = MECHANICAL limits (NOT the sweep ranges). The straight base
  // holds q4=0, which is below the q4 sweep lower (0.2) — clamping to the sweep
  // range would force q4 up to 0.2 and bend the arm. Mechanical bounds (q4
  // [0,1.8]) allow the straight base while still blocking impossible targets.
  const double ML_L[DOF] = {-3.34, -3.27, -1.57, 0.0, -1.5, -0.75, -1.4};
  const double MH_L[DOF] = { 0.91,  0.13,  1.57, 1.8,  1.5,  0.75,  1.4};
  Vec7 mlo, mhi;
  for (int i = 0; i < DOF; ++i) {
    if (side == "right" && MIR[i] < 0) { mlo[i] = -MH_L[i]; mhi[i] = -ML_L[i]; }
    else { mlo[i] = ML_L[i]; mhi[i] = MH_L[i]; }
  }
  arm.set_pos_limits(mlo, mhi);

  std::ofstream csv(out);
  csv << "joint,q,dq,tau_meas,tau_gravity_model\n";

  const double dt = 0.004;
  Vec7 q{}, dq{}, tau{};
  arm.read(q, dq, tau);

  // --- TEMP loop-rate diagnostic: cmd() is the single command point for every
  // segment (moves + sweeps), so this measures the actual control-loop period.
  // ~250 Hz steady + ~4-5 ms max gap = healthy. Low Hz / big gaps = the loop is
  // stuttering (non-RT sleep_for or slow CAN read) -> that's the "팡팡" cause.
  auto t_prev = std::chrono::steady_clock::now();
  double win_sum = 0.0, win_max = 0.0; long win_cnt = 0; bool first_tick = true;
  auto cmd = [&](const Vec7& ref) {
    auto tnow = std::chrono::steady_clock::now();
    double d = std::chrono::duration<double>(tnow - t_prev).count();
    t_prev = tnow;
    if (!first_tick) {
      win_sum += d; if (d > win_max) win_max = d;
      if (++win_cnt == 500) {
        std::printf("[loop] %.0f Hz avg, max gap %.1f ms (last 500 cmds)\n",
                    500.0 / win_sum, win_max * 1000.0);
        win_sum = 0.0; win_max = 0.0; win_cnt = 0;
      }
    }
    first_tick = false;
    Vec7 g{};
    arm.read(q, dq, tau);
    grav.gravity(q, g);
    for (int i = 0; i < DOF; ++i)
      g[i] = std::clamp(g[i], -TAU_FF_MAX[i], TAU_FF_MAX[i]);
    Vec7 zero{};
    arm.write_mit(KP, KD, ref, zero, g);
  };
  auto move_quintic = [&](const Vec7& a, const Vec7& b, double T) {
    int n = static_cast<int>(T / dt);
    for (int s = 0; s < n; ++s) {
      double al = quintic(static_cast<double>(s) / n);
      Vec7 ref;
      for (int i = 0; i < DOF; ++i) ref[i] = a[i] + al * (b[i] - a[i]);
      cmd(ref);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
  };
  // go to base
  Vec7 cur = q;
  move_quintic(cur, base, 4.0);

  for (int j = 0; j < DOF; ++j) {
    if (only_joint > 0 && j != only_joint - 1) continue;
    std::printf("== joint %d: sweeps [%.2f, %.2f] ==\n", j + 1, lo[j], hi[j]);

    for (double sp : SPEEDS) {
      // shrink range for slow passes so each pass stays < ~10 s
      double span = hi[j] - lo[j];
      double use = std::min(span, std::max(0.8, sp * 6.0));
      double mid = 0.5 * (lo[j] + hi[j]);
      double a = mid - use / 2, b = mid + use / 2;

      // pre-position to a
      Vec7 p0 = base; p0[j] = a;
      Vec7 here = base; here[j] = q[j];
      move_quintic(here, p0, 2.5);

      // BOTH directions (a->b then b->a): +v/-v averaging cancels symmetric
      // friction -> pure gravity torque (needed for the leader/follower mass &
      // output-ratio measurement). Original linear-ramp sweep via cmd() (the
      // smooth-era motion, also loop-rate instrumented).
      for (int dir = 0; dir < 2; ++dir) {
        double from = dir == 0 ? a : b, to = dir == 0 ? b : a;
        double Tpass = std::abs(to - from) / sp;
        int n = static_cast<int>(Tpass / dt);
        for (int s = 0; s < n; ++s) {
          Vec7 ref = base;
          ref[j] = from + (to - from) * (static_cast<double>(s) / n);
          cmd(ref);
          if (s > n * 0.15 && s < n * 0.85) {
            Vec7 g{};
            grav.gravity(q, g);
            csv << (j + 1) << ',' << q[j] << ',' << dq[j] << ','
                << tau[j] << ',' << g[j] << "\n";
          }
          std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }
        std::printf("  v=%.2f dir=%s done\n", sp, dir == 0 ? "+" : "-");
      }
      csv.flush();
      // back to base joint value
      Vec7 pb = base; pb[j] = a;
      Vec7 pc = base; pc[j] = q[j];
      move_quintic(pc, base, 2.0);
    }
  }

  // return to base and disable
  Vec7 pc = base;
  arm.read(q, dq, tau);
  for (int i = 0; i < DOF; ++i) pc[i] = q[i];
  move_quintic(pc, base, 3.0);
  arm.disable();
  std::printf("done -> %s\n", out.c_str());
  return 0;
}
