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

// Trapezoidal position profile s(a), a in [0,1]: 20% accel / 60% cruise /
// 20% decel. Replaced the earlier quintic for MOVES — quintic's long
// near-zero-velocity tails sit right in the stiction band and caused
// stick-slip "judder".
double trapezoid(double a) {
  a = std::clamp(a, 0.0, 1.0);
  const double ta = 0.2;                  // accel fraction
  const double vmax = 1.0 / (1.0 - ta);   // unit-area cruise velocity
  if (a < ta) return 0.5 * vmax * a * a / ta;
  if (a < 1.0 - ta) return 0.5 * vmax * ta + vmax * (a - ta);
  double r = 1.0 - a;
  return 1.0 - 0.5 * vmax * r * r / ta;
}

// Calibration poses (LEFT arm conventions; j1/j2 sign-flipped for right).
// Chosen to excite j1/j2/j3/j4 gravity terms over their (conservative) ranges.
// NOTE: j4 >= 0.3 everywhere — j4's range is [0,1.8] and q4=0 is the elbow
// HARD STOP: resting on it makes the measured torque contact-contaminated.
const std::vector<Vec7> POSES_LEFT = {
    {0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0},
    {0.5, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0},
    {-0.6, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0},
    {-1.2, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0},
    {-1.6, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0},
    {-2.2, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0},
    {0.0, -0.6, 0.0, 0.3, 0.0, 0.0, 0.0},
    {0.0, -1.2, 0.0, 0.3, 0.0, 0.0, 0.0},
    {0.0, -1.6, 0.0, 0.3, 0.0, 0.0, 0.0},
    {0.0, -2.2, 0.0, 0.3, 0.0, 0.0, 0.0},
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

// Joint limits (left); used to clamp the two-sided approach offsets.
const double LIM_LO_L[DOF] = {-3.34, -3.27, -1.57, 0.0, -1.5, -0.75, -1.4};
const double LIM_HI_L[DOF] = {0.91, 0.13, 1.57, 1.8, 1.5, 0.75, 1.4};

}  // namespace

int main(int argc, char** argv) {
  std::string can = "can1", urdf, out = "/tmp/gravity_cali.csv", side = "left";
  std::string poses_file, config_path;
  double gz = 9.81, dwell = 2.0;
  bool fd = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--can") can = next();
    else if (a == "--urdf") urdf = next();
    else if (a == "--out") out = next();
    else if (a == "--side") side = next();
    else if (a == "--poses") poses_file = next();   // gen_cali_poses.py output
    else if (a == "--gz") gz = std::stod(next());
    else if (a == "--dwell") dwell = std::stod(next());
    else if (a == "--config") config_path = next();   // oa_fd.yaml (friction FF)
    else if (a == "--fd") fd = true;
  }
  if (urdf.empty()) {
    std::fprintf(stderr, "need --urdf <model.urdf>\n");
    return 1;
  }

  // friction FF during MOVES (the identified yaml values): cancels joint
  // friction so the impedance doesn't have to drag through it (stick-slip,
  // "struggling" moves). Excluded during the static measurement windows —
  // velocity noise through tanh would contaminate them (and v=0 anyway).
  ArmCfg fcfg;       // friction block only; zeros if no --config
  GlobalCfg gdummy;
  if (!config_path.empty()) {
    if (load_arm_config(config_path, fcfg, gdummy))
      std::printf("friction FF from %s (Fc1=%.2f ...)\n",
                  config_path.c_str(), fcfg.fric_Fc[0]);
    else
      std::fprintf(stderr, "config load failed; friction FF = 0\n");
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

  // poses: built-in list, or a gen_cali_poses.py file (one pose per line,
  // 7 comma-separated radians, '#' comments). Always LEFT-arm conventions.
  std::vector<Vec7> poses = POSES_LEFT;
  if (!poses_file.empty()) {
    poses.clear();
    std::ifstream pf(poses_file);
    if (!pf.good()) {
      std::fprintf(stderr, "cannot read --poses %s\n", poses_file.c_str());
      return 1;
    }
    std::string line;
    while (std::getline(pf, line)) {
      if (line.empty() || line[0] == '#') continue;
      Vec7 p{};
      if (std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                      &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6]) == 7)
        poses.push_back(p);
    }
    std::printf("loaded %zu poses from %s\n", poses.size(), poses_file.c_str());
  }
  // Right arm = y-mirror of left: joints q1,q2,q3,q5,q7 flip, q4,q6 same
  // (matches gravity.mirror_right; verified on HW that q1/q3/q5/q7 mirror).
  // NOTE: the old code mirrored only j1/j2 — wrong map. Now: a --poses file
  // is taken AS-IS (gen_cali_poses --side right already wrote right-frame
  // values); only the built-in POSES_LEFT is mirrored. Limits are mirrored
  // per-joint for the clamp either way.
  static const double MIR[DOF] = {-1, -1, -1, 1, -1, 1, -1};
  double lim_lo[DOF], lim_hi[DOF];
  for (int i = 0; i < DOF; ++i) { lim_lo[i] = LIM_LO_L[i]; lim_hi[i] = LIM_HI_L[i]; }
  if (side == "right") {
    if (poses_file.empty())                       // mirror only the built-in set
      for (auto& p : poses)
        for (int i = 0; i < DOF; ++i) p[i] *= MIR[i];
    for (int i = 0; i < DOF; ++i)
      if (MIR[i] < 0) { lim_lo[i] = -LIM_HI_L[i]; lim_hi[i] = -LIM_LO_L[i]; }
  }

  // Stiffer than v1: Kp=60 stick-slipped against joint friction (arm lagged
  // the trajectory, error built up, then jerked forward in bursts).
  const Vec7 KP = {110, 110, 100, 100, 16, 16, 12};
  // KD matched to oa_friction_cali (3.5/3.0/0.4): the lower 2.0/1.8 here left
  // the pose->pose moves underdamped -> overshoot/"popping". Higher damping
  // (+ slower big move below) makes gravity-cali moves as smooth as friction.
  const Vec7 KD = {3.5, 3.5, 3.0, 3.0, 0.4, 0.4, 0.3};
  const Vec7 TAU_FF_MAX = {40, 40, 25, 25, 8, 8, 8};

  // SAFETY pre-flight: refuse to run if any pose is outside the mechanical
  // range (a bad pose would make the motor fault and drop torque -> limp arm).
  // Done BEFORE enabling the motors so nothing moves on a bad config.
  int bad = 0;
  for (size_t pi = 0; pi < poses.size(); ++pi)
    for (int i = 0; i < DOF; ++i)
      if (poses[pi][i] < lim_lo[i] - 1e-6 || poses[pi][i] > lim_hi[i] + 1e-6) {
        std::fprintf(stderr, "[oa_gravity_cali] pose %zu j%d = %.3f OUT OF RANGE "
                     "[%.3f,%.3f]\n", pi, i + 1, poses[pi][i], lim_lo[i], lim_hi[i]);
        ++bad;
      }
  if (bad) {
    std::fprintf(stderr, "REFUSING to run: %d out-of-range pose value(s). "
                 "Check --poses / --side.\n", bad);
    return 1;
  }

  OaxArm arm(can, fd, 500);
  if (!arm.init() || !arm.enable()) {
    std::fprintf(stderr, "arm init/enable failed on %s\n", can.c_str());
    return 1;
  }
  arm.set_pos_limits({lim_lo[0],lim_lo[1],lim_lo[2],lim_lo[3],lim_lo[4],lim_lo[5],lim_lo[6]},
                     {lim_hi[0],lim_hi[1],lim_hi[2],lim_hi[3],lim_hi[4],lim_hi[5],lim_hi[6]});

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

  bool friction_ff_on = true;   // moves: ON, static measurement windows: OFF
  auto hold_cmd = [&](const Vec7& target) {
    Vec7 g{};
    arm.read(q, dq, tau);
    grav.gravity(q, g);
    for (int i = 0; i < DOF; ++i) {
      if (friction_ff_on) {
        const double boost = 1.25;   // yaml has x0.75 safety; ~full during moves
        g[i] += boost * (fcfg.fric_Fc[i] * std::tanh(fcfg.fric_k[i] * dq[i])
              + fcfg.fric_Fv[i] * dq[i] + fcfg.fric_Fo[i]);
      }
      g[i] = std::clamp(g[i], -TAU_FF_MAX[i], TAU_FF_MAX[i]);
    }
    Vec7 zero{};   // velocity ref 0: Kd is pure damping (no FF kick -> no lurch)
    arm.write_mit(KP, KD, target, zero, g);
  };

  std::printf("oa_gravity_cali: %zu poses on %s (%s arm), dwell %.1fs\n",
              poses.size(), can.c_str(), side.c_str(), dwell);

  auto move_to = [&](const Vec7& a, const Vec7& bgt, double T) {
    int nmove = static_cast<int>(T / dt);
    for (int s = 0; s < nmove; ++s) {
      double al = trapezoid(static_cast<double>(s) / nmove);
      Vec7 ref;
      for (int i = 0; i < DOF; ++i) ref[i] = a[i] + al * (bgt[i] - a[i]);
      hold_cmd(ref);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
  };
  auto settle_and_measure = [&](const Vec7& target, Vec7& qa, Vec7& ta) {
    friction_ff_on = false;   // static window: no FF noise via tanh(v_noise)
    int nsettle = static_cast<int>(dwell / dt);
    for (int s = 0; s < nsettle; ++s) {
      hold_cmd(target);
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
    qa.fill(0); ta.fill(0);
    int nmeas = static_cast<int>(1.0 / dt);
    for (int s = 0; s < nmeas; ++s) {
      hold_cmd(target);
      for (int i = 0; i < DOF; ++i) { qa[i] += q[i]; ta[i] += tau[i]; }
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }
    for (int i = 0; i < DOF; ++i) { qa[i] /= nmeas; ta[i] /= nmeas; }
    friction_ff_on = true;
  };

  for (size_t pi = 0; pi <= poses.size(); ++pi) {
    // last "pose" = return near-hang (elbow off its stop)
    Vec7 target{};
    target[3] = 0.3;
    if (pi < poses.size()) target = poses[pi];

    move_to(from, target, 6.0);   // slower big reconfiguration -> smoother
    from = target;
    if (pi == poses.size()) break;  // returned home — done

    // STICTION DE-BIAS: approach the pose from BOTH sides and average.
    // Friction holds part of gravity depending on approach direction; the
    // two-sided mean cancels the first-order stiction bias.
    const double DELTA = 0.12;
    Vec7 above = target, below = target;
    for (int i = 0; i < 4; ++i) {   // offset only the gravity-loaded joints
      above[i] = std::clamp(target[i] + DELTA, lim_lo[i] + 0.1, lim_hi[i] - 0.1);
      below[i] = std::clamp(target[i] - DELTA, lim_lo[i] + 0.1, lim_hi[i] - 0.1);
    }
    Vec7 qa1{}, ta1{}, qa2{}, ta2{};
    move_to(target, above, 1.0);
    move_to(above, target, 1.5);
    settle_and_measure(target, qa1, ta1);
    move_to(target, below, 1.0);
    move_to(below, target, 1.5);
    settle_and_measure(target, qa2, ta2);

    Vec7 qa{}, ta{}, gm{};
    for (int i = 0; i < DOF; ++i) {
      qa[i] = 0.5 * (qa1[i] + qa2[i]);
      ta[i] = 0.5 * (ta1[i] + ta2[i]);
    }
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
