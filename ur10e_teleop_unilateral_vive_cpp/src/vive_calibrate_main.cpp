// vive_calibrate_main.cpp — interactive 3-point calibration utility.
//
// Captures the tracker pose at N (default 3) UR-base-frame positions
// supplied by the operator, then solves the rigid transform
//   T_ur_from_tracker  s.t.  ur_pos ≈ T · tracker_pos
// via Umeyama / Kabsch (already implemented in Calibration) and writes
// the 4×4 to YAML.
//
// Typical workflow (operator holding tracker by hand):
//   1. Identify 3 distinct points in the UR's base frame whose XYZ you
//      know — corners of a fixture, marked spots on the table, or
//      end-effector positions from the follower's teach pendant.
//   2. Run this utility with --serial and --out paths.
//   3. For each point, place the tracker at that physical location,
//      hold steady, and press Enter. The utility averages the pose
//      over ~0.5 s to reject jitter.
//   4. The YAML is written; pass it to the leader via --left-calib /
//      --right-calib (or set it in the launch file).

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "ur10e_teleop_unilateral_vive_cpp/calibration.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/config.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/ur_jacobian.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/vive_tracker.hpp"

using ur10e_teleop_unilateral_vive_cpp::Calibration;
using ur10e_teleop_unilateral_vive_cpp::ViveTracker;

namespace {

void print_usage(const char* prog) {
  std::fprintf(stderr,
    "Usage: %s --serial LHR-XXXX --out path/to/calibration.yaml\n"
    "         [--home-mode] [--config PATH] [--robot ur3e|ur10e|ur5e]\n"
    "         [--side left|right]  [--displacement 0.3]\n"
    "         [--points 'x1,y1,z1 x2,y2,z2 x3,y3,z3 ...']\n"
    "         [--avg-ms 500]\n"
    "\n"
    "Two modes:\n"
    "  (a) --home-mode  — operator-friendly. Targets are auto-generated\n"
    "      from the UR home pose (origin = home EE) plus orthogonal\n"
    "      displacements of `--displacement` m. Operator places the\n"
    "      tracker at their neutral pose, then displaces in +X/+Y/+Z\n"
    "      of their own intuitive frame. Requires --config / --robot /\n"
    "      --side so the home_qpos and FK target match the arm being\n"
    "      calibrated.\n"
    "\n"
    "  (b) --points 'x1,y1,z1 ...'  — explicit. Operator supplies the\n"
    "      UR-frame XYZ of each calibration point (e.g. read off the\n"
    "      teach pendant's TCP-position display).\n"
    "\n"
    "Default --home-mode displacements (UR base frame, meters):\n"
    "   home_EE + ( 0.3,  0.0,  0.0 )   ('forward' from operator view)\n"
    "   home_EE + ( 0.0,  0.3,  0.0 )   ('left')\n"
    "   home_EE + ( 0.0,  0.0,  0.3 )   ('up')\n"
    "(--displacement scales these.)\n",
    prog);
}

bool parse_point(const std::string& s, Eigen::Vector3d& out) {
  // "x,y,z"
  double x, y, z;
  if (std::sscanf(s.c_str(), "%lf,%lf,%lf", &x, &y, &z) == 3) {
    out << x, y, z;
    return true;
  }
  return false;
}

std::vector<Eigen::Vector3d> parse_points_arg(const std::string& arg) {
  std::vector<Eigen::Vector3d> pts;
  std::string token;
  for (char c : arg) {
    if (c == ' ') {
      if (!token.empty()) {
        Eigen::Vector3d p;
        if (parse_point(token, p)) pts.push_back(p);
        token.clear();
      }
    } else {
      token += c;
    }
  }
  if (!token.empty()) {
    Eigen::Vector3d p;
    if (parse_point(token, p)) pts.push_back(p);
  }
  return pts;
}

// Average tracker pose over duration_ms. Returns translation only —
// orientation isn't used for the Umeyama 3-point solve.
bool average_pose(ViveTracker& tracker, double duration_ms,
                  Eigen::Vector3d& out_pos) {
  using clk = std::chrono::steady_clock;
  const auto t_end = clk::now()
      + std::chrono::milliseconds(static_cast<int>(duration_ms));
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int n = 0;
  while (clk::now() < t_end) {
    Eigen::Matrix4d T;
    if (tracker.poll(T)) {
      sum += T.block<3, 1>(0, 3);
      ++n;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (n < 5) return false;  // tracker mostly lost during averaging window
  out_pos = sum / static_cast<double>(n);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::string serial;
  std::string out_path;
  std::string points_arg;
  std::string config_path;
  std::string robot_type = "ur3e";
  std::string side;             // "left" | "right" — selects which home_qpos
  bool home_mode = false;
  double displacement = 0.3;
  double avg_ms = 500.0;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* n) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", n);
        std::exit(2);
      }
      return argv[++i];
    };
    if      (a == "--serial")        serial = need("--serial");
    else if (a == "--out")           out_path = need("--out");
    else if (a == "--points")        points_arg = need("--points");
    else if (a == "--avg-ms")        avg_ms = std::stod(need("--avg-ms"));
    else if (a == "--home-mode")     home_mode = true;
    else if (a == "--config")        config_path = need("--config");
    else if (a == "--robot")         robot_type = need("--robot");
    else if (a == "--side")          side = need("--side");
    else if (a == "--displacement")  displacement = std::stod(need("--displacement"));
    else if (a == "--help" || a == "-h") { print_usage(argv[0]); return 0; }
    else {
      std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
      print_usage(argv[0]);
      return 2;
    }
  }
  if (serial.empty() || out_path.empty()) {
    print_usage(argv[0]);
    return 2;
  }

  std::vector<Eigen::Vector3d> ur_points;
  std::vector<std::string> ur_labels;        // for operator-friendly prompts

  if (home_mode) {
    using namespace ur10e_teleop_unilateral_vive_cpp;
    if (config_path.empty()) {
      std::fprintf(stderr,
          "--home-mode needs --config PATH (to read home_qpos)\n");
      return 2;
    }
    if (side != "left" && side != "right") {
      std::fprintf(stderr, "--side must be 'left' or 'right'\n");
      return 2;
    }
    ControlConfig cfg;
    if (!load_config(config_path, cfg)) {
      std::fprintf(stderr, "failed to load config: %s\n", config_path.c_str());
      return 2;
    }
    const std::array<double, 6>& home_q = (side == "left")
        ? cfg.leader_home_left : cfg.leader_home_right;

    Eigen::Matrix4d T_home;
    forward_kinematics(home_q, robot_type, T_home);
    const Eigen::Vector3d home_pos = T_home.block<3, 1>(0, 3);

    // Auto targets: home (origin) + orthogonal displacements.
    ur_points = {
      home_pos,
      home_pos + Eigen::Vector3d{displacement, 0.0, 0.0},
      home_pos + Eigen::Vector3d{0.0, displacement, 0.0},
      home_pos + Eigen::Vector3d{0.0, 0.0, displacement},
    };
    ur_labels = {
      "NEUTRAL  (your relaxed forward pose — corresponds to UR home EE)",
      "FORWARD  (move +X by " + std::to_string(displacement) + " m from neutral)",
      "LEFT     (move +Y by " + std::to_string(displacement) + " m from neutral)",
      "UP       (move +Z by " + std::to_string(displacement) + " m from neutral)",
    };
    std::printf(">>> home-mode: %s arm, robot=%s, home_EE=(%.3f, %.3f, %.3f), disp=%.2fm\n",
                side.c_str(), robot_type.c_str(),
                home_pos.x(), home_pos.y(), home_pos.z(), displacement);
  } else if (!points_arg.empty()) {
    ur_points = parse_points_arg(points_arg);
  } else {
    ur_points = {
      Eigen::Vector3d{0.3, 0.0, 0.4},
      Eigen::Vector3d{0.0, 0.3, 0.4},
      Eigen::Vector3d{0.0, 0.0, 0.6},
    };
  }
  if (ur_points.size() < 3) {
    std::fprintf(stderr,
        "need at least 3 points (got %zu) — non-collinear\n",
        ur_points.size());
    return 2;
  }
  // Pad labels if --points / default mode didn't set any.
  while (ur_labels.size() < ur_points.size()) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "point %zu", ur_labels.size() + 1);
    ur_labels.emplace_back(buf);
  }

  // Tracker init
  ViveTracker tracker;
  ViveTracker::Config tc;
  tc.target_serial = serial;
  tc.init_timeout_sec = 10.0;
  if (!tracker.init(tc)) {
    std::fprintf(stderr, "ViveTracker.init failed for serial=%s\n",
                 serial.c_str());
    return 3;
  }
  std::printf(">>> tracker ready: %s\n", tracker.serial().c_str());

  // Capture loop
  std::vector<Eigen::Vector3d> tracker_points;
  tracker_points.reserve(ur_points.size());
  for (std::size_t k = 0; k < ur_points.size(); ++k) {
    const Eigen::Vector3d& p = ur_points[k];
    std::printf("\n--- point %zu / %zu : %s ---\n",
                k + 1, ur_points.size(), ur_labels[k].c_str());
    std::printf("  target (UR base frame): (%.3f, %.3f, %.3f)\n",
                p.x(), p.y(), p.z());
    std::printf("  hold tracker steady at this position, then press Enter...\n");
    std::cout.flush();
    std::string line;
    if (!std::getline(std::cin, line)) {
      std::fprintf(stderr, "input closed — aborting\n");
      return 4;
    }
    Eigen::Vector3d tp;
    if (!average_pose(tracker, avg_ms, tp)) {
      std::fprintf(stderr,
          "  tracker not visible during %.0fms averaging window — retry\n",
          avg_ms);
      --k;
      continue;
    }
    std::printf("  captured tracker pos = (%.4f, %.4f, %.4f)\n",
                tp.x(), tp.y(), tp.z());
    tracker_points.push_back(tp);
  }

  // Solve
  Eigen::Matrix4d T_ur_from_tracker;
  if (!Calibration::solve_from_points(tracker_points, ur_points,
                                       T_ur_from_tracker)) {
    std::fprintf(stderr,
        "Umeyama solve failed — points may be collinear or duplicated\n");
    return 5;
  }

  // Save
  Calibration calib(T_ur_from_tracker);
  if (!calib.save(out_path)) {
    std::fprintf(stderr, "failed to write YAML to %s\n", out_path.c_str());
    return 6;
  }
  std::printf("\n>>> calibration written to %s\n", out_path.c_str());

  // Print residual for sanity
  double err_sq = 0.0;
  for (std::size_t k = 0; k < ur_points.size(); ++k) {
    Eigen::Vector4d th;
    th << tracker_points[k], 1.0;
    Eigen::Vector4d uh = T_ur_from_tracker * th;
    err_sq += (uh.head<3>() - ur_points[k]).squaredNorm();
  }
  std::printf(">>> RMS residual = %.4f m  (good: < 0.01)\n",
              std::sqrt(err_sq / ur_points.size()));

  tracker.shutdown();
  return 0;
}
