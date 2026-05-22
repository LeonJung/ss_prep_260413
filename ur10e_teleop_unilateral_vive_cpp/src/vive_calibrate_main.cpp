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

#include <sys/stat.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ur10e_teleop_unilateral_vive_cpp/calibration.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/config.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/ur_jacobian.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/vive_tracker.hpp"

using ur10e_teleop_unilateral_vive_cpp::Calibration;
using ur10e_teleop_unilateral_vive_cpp::ViveTracker;

namespace {

void print_usage(const char* prog) {
  std::fprintf(stderr,
    "Usage: %s left|right [--displacement 0.3]\n"
    "                     [--serial LHR-XXXX] [--out PATH]\n"
    "                     [--config PATH]    [--robot ur10e|ur3e|ur5e]\n"
    "                     [--points 'x1,y1,z1 x2,y2,z2 ...']\n"
    "                     [--post-rot-z DEG] [--avg-ms 500]\n"
    "\n"
    "Single positional arg (left|right) selects the arm. Sensible\n"
    "defaults are filled in from the package's share/config dir and\n"
    "from the baked-in lab tracker serials:\n"
    "   left  → LHR-B4BFDF90\n"
    "   right → LHR-C21814A6\n"
    "Out / config paths default to:\n"
    "   <pkg-share>/config/calibration_<side>.yaml\n"
    "   <pkg-share>/config/real_ur.yaml\n"
    "Robot defaults to ur10e (the actual follower).\n"
    "\n"
    "Default capture sequence (operator-frame, displacement 0.3 m):\n"
    "  1. NEUTRAL   (your relaxed arm-forward pose ↔ UR home EE)\n"
    "  2. FORWARD   (+X 0.3 m from neutral)\n"
    "  3. LEFT      (+Y 0.3 m from neutral)\n"
    "  4. UP        (+Z 0.3 m from neutral)\n"
    "\n"
    "Override --points to use teach-pendant TCP triplets instead\n"
    "(useful when the UR base frame is rotated relative to operator).\n",
    prog);
}

// Lab-bound tracker serials. Baked here so a positional `side` arg
// alone is enough to pick the right tracker.
const char* serial_for_side(const std::string& side) {
  if (side == "left")  return "LHR-B4BFDF90";
  if (side == "right") return "LHR-C21814A6";
  return nullptr;
}

std::string pkg_share() {
  return ament_index_cpp::get_package_share_directory(
      "ur10e_teleop_unilateral_vive_cpp");
}

// Per-user calibration directory. We deliberately don't write to
// pkg_share/config because colcon build copies src/config → install,
// which overwrites cali results with the placeholder YAMLs on every
// rebuild. ~/.ros is the canonical ROS user-data location and survives
// builds cleanly.
std::string user_calib_dir() {
  const char* home = std::getenv("HOME");
  const std::string base = home ? std::string(home) : std::string(".");
  const std::string dir = base + "/.ros/ur10e_teleop_unilateral_vive_cpp";
  // mkdir -p (best-effort; save() will report errors on write failure)
  ::mkdir((base + "/.ros").c_str(), 0755);
  ::mkdir(dir.c_str(), 0755);
  return dir;
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
  std::string robot_type = "ur10e";
  std::string side;                 // "left" | "right"
  double displacement = 0.3;
  double avg_ms = 500.0;
  double post_rot_z_deg = 0.0;      // post-rotation about UR Z, applied to
                                    // the solved calibration (pre-multiply).
                                    // Lab finding (2026-05-22): this setup
                                    // needs -90° to align tracker motions
                                    // with operator-frame axes.

  // Positional first arg = side (if not a --flag).
  int i = 1;
  if (i < argc && argv[i][0] != '-') {
    side = argv[i++];
  }

  for (; i < argc; ++i) {
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
    else if (a == "--config")        config_path = need("--config");
    else if (a == "--robot")         robot_type = need("--robot");
    else if (a == "--side")          side = need("--side");
    else if (a == "--displacement")  displacement = std::stod(need("--displacement"));
    else if (a == "--post-rot-z")    post_rot_z_deg = std::stod(need("--post-rot-z"));
    else if (a == "--help" || a == "-h") { print_usage(argv[0]); return 0; }
    else {
      std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
      print_usage(argv[0]);
      return 2;
    }
  }

  if (side != "left" && side != "right") {
    std::fprintf(stderr, "side must be 'left' or 'right' (positional or --side)\n");
    print_usage(argv[0]);
    return 2;
  }

  // Auto-fill defaults from package share dir + baked-in serial map.
  if (serial.empty())      serial = serial_for_side(side);
  std::string share;
  try { share = pkg_share(); }
  catch (const std::exception& e) {
    std::fprintf(stderr,
        "failed to locate package share dir: %s\n"
        "(did you source install/setup.bash?)\n", e.what());
    return 2;
  }
  if (out_path.empty())    out_path    = user_calib_dir() + "/calibration_" + side + ".yaml";
  if (config_path.empty()) config_path = share + "/config/real_ur.yaml";

  std::printf(">>> side=%s  serial=%s\n", side.c_str(), serial.c_str());
  std::printf(">>> out=%s\n", out_path.c_str());
  std::printf(">>> config=%s  robot=%s  displacement=%.2fm\n",
              config_path.c_str(), robot_type.c_str(), displacement);

  // home_mode is the default. If --points is given, explicit mode wins.
  const bool home_mode = points_arg.empty();

  std::vector<Eigen::Vector3d> ur_points;
  std::vector<std::string> ur_labels;        // for operator-friendly prompts

  if (home_mode) {
    using namespace ur10e_teleop_unilateral_vive_cpp;
    ControlConfig cfg;
    if (!load_config(config_path, cfg)) {
      std::fprintf(stderr, "failed to load config: %s\n", config_path.c_str());
      return 2;
    }
    // Use follower_home_* because robot_type defaults to ur10e and
    // those are the UR10e-space home values; UR3e leader_home is a
    // leftover from the parent unilateral package and not used here.
    const std::array<double, 6>& home_q = (side == "left")
        ? cfg.follower_home_left : cfg.follower_home_right;

    Eigen::Matrix4d T_home;
    forward_kinematics(home_q, robot_type, T_home);
    const Eigen::Vector3d home_pos = T_home.block<3, 1>(0, 3);

    // The "Y" displacement is mirrored for the right hand so that
    // moving the right tracker toward the operator's own right (which
    // is world −Y) maps to UR-Right's −Y direction. This makes the
    // EE physically move to the operator's right, matching intuition.
    // Left hand stays with +Y = operator's left (no mirror).
    const double y_sign = (side == "right") ? -1.0 : +1.0;
    const std::string y_label =
        (side == "right") ? "RIGHT    (move -Y direction — to YOUR right — by "
                          : "LEFT     (move +Y direction — to YOUR left — by ";

    ur_points = {
      home_pos,
      home_pos + Eigen::Vector3d{displacement, 0.0, 0.0},
      home_pos + Eigen::Vector3d{0.0, y_sign * displacement, 0.0},
      home_pos + Eigen::Vector3d{0.0, 0.0, displacement},
    };
    ur_labels = {
      "NEUTRAL  (your relaxed forward pose — corresponds to UR home EE)",
      "FORWARD  (move +X by " + std::to_string(displacement) + " m from neutral)",
      y_label + std::to_string(displacement) + " m from neutral)",
      "UP       (move +Z by " + std::to_string(displacement) + " m from neutral)",
    };
    std::printf(">>> home-mode: %s arm, robot=%s, home_EE=(%.3f, %.3f, %.3f), disp=%.2fm  y_sign=%+.0f\n",
                side.c_str(), robot_type.c_str(),
                home_pos.x(), home_pos.y(), home_pos.z(),
                displacement, y_sign);
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

  // Optional post-rotation about UR Z. Applied as M · T (pre-multiply on
  // the output side) so the user's intuitive motion in the operator
  // frame ends up aligned with UR's axes when the raw cali has a known
  // systematic offset.
  if (std::abs(post_rot_z_deg) > 1e-6) {
    const double rad = post_rot_z_deg * M_PI / 180.0;
    const double c = std::cos(rad), s = std::sin(rad);
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(0, 0) =  c;  M(0, 1) = -s;
    M(1, 0) =  s;  M(1, 1) =  c;
    T_ur_from_tracker = M * T_ur_from_tracker;
    std::printf(">>> applied post-rotation: %.1f° about UR Z\n",
                post_rot_z_deg);
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
