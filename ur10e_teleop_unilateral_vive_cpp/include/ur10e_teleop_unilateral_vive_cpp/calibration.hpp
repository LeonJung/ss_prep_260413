// calibration.hpp — 3-point (or N-point) tracker→UR-base calibration.
//
// The operator places the Vive tracker at 3 (or more) known UR base-
// frame positions and records the tracker pose at each. From the two
// sets of corresponding points we solve for the rigid-body transform
//   T_ur_base_from_tracker  s.t.  ur_pos = T * tracker_pos
// using the Umeyama / Kabsch closed-form algorithm. 3 points is the
// minimum (non-collinear) for full 6-DoF; > 3 points improves accuracy
// via least squares.
//
// Persistence: load/save the 4×4 transform as YAML so calibration
// survives node restarts.

#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace ur10e_teleop_unilateral_vive_cpp {

class Calibration {
 public:
  Calibration() = default;
  explicit Calibration(const Eigen::Matrix4d& T_ur_from_tracker)
      : T_(T_ur_from_tracker) {}

  // Apply: convert a tracker-frame pose to UR-base-frame pose.
  //   T_ur_ee = T_ur_from_tracker * T_tracker_pose
  Eigen::Matrix4d apply(const Eigen::Matrix4d& T_tracker_pose) const {
    return T_ * T_tracker_pose;
  }

  // Pure rotation+translation extraction (no scale).
  const Eigen::Matrix4d& transform() const { return T_; }

  // Load 4×4 from a YAML file. Layout:
  //   T_ur_from_tracker:
  //     - [r00, r01, r02, tx]
  //     - [r10, r11, r12, ty]
  //     - [r20, r21, r22, tz]
  //     - [0,   0,   0,   1]
  bool load(const std::string& yaml_path);

  // Save in the same format.
  bool save(const std::string& yaml_path) const;

  // Solve for the transform from corresponding points (Umeyama / Kabsch).
  // Both vectors must be same size, ≥ 3, and not collinear.
  // Returns false if degenerate.
  static bool solve_from_points(
      const std::vector<Eigen::Vector3d>& tracker_points,
      const std::vector<Eigen::Vector3d>& ur_points,
      Eigen::Matrix4d& T_out);

 private:
  Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();
};

}  // namespace ur10e_teleop_unilateral_vive_cpp
