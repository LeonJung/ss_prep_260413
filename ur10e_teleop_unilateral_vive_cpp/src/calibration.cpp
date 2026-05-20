#include "ur10e_teleop_unilateral_vive_cpp/calibration.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace ur10e_teleop_unilateral_vive_cpp {

bool Calibration::load(const std::string& yaml_path) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);
    auto m = root["T_ur_from_tracker"];
    if (!m || m.size() != 4) return false;
    Eigen::Matrix4d T;
    for (int i = 0; i < 4; ++i) {
      if (m[i].size() != 4) return false;
      for (int j = 0; j < 4; ++j) {
        T(i, j) = m[i][j].as<double>();
      }
    }
    T_ = T;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool Calibration::save(const std::string& yaml_path) const {
  std::ofstream f(yaml_path);
  if (!f) return false;
  f << "# calibration.yaml — tracker-frame → UR-base-frame rigid transform\n";
  f << "T_ur_from_tracker:\n";
  f << std::scientific << std::setprecision(12);
  for (int i = 0; i < 4; ++i) {
    f << "  - [";
    for (int j = 0; j < 4; ++j) {
      f << T_(i, j);
      if (j < 3) f << ", ";
    }
    f << "]\n";
  }
  return true;
}

bool Calibration::solve_from_points(
    const std::vector<Eigen::Vector3d>& tracker_points,
    const std::vector<Eigen::Vector3d>& ur_points,
    Eigen::Matrix4d& T_out) {
  if (tracker_points.size() < 3 || tracker_points.size() != ur_points.size()) {
    return false;
  }

  // Umeyama (no scaling): minimize sum |ur - (R * tracker + t)|².
  const Eigen::Index n = static_cast<Eigen::Index>(tracker_points.size());

  // Centroids
  Eigen::Vector3d c_tracker = Eigen::Vector3d::Zero();
  Eigen::Vector3d c_ur      = Eigen::Vector3d::Zero();
  for (Eigen::Index i = 0; i < n; ++i) {
    c_tracker += tracker_points[i];
    c_ur      += ur_points[i];
  }
  c_tracker /= static_cast<double>(n);
  c_ur      /= static_cast<double>(n);

  // Cross-covariance H = Σ (tracker_i − c_tracker)(ur_i − c_ur)^T
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (Eigen::Index i = 0; i < n; ++i) {
    H += (tracker_points[i] - c_tracker) * (ur_points[i] - c_ur).transpose();
  }

  // SVD → R = V * diag(1,1,det(V U^T)) * U^T  (so det(R) = +1, proper rot)
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  if (svd.singularValues().minCoeff() < 1e-9) {
    // Degenerate (collinear or coincident points)
    return false;
  }
  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  if ((V * U.transpose()).determinant() < 0.0) D(2, 2) = -1.0;
  Eigen::Matrix3d R = V * D * U.transpose();
  Eigen::Vector3d t = c_ur - R * c_tracker;

  T_out.setIdentity();
  T_out.block<3, 3>(0, 0) = R;
  T_out.block<3, 1>(0, 3) = t;
  return true;
}

}  // namespace ur10e_teleop_unilateral_vive_cpp
