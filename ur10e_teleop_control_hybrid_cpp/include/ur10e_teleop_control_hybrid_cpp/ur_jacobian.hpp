// ur_jacobian.hpp — analytical geometric Jacobian + forward kinematics
// for UR e-Series robots (UR3e / UR10e / UR5e).
//
// No external deps beyond Eigen. Used by follower_node for:
//   (a) TCP position from actual_q
//   (b) mapping a Cartesian virtual-wall force to joint torque (J^T · F)

#pragma once
#include <array>
#include <string>

#include <Eigen/Dense>

namespace ur10e_teleop_control_hybrid_cpp {

using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Mat66 = Eigen::Matrix<double, 6, 6>;

// Fill in one 4x4 link transform (standard DH).
Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta);

// Forward kinematics. Returns T_0_6 via out_T. If out_Ts != nullptr, fills
// with the 6 cumulative transforms T_0_1 .. T_0_6.
void forward_kinematics(const std::array<double, 6>& q,
                        const std::string& robot,
                        Eigen::Matrix4d& out_T,
                        std::array<Eigen::Matrix4d, 6>* out_Ts = nullptr);

// Geometric Jacobian (6x6) such that [v; w] = J * qdot (base frame).
Mat66 ur_jacobian(const std::array<double, 6>& q,
                  const std::string& robot);

// TCP position (3-vector) in base frame.
Vec3 tcp_position(const std::array<double, 6>& q,
                  const std::string& robot);

}  // namespace ur10e_teleop_control_hybrid_cpp
