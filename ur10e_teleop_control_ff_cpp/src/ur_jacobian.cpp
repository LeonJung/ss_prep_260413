// ur_jacobian.cpp — UR e-Series forward kinematics + geometric Jacobian.

#include "ur10e_teleop_control_ff_cpp/ur_jacobian.hpp"

#include <cmath>
#include <stdexcept>

namespace ur10e_teleop_control_ff_cpp {

namespace {

struct DH {
  std::array<double, 6> a;
  std::array<double, 6> d;
  std::array<double, 6> alpha;
};

const DH& dh_for(const std::string& robot) {
  static const DH UR3E{
    {0.0, -0.24355, -0.21325, 0.0, 0.0, 0.0},
    {0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921},
    {M_PI_2, 0.0, 0.0, M_PI_2, -M_PI_2, 0.0}
  };
  static const DH UR10E{
    {0.0, -0.6127, -0.5716, 0.0, 0.0, 0.0},
    {0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655},
    {M_PI_2, 0.0, 0.0, M_PI_2, -M_PI_2, 0.0}
  };
  static const DH UR5E{
    {0.0, -0.425, -0.3922, 0.0, 0.0, 0.0},
    {0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996},
    {M_PI_2, 0.0, 0.0, M_PI_2, -M_PI_2, 0.0}
  };
  if (robot == "ur3e")  return UR3E;
  if (robot == "ur10e") return UR10E;
  if (robot == "ur5e")  return UR5E;
  throw std::invalid_argument("ur_jacobian: unknown robot '" + robot + "'");
}

}  // namespace

Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta) {
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  const double ct = std::cos(theta), st = std::sin(theta);
  Eigen::Matrix4d T;
  T <<  ct, -st * ca,  st * sa, a * ct,
        st,  ct * ca, -ct * sa, a * st,
       0.0,       sa,       ca,      d,
       0.0,      0.0,      0.0,    1.0;
  return T;
}

void forward_kinematics(const std::array<double, 6>& q,
                        const std::string& robot,
                        Eigen::Matrix4d& out_T,
                        std::array<Eigen::Matrix4d, 6>* out_Ts) {
  const DH& dh = dh_for(robot);
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 6; ++i) {
    T = T * dh_transform(dh.a[i], dh.alpha[i], dh.d[i], q[i]);
    if (out_Ts) (*out_Ts)[i] = T;
  }
  out_T = T;
}

Mat66 ur_jacobian(const std::array<double, 6>& q, const std::string& robot) {
  std::array<Eigen::Matrix4d, 6> Ts;
  Eigen::Matrix4d T;
  forward_kinematics(q, robot, T, &Ts);
  Vec3 p_n = Ts.back().block<3, 1>(0, 3);

  Mat66 J = Mat66::Zero();
  Vec3 z_prev(0.0, 0.0, 1.0);     // base +z (joint 0 axis)
  Vec3 p_prev = Vec3::Zero();
  for (int i = 0; i < 6; ++i) {
    J.block<3, 1>(0, i) = z_prev.cross(p_n - p_prev);
    J.block<3, 1>(3, i) = z_prev;
    z_prev = Ts[i].block<3, 1>(0, 2);
    p_prev = Ts[i].block<3, 1>(0, 3);
  }
  return J;
}

Vec3 tcp_position(const std::array<double, 6>& q, const std::string& robot) {
  Eigen::Matrix4d T;
  forward_kinematics(q, robot, T);
  return T.block<3, 1>(0, 3);
}

}  // namespace ur10e_teleop_control_ff_cpp
