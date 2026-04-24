#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

namespace ur10e_teleop_control_hybrid_cpp {

// KDL-backed rigid-body dynamics for a 6-DOF UR arm. Loads URDF once at
// construction; each query recomputes M(q), C(q,q̇)q̇, g(q) for the
// current state. Not thread-safe (one instance per RT thread).
class DynamicsModel {
 public:
  // Parses the URDF at `urdf_path`, extracts a chain from `base_link`
  // to `tip_link`, and initializes KDL dynamics solvers. Throws if the
  // chain is not 6-DOF revolute.
  DynamicsModel(const std::string& urdf_path,
                const std::string& base_link,
                const std::string& tip_link,
                const std::array<double, 3>& gravity = {0.0, 0.0, -9.81});

  // M(q) — 6x6 joint-space inertia matrix.
  const Eigen::Matrix<double, 6, 6>& mass(const Eigen::VectorXd& q);

  // C(q,q̇)·q̇ — 6x1 Coriolis/centrifugal torque vector.
  const Eigen::Matrix<double, 6, 1>& coriolis(const Eigen::VectorXd& q,
                                              const Eigen::VectorXd& q_dot);

  // g(q) — 6x1 gravity torque vector.
  const Eigen::Matrix<double, 6, 1>& gravity(const Eigen::VectorXd& q);

  std::size_t n_joints() const { return n_; }

 private:
  void load_chain_(const std::string& urdf_path,
                   const std::string& base_link,
                   const std::string& tip_link);

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  std::size_t n_{0};

  // Reused KDL scratch to avoid per-call heap allocation.
  KDL::JntArray kq_;
  KDL::JntArray kqd_;
  KDL::JntArray kout_;
  KDL::JntSpaceInertiaMatrix kmass_;

  // Return-by-reference caches.
  Eigen::Matrix<double, 6, 6> M_cache_;
  Eigen::Matrix<double, 6, 1> C_cache_;
  Eigen::Matrix<double, 6, 1> g_cache_;
};

}  // namespace ur10e_teleop_control_hybrid_cpp
