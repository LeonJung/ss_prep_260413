#include "ur10e_teleop_control_hybrid_cpp/dynamics_model.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace ur10e_teleop_control_hybrid_cpp {

DynamicsModel::DynamicsModel(const std::string& urdf_path,
                             const std::string& base_link,
                             const std::string& tip_link,
                             const std::array<double, 3>& gravity) {
  load_chain_(urdf_path, base_link, tip_link);
  n_ = chain_.getNrOfJoints();
  if (n_ != 6) {
    std::ostringstream oss;
    oss << "DynamicsModel: expected 6-DOF chain, got " << n_
        << " joints (" << base_link << " → " << tip_link << ")";
    throw std::runtime_error(oss.str());
  }

  KDL::Vector g_kdl(gravity[0], gravity[1], gravity[2]);
  dyn_ = std::make_unique<KDL::ChainDynParam>(chain_, g_kdl);

  kq_.resize(n_);
  kqd_.resize(n_);
  kout_.resize(n_);
  kmass_.resize(n_);
  M_cache_.setZero();
  C_cache_.setZero();
  g_cache_.setZero();
}

void DynamicsModel::load_chain_(const std::string& urdf_path,
                                const std::string& base_link,
                                const std::string& tip_link) {
  std::ifstream f(urdf_path);
  if (!f.is_open()) {
    throw std::runtime_error("DynamicsModel: cannot open URDF at " + urdf_path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string xml = ss.str();

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(xml, tree)) {
    throw std::runtime_error("DynamicsModel: kdl_parser failed for " + urdf_path);
  }
  if (!tree.getChain(base_link, tip_link, chain_)) {
    throw std::runtime_error("DynamicsModel: no chain from " + base_link +
                             " to " + tip_link);
  }
}

const Eigen::Matrix<double, 6, 6>& DynamicsModel::mass(
    const Eigen::VectorXd& q) {
  for (std::size_t i = 0; i < n_; ++i) kq_(i) = q(i);
  dyn_->JntToMass(kq_, kmass_);
  for (std::size_t r = 0; r < n_; ++r)
    for (std::size_t c = 0; c < n_; ++c) M_cache_(r, c) = kmass_(r, c);
  return M_cache_;
}

const Eigen::Matrix<double, 6, 1>& DynamicsModel::coriolis(
    const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot) {
  for (std::size_t i = 0; i < n_; ++i) {
    kq_(i) = q(i);
    kqd_(i) = q_dot(i);
  }
  dyn_->JntToCoriolis(kq_, kqd_, kout_);
  for (std::size_t i = 0; i < n_; ++i) C_cache_(i) = kout_(i);
  return C_cache_;
}

const Eigen::Matrix<double, 6, 1>& DynamicsModel::gravity(
    const Eigen::VectorXd& q) {
  for (std::size_t i = 0; i < n_; ++i) kq_(i) = q(i);
  dyn_->JntToGravity(kq_, kout_);
  for (std::size_t i = 0; i < n_; ++i) g_cache_(i) = kout_(i);
  return g_cache_;
}

}  // namespace ur10e_teleop_control_hybrid_cpp
