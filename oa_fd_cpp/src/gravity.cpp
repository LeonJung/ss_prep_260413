// gravity.cpp — KDL gravity model. See gravity.hpp.

#include "oa_fd_cpp/gravity.hpp"

#include <cstdio>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace oa_fd {

GravityModel::~GravityModel() = default;

bool GravityModel::load(const GravityCfg& cfg) {
  ok_ = false;
  if (!cfg.enabled) return false;
  if (cfg.urdf.empty()) {
    std::fprintf(stderr, "[gravity] enabled but no URDF path; gravity comp OFF\n");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(cfg.urdf, tree)) {
    std::fprintf(stderr, "[gravity] failed to parse URDF '%s'\n", cfg.urdf.c_str());
    return false;
  }

  KDL::Chain chain;
  if (!tree.getChain(cfg.root_link, cfg.tip_link, chain)) {
    std::fprintf(stderr, "[gravity] no chain %s -> %s in URDF\n",
                 cfg.root_link.c_str(), cfg.tip_link.c_str());
    return false;
  }

  n_ = static_cast<int>(chain.getNrOfJoints());
  if (n_ != DOF) {
    std::fprintf(stderr,
      "[gravity] chain has %d joints, expected %d (root=%s tip=%s). "
      "Check root/tip links.\n", n_, DOF, cfg.root_link.c_str(), cfg.tip_link.c_str());
    // still proceed if n_ > 0, clamping in gravity()
  }

  KDL::Vector g(cfg.vec[0], cfg.vec[1], cfg.vec[2]);
  dyn_ = std::make_unique<KDL::ChainDynParam>(chain, g);
  scale_ = cfg.scale;
  ok_ = (n_ > 0);
  if (ok_)
    std::fprintf(stderr, "[gravity] loaded: %d joints, g=(%.2f,%.2f,%.2f), scale=%.3f\n",
                 n_, cfg.vec[0], cfg.vec[1], cfg.vec[2], scale_);
  return ok_;
}

void GravityModel::gravity(const Vec7& q, Vec7& tau_g) const {
  tau_g.fill(0.0);
  if (!ok_ || !dyn_) return;
  KDL::JntArray jq(n_), jg(n_);
  for (int i = 0; i < n_ && i < DOF; ++i) jq(i) = q[i];
  if (dyn_->JntToGravity(jq, jg) != 0) return;
  for (int i = 0; i < n_ && i < DOF; ++i) tau_g[i] = scale_ * jg(i);
}

}  // namespace oa_fd
