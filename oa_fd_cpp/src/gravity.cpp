// gravity.cpp — KDL gravity model. See gravity.hpp.

#include "oa_fd_cpp/gravity.hpp"

#include <cstdio>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace oa_fd {

GravityModel::GravityModel() = default;
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

  chain_ = std::make_unique<KDL::Chain>();
  if (!tree.getChain(cfg.root_link, cfg.tip_link, *chain_)) {
    std::fprintf(stderr, "[gravity] no chain %s -> %s in URDF\n",
                 cfg.root_link.c_str(), cfg.tip_link.c_str());
    return false;
  }
  KDL::Chain& chain = *chain_;   // owned; outlives dyn_

  n_ = static_cast<int>(chain.getNrOfJoints());
  if (n_ != DOF) {
    std::fprintf(stderr,
      "[gravity] chain has %d joints, expected %d (root=%s tip=%s). "
      "Check root/tip links.\n", n_, DOF, cfg.root_link.c_str(), cfg.tip_link.c_str());
    // still proceed if n_ > 0, clamping in gravity()
  }

  // Sanity: how much mass did kdl_parser actually attach to the chain?
  // (Some kdl_parser backends silently drop URDF <inertial> -> zero gravity.)
  double total_mass = 0.0;
  for (unsigned i = 0; i < chain.getNrOfSegments(); ++i)
    total_mass += chain.getSegment(i).getInertia().getMass();

  KDL::Vector g(cfg.vec[0], cfg.vec[1], cfg.vec[2]);
  dyn_ = std::make_unique<KDL::ChainDynParam>(chain, g);
  scale_ = cfg.scale;
  scale_joints_ = cfg.scale_joints;
  ok_ = (n_ > 0);
  if (ok_) {
    std::fprintf(stderr,
      "[gravity] loaded: %d joints, %u segs, total_mass=%.4f kg, "
      "g=(%.2f,%.2f,%.2f), scale=%.3f\n",
      n_, chain.getNrOfSegments(), total_mass,
      cfg.vec[0], cfg.vec[1], cfg.vec[2], scale_);
    if (total_mass < 1e-6)
      std::fprintf(stderr,
        "[gravity] WARNING: chain total mass ~0 -> kdl_parser dropped inertials; "
        "gravity will be 0 (URDF has masses, so it's a parser issue).\n");
  }
  return ok_;
}

void GravityModel::gravity(const Vec7& q, Vec7& tau_g) const {
  tau_g.fill(0.0);
  if (!ok_ || !dyn_) return;
  KDL::JntArray jq(n_), jg(n_);
  for (int i = 0; i < n_ && i < DOF; ++i) jq(i) = q[i];
  last_rc_ = dyn_->JntToGravity(jq, jg);
  if (last_rc_ != 0) return;   // KDL error -> leave zeros
  for (int i = 0; i < n_ && i < DOF; ++i)
    tau_g[i] = scale_ * scale_joints_[i] * jg(i);
}

}  // namespace oa_fd
