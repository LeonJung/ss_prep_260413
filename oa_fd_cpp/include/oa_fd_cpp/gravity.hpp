// gravity.hpp — per-joint gravity torque from a URDF via KDL.
//
// openarmx motors have NO firmware gravity compensation (unlike the UR
// firmware that ur10e_teleop_real_cpp relied on). For pure-torque MIT
// control the arm would sag under gravity, so we add g(q) as feedforward.
// Mirrors openarmx_gravity_comp (KDL ChainDynParam::JntToGravity).

#pragma once
#include <memory>
#include <string>

#include "oa_fd_cpp/config.hpp"   // DOF, Vec7, GravityCfg

namespace KDL { class Chain; class ChainDynParam; }

namespace oa_fd {

class GravityModel {
public:
  GravityModel();
  ~GravityModel();

  // Build KDL chain from URDF root->tip. Returns false on failure (then
  // gravity() yields zeros and the caller should treat grav comp as off).
  bool load(const GravityCfg& cfg);

  bool ok() const { return ok_; }

  // g(q) per joint [Nm], already scaled by cfg.scale. Zeros if !ok().
  void gravity(const Vec7& q, Vec7& tau_g) const;

  int last_rc() const { return last_rc_; }   // last JntToGravity() return code

private:
  bool ok_ = false;
  double scale_ = 1.0;
  std::array<double, DOF> scale_joints_ = {1, 1, 1, 1, 1, 1, 1};
  // ChainDynParam stores a const reference to the Chain — it must outlive dyn_,
  // so we own it here (do NOT pass a local chain).
  std::unique_ptr<KDL::Chain> chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  int n_ = 0;
  mutable int last_rc_ = -99;
};

}  // namespace oa_fd
