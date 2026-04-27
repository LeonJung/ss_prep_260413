// config.cpp — yaml-cpp based loader for ControlConfig.

#include "ur10e_teleop_control_hybrid_cpp/config.hpp"

#include <cstdio>
#include <yaml-cpp/yaml.h>

namespace ur10e_teleop_control_hybrid_cpp {

namespace {

bool read_vec6(const YAML::Node& node, Vec6& out) {
  if (!node || !node.IsSequence() || node.size() != 6) return false;
  for (size_t i = 0; i < 6; ++i) out[i] = node[i].as<double>();
  return true;
}

void try_vec6(const YAML::Node& parent, const std::string& key, Vec6& out) {
  if (parent[key]) {
    if (!read_vec6(parent[key], out)) {
      std::fprintf(stderr, "[config] field '%s' is not a 6-value sequence; ignored\n",
                   key.c_str());
    }
  }
}

bool try_vec6_flag(const YAML::Node& parent, const std::string& key, Vec6& out) {
  if (parent[key]) {
    if (read_vec6(parent[key], out)) return true;
    std::fprintf(stderr, "[config] field '%s' is not a 6-value sequence; ignored\n",
                 key.c_str());
  }
  return false;
}

template <typename T>
void try_scalar(const YAML::Node& parent, const std::string& key, T& out) {
  if (parent[key]) {
    try {
      out = parent[key].as<T>();
    } catch (...) {
      std::fprintf(stderr, "[config] could not parse '%s'; keeping default\n",
                   key.c_str());
    }
  }
}

}  // namespace

bool load_config(const std::string& path, ControlConfig& out) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[config] failed to load '%s': %s\n", path.c_str(), e.what());
    return false;
  }

  // top-level
  try_scalar(root, "friction_comp",           out.friction_comp);
  try_scalar(root, "gravity_comp_internal",   out.gravity_comp_internal);
  try_scalar(root, "auto_home_on_start",      out.auto_home_on_start);
  try_scalar(root, "homing_duration",         out.homing_duration);
  try_scalar(root, "homing_contact_threshold", out.homing_contact_threshold);
  try_scalar(root, "auto_power_cycle",        out.auto_power_cycle);
  try_scalar(root, "timestep",                out.timestep);

  // workspace_limits: {enabled, xyz_min[3], xyz_max[3], k_wall, soft_penetration}
  if (auto wl = root["workspace_limits"]) {
    try_scalar(wl, "enabled", out.ws_enabled);
    try_scalar(wl, "k_wall",  out.ws_k_wall);
    try_scalar(wl, "soft_penetration", out.ws_soft_penetration);
    auto read3 = [](const YAML::Node& n, std::array<double, 3>& o) {
      if (!n || !n.IsSequence() || n.size() != 3) return;
      for (size_t i = 0; i < 3; ++i) o[i] = n[i].as<double>();
    };
    read3(wl["xyz_min"], out.ws_xyz_min);
    read3(wl["xyz_max"], out.ws_xyz_max);
  }

  try_vec6(root, "torque_limit",    out.torque_limit);
  out.has_leader_torque_limit =
    try_vec6_flag(root, "leader_torque_limit",   out.leader_torque_limit);
  out.has_follower_torque_limit =
    try_vec6_flag(root, "follower_torque_limit", out.follower_torque_limit);

  try_vec6(root, "leader_home",    out.leader_home);
  try_vec6(root, "follower_home",  out.follower_home);

  if (root["joint_mirror"] && root["joint_mirror"]["sign"]) {
    try_vec6(root["joint_mirror"], "sign", out.mirror_sign);
  }

  if (const auto& ld = root["leader"]) {
    try_vec6(ld, "KP_USER",  out.leader_kp_user);
    try_vec6(ld, "KD_USER",  out.leader_kd_user);
    try_vec6(ld, "KP_HOLD",  out.leader_kp_hold);
    try_vec6(ld, "KD_HOLD",  out.leader_kd_hold);
    try_vec6(ld, "KP_BI",    out.leader_kp_bi);
    try_vec6(ld, "KD_BI",    out.leader_kd_bi);
    try_vec6(ld, "TAU_BI_DEADBAND",      out.leader_tau_bi_deadband);
    try_vec6(ld, "OVERFORCE_USER",       out.leader_overforce_user);
    try_vec6(ld, "OVERFORCE_CONSTRAINT", out.leader_overforce_constraint);
    try_vec6(ld, "K_FT",                 out.leader_k_ft);
    try_vec6(ld, "KD_ACTIVE",            out.leader_kd_active);
  }

  if (const auto& fo = root["follower"]) {
    try_vec6(fo, "KP_TRACK", out.follower_kp_track);
    try_vec6(fo, "KD_TRACK", out.follower_kd_track);
    try_vec6(fo, "KP_HOLD",  out.follower_kp_hold);
    try_vec6(fo, "KD_HOLD",  out.follower_kd_hold);
    try_vec6(fo, "OVERFORCE_CONSTRAINT", out.follower_overforce_constraint);
  }

  if (const auto& fr = root["friction"]) {
    try_vec6(fr, "Fc", out.friction_fc);
    try_vec6(fr, "Fv", out.friction_fv);
    try_vec6(fr, "k",  out.friction_k);
  }

  if (const auto& hy = root["hybrid"]) {
    try_vec6(hy, "KP", out.hybrid_kp);
    try_vec6(hy, "KD", out.hybrid_kd);
    try_vec6(hy, "KF", out.hybrid_kf);
    try_vec6(hy, "D_VISCOUS", out.hybrid_d_viscous);
    try_scalar(hy, "dob_cutoff_hz",        out.hybrid_dob_cutoff_hz);
    try_scalar(hy, "dob_accel_cutoff_hz",  out.hybrid_dob_accel_cutoff_hz);
    try_scalar(hy, "velocity_cutoff_hz",   out.hybrid_velocity_cutoff_hz);
    try_scalar(hy, "tau_ext_cancel_gain",  out.hybrid_tau_ext_cancel_gain);
    try_scalar(hy, "base_link", out.hybrid_base_link);
    try_scalar(hy, "tip_link",  out.hybrid_tip_link);

    if (const auto& tk = hy["tank"]) {
      try_scalar(tk, "enabled",         out.hybrid_tank_enabled);
      try_scalar(tk, "e_max",           out.hybrid_tank_e_max);
      try_scalar(tk, "e_init",          out.hybrid_tank_e_init);
      try_scalar(tk, "refill_ceiling",  out.hybrid_tank_refill_ceiling);
      try_vec6(tk,   "D_DISSIPATION",   out.hybrid_tank_d_dissipation);
    }
  }

  return true;
}

}  // namespace ur10e_teleop_control_hybrid_cpp
