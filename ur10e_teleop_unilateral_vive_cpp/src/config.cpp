// config.cpp — yaml-cpp based loader for ControlConfig.

#include "ur10e_teleop_unilateral_vive_cpp/config.hpp"

#include <cstdio>
#include <yaml-cpp/yaml.h>

namespace ur10e_teleop_unilateral_vive_cpp {

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

void try_vec3(const YAML::Node& parent, const std::string& key, Vec3a& out) {
  if (!parent[key]) return;
  const auto& n = parent[key];
  if (!n.IsSequence() || n.size() != 3) {
    std::fprintf(stderr, "[config] field '%s' is not a 3-value sequence; ignored\n",
                 key.c_str());
    return;
  }
  for (int i = 0; i < 3; ++i) out[i] = n[i].as<double>();
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

  try_vec6(root, "joint_vel_limit",           out.joint_vel_limit);

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

  // Bimanual side-specific homes. If the YAML doesn't carry them, fall
  // back to the single-arm leader_home / follower_home.
  out.has_leader_home_left =
      try_vec6_flag(root, "leader_home_left",  out.leader_home_left);
  out.has_leader_home_right =
      try_vec6_flag(root, "leader_home_right", out.leader_home_right);
  out.has_follower_home_left =
      try_vec6_flag(root, "follower_home_left",  out.follower_home_left);
  out.has_follower_home_right =
      try_vec6_flag(root, "follower_home_right", out.follower_home_right);
  if (!out.has_leader_home_left)    out.leader_home_left    = out.leader_home;
  if (!out.has_leader_home_right)   out.leader_home_right   = out.leader_home;
  if (!out.has_follower_home_left)  out.follower_home_left  = out.follower_home;
  if (!out.has_follower_home_right) out.follower_home_right = out.follower_home;

  // Tool mode + offsets (Vive-leader only — followers ignore).
  try_scalar(root, "tool_mode", out.tool_mode);
  if (const auto& tn = root["tool_offsets"]) {
    try_vec3(tn, "arm",  out.tool_offset_arm);
    try_vec3(tn, "hand", out.tool_offset_hand);
  }
  try_vec3(root, "vive_pivot_offset", out.vive_pivot_offset);

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

  return true;
}

}  // namespace ur10e_teleop_unilateral_vive_cpp
