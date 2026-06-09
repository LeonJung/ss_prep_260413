// config.cpp — yaml-cpp loader for OaPpConfig.

#include "oa_pp_cpp/config.hpp"

#include <cstdio>
#include <yaml-cpp/yaml.h>

namespace oa_pp {

namespace {

bool read_vec7(const YAML::Node& n, Vec7& out) {
  if (!n || !n.IsSequence() || n.size() != DOF) return false;
  for (int i = 0; i < DOF; ++i) out[i] = n[i].as<double>();
  return true;
}

void try_vec7(const YAML::Node& parent, const std::string& key, Vec7& out) {
  if (parent[key] && !read_vec7(parent[key], out)) {
    std::fprintf(stderr, "[config] '%s' is not a %d-value sequence; ignored\n",
                 key.c_str(), DOF);
  }
}

template <typename T>
void try_scalar(const YAML::Node& parent, const std::string& key, T& out) {
  if (parent[key]) {
    try { out = parent[key].as<T>(); }
    catch (...) {
      std::fprintf(stderr, "[config] could not parse '%s'; keeping default\n",
                   key.c_str());
    }
  }
}

}  // namespace

bool load_config(const std::string& path, OaPpConfig& c) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[config] failed to load '%s': %s\n", path.c_str(), e.what());
    return false;
  }

  if (const auto& can = root["can"]) {
    try_scalar(can, "leader_right",   c.can_leader_right);
    try_scalar(can, "leader_left",    c.can_leader_left);
    try_scalar(can, "follower_right", c.can_follower_right);
    try_scalar(can, "follower_left",  c.can_follower_left);
    try_scalar(can, "fd",             c.can_fd);
    try_scalar(can, "recv_timeout_us", c.recv_timeout_us);
  }

  try_scalar(root, "timestep",           c.timestep);
  try_scalar(root, "auto_home_on_start", c.auto_home_on_start);
  try_scalar(root, "homing_duration",    c.homing_duration);

  try_vec7(root, "torque_limit", c.torque_limit);
  try_vec7(root, "home",         c.home);

  if (const auto& ld = root["leader"]) {
    try_vec7(ld, "KP_BI",    c.leader_kp_bi);
    try_vec7(ld, "KD_BI",    c.leader_kd_bi);
    try_vec7(ld, "DEADBAND", c.leader_deadband);
    try_vec7(ld, "KP_HOLD",  c.leader_kp_hold);
    try_vec7(ld, "KD_HOLD",  c.leader_kd_hold);
  }
  if (const auto& fo = root["follower"]) {
    try_vec7(fo, "KP_TRACK", c.follower_kp_track);
    try_vec7(fo, "KD_TRACK", c.follower_kd_track);
    try_vec7(fo, "KP_HOLD",  c.follower_kp_hold);
    try_vec7(fo, "KD_HOLD",  c.follower_kd_hold);
  }
  if (const auto& mr = root["mirror"]) {
    try_vec7(mr, "right", c.mirror_right);
    try_vec7(mr, "left",  c.mirror_left);
  }

  if (const auto& g = root["gravity"]) {
    try_scalar(g, "enabled",   c.gravity.enabled);
    try_scalar(g, "urdf",      c.gravity.urdf);
    try_scalar(g, "root_link", c.gravity.root_link);
    try_scalar(g, "tip_link",  c.gravity.tip_link);
    try_scalar(g, "scale",     c.gravity.scale);
    if (g["vec"] && g["vec"].IsSequence() && g["vec"].size() == 3)
      for (int i = 0; i < 3; ++i) c.gravity.vec[i] = g["vec"][i].as<double>();
  }

  return true;
}

}  // namespace oa_pp
