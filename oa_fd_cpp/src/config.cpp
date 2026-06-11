// config.cpp — yaml-cpp loader for OaFdConfig.

#include "oa_fd_cpp/config.hpp"

#include <cstdio>
#include <yaml-cpp/yaml.h>

namespace oa_fd {

namespace {
bool read_vec7(const YAML::Node& n, Vec7& out) {
  if (!n || !n.IsSequence() || n.size() != DOF) return false;
  for (int i = 0; i < DOF; ++i) out[i] = n[i].as<double>();
  return true;
}
void try_vec7(const YAML::Node& parent, const std::string& key, Vec7& out) {
  if (parent[key] && !read_vec7(parent[key], out))
    std::fprintf(stderr, "[config] '%s' not a %d-vec; ignored\n", key.c_str(), DOF);
}
template <typename T>
void try_scalar(const YAML::Node& parent, const std::string& key, T& out) {
  if (parent[key]) { try { out = parent[key].as<T>(); } catch (...) {
    std::fprintf(stderr, "[config] could not parse '%s'\n", key.c_str()); } }
}
}  // namespace

bool load_config(const std::string& path, OaFdConfig& c) {
  YAML::Node root;
  try { root = YAML::LoadFile(path); }
  catch (const std::exception& e) {
    std::fprintf(stderr, "[config] load '%s' failed: %s\n", path.c_str(), e.what());
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
  try_vec7(root, "Kp",           c.Kp);
  try_vec7(root, "Kd",           c.Kd);

  if (const auto& fr = root["friction"]) {
    try_vec7(fr, "Fc", c.fric_Fc);
    try_vec7(fr, "k",  c.fric_k);
    try_vec7(fr, "Fv", c.fric_Fv);
    try_vec7(fr, "Fo", c.fric_Fo);
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
    if (g["scale_joints"] && g["scale_joints"].IsSequence() &&
        g["scale_joints"].size() == DOF)
      for (int i = 0; i < DOF; ++i)
        c.gravity.scale_joints[i] = g["scale_joints"][i].as<double>();
    auto read3 = [](const YAML::Node& n, std::array<double, 3>& o) {
      if (n && n.IsSequence() && n.size() == 3)
        for (int i = 0; i < 3; ++i) o[i] = n[i].as<double>();
    };
    // global 'vec' is the fallback for both sides
    read3(g["vec"], c.gravity.vec);
    c.grav_vec_right = c.gravity.vec;
    c.grav_vec_left  = c.gravity.vec;
    read3(g["vec_right"], c.grav_vec_right);
    read3(g["vec_left"],  c.grav_vec_left);

    // per-side chain endpoints: a single 'root_link'/'tip_link' applies to both;
    // per-side keys override.
    if (g["root_link"]) { c.root_link_right = c.root_link_left = c.gravity.root_link; }
    if (g["tip_link"])  { c.tip_link_right  = c.tip_link_left  = c.gravity.tip_link; }
    try_scalar(g, "root_link_right", c.root_link_right);
    try_scalar(g, "tip_link_right",  c.tip_link_right);
    try_scalar(g, "root_link_left",  c.root_link_left);
    try_scalar(g, "tip_link_left",   c.tip_link_left);
  }
  return true;
}

}  // namespace oa_fd
