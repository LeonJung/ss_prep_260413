// config.cpp — yaml-cpp loader for one ArmCfg + loop globals.

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
// 7-vec OR scalar-applied-to-all (for the velocity gate keys)
void try_vec7_or_scalar(const YAML::Node& parent, const std::string& key, Vec7& out) {
  if (!parent[key]) return;
  if (parent[key].IsSequence()) { try_vec7(parent, key, out); }
  else { double v = out[0]; try_scalar(parent, key, v); out.fill(v); }
}
void read3(const YAML::Node& n, std::array<double, 3>& o) {
  if (n && n.IsSequence() && n.size() == 3)
    for (int i = 0; i < 3; ++i) o[i] = n[i].as<double>();
}
}  // namespace

bool load_arm_config(const std::string& path, ArmCfg& a, GlobalCfg& g) {
  YAML::Node root;
  try { root = YAML::LoadFile(path); }
  catch (const std::exception& e) {
    std::fprintf(stderr, "[config] load '%s' failed: %s\n", path.c_str(), e.what());
    return false;
  }

  // ---- loop globals (every arm file carries these; identical across files) ----
  try_scalar(root, "can",                a.can);
  try_scalar(root, "fd",                 g.can_fd);
  try_scalar(root, "recv_timeout_us",    g.recv_timeout_us);
  try_scalar(root, "timestep",           g.timestep);
  try_scalar(root, "vel_filter_alpha",   g.vel_filter_alpha);
  try_scalar(root, "auto_home_on_start", g.auto_home_on_start);
  try_scalar(root, "homing_duration",    g.homing_duration);
  try_scalar(root, "couple_vel_ff",      g.couple_vel_ff);
  try_scalar(root, "couple_kp_scale",    g.couple_kp_scale);

  // ---- per-arm motion / impedance ----
  try_vec7(root, "Kp", a.Kp);
  try_vec7(root, "Kd", a.Kd);
  try_vec7(root, "home", a.home);
  try_vec7(root, "torque_limit", a.torque_limit);
  try_vec7(root, "couple_mirror", a.couple_mirror);

  // ---- friction ----
  if (const auto& fr = root["friction"]) {
    try_vec7(fr, "Fc", a.fric_Fc);
    try_vec7(fr, "k",  a.fric_k);
    try_vec7(fr, "Fv", a.fric_Fv);
    try_vec7(fr, "Fo", a.fric_Fo);
    try_vec7_or_scalar(fr, "v_start", a.fric_v_start);
    try_vec7_or_scalar(fr, "v_full",  a.fric_v_full);
  }

  // ---- gravity ----
  if (const auto& gr = root["gravity"]) {
    try_scalar(gr, "enabled",   a.grav_enabled);
    try_scalar(gr, "urdf",      a.grav_urdf);
    try_scalar(gr, "root_link", a.root_link);
    try_scalar(gr, "tip_link",  a.tip_link);
    try_scalar(gr, "scale",     a.grav_scale);
    try_vec7(gr, "scale_joints", a.grav_scale_joints);
    try_vec7(gr, "mirror",       a.grav_mirror);
    read3(gr["vec"], a.grav_vec);
  }

  // ---- FREEDRIVE shaping ----
  if (const auto& fd = root["freedrive"]) {
    try_vec7(fd, "posture_kp", a.fd_posture_kp);
    try_vec7(fd, "posture_kd", a.fd_posture_kd);
    try_vec7(fd, "posture_q",  a.fd_posture_q);
    try_vec7(fd, "limit_kp",   a.fd_limit_kp);
    try_vec7(fd, "limit_kd",   a.fd_limit_kd);
    try_vec7(fd, "limit_fmax", a.fd_limit_fmax);
    try_vec7(fd, "limit_push", a.fd_limit_push);
    try_vec7_or_scalar(fd, "limit_margin", a.fd_limit_margin);
    try_scalar(fd, "limit_exit_kd", a.fd_limit_exit_kd);
  }

  // ---- joint limits ----
  if (const auto& jl = root["joint_limits"]) {
    try_vec7(jl, "lower", a.limit_lower);
    try_vec7(jl, "upper", a.limit_upper);
  }
  return true;
}

}  // namespace oa_fd
