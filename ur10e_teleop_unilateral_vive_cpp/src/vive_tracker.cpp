#include "ur10e_teleop_unilateral_vive_cpp/vive_tracker.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

#include <openvr.h>

namespace ur10e_teleop_unilateral_vive_cpp {

namespace {

// Convert OpenVR's 3×4 matrix to Eigen 4×4.
Eigen::Matrix4d to_eigen(const vr::HmdMatrix34_t& m) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      T(r, c) = static_cast<double>(m.m[r][c]);
    }
  }
  return T;
}

std::string device_serial(vr::IVRSystem* sys, std::uint32_t idx) {
  char buf[256] = {0};
  vr::ETrackedPropertyError err = vr::TrackedProp_Success;
  sys->GetStringTrackedDeviceProperty(idx, vr::Prop_SerialNumber_String,
                                       buf, sizeof(buf), &err);
  return std::string(buf);
}

}  // namespace

ViveTracker::ViveTracker() = default;

ViveTracker::~ViveTracker() { shutdown(); }

bool ViveTracker::init(const Config& cfg) {
  if (open_.load()) return true;

  vr::EVRInitError err = vr::VRInitError_None;
  system_ = vr::VR_Init(&err, vr::VRApplication_Background);
  if (err != vr::VRInitError_None || system_ == nullptr) {
    std::fprintf(stderr,
        "[ViveTracker] VR_Init failed: %s\n",
        vr::VR_GetVRInitErrorAsEnglishDescription(err));
    system_ = nullptr;
    return false;
  }

  // Find target tracker. Poll for up to init_timeout_sec since the
  // device may take a moment to show up after SteamVR start.
  using clk = std::chrono::steady_clock;
  const auto t_deadline =
      clk::now() + std::chrono::duration<double>(cfg.init_timeout_sec);

  while (clk::now() < t_deadline) {
    for (std::uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
      if (!system_->IsTrackedDeviceConnected(i)) continue;
      auto cls = system_->GetTrackedDeviceClass(i);
      if (cls != vr::TrackedDeviceClass_GenericTracker) continue;
      const std::string sn = device_serial(system_, i);
      if (!cfg.target_serial.empty() && sn != cfg.target_serial) continue;
      device_idx_ = i;
      serial_     = sn;
      open_.store(true);
      std::fprintf(stderr,
          "[ViveTracker] using tracker idx=%u serial=%s\n",
          device_idx_, serial_.c_str());
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  std::fprintf(stderr,
      "[ViveTracker] no tracker found within %.1fs (target_serial='%s')\n",
      cfg.init_timeout_sec, cfg.target_serial.c_str());
  vr::VR_Shutdown();
  system_ = nullptr;
  return false;
}

void ViveTracker::shutdown() {
  if (open_.exchange(false)) {
    vr::VR_Shutdown();
    system_ = nullptr;
  }
}

bool ViveTracker::poll(Eigen::Matrix4d& out_T) {
  if (!open_.load() || system_ == nullptr) return false;

  vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
  system_->GetDeviceToAbsoluteTrackingPose(
      vr::TrackingUniverseStanding, 0.0f, poses,
      vr::k_unMaxTrackedDeviceCount);

  const auto& p = poses[device_idx_];
  if (!p.bDeviceIsConnected || !p.bPoseIsValid ||
      p.eTrackingResult != vr::TrackingResult_Running_OK) {
    return false;
  }
  out_T = to_eigen(p.mDeviceToAbsoluteTracking);
  return true;
}

}  // namespace ur10e_teleop_unilateral_vive_cpp
