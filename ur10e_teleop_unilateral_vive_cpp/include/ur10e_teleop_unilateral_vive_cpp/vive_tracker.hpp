// vive_tracker.hpp — OpenVR wrapper for a single Vive tracker.
//
// Talks to SteamVR via the OpenVR runtime. Polls one specific tracker
// (selected by serial number from config, or by "first generic-tracker
// device found") and returns its current 4×4 pose in the SteamVR
// tracking space (Y-up, right-handed, in meters).
//
// The conversion from SteamVR's frame to the UR base frame is handled
// elsewhere by Calibration — this module just delivers raw tracker
// pose.

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Dense>

// Forward-declare the OpenVR system pointer so callers don't have to
// pull in openvr.h. The actual type is vr::IVRSystem*.
namespace vr { class IVRSystem; }

namespace ur10e_teleop_unilateral_vive_cpp {

class ViveTracker {
 public:
  struct Config {
    // If empty, picks the first device with class GenericTracker.
    // Otherwise matches against the device's serial number.
    std::string target_serial;
    // Soft timeout for waiting at init() time.
    double init_timeout_sec = 10.0;
  };

  ViveTracker();
  ~ViveTracker();

  // Initialize the OpenVR runtime and locate the target tracker.
  // Returns false on any failure (no runtime, no tracker, timeout).
  bool init(const Config& cfg);

  // Tear down the OpenVR session. Idempotent.
  void shutdown();

  // Poll the latest pose. Returns false if the tracker is not yet
  // tracking (occluded / outside the base-station volume / etc.).
  //   out_T : 4×4 pose in SteamVR tracking space (Y-up, meters)
  bool poll(Eigen::Matrix4d& out_T);

  // Serial of the currently-tracked device (after init()).
  const std::string& serial() const { return serial_; }

  bool is_open() const { return open_.load(); }

 private:
  vr::IVRSystem*       system_ = nullptr;   // OpenVR system handle
  std::uint32_t        device_idx_ = 0;     // tracker's device index
  std::string          serial_;
  std::atomic<bool>    open_{false};
};

}  // namespace ur10e_teleop_unilateral_vive_cpp
