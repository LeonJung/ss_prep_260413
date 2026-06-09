// oax_driver.hpp — thin wrapper around the openarmx CAN library for ONE arm
// (one CAN interface, 7 Robstride arm motors). Mirrors how
// openarmx_hardware/v10_simple_hardware.cpp drives the arm, but as a
// standalone (non-ros2_control) object usable from a real-time loop.
//
// Ground truth: /usr/include/openarmx/** (openarmx-can_1.0.0 .deb) and
// openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp.
//
//   motors          : 7 arm joints
//   types           : RS04,RS04,RS03,RS03,RS00,RS00,RS00   (joints 1..7)
//   send/recv CAN id : 1..7
//   direction sign   : -1 on every joint (matches openarmx_hardware)
//   command          : MIT / MotionControlParam{kp,kd,pos,vel,torque}

#pragma once
#include <array>
#include <memory>
#include <string>

#include "oa_pp_cpp/config.hpp"   // DOF, Vec7

// Forward declare to keep openarmx headers out of this header's consumers.
namespace openarmx::can::socket { class OpenArmX; }

namespace oa_pp {

class OaxArm {
public:
  // can_iface e.g. "can0"; fd = classic CAN (false) vs CAN-FD (true).
  OaxArm(std::string can_iface, bool fd, int recv_timeout_us);
  ~OaxArm();

  OaxArm(const OaxArm&) = delete;
  OaxArm& operator=(const OaxArm&) = delete;

  const std::string& iface() const { return iface_; }

  // Construct the OpenArmX device + init the 7 arm motors. Must be called
  // before enable(). Returns false on failure.
  bool init();

  // Put motors in STATE-callback mode and close the control loop (enable_all).
  // NOTE: assumes motors power up in MIT / MOTION_CONTROL run mode (the
  // openarmx default). If a unit ships in CSP, a one-time run-mode switch is
  // needed — see README "MIT mode" note.
  bool enable();

  // Send 0-torque disable to all motors.
  void disable();

  // Refresh + receive, then copy per-joint state (already direction-corrected).
  void read(Vec7& q, Vec7& qd, Vec7& tau);

  // Pure-torque command: MIT with kp=kd=0, pos=vel=0, torque=tau.
  // (Position-position bilateral computes the full torque on the PC side.)
  void write_torque(const Vec7& tau);

  // Full MIT command (used if you want motor-side impedance; unused by oa_pp).
  void write_mit(const Vec7& kp, const Vec7& kd,
                 const Vec7& pos, const Vec7& vel, const Vec7& tau);

private:
  std::string iface_;
  bool        fd_;
  int         recv_timeout_us_;
  std::unique_ptr<openarmx::can::socket::OpenArmX> dev_;
  std::array<double, DOF> dir_;   // direction multiplier per joint (-1)
  bool inited_ = false;
};

}  // namespace oa_pp
