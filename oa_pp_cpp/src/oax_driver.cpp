// oax_driver.cpp — openarmx CAN wrapper implementation. See oax_driver.hpp.

#include "oa_pp_cpp/oax_driver.hpp"

#include <cstdio>
#include <vector>

#include <openarmx/can/socket/openarmx.hpp>
#include <openarmx/robstride_motor/rs_motor.hpp>
#include <openarmx/robstride_motor/rs_motor_control.hpp>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>

namespace oa_pp {

namespace rm = openarmx::robstride_motor;

OaxArm::OaxArm(std::string can_iface, bool fd, int recv_timeout_us)
    : iface_(std::move(can_iface)), fd_(fd), recv_timeout_us_(recv_timeout_us) {
  dir_.fill(-1.0);   // matches openarmx_hardware get_motor_direction_multipliers()
}

OaxArm::~OaxArm() {
  if (dev_) { try { disable(); } catch (...) {} }
}

bool OaxArm::init() {
  try {
    dev_ = std::make_unique<openarmx::can::socket::OpenArmX>(iface_, fd_);
    // Joints 1..7 -> motor types per openarmx V10 mapping.
    const std::vector<rm::MotorType> types = {
        rm::MotorType::RS04, rm::MotorType::RS04,   // joints 1-2
        rm::MotorType::RS03, rm::MotorType::RS03,   // joints 3-4
        rm::MotorType::RS00, rm::MotorType::RS00, rm::MotorType::RS00};  // 5-7
    const std::vector<uint32_t> send_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    const std::vector<uint32_t> recv_ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    dev_->init_arm_motors(types, send_ids, recv_ids);
    inited_ = true;
    return true;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[oax_driver:%s] init failed: %s\n", iface_.c_str(), e.what());
    dev_.reset();
    return false;
  }
}

bool OaxArm::enable() {
  if (!inited_) return false;
  dev_->set_callback_mode_all(rm::CallbackMode::STATE);
  if (!dev_->enable_all()) {
    std::fprintf(stderr, "[oax_driver:%s] enable_all() reported failure\n", iface_.c_str());
    return false;
  }
  return true;
}

void OaxArm::disable() {
  if (dev_) dev_->disable_all();
}

void OaxArm::read(Vec7& q, Vec7& qd, Vec7& tau) {
  if (!dev_) return;
  dev_->refresh_all();
  dev_->recv_all(recv_timeout_us_);
  auto motors = dev_->get_arm().get_motors();   // std::vector<Motor*>
  const int n = std::min<int>(DOF, static_cast<int>(motors.size()));
  for (int i = 0; i < n; ++i) {
    q[i]   = motors[i]->get_position() * dir_[i];
    qd[i]  = motors[i]->get_velocity() * dir_[i];
    tau[i] = motors[i]->get_torque()   * dir_[i];
  }
}

void OaxArm::write_torque(const Vec7& tau) {
  Vec7 zero{};
  write_mit(zero, zero, zero, zero, tau);
}

void OaxArm::write_mit(const Vec7& kp, const Vec7& kd,
                       const Vec7& pos, const Vec7& vel, const Vec7& tau) {
  if (!dev_) return;
  std::vector<rm::MotionControlParam> params(DOF);
  for (int i = 0; i < DOF; ++i) {
    rm::MotionControlParam p;
    p.kp       = kp[i];
    p.kd       = kd[i];
    p.position = pos[i] * dir_[i];
    p.velocity = vel[i] * dir_[i];
    p.torque   = tau[i] * dir_[i];
    params[i] = p;
  }
  dev_->get_arm().send_motion_control_commands(params);
}

}  // namespace oa_pp
