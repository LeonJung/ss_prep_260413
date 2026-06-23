// oa_mit_arm.hpp — one OpenArm arm on direct openarmx_can MIT: read state,
// KDL gravity, send MIT (per-joint coupling kp/kd toward a peer + gravity FF).
// Shared by the single-loop and the per-arm-threaded dual bilateral nodes.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <openarmx/can/socket/openarmx.hpp>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>

#include "dynamics.hpp"

// One arm: its CAN bus, KDL gravity model, state, and per-arm params.
struct Arm {
    std::unique_ptr<openarmx::can::socket::OpenArmX> bus;
    std::unique_ptr<Dynamics> dyn;
    std::string name;
    std::vector<double> dir;             // motor dir signs (joint = dir*motor)
    std::vector<double> tau_limit;       // FF torque clamp [Nm], joint frame
    std::vector<double> kp_vec, kd_vec;  // PER-JOINT motor-side MIT gains
    double gain = 0.0;                   // coupling on/off + scale (0 => weightless)
    bool vel_ff = false;                 // inject peer velocity? (off = smooth)
    double g_scale = 0.9;
    size_t nj = 7;
    // live state
    std::vector<double> qm, dqm;         // raw motor pos/vel
    std::vector<double> q, dq;           // joint frame (= dir*motor)
    std::vector<double> tau_g;           // gravity (joint frame)

    bool init(const std::string& can, const std::string& urdf,
              const std::string& root, const std::string& leaf,
              double gx, double gy, double gz, rclcpp::Logger log) {
        dyn = std::make_unique<Dynamics>(urdf, root, leaf);
        if (!dyn->Init()) { RCLCPP_FATAL(log, "[%s] KDL init failed", name.c_str()); return false; }
        dyn->SetGravityVector(gx, gy, gz);
        std::vector<openarmx::robstride_motor::MotorType> motor_types = {
            openarmx::robstride_motor::MotorType::RS04, openarmx::robstride_motor::MotorType::RS04,
            openarmx::robstride_motor::MotorType::RS03, openarmx::robstride_motor::MotorType::RS03,
            openarmx::robstride_motor::MotorType::RS00, openarmx::robstride_motor::MotorType::RS00,
            openarmx::robstride_motor::MotorType::RS00};
        std::vector<uint32_t> ids = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        bus = std::make_unique<openarmx::can::socket::OpenArmX>(can, false);
        bus->init_arm_motors(motor_types, ids, ids);
        bus->init_gripper_motor(openarmx::robstride_motor::MotorType::RS00, 0x08, 0x08);
        bus->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);
        if (!bus->enable_all()) { RCLCPP_FATAL(log, "[%s] enable failed", name.c_str()); return false; }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bus->recv_all(1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bus->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);
        std::vector<openarmx::robstride_motor::MotionControlParam> gc;
        for (auto* m : bus->get_gripper().get_motors()) {
            openarmx::robstride_motor::MotionControlParam p{};
            p.position = m->get_position(); p.velocity = 0; p.torque = 0; p.kp = 0; p.kd = 0;
            gc.push_back(p);
        }
        bus->get_gripper().send_motion_control_commands(gc);
        nj = bus->get_arm().get_motors().size();
        if (dir.size() != nj) dir.assign(nj, -1.0);
        qm.assign(nj, 0.0); dqm.assign(nj, 0.0);
        q.assign(nj, 0.0);  dq.assign(nj, 0.0);  tau_g.assign(nj, 0.0);
        return true;
    }

    void read() {
        bus->refresh_all();
        bus->recv_all(500);
        auto motors = bus->get_arm().get_motors();
        size_t n = std::min(nj, motors.size());
        for (size_t i = 0; i < n; ++i) {
            qm[i] = motors[i]->get_position();
            dqm[i] = motors[i]->get_velocity();
            q[i] = dir[i] * qm[i];
            dq[i] = dir[i] * dqm[i];
        }
    }

    void gravity() { dyn->GetGravity(q.data(), tau_g.data()); }

    // MIT: track peer (motor frame) + gravity FF. couple_on=false -> weightless.
    void command(bool couple_on, double couple_sign,
                 const std::vector<double>& peer_qm, const std::vector<double>& peer_dqm) {
        std::vector<openarmx::robstride_motor::MotionControlParam> cmds;
        cmds.reserve(nj);
        for (size_t i = 0; i < nj; ++i) {
            double tau_ff_joint = g_scale * tau_g[i];
            double s = dir[i];
            double lim = (i < tau_limit.size()) ? tau_limit[i] : 1e9;
            double tau_motor = std::clamp(s * tau_ff_joint, -lim, lim);
            openarmx::robstride_motor::MotionControlParam p{};
            if (couple_on) {
                p.kp = (i < kp_vec.size()) ? kp_vec[i] : 0.0;
                p.kd = (i < kd_vec.size()) ? kd_vec[i] : 0.0;
                p.position = couple_sign * (i < peer_qm.size() ? peer_qm[i] : 0.0);
                p.velocity = vel_ff ? couple_sign * (i < peer_dqm.size() ? peer_dqm[i] : 0.0) : 0.0;
            } else { p.kp = 0; p.kd = 0; p.position = 0; p.velocity = 0; }
            p.torque = tau_motor;
            cmds.push_back(p);
        }
        bus->get_arm().send_motion_control_commands(cmds);
    }

    void relax_disable() {
        if (!bus) return;
        std::vector<openarmx::robstride_motor::MotionControlParam> cmds;
        for (auto* m : bus->get_arm().get_motors()) {
            openarmx::robstride_motor::MotionControlParam p{};
            p.position = m->get_position(); p.velocity = 0; p.torque = 0; p.kp = 0; p.kd = 0;
            cmds.push_back(p);
        }
        bus->get_arm().send_motion_control_commands(cmds);
        bus->disable_all();
    }
};
