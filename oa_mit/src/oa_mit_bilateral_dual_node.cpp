// oa_mit_bilateral_dual_node.cpp — M1 single-process bilateral (one PC, both
// arms on direct CAN, IN-MEMORY state exchange — NO ROS topics in the loop).
//
// This is the architecture the working references use (enactic openarm_teleop;
// ur10e_teleop/bilateral_control): one process owns leader+follower, exchanges
// state in memory (~µs, no delay loop), runs the convergent symmetric
// position-position law. We deliver it via the Robstride MOTOR-SIDE MIT loop
// (our asset): each arm is servoed toward the peer's position by its own motor
// loop (delay-free, passive), plus gravity feedforward.
//
//   per arm:  MIT{ kp, kd, pos = couple_sign*q_peer_motor,
//                  vel = couple_sign*dq_peer_motor, tau = gravity FF }
//
// Free space: each tracks the other -> light. Contact: peer lags -> spring ->
// the operator feels resistance (true bilateral force feedback). Asymmetric
// gains recommended: follower stiff (kp~50, does the tracking), leader soft
// (kp~10-20, light + reflects force). couple_sign default -1 matches the
// proven oa_mit relay convention (follower_motor = -leader_motor); VERIFY on HW
// with a TINY kp first (pull-together = correct; push-apart = flip the sign).
//
// Gravity / friction code reused from oa_mit (KDL Dynamics). NO DOB, NO energy
// tank (those failed for the operator on UR10e). Derived from openarmx
// teleop_bimanual gravity-comp node (CC-BY-NC-SA-4.0).

#include <algorithm>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <openarmx/can/socket/openarmx.hpp>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>

#include "dynamics.hpp"

using namespace std::chrono_literals;

namespace {
const std::vector<openarmx::robstride_motor::MotorType> ARM_MOTOR_TYPES = {
    openarmx::robstride_motor::MotorType::RS04, openarmx::robstride_motor::MotorType::RS04,
    openarmx::robstride_motor::MotorType::RS03, openarmx::robstride_motor::MotorType::RS03,
    openarmx::robstride_motor::MotorType::RS00, openarmx::robstride_motor::MotorType::RS00,
    openarmx::robstride_motor::MotorType::RS00};
const std::vector<uint32_t> ARM_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
}  // namespace

// One arm: its CAN bus, KDL gravity model, state, and per-arm params.
struct Arm {
    std::unique_ptr<openarmx::can::socket::OpenArmX> bus;
    std::unique_ptr<Dynamics> dyn;
    std::string name;
    std::vector<double> dir;          // motor dir signs (joint = dir*motor)
    std::vector<double> tau_limit;    // FF torque clamp [Nm], joint frame
    double kp = 0.0, kd = 1.0;        // motor-side coupling MIT gains
    double g_scale = 0.9;
    size_t nj = 7;
    // live state
    std::vector<double> qm, dqm;      // raw motor pos/vel
    std::vector<double> q, dq;        // joint frame (= dir*motor)
    std::vector<double> tau_g;        // gravity (joint frame)

    bool init(const std::string& can, const std::string& urdf,
              const std::string& root, const std::string& leaf,
              double gx, double gy, double gz, rclcpp::Logger log) {
        dyn = std::make_unique<Dynamics>(urdf, root, leaf);
        if (!dyn->Init()) { RCLCPP_FATAL(log, "[%s] KDL init failed", name.c_str()); return false; }
        dyn->SetGravityVector(gx, gy, gz);
        bus = std::make_unique<openarmx::can::socket::OpenArmX>(can, false);
        bus->init_arm_motors(ARM_MOTOR_TYPES, ARM_IDS, ARM_IDS);
        bus->init_gripper_motor(openarmx::robstride_motor::MotorType::RS00, 0x08, 0x08);
        bus->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);
        if (!bus->enable_all()) { RCLCPP_FATAL(log, "[%s] enable failed", name.c_str()); return false; }
        std::this_thread::sleep_for(50ms);
        bus->recv_all(1000);
        std::this_thread::sleep_for(50ms);
        bus->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);
        // relax gripper (kp=kd=0, hold pos)
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

    // Send MIT: track peer (motor frame) + gravity FF. couple_on=false -> kp=kd=0
    // (weightless gravity float, safe).
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
                p.kp = kp; p.kd = kd;
                p.position = couple_sign * (i < peer_qm.size() ? peer_qm[i] : 0.0);
                p.velocity = couple_sign * (i < peer_dqm.size() ? peer_dqm[i] : 0.0);
            } else { p.kp = 0; p.kd = 0; p.position = 0; p.velocity = 0; }
            p.torque = tau_motor;
            cmds.push_back(p);
        }
        bus->get_arm().send_motion_control_commands(cmds);
    }

    void relax_disable(rclcpp::Logger) {
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

class BilateralDual : public rclcpp::Node {
public:
    BilateralDual() : Node("oa_mit_bilateral_dual") {
        declare_parameter<std::string>("arm_side", "right_arm");
        declare_parameter<std::string>("leader_can", "can0");
        declare_parameter<std::string>("follower_can", "can2");
        declare_parameter<std::string>("urdf_path", "/tmp/v10_bimanual.urdf");
        declare_parameter<int>("control_rate_hz", 250);
        declare_parameter<double>("g_scale", 0.9);
        // gdir derived from arm_side (right=(0,-9.81,0), left=(0,+9.81,0)) per the
        // openarmx mount convention; override via gx/gy/gz if needed.
        declare_parameter<double>("gy_override", 0.0);  // 0 => auto from arm_side
        // asymmetric coupling (motor-side MIT): follower stiff, leader soft.
        declare_parameter<double>("leader_kp", 0.0);   // 0 => weightless parity; raise (10-20)
        declare_parameter<double>("leader_kd", 1.0);
        declare_parameter<double>("follower_kp", 0.0); // 0 => weightless parity; raise (30-50)
        declare_parameter<double>("follower_kd", 1.5);
        declare_parameter<double>("couple_sign", -1.0);  // relay convention; verify on HW
        declare_parameter<bool>("verbose", false);
        // select which arm(s) of the pair to bring up. Run one alone (e.g.
        // enable_follower:=false) to test a single arm's gravity comp safely.
        declare_parameter<bool>("enable_leader", true);
        declare_parameter<bool>("enable_follower", true);

        side_ = get_parameter("arm_side").as_string();
        std::string lcan = get_parameter("leader_can").as_string();
        std::string fcan = get_parameter("follower_can").as_string();
        std::string urdf = get_parameter("urdf_path").as_string();
        int rate = get_parameter("control_rate_hz").as_int();
        double gscale = get_parameter("g_scale").as_double();
        double gx = 0.0, gz = 0.0;
        double gy = (side_ == "left_arm") ? 9.81 : -9.81;   // mount convention
        double gy_ovr = get_parameter("gy_override").as_double();
        if (gy_ovr != 0.0) gy = gy_ovr;
        couple_sign_ = get_parameter("couple_sign").as_double();
        verbose_ = get_parameter("verbose").as_bool();

        std::string root = (side_ == "left_arm") ? "openarmx_left_link0" : "openarmx_right_link0";
        std::string leaf = (side_ == "left_arm") ? "openarmx_left_link7" : "openarmx_right_link7";

        const std::vector<double> tau_lim = {10, 10, 5, 5, 2, 2, 2};
        leader_.name = "leader";   leader_.g_scale = gscale;  leader_.tau_limit = tau_lim;
        leader_.kp = get_parameter("leader_kp").as_double();
        leader_.kd = get_parameter("leader_kd").as_double();
        follower_.name = "follower"; follower_.g_scale = gscale; follower_.tau_limit = tau_lim;
        follower_.kp = get_parameter("follower_kp").as_double();
        follower_.kd = get_parameter("follower_kd").as_double();

        RCLCPP_INFO(get_logger(), "init leader=%s follower=%s side=%s rate=%d gdir=[%.1f %.1f %.1f]",
                    lcan.c_str(), fcan.c_str(), side_.c_str(), rate, gx, gy, gz);
        RCLCPP_INFO(get_logger(), "leader kp=%.1f kd=%.1f | follower kp=%.1f kd=%.1f | couple_sign=%.0f",
                    leader_.kp, leader_.kd, follower_.kp, follower_.kd, couple_sign_);
        en_leader_   = get_parameter("enable_leader").as_bool();
        en_follower_ = get_parameter("enable_follower").as_bool();
        if (!en_leader_ && !en_follower_)
            throw std::runtime_error("both arms disabled — nothing to do");
        RCLCPP_INFO(get_logger(), "enabled: leader=%s follower=%s",
                    en_leader_ ? "yes" : "no", en_follower_ ? "yes" : "no");
        if (en_leader_ && !leader_.init(lcan, urdf, root, leaf, gx, gy, gz, get_logger()))
            throw std::runtime_error("leader init failed");
        if (en_follower_ && !follower_.init(fcan, urdf, root, leaf, gx, gy, gz, get_logger()))
            throw std::runtime_error("follower init failed");

        auto period = std::chrono::microseconds(1000000 / std::max(1, rate));
        timer_ = create_wall_timer(period, std::bind(&BilateralDual::loop, this));
        RCLCPP_INFO(get_logger(), "bilateral dual started (couple on when kp>0)");
    }

    ~BilateralDual() override {
        leader_.relax_disable(get_logger());
        follower_.relax_disable(get_logger());
    }

private:
    void loop() {
        // read enabled arm(s) fresh this cycle (in-memory, no topic)
        if (en_leader_)   { leader_.read();   leader_.gravity(); }
        if (en_follower_) { follower_.read(); follower_.gravity(); }
        // symmetric coupling: each tracks the other — but only if the PEER is
        // enabled (single-arm mode => no peer => weightless gravity float).
        if (en_leader_)
            leader_.command(en_follower_ && leader_.kp > 0.0, couple_sign_,
                            follower_.qm, follower_.dqm);
        if (en_follower_)
            follower_.command(en_leader_ && follower_.kp > 0.0, couple_sign_,
                              leader_.qm, leader_.dqm);
        if (verbose_ && (++tick_ % 250) == 0 && en_leader_ && en_follower_) {
            RCLCPP_INFO(get_logger(), "q_L0=%.3f q_F0=%.3f  err0=%.3f",
                        leader_.q[0], follower_.q[0], leader_.q[0] - follower_.q[0]);
        }
    }

    Arm leader_, follower_;
    std::string side_;
    double couple_sign_ = -1.0;
    bool verbose_ = false;
    bool en_leader_ = true, en_follower_ = true;
    long tick_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<BilateralDual>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("oa_mit_bilateral_dual"), "fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
