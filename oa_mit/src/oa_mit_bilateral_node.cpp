// oa_mit_bilateral_node.cpp — bilateral force-feedback teleop on the openarmx
// MIT layer.  DERIVED FROM openarmx_teleop_bimanual /
// teleop_bimanual_with_gravitycomp_single.cpp (Chengdu Changshu Robot Co.,Ltd,
// CC-BY-NC-SA-4.0).  Modification (research use): the leader node also
// subscribes to the FOLLOWER's /joint_states and adds a force-reflection
// coupling torque  tau_couple = couple_kp*(q_follower - q_leader)
// - couple_kd*qd_leader  to the leader's MIT torque.
//   couple_kp = 0  -> identical to the proven Mode-2 weightless float (safe
//                     default; raise gradually to add force feedback).
// In free space the follower tracks the leader (q_follower≈q_leader) so the
// coupling ~0 (stays weightless); on contact the follower lags -> the leader
// feels resistance.  Sign verified: follower /joint_states maps to the leader
// joint frame as q_follower≈q_leader, so (q_follower - q_leader) opposes the
// leader when the follower is blocked (passive / negative feedback).
//
// Original license header (retained):
// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. https://www.openarmx.com
// CC BY-NC-SA 4.0  — http://creativecommons.org/licenses/by-nc-sa/4.0/
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.


#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <csignal>
#include <thread>
#include <limits>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <openarmx/can/socket/openarmx.hpp>
#include <openarmx/robstride_motor/rs_motor_constants.hpp>

#include "dynamics.hpp"

using namespace std::chrono_literals;

class TeleopLeaderWithGravitySingle : public rclcpp::Node
{
public:
    TeleopLeaderWithGravitySingle()
    : Node("teleop_bimanual_with_gravity_single")
    {
        // Core params
        this->declare_parameter<std::string>("arm_side", "right_arm");
        this->declare_parameter<std::string>("leader_can", "can0");
        this->declare_parameter<std::string>("leader_urdf_path", "/tmp/v10_bimanual.urdf");
        this->declare_parameter<std::string>("follower_prefix", "right");
        this->declare_parameter<int>("control_rate_hz", 200);

        // Gravity params
        this->declare_parameter<bool>("verbose", false);
        this->declare_parameter<double>("g_scale", 0.9);
        this->declare_parameter<double>("kd_damp", 0.0);
        this->declare_parameter<double>("kp_hold", 0.0);
        this->declare_parameter<double>("vel_hold_thresh", 0.02);
        this->declare_parameter<int>("hold_settle_ms", 300);
        this->declare_parameter<std::vector<double>>("gdir", {0.0, -9.81, 0.0});

        // Bilateral force-reflection coupling (NEW): leader feels follower lag.
        // couple_kp=0 -> pure Mode-2 weightless (safe default). Raise to add FF.
        this->declare_parameter<double>("couple_kp", 0.0);
        this->declare_parameter<double>("couple_kd", 0.0);
        this->declare_parameter<double>("couple_tau_limit", 8.0);  // per-joint |Nm| on the coupling term
        // EMA on the (slow, ~74Hz, stale) follower position before coupling, to
        // smooth its stair-steps that excite the delayed-loop vibration. 1.0 =
        // no filter; lower = smoother but laggier FF. e.g. 0.3-0.5.
        this->declare_parameter<double>("couple_lpf", 1.0);

        arm_side_       = this->get_parameter("arm_side").as_string();
        std::string can = this->get_parameter("leader_can").as_string();
        std::string urdf_path = this->get_parameter("leader_urdf_path").as_string();
        follower_prefix_ = this->get_parameter("follower_prefix").as_string();
        int control_rate_hz = this->get_parameter("control_rate_hz").as_int();

        g_scale_         = this->get_parameter("g_scale").as_double();
        kd_damp_         = this->get_parameter("kd_damp").as_double();
        kp_hold_         = this->get_parameter("kp_hold").as_double();
        vel_hold_thresh_ = this->get_parameter("vel_hold_thresh").as_double();
        hold_settle_ms_  = this->get_parameter("hold_settle_ms").as_int();
        verbose_        = this->get_parameter("verbose").as_bool();
        auto gdir_vec    = this->get_parameter("gdir").as_double_array();
        if (gdir_vec.size() == 3) {
            gx_ = gdir_vec[0]; gy_ = gdir_vec[1]; gz_ = gdir_vec[2];
        }
        couple_kp_        = this->get_parameter("couple_kp").as_double();
        couple_kd_        = this->get_parameter("couple_kd").as_double();
        couple_tau_limit_ = this->get_parameter("couple_tau_limit").as_double();
        couple_lpf_       = std::clamp(this->get_parameter("couple_lpf").as_double(), 0.0, 1.0);

        RCLCPP_INFO(get_logger(), "=== Teleop Leader Single + GravityComp 初始化 ===");
        RCLCPP_INFO(get_logger(), "arm_side=%s, can=%s, urdf=%s", arm_side_.c_str(), can.c_str(), urdf_path.c_str());
        RCLCPP_INFO(get_logger(), "g_scale=%.3f, kd=%.3f, kp=%.3f", g_scale_, kd_damp_, kp_hold_);
        RCLCPP_INFO(get_logger(), "gdir=[%.3f, %.3f, %.3f]", gx_, gy_, gz_);

        // Decide KDL chain endpoints based on arm side (match gravity_compasation_main.cpp)
        std::string root_link, leaf_link;
        if (arm_side_ == "right_arm") {
            root_link = "openarmx_right_link0";
            leaf_link = "openarmx_right_link7";
        } else if (arm_side_ == "left_arm") {
            root_link = "openarmx_left_link0";
            leaf_link = "openarmx_left_link7";
        } else {
            RCLCPP_FATAL(get_logger(), "arm_side 必须是 right_arm 或 left_arm, 当前: %s", arm_side_.c_str());
            throw std::runtime_error("invalid arm_side");
        }

        // Init dynamics
        auto dyn = std::make_unique<Dynamics>(urdf_path, root_link, leaf_link);
        if (!dyn || !dyn->Init()) {
            RCLCPP_FATAL(get_logger(), "KDL 动力学初始化失败");
            throw std::runtime_error("failed to init dynamics");
        }
        arm_dyn_ = std::move(dyn);
        arm_dyn_->SetGravityVector(gx_, gy_, gz_);

        // Torque limits (same as gravity_comp)
        tau_limits_ = {10.0, 10.0, 5.0, 5.0, 2.0, 2.0, 2.0};

        // Direction signs (default --dir-all -1)
        dir_signs_ = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
        joint_offsets_.assign(7, 0.0);

        // Init CAN/OpenArmX
        try {
            RCLCPP_INFO(get_logger(), "初始化主动端单臂 OpenArmX: %s", can.c_str());
            arm_bus_ = std::make_unique<openarmx::can::socket::OpenArmX>(can, false);

            std::vector<openarmx::robstride_motor::MotorType> motor_types = {
                openarmx::robstride_motor::MotorType::RS04,
                openarmx::robstride_motor::MotorType::RS04,
                openarmx::robstride_motor::MotorType::RS03,
                openarmx::robstride_motor::MotorType::RS03,
                openarmx::robstride_motor::MotorType::RS00,
                openarmx::robstride_motor::MotorType::RS00,
                openarmx::robstride_motor::MotorType::RS00};
            std::vector<uint32_t> ids = {0x01,0x02,0x03,0x04,0x05,0x06,0x07};
            arm_bus_->init_arm_motors(motor_types, ids, ids);

            arm_bus_->init_gripper_motor(openarmx::robstride_motor::MotorType::RS00, 0x08, 0x08);
            arm_bus_->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);

            RCLCPP_INFO(get_logger(), "使能该臂的所有电机...");
            if (!arm_bus_->enable_all()) {
                RCLCPP_ERROR(get_logger(), "部分或全部电机使能失败");
                throw std::runtime_error("Failed to enable all motors");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            arm_bus_->recv_all(1000);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            arm_bus_->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);

            // 初始化夹爪为 kp=kd=0
            std::vector<openarmx::robstride_motor::MotionControlParam> gripper_cmds;
            for (auto* motor : arm_bus_->get_gripper().get_motors()) {
                openarmx::robstride_motor::MotionControlParam p;
                p.position = motor->get_position();
                p.velocity = 0.0;
                p.torque   = 0.0;
                p.kp       = 0.0;
                p.kd       = 0.0;
                gripper_cmds.push_back(p);
            }
            arm_bus_->get_gripper().send_motion_control_commands(gripper_cmds);

        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "初始化单臂 OpenArmX 失败: %s", e.what());
            throw;
        }

        // Allocate state
        size_t nj = arm_bus_->get_arm().get_motors().size();
        q_.assign(nj, 0.0);
        qd_.assign(nj, 0.0);
        tau_g_.assign(nj, 0.0);
        tau_damp_.assign(nj, 0.0);
        tau_hold_.assign(nj, 0.0);
        tau_meas_.assign(nj, 0.0);
        hold_target_.assign(nj, 0.0);
        tau_couple_.assign(nj, 0.0);

        start_time_      = std::chrono::high_resolution_clock::now();
        last_hz_display_ = start_time_;

        // Publisher to follower
        std::string topic = "/" + follower_prefix_ + "_forward_position_controller/commands";
        follower_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
        RCLCPP_INFO(get_logger(), "从动端话题: %s", topic.c_str());

        // Bilateral FF: subscribe to follower joint states for q_follower.
        // Follower arm joint names = "openarmx_<prefix>_joint1..7".
        q_follower_.assign(nj, 0.0);
        follower_have_ = false;
        for (size_t i = 0; i < 7; ++i)
            follower_joint_names_.push_back(
                "openarmx_" + follower_prefix_ + "_joint" + std::to_string(i + 1));
        follower_js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TeleopLeaderWithGravitySingle::followerJointStateCb, this,
                      std::placeholders::_1));
        RCLCPP_INFO(get_logger(),
            "bilateral coupling: couple_kp=%.2f couple_kd=%.2f limit=%.1f (kp=0 => Mode2 weightless)",
            couple_kp_, couple_kd_, couple_tau_limit_);

        // Timer
        auto period = std::chrono::microseconds(1000000 / control_rate_hz);
        timer_ = create_wall_timer(period, std::bind(&TeleopLeaderWithGravitySingle::controlLoop, this));

        RCLCPP_INFO(get_logger(), "TeleopLeaderWithGravitySingle 初始化完成");
    }

    ~TeleopLeaderWithGravitySingle()
    {
        RCLCPP_INFO(get_logger(), "关闭 TeleopLeaderWithGravitySingle...");
        if (arm_bus_) {
            try {
                // 发送 kp=kd=0 放松
                std::vector<openarmx::robstride_motor::MotionControlParam> cmds;
                for (auto* m : arm_bus_->get_arm().get_motors()) {
                    openarmx::robstride_motor::MotionControlParam p{};
                    p.position = m->get_position();
                    p.velocity = 0.0;
                    p.torque   = 0.0;
                    p.kp       = 0.0;
                    p.kd       = 0.0;
                    cmds.push_back(p);
                }
                arm_bus_->get_arm().send_motion_control_commands(cmds);
                if (!arm_bus_->disable_all()) {
                    RCLCPP_WARN(get_logger(), "部分电机禁用失败");
                }
            } catch (...) {
            }
        }
    }

private:
    void controlLoop()
    {
        try {
            sendGravityCompTorques();
            publishFollowerCommand();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "controlLoop 异常: %s", e.what());
        }
    }

    void sendGravityCompTorques()
    {
        if (!arm_bus_ || !arm_dyn_) return;

        frame_count_++;
        auto now = std::chrono::high_resolution_clock::now();
        auto ms_since_last_display =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_hz_display_).count();

        arm_bus_->refresh_all();
        arm_bus_->recv_all(500);

        auto motors = arm_bus_->get_arm().get_motors();
        const size_t nj_hw = motors.size();
        const size_t nj_kdl = arm_dyn_->NumJoints();
        const size_t nj = std::min(nj_hw, nj_kdl);

        // tau_meas (joint)
        for (size_t i = 0; i < nj; ++i) {
            double tau_motor = motors[i]->get_torque();
            double s = (i < dir_signs_.size()) ? dir_signs_[i] : 1.0;
            tau_meas_[i] = s * tau_motor;
        }

        bool all_steady = true;
        for (size_t i = 0; i < nj; ++i) {
            double mpos = motors[i]->get_position();
            double mvel = motors[i]->get_velocity();
            double s = (i < dir_signs_.size()) ? dir_signs_[i] : 1.0;
            double offs = (i < joint_offsets_.size()) ? joint_offsets_[i] : 0.0;
            double q = s * mpos + offs;
            double qd = s * mvel;
            q_[i]  = q;
            qd_[i] = qd;
            if (std::fabs(qd_[i]) > vel_hold_thresh_) all_steady = false;
        }

        if (!all_steady) {
            last_unsteady_time_ = now;
            hold_latched_ = false;
        } else {
            auto ms_since = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_unsteady_time_).count();
            if (ms_since >= hold_settle_ms_) {
                if (!hold_latched_) {
                    for (size_t i = 0; i < nj; ++i) hold_target_[i] = q_[i];
                    hold_latched_ = true;
                }
            }
        }

        arm_dyn_->GetGravity(q_.data(), tau_g_.data());

        // Bilateral force-reflection coupling (NEW). Snapshot follower q under lock.
        std::vector<double> qf(nj, 0.0);
        bool have_f;
        {
            std::lock_guard<std::mutex> lk(follower_mtx_);
            have_f = follower_have_;
            for (size_t i = 0; i < nj && i < q_follower_.size(); ++i) qf[i] = q_follower_[i];
        }

        double max_abs_tau = 0.0;
        for (size_t i = 0; i < nj; ++i) {
            tau_g_[i]    *= g_scale_;
            tau_damp_[i] = -kd_damp_ * qd_[i];
            tau_hold_[i] = hold_latched_ ? (kp_hold_ * (hold_target_[i] - q_[i])) : 0.0;
            // force feedback: pull leader toward follower; ~0 in free space,
            // resists when follower lags (contact). Off until follower data seen.
            if (have_f && couple_kp_ > 0.0) {
                double tc = couple_kp_ * (qf[i] - q_[i]) - couple_kd_ * qd_[i];
                tc = std::clamp(tc, -couple_tau_limit_, couple_tau_limit_);
                tau_couple_[i] = tc;
            } else {
                tau_couple_[i] = 0.0;
            }
            max_abs_tau = std::max(max_abs_tau,
                std::fabs(tau_g_[i] + tau_damp_[i] + tau_hold_[i] + tau_couple_[i]));
        }

        std::vector<openarmx::robstride_motor::MotionControlParam> cmds;
        cmds.reserve(nj);
        for (size_t i = 0; i < nj; ++i) {
            double tau_joint = tau_g_[i] + tau_damp_[i] + tau_hold_[i] + tau_couple_[i];
            double s = (i < dir_signs_.size()) ? dir_signs_[i] : 1.0;
            double limit = (i < tau_limits_.size()) ? tau_limits_[i] : std::numeric_limits<double>::infinity();

            double tau_motor = s * tau_joint;
            if (std::isfinite(limit)) {
                double abs_tau = std::fabs(tau_motor);
                if (abs_tau > limit) {
                    double sign = (tau_motor >= 0.0) ? 1.0 : -1.0;
                    tau_motor = sign * limit;
                }
            }

            openarmx::robstride_motor::MotionControlParam p{};
            p.position = 0.0;
            p.velocity = 0.0;
            p.kp       = 0.0;
            p.kd       = 0.0;
            p.torque   = tau_motor;
            cmds.push_back(p);
        }

        arm_bus_->get_arm().send_motion_control_commands(cmds);

        if (ms_since_last_display >= 1000) {
            auto total_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
            double hz = (frame_count_ * 1000.0) / std::max(1.0, static_cast<double>(total_ms));
            std::cout << "=== Loop Frequency: " << hz << " Hz ===" << std::endl;
        }

        if (verbose_ && ms_since_last_display >= 1000) {
            std::cout << "  |max| torque cmd: " << max_abs_tau
                      << ", hold_latched=" << (hold_latched_ ? "yes" : "no")
                      << std::endl;

            std::cout << std::fixed << std::setprecision(3);
            for (size_t i = 0; i < nj; ++i) {
                double tau_cmd = tau_g_[i] + tau_damp_[i] + tau_hold_[i];
                double tau_err = tau_cmd - tau_meas_[i];
                std::cout << "    j" << i
                          << ": q=" << q_[i]
                          << ", qd=" << qd_[i]
                          << ", g=" << tau_g_[i]
                          << ", d=" << tau_damp_[i]
                          << ", h=" << tau_hold_[i]
                          << ", cmd=" << tau_cmd
                          << ", tau_meas=" << tau_meas_[i]
                          << ", err=" << tau_err
                          << std::endl;
            }
            std::cout.unsetf(std::ios::floatfield);

            last_hz_display_ = now;
        }
    }

    void publishFollowerCommand()
    {
        if (!arm_bus_ || !follower_pub_) return;

        // 7 关节 + 1 夹爪
        std::vector<double> data;
        data.reserve(8);

        try {
            arm_bus_->refresh_all();
            arm_bus_->recv_all(1500);

            for (auto* m : arm_bus_->get_arm().get_motors()) {
                data.push_back(-m->get_position()); // 从动臂角度取反
            }

            for (auto* m : arm_bus_->get_gripper().get_motors()) {
                double motor_rad = m->get_position();
                double joint_m = 0.044 * (motor_rad / 1.0472); // 与 hardware 中一致
                data.push_back(joint_m);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "读取主臂位置失败: %s", e.what());
            return;
        }

        if (data.size() == 8) {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = data;
            follower_pub_->publish(msg);
        }
    }

    // Bilateral FF: cache follower joint positions, mapped into the leader joint
    // frame. q_follower≈q_leader in free space (verified), so the coupling term
    // (q_follower - q_leader) is ~0 when tracking and grows when the follower
    // lags (contact) -> reflected to the leader as resistance.
    void followerJointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(follower_mtx_);
        for (size_t j = 0; j < follower_joint_names_.size() && j < q_follower_.size(); ++j) {
            for (size_t k = 0; k < msg->name.size(); ++k) {
                if (msg->name[k] == follower_joint_names_[j]) {
                    if (k < msg->position.size()) {
                        double v = msg->position[k];
                        // EMA to smooth the ~74Hz stale steps (anti-vibration)
                        q_follower_[j] = follower_have_
                            ? (couple_lpf_ * v + (1.0 - couple_lpf_) * q_follower_[j])
                            : v;
                    }
                    break;
                }
            }
        }
        follower_have_ = true;
    }

    // Node state
    std::string arm_side_;
    std::string follower_prefix_;

    std::unique_ptr<openarmx::can::socket::OpenArmX> arm_bus_;
    std::unique_ptr<Dynamics> arm_dyn_;

    std::vector<double> tau_limits_;
    std::vector<double> dir_signs_;
    std::vector<double> joint_offsets_;

    double g_scale_ = 0.9;
    double kd_damp_ = 0.0;
    double kp_hold_ = 0.0;
    double vel_hold_thresh_ = 0.02;
    bool verbose_ = false;
    int hold_settle_ms_ = 300;
    double gx_ = 0.0, gy_ = -9.81, gz_ = 0.0;

    std::vector<double> q_;
    std::vector<double> qd_;
    std::vector<double> tau_g_;
    std::vector<double> tau_damp_;
    std::vector<double> tau_hold_;
    std::vector<double> tau_meas_;
    std::vector<double> hold_target_;
    bool hold_latched_ = false;

    // Bilateral force-reflection coupling
    double couple_kp_ = 0.0;
    double couple_kd_ = 0.0;
    double couple_tau_limit_ = 8.0;
    double couple_lpf_ = 1.0;
    std::vector<double> tau_couple_;
    std::vector<double> q_follower_;
    std::vector<std::string> follower_joint_names_;
    bool follower_have_ = false;
    std::mutex follower_mtx_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_js_sub_;

    std::chrono::high_resolution_clock::time_point last_unsteady_time_{};
    std::chrono::high_resolution_clock::time_point start_time_{};
    std::chrono::high_resolution_clock::time_point last_hz_display_{};
    int frame_count_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr follower_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

static std::shared_ptr<TeleopLeaderWithGravitySingle> g_node;

void signal_handler(int)
{
    if (g_node) {
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);

    try {
        g_node = std::make_shared<TeleopLeaderWithGravitySingle>();
        rclcpp::spin(g_node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("teleop_bimanual_with_gravity_single"),
                     "致命错误: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
