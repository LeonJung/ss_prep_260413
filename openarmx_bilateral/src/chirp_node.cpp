// chirp_node.cpp — drive ONE follower joint with a position CHIRP (sine sweep) and
// log commanded vs actual + timestamps. Repeatable input for OBJECTIVE control-loop
// transparency comparison (e.g. generic vs PreemptRT kernel): bandwidth, phase lag,
// tracking latency, loop timing jitter.
//
// Setup: follower bringup up (forward_position_controller active). NO relay / leader
// (this node owns the position command). Others held at start pose; the chosen joint
// follows  q0 + amp*sin(phase),  with instantaneous freq sweeping f0 -> f1 (linear
// chirp) over `duration` seconds. Commands are clamped to per-joint LEFT limits.
//
//   ros2 run openarmx_bilateral chirp_node --ros-args -p csv:=/tmp/chirp_generic.csv
//   (then on the RT kernel: -p csv:=/tmp/chirp_rt.csv)
//
// CSV (one row per follower joint-state sample): t, joint, freq, cmd, act, vel, eff
// Offline: H(f)=FFT(act)/FFT(cmd) -> magnitude(-3dB bandwidth)+phase(lag); cross-corr
// (cmd,act)->latency; dt std/max -> loop jitter.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ChirpNode : public rclcpp::Node {
public:
    ChirpNode() : Node("openarmx_chirp") {
        declare_parameter<std::string>("joint_prefix", "openarmx_left_joint");
        declare_parameter<int>("n_joints", 7);
        declare_parameter<int>("joint", 4);          // which joint sweeps (1..n)
        declare_parameter<double>("amp", 0.12);      // amplitude [rad]
        declare_parameter<double>("f0", 0.2);        // start freq [Hz]
        declare_parameter<double>("f1", 3.0);        // end freq [Hz]
        declare_parameter<double>("duration", 20.0); // sweep time [s]
        declare_parameter<int>("rate_hz", 200);      // command publish rate
        declare_parameter<std::string>("cmd",
            "/follower/left_forward_position_controller/commands");
        declare_parameter<std::string>("states", "/follower/joint_states");
        declare_parameter<std::string>("csv", "/tmp/chirp.csv");
        // the left_forward_position_controller is 8-DOF (7 joints + finger gripper),
        // so commands must carry 8 values (gripper held). Match the relay.
        declare_parameter<std::string>("gripper_joint", "openarmx_left_finger_joint1");
        declare_parameter<bool>("include_gripper", true);

        nj_ = get_parameter("n_joints").as_int();
        gripper_name_ = get_parameter("gripper_joint").as_string();
        include_gripper_ = get_parameter("include_gripper").as_bool();
        jx_ = get_parameter("joint").as_int() - 1;
        amp_ = get_parameter("amp").as_double();
        f0_ = get_parameter("f0").as_double();
        f1_ = get_parameter("f1").as_double();
        dur_ = get_parameter("duration").as_double();
        std::string jp = get_parameter("joint_prefix").as_string();
        for (int i = 0; i < nj_; ++i) names_.push_back(jp + std::to_string(i + 1));
        q_.assign(nj_, 0.0); q0_.assign(nj_, 0.0);
        cmd_now_ = 0.0;

        std::string path = get_parameter("csv").as_string();
        f_.open(path);
        if (f_.is_open()) f_ << "t,joint,freq,cmd,act,vel,eff\n";
        RCLCPP_INFO(get_logger(), "chirp -> %s : joint %d amp %.3f f %.2f->%.2fHz %.0fs",
                    path.c_str(), jx_ + 1, amp_, f0_, f1_, dur_);

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            get_parameter("cmd").as_string(), 10);
        st_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("states").as_string(), 50,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { onState(m); });

        int rate = std::max(1, (int)get_parameter("rate_hz").as_int());
        dt_ = 1.0 / rate;
        timer_ = create_wall_timer(std::chrono::microseconds((long)(1e6 / rate)),
                                   [this] { tick(); });
    }
    ~ChirpNode() override { if (f_.is_open()) f_.close(); }

private:
    // per-joint LEFT safety limits [rad] (friction-test-derived; clamp only)
    double lo(int j) { static const double L[7]={-2.356,-2.356,-0.785,0.0,-0.785,0.0,-1.571}; return L[j]; }
    double hi(int j) { static const double H[7]={0.785,0.0,1.571,1.745,1.571,0.785,1.396}; return H[j]; }

    void onState(const sensor_msgs::msg::JointState::SharedPtr& m) {
        for (int j = 0; j < nj_; ++j)
            for (size_t k = 0; k < m->name.size(); ++k)
                if (m->name[k] == names_[j]) {
                    if (k < m->position.size()) q_[j] = m->position[k];
                    if (j == jx_) {
                        double vel = (k < m->velocity.size()) ? m->velocity[k] : 0.0;
                        double eff = (k < m->effort.size()) ? m->effort[k] : 0.0;
                        if (started_ && f_.is_open()) {
                            char b[160];
                            std::snprintf(b, sizeof(b), "%.6f,%d,%.4f,%.6f,%.6f,%.6f,%.6f\n",
                                (now() - t0_).seconds(), jx_ + 1, freq_now_, cmd_now_,
                                m->position[k], vel, eff);
                            f_ << b;
                        }
                    }
                    break;
                }
        for (size_t k = 0; k < m->name.size(); ++k)
            if (m->name[k] == gripper_name_ && k < m->position.size()) { grip_ = m->position[k]; break; }
        have_ = true;
    }

    void tick() {
        if (!have_) return;
        if (!started_) { q0_ = q_; grip0_ = grip_; started_ = true; t0_ = now(); phase_ = 0.0; return; }
        double t = (now() - t0_).seconds();
        std_msgs::msg::Float64MultiArray msg;
        msg.data = q0_;                       // hold all joints at start
        if (t <= dur_) {
            freq_now_ = f0_ + (f1_ - f0_) * (t / dur_);    // linear chirp
            phase_ += 2.0 * M_PI * freq_now_ * dt_;
            double c = q0_[jx_] + amp_ * std::sin(phase_);
            c = std::clamp(c, lo(jx_), hi(jx_));
            cmd_now_ = c;
            msg.data[jx_] = c;
        } else {
            cmd_now_ = q0_[jx_]; msg.data[jx_] = q0_[jx_];
            if (!done_) { done_ = true; RCLCPP_INFO(get_logger(), "chirp DONE -> CSV. Ctrl+C."); }
        }
        if (include_gripper_) msg.data.push_back(grip0_);   // 8th value = gripper (held)
        pub_->publish(msg);
    }

    int nj_ = 7, jx_ = 3;
    double amp_ = 0.12, f0_ = 0.2, f1_ = 3.0, dur_ = 20.0, dt_ = 0.005;
    double phase_ = 0.0, freq_now_ = 0.0, cmd_now_ = 0.0;
    bool have_ = false, started_ = false, done_ = false, include_gripper_ = true;
    double grip_ = 0.0, grip0_ = 0.0;
    std::string gripper_name_;
    rclcpp::Time t0_;
    std::vector<std::string> names_;
    std::vector<double> q_, q0_;
    std::ofstream f_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr st_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChirpNode>());
    rclcpp::shutdown();
    return 0;
}
