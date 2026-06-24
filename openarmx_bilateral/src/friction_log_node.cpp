// friction_log_node.cpp — Phase 3 friction-comp A/B logger.
//
// Friction comp doesn't change the PHYSICAL friction; it makes the motor ACTIVELY
// push in the direction of motion so a hand-moved (kp=kd=0) joint feels lighter.
// So the signal to compare is the effort breakdown while you move the arm by hand:
//   eff_cmd (sent to effort ctrl) = grav + friction_comp
//   fric    = eff_cmd - grav      = the friction assist  (~0 when OFF, ~scale*Fc when ON)
//   eff_meas= motor-reported torque
// With comp ON, `fric` (and the motor's assist aligned with velocity) appears;
// move each joint back-and-forth at a steady pace, log OFF then ON, compare.
//
//   # leader (default; the hand-pushed arm):
//   ros2 run openarmx_bilateral friction_log_node --ros-args -p csv:=/tmp/fric_OFF.csv
//   # follower: override topics
//   ros2 run openarmx_bilateral friction_log_node --ros-args -p csv:=/tmp/fric_ON.csv \
//     -p states:=/follower/joint_states \
//     -p grav:=/follower/grav_only \
//     -p effort_cmd:=/follower/left_forward_effort_controller/commands
//
// CSV (one row per joint per joint-state sample):
//   t, joint, q, vel, eff_meas, eff_cmd, grav, fric(=eff_cmd-grav)

#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class FrictionLogNode : public rclcpp::Node {
public:
    FrictionLogNode() : Node("openarmx_friction_log") {
        declare_parameter<std::string>("states", "/joint_states");                 // leader default
        declare_parameter<std::string>("grav", "/grav_only");
        declare_parameter<std::string>("effort_cmd", "/left_forward_effort_controller/commands");
        declare_parameter<std::string>("joint_prefix", "openarmx_left_joint");
        declare_parameter<int>("n_joints", 7);
        declare_parameter<std::string>("csv", "/tmp/fric_log.csv");

        nj_ = get_parameter("n_joints").as_int();
        std::string jp = get_parameter("joint_prefix").as_string();
        for (int i = 0; i < nj_; ++i) names_.push_back(jp + std::to_string(i + 1));
        grav_.assign(nj_, 0.0); cmd_.assign(nj_, 0.0);

        std::string path = get_parameter("csv").as_string();
        f_.open(path);
        if (f_.is_open()) f_ << "t,joint,q,vel,eff_meas,eff_cmd,grav,fric\n";
        RCLCPP_INFO(get_logger(), "friction log -> %s  (states=%s grav=%s eff=%s)",
                    path.c_str(), get_parameter("states").as_string().c_str(),
                    get_parameter("grav").as_string().c_str(),
                    get_parameter("effort_cmd").as_string().c_str());

        grav_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            get_parameter("grav").as_string(), 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
                for (int j = 0; j < nj_ && j < (int)m->data.size(); ++j) grav_[j] = m->data[j];
            });
        cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            get_parameter("effort_cmd").as_string(), 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
                for (int j = 0; j < nj_ && j < (int)m->data.size(); ++j) cmd_[j] = m->data[j];
                have_cmd_ = true;
            });
        st_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("states").as_string(), 50,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { onState(m); });
    }
    ~FrictionLogNode() override { if (f_.is_open()) f_.close(); }

private:
    void onState(const sensor_msgs::msg::JointState::SharedPtr& m) {
        if (!have_cmd_ || !f_.is_open()) return;
        double t = this->now().seconds();
        for (int j = 0; j < nj_; ++j) {
            double q = 0, vel = 0, eff = 0; bool found = false;
            for (size_t k = 0; k < m->name.size(); ++k)
                if (m->name[k] == names_[j]) {
                    if (k < m->position.size()) q = m->position[k];
                    if (k < m->velocity.size()) vel = m->velocity[k];
                    if (k < m->effort.size())   eff = m->effort[k];
                    found = true; break;
                }
            if (!found) continue;
            char buf[200];
            std::snprintf(buf, sizeof(buf), "%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                          t, j + 1, q, vel, eff, cmd_[j], grav_[j], cmd_[j] - grav_[j]);
            f_ << buf;
        }
    }
    int nj_ = 7;
    bool have_cmd_ = false;
    std::vector<std::string> names_;
    std::vector<double> grav_, cmd_;
    std::ofstream f_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grav_sub_, cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr st_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrictionLogNode>());
    rclcpp::shutdown();
    return 0;
}
