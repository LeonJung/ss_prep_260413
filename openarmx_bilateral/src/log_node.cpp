// log_node.cpp — Phase-1 data logger for the follower gravity-comp A/B test.
// Logs, per follower joint-state sample, the COMMANDED position (relay -> follower
// forward_position_controller) vs the follower's ACTUAL position/velocity/effort,
// and the tracking error. Run once with follower gravity comp ON and once OFF,
// give both CSVs -> objective comparison (less sag / lower effort = comp helps).
//
//   ros2 run openarmx_bilateral log_node --ros-args -p csv:=/tmp/grav_ON.csv
//
// CSV columns (long format, one row per joint per sample):
//   t, joint, cmd, act, err(=cmd-act), vel, eff

#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class LogNode : public rclcpp::Node {
public:
    LogNode() : Node("openarmx_bilateral_log") {
        declare_parameter<std::string>("states", "/follower/joint_states");
        declare_parameter<std::string>("cmd", "/follower/left_forward_position_controller/commands");
        declare_parameter<std::string>("joint_prefix", "openarmx_left_joint");
        declare_parameter<int>("n_joints", 7);
        declare_parameter<std::string>("csv", "/tmp/openarmx_grav.csv");

        nj_ = get_parameter("n_joints").as_int();
        std::string jp = get_parameter("joint_prefix").as_string();
        for (int i = 0; i < nj_; ++i) names_.push_back(jp + std::to_string(i + 1));
        cmd_.assign(nj_, 0.0);

        std::string path = get_parameter("csv").as_string();
        f_.open(path);
        if (f_.is_open()) f_ << "t,joint,cmd,act,err,vel,eff\n";
        RCLCPP_INFO(get_logger(), "logging follower cmd-vs-actual -> %s", path.c_str());

        cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            get_parameter("cmd").as_string(), 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
                for (int i = 0; i < nj_ && i < (int)m->data.size(); ++i) cmd_[i] = m->data[i];
                have_cmd_ = true;
            });
        st_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("states").as_string(), 50,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { onState(m); });
    }
    ~LogNode() override { if (f_.is_open()) f_.close(); }

private:
    void onState(const sensor_msgs::msg::JointState::SharedPtr& m) {
        if (!have_cmd_ || !f_.is_open()) return;
        double t = this->now().seconds();
        for (int j = 0; j < nj_; ++j) {
            double act = 0, vel = 0, eff = 0; bool found = false;
            for (size_t k = 0; k < m->name.size(); ++k)
                if (m->name[k] == names_[j]) {
                    if (k < m->position.size()) act = m->position[k];
                    if (k < m->velocity.size()) vel = m->velocity[k];
                    if (k < m->effort.size())   eff = m->effort[k];
                    found = true; break;
                }
            if (!found) continue;
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                          t, j + 1, cmd_[j], act, cmd_[j] - act, vel, eff);
            f_ << buf;
        }
    }
    int nj_ = 7;
    bool have_cmd_ = false;
    std::vector<std::string> names_;
    std::vector<double> cmd_;
    std::ofstream f_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr st_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogNode>());
    rclcpp::shutdown();
    return 0;
}
