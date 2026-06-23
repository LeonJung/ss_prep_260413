// joint_echo_node.cpp — TEMPORARY helper: print per-joint angles for BOTH arms
// of leader and follower, so the operator can read off real limits by moving the
// arms (in grav+velff). Prints every joint in each /joint_states message (a
// bimanual robot publishes left AND right joints), throttled to ~2 Hz.
//
//   ros2 run openarmx_bilateral joint_echo_node
//   # override topics if your namespacing differs:
//   ros2 run openarmx_bilateral joint_echo_node --ros-args \
//       -p leader:=/joint_states -p follower:=/follower/joint_states

#include <algorithm>
#include <cstdio>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointEcho : public rclcpp::Node {
public:
    JointEcho() : Node("openarmx_joint_echo") {
        declare_parameter<std::string>("leader", "/joint_states");
        declare_parameter<std::string>("follower", "/follower/joint_states");
        declare_parameter<double>("rate_hz", 2.0);

        lsub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("leader").as_string(), 10,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { store(lmap_, m); have_l_ = true; });
        fsub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("follower").as_string(), 10,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { store(fmap_, m); have_f_ = true; });

        double hz = get_parameter("rate_hz").as_double();
        if (hz <= 0) hz = 2.0;
        timer_ = create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / hz)), [this] { dump(); });
        RCLCPP_INFO(get_logger(), "joint_echo: move the arms; reading leader=%s follower=%s",
                    get_parameter("leader").as_string().c_str(),
                    get_parameter("follower").as_string().c_str());
    }

private:
    static void store(std::map<std::string, double>& m,
                      const sensor_msgs::msg::JointState::SharedPtr& msg) {
        for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k)
            m[msg->name[k]] = msg->position[k];
    }
    // one line per arm: [n, n, ...] j1..j7 angles in degrees
    static std::string arm_line(const std::map<std::string, double>& m,
                                const char* label, const std::string& side) {
        char buf[32];
        std::string s = std::string(label) + " [";
        for (int j = 1; j <= 7; ++j) {
            auto it = m.find("openarmx_" + side + "_joint" + std::to_string(j));
            if (it == m.end()) s += "--";
            else { std::snprintf(buf, sizeof(buf), "%.1f", it->second * 180.0 / 3.14159265358979); s += buf; }
            if (j < 7) s += ", ";
        }
        return s + "]\n";
    }
    void dump() {
        std::string out = "\n===== JOINT ANGLES (deg) =====\n";
        if (have_l_) {
            out += arm_line(lmap_, "[LEADER  L]", "left");
            out += arm_line(lmap_, "[LEADER  R]", "right");
        } else out += "[LEADER]   (no data)\n";
        if (have_f_) {
            out += arm_line(fmap_, "[FOLLOW  L]", "left");
            out += arm_line(fmap_, "[FOLLOW  R]", "right");
        } else out += "[FOLLOWER] (no data)\n";
        RCLCPP_INFO(get_logger(), "%s", out.c_str());
    }

    bool have_l_ = false, have_f_ = false;
    std::map<std::string, double> lmap_, fmap_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr lsub_, fsub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointEcho>());
    rclcpp::shutdown();
    return 0;
}
