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
    static std::string fmt(const std::map<std::string, double>& m) {
        std::string s;
        char buf[80];
        for (const auto& kv : m) {   // std::map => sorted by joint name
            std::snprintf(buf, sizeof(buf), "  %-26s % .4f rad (% 7.2f deg)\n",
                          kv.first.c_str(), kv.second, kv.second * 180.0 / 3.14159265358979);
            s += buf;
        }
        return s;
    }
    void dump() {
        std::string out = "\n===== JOINT ANGLES =====\n";
        out += have_l_ ? "[LEADER]\n"   + fmt(lmap_) : "[LEADER]   (no data)\n";
        out += have_f_ ? "[FOLLOWER]\n" + fmt(fmap_) : "[FOLLOWER] (no data)\n";
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
