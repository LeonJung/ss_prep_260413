// oa_mit_relay_node.cpp — Version A (full ros2_control bilateral) cross-relay.
//
// Both arms run the OFFICIAL ros2_control stack (forward_position_controller +
// optional forward_effort_controller gravity comp), each with its own dedicated
// per-arm HW loop (the smooth path). This node closes the bilateral loop at the
// ROS layer: it reads each arm's /joint_states and publishes the PEER's joint
// positions to the other arm's forward_position_controller. The MIT kp in the
// HW interface then provides the coupling spring on BOTH arms = force feedback
// (follower stiff kp, leader soft kp via the HW kp_joint params).
//
// Fully topic-parameterized (works regardless of how the two bringups are
// namespaced — set the 4 topics to match your /leader, /follower namespaces).
// Joint mapping by NAME; couple_sign matches the openarmx relay convention.
//
//   ros2 run oa_mit oa_mit_relay_node --ros-args \
//     -p leader_states:=/leader/joint_states \
//     -p follower_states:=/follower/joint_states \
//     -p leader_cmd:=/leader/right_forward_position_controller/commands \
//     -p follower_cmd:=/follower/right_forward_position_controller/commands

#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class RelayNode : public rclcpp::Node {
public:
    RelayNode() : Node("oa_mit_relay") {
        declare_parameter<std::string>("leader_states", "/leader/joint_states");
        declare_parameter<std::string>("follower_states", "/follower/joint_states");
        declare_parameter<std::string>("leader_cmd",
            "/leader/right_forward_position_controller/commands");
        declare_parameter<std::string>("follower_cmd",
            "/follower/right_forward_position_controller/commands");
        declare_parameter<std::string>("joint_prefix", "openarmx_right_joint");  // +1..7
        declare_parameter<int>("n_joints", 7);
        declare_parameter<bool>("include_gripper", true);   // append 1 value (0.0)
        declare_parameter<double>("couple_sign", -1.0);     // relay convention
        declare_parameter<int>("rate_hz", 200);

        nj_ = get_parameter("n_joints").as_int();
        std::string pref = get_parameter("joint_prefix").as_string();
        include_gripper_ = get_parameter("include_gripper").as_bool();
        couple_sign_ = get_parameter("couple_sign").as_double();
        int rate = get_parameter("rate_hz").as_int();
        for (int i = 0; i < nj_; ++i) names_.push_back(pref + std::to_string(i + 1));

        q_leader_.assign(nj_, 0.0); q_follower_.assign(nj_, 0.0);

        leader_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            get_parameter("leader_cmd").as_string(), 10);
        follower_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            get_parameter("follower_cmd").as_string(), 10);
        leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("leader_states").as_string(), 10,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { grab(m, q_leader_, have_l_); });
        follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("follower_states").as_string(), 10,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { grab(m, q_follower_, have_f_); });

        timer_ = create_wall_timer(std::chrono::microseconds(1000000 / std::max(1, rate)),
                                   [this] { tick(); });
        RCLCPP_INFO(get_logger(), "relay up: n=%d sign=%.0f gripper=%s",
                    nj_, couple_sign_, include_gripper_ ? "yes" : "no");
    }

private:
    void grab(const sensor_msgs::msg::JointState::SharedPtr& m,
              std::vector<double>& q, bool& have) {
        for (int j = 0; j < nj_; ++j)
            for (size_t k = 0; k < m->name.size(); ++k)
                if (m->name[k] == names_[j] && k < m->position.size()) { q[j] = m->position[k]; break; }
        have = true;
    }
    std_msgs::msg::Float64MultiArray make(const std::vector<double>& src) {
        std_msgs::msg::Float64MultiArray msg;
        for (int j = 0; j < nj_; ++j) msg.data.push_back(couple_sign_ * src[j]);
        if (include_gripper_) msg.data.push_back(0.0);   // gripper: hold 0
        return msg;
    }
    void tick() {
        // follower tracks leader; leader tracks follower (force feedback)
        if (have_l_) follower_cmd_pub_->publish(make(q_leader_));
        if (have_f_) leader_cmd_pub_->publish(make(q_follower_));
    }

    int nj_ = 7;
    bool include_gripper_ = true, have_l_ = false, have_f_ = false;
    double couple_sign_ = -1.0;
    std::vector<std::string> names_;
    std::vector<double> q_leader_, q_follower_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leader_cmd_pub_, follower_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_, follower_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelayNode>());
    rclcpp::shutdown();
    return 0;
}
