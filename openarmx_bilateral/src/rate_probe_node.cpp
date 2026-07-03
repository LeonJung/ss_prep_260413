// rate_probe_node — zero-arg measurement of the ROS-side (software) loop rate.
//
// candump showed the CAN loop runs 154-173 Hz, yet `ros2 topic hz /joint_states`
// reported ~89 Hz. That gap is the controller_manager / broadcaster / DDS software
// ceiling. This node measures, with NO args, the ACTUAL publish rate + jitter of the
// key topics (joint_states + controller command topics) and reads each
// controller_manager's configured update_rate, so we can confirm where the ceiling is.
//
//   ros2 run  openarmx_bilateral rate_probe_node            # no args
//   ros2 launch openarmx_bilateral rate_probe.launch.py     # no args
//
// It auto-discovers every topic whose name matches a baked-in pattern (so it works for
// left/right/both and leader/follower without being told), subscribes generically
// (type-agnostic — only arrival timestamps are used), prints a table every 2 s, and
// writes /tmp/rate_probe.csv. Move the leader while it runs to load the loop.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {
// A topic is probed if its name contains any of these substrings.
const std::vector<std::string> kPatterns = {
    "joint_states",
    "forward_position_controller/commands",
    "forward_velocity_controller/commands",
    "forward_effort_controller/commands",
};
const std::vector<std::string> kCmNodes = {
    "/controller_manager", "/follower/controller_manager"};

bool matches(const std::string & topic)
{
  for (const auto & p : kPatterns) {
    if (topic.find(p) != std::string::npos) return true;
  }
  return false;
}
}  // namespace

class RateProbe : public rclcpp::Node
{
public:
  RateProbe()
  : Node("rate_probe")
  {
    csv_.open("/tmp/rate_probe.csv");
    csv_ << "t_s,topic,n_window,rate_hz,dt_mean_ms,dt_std_ms,dt_max_ms\n";
    csv_.flush();
    start_ = now();
    // Let the graph populate before discovering topics; keep re-scanning in case
    // controllers spawn late.
    discover_timer_ = create_wall_timer(1500ms, [this]() {this->discover();});
    report_timer_ = create_wall_timer(2s, [this]() {this->report();});
    query_update_rates();
    RCLCPP_INFO(get_logger(),
      "rate_probe: discovering topics, reporting every 2s -> /tmp/rate_probe.csv "
      "(move the leader to load the loop; Ctrl+C when done)");
  }

private:
  struct Stat
  {
    double last = -1.0;             // last arrival (s), for gap continuity
    std::vector<double> gaps;       // inter-arrival gaps within the window (s)
  };

  void discover()
  {
    const auto names_types = get_topic_names_and_types();
    for (const auto & [topic, types] : names_types) {
      if (subs_.count(topic) || types.empty() || !matches(topic)) continue;
      const std::string & type = types.front();
      try {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        auto sub = create_generic_subscription(
          topic, type, qos,
          [this, topic](std::shared_ptr<const rclcpp::SerializedMessage>) {
            this->on_msg(topic);
          });
        subs_[topic] = sub;
        stats_[topic];  // create entry
        RCLCPP_INFO(get_logger(), "  probing %s [%s]", topic.c_str(), type.c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "  skip %s: %s", topic.c_str(), e.what());
      }
    }
  }

  void on_msg(const std::string & topic)
  {
    const double t = (now() - start_).seconds();
    auto & s = stats_[topic];
    if (s.last >= 0.0) s.gaps.push_back(t - s.last);
    s.last = t;
  }

  void report()
  {
    const double t = (now() - start_).seconds();
    if (subs_.empty()) {
      RCLCPP_INFO(get_logger(), "[%.1fs] no matching topics yet...", t);
      return;
    }
    RCLCPP_INFO(get_logger(), "---- t=%.1fs ----  topic : rate(Hz)  dt mean/std/max (ms)", t);
    // stable order
    std::vector<std::string> topics;
    for (auto & kv : stats_) topics.push_back(kv.first);
    std::sort(topics.begin(), topics.end());
    for (const auto & topic : topics) {
      auto & s = stats_[topic];
      const size_t n = s.gaps.size();
      if (n == 0) {
        RCLCPP_INFO(get_logger(), "  %-52s : (no msgs)", topic.c_str());
        continue;
      }
      double sum = 0.0, mx = 0.0;
      for (double g : s.gaps) {sum += g; mx = std::max(mx, g);}
      const double mean = sum / n;
      double var = 0.0;
      for (double g : s.gaps) var += (g - mean) * (g - mean);
      const double sd = std::sqrt(var / n);
      const double rate = mean > 0 ? 1.0 / mean : 0.0;
      RCLCPP_INFO(get_logger(), "  %-52s : %6.1f   %.2f / %.2f / %.2f",
        topic.c_str(), rate, mean * 1e3, sd * 1e3, mx * 1e3);
      csv_ << t << "," << topic << "," << n << "," << rate << ","
           << mean * 1e3 << "," << sd * 1e3 << "," << mx * 1e3 << "\n";
      s.gaps.clear();  // reset window (keep s.last for continuity)
    }
    csv_.flush();
  }

  void query_update_rates()
  {
    for (const auto & node : kCmNodes) {
      auto cli = std::make_shared<rclcpp::AsyncParametersClient>(this, node);
      cm_clients_.push_back(cli);
      if (!cli->wait_for_service(1s)) {
        RCLCPP_INFO(get_logger(), "%s: not present (skip)", node.c_str());
        continue;
      }
      cli->get_parameters(
        {"update_rate"},
        [this, node](std::shared_future<std::vector<rclcpp::Parameter>> fut) {
          const auto params = fut.get();
          if (!params.empty()) {
            RCLCPP_INFO(get_logger(), "%s update_rate = %s Hz (configured)",
              node.c_str(), params[0].value_to_string().c_str());
          }
        });
    }
  }

  rclcpp::Time start_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subs_;
  std::map<std::string, Stat> stats_;
  std::vector<rclcpp::AsyncParametersClient::SharedPtr> cm_clients_;
  rclcpp::TimerBase::SharedPtr discover_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  std::ofstream csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RateProbe>());
  rclcpp::shutdown();
  return 0;
}
