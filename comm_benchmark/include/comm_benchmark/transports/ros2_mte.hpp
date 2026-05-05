#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "comm_benchmark/msg/payload.hpp"
#include "comm_benchmark/transport.hpp"

namespace comm_benchmark {

// B6 — ROS2 with BEST_EFFORT QoS + MultiThreadedExecutor + Reentrant
// callback group. The recv subscription gets its own callback group so
// it doesn't serialize behind anything else. Optional SCHED_FIFO on
// the executor's worker threads for additional jitter resistance.
class Ros2MteTransport : public Transport {
 public:
  struct Config {
    std::string node_name;
    std::string out_topic;
    std::string in_topic;
    int num_threads{2};      // executor worker threads
    int rt_priority{0};      // 0 = no RT scheduling on workers
  };

  explicit Ros2MteTransport(const Config& cfg);
  ~Ros2MteTransport() override;

  void start() override;
  void stop() override;
  void send(const Payload& msg) override;
  void set_recv_callback(RecvCallback cb) override;
  std::string name() const override { return "ros2_mte"; }

 private:
  Config cfg_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<comm_benchmark::msg::Payload>::SharedPtr pub_;
  rclcpp::Subscription<comm_benchmark::msg::Payload>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr recv_cbgroup_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr exec_;
  std::thread spin_thread_;
  RecvCallback cb_;
  std::atomic<bool> running_{false};
};

}  // namespace comm_benchmark
