#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "comm_benchmark/msg/payload.hpp"
#include "comm_benchmark/transport.hpp"

namespace comm_benchmark {

// B1 — ROS2 with BEST_EFFORT QoS, KEEP_LAST 1, VOLATILE durability.
// Default SingleThreadedExecutor on a non-RT thread. The cheapest
// possible ROS2-layer change vs the current bilateral default.
class Ros2BeTransport : public Transport {
 public:
  struct Config {
    std::string node_name;       // e.g. "bench_a"
    std::string out_topic;       // topic this side publishes
    std::string in_topic;        // topic this side subscribes
  };

  explicit Ros2BeTransport(const Config& cfg);
  ~Ros2BeTransport() override;

  void start() override;
  void stop() override;
  void send(const Payload& msg) override;
  void set_recv_callback(RecvCallback cb) override;
  std::string name() const override { return "ros2_be"; }

 private:
  Config cfg_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<comm_benchmark::msg::Payload>::SharedPtr pub_;
  rclcpp::Subscription<comm_benchmark::msg::Payload>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread spin_thread_;
  RecvCallback cb_;
  std::atomic<bool> running_{false};
};

}  // namespace comm_benchmark
