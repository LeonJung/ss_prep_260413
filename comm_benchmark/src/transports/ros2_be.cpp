#include "comm_benchmark/transports/ros2_be.hpp"

#include <chrono>

namespace comm_benchmark {

namespace {
int64_t now_mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

Ros2BeTransport::Ros2BeTransport(const Config& cfg) : cfg_(cfg) {}

Ros2BeTransport::~Ros2BeTransport() { stop(); }

void Ros2BeTransport::set_recv_callback(RecvCallback cb) { cb_ = std::move(cb); }

void Ros2BeTransport::start() {
  if (running_.exchange(true)) return;

  node_ = rclcpp::Node::make_shared(cfg_.node_name);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();

  pub_ = node_->create_publisher<comm_benchmark::msg::Payload>(
      cfg_.out_topic, qos);

  sub_ = node_->create_subscription<comm_benchmark::msg::Payload>(
      cfg_.in_topic, qos,
      [this](comm_benchmark::msg::Payload::SharedPtr msg) {
        const int64_t recv = now_mono_ns();
        if (!cb_) return;
        Payload c;
        from_msg(*msg, c);
        cb_(c, recv);
      });

  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  spin_thread_ = std::thread([this]() { exec_->spin(); });
}

void Ros2BeTransport::stop() {
  if (!running_.exchange(false)) return;
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  sub_.reset();
  pub_.reset();
  exec_.reset();
  node_.reset();
}

void Ros2BeTransport::send(const Payload& msg) {
  if (!pub_) return;
  comm_benchmark::msg::Payload m;
  to_msg(msg, m);
  pub_->publish(m);
}

}  // namespace comm_benchmark
