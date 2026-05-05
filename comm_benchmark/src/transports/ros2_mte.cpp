#include "comm_benchmark/transports/ros2_mte.hpp"

#include <pthread.h>

#include <chrono>
#include <cstdio>
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace comm_benchmark {

namespace {
int64_t now_mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

Ros2MteTransport::Ros2MteTransport(const Config& cfg) : cfg_(cfg) {}

Ros2MteTransport::~Ros2MteTransport() { stop(); }

void Ros2MteTransport::set_recv_callback(RecvCallback cb) {
  cb_ = std::move(cb);
}

void Ros2MteTransport::start() {
  if (running_.exchange(true)) return;

  node_ = rclcpp::Node::make_shared(cfg_.node_name);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();

  pub_ = node_->create_publisher<comm_benchmark::msg::Payload>(
      cfg_.out_topic, qos);

  // Dedicated reentrant callback group for the recv subscription —
  // separates it from any default-group callbacks (timers, etc.).
  recv_cbgroup_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = recv_cbgroup_;

  sub_ = node_->create_subscription<comm_benchmark::msg::Payload>(
      cfg_.in_topic, qos,
      [this](comm_benchmark::msg::Payload::SharedPtr msg) {
        const int64_t recv = now_mono_ns();
        if (!cb_) return;
        Payload c;
        from_msg(*msg, c);
        cb_(c, recv);
      },
      sub_opts);

  rclcpp::ExecutorOptions exec_opts;
  exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      exec_opts, cfg_.num_threads);
  exec_->add_node(node_);

  spin_thread_ = std::thread([this]() { exec_->spin(); });

  if (cfg_.rt_priority > 0) {
    sched_param sp{};
    sp.sched_priority = cfg_.rt_priority;
    if (pthread_setschedparam(spin_thread_.native_handle(),
                              SCHED_FIFO, &sp) != 0) {
      std::fprintf(stderr,
          "[ros2_mte] pthread_setschedparam(SCHED_FIFO,%d) failed — "
          "RT cap missing?\n", cfg_.rt_priority);
    }
  }
}

void Ros2MteTransport::stop() {
  if (!running_.exchange(false)) return;
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  sub_.reset();
  pub_.reset();
  recv_cbgroup_.reset();
  exec_.reset();
  node_.reset();
}

void Ros2MteTransport::send(const Payload& msg) {
  if (!pub_) return;
  comm_benchmark::msg::Payload m;
  to_msg(msg, m);
  pub_->publish(m);
}

}  // namespace comm_benchmark
