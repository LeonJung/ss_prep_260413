#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "comm_benchmark/payload.hpp"

namespace comm_benchmark {

// Abstract transport for the bake-off. Concrete implementations:
//   - Ros2BeTransport     (B1: ROS2 BEST_EFFORT QoS, single-threaded executor)
//   - Ros2MteTransport    (B6: ROS2 BEST_EFFORT QoS + MultiThreadedExecutor + RT priority)
//   - RawUdpTransport     (E1: bare AF_INET / SOCK_DGRAM)
//
// The runner thread calls send() at the configured rate. Each transport
// invokes the registered receive callback for every inbound payload,
// supplying the local CLOCK_MONOTONIC timestamp at receive.
class Transport {
 public:
  using RecvCallback = std::function<void(const Payload&, int64_t recv_ts_ns)>;

  virtual ~Transport() = default;

  // Bring the transport up. May spawn internal threads. Idempotent.
  virtual void start() = 0;

  // Tear down. Must be safe to call after start() failed or never ran.
  virtual void stop() = 0;

  // Publish exactly one message. Thread-safe with respect to itself.
  virtual void send(const Payload& msg) = 0;

  // Register the inbound callback. Must be called before start().
  virtual void set_recv_callback(RecvCallback cb) = 0;

  // Human-readable name for log lines and CSV.
  virtual std::string name() const = 0;
};

}  // namespace comm_benchmark
