#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "comm_benchmark/transport.hpp"

namespace comm_benchmark {

// E1 — bare AF_INET / SOCK_DGRAM. No RTPS, no discovery, no library.
// Each side binds to local_port and sends to peer_ip:peer_port. The
// receive loop runs on its own thread with optional SCHED_FIFO.
class RawUdpTransport : public Transport {
 public:
  struct Config {
    std::string peer_ip;
    uint16_t    local_port{0};
    uint16_t    peer_port{0};
    int         rt_priority{0};   // 0 = no RT scheduling, else SCHED_FIFO prio
    int         rcvbuf_bytes{0};  // 0 = system default; else SO_RCVBUF
    int         sndbuf_bytes{0};
    bool        busy_poll{false}; // SO_BUSY_POLL on (uses ~50 µs)
  };

  explicit RawUdpTransport(const Config& cfg);
  ~RawUdpTransport() override;

  void start() override;
  void stop() override;
  void send(const Payload& msg) override;
  void set_recv_callback(RecvCallback cb) override;
  std::string name() const override { return "raw_udp"; }

 private:
  void recv_loop_();

  Config cfg_;
  int sock_fd_{-1};
  RecvCallback cb_;
  std::atomic<bool> running_{false};
  std::thread recv_thread_;
};

}  // namespace comm_benchmark
