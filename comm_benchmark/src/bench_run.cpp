// Common runner used by bench_a_main.cpp and bench_b_main.cpp.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "comm_benchmark/cli.hpp"
#include "comm_benchmark/payload.hpp"
#include "comm_benchmark/stats.hpp"
#include "comm_benchmark/transport.hpp"
#include "comm_benchmark/transports/raw_udp.hpp"
#include "comm_benchmark/transports/ros2_be.hpp"
#include "comm_benchmark/transports/ros2_mte.hpp"

namespace comm_benchmark {

namespace {
int64_t now_mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
      .count();
}
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }
}  // namespace

int run(const CliConfig& c) {
  // Topics depend on role: each side publishes its own and subscribes to peer.
  const std::string my_topic = (c.role == Role::A) ? "/comm_bench/a_to_b"
                                                   : "/comm_bench/b_to_a";
  const std::string peer_topic = (c.role == Role::A) ? "/comm_bench/b_to_a"
                                                     : "/comm_bench/a_to_b";
  const std::string node_name = (c.role == Role::A) ? "comm_bench_a"
                                                    : "comm_bench_b";

  // Build transport.
  std::unique_ptr<Transport> tx;
  bool needs_ros = false;
  switch (c.transport) {
    case TransportKind::Ros2Be: {
      Ros2BeTransport::Config cfg{node_name, my_topic, peer_topic};
      tx = std::make_unique<Ros2BeTransport>(cfg);
      needs_ros = true;
      break;
    }
    case TransportKind::Ros2Mte: {
      Ros2MteTransport::Config cfg;
      cfg.node_name   = node_name;
      cfg.out_topic   = my_topic;
      cfg.in_topic    = peer_topic;
      cfg.num_threads = c.num_threads;
      cfg.rt_priority = c.rt_priority;
      tx = std::make_unique<Ros2MteTransport>(cfg);
      needs_ros = true;
      break;
    }
    case TransportKind::RawUdp: {
      RawUdpTransport::Config cfg;
      cfg.peer_ip      = c.peer_ip;
      cfg.local_port   = c.local_port;
      cfg.peer_port    = c.peer_port;
      cfg.rt_priority  = c.rt_priority;
      cfg.rcvbuf_bytes = c.rcvbuf_bytes;
      cfg.sndbuf_bytes = c.sndbuf_bytes;
      cfg.busy_poll    = c.busy_poll;
      tx = std::make_unique<RawUdpTransport>(cfg);
      break;
    }
  }

  if (needs_ros) rclcpp::init(0, nullptr);

  StatsRecorder stats(c.csv_path);

  // Recv callback: record & track peer's last seq + send time.
  std::atomic<uint32_t> peer_last_seq_recv{0};
  std::atomic<int64_t>  peer_last_recv_ts{0};
  tx->set_recv_callback(
      [&](const Payload& msg, int64_t recv_ts_ns) {
        peer_last_seq_recv = msg.own_seq;
        peer_last_recv_ts  = recv_ts_ns;
        stats.on_recv(recv_ts_ns, msg.own_seq, msg.own_send_ts_ns,
                      msg.peer_last_seq);  // peer_last_seq = our own_seq they ack
      });

  tx->start();

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  const auto period_ns =
      static_cast<int64_t>(1e9 / std::max(1.0, c.rate_hz));
  const auto t0 = now_mono_ns();
  const auto t_end =
      t0 + static_cast<int64_t>(c.duration_sec * 1e9);

  Payload pkt;
  zero(pkt);
  uint32_t my_seq = 0;
  auto deadline = t0;

  std::printf("[bench_%s] transport=%s rate=%.0fHz duration=%.0fs csv=%s\n",
              c.role == Role::A ? "a" : "b",
              transport_name(c.transport), c.rate_hz, c.duration_sec,
              c.csv_path.c_str());

  // Optional SCHED_FIFO on the send loop's own thread.
  if (c.rt_priority > 0) {
    sched_param sp{};
    sp.sched_priority = c.rt_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      std::fprintf(stderr,
          "[bench] pthread_setschedparam(SCHED_FIFO,%d) failed on send thread "
          "— RT cap missing?\n", c.rt_priority);
    }
  }

  int64_t last_log = t0;
  while (!g_stop) {
    const auto now = now_mono_ns();
    if (now >= t_end) break;
    if (now < deadline) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(deadline - now));
      continue;
    }
    deadline += period_ns;

    // Fill payload — dummy sinusoid for q/qd/tau_ext, header for measurement.
    ++my_seq;
    pkt.own_seq         = my_seq;
    pkt.peer_last_seq   = peer_last_seq_recv.load(std::memory_order_relaxed);
    pkt.peer_recv_ts_ns = peer_last_recv_ts.load(std::memory_order_relaxed);
    pkt.own_send_ts_ns  = now;
    const double t = (now - t0) * 1e-9;
    for (int i = 0; i < 6; ++i) {
      pkt.q[i]       = std::sin(t + 0.1 * i);
      pkt.qd[i]      = std::cos(t + 0.1 * i);
      pkt.tau_ext[i] = 0.0;
    }
    // padding stays zero — we just need the bytes on the wire.

    stats.on_send(my_seq, now);
    tx->send(pkt);

    if (now - last_log > static_cast<int64_t>(2e9)) {
      std::printf("[bench] tx_seq=%u  %s\n", my_seq, stats.summary().c_str());
      last_log = now;
    }
  }

  std::printf("[bench] FINAL  %s\n", stats.summary().c_str());

  tx->stop();
  if (needs_ros) rclcpp::shutdown();
  return 0;
}

}  // namespace comm_benchmark
