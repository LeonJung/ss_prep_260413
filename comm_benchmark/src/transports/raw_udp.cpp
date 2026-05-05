#include "comm_benchmark/transports/raw_udp.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <pthread.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace comm_benchmark {

namespace {
int64_t now_mono_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

RawUdpTransport::RawUdpTransport(const Config& cfg) : cfg_(cfg) {
  if (cfg_.peer_ip.empty()) throw std::invalid_argument("RawUdp: peer_ip empty");
  if (cfg_.local_port == 0) throw std::invalid_argument("RawUdp: local_port 0");
  if (cfg_.peer_port == 0)  throw std::invalid_argument("RawUdp: peer_port 0");
}

RawUdpTransport::~RawUdpTransport() { stop(); }

void RawUdpTransport::set_recv_callback(RecvCallback cb) { cb_ = std::move(cb); }

void RawUdpTransport::start() {
  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) throw std::runtime_error("socket(): " + std::string(strerror(errno)));

  // Optional SO_RCVBUF / SO_SNDBUF.
  if (cfg_.rcvbuf_bytes > 0) {
    int v = cfg_.rcvbuf_bytes;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &v, sizeof(v));
  }
  if (cfg_.sndbuf_bytes > 0) {
    int v = cfg_.sndbuf_bytes;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &v, sizeof(v));
  }
  // Optional SO_BUSY_POLL — application-level low-latency receive.
  if (cfg_.busy_poll) {
#ifdef SO_BUSY_POLL
    int us = 50;
    ::setsockopt(sock_fd_, SOL_SOCKET, SO_BUSY_POLL, &us, sizeof(us));
#else
    std::fprintf(stderr, "[raw_udp] SO_BUSY_POLL not in this kernel\n");
#endif
  }

  // Reuse + bind.
  int reuse = 1;
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(cfg_.local_port);
  if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
    const int e = errno;
    ::close(sock_fd_);
    sock_fd_ = -1;
    throw std::runtime_error("bind(): " + std::string(strerror(e)));
  }

  running_ = true;
  recv_thread_ = std::thread(&RawUdpTransport::recv_loop_, this);

  // Optional SCHED_FIFO on the recv thread.
  if (cfg_.rt_priority > 0) {
    sched_param sp{};
    sp.sched_priority = cfg_.rt_priority;
    if (pthread_setschedparam(recv_thread_.native_handle(), SCHED_FIFO, &sp) != 0) {
      std::fprintf(stderr,
          "[raw_udp] pthread_setschedparam(SCHED_FIFO,%d) failed — "
          "RT cap missing? Continuing at default scheduling.\n",
          cfg_.rt_priority);
    }
  }
}

void RawUdpTransport::stop() {
  if (!running_.exchange(false)) return;
  if (sock_fd_ >= 0) {
    ::shutdown(sock_fd_, SHUT_RDWR);
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  if (recv_thread_.joinable()) recv_thread_.join();
}

void RawUdpTransport::send(const Payload& msg) {
  if (sock_fd_ < 0) return;
  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  peer.sin_port   = htons(cfg_.peer_port);
  if (inet_pton(AF_INET, cfg_.peer_ip.c_str(), &peer.sin_addr) != 1) return;
  ::sendto(sock_fd_, &msg, sizeof(msg), 0,
           reinterpret_cast<sockaddr*>(&peer), sizeof(peer));
}

void RawUdpTransport::recv_loop_() {
  Payload buf;
  while (running_) {
    sockaddr_in src{};
    socklen_t   sl = sizeof(src);
    ssize_t n = ::recvfrom(sock_fd_, &buf, sizeof(buf), 0,
                           reinterpret_cast<sockaddr*>(&src), &sl);
    if (n != static_cast<ssize_t>(sizeof(buf))) {
      if (!running_) break;
      continue;  // dropped malformed / partial
    }
    if (cb_) cb_(buf, now_mono_ns());
  }
}

}  // namespace comm_benchmark
