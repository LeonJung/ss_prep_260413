#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>

#include "comm_benchmark/msg/payload.hpp"

namespace comm_benchmark {

// On-wire C struct used by the raw-UDP transport. Layout is fully
// natural-aligned (no packed) so it matches ROS2 CDR for the same
// fields. static_assert below pins the size to exactly 1 KiB.
struct Payload {
  uint32_t own_seq;          // 0
  uint32_t peer_last_seq;    // 4
  int64_t  own_send_ts_ns;   // 8
  int64_t  peer_recv_ts_ns;  // 16
  double   q[6];             // 24
  double   qd[6];            // 72
  double   tau_ext[6];       // 120
  uint8_t  padding[856];     // 168
                             // 1024
};
static_assert(sizeof(Payload) == 1024,
              "Payload struct must be exactly 1 KiB");
static_assert(std::is_trivially_copyable<Payload>::value,
              "Payload must be trivially copyable for memcpy");

inline void zero(Payload& p) noexcept { std::memset(&p, 0, sizeof(p)); }

// ROS2 msg ↔ C struct conversion helpers.
inline void to_msg(const Payload& src,
                   comm_benchmark::msg::Payload& dst) noexcept {
  dst.own_seq          = src.own_seq;
  dst.peer_last_seq    = src.peer_last_seq;
  dst.own_send_ts_ns   = src.own_send_ts_ns;
  dst.peer_recv_ts_ns  = src.peer_recv_ts_ns;
  for (int i = 0; i < 6; ++i) {
    dst.q[i] = src.q[i];
    dst.qd[i] = src.qd[i];
    dst.tau_ext[i] = src.tau_ext[i];
  }
  std::memcpy(dst.padding.data(), src.padding, sizeof(src.padding));
}

inline void from_msg(const comm_benchmark::msg::Payload& src,
                     Payload& dst) noexcept {
  dst.own_seq          = src.own_seq;
  dst.peer_last_seq    = src.peer_last_seq;
  dst.own_send_ts_ns   = src.own_send_ts_ns;
  dst.peer_recv_ts_ns  = src.peer_recv_ts_ns;
  for (int i = 0; i < 6; ++i) {
    dst.q[i] = src.q[i];
    dst.qd[i] = src.qd[i];
    dst.tau_ext[i] = src.tau_ext[i];
  }
  std::memcpy(dst.padding, src.padding.data(), sizeof(dst.padding));
}

}  // namespace comm_benchmark
