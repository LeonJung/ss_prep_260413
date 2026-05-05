#pragma once

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace comm_benchmark {

// CSV recorder + on-the-fly summary statistics.
//
// Each receive event yields one row:
//   recv_ts_ns,own_seq_at_send,peer_seq_just_recv,peer_send_ts_ns,
//   computed_rtt_ns,inferred_loss
//
// "computed_rtt_ns" is filled when the inbound message echoes back
// our previously-sent own_seq (peer_last_seq); it is then
//   recv_ts_ns − send_ts_ns_we_recorded_for(own_seq_we_sent).
// We keep a small ring of {own_seq → send_ts_ns} entries to support
// this lookup without unbounded memory growth.
class StatsRecorder {
 public:
  StatsRecorder(const std::string& csv_path, std::size_t ring_size = 65536);
  ~StatsRecorder();

  // Call right after we publish own_seq at send_ts_ns.
  void on_send(uint32_t own_seq, int64_t send_ts_ns);

  // Call from the recv callback. peer_seq is the seq the peer just sent;
  // peer_acked_own_seq is the seq the peer reports last receiving from
  // us (used to compute round-trip latency for that one).
  void on_recv(int64_t recv_ts_ns,
               uint32_t peer_seq,
               int64_t peer_send_ts_ns,
               uint32_t peer_acked_own_seq);

  // One-shot summary line (p50/p90/p99/p99.9/max).
  std::string summary() const;

  // Number of rows written.
  std::size_t row_count() const { return row_count_; }

 private:
  struct SendRecord {
    uint32_t own_seq;
    int64_t  send_ts_ns;
  };

  std::ofstream csv_;
  mutable std::mutex mtx_;
  std::vector<SendRecord> ring_;
  std::size_t ring_head_{0};
  std::size_t row_count_{0};
  std::vector<int64_t> rtts_ns_;  // for percentile compute
  uint32_t last_peer_seq_seen_{0};
  bool any_peer_seq_{false};
  std::size_t loss_count_{0};
};

}  // namespace comm_benchmark
