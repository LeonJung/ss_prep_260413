#include "comm_benchmark/stats.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace comm_benchmark {

StatsRecorder::StatsRecorder(const std::string& csv_path, std::size_t ring_size)
    : ring_(ring_size, SendRecord{0, 0}) {
  csv_.open(csv_path, std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    std::fprintf(stderr,
                 "[stats] WARNING: cannot open '%s' for write — CSV disabled\n",
                 csv_path.c_str());
  } else {
    csv_ << "recv_ts_ns,peer_seq,peer_send_ts_ns,"
            "peer_acked_own_seq,computed_rtt_ns,inferred_loss\n";
  }
  rtts_ns_.reserve(1 << 20);
}

StatsRecorder::~StatsRecorder() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (csv_.is_open()) csv_.close();
}

void StatsRecorder::on_send(uint32_t own_seq, int64_t send_ts_ns) {
  std::lock_guard<std::mutex> lk(mtx_);
  ring_[ring_head_] = SendRecord{own_seq, send_ts_ns};
  ring_head_ = (ring_head_ + 1) % ring_.size();
}

void StatsRecorder::on_recv(int64_t recv_ts_ns,
                            uint32_t peer_seq,
                            int64_t peer_send_ts_ns,
                            uint32_t peer_acked_own_seq) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Sequential-loss detection on the inbound stream from peer.
  std::size_t loss_now = 0;
  if (any_peer_seq_) {
    const uint32_t expected = last_peer_seq_seen_ + 1;
    if (peer_seq > expected) loss_now = peer_seq - expected;
  }
  loss_count_ += loss_now;
  last_peer_seq_seen_ = peer_seq;
  any_peer_seq_ = true;

  // Look up our send timestamp for peer_acked_own_seq.
  int64_t rtt = -1;
  if (peer_acked_own_seq != 0) {
    for (const auto& rec : ring_) {
      if (rec.own_seq == peer_acked_own_seq) {
        rtt = recv_ts_ns - rec.send_ts_ns;
        rtts_ns_.push_back(rtt);
        break;
      }
    }
  }

  if (csv_.is_open()) {
    csv_ << recv_ts_ns << ',' << peer_seq << ',' << peer_send_ts_ns << ','
         << peer_acked_own_seq << ',' << rtt << ',' << loss_now << '\n';
  }
  ++row_count_;
}

namespace {
int64_t percentile(std::vector<int64_t>& v, double p) {
  if (v.empty()) return -1;
  const auto idx = static_cast<std::size_t>(p * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + idx, v.end());
  return v[idx];
}
}  // namespace

std::string StatsRecorder::summary() const {
  std::lock_guard<std::mutex> lk(mtx_);
  std::vector<int64_t> copy = rtts_ns_;
  std::ostringstream oss;
  oss << "rows=" << row_count_ << "  rtt_samples=" << copy.size()
      << "  loss=" << loss_count_;
  if (!copy.empty()) {
    auto p50 = percentile(copy, 0.50);
    auto p90 = percentile(copy, 0.90);
    auto p99 = percentile(copy, 0.99);
    auto p999 = percentile(copy, 0.999);
    auto pmax = percentile(copy, 1.00);
    oss << "  rtt_us[p50/p90/p99/p99.9/max]="
        << (p50 / 1000) << '/' << (p90 / 1000) << '/' << (p99 / 1000) << '/'
        << (p999 / 1000) << '/' << (pmax / 1000);
  }
  return oss.str();
}

}  // namespace comm_benchmark
