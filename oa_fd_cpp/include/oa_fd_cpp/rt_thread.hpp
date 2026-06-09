// rt_thread.hpp — RT scheduling + memory locking + cyclic timer utilities.
// Ported verbatim from ur10e_teleop_real_cpp (namespace oa_fd).

#pragma once
#include <chrono>
#include <cstdint>
#include <string>

namespace oa_fd {

struct RTConfig {
  bool enabled = false;       // false = normal priority (no-op)
  int  priority = 80;         // SCHED_FIFO priority (1..99)
  int  cpu_affinity = -1;     // -1 = any CPU; else pin to this core
};

bool init_rt_thread(const RTConfig& cfg);
bool lock_process_memory();
std::chrono::nanoseconds now_monotonic();
std::chrono::nanoseconds sleep_until(std::chrono::nanoseconds deadline);

class JitterTracker {
public:
  explicit JitterTracker(std::chrono::nanoseconds target_period);
  void tick(std::chrono::nanoseconds now);
  std::string log_line(const std::string& prefix);
  void reset();
  std::chrono::nanoseconds target_period() const { return target_period_; }

private:
  std::chrono::nanoseconds target_period_;
  std::chrono::nanoseconds prev_now_{0};
  std::chrono::nanoseconds min_dt_{std::chrono::nanoseconds::max()};
  std::chrono::nanoseconds max_dt_{0};
  std::chrono::nanoseconds sum_dt_{0};
  uint64_t count_{0};
};

}  // namespace oa_fd
