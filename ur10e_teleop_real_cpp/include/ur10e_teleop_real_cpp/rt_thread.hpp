// rt_thread.hpp — RT scheduling + memory locking + cyclic timer utilities.
//
// Used by both leader_node and follower_node's control loops. When
// RTConfig::enabled is true, the calling thread is promoted to SCHED_FIFO
// at the given priority (requires CAP_SYS_NICE or root) and optional CPU
// affinity. When disabled, all calls are no-ops.
//
// Typical usage in a control thread:
//   RTConfig cfg{.enabled = true, .priority = 80, .cpu_affinity = 2};
//   init_rt_thread(cfg);
//   lock_process_memory();                    // call once in main
//   auto period = std::chrono::microseconds(2000);       // 500 Hz
//   auto deadline = now_ns();
//   while (running) {
//     // ... read state, compute tau, write torque ...
//     deadline = sleep_until(deadline + period);
//   }

#pragma once
#include <chrono>
#include <cstdint>
#include <string>

namespace ur10e_teleop_real_cpp {

struct RTConfig {
  bool enabled = false;       // false = normal priority (no-op)
  int  priority = 80;         // SCHED_FIFO priority (1..99)
  int  cpu_affinity = -1;     // -1 = any CPU; else pin to this core
};

// Promote the calling thread to SCHED_FIFO at cfg.priority and
// (optionally) pin it to a CPU. Returns true on success; prints a
// warning and returns false on failure (e.g. missing caps).
bool init_rt_thread(const RTConfig& cfg);

// mlockall(MCL_CURRENT | MCL_FUTURE). Call ONCE from the process's
// main thread, before heavy allocation / before starting the RT
// control thread. Returns false on failure (missing CAP_IPC_LOCK).
bool lock_process_memory();

// CLOCK_MONOTONIC now, in nanoseconds since some epoch.
std::chrono::nanoseconds now_monotonic();

// Sleep absolutely (CLOCK_MONOTONIC) until `deadline`. Use in a tight
// control loop to maintain the exact period without drift.
// Returns the deadline for the NEXT cycle (= deadline + period).
std::chrono::nanoseconds sleep_until(std::chrono::nanoseconds deadline);

// Running statistics for control-loop period jitter.
// Call tick() at the top of every cycle with the wakeup timestamp;
// call log_and_reset() at your preferred cadence (e.g., every 1 s).
class JitterTracker {
public:
  explicit JitterTracker(std::chrono::nanoseconds target_period);
  void tick(std::chrono::nanoseconds now);
  // Returns a one-line summary (min/mean/max vs target); resets counters.
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

}  // namespace ur10e_teleop_real_cpp
