// rt_thread.cpp — RT scheduling + memory locking + cyclic timer implementation.

#include "ur10e_teleop_control_unilateral_cpp/rt_thread.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sstream>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

namespace ur10e_teleop_control_unilateral_cpp {

bool init_rt_thread(const RTConfig& cfg) {
  if (!cfg.enabled) return true;

  // SCHED_FIFO priority
  struct sched_param sp;
  sp.sched_priority = cfg.priority;
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) {
    std::fprintf(stderr,
      "[rt_thread] pthread_setschedparam(SCHED_FIFO, prio=%d) failed: %s\n"
      "  Hint: run with CAP_SYS_NICE or use\n"
      "        sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep <binary>\n",
      cfg.priority, std::strerror(rc));
    return false;
  }

  // CPU affinity (optional)
  if (cfg.cpu_affinity >= 0) {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(cfg.cpu_affinity, &mask);
    rc = pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
    if (rc != 0) {
      std::fprintf(stderr, "[rt_thread] pthread_setaffinity_np(%d) failed: %s\n",
                   cfg.cpu_affinity, std::strerror(rc));
    }
  }
  return true;
}

bool lock_process_memory() {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::fprintf(stderr, "[rt_thread] mlockall failed: %s\n"
      "  Hint: run with CAP_IPC_LOCK or use\n"
      "        sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep <binary>\n",
      std::strerror(errno));
    return false;
  }
  return true;
}

std::chrono::nanoseconds now_monotonic() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return std::chrono::nanoseconds(
      static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec);
}

std::chrono::nanoseconds sleep_until(std::chrono::nanoseconds deadline) {
  struct timespec ts;
  ts.tv_sec  = deadline.count() / 1'000'000'000LL;
  ts.tv_nsec = deadline.count() % 1'000'000'000LL;
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
  return deadline;
}

JitterTracker::JitterTracker(std::chrono::nanoseconds target_period)
    : target_period_(target_period) {}

void JitterTracker::tick(std::chrono::nanoseconds now) {
  if (prev_now_.count() != 0) {
    auto dt = now - prev_now_;
    if (dt < min_dt_) min_dt_ = dt;
    if (dt > max_dt_) max_dt_ = dt;
    sum_dt_ += dt;
    ++count_;
  }
  prev_now_ = now;
}

std::string JitterTracker::log_line(const std::string& prefix) {
  std::ostringstream os;
  if (count_ == 0) {
    os << prefix << " jitter: (no samples)";
    return os.str();
  }
  auto mean = sum_dt_.count() / static_cast<int64_t>(count_);
  os << prefix << " period n=" << count_
     << "  target=" << target_period_.count() / 1000 << "us"
     << "  mean=" << mean / 1000 << "us"
     << "  min=" << min_dt_.count() / 1000 << "us"
     << "  max=" << max_dt_.count() / 1000 << "us";
  reset();
  return os.str();
}

void JitterTracker::reset() {
  prev_now_ = std::chrono::nanoseconds{0};
  min_dt_   = std::chrono::nanoseconds::max();
  max_dt_   = std::chrono::nanoseconds{0};
  sum_dt_   = std::chrono::nanoseconds{0};
  count_    = 0;
}

}  // namespace ur10e_teleop_control_unilateral_cpp
