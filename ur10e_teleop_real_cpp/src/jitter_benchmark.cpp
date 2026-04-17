// jitter_benchmark.cpp — standalone cyclic-timer jitter measurement.
//
// Purpose: quantify PREEMPT_RT benefit on this host, isolated from ROS and
// from UR I/O. Runs the same CLOCK_MONOTONIC absolute-deadline sleep loop
// that the control nodes use, for N seconds, and writes per-cycle dt to
// stdout as CSV. Human-readable summary goes to stderr.
//
// Typical comparison:
//   ./jitter_benchmark --rt-mode false --duration 30 > nonrt.csv
//   ./jitter_benchmark --rt-mode true  --duration 30 > rt.csv
//
// The RT run needs CAP_SYS_NICE + CAP_IPC_LOCK (else it prints a warning
// and falls through to normal scheduling).

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <string>
#include <vector>

#include "ur10e_teleop_real_cpp/rt_thread.hpp"

using namespace ur10e_teleop_real_cpp;

static bool parse_bool(const char* s) {
  std::string v = s;
  for (auto& c : v) c = static_cast<char>(std::tolower(c));
  return (v == "true" || v == "1" || v == "on" || v == "yes");
}

int main(int argc, char** argv) {
  RTConfig rt_cfg;
  int    period_us = 2000;     // 500 Hz
  double duration  = 10.0;     // seconds
  bool   csv       = true;     // emit per-cycle CSV to stdout

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* n) -> const char* {
      if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", n); std::exit(2); }
      return argv[++i];
    };
    if      (a == "--rt-mode")     rt_cfg.enabled = parse_bool(need("--rt-mode"));
    else if (a == "--rt")          rt_cfg.enabled = true;
    else if (a == "--no-rt")       rt_cfg.enabled = false;
    else if (a == "--rt-priority") rt_cfg.priority = std::atoi(need("--rt-priority"));
    else if (a == "--rt-cpu")      rt_cfg.cpu_affinity = std::atoi(need("--rt-cpu"));
    else if (a == "--period-us")   period_us = std::atoi(need("--period-us"));
    else if (a == "--duration")    duration = std::atof(need("--duration"));
    else if (a == "--no-csv")      csv = false;
    else if (a == "--help" || a == "-h") {
      std::fprintf(stderr,
        "Usage: %s [--rt-mode true|false] [--rt-priority N] [--rt-cpu N]\n"
        "         [--period-us US] [--duration SEC] [--no-csv]\n", argv[0]);
      return 0;
    }
  }

  std::fprintf(stderr,
    "# jitter_benchmark  rt=%s  priority=%d  cpu=%d  period=%d us  duration=%.1f s\n",
    rt_cfg.enabled ? "ON" : "OFF", rt_cfg.priority, rt_cfg.cpu_affinity,
    period_us, duration);

  if (rt_cfg.enabled) {
    lock_process_memory();
  }
  init_rt_thread(rt_cfg);

  auto period = std::chrono::microseconds(period_us);
  JitterTracker jt(period);
  auto deadline = now_monotonic();
  auto start    = deadline;
  auto end_time = start + std::chrono::nanoseconds(
      static_cast<int64_t>(duration * 1e9));

  // Pre-allocate the CSV buffer so no heap activity in the hot loop.
  const int64_t expected_cycles =
      static_cast<int64_t>(duration * 1e6) / period_us + 16;
  std::vector<int32_t> dt_us_buf;
  std::vector<int64_t> now_ns_buf;
  if (csv) {
    dt_us_buf.reserve(expected_cycles);
    now_ns_buf.reserve(expected_cycles);
  }

  std::chrono::nanoseconds prev_now{0};
  int cycle = 0;

  while (deadline < end_time) {
    auto t = now_monotonic();
    jt.tick(t);

    if (csv && prev_now.count() != 0) {
      auto dt_us = (t - prev_now).count() / 1000;
      // push_back is O(1) amortized; capacity reserved above so no realloc.
      dt_us_buf.push_back(static_cast<int32_t>(dt_us));
      now_ns_buf.push_back(t.count());
    }
    prev_now = t;
    ++cycle;

    deadline += period;
    sleep_until(deadline);
  }

  // ---- now that the timing-sensitive loop is done, dump CSV ----
  if (csv) {
    std::printf("cycle,now_ns,dt_us\n");
    for (size_t i = 0; i < dt_us_buf.size(); ++i) {
      std::printf("%zu,%ld,%d\n", i + 1, (long)now_ns_buf[i], dt_us_buf[i]);
    }
  }

  std::fprintf(stderr, "# summary: %s\n",
               jt.log_line("jitter_benchmark").c_str());
  return 0;
}
