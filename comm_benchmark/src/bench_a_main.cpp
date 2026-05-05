#include <cstdio>
#include "comm_benchmark/cli.hpp"

namespace comm_benchmark { int run(const CliConfig&); }

int main(int argc, char** argv) {
  try {
    auto cfg = comm_benchmark::parse_cli(argc, argv);
    cfg.role = comm_benchmark::Role::A;  // pinned by binary name
    return comm_benchmark::run(cfg);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "bench_a: %s\n", e.what());
    return 1;
  }
}
