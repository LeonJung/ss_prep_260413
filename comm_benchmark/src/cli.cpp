#include "comm_benchmark/cli.hpp"

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace comm_benchmark {

namespace {

bool starts_with(const char* s, const char* p) {
  return std::strncmp(s, p, std::strlen(p)) == 0;
}

void usage(const char* prog) {
  std::cerr <<
    "usage: " << prog << " [--role a|b]\n"
    "       --transport ros2_be|ros2_mte|raw_udp\n"
    "       [--rate-hz 500] [--duration-sec 120] [--csv path]\n"
    "       [--rt-priority 0]\n"
    "       (raw_udp only) --peer-ip IP [--local-port P] [--peer-port P]\n"
    "                      [--rcvbuf BYTES] [--sndbuf BYTES] [--busy-poll]\n"
    "       (ros2_mte only) [--num-threads 2]\n";
}

}  // namespace

const char* transport_name(TransportKind t) {
  switch (t) {
    case TransportKind::Ros2Be: return "ros2_be";
    case TransportKind::Ros2Mte: return "ros2_mte";
    case TransportKind::RawUdp: return "raw_udp";
  }
  return "?";
}

CliConfig parse_cli(int argc, char** argv) {
  CliConfig c;

  // Default role from argv[0] basename.
  if (argc > 0 && argv[0]) {
    const char* p = std::strrchr(argv[0], '/');
    p = p ? p + 1 : argv[0];
    if (std::strstr(p, "bench_b")) c.role = Role::B;
    else                            c.role = Role::A;
  }
  bool transport_set = false;

  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    auto need = [&](int more) -> const char* {
      if (i + more >= argc) {
        usage(argv[0]);
        throw std::runtime_error(std::string("missing value for ") + a);
      }
      return argv[++i];
    };
    if (!std::strcmp(a, "--role")) {
      const char* v = need(1);
      if (!std::strcmp(v, "a")) c.role = Role::A;
      else if (!std::strcmp(v, "b")) c.role = Role::B;
      else throw std::runtime_error("--role must be 'a' or 'b'");
    } else if (!std::strcmp(a, "--transport")) {
      const char* v = need(1);
      transport_set = true;
      if (!std::strcmp(v, "ros2_be"))  c.transport = TransportKind::Ros2Be;
      else if (!std::strcmp(v, "ros2_mte")) c.transport = TransportKind::Ros2Mte;
      else if (!std::strcmp(v, "raw_udp"))  c.transport = TransportKind::RawUdp;
      else throw std::runtime_error("unknown transport: " + std::string(v));
    } else if (!std::strcmp(a, "--rate-hz")) {
      c.rate_hz = std::stod(need(1));
    } else if (!std::strcmp(a, "--duration-sec")) {
      c.duration_sec = std::stod(need(1));
    } else if (!std::strcmp(a, "--csv")) {
      c.csv_path = need(1);
    } else if (!std::strcmp(a, "--rt-priority")) {
      c.rt_priority = std::stoi(need(1));
    } else if (!std::strcmp(a, "--peer-ip")) {
      c.peer_ip = need(1);
    } else if (!std::strcmp(a, "--local-port")) {
      c.local_port = static_cast<uint16_t>(std::stoi(need(1)));
    } else if (!std::strcmp(a, "--peer-port")) {
      c.peer_port = static_cast<uint16_t>(std::stoi(need(1)));
    } else if (!std::strcmp(a, "--rcvbuf")) {
      c.rcvbuf_bytes = std::stoi(need(1));
    } else if (!std::strcmp(a, "--sndbuf")) {
      c.sndbuf_bytes = std::stoi(need(1));
    } else if (!std::strcmp(a, "--busy-poll")) {
      c.busy_poll = true;
    } else if (!std::strcmp(a, "--num-threads")) {
      c.num_threads = std::stoi(need(1));
    } else if (!std::strcmp(a, "-h") || !std::strcmp(a, "--help")) {
      usage(argv[0]);
      std::exit(0);
    } else if (starts_with(a, "--")) {
      throw std::runtime_error("unknown flag: " + std::string(a));
    }
  }
  if (!transport_set) {
    throw std::runtime_error("--transport is required");
  }
  // Default ports per role: A binds 18000 → B:18001, B binds 18001 → A:18000.
  if (c.transport == TransportKind::RawUdp) {
    if (c.role == Role::A) {
      if (c.local_port == 0) c.local_port = 18000;
      if (c.peer_port  == 0) c.peer_port  = 18001;
    } else {
      if (c.local_port == 0) c.local_port = 18001;
      if (c.peer_port  == 0) c.peer_port  = 18000;
    }
  }
  return c;
}

}  // namespace comm_benchmark
