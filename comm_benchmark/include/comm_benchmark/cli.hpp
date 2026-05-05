#pragma once

#include <cstdint>
#include <string>

namespace comm_benchmark {

enum class Role { A, B };
enum class TransportKind { Ros2Be, Ros2Mte, RawUdp };

struct CliConfig {
  Role role{Role::A};
  TransportKind transport{TransportKind::Ros2Be};
  double rate_hz{500.0};
  double duration_sec{120.0};
  std::string csv_path{"bench.csv"};

  // raw_udp only:
  std::string peer_ip{"127.0.0.1"};
  uint16_t local_port{0};
  uint16_t peer_port{0};
  int rcvbuf_bytes{0};
  int sndbuf_bytes{0};
  bool busy_poll{false};

  // ros2_mte only:
  int num_threads{2};

  // shared:
  int rt_priority{0};
};

// Parse argv. Throws std::runtime_error on bad input. Sets `role`
// from argv[0] basename (bench_a / bench_b) but still allows --role
// override. Default ports per role.
CliConfig parse_cli(int argc, char** argv);

const char* transport_name(TransportKind t);

}  // namespace comm_benchmark
