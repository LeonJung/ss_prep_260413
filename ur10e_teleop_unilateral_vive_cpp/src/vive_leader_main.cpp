// vive_leader_main.cpp — entry point for the Vive-tracker-based leader.

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ur10e_teleop_unilateral_vive_cpp/rt_thread.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/vive_leader_node.hpp"

namespace {

void print_usage(const char* prog) {
  std::fprintf(stderr,
    "Usage: %s [--robot ur3e|ur10e|ur5e] [--config PATH] [--calib PATH]\n"
    "         [--tracker-serial S] [--rate-hz 500]\n"
    "         [--rt-mode true|false] [--rt-priority N] [--rt-cpu N]\n"
    "Defaults: robot=ur3e  calib=<unset> (identity transform)\n",
    prog);
}

bool parse_bool(const char* s) {
  std::string v = s;
  for (auto& c : v) c = static_cast<char>(std::tolower(c));
  return (v == "true" || v == "1" || v == "on" || v == "yes");
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ur10e_teleop_unilateral_vive_cpp::ViveLeaderNode::Options opts;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* n) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", n);
        std::exit(2);
      }
      return argv[++i];
    };
    if      (a == "--robot")          opts.robot_type = need("--robot");
    else if (a == "--config")         opts.config_path = need("--config");
    else if (a == "--calib")          opts.calib_path = need("--calib");
    else if (a == "--tracker-serial") opts.tracker_serial = need("--tracker-serial");
    else if (a == "--rate-hz")        opts.control_rate_hz = std::stod(need("--rate-hz"));
    else if (a == "--rt-mode")        opts.use_rt = parse_bool(need("--rt-mode"));
    else if (a == "--rt")             opts.use_rt = true;
    else if (a == "--no-rt")          opts.use_rt = false;
    else if (a == "--rt-priority")    opts.rt_priority = std::atoi(need("--rt-priority"));
    else if (a == "--rt-cpu")         opts.rt_cpu = std::atoi(need("--rt-cpu"));
    else if (a == "--help" || a == "-h") { print_usage(argv[0]); return 0; }
    else if (a.rfind("--ros-args", 0) == 0) break;
  }

  if (opts.use_rt) {
    ur10e_teleop_unilateral_vive_cpp::lock_process_memory();
  }

  auto node =
      std::make_shared<ur10e_teleop_unilateral_vive_cpp::ViveLeaderNode>(opts);
  if (!node->run()) {
    std::fprintf(stderr, "vive_leader: run() failed (tracker init?)\n");
    rclcpp::shutdown();
    return 3;
  }
  rclcpp::spin(node);
  node->stop();
  rclcpp::shutdown();
  return 0;
}
