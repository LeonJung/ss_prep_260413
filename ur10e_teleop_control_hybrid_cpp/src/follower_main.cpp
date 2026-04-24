// follower_main.cpp — entry point for the follower executable.

#include <cstring>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ur10e_teleop_control_hybrid_cpp/follower_node.hpp"
#include "ur10e_teleop_control_hybrid_cpp/rt_thread.hpp"

namespace {

void print_usage(const char* prog) {
  std::fprintf(stderr,
    "Usage: %s [--robot-ip IP] [--robot ur10e|ur3e] [--config PATH]\n"
    "         [--rt-mode true|false] [--rt-priority N] [--rt-cpu N]\n"
    "Defaults: robot-ip=169.254.186.92  robot=ur10e  rt-mode=false\n",
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

  ur10e_teleop_control_hybrid_cpp::FollowerNode::Options opts;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* n) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", n);
        std::exit(2);
      }
      return argv[++i];
    };
    if      (a == "--robot-ip")    opts.robot_ip = need("--robot-ip");
    else if (a == "--robot")       opts.robot_type = need("--robot");
    else if (a == "--config")      opts.config_path = need("--config");
    else if (a == "--resources-dir") opts.resources_dir = need("--resources-dir");
    else if (a == "--rt-mode")     opts.use_rt = parse_bool(need("--rt-mode"));
    else if (a == "--rt")          opts.use_rt = true;
    else if (a == "--no-rt")       opts.use_rt = false;
    else if (a == "--rt-priority") opts.rt_priority = std::atoi(need("--rt-priority"));
    else if (a == "--rt-cpu")      opts.rt_cpu = std::atoi(need("--rt-cpu"));
    else if (a == "--help" || a == "-h") { print_usage(argv[0]); return 0; }
    else if (a.rfind("--ros-args", 0) == 0) break;
  }

  if (opts.use_rt) {
    ur10e_teleop_control_hybrid_cpp::lock_process_memory();
  }

  auto node = std::make_shared<ur10e_teleop_control_hybrid_cpp::FollowerNode>(opts);
  node->run();
  rclcpp::spin(node);
  node->stop();
  rclcpp::shutdown();
  return 0;
}
