// main.cpp — oa_fd_node entry point.

#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "oa_fd_cpp/rt_thread.hpp"
#include "oa_fd_cpp/teleop_node.hpp"

using oa_fd::TeleopNode;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  TeleopNode::Options opts;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--config")            opts.config_path = next();
    else if (a == "--urdf")          opts.urdf_override = next();
    else if (a == "--urdf-leader")   opts.urdf_leader_override = next();
    else if (a == "--urdf-follower") opts.urdf_follower_override = next();
    else if (a == "--arms")         opts.arms = next();   // right|left|both
    else if (a == "--role")         opts.role = next();   // leader|follower|both
    else if (a == "--rt-mode")      opts.use_rt = (next() == "true");
    else if (a == "--rt")           opts.use_rt = true;
    else if (a == "--no-rt")        opts.use_rt = false;
    else if (a == "--rt-priority")  opts.rt_priority = std::stoi(next());
    else if (a == "--rt-cpu")       opts.rt_cpu = std::stoi(next());
  }

  if (opts.use_rt) oa_fd::lock_process_memory();

  std::shared_ptr<TeleopNode> node;
  try {
    node = std::make_shared<TeleopNode>(opts);
    if (!node->connect()) {
      RCLCPP_FATAL(node->get_logger(), "connect() failed — aborting");
      rclcpp::shutdown();
      return 1;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  node->run();
  rclcpp::spin(node);
  node->stop();
  rclcpp::shutdown();
  return 0;
}
