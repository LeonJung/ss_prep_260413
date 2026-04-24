#!/usr/bin/env bash
# install.sh — Install dependencies for ur10e_teleop_control_hybrid_cpp.
#
# Auto-detects ROS distro from $ROS_DISTRO (set when you sourced
# /opt/ros/<distro>/setup.bash). Defaults to 'jazzy' if unset.
#
# Usage:
#   source /opt/ros/<your-distro>/setup.bash   # humble | jazzy | rolling ...
#   bash script/install.sh

set -euo pipefail

DISTRO="${ROS_DISTRO:-jazzy}"
echo "[install] ROS distro: ${DISTRO}"

echo "[install] Installing apt packages..."
sudo apt update
sudo apt install -y \
    "ros-${DISTRO}-rclcpp" \
    "ros-${DISTRO}-sensor-msgs" \
    "ros-${DISTRO}-std-msgs" \
    "ros-${DISTRO}-ament-cmake" \
    "ros-${DISTRO}-ur-client-library" \
    libeigen3-dev \
    libyaml-cpp-dev \
    libcap2-bin

echo "[install] Done."
echo ""
echo "=== Build (example colcon_ws at ~/colcon_ws) ==="
echo "  cd ~/colcon_ws && colcon build --packages-select ur10e_teleop_control_hybrid_cpp"
echo "  source install/setup.bash"
echo ""
echo "=== Grant RT capabilities (one-time per rebuild) ==="
echo "  bash src/ur10e_teleop_control_hybrid_cpp/script/setcap_rt.sh"
echo ""
echo "=== Jitter benchmark ==="
echo "  python3 src/ur10e_teleop_control_hybrid_cpp/script/rt_comparison.py --duration 30 --rt-cpu 2 --load"
echo ""
echo "=== Real bilateral ==="
echo "  ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real.launch.py leader_rt:=true follower_rt:=true"
