#!/usr/bin/env bash
# install.sh — Install dependencies for ur10e_teleop_real_py.
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

echo "[install] Installing Python dependencies..."
pip3 install --user numpy pynput python-xlib pyyaml

echo "[install] Installing ROS2 packages..."
sudo apt install -y \
  "ros-${DISTRO}-sensor-msgs" \
  "ros-${DISTRO}-std-msgs" \
  "ros-${DISTRO}-launch" \
  "ros-${DISTRO}-ament-index-python"

echo "[install] Done."
echo ""
echo "=== Build ==="
echo "  cd ~/colcon_ws && colcon build --packages-select ur10e_teleop_real_py"
echo ""
echo "=== Run (dummy mode — no hardware) ==="
echo "  ros2 launch ur10e_teleop_real_py teleop_dummy.launch.py"
echo ""
echo "=== Verify which robot is at which IP FIRST ==="
echo "  python3 script/identify_robots.py --ips <IP_A> <IP_B>"
echo ""
echo "=== Run (real UR — two robots; launch defaults set to project network) ==="
echo "  ros2 launch ur10e_teleop_real_py teleop_real.launch.py"
echo "  # or override IPs:"
echo "  ros2 launch ur10e_teleop_real_py teleop_real.launch.py leader_ip:=<UR3e_IP> follower_ip:=<UR10e_IP>"
echo ""
echo "=== After homing, switch to ACTIVE ==="
echo "  ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray \"data: [0.0, 0.0, 0.0]\""
