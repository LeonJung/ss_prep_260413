#!/usr/bin/env bash
# install.sh — Install dependencies for ur10e_teleop_real
#
# Usage:
#   bash install.sh

set -euo pipefail

echo "[install] Installing Python dependencies..."
pip3 install --user numpy pynput python-xlib pyyaml

echo "[install] Installing ROS2 packages..."
sudo apt install -y \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-launch \
  ros-jazzy-ament-index-python

echo "[install] Done."
echo ""
echo "=== Build ==="
echo "  cd ~/colcon_ws && colcon build --packages-select ur10e_teleop_real"
echo ""
echo "=== Run (dummy mode — no hardware) ==="
echo "  ros2 launch ur10e_teleop_real teleop_dummy.launch.py"
echo ""
echo "=== Run (real UR — two robots) ==="
echo "  ros2 launch ur10e_teleop_real teleop_real.launch.py leader_ip:=192.168.1.100 follower_ip:=192.168.1.101"
echo ""
echo "=== Run (real UR — single robot manual) ==="
echo "  ros2 run ur10e_teleop_real leader_real_node.py --client rtde --robot-ip 192.168.1.100 --config real_ur.yaml"
echo "  ros2 run ur10e_teleop_real follower_real_node.py --client rtde --robot-ip 192.168.1.101 --config real_ur.yaml"
