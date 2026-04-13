#!/usr/bin/env bash
# sim_install.sh — Install Python dependencies for ur10e_teleop_mujoco
#
# Usage:
#   bash sim_install.sh

set -euo pipefail

echo "[sim_install] Installing Python dependencies..."

pip3 install --user mujoco numpy pynput python-xlib

echo "[sim_install] Installing ROS2 message packages..."
sudo apt install -y \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-launch \
  ros-jazzy-ament-index-python

echo "[sim_install] Done."
echo ""
echo "To build the package:"
echo "  cd ~/colcon_ws && colcon build --packages-select ur10e_teleop_mujoco"
echo ""
echo "To run (shm mode):"
echo "  source ~/colcon_ws/install/setup.bash"
echo "  exec_bilateral_sim.sh [ur10e|ur3e]"
echo ""
echo "To run (ROS2 mode):"
echo "  source ~/colcon_ws/install/setup.bash"
echo "  ros2 launch ur10e_teleop_mujoco teleop_sim.launch.py robot:=ur3e"
