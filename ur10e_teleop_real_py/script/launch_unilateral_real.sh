#!/usr/bin/env bash
# Copyright 2025 UR10E Migration Project
# Licensed under the Apache License, Version 2.0
#
# Launch unilateral teleop on REAL UR10E hardware via RTDE.
#
# Usage:
#   launch_unilateral_real.sh <leader_ip> <follower_ip> [freq_hz]

set -euo pipefail

LEADER_IP="${1:-192.168.1.10}"
FOLLOWER_IP="${2:-192.168.1.11}"
FREQ="${3:-500}"

INSTALL="${HOME}/colcon_ws/install"
BINARY="${INSTALL}/ur10e_teleop_real/lib/ur10e_teleop_real/unilateral_control"

if [ ! -f "${BINARY}" ]; then
  echo "[launch_unilateral_real] ERROR: Binary not found at ${BINARY}"
  echo "  Run: colcon build --packages-select ur10e_teleop_real"
  exit 1
fi

echo "[launch_unilateral_real] leader  IP : ${LEADER_IP}"
echo "[launch_unilateral_real] follower IP : ${FOLLOWER_IP}"
echo "[launch_unilateral_real] frequency   : ${FREQ} Hz"

URDF_DIR="/tmp/ur10e_urdf_gen"
mkdir -p "${URDF_DIR}"

XACRO_FILE="${INSTALL}/ur10e_description/share/ur10e_description/urdf/robot/ur10e.urdf.xacro"

xacro "${XACRO_FILE}" robot_ip:="${LEADER_IP}"   prefix:="leader_"   ros2_control:=false \
  > "${URDF_DIR}/ur10e_leader.urdf"
xacro "${XACRO_FILE}" robot_ip:="${FOLLOWER_IP}" prefix:="follower_" ros2_control:=false \
  > "${URDF_DIR}/ur10e_follower.urdf"

cleanup() {
  rm -f "${URDF_DIR}/ur10e_leader.urdf" "${URDF_DIR}/ur10e_follower.urdf"
}
trap cleanup EXIT

echo "[launch_unilateral_real] Launching unilateral control..."
"${BINARY}" "${LEADER_IP}" "${FOLLOWER_IP}" "${FREQ}"
