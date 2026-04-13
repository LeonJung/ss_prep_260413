#!/usr/bin/env bash
# Copyright 2025 UR10E Migration Project
# Licensed under the Apache License, Version 2.0
#
# Run the MuJoCo bilateral teleop stack (shm-based):
#   1. leader_sim.py    — MuJoCo window 1 (leader arm, keyboard + drag)
#   2. follower_sim.py  — MuJoCo window 2 (follower arm + box, always UR10e)
#
# Each sim reads the other's state directly via /dev/shm — no separate
# bilateral_control binary needed.
#
# Usage:
#   exec_bilateral_sim.sh [leader_robot]
#     leader_robot : ur10e (default) | ur3e

set -euo pipefail

LEADER_ROBOT="${1:-ur10e}"

case "${LEADER_ROBOT}" in
  ur10e|ur3e) ;;
  *)
    echo "[exec_bilateral_sim] ERROR: leader_robot must be 'ur10e' or 'ur3e' (got '${LEADER_ROBOT}')"
    exit 1
    ;;
esac

INSTALL="${HOME}/colcon_ws/install"
SRC_DIR="${INSTALL}/ur10e_teleop_mujoco/share/ur10e_teleop_mujoco/src"
XML_DIR="${INSTALL}/ur10e_teleop_mujoco/share/ur10e_teleop_mujoco/xml"
FOLLOWER_MODEL="${XML_DIR}/ur10e_follower_scene.xml"

echo "[exec_bilateral_sim] SIM mode (shm-based, bilateral built-in)"
echo "[exec_bilateral_sim]   leader_robot : ${LEADER_ROBOT}"
echo "[exec_bilateral_sim]   follower     : ur10e (fixed)"

cleanup_sim() {
  echo "[exec_bilateral_sim] Stopping simulation processes..."
  kill "${LEADER_PID:-}" "${FOLLOWER_PID:-}" 2>/dev/null || true
}
trap cleanup_sim EXIT

# Launch leader simulation (window 1)
python3 "${SRC_DIR}/leader_sim.py" --robot "${LEADER_ROBOT}" &
LEADER_PID=$!
echo "[exec_bilateral_sim] Leader sim PID: ${LEADER_PID}"

# Launch follower simulation (window 2) — always UR10e
python3 "${SRC_DIR}/follower_sim.py" --model "${FOLLOWER_MODEL}" &
FOLLOWER_PID=$!
echo "[exec_bilateral_sim] Follower sim PID: ${FOLLOWER_PID}"

echo "[exec_bilateral_sim] Both sims running. Press Ctrl+C to stop."
wait
