#!/usr/bin/env bash
# logging.sh — Record all ROS2 topics and system state during teleop.
#
# Creates a timestamped log directory with:
#   - leader/follower joint_state CSV
#   - mode transitions CSV
#   - rosout (all node logs) + filtered warnings
#   - topic hz measurements
#   - config snapshot
#   - real-time monitor output
#
# Usage:
#   bash logging.sh [log_dir] [config_yaml]
#
# Run this WHILE teleop is already running in another terminal.
# Press Ctrl+C to stop recording.

set -euo pipefail

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "${SCRIPT_DIR}")"
LOG_DIR="${1:-${PKG_DIR}/log/teleop_${TIMESTAMP}}"
CONFIG_FILE="${2:-}"

mkdir -p "${LOG_DIR}"
echo "[logging] Log directory: ${LOG_DIR}"
echo "[logging] Press Ctrl+C to stop recording."

# ---------------------------------------------------------------------------
# Config snapshot
# ---------------------------------------------------------------------------
if [ -n "${CONFIG_FILE}" ] && [ -f "${CONFIG_FILE}" ]; then
    cp "${CONFIG_FILE}" "${LOG_DIR}/config_snapshot.yaml"
    echo "[logging] Config saved: ${CONFIG_FILE}"
fi

# System info
echo "Date: $(date)" > "${LOG_DIR}/system_info.txt"
echo "Hostname: $(hostname)" >> "${LOG_DIR}/system_info.txt"
echo "User: $(whoami)" >> "${LOG_DIR}/system_info.txt"
echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}" >> "${LOG_DIR}/system_info.txt"

# ---------------------------------------------------------------------------
# Snapshot: node list + topic list + topic hz
# ---------------------------------------------------------------------------
echo "[logging] Taking system snapshot..."
ros2 node list > "${LOG_DIR}/node_list.txt" 2>&1 || true
ros2 topic list > "${LOG_DIR}/topic_list.txt" 2>&1 || true

timeout 3 ros2 topic hz /ur10e/leader/joint_state > "${LOG_DIR}/hz_leader.txt" 2>&1 &
timeout 3 ros2 topic hz /ur10e/follower/joint_state > "${LOG_DIR}/hz_follower.txt" 2>&1 &
timeout 3 ros2 topic hz /ur10e/mode > "${LOG_DIR}/hz_mode.txt" 2>&1 &
wait

echo "[logging] Snapshot saved. Starting continuous recording..."

# ---------------------------------------------------------------------------
# Continuous recording
# ---------------------------------------------------------------------------
PIDS=()

ros2 topic echo /ur10e/leader/joint_state --csv \
  > "${LOG_DIR}/leader_state.csv" 2>&1 &
PIDS+=($!)

ros2 topic echo /ur10e/follower/joint_state --csv \
  > "${LOG_DIR}/follower_state.csv" 2>&1 &
PIDS+=($!)

ros2 topic echo /ur10e/mode --csv \
  > "${LOG_DIR}/mode.csv" 2>&1 &
PIDS+=($!)

# Full rosout
ros2 topic echo /rosout --csv \
  > "${LOG_DIR}/rosout.csv" 2>&1 &
PIDS+=($!)

# Filtered warnings/errors only
ros2 topic echo /rosout --csv 2>/dev/null | \
  grep -i --line-buffered "WARN\|ERROR\|OVER-FORCE\|PAUSED\|HOMING\|FAIL" \
  > "${LOG_DIR}/warnings_only.csv" 2>&1 &
PIDS+=($!)

# Run monitor (terminal + file)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "${SCRIPT_DIR}/log_monitor.py" "${LOG_DIR}" 2>&1 | \
  tee "${LOG_DIR}/monitor_output.log" &
PIDS+=($!)

# ---------------------------------------------------------------------------
# Wait for Ctrl+C
# ---------------------------------------------------------------------------
cleanup() {
  echo ""
  echo "[logging] Stopping recorders..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null

  echo "[logging] Recording summary:"
  for f in leader_state.csv follower_state.csv mode.csv rosout.csv warnings_only.csv monitor_output.log; do
    if [ -f "${LOG_DIR}/${f}" ]; then
      LINES=$(wc -l < "${LOG_DIR}/${f}")
      echo "  ${f}: ${LINES} lines"
    fi
  done

  echo "[logging] Logs saved to: ${LOG_DIR}"
  echo "[logging] To share for debugging:"
  echo "  zip -r teleop_logs.zip ${LOG_DIR}"
}

trap cleanup EXIT
wait
