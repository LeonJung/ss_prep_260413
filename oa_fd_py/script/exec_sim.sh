#!/usr/bin/env bash
# Launch leader + follower viewer processes (shm-coupled bilateral sim).
# Usage: bash script/exec_sim.sh
set -e
cd "$(dirname "$0")/.."
python3 src/oa_fd_sim.py --role follower &
FPID=$!
sleep 1
python3 src/oa_fd_sim.py --role leader
kill $FPID 2>/dev/null || true
