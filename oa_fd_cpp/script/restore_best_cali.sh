#!/usr/bin/env bash
# restore_best_cali.sh — regenerate the BEST left-leader gravity calibration
# (the COM-only fit the user validated as best: link7 mass stays 0.625,
#  after-mean ≈ 0.875 Nm) and commit it to the repo for permanent keeping.
#
# Run on the CONTROL PC (needs /tmp/gravity_cali_left3.csv).
#   bash ~/git_ws/ss_prep_260413/oa_fd_cpp/script/restore_best_cali.sh
set -e

REPO=~/git_ws/ss_prep_260413
WS=~/colcon_ws
CSV=/tmp/gravity_cali_left3.csv
OUT=$REPO/oa_fd_cpp/urdf/openarmx_arm_cali_left_leader.urdf

[ -f "$CSV" ] || { echo "ERROR: $CSV not found (rerun oa_gravity_cali first)"; exit 1; }

python3 $WS/src/oa_fd_cpp/script/fit_gravity.py \
    --csv "$CSV" \
    --urdf $WS/src/oa_fd_cpp/urdf/openarmx_arm_v2com.urdf \
    --out  "$OUT" \
    --fit-links 1,2,3,4,5 --drop-joints 6,7

echo
echo ">> CHECK the output above: 'openarmx_link7: m 0.625 -> 0.625' and"
echo ">> 'after mean' ~= 0.875. If so, this is the validated-best version."
echo

cd $REPO
git add oa_fd_cpp/urdf/openarmx_arm_cali_left_leader.urdf
git commit -m "left leader calibrated URDF (best: COM-only fit)"
git push origin main
echo "DONE — preserved in repo. Launch with:"
echo "  ros2 launch oa_fd_cpp oa_fd.launch.py arms:=left role:=leader \\"
echo "    urdf:=\$HOME/colcon_ws/install/oa_fd_cpp/share/oa_fd_cpp/urdf/openarmx_arm_cali_left_leader.urdf"
