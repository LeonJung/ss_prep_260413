# Gravity / Friction comp — calibration status (snapshot 2026-06-20)

Per-arm config is 4 self-contained files: `config/oa_fd_{leader,follower}_{left,right}.yaml`.
Gravity URDFs: `urdf/openarmx_arm_cali_{left,right}_{leader,follower}.urdf`.

## Status matrix
| arm | CAN | gravity comp | friction comp | end (link7) mass | notes |
|-----|-----|--------------|---------------|------------------|-------|
| leader-left   | can1 | ✅ identified (cali) | ✅ identified | 0.625 (handle) | working / smooth |
| leader-right  | can0 | ✅ identified (cali) | ✅ identified | 0.625 (handle) | working; q1/q2/q3/q5 good |
| follower-left | can3 | = leader-left model | = leader-left (transplanted) | 0.625 | params copied from leader-left (only CAN differs) |
| follower-right| can2 | = leader-right model | = leader-right (transplanted) | **experimental** | params copied from leader-right; link7 mass being probed |

## follower-right end-mass probe (open)
- URDF `openarmx_arm_cali_right_follower.urdf` link7 mass tried: 1.5× → **overcomp**;
  0.5× (skipped report); **currently 1.1×** (0.687469). COM unchanged.
- Goal: measure the multiple instead of guessing — via q7 ±sweep ratio
  (operator: "q7로 재야 해", q7 bidirectional OK). See TODO below.

## Key confirmed facts (do not relitigate)
- **Gravity model is accurate on all 4** (mirrored cali residual ~0.1 Nm; right
  needs joint-axis mirror `m=[-1,-1,-1,1,-1,1,-1]`, eval must mirror q/τ too).
- **follower ≈ leader arm** (gripper gravity contribution ≈ noise) — D20.
- **scaling banned** — fix via cali/measurement, never a fudge scale (D-history).
- **q7 cali "팡팡" = stick-slip**, not a regression, not mass; q7 gravity ≈ 0
  (COM on roll axis). Cali-only artifact (friction-comp OFF + weak Kp). See
  CALI_NOTES.md / DECISIONS D24.
- **follower coast in single-role = damping absence**, not over-comp (D23). Real
  bilateral has Kd=2 so it won't coast.

## TODO — follower gravity/friction comp (parked to do bilateral teleop first)
1. **follower-right end (link7) mass via q7 ± ratio**:
   - leader-right (can0) and follower-right (can2) each one q7 sweep, SEQUENTIAL.
   - feed both CSVs to `script/tmp_lf_motor_ratio.py --joint 7` → follower/leader
     ratio = link7 mass multiple → set in `urdf/openarmx_arm_cali_right_follower.urdf`.
   - (NOTE earlier finding: q7 gravity signal is tiny; if ratio R² is low, fall
     back to q6 lever method. Operator wants q7 first.)
2. follower-left end mass likewise if it diverges from leader-left.
3. Re-confirm follower gravity/friction feel after the mass is set (the
   "transplanted = leader" values are a starting point, not final).
4. Optional: per-joint leader↔follower motor-output ratio (q1/q2/q4) if any joint
   still mismatches after mass is right.

## Run commands (control PC) — q7 ± sweep, leader then follower
```bash
cd ~/git_ws/ss_prep_260413 && git pull origin main
rsync -a --delete --exclude=build --exclude=install --exclude=log \
      ~/git_ws/ss_prep_260413/oa_fd_cpp/ ~/colcon_ws/src/oa_fd_cpp/
cd ~/colcon_ws && colcon build --packages-select oa_fd_cpp && source install/setup.bash

URDF=~/colcon_ws/install/oa_fd_cpp/share/oa_fd_cpp/urdf/openarmx_arm_cali_right_leader.urdf
# leader-right FIRST (can0), then follower-right (can2) — NEVER both at once (CAN bw)
ros2 run oa_fd_cpp oa_friction_cali --can can0 --side right --urdf $URDF --joint 7 --out /tmp/q7_leadR.csv
ros2 run oa_fd_cpp oa_friction_cali --can can2 --side right --urdf $URDF --joint 7 --out /tmp/q7_follR.csv
```
Send both CSVs → I compute follower/leader ratio = follower link7 mass multiple
and write it into the follower-right URDF.

## Temporary diagnostic tools (delete when cali done)
- `oa_friction_cali_v0` — eab5a3d baseline (A/B for 팡팡; proved q7 stick-slip
  is pre-existing). src `friction_cali_v0_main.cpp`.
- `cmd()` loop-rate print in `oa_friction_cali` — measured ~195 Hz steady.
- `script/tmp_lf_motor_ratio.py` — leader/follower ratio estimator.
