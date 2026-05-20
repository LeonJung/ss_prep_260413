# ur10e_teleop_unilateral_vive_cpp

Vive-tracker-driven unilateral (one-way) teleoperation for the UR10e.
The operator holds a Vive tracker; the new `vive_leader_node` polls its
6-DoF pose via SteamVR/OpenVR, applies a calibrated tracker→UR-base
transform, solves UR3e inverse kinematics, and publishes the resulting
joint state on the same topics the existing UR3e leader did. The
follower from the sibling `ur10e_teleop_control_unilateral_cpp`
package is copied in unchanged so the package is self-contained.

```
[Vive tracker] ──OpenVR──► vive_leader_node ──/ur10e/leader/joint_state──► follower_node ──► UR10e
                                                  (+ /ur10e/mode latched)
```

This is a drop-in replacement for the UR3e leader: same topic schema,
same 4-mode state machine (ACTIVE / PAUSED / HOMING / FREEDRIVE),
same QoS, same RT thread infrastructure. Only the source of `q` and
`dq` changes — synthesized from tracker pose via IK instead of read
from a UR3e via RTDE.

## Pipeline (per cycle, 500 Hz default)

```
ViveTracker.poll()              # raw 4×4 pose in SteamVR tracking frame
   └→ Calibration.apply()       # rigid transform to UR base frame
        └→ ur_ik_solve()        # iterative DLS-Jacobian IK (warm-started)
             └→ publish_state(q, dq)
```

`dq` is numerical-diff of `q` with an exponential low-pass
(`dq_filter_alpha`, default 0.2) — Vive's high-rate tracking is noisy at
mm scale, and the LPF protects the follower's KD term from amplifying it.

### Modes

| mode      | output                                              |
|-----------|-----------------------------------------------------|
| ACTIVE    | IK(tracker_pose)                                    |
| FREEDRIVE | same as ACTIVE (no UR3e to put in freedrive)        |
| PAUSED    | hold last `q`, `dq = 0`                             |
| HOMING    | quintic interpolate current_q → `leader_home` over `homing_duration` |

Reset (`/ur10e/reset`) from `ACTIVE` → `HOMING`. Subject to a
2 s startup grace.

## Dependencies

Same ROS2 stack as the parent unilateral package, plus OpenVR:

```bash
sudo apt install -y libopenvr-dev
```

OpenVR is only needed at the leader-PC build time. The follower PC
doesn't need it.

For the leader PC at runtime: SteamVR must be running with a paired
tracker in the room (Vive base stations + tracker powered on and seen
by the SteamVR status window).

## Build

```bash
cd ~/colcon_ws
colcon build --packages-select ur10e_teleop_unilateral_vive_cpp
source install/setup.bash
```

`--symlink-install` and the `setcap_rt.sh` caveats from the parent
package apply identically here — run `bash script/setcap_rt.sh` after
each rebuild if you intend to use `--rt-mode true`.

## Calibration

The tracker pose lives in SteamVR's room-scale frame (Y-up). The UR
base frame is Z-up at the robot mount. The calibration step solves
for the rigid transform between them.

A 3-point capture helper will live at `script/calibrate.py` (TODO).
The flow is:

1. Place the tracker at 3 known UR base-frame positions
   (e.g. (0, 0, 0.5), (0.3, 0, 0.5), (0, 0.3, 0.5)).
2. The helper records tracker pose at each.
3. It calls `Calibration::solve_from_points` (Umeyama / Kabsch) and
   writes the resulting 4×4 to `config/calibration.yaml`.

If `calibration.yaml` is missing the leader runs with an identity
transform — useful for end-to-end pipeline tests but does not give a
meaningful pose mapping.

## Launch

Single PC (vive leader + follower on the same host):

```bash
ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real.launch.py
ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real.launch.py \
    leader_rt:=true follower_rt:=true
```

Distributed (vive leader on PC A, follower on PC B):

```bash
# PC A (Vive + SteamVR)
ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_leader.launch.py \
    rt:=true tracker_serial:=LHR-XXXXXXXX

# PC B (UR10e)
ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_follower.launch.py \
    rt:=true
```

`tracker_serial` is optional — if empty, the leader picks the first
generic tracker SteamVR reports.

### Mode topic

After homing completes, switch to ACTIVE:

```bash
ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0]"
```

Modes: 0=ACTIVE · 1=PAUSED · 2=HOMING · 3=FREEDRIVE.

## CLI args (`vive_leader_node`)

| flag | default | meaning |
|---|---|---|
| `--robot ur3e\|ur10e\|ur5e` | `ur3e` | IK target frame (must match follower's mirror_sign) |
| `--config PATH`             | —      | YAML config (shared with parent package) |
| `--calib PATH`              | —      | YAML with `T_ur_from_tracker` (4×4) |
| `--tracker-serial S`        | —      | Specific tracker; empty = first found |
| `--rate-hz F`               | 500    | Control loop rate |
| `--rt-mode true\|false`     | false  | SCHED_FIFO + mlockall on the control thread |
| `--rt-priority N`           | 80     | SCHED_FIFO priority (1..99) |
| `--rt-cpu N`                | -1     | Pin control thread to CPU N |

## Relationship to other packages

- **`ur10e_teleop_control_unilateral_cpp`** — origin. Same follower,
  same state machine, same config schema. The Vive leader replaces
  its UR3e leader.
- **`ur10e_teleop_real_cpp`** (bilateral) — referenced for mature
  patterns (state machine, soft-start, FK/Jacobian module). Not
  imported as a build dep.

## TODO

- [ ] **`script/calibrate.py`** — interactive 3-point capture script
      that prompts for tracker positions and writes `calibration.yaml`.
- [ ] **Closed-form 8-branch UR IK** — current iterative DLS can pick
      a different branch from the operator's intent at workspace edges.
      Swap to analytical IK with branch selection (Hawkins).
- [ ] **Tracker frame visualization** — optional rviz marker pub for
      live pose so the operator can see what the leader sees.
- [ ] **Multi-tracker support** — second tracker for gripper / stereo
      input. Currently single-tracker only.
