# oa_pp_cpp

**OpenArm A2 (openarmx) bimanual BILATERAL teleoperation — position-position control.**

openarmx Robstride/CAN driver (MIT mode) grafted with the position-position
bilateral + continuous-deadband control law ported from `ur10e_teleop_real_cpp`.

## Topology (single control PC, 4 USB2CAN)

```
            RIGHT pair                         LEFT pair
  leader_right (can0) ──┐              leader_left (can1) ──┐
                        │ pos-pos                           │ pos-pos
  follower_right(can2) ─┘              follower_left(can3) ─┘
```

One process, one RT control thread, reads all 4 arms each cycle, computes two
independent bilateral couplings, writes pure-torque MIT commands to all 4.

## Control law (per pair)

```
leader (operator arm):
  ACTIVE : τ = g(q) + ramp·sign(raw)·max(0, |raw| − DEADBAND)
           raw = KP_BI·(f_mirrored − l) + KD_BI·(ḟ_mirrored − l̇)
  PAUSED : τ = g(q) + KP_HOLD·(home − q) − KD_HOLD·q̇
follower (environment arm):
  ACTIVE : τ = g(q) + KP_TRACK·(l_mirrored − f) + KD_TRACK·(l̇_mirrored − ḟ)
HOMING   : both arms PD to quintic-ramped target + g(q)
FREEDRIVE: τ = g(q)   (gravity-balanced, backdrivable)
```

Force feedback is **implicit** (position-divergence spring), exactly like
`ur10e_teleop_real_cpp`. **Difference vs UR:** openarmx has no firmware gravity
compensation, so `g(q)` is computed on the PC (KDL from URDF) and added.

## Prerequisites (on the control PC)

1. **openarmx-can library** (provides headers + `libopenarmx_can.so` +
   CMake package `openarmx_can`):
   ```bash
   sudo dpkg -i openarmx-can_1.0.0_amd64.deb   # from openarmx_ros2/
   ```
2. **4 CAN interfaces up** (USB2CAN). Classic CAN 1 Mbit example:
   ```bash
   for c in can0 can1 can2 can3; do
     sudo ip link set $c up type can bitrate 1000000
   done
   ip -details link show can0
   ```
   (Use `dbitrate` + `fd on` and set `can.fd: true` in the yaml if running CAN-FD.)
3. **ROS 2** (Humble/Jazzy) + `orocos_kdl_vendor kdl_parser urdf yaml-cpp`.
4. **A single-arm OpenArm A2 URDF** — set its absolute path in
   `config/oa_pp.yaml: gravity.urdf` (REQUIRED, else the arm sags).

## Build

```bash
cd <ws>
colcon build --packages-select oa_pp_cpp
source install/setup.bash
# RT caps (re-run after every build):
sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep \
     install/oa_pp_cpp/lib/oa_pp_cpp/oa_pp_node
```

## Run / test sequence (on the control PC, arms clear & e-stop in reach)

```bash
# 0. CAN up (above), URDF path set, gains reviewed.
# 1. Launch (start NON-RT first to verify, then rt:=true)
ros2 launch oa_pp_cpp oa_pp.launch.py
#    -> auto-homes both pairs to `home`, then PAUSED.

# 2. Watch state
ros2 topic echo /oa/leader_right/joint_state
ros2 topic echo /oa/mode

# 3. Modes (Float64MultiArray [mode, t_start, duration]):
#    3=FREEDRIVE first to confirm gravity comp holds the arm:
ros2 topic pub --once /oa/mode std_msgs/msg/Float64MultiArray "{data: [3,0,0]}"
#    re-home if needed:
ros2 topic pub --once /oa/mode std_msgs/msg/Float64MultiArray "{data: [2, 0, 5]}"
#    go bilateral:
ros2 topic pub --once /oa/mode std_msgs/msg/Float64MultiArray "{data: [0,0,0]}"
#    pause:
ros2 topic pub --once /oa/mode std_msgs/msg/Float64MultiArray "{data: [1,0,0]}"
```

### Recommended bring-up order (safety)
1. **FREEDRIVE** with low/zero gains → confirm `g(q)` alone holds the arm
   (tune `gravity.scale`, check `gravity.vec` sign vs mounting).
2. **PAUSED** → confirm each arm holds `home` without oscillation (tune KP/KD_HOLD).
3. One pair only (unplug/disable the other), **ACTIVE** at low KP_BI/KP_TRACK,
   move leader by hand → follower should track; pushing follower → felt on leader.
4. Verify `mirror.{right,left}` signs (a joint moving the wrong way = flip sign).
5. Raise gains; enable RT (`rt:=true rt_cpu:=N`).

## ⚠️ Status / caveats

- **Not compiled in this workspace** — built/tested only on the control PC that
  has `openarmx-can` + CAN hardware. Treat as first-cut; expect gain re-tuning.
- **MIT run-mode assumption:** `OaxArm::enable()` does
  `set_callback_mode_all(STATE)` + `enable_all()`, assuming motors power up in
  MIT/MOTION_CONTROL. If a unit ships in CSP, add a run-mode switch (cf.
  `openarmx_hardware/src/v10_simple_hardware.cpp:on_activate`).
- **Gains are placeholders** (Robstride ≠ Damiao/UR). Re-tune per joint.
- **4×USB2CAN at 500 Hz** may saturate; if `[DIAG]` shows period >> target,
  drop `timestep` to 0.004 (250 Hz).
- Gripper (8th DOF) excluded by design (7-DOF arm only).

## Sibling
`oa_fd_cpp` — same driver/topology, but enactic-style control (gravity +
friction feedforward + motor-side MIT impedance, cross-coupled).
