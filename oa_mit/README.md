# oa_mit — bilateral force-feedback teleop on the openarmx MIT layer

Built **on top of** the proven openarmx stack (no raw single-process control of
our own). The leader runs the official Mode-2 weightless gravity-comp recipe
(MIT `kp=0, kd=0, torque = g_scale·gravity`) and relays its position to the
follower's `forward_position_controller`. **Added:** a force-reflection coupling
so the leader feels the follower.

## How force feedback works (D33: motor-side MIT coupling)
The leader node subscribes to the follower's `/joint_states` and servoes the
leader toward the (delayed) follower position **via the motor's own MIT loop**:
```
leader MIT = { kp=couple_kp, kd=couple_kd, position = s·q_follower(delayed),
               velocity = 0, torque = gravity(+hold) }
```
The motor closes `couple_kp·(q_follower − q_leader)` on FRESH local encoder q
(delay-free, passive); only the follower setpoint is delayed (benign — the
follower side already does delayed-setpoint MIT and is smooth). This is the
convergent enactic / working-UR10e symmetric position-position law, NOT a
PC-side torque spring (v1 did that and vibrated). No DOB, no energy tank.
- **Free space**: the follower tracks the leader, so `q_follower ≈ q_leader` →
  coupling ≈ 0 → leader stays **weightless** (= Mode 2).
- **Contact**: the follower can't reach `q_leader` and lags → the term grows
  and **opposes** the leader's motion → the operator feels resistance.

Sign verified: the relay sends the follower `−leader_motor_pos`; after the
follower HW dir-mapping, `q_follower(/joint_states) ≈ q_leader(joint frame)` in
free space, so `(q_follower − q_leader)` is a passive (negative-feedback)
restoring term.

`couple_kp = 0.0` (default) ⇒ **identical to Mode 2** (weightless, no FF). Raise
gradually. Lessons from oa_fd: tremor came from high stiffness (Kp=120) over a
delayed channel — start LOW (couple_kp ≈ 5–10) and rely on the motor's MIT loop.

## Prereqs (same as openarmx Mode 2)
1. Leader URDF at `/tmp/v10_bimanual.urdf`:
   ```bash
   xacro <ws>/src/openarmx_description/urdf/robot/v10.urdf.xacro \
         arm_type:=v10 bimanual:=true > /tmp/v10_bimanual.urdf
   ```
2. Follower bringup (can2/can3, position control):
   ```bash
   ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
        right_can_interface:=can2 left_can_interface:=can3 \
        control_mode:=mit robot_controller:=forward_position_controller
   ```

## Run
```bash
# weightless (== Mode 2; confirm parity first)
ros2 launch oa_mit oa_mit_bilateral.launch.py couple_kp:=0.0
# add force feedback, tune up
ros2 launch oa_mit oa_mit_bilateral.launch.py couple_kp:=8.0
```

## Params (per arm, set via launch)
| param | default | meaning |
|-------|---------|---------|
| `couple_kp` | 0.0 | force-reflection gain (0 = weightless Mode 2) |
| `couple_kd` | 0.0 | coupling damping on leader velocity |
| `couple_tau_limit` | 8.0 | per-joint clamp on the coupling torque [Nm] |
| `g_scale` (right/left) | 0.9 / 0.8 | gravity scale (<1 lighter) |
| `kp_hold` | 0.0 | position-hold stiffness when settled |
| `kd_damp` | 0.0 | global leader damping |
| `gdir` | (0,∓9.81,0) | gravity in link0 (Y axis per mount) |

## Provenance / license
`src/oa_mit_bilateral_node.cpp` is derived from, and `src/dynamics.cpp` +
`include/dynamics.hpp` are copied verbatim from,
`openarmx/openarmx_teleop_bimanual` (Chengdu Changshu Robot Co., Ltd),
licensed **CC-BY-NC-SA-4.0**. This package keeps that license; research /
non-commercial use. The force-reflection coupling is the local addition.
