# oa_fd_cpp

**OpenArm A2 (openarmx) bimanual force-feedback BILATERAL — enactic-style control.**

openarmx Robstride/CAN driver (MIT mode) grafted with the control law ported
from **enactic `openarm_teleop`**: per-joint motor-side MIT impedance plus
gravity + friction feedforward, with cross-coupled leader/follower references.

## Topology (single control PC, 4 USB2CAN) — identical to `oa_pp_cpp`

```
  RIGHT: leader can0 <-> follower can2      LEFT: leader can1 <-> follower can3
```

## Control law (per joint, both arms)

```
tau_motor = Kp·(q_ref − q) + Kd·(dq_ref − dq) + g(q) + friction(q̇)
  friction(v) = Fc·tanh(k·v) + Fv·v + Fo
  ACTIVE   : q_ref,dq_ref = mirrored peer state   (leader_ref=follower, follower_ref=leader)
  PAUSED   : q_ref = home, dq_ref = 0
  HOMING   : q_ref = quintic ramp (start→home)
  FREEDRIVE: Kp=Kd=0  → tau = g(q)+friction  (backdrivable)
```

The `Kp·Δq + Kd·Δq̇` impedance is executed **motor-side** (sent in the MIT
packet's kp/kd/pos/vel fields); `g(q)+friction` ride in the torque-feedforward
field. This mirrors enactic's `bilateral_step` exactly, but with the OpenArmX
CAN library swapped in for enactic's Damiao CAN I/O.

### vs `oa_pp_cpp`
| | oa_pp_cpp | oa_fd_cpp |
|---|---|---|
| force feedback | implicit (pos-pos spring) | explicit (symmetric MIT impedance + FF) |
| impedance | PC computes pure torque | motor-side (MIT kp/kd) |
| feedforward | gravity | gravity + friction |
| source law | ur10e_teleop_real_cpp | enactic openarm_teleop |
| rate (default) | 500 Hz | 1 kHz |

## Prerequisites / Build / CAN setup
Identical to `oa_pp_cpp` (see its README): install `openarmx-can_1.0.0.deb`,
bring up `can0..can3`, set the URDF path in `config/oa_fd.yaml: gravity.urdf`.

```bash
colcon build --packages-select oa_fd_cpp
source install/setup.bash
sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep \
     install/oa_fd_cpp/lib/oa_fd_cpp/oa_fd_node
```

## Run / test
```bash
ros2 launch oa_fd_cpp oa_fd.launch.py            # auto-homes -> PAUSED
ros2 topic echo /oa/leader_right/joint_state
# modes [m,t,dur]: 0=ACTIVE 1=PAUSED 2=HOMING 3=FREEDRIVE
ros2 topic pub --once /oa/mode std_msgs/msg/Float64MultiArray "{data: [0,0,0]}"
```

### Bring-up order (safety)
1. **FREEDRIVE** → tune `gravity.scale` / `gravity.vec` so the arm floats.
2. Identify **friction** (Fc/k/Fv/Fo) per joint, enable gradually.
3. **PAUSED** → confirm hold-at-home with low `Kp`, raise until stiff & stable.
4. One pair only, **ACTIVE**, low `Kp` → operator moves leader, follower mirrors
   and reflects contact force. Check `mirror` signs.
5. Raise `Kp/Kd`, enable RT.

## ⚠️ Status / caveats
- **Not compiled here** — build/test on the control PC (openarmx-can + CAN hw).
- **Gains/friction are placeholders** — enactic's are Damiao-identified; Robstride
  needs its own tuning + friction identification.
- **MIT run-mode** assumption same as `oa_pp_cpp` (see its README).
- 1 kHz × 4 USB2CAN may not be sustainable; watch `[DIAG]` period and raise
  `timestep` if needed.
- Gripper (8th DOF) excluded (7-DOF arm only).
