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
ros2 launch oa_fd_cpp oa_fd.launch.py                          # all 4 arms
ros2 launch oa_fd_cpp oa_fd.launch.py arms:=left role:=leader  # one arm only
# per-role URDFs (leader = handle tip, follower = gripper tip):
ros2 launch oa_fd_cpp oa_fd.launch.py \
    urdf_leader:=$HOME/git_ws/ss_prep_260413/oa_fd_cpp/urdf/openarmx_arm_cali_left_leader.urdf
# modes [m,t,dur]: 0=ACTIVE 1=PAUSED 2=HOMING 3=FREEDRIVE (startup = FREEDRIVE)
ros2 topic pub --once /oa/mode std_msgs/msg/Float64MultiArray "{data: [0,0,0]}"
```
Launch args: `arms:=right|left|both`, `role:=leader|follower|both` (single
role disables ACTIVE coupling — gravity-only), `urdf:=` (common fallback),
`urdf_leader:=` / `urdf_follower:=`, `rt:= rt_priority:= rt_cpu:=`.

## Tools (calibration & diagnostics)

| tool | purpose |
|---|---|
| `oa_diag [canX ...] [--enable]` | per-motor link/health: responding, temp, RUN pattern, error codes |
| `oa_gravity_cali` | static gravity ID: pose grid (collision-checked `--poses` file), two-sided approach (stiction cancel), trapezoid moves + friction FF (`--config`) |
| `oa_friction_cali` | per-joint constant-velocity sweeps -> friction samples |
| `fit_gravity.py` | per-link COM fit (masses pinned). Key flags: `--fit-links`, `--drop-joints 6,7` (wrist torque telemetry is broken: ~8Nm static), `--friction-csv` (ingest sweeps as extra gravity data, +v/-v bin-averaged, per-joint bias absorbs Fo), `--fit-mass-links` (use sparingly — overfits) |
| `fit_friction.py` | tanh friction fit, Fc/Fv>=0 enforced, Fo capped; prints yaml block |
| `gen_cali_poses.py` | random all-joint poses with self-collision / torso / mount checks |
| `restore_best_cali.sh` | one-command regen+commit of the validated-best left-leader URDF |

Calibration recipe (per arm): friction cali -> fit -> paste yaml -> gravity
cali -> `fit_gravity.py --csv ... --friction-csv ... --fit-links 1,2,3,4,5
--drop-joints 6,7` -> commit URDF -> launch with `urdf_leader:=`/`urdf_follower:=`.

### Bring-up order (safety)
1. `oa_diag` — all motors responding, no error codes.
2. Launch (starts in **FREEDRIVE**) -> verify gravity float at several poses.
3. PAUSED -> holds the captured pose (no slam). 4. One pair ACTIVE, low Kp.
5. Check `mirror` signs, raise gains, enable RT.

## ⚠️ Status / caveats
- **Not compiled here** — build/test on the control PC (openarmx-can + CAN hw).
- **Gains/friction are placeholders** — enactic's are Damiao-identified; Robstride
  needs its own tuning + friction identification.
- **MIT run-mode** assumption same as `oa_pp_cpp` (see its README).
- 1 kHz × 4 USB2CAN may not be sustainable; watch `[DIAG]` period and raise
  `timestep` if needed.
- Gripper (8th DOF) excluded (7-DOF arm only).
