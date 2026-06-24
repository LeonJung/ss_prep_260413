# openarmx_bilateral — STATUS

Bilateral force-feedback teleop between two openarmx bimanual robots
(leader + follower), all via **ros2_control MIT** driver control (NOT raw CAN).
Test arm = **LEFT only** for now (everything defaults to left; no arm_side typing).

Operator workflow: bring up the two robots yourself, PLAIN (no enable_forward_effort) —
leader NON-namespaced, follower `namespace:=follower` — then run ONE launch.

---

## Phases (each A/B tested on HW with logged data)

| Phase | What | Result | Toggle |
|---|---|---|---|
| **1 Gravity comp** | follower (+leader) gravity via gravity_comp_node → effort ctrl | **ADOPTED** — follower tracking err −48%, sag removed | always ON |
| **2 Velocity FF** | relay peer joint velocity → forward_velocity_controller (MIT vel) | **ADOPTED** — dynamic lag −50% | `vel_ff:=true` |
| **3 Friction comp** | `+ scale·Fc·tanh(0.1·k·ω)` added to effort, per robot | **ADOPTED** — clearly lighter; scale tuning pending | `friction:=true` |
| **Final Bilateral FF** | two-way cross-relay + leader force-reflection kp | **TUNING in progress** | `bilateral:=true leader_kp:=…` |

Identified friction is **Coulomb-dominated (Fv≈0)**; per-robot Fc/k baked into
`bilateral.launch.py` (LEADER_FC/K, FOLLOWER_FC/K). enactic confirmed friction on
BOTH arms (leader.yaml + follower.yaml).

Known issue: **friction comp startup tremor** at ~zero velocity (steep tanh on
velocity noise, worst on J1). Not felt during active manipulation. Fix ready:
`vel_eps` deadband / velocity low-pass (tune later). scale 0.7 current.

---

## Commands ([제어 PC]; build: `colcon build --packages-select openarmx_bilateral`)

Unilateral teleop (P1 only, leader free):
```
ros2 launch openarmx_bilateral bilateral.launch.py
```
+ velocity FF + friction comp:
```
ros2 launch openarmx_bilateral bilateral.launch.py vel_ff:=true friction:=true
```
**Bilateral force feedback (P1+P2+P3 + two-way):**
```
ros2 launch openarmx_bilateral bilateral.launch.py \
  bilateral:=true vel_ff:=true friction:=true leader_kp:=10.0 leader_kd:=0.5
```

Tuning knobs (launch args, no rebuild): `friction_scale` (lightness),
`leader_kp` (force-reflection strength), `leader_kd` (damping/stability),
`follower_kp`/`follower_kd` (follower tracking; '' = HW default), `couple_sign` (+1 HW-verified).

---

## Nodes (src/)
- `relay_node` — cross-relay pos (+vel when vel_ff); bilateral toggle.
- `friction_comp_node` — sits between gravity_comp and effort ctrl: `out = grav + scale·Fc·tanh(0.1k·ω)`. Single publisher; openarmx package untouched.
- `gravity_comp_node` — official (openarmx_gravity_comp), output routed to `grav_only`.
- `log_node` — follower cmd-vs-actual CSV (P1/P2 A/B).
- `friction_log_node` — per-joint vel + effort breakdown (meas/cmd/grav/fric) for P3 A/B.
- `friction_id_node` — friction identification: shuttle each joint at constant speeds, log `fric = eff − grav` (schedule ALWAYS 1..nj_, built directly).
- `joint_echo_node` — temp helper, per-joint angles both arms.

## Identification recap
`friction_id.launch.py target:=follower|leader` → CSV → offline fit
`τ_fric(ω)=Fc·tanh(0.1·k·ω)` (odd-symmetrized to cancel gravity residual).

## Backups
- v1_best (unilateral works): tag `openarmx_bilateral_v1_best`
- v1.1 (one-shot launch + P1+P2): tag `openarmx_bilateral_v1.1`
- **v2.0 (P1+P2+P3 adopted, bilateral FF tuning): tag `openarmx_bilateral_v2.0`**
