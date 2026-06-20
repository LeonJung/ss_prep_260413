# openarmx ros2_control vs our oa_fd_cpp — why theirs is smooth (2026-06-20)

Operator ran OFFICIAL openarmx unilateral on the SAME hardware (PC/robot/USB2CAN):
super smooth, no q3~7 popping, all 4 arms fine — while our custom stack had
tremor/stiffness/popping with even 1 leader. So the gap is our code/approach,
not HW. Read of the official source (openarmx_ros2 + openarmx_description):

## What the official stack actually is
- **ros2_control**, `update_rate: 100 Hz` (controller_manager).
- Follower = `forward_position_controller` (ForwardCommandController) → streams
  POSITION only.
- Leader backdrive = `openarmx_gravity_comp` node → publishes GRAVITY torques to
  `forward_effort_controller`. **No friction comp at all.** tau_limits
  {20,20,7,7,2,2,2}.
- HW interface `openarmx_hardware/OpenArmX_v10HW` (v10_simple_hardware.cpp),
  `control_mode:=mit`.

## How they use the MIT controller (the crux)
In `write()` (called at 100 Hz) the HW interface sends ONE MIT packet per motor:
```
MITParam{ kp, kd, position=cmd, velocity=cmd(0 for fwd-pos), torque=cmd(0) }
kp = {50,50,50,50,10,10,10}      // big joints 50, wrist 10
kd = {2.5,2.5,2.5,2.5,0.5,0.5,0.5}
```
Then `recv_all(1000)`. That's it. **The Robstride motor's onboard MIT loop
(high internal rate) closes the position/velocity loop; the PC only streams
setpoints at 100 Hz with torque=0.** They TRUST the motor.

## Side-by-side
| aspect | openarmx (smooth) | ours (tremor/stiff) |
|---|---|---|
| who runs the fast loop | the **motor** (onboard MIT, ~kHz); PC streams 100Hz | **PC** at 1000Hz tries to be the loop |
| MIT kp (big / wrist) | 50 / 10 | 120 / 16-20 |
| MIT kd (big / wrist) | 2.5 / 0.5 | 2.0 / 0.2 |
| **kd/kp ratio** | **~0.05 (well damped)** | **~0.017 big, ~0.011 wrist (3-5× UNDER-damped)** |
| torque FF (follower) | **0** (pure position MIT) | gravity+friction every cycle via CAN |
| torque FF (leader) | gravity ONLY (no friction) | gravity+friction+gates |
| philosophy | minimal PC, trust motor | PC micromanages through delayed CAN |

## Why this makes theirs smooth & ours not
1. **Damping.** kd/kp ≈ 0.05 vs our ≈0.011-0.017. Our gains are stiff AND
   under-damped → the ~4 Hz hold tremor + ringing. Theirs is soft + damped.
2. **Let the motor close the loop.** 100 Hz setpoint streaming + the Robstride's
   internal high-rate MIT loop = smooth interpolation. Our 1000 Hz PC FF rides
   the CAN delay → the non-passivity (our own D2) that trembles.
3. **Less PC torque.** Follower torque=0 (motor does it); leader gravity-only.
   Our extra friction-comp FF + gates inject noise/drag through the delay and
   barely helped (friction-gate test felt "similar").

## Implications / directions (to discuss — dev paused)
- **Cheap win**: re-tune our MIT gains toward theirs — much higher kd/kp (e.g.
  Kp 50, Kd 2.5 big; Kp 10, Kd 0.5 wrist). Likely kills tremor + softens stiff.
- **Question PC-side friction FF**: theirs omits it and is smooth. Maybe drop/
  shrink ours.
- **Bigger**: build our BILATERAL on top of the official ros2_control HW
  interface (proven-smooth layer) instead of raw single-process CAN. Bilateral
  = cross-feed two arms' position/effort through controllers. Keeps their motor-
  trust model; we add force reflection.
- Note: bilateral still needs SOME coupling stiffness for force feedback, but at
  their damped gains. The lesson is gains + trust-the-motor, not architecture
  alone.

## Misc facts noticed
- gravity vec in link0 = (0, ±9.81, 0) [Y] for them (rpy ±1.5708 mount) — ours
  uses Z because of our URDF base frame; not a bug, just frame convention.
- per-arm separate ros2_control `system` (left/right), each its own CAN iface —
  same as our per-arm split.
- dir multipliers per motor (sign) — same as ours.
- They ship a gripper stall-detect in write(); irrelevant to arm smoothness.
