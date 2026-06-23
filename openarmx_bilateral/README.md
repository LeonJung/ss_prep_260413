# openarmx_bilateral — OpenArmX bilateral force-feedback teleop

Topic-only cross-relay built on the official **ros2_control MIT** stack (the
standard openarmx control method — the HW interface does CAN/MIT; this node only
moves joint positions between the two robots). References the unilateral teleop
(`openarmx_teleop_bimanual`) one-way relay and generalizes it to bidirectional.

- `bilateral:=false` (default) → **UNILATERAL (step b)**: follower-left tracks
  leader-left. Leader is gravity-float (its bringup runs gravity_comp + soft kp).
- `bilateral:=true` → also follower→leader → **force feedback** from the leader
  HW MIT kp (set leader kp soft via the kp_joint param service).

Defaults are **LEFT arm only** (no arm_side typing needed).

## Architecture (all ros2_control)
```
Leader robot bringup (can1=left)             Follower robot bringup (can3=left)
  forward_position_controller  <--leader_cmd-- relay --follower_cmd--> forward_position_controller
  forward_effort_controller (gravity_comp)                            (+ optional gravity_comp)
        ▲ kp soft (force fb, bilateral only)        ▲ kp normal (tracking)
```
Force feedback = the HW MIT kp coupling on each arm toward the relayed peer
position (no DOB / energy tank — none exist upstream either).

## Step (b): left UNILATERAL — bring-up
1. Leader-left bringup (can1), gravity comp on, kp soft so it's hand-movable.
2. Follower-left bringup (can3), forward_position_controller.
   (Bring up the two robots in namespaces /leader and /follower — see openarmx
   bringup `arm_prefix`/namespacing; right arm can be left unpowered for now.)
3. URDF for gravity_comp if used: `/tmp/v10_bimanual.urdf` (xacro v10 bimanual).
4. Relay (unilateral):
```bash
ros2 launch openarmx_bilateral relay.launch.py        # left, bilateral:=false
```
Verify: move leader-left by hand → follower-left tracks smoothly (q1-q7). Check
the topic names with `ros2 topic list` and override leader_states/follower_states
/leader_cmd/follower_cmd to match your bringup namespaces.

## Step (c): bilateral force feedback
```bash
ros2 launch openarmx_bilateral relay.launch.py bilateral:=true
# leader kp soft for force feel, e.g.:
ros2 param set /leader/openarmx_left_hardware_params kp_joint1 15.0   # ...joint2..7
```
Block the follower by hand → the leader should resist (force feedback). Tune the
leader kp_joint* (soft = lighter + weaker FF; higher = stronger FF).

## Params
| param | default | meaning |
|-------|---------|---------|
| arm_side | left_arm | left_arm / right_arm (sets joint prefix + default topics) |
| bilateral | false | false=unilateral, true=add follower→leader (force fb) |
| couple_sign | -1.0 | leader↔follower position sign (openarmx relay convention) |
| leader_states / follower_states | /leader//follower /joint_states | state topics |
| leader_cmd / follower_cmd | …/left_forward_position_controller/commands | command topics |
| rate_hz | 200 | relay publish rate |

Notes: relay has NO openarmx_can dependency (pure ROS topics) — builds anywhere;
needs the official openarmx bringup running at runtime. couple_sign / topic names
may need to match your exact bringup namespacing — verify with `ros2 topic list`.
