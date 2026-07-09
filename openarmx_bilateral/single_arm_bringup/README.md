# Single-arm bringup — bring up ONE arm (left OR right) for openarmx_bilateral

Purpose: run openarmx_bilateral (old ros2_control system) with a SINGLE arm, to isolate
whether the high-kp stability came from shared-memory (new port) vs single-arm (fewer active
CAN channels / less USB-hub contention). The stock `openarmx.bimanual.launch.py` always brings
up BOTH arms; this adds a single-arm path that keeps LEFT/RIGHT naming (so openarmx_bilateral's
relay/controllers still match).

> ⚠️ These are **upstream openarmx** changes (in `~/openarmx_ws`, NOT this git repo). Apply them
> on the control PC. Verified on the dev box: URDF generates single-arm (1 hardware block),
> bimanual output functionally unchanged (only 2 XML comments differ). ros2_control spawn +
> hardware NOT tested (needs the robot).

## Part A — 3 xacro edits (thread enable_left/enable_right through)
Defaults are `true` → **the normal bimanual bringup is unaffected.**

### 1) `openarmx_description/urdf/ros2_control/openarmx.bimanual.ros2_control.xacro`
- Add to the macro `params`: `enable_left:=^|true  enable_right:=^|true`
- Move the `configure_joint` sub-macro definition to the TOP of the macro body (before the
  left `<ros2_control>` block) so it stays defined when a block is disabled.
- Wrap the left block:  `<xacro:if value="${enable_left}"> <ros2_control name="openarmx_left_hardware_interface" ...> ... </ros2_control> </xacro:if>`
- Wrap the right block similarly with `${enable_right}`.
- (A ready copy of the full edited file is not vendored to avoid drift; the edit is small.)

### 2) `openarmx_description/urdf/robot/openarmx_robot.xacro`
- Macro params (near `bimanual:=false`): add `enable_left:='true'  enable_right:='true'`
- In the `<xacro:openarmx_bimanual_ros2_control ...>` call add:
  `enable_left="${enable_left}"  enable_right="${enable_right}"`

### 3) `openarmx_description/urdf/robot/v10.urdf.xacro`
- Near `<xacro:arg name="bimanual" .../>` add:
  `<xacro:arg name="enable_left" default="true"/>` and `<xacro:arg name="enable_right" default="true"/>`
- In the `<xacro:openarmx_robot ...>` call (near `bimanual="$(arg bimanual)"`) add:
  `enable_left="$(arg enable_left)"  enable_right="$(arg enable_right)"`

## Part B — the launch (this dir → openarmx_bringup)
```bash
cp openarmx.single.launch.py ~/openarmx_ws/src/openarmx_ros2/openarmx_bringup/launch/
cd ~/openarmx_ws && colcon build --packages-select openarmx_description openarmx_bringup && source install/setup.bash
```

## Usage (single-arm bringup for openarmx_bilateral)
Bring up leader single-arm + follower single-arm (2 CAN channels total instead of 4):
```bash
# T1 leader, RIGHT arm only (leader right = can0)
ros2 launch openarmx_bringup openarmx.single.launch.py \
  arm_side:=right right_can_interface:=can0 control_mode:=mit robot_controller:=forward_position_controller
# T2 follower, RIGHT arm only (follower right = can2)
ros2 launch openarmx_bringup openarmx.single.launch.py \
  arm_prefix:=follower arm_side:=right right_can_interface:=can2 control_mode:=mit robot_controller:=forward_position_controller
# T3 openarmx_bilateral, right only
ros2 launch openarmx_bilateral bilateral.launch.py arm:=right bilateral:=true friction:=true vel_ff:=true leader_kp:=60
```
(For left arm: `arm_side:=left left_can_interface:=can1` / follower `can3`, and `arm:=left`.)

## What it does
- URDF: `enable_left/enable_right` = one side only → **only that arm's ros2_control hardware is
  created → only that CAN opens.** Other arm's geometry still shown in RViz (harmless).
- Spawns: `joint_state_broadcaster` + `<side>_forward_position_controller` only. effort/velocity/
  gravity/friction come from openarmx_bilateral's bilateral.launch (as usual).
- Now only 2 CAN channels active (leader+follower of one side) → isolates USB-hub contention.
