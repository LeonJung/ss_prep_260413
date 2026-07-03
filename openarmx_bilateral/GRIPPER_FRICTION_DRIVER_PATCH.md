# Gripper (joint8) friction comp — required upstream driver patches

Friction compensation for the gripper needs TWO one-line edits in the **upstream**
openarmx driver (NOT in this repo, so record here — a fresh openarmx clone loses them):

File: `~/openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp`

### 1) read(): give the gripper a real velocity (was hardcoded 0)
The friction model is velocity-based `Fc*tanh(0.1*k*w)`; with velocity 0 the gripper
term is always 0. (~line 515)
```cpp
- vel_states_[ARM_DOF] = 0;  // gripper_motors[0]->get_velocity();
+ vel_states_[ARM_DOF] = gripper_motors[0]->get_velocity();   // motor frame
```

### 2) write(): feed the gripper effort command interface (was hardcoded 0)
So the friction/torque feedforward actually reaches the gripper. (~line 635, MIT path)
```cpp
- gripper_param.torque   = 0.0;
+ gripper_param.torque   = tau_commands_[ARM_DOF];
```

### Build + test
```bash
cd ~/openarmx_ws && colcon build --packages-select openarmx_hardware && source install/setup.bash
# then relaunch bringup + bilateral (friction:=true) as usual
```

### Notes
- Velocity is in MOTOR frame; torque command is also motor frame, so the sign is
  self-consistent (friction FF assists motor motion = cancels friction). If the gripper
  feels HEAVIER instead of lighter, negate one side (e.g. GRIP_FC in bilateral.launch.py,
  or the torque line above).
- Strength: `GRIP_FC` / `GRIP_K` in `bilateral.launch.py` (rough placeholders, not
  friction_id'd). Increase GRIP_FC if still too stiff.
- Package side (this repo) already handles the 8th joint: friction_comp_node
  (gripper_joint param + max(n_joints,gravity) output), {left,right}_effort_controller.yaml
  (finger_joint1 last), bilateral.launch.py (N_JOINTS=8).
