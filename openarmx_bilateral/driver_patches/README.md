# driver_patches — upstream openarmx_hardware edits

The openarmx driver lives in a separate workspace (not this git repo). We do NOT vendor the
whole file — versions differ per machine (e.g. some have `node_namespace_`, some don't), so a
full-file swap breaks the build. Apply the small edits by hand instead.

File: `~/openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp`

## Two independent edit sets

### A) Gripper friction comp — PARKED (see tech_debt.md §11 TODO)
Not part of the current work. Apply only when resuming gripper tuning.
Full before/after: `../GRIPPER_FRICTION_DRIVER_PATCH.md`
1. read() ~L515: `vel_states_[ARM_DOF] = gripper_motors[0]->get_velocity();` (was `0`)
2. write() ~L635: `gripper_param.torque = tau_commands_[ARM_DOF];` (was `0.0`) —
   the write() one only, NOT return_to_zero()'s.

### B) Latency lever — ACTIVE (pipeline CAN I/O, expect CM 75/86 -> ~130-150 Hz)
Full before/after + test protocol: `../LATENCY_LEVER_DRIVER_PATCH.md`
3. read() ~L475: comment out `openarmx->refresh_all();`
4. write() ~L664: comment out `openarmx->recv_all(1000);`

## Build
```bash
cd ~/openarmx_ws && colcon build --packages-select openarmx_hardware && source install/setup.bash
```

## Revert
B: uncomment the two lines. A: restore `0` / `0.0`. Or `git checkout` the file in
`~/openarmx_ws/src/openarmx_ros2` (it is a git repo).
