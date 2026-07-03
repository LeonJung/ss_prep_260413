# driver_patches — patched upstream openarmx_hardware

`v10_simple_hardware.cpp` here is a **full patched copy** of the upstream driver file so
it can be swapped in directly (the upstream lives in a separate workspace, not this git
repo). Patches applied (details: `../GRIPPER_FRICTION_DRIVER_PATCH.md`,
`../LATENCY_LEVER_DRIVER_PATCH.md`):

1. **Gripper velocity** (read, ~L523): `vel_states_[ARM_DOF] = gripper_motors[0]->get_velocity();`
   (was `0`) — needed for gripper friction comp.
2. **Gripper torque** (write, ~L646): `gripper_param.torque = tau_commands_[ARM_DOF];`
   (was `0.0`) — feeds gripper effort/friction FF.
3. **Latency lever** (read, ~L483): `openarmx->refresh_all();` commented out — drop the
   redundant status-request round-trip.
4. **Latency lever** (write, ~L678): `openarmx->recv_all(1000);` commented out — feedback
   is drained at the top of the next read() (pipelined I/O).

## Install (control PC)
```bash
cp ~/git_ws/ss_prep_260413/openarmx_bilateral/driver_patches/v10_simple_hardware.cpp \
   ~/openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp
cd ~/openarmx_ws && colcon build --packages-select openarmx_hardware && source install/setup.bash
```
(Back up the original first if you want an easy revert:
`cp .../src/v10_simple_hardware.cpp /tmp/v10_simple_hardware.orig.cpp`)

## ⚠️ Caveat — full-file copy can go stale
This is a whole-file copy, not a patch. If upstream openarmx changes this file, swapping
this copy in will REVERT those upstream changes. Before swapping, `diff` against the current
upstream file to confirm the only differences are the 4 patches above:
```bash
diff ~/openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp \
     ~/git_ws/ss_prep_260413/openarmx_bilateral/driver_patches/v10_simple_hardware.cpp
```
If upstream moved on, re-apply the 4 edits by hand (they're small) instead of copying.
