# ur10e_teleop_real_cpp

C++ implementation of bilateral force-feedback teleoperation for real UR robots,
intended to take advantage of a PREEMPT_RT kernel for tight control-loop timing.

Sibling of [`ur10e_teleop_real_py`](../ur10e_teleop_real_py/) — same topic
namespace, same control semantics, same config format. Only one of the two
packages is launched at a time.

## Current status: **Phase 2 skeleton**

- Package builds.
- `leader_node` / `follower_node` executables start but do nothing functional yet
  (no robot connection, no control loop — just a jitter-tracked RT-sleep loop).
- `rt_thread` utilities implemented: `init_rt_thread`, `lock_process_memory`,
  `sleep_until` (CLOCK_MONOTONIC), `JitterTracker`.
- Config copied from `_py` (`config/real_ur.yaml`).
- `resources/external_control.urscript` copied from ur_client_library — will be
  templated by the driver at connect time.

Phases remaining: 3 (RT toggle already hooked up), 4 (bilateral control law
porting from `_py`), 5 (rclcpp wiring + ur_client_library integration),
7 (RT-vs-nonRT comparison tooling), 8/9 (README expansion, build helpers).

## Dependencies

- ROS2 Jazzy: `rclcpp`, `sensor_msgs`, `std_msgs`
- `ur_client_library` (source at `/home/bpearson/install_ws/Universal_Robots_Client_Library`)
- `Eigen3`
- `yaml-cpp`
- C++17

## Build

```bash
cd ~/colcon_ws
colcon build --packages-select ur10e_teleop_real_cpp
source install/setup.bash
```

## RT mode — permissions

To actually enter SCHED_FIFO and mlockall, the binaries need the right caps:

```bash
sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep \
    install/ur10e_teleop_real_cpp/lib/ur10e_teleop_real_cpp/leader_node
sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep \
    install/ur10e_teleop_real_cpp/lib/ur10e_teleop_real_cpp/follower_node
```

Without caps the executables still run, but `--rt-mode true` will print a
warning and fall back to normal scheduling.

## RT vs non-RT benchmark

Standalone timing test (no ROS, no robot — just the cyclic sleep loop):

```bash
# before `setcap`: both runs fall through to normal scheduling
python3 script/rt_comparison.py --duration 30

# one-time: grant caps so RT actually engages
sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep \
    install/ur10e_teleop_real_cpp/lib/ur10e_teleop_real_cpp/jitter_benchmark
sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep \
    install/ur10e_teleop_real_cpp/lib/ur10e_teleop_real_cpp/leader_node
sudo setcap cap_sys_nice+ep,cap_ipc_lock+ep \
    install/ur10e_teleop_real_cpp/lib/ur10e_teleop_real_cpp/follower_node

# then:
python3 script/rt_comparison.py --duration 30 --save-csv /tmp/jit
python3 script/rt_comparison.py --duration 30 --plot   # needs matplotlib
```

What you'd expect to see (order-of-magnitude):

| metric           | normal kernel     | PREEMPT_RT + caps |
|------------------|-------------------|-------------------|
| period mean      | ≈ target          | ≈ target          |
| p99              | target + 100-500µs| target + 50-100µs |
| p99.9            | target + 1-5 ms   | target + 100-300µs|
| worst-case (max) | target + 5-50 ms  | target + 100-500µs|

Direct binary:

```bash
jitter_benchmark --rt-mode true --duration 30 --period-us 2000 > run.csv
```

Stdout = per-cycle CSV, stderr = jitter summary.

## Launch (placeholder — control loop TODO)

```bash
# both nodes on one PC, non-RT
ros2 launch ur10e_teleop_real_cpp teleop_real.launch.py

# both nodes on one PC, RT
ros2 launch ur10e_teleop_real_cpp teleop_real.launch.py rt:=true

# distributed: leader on PC A
ros2 launch ur10e_teleop_real_cpp teleop_real_leader.launch.py rt:=true
# distributed: follower on PC B
ros2 launch ur10e_teleop_real_cpp teleop_real_follower.launch.py rt:=true
```

## CLI args

Both executables accept:

| flag | default | meaning |
|---|---|---|
| `--robot-ip IP` | UR3e: `.94` / UR10e: `.92` | robot's IP |
| `--robot ur3e\|ur10e` | `ur3e` / `ur10e` | robot type (affects torque limits) |
| `--config PATH` | — | YAML config path |
| `--rt-mode true\|false` | `false` | enable SCHED_FIFO + mlockall |
| `--rt` / `--no-rt` | — | shortcut for true/false |
| `--rt-priority N` | `80` | SCHED_FIFO priority (1..99) |
| `--rt-cpu N` | `-1` | pin control thread to CPU core |
