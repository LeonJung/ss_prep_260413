# ur10e_teleop_real_cpp

C++ implementation of bilateral force-feedback teleoperation for real UR
robots. Uses `ur_client_library` for the UR I/O layer and supports
PREEMPT_RT kernel scheduling (SCHED_FIFO + mlockall) for tight control-loop
timing.

Sibling of [`ur10e_teleop_real_py`](../ur10e_teleop_real_py/) — same topic
namespace, same control semantics, same config schema. Only one of the
two packages is launched at a time.

## Status

Feature-complete port of `_py`:

| feature                                  | _py | _cpp |
|------------------------------------------|-----|------|
| 4-mode state machine (ACTIVE/PAUSED/HOMING/FREEDRIVE) | ✅ | ✅ |
| Bilateral PD + continuous deadband       | ✅ | ✅ |
| ACTIVE-entry soft-start ramp (0.5 s)     | ✅ | ✅ |
| Quintic-spline auto-homing               | ✅ | ✅ |
| Reset counter → HOMING (with startup grace) | ✅ | ✅ |
| Per-robot torque limits                  | ✅ | ✅ |
| Joint mirror transform                   | ✅ | ✅ |
| firmware friction_comp via URScript      | ✅ | ✅ |
| 1 Hz [DIAG] status log                   | ✅ | ✅ |
| PREEMPT_RT scheduling (SCHED_FIFO + mlockall) | —  | ✅ |
| Jitter benchmark utility                 | —  | ✅ |
| tau_contact (F/T-derived) + OVER-FORCE   | —  | — (both TODO) |

## Dependencies

```bash
source /opt/ros/<your-distro>/setup.bash   # humble | jazzy | rolling | ...
bash script/install.sh
```

Auto-detects `ROS_DISTRO` so the same script works on Humble, Jazzy, etc.

Installs:
- `ros-<DISTRO>-{rclcpp,sensor-msgs,std-msgs,ament-cmake,ur-client-library}`
- `libeigen3-dev`, `libyaml-cpp-dev`, `libcap2-bin`

## Build

```bash
cd <your_ws>
colcon build --packages-select ur10e_teleop_real_cpp
source install/setup.bash
```

### ⚠️ `--symlink-install` caveat

`colcon build --symlink-install` places a SYMLINK at
`install/.../<binary>` pointing into `build/`. Linux's `setcap` refuses
symlinks with `"filename must be a regular file"`. The helper
`setcap_rt.sh` compensates by calling `readlink -f` and applying caps to
the real build-side file, but after every rebuild the build-side file
is overwritten and caps are lost — **re-run `setcap_rt.sh` after every
build** (you need to anyway; see below).

If you prefer, build that one package without symlink-install:
```bash
colcon build --packages-select ur10e_teleop_real_cpp  # no --symlink-install
```
Other packages in the workspace can still use `--symlink-install`; only
this one needs a real binary for setcap.

## RT capabilities (one-time per build)

```bash
bash script/setcap_rt.sh
```

Paths are auto-discovered from `ros2 pkg prefix`, so it works regardless
of your workspace name or ROS distro — just source the workspace first.

Options:
- `bash script/setcap_rt.sh leader_node`   — specific binary
- `bash script/setcap_rt.sh --clear`        — remove caps (debug)

**Re-run after every `colcon build`** — rebuilt binaries lose their caps.

Without caps the executables still run, but `--rt-mode true` prints a
warning and falls back to normal scheduling.

## Launch

### Single PC (both nodes)

```bash
# non-RT
ros2 launch ur10e_teleop_real_cpp teleop_real.launch.py

# both nodes RT
ros2 launch ur10e_teleop_real_cpp teleop_real.launch.py \
    leader_rt:=true follower_rt:=true

# only leader RT (mixed environment)
ros2 launch ur10e_teleop_real_cpp teleop_real.launch.py \
    leader_rt:=true follower_rt:=false
```

### Distributed (leader on PC A, follower on PC B)

```bash
# PC A — UR3e side
ros2 launch ur10e_teleop_real_cpp teleop_real_leader.launch.py rt:=true

# PC B — UR10e side
ros2 launch ur10e_teleop_real_cpp teleop_real_follower.launch.py rt:=true
```

Each PC chooses its own `rt:=true|false` independently, so a setup
where only one of the two PCs has the PREEMPT_RT kernel works cleanly.

### Mode topic (from anywhere)

After homing completes, switch to bilateral:
```bash
ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0]"
```

Modes: 0=ACTIVE · 1=PAUSED · 2=HOMING · 3=FREEDRIVE.

## RT vs non-RT benchmark

Standalone timing test (no ROS, no robot — just the cyclic sleep loop):

```bash
bash script/setcap_rt.sh                      # caps granted once
python3 script/rt_comparison.py --duration 30 --rt-cpu 2
python3 script/rt_comparison.py --duration 30 --rt-cpu 2 --load
python3 script/rt_comparison.py --duration 30 --plot   # matplotlib
```

`--load` spawns `N-1` busy processes so the RT advantage surfaces;
`--rt-cpu N` pins the RT thread to a specific core.

Direct binary (CSV to stdout, summary to stderr):
```bash
jitter_benchmark --rt-mode true --duration 30 --period-us 2000 > run.csv
```

### What to expect

On a PREEMPT_RT kernel:
| metric | non-RT | RT |
|---|---|---|
| mean | ≈ target | ≈ target |
| worst-case (idle) | target +100-300 µs | target +50-200 µs |
| worst-case (under load) | **target +ms possible** | target +few-hundred µs |

PREEMPT_RT kernel already improves non-RT threads dramatically; SCHED_FIFO
on top gives the final tightening, mostly in worst-case (tail) latency.

On a normal (non-PREEMPT_RT) kernel the gap is much wider (ms vs hundreds
of µs).

## CLI args (both executables)

| flag | default | meaning |
|---|---|---|
| `--robot-ip IP`         | UR3e: .94 / UR10e: .92 | robot's IP |
| `--robot ur3e\|ur10e`    | ur3e / ur10e           | robot type |
| `--config PATH`         | —                      | YAML config path |
| `--resources-dir PATH`  | —                      | dir with rtde_*_recipe.txt + external_control.urscript |
| `--rt-mode true\|false`  | false                  | enable SCHED_FIFO + mlockall |
| `--rt` / `--no-rt`       | —                      | shortcut |
| `--rt-priority N`       | 80                     | SCHED_FIFO priority (1..99) |
| `--rt-cpu N`            | -1                     | pin control thread to CPU core |

The launch files pass `--resources-dir` automatically from the installed
`share/ur10e_teleop_real_cpp/resources`.

## Known limitations / TODO

- **tau_contact / OVER-FORCE not implemented** — `actual_TCP_force` is
  read but not converted to joint-space. Carried over from `_py`.
- **Auto power-on / brake-release** — still manual on the pendant.
- **Force-feedback variant** — future `_ff` package will replace the
  position-spring coupling with Jacobian-mapped TCP force.

## Architecture overview

```
+-----------------+        +-----------------+
| leader_real_     |        | follower_real_  |
| node (UR3e)     |<------>| node (UR10e)    |
|                 |  ROS   |                 |
| rclcpp + rt_     |        | rclcpp + rt_    |
| thread::SCHED_   |        | thread::SCHED_  |
| FIFO @ prio 80  |        | FIFO @ prio 80  |
|        |        |        |        |        |
|     UrDriver    |        |     UrDriver    |
+--------+--------+        +--------+--------+
         |                          |
    RTDE/URScript              RTDE/URScript
         |                          |
    [UR3e robot]               [UR10e robot]
```

## Comparison script

`script/rt_comparison.py`:
- Runs the standalone benchmark twice (non-RT + RT) with optional load
- Parses CSV, prints percentile stats (mean/min/p50/p90/p99/p99.9/max)
- `--save-csv DIR` for raw dump
- `--plot` for matplotlib histogram
