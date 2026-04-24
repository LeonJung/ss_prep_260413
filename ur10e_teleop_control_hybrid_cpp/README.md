# ur10e_teleop_control_hybrid_cpp

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
colcon build --packages-select ur10e_teleop_control_hybrid_cpp
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
colcon build --packages-select ur10e_teleop_control_hybrid_cpp  # no --symlink-install
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
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real.launch.py

# both nodes RT
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real.launch.py \
    leader_rt:=true follower_rt:=true

# only leader RT (mixed environment)
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real.launch.py \
    leader_rt:=true follower_rt:=false
```

### Distributed (leader on PC A, follower on PC B)

```bash
# PC A — UR3e side
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real_leader.launch.py rt:=true

# PC B — UR10e side
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real_follower.launch.py rt:=true
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
`share/ur10e_teleop_control_hybrid_cpp/resources`.

## TODO

Mirrored in `ur10e_teleop_real_py/README.md` — changes should land in
both packages.

### 🔴 High priority

- [ ] **Auto power-on / brake-release + graceful shutdown**
      Currently the operator runs Power On → Booting → Release Brakes
      on each teach pendant before every session, and powers the
      robot back down afterwards. Automate via UR Dashboard Server
      (port 29999):
        startup:  `power on` → wait BOOTING → IDLE → `brake release`
                  → wait RUNNING → then URScript upload + homing
        shutdown: reverse — stop URScript, `brake engage` (from a safe
                  pose), `power off`, close dashboard socket
      Implementation sketch:
        - add a `dashboard_client` module (C++): open TCP to :29999,
          send commands, poll `robotmode`/`safetymode` with per-step
          timeouts
        - leader_node / follower_node call startup in `connect_robot()`
          and shutdown in `stop()`
        - opt-in via config flag `auto_power_cycle: true`

- [ ] **Position-control homing (fix for PAUSED→ACTIVE jerk)**
      Root cause of the follower jerk: torque-based homing via
      KP_HOLD*(q_des − q) can't settle exactly at home. The spring
      balance point is home + (gravity + friction)/KP_HOLD, not home
      itself. At ACTIVE entry tracking error is nonzero → KP_TRACK
      fires a kick.

      Plan — two URScript programs, hot-swapped via port 30002 at
      HOMING ↔ non-HOMING transitions:

        URScript A — existing torque-loop (direct_torque)
          Used for PAUSED / ACTIVE / FREEDRIVE.

        URScript B — native position control (movej / servoj to home)
          Used only for HOMING. Settles at the exact commanded q.

      Transition flow:
        user sends MODE_HOMING
          → node stops current torque writes
          → uploads URScript B with target home_q in payload
          → URScript B runs movej, reports completion
          → node uploads URScript A (torque loop)
          → node publishes MODE_PAUSED
        user later sends MODE_ACTIVE / etc. — torque loop handles it.

      Hot-swap via port 30002 takes ~1 s; homing already takes ~5 s so
      no user-visible latency added. The torque loop starts from
      exactly zero error, so there's no kick on the first active cycle.

### 🟡 Medium priority

- [ ] **F/T sensor integration — OVER-FORCE detection + contact torque**
      `actual_TCP_force` is in the RTDE output recipe but unused.
      Convert to joint-space via J^T·F (requires UR Jacobian; DH
      params available) and feed into:
        - existing over-force detection path (currently zeros →
          never fires)
        - HOMING collision detection (currently disabled for same reason)
        - optional: explicit haptic feedback onto leader for crisper
          "freedrive vs contact" distinction

- [ ] **TCP workspace safety — 2-tier virtual wall**
      Bounding box in base frame (xyz_min, xyz_max). Two layers:
        (i)  SOFT safety — when follower's TCP touches the box, inject
             a synthetic contact force through the same haptic path
             the real F/T feedback would use:
               F_wall = K_wall * penetration  (spring into the wall)
               tau   += J^T * F_wall          (mapped to joint torque)
             User feels the wall on leader exactly like a physical
             obstacle. Great for "gentle reminder" behavior.
        (ii) HARD safety — if user keeps pushing past the soft wall
             and penetration exceeds a deeper threshold (or |F_wall|
             exceeds a cutoff), escalate to MODE_PAUSED.
      Opt-in via config:
        workspace_limits:
          xyz_min: [...]
          xyz_max: [...]
          k_wall: 2000              # N/m, soft-wall stiffness
          soft_penetration: 0.02    # m, threshold before hard escalation
      Performance: estimated overhead ~15 µs/cycle on top of the
      existing Jacobian work for the F/T integration item — negligible
      (< 1% of a 2 ms cycle). Shares the Jacobian module with that item.

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
