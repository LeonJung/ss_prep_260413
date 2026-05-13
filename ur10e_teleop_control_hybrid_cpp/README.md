# ur10e_teleop_control_hybrid_cpp

C++ implementation of bilateral force-feedback teleoperation for real UR
robots (UR3e leader + UR10e follower) combining two SOTA techniques:

- **Tier B1** — Sensorless 4-Channel Lawrence control with model-based
  feedforward (`M·u + C·q̇ + D·q̇`) and a joint-space Disturbance Observer
  for τ̂_ext estimation, after Buamanee et al., *"Fast Bilateral
  Teleoperation and Imitation Learning Using Sensorless Force Control
  via Accurate Dynamics Model"*, arXiv:2507.06174 (2025).
- **Tier C1** — Two-Layer Energy-Tank passivity wrapper modulating the
  4CH tracking bracket, after Franken/Stramigioli (2011) and Minelli
  et al. (2023–24).

Built on top of `ur_client_library` for the UR I/O layer and supports
PREEMPT_RT kernel scheduling (SCHED_FIFO + mlockall) for tight
control-loop timing.

The package was created by cloning
[`ur10e_teleop_control_ff_cpp`](../ur10e_teleop_control_ff_cpp/) and
replacing its passive-leader + F/T-reflection control with the 4CH+DOB
formulation. The state machine, RT plumbing, dashboard handshake, and
launch infrastructure are inherited unchanged.

## Status

What's coded and verified in software (selftest):

| layer                                              | status |
|----------------------------------------------------|--------|
| `DynamicsModel` — KDL-backed M(q), C(q,q̇)q̇, g(q)   | ✅ |
| `VelocityEstimator` — first-order LPF on diff(q)   | ✅ |
| `DisturbanceObserver` — Q-filter form, fw-grav-aware| ✅ |
| `FourChannelController` — full M, per-side gains, split Kf_self / Kf_peer | ✅ |
| `EnergyTank` — Two-Layer wrapper (refill, drain, α-throttle) | ✅ (HW-untested) |
| Selftest covering all of the above                 | ✅ |

What's verified on real hardware (UR3e + UR10e):

| behaviour                                          | status |
|----------------------------------------------------|--------|
| Cross-coupling-free position tracking              | ✅ |
| DOB-based contact reflection                       | ✅ (weak under URDF model) |
| Stable PAUSED / ACTIVE / HOMING / FREEDRIVE        | ✅ |
| Idle stability (no drift, no oscillation)          | ✅ |
| "Apparent inertia < physical inertia" on shoulder/elbow | ❌ — hard limit; see *Tuning journey* |

## Control law — 4-Channel + Disturbance Observer (+ optional Energy Tank)

매 2 ms tick마다 양쪽(leader, follower)에서 아래 4단계를 거친다.

### 1) 속도 추정

```
q̇̂ = LPF_80Hz( d/dt q )
```

관측된 위치를 시간 미분하면 잡음이 크므로 1차 저역통과 필터(컷오프 80 Hz)로
매끄럽게 만든다. 이 q̇̂가 DOB와 inner law의 속도 입력 모두에 쓰임.

### 2) Disturbance Observer (DOB) — sensorless τ̂_ext

강체 동역학:

```
M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ_applied + τ_ext
```

좌변(model)을 알면 외란은:

```
τ̂_ext = Q(s) · [ M(q)·q̈̂  +  C(q,q̇̂)·q̇̂  +  g(q)  −  τ_applied ]
```

| term | meaning |
|------|---------|
| `M(q)` | inertia matrix (URDF → KDL 계산). |
| `C·q̇̂` | Coriolis+원심항. |
| `g(q)` | 중력 토크. UR 펌웨어가 grav-comp을 켜면 생략(`fw_grav_comp` 플래그). |
| `τ_applied` | 이전 step 명령 토크. |
| `Q(s)` | 2차 저역통과 (cutoff 30 Hz) — 모델 오차/미분 잡음이 ext에 새는 걸 막는 realizability filter. |

핵심: **F/T 센서 없이도** "모델이 예측한 토크"와 "실제 가속이 가리키는
토크"의 차이가 환경 외란. URDF 정확도가 그대로 추정 품질이 되며, 그래서
`dynamics_selftest`가 사전 검증 단계에 있다. 구현: `src/disturbance_observer.cpp`.

### 3) 4-Channel Inner Law (Lawrence)

```
u_i = Kp_i · (q_peer_i − q_i)
    + Kd_i · (q̇̂_peer_i − q̇̂_i)
    + Kf_self_i · τ̂_ext_i
    + Kf_peer_i · τ̂_ext_peer_i
```

Lawrence(1993) 4-channel framework — 양방향 teleop의 정보 흐름을 4 채널로
분해. 양쪽이 같은 식을 돌리면 forward / backward 양 방향이 자동 성립.

| 채널 | 의미 |
|------|------|
| C1/C2 (P) | `Kp·Δq` — 위치 결합. 양쪽에서 보면 leader→foll, foll→leader 두 방향. |
| C3/C4 (F) | `Kf_peer·τ̂_ext_peer` — peer의 외력을 받음. |
| (확장) | `Kf_self·τ̂_ext` — 본인이 받는 외력을 추가 토크로(보통 0이거나 작음). |

이론적 transparency 최대화에는 `C1=C3, C2=C4`의 특정 조합이 필요(Lawrence).
이게 9차원 + κ 튜닝이 까다로운 이유. 구현:
`src/four_channel_controller.cpp:41–60`.

### 4) Outer (Computed-Torque) Law

```
τ_4ch = M(q) · ( ramp · u )
      − κ · τ̂_ext                     # κ = tau_ext_cancel_gain (≈ 0.7)
      + C(q,q̇̂) · q̇̂
      + D ⊙ q̇̂
      + ( fw_grav_comp ? 0 : g(q) )
```

| term | meaning |
|------|---------|
| `M·(ramp·u)` | inner law(가속 명령)에 mass shaping ⇒ 토크 단위. `ramp`는 ACTIVE 진입시 0→1. |
| `−κ·τ̂_ext` | 본인이 받는 외란을 **부분적**으로 cancel. κ=1이면 완전 stiff, κ=0이면 외란을 그대로 느낌. 0.7이 trade-off 지점. |
| `C·q̇̂` | Coriolis feedforward — 빠른 다관절 동작의 관성 결합 상쇄. |
| `D⊙q̇̂` | 점성 댐핑(기본 0). 안정성 보강용 knob. |
| `g(q)` | 중력 FF. 펌웨어 grav-comp가 켜져있으면 이중 보상 방지 위해 생략. |

### 5) (옵션) Energy Tank — Two-Layer Passivity

기본 disabled. 켜면:

```
P_out = (τ_4ch − τ_static) · q̇̂                   # controller가 로봇에 주입하는 파워
Ė_tank = D_dissip · q̇̂²  −  P_out                 # 가상 마찰로 충전, 출력으로 소진
E_tank ∈ [0, E_max]                              # E_max=5 J, 충전 천장 0.9·E_max
α = (E_tank > 0 ? 1 : 0)                         # binary throttle
τ_cmd = τ_static + α · (τ_4ch − τ_static)
```

직관: "이 컨트롤러가 system으로 보내는 누적 에너지는 시스템에서 빼낸
가상 마찰 손실을 초과할 수 없다" — 이게 **passivity**. Passive
environment에 결합되어 있으면 passivity ⇒ stability. 모델 오차가 컨트롤러를
공격적으로 만들어 탱크를 비우면 α=0으로 토크가 잠시 꺼지면서 발산을 차단.

구현: `src/energy_tank.cpp:26–64`.

### 최종 출력

`τ_cmd`는 per-joint 토크 리밋으로 clip 후 RTDE `MODE_TORQUE`로 UR에 송신
(HOMING 중에는 `MODE_SERVOJ`).

## Architecture

```
                    /ur10e/mode (Float64MultiArray)
                            ↓
+-------------------+                    +-------------------+
| leader_hybrid_node|◀── /ur10e/leader/  | follower_hybrid_  |
| (UR3e, port 50001)|    joint_state ──▶ | node              |
|                   |                    | (UR10e, port 50011)|
| DynamicsModel     |                    |                   |
| VelocityEstimator |                    | (same modules)    |
| DisturbanceObserver|                    |                   |
| FourChannelController                   |                  |
| EnergyTank (opt)  |                    |                   |
|        |          |                    |        |          |
|     UrDriver      |                    |     UrDriver     |
+--------+----------+                    +--------+----------+
         |                                        |
    RTDE / URScript                          RTDE / URScript
         |                                        |
    [UR3e robot]                             [UR10e robot]
```

Joint-state effort field carries each side's `τ̂_ext` so the peer can
use it as `τ̂_ext_peer` in its 4CH coupling term — no F/T sensor
required.

## Dependencies

```bash
source /opt/ros/<your-distro>/setup.bash   # humble | jazzy | rolling | ...
bash script/install.sh
```

Auto-detects `ROS_DISTRO`. Installs:

- `ros-<DISTRO>-{rclcpp,sensor-msgs,std-msgs,ament-cmake,ur-client-library}`
- `ros-<DISTRO>-{kdl-parser,orocos-kdl-vendor}` (added vs `_ff_cpp`)
- `libeigen3-dev`, `libyaml-cpp-dev`, `libcap2-bin`

## Build

```bash
cd <your_ws>
colcon build --packages-select ur10e_teleop_control_hybrid_cpp
source install/setup.bash
```

`--symlink-install` caveat: same as `_ff_cpp` — `setcap_rt.sh` resolves
the symlink and applies caps to the real build-side binary; rerun after
every build.

## RT capabilities (re-run after every build)

```bash
bash script/setcap_rt.sh
```

Without caps the executables still run, but `--rt-mode true` prints a
warning and falls back to normal scheduling.

## Launch

### Single PC (both nodes)

```bash
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real.launch.py \
    leader_rt:=true follower_rt:=true
```

### Distributed (leader on PC A, follower on PC B)

```bash
# PC A — UR3e side
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real_leader.launch.py rt:=true
# PC B — UR10e side
ros2 launch ur10e_teleop_control_hybrid_cpp teleop_real_follower.launch.py rt:=true
```

### Mode topic

After auto-homing finishes, switch to bilateral:

```bash
ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0]"
```

Modes: 0=ACTIVE · 1=PAUSED · 2=HOMING · 3=FREEDRIVE.

## Self-test (no robot needed)

Verifies dynamics model, DOB, 4CH compute, energy tank — all the
math-only modules:

```bash
ros2 run ur10e_teleop_control_hybrid_cpp dynamics_selftest \
    src/ur10e_teleop_control_hybrid_cpp/resources/ur10e.urdf
```

## Configuration (`config/real_ur.yaml`)

Hybrid-specific block:

| key                       | meaning                                                |
|---------------------------|--------------------------------------------------------|
| `KP`, `KD`                | shared bilateral PD gains (legacy; seed both sides)    |
| `KF`                      | shared force-feedback gain (legacy; seeds Kf_self & Kf_peer for both sides) |
| `leader_KP`, `leader_KD`, `leader_KF`     | leader-only overrides              |
| `follower_KP`, `follower_KD`, `follower_KF`| follower-only overrides           |
| `leader_KF_PEER`, `follower_KF_PEER`       | per-channel split (peer reflection only) |
| `D_VISCOUS`               | modeled viscous damping (vec6)                         |
| `dob_cutoff_hz`           | Q-filter cutoff for τ̂_ext (default 30)                 |
| `dob_accel_cutoff_hz`     | q̈̂ pre-filter cutoff (default 50)                      |
| `velocity_cutoff_hz`      | q̇̂ filter cutoff (default 80)                           |
| `tau_ext_cancel_gain`     | scale on −τ̂_ext term (κ in math; default 0.7)          |
| `use_diagonal_inertia`    | diagnostic only — drops M off-diagonals (default false)|
| `tank.enabled`, `tank.e_max`, `tank.e_init`, `tank.refill_ceiling`, `tank.D_DISSIPATION` | EnergyTank parameters |

The current YAML is the **round-9 baseline** (see *Tuning journey*):
symmetric KP/KD/KF + UR-official URDF + κ=0.7 + tank disabled.

## Tuning journey (HW iteration record)

This section is the lab-notebook record of nine hardware-tuning rounds.
Useful when a future contributor wonders "why are the numbers what they
are." All commits are on branch `main` of `git@github.com:LeonJung/ss_prep_260413`.

| round | what changed | observed | takeaway |
|------:|-------------|----------|----------|
| 1 | First HW run with stock 4CH, KF=0, κ=0.7, full-M, fw_grav_comp=true | wrists drift up after ACTIVE; user grip stops drift; oscillation under user motion | gravity double-comp + DOB feedback under URDF model error |
| 2 | Add `firmware_grav_comp` flag (omit +g(q) in 4CH and DOB residual) | initial idle holds, but motion → release re-drifts | DOB picks up firmware-grav-comp residual + URDF dynamic error |
| 3 | Add `tau_ext_cancel_gain` knob (κ); set to 0; lower DOB & velocity cutoffs | shoulders/elbow track cleanly, all three wrists frozen (stiction, M·KP too small) | M·u_inner formulation gives tiny actuator gain on low-inertia joints |
| 4 | Wrist KP up: [30,30,20] → [1000,5000,30000] | wrist tracks; pushing wrist makes elbow / shoulder surge then bounce | high KP × M off-diagonal × URDF model error → cross-coupling |
| 5 | Halve wrist KP back to [500,2000,10000]; full M kept | tracks, cross-coupling damped to acceptable | linear amplitude scaling — but not a real fix |
| 6 | Replace URDFs with UR-official ROS2 description (UR10e wrist_3 mass 0.615 → 0.202) | cross-coupling gone, tracking responsive, idle stable; wrist now correctly heavier; shoulder still heavy; contact reflection feels weak | URDF accuracy IS the gating factor for 4CH transparency |
| 7 | Split KF into Kf_self / Kf_peer; raise leader_KF_PEER 3× | shoulder *much* heavier, tracking lag, contact crisper | high Kf_peer amplifies follower-side M-model leakage as drag |
| 8 | Halve leader_KF_PEER back (1.7× round-2) | shoulder still heavy, slight contact lag returning | contact-strength vs free-motion-drag are coupled by model error |
| 9 | Revert leader_KF_PEER → match Kf_self (round-6 baseline restored) | round-6 behaviour: cross-coupling-free, stable, shoulder mildly heavy, contact mildly weak | this is the **practical optimum under the URDF model** |

### The fundamental inequality

For UR3e shoulder M ≈ 0.28, κ = 0.7:

```
"lighter than passive"   requires KF_self > κ / M ≈ 2.5
"closed-loop stability" requires KF_self < ~0.5 / M ≈ 1.79     (with model-error margin)
```

The two ranges **do not overlap**. Pushing past the stability limit
(round 4, round 7's wrists) triggers the M-model-error feedback we saw.
Staying inside it (round 6/9) leaves the shoulder noticeably heavier
than passive.

The only known way to widen the stability bound is to shrink the
`(M_real − M_model)` term — i.e., **system identification**. See TODO.

## CLI args (both executables)

| flag | default | meaning |
|---|---|---|
| `--robot-ip IP`         | UR3e: .94 / UR10e: .92 | robot's IP |
| `--robot ur3e\|ur10e`    | ur3e / ur10e           | robot type |
| `--config PATH`         | —                      | YAML config path |
| `--resources-dir PATH`  | —                      | dir with `ur*.urdf` + `rtde_*_recipe.txt` + `external_control.urscript` |
| `--rt-mode true\|false`  | false                  | enable SCHED_FIFO + mlockall |
| `--rt` / `--no-rt`       | —                      | shortcut |
| `--rt-priority N`       | 80                     | SCHED_FIFO priority (1..99) |
| `--rt-cpu N`            | -1                     | pin control thread to CPU core |

Launch files pass `--config` and `--resources-dir` automatically from
the installed `share/ur10e_teleop_control_hybrid_cpp/`.

## RT vs non-RT benchmark

```bash
bash script/setcap_rt.sh
python3 script/rt_comparison.py --duration 30 --rt-cpu 2
python3 script/rt_comparison.py --duration 30 --rt-cpu 2 --load
```

Same as `_ff_cpp` — see that package's README for expected percentile
behaviour. The 4CH/DOB stack adds ~6 µs (KDL `M+C+g` call) per cycle on
top of the existing PD; well under the 2 ms budget.

## TODO

### 🔴 High priority — blocking further transparency improvement

- [ ] **System Identification of UR3e and UR10e dynamics**
      The single fix for the inequality above. Steps:
        1. Define excitation trajectories (Fourier-series sweep per joint
           or random, satisfying joint/torque limits).
        2. Add a "data collection" mode in leader/follower nodes that
           runs the trajectory and logs (t, q, q̇, τ_we_send,
           actual_TCP_force) at 500 Hz to CSV.
        3. Build the regression matrix Φ(q, q̇, q̈) (Hollerbach–Khalil
           form, ~37 parameters per arm) and solve `θ = (ΦᵀΦ)⁻¹Φᵀτ`
           by least-squares (SVD-truncated for ill-conditioned cols).
        4. Cross-validate on a held-out trajectory.
        5. Replace `DynamicsModel`'s URDF source with the identified
           parameter set (extend `DynamicsModel` to accept either a
           URDF path or an inline parameter struct).
      Reference: Buamanee et al. 2025 §4; the `shamilmamedov/dynamic_calibration`
      MATLAB reference for the regressor structure.

### 🟡 Medium priority — completes the original B1+C1 mission

- [ ] **Hardware validation of the EnergyTank wrapper (Tier C1)**
      Code is complete and selftested; never run on real arms.
      Workflow:
        1. From the round-9 baseline, set `tank.enabled: true` in YAML.
        2. Confirm idle behaviour unchanged (`α ≈ 1.0` in `[DIAG]`).
        3. Probe ACTIVE-mode contact transients — push the follower into
           a stiff fixture; tank should maintain `α = 1` while the
           refill mechanism replenishes from D·q̇² dissipation.
        4. Stress test: deliberately low `tank.e_max` to trigger
           α throttling; verify the leader feels softer rather than
           losing stability.
        5. With tank on, retry pushing KF higher than the round-9
           baseline and see whether the passivity bound widens
           (this is the central C1 claim).

- [ ] **Architecture A/B option: passive leader + DOB force reflection**
      The `ur10e_teleop_control_ff_cpp` pattern (KP_USER=0, K_FT × peer
      effort) gives the user a genuinely free leader at the cost of
      symmetric 4CH transparency math. Reuse this package's DOB
      output as the peer-effort signal in place of `_ff_cpp`'s
      `J^T·F_TCP` so the architecture pivot stays sensorless.
      Should be a few hours of plumbing inside a clone of `_ff_cpp`.

### 🟢 Lower priority / inherited from `_ff_cpp` lineage

- [ ] **Auto power-on / brake-release + graceful shutdown** (UR Dashboard
      Server, port 29999). Already partially in `dashboard_client.cpp`;
      verify and finish.

- [ ] **Position-control homing (fix for PAUSED→ACTIVE jerk)**.
      Hot-swap two URScript programs on port 30002 — torque loop for
      ACTIVE/PAUSED/FREEDRIVE, position loop (`movej`/`servoj`) only
      during HOMING. Removes the gravity-balance offset that makes the
      torque loop start with a non-zero error on ACTIVE entry.

- [ ] **TCP workspace safety — 2-tier virtual wall**. SOFT layer injects
      `J^T · K_wall · penetration` torque, HARD layer auto-PAUSES on
      excess penetration. Roughly 15 µs / cycle.

- [ ] **OVER-FORCE detection via DOB τ̂_ext or RTDE `actual_TCP_force`**.
      Trigger MODE_PAUSED on threshold breach. Currently the relevant
      thresholds are in YAML but not wired.

## Comparison script

`script/rt_comparison.py` — same usage as in `_ff_cpp` (no behaviour
change in this package).
