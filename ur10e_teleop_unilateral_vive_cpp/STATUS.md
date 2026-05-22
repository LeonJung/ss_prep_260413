# STATUS — ur10e_teleop_unilateral_vive_cpp

마지막 작업 시점 스냅샷. 새 세션이 진입했을 때 여기 한 번 읽고 들어가면
컨텍스트 복원됨.

## 현재 동작 상태 (2026-05-22 기준)

✅ **양손 (bimanual) Vive teleop 동작 중**. 사용자가 양손에 Manus glove +
   Vive tracker 끼고 움직이면 두 대의 UR10e (PC e 에 연결) 가 각자 추적.

- Position 추적: 트래커 위치 변위 → UR EE 위치 변위 (axis aligned)
- Orientation 추적: 트래커 회전 → UR EE 회전 (delta-from-startup 방식)
- ACTIVE 진입 시점이 새 영점 (re-tare every ACTIVE entry)
- 떨림 LPF (q_filter_alpha = 0.1) 적용
- Tool offset 둠: `tool_mode=arm` (EE=flange) / `tool_mode=hand` (EE=dg5f palm)

## 마지막 commit
`12a511c` — `unilateral_vive: tool_mode = arm | hand`

## 하드웨어 (PC e 의 lab setup)

| 디바이스 | 시리얼 / IP |
|---|---|
| Vive tracker Left  | `LHR-B4BFDF90` |
| Vive tracker Right | `LHR-C21814A6` |
| Base station 1     | `LHB-45131F3B` |
| Base station 2     | `LHB-BB0267D2` |
| UR10e Left         | `169.254.186.93` |
| UR10e Right        | `169.254.186.92`  ← bilateral 의 follower 와 동일 |

UR 들 cable 뒤로 마운트 → UR base frame `+X` = operator forward,
`+Y` = operator left, `+Z` = up. 좌우 UR 동일 회전.

## Calibration 상태

- **두 YAML 모두 `src/config/`에 commit** (단순 식별로 빌드마다 install 로
  복사됨, 따라서 사용자가 별도로 ~/.ros 안 채워도 됨).
  - `calibration_left.yaml`  — post-rot-z -90 적용된 값
  - `calibration_right.yaml` — 동일
- 사용자가 다시 cali 하면 `~/.ros/ur10e_teleop_unilateral_vive_cpp/` 에
  저장됨. Launch 가 거기를 먼저 찾고 없으면 `<pkg-share>/config/` fallback.

**중요 발견**: 이 lab setup 은 raw cali 결과가 일관적으로 **90° CW
horizontal 회전** 만큼 어긋남. `vive_calibrate --post-rot-z -90` 으로
보정해야 X→X, Y→Y 매핑 정상. Root cause 미규명 (operator body
orientation vs SteamVR room frame 의 일관적 mismatch 가설 유력).

## Home pose 좌우 분리

`config/real_ur.yaml`:
- `follower_home_right`: bilateral baseline (UR10e right 가 잘 가는 자세)
- `follower_home_left`: right 값에 mirror 변환 적용
  - `lq0 = -q0`, `lq1 = -π - q1`, `lq2 = -q2`,
  - `lq3 = -π - q3`, `lq4 = -q4`, `lq5 = q5`

## Tool mode

`config/real_ur.yaml`:
```yaml
tool_mode: arm
tool_offsets:
  arm:  [0.0, 0.0,  0.0]
  hand: [0.0, 0.02, 0.15]   # dg5f palm in flange frame
```

Launch 별 모드:
- `teleop_real.launch.py` (default = `arm`)
- `teleop_real_hand.launch.py` (= `hand`)
- CLI 로 직접 `--tool-mode arm|hand`

## 사용 절차 (운영 시)

```bash
# (Leader PC = PC e — SteamVR 켜져있고 트래커 두 개 다 초록)
cd ~/colcon_ws && source install/setup.bash

# Hand 모드 (그리퍼 장착 상태)
ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real_hand.launch.py

# 또는 Arm 모드 (flange 가 EE)
ros2 launch ur10e_teleop_unilateral_vive_cpp teleop_real.launch.py

# 모드 토글 (다른 셸):
ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray "data: [2.0, 0.0, 5.0]"  # HOMING 5s
ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0]"  # ACTIVE
```

런치 시 로그 확인:
- `left calib loaded: ...` + `right calib loaded: ...`
- `tool_mode=hand offset_in_flange=[0.000 0.020 0.150] m` (또는 arm)
- ACTIVE 보내면 `tare on ACTIVE entry — current tracker pose ↦ UR home EE`

## 핵심 코드 구조

```
[Vive tracker × 2] ──OpenVR (shared refcount)──► ViveTracker.poll()
                                                       │
                                                       ▼
                                        Calibration.apply()       (cali rotation = R_cali, t)
                                                       │
                            delta-from-startup tracking:
                              ΔR_ur = R_cali · ΔR_tracker · R_caliᵀ
                              Δp_ur = R_cali · Δp_tracker
                              T_palm = (ΔR_ur · T_home_palm.rot,
                                        T_home_palm.pos + Δp_ur)
                                                       │
                              tool offset 적용:
                              T_flange = T_palm · T_tool_offset⁻¹
                                                       │
                                                       ▼
                              ur_ik_solve(T_flange, ...) → q
                              q LPF 거쳐서:
                                                       │
                                                       ▼
                       /ur10e/{left,right}/leader/joint_state
                                                       │
                                          (follower 가 구독)
                                                       │
                                                       ▼
                              UR10e (각 측 RTDE 로 driver)
```

## 패키지 파일 맵

```
ur10e_teleop_unilateral_vive_cpp/
├─ include/.../
│  ├─ vive_tracker.hpp        — OpenVR refcount wrapper
│  ├─ calibration.hpp         — Umeyama 3-point solver + YAML
│  ├─ ur_ik.hpp               — DLS Jacobian IK (iterative)
│  ├─ ur_jacobian.hpp         — UR FK + analytical Jacobian
│  ├─ vive_leader_node.hpp    — 양손 leader, mode SM, tool offset
│  ├─ follower_node.hpp       — UR10e RTDE driver
│  └─ config.hpp              — YAML loader struct
├─ src/                        — 대응 .cpp 들 + main 진입점
├─ config/
│  ├─ real_ur.yaml             — gains / homes / tool_offsets
│  ├─ calibration_left.yaml    — lab cali (post-rot-z -90)
│  └─ calibration_right.yaml   — lab cali (post-rot-z -90)
└─ launch/
   ├─ teleop_real.launch.py        — bimanual, tool_mode=arm
   ├─ teleop_real_hand.launch.py   — bimanual, tool_mode=hand
   ├─ teleop_single.launch.py      — single side, debug용
   ├─ teleop_real_leader.launch.py — distributed leader 만
   └─ teleop_real_follower.launch.py — distributed follower 만 (양측)
```

## 알려진 한계 / TODO

| 항목 | 상태 |
|---|---|
| Closed-form 8-branch UR IK | 🟡 현재 iterative DLS. workspace 모서리에서 branch flip 가능 |
| Rotation LPF (현재 position 만) | 🟡 jitter 심하면 추가 |
| Tracker pose rviz 시각화 | 🟡 디버깅 / 데모용 |
| 90° CW systematic offset root cause | 🟡 미규명. `--post-rot-z -90` 으로 우회 |
| Hand offset `[0, 0.02, 0.15]` 정확도 | 🟡 dg5f 실측 안 함, 시각으로만 검증 — 더 정확히 하려면 측정 |
| Service 로 runtime tool_mode 전환 | ❌ 일부러 안 함 (launch 두 개로 충분) |

## 관련 메모리

- `[[vive-hardware-serials-lab]]` — 하드웨어 시리얼 + 마운팅
- `[[comm-benchmark-network-topology-and-routing]]` — PC e/f 가 등장하는
  다른 작업 (별도)
