# ur10e_teleop_real

Bilateral force-feedback teleoperation for real UR robots (Leader: UR3e, Follower: UR10e) via RTDE.
No external ROS2 package dependencies — standalone package.

## Package Structure

```
ur10e_teleop_real/
├── src/
│   ├── leader_real_node.py            # Leader ROS2 node (keyboard + bilateral PD + deadband)
│   ├── follower_real_node.py          # Follower ROS2 node (bilateral PD tracking)
│   ├── control.py                     # URControl — RTDE client + URScript upload
│   ├── rtde_connection.py             # RTDE binary protocol implementation
│   ├── dummy_control.py               # DummyControl — local dynamics for testing
│   ├── ur_server_dummy.py             # RTDE dummy server [TODO: dynamics improvement]
│   └── environment_sensing_data_emulator.py
├── config/
│   ├── real_ur.yaml                   # Real UR hardware (tuned bilateral + deadband)
│   ├── dummy.yaml                     # Dummy testing
│   └── rtde_dummy.yaml                # RTDE + dummy server
├── launch/
│   ├── teleop_real.launch.py          # Real UR robots (default IPs set)
│   ├── teleop_dummy.launch.py         # Dummy mode (no hardware)
│   └── teleop_rtde_dummy.launch.py    # RTDE + dummy server
├── tests/
│   ├── test_full_stack_real.py        # Dummy mode test
│   ├── test_full_stack_real_hw.py     # Real hardware test (ROS2 topics only)
│   └── test_full_stack_real_ur_server_dummy.py
├── script/
│   ├── install.sh                     # Dependency installer
│   ├── identify_robots.py             # Dashboard-based robot model/state query
│   ├── calibrate_bilateral.py         # Interactive bilateral home calibration
│   ├── test_rtde_connection.py        # RTDE connection + motion test
│   ├── test_rtde_minimal.py           # Safe minimal-motion torque test
│   ├── logging.sh                     # Auto-logging (topics + monitor)
│   └── log_monitor.py                 # Real-time status monitor
├── log/                               # Log output directory (auto-created)
├── CMakeLists.txt
└── package.xml
```

## Install

```bash
cd ~/colcon_ws/src/ur10e_teleop_real
bash script/install.sh

cd ~/colcon_ws
colcon build --packages-select ur10e_teleop_real
source install/setup.bash
```

Dependencies (auto-installed by `install.sh`):
- Python: `numpy`, `pynput` (optional, keyboard), `python-xlib`, `pyyaml`
- ROS2 Jazzy: `sensor_msgs`, `std_msgs`, `launch`, `ament_index_python`

No other ROS2 packages required.

## Quick Start — Real UR Robots

### 1. Identify which robot is at which IP (always do this first)

```bash
python3 script/identify_robots.py --ips <IP_A> <IP_B>
```

Dashboard Server (port 29999) 에 `get robot model` 를 쿼리해서 각 IP 가 UR3e 인지 UR10e 인지 확인. Launch IP 순서 잘못되면 leader/follower 설정이 엉켜서 모든 게 이상해지므로 필수 확인.

### 2. Launch bilateral teleop

```bash
ros2 launch ur10e_teleop_real teleop_real.launch.py
```

기본 IP (launch file default):
- `leader_ip = 169.254.186.94` (UR3e)
- `follower_ip = 169.254.186.92` (UR10e)

다른 IP 쓰려면:
```bash
ros2 launch ur10e_teleop_real teleop_real.launch.py \
  leader_ip:=<UR3e_IP> follower_ip:=<UR10e_IP>
```

### 3. Wait for homing to complete (≈5 s quintic spline)

Pendant 로그에 양쪽 모두 `Program rtde_torque_ctrl started` + `HOMING complete → PAUSED` 떠야 함. PC 콘솔 `[DIAG]` 로그로도 확인.

### 4. Switch to ACTIVE (start bilateral)

```bash
ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0]"
```

Mode 숫자:
| value | mode | 효과 |
|---|---|---|
| 0 | ACTIVE | Bilateral tracking — 이걸 주로 씀 |
| 1 | PAUSED | 양팔이 각자 home 에 hold (수동 움직임 막힘) |
| 2 | HOMING | 양팔을 home 으로 quintic spline 이동 (auto_home_on_start=true 로 시작 시 자동) |
| 3 | FREEDRIVE | 양팔 torque=0 (firmware gravity-comp 만) — 손으로 자유 positioning 가능 |

### 5. Operate

- Leader (UR3e) 를 손으로 이리저리 움직이면 Follower (UR10e) 가 실시간으로 따라옴
- Follower 가 어딘가에 닿으면 leader 에 약한 force feedback (contact 감)

## Distributed Setup (Leader and Follower on separate PCs)

각 로봇을 **다른 PC** 에서 제어하려면 split launch files 사용.

### 전제

- 두 PC 가 동일 **RMW** (e.g. `rmw_zenoh_cpp`) 및 **같은 `ROS_DOMAIN_ID`** (기본 0)
- 각 PC → 자기 담당 로봇 IP 로 RTDE(30004) + Secondary(30002) 접근 가능
- 방화벽이 해당 RMW 의 traffic 허용
- (권장) 두 PC 간 NTP 시간 동기화

### PC A — Leader (UR3e) 쪽

```bash
# teleop_real_leader.launch.py 는 leader 노드만 실행
ros2 launch ur10e_teleop_real teleop_real_leader.launch.py
# or override IP:
ros2 launch ur10e_teleop_real teleop_real_leader.launch.py leader_ip:=<UR3e_IP>
```

### PC B — Follower (UR10e) 쪽

```bash
ros2 launch ur10e_teleop_real teleop_real_follower.launch.py
# or override IP:
ros2 launch ur10e_teleop_real teleop_real_follower.launch.py follower_ip:=<UR10e_IP>
```

### 모드 전환 (아무 PC 에서나)

```bash
ros2 topic pub --once /ur10e/mode std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0]"
```

### 분산 주의사항

1. **Latency 증가**: 같은 PC 에서 ~0-1 ms vs LAN 분산 +1-5 ms. 일반 LAN 에서 체감 거의 없지만, bilateral tuning (특히 KD_BI, deadband) 에 약간 영향 줄 수 있음.
2. **RMW 동일성**: 두 PC 가 같은 `RMW_IMPLEMENTATION` 이어야 함. Zenoh 쓰면 cross-subnet discovery 가 대체로 깔끔 (router 토폴로지 지원). 다른 subnet 에서 FastDDS/CycloneDDS 쓸 때는 unicast profile 또는 `ROS_DISCOVERY_SERVER` 필요.
3. **Config 동기화**: 두 PC 가 `config/real_ur.yaml` 같은 내용 유지. 한 쪽만 수정하면 gain 불일치.

## Run — Dummy Mode (No Hardware)

```bash
ros2 launch ur10e_teleop_real teleop_dummy.launch.py
```

로컬 simulation (URControl 대신 DummyControl). URScript 자동 업로드 스킵.

## Calibration — Bilateral Home Tuning

Leader 와 follower 가 "같은 pose 에 있는 것" 을 code 가 정확히 알아야 bilateral 제어가 매끄러움. 초기 default home 이 맞지 않으면 ACTIVE 전환 시 로봇이 튀어나감 등의 증상.

### Workflow

```bash
# 1. Teleop 실행 중 상태에서 (homing 완료 후)
ros2 launch ur10e_teleop_real teleop_real.launch.py

# 2. 별도 터미널에서 calibration 시작
python3 script/calibrate_bilateral.py --n 5
```

진행:
1. Script 가 `MODE_FREEDRIVE` 를 발행 → 두 팔 torque=0 상태
2. 사용자가 양팔을 눈으로 봤을 때 sync 된 자세 (mirror 가 아닌 **같은 모양**) 로 손으로 옮김
3. Enter 누름 → 스크립트가 두 팔 joint_state 를 ~100 ms 평균해서 기록
4. 총 5개 pose 에서 반복 (다양한 workspace 자세로)
5. 스크립트가 `mean_offset`, `std`, `fit verdict` 출력 + 추천 home 값 제시

### 결과 해석

```
mean OFFSET = [...]   ← follower_q - leader_q 평균 (rad)
std         = [...]   ← 포즈별 offset 편차
fit verdict : GOOD (max std < 0.01 rad)
            : OK   (max std < 0.05)
            : POOR (max std > 0.05 — sign flip / scale issue)
```

- GOOD/OK → 출력된 suggested `leader_home`, `follower_home` 을 `config/real_ur.yaml` 에 복사
- POOR → 데이터 원인 분석 (특정 joint 에 scale 이나 sign 차이 있는 경우)

Script 종료 시 자동으로 `MODE_PAUSED` 발행 → 양팔 home 잡음.

## Testing

### 1. 실물 UR 연결 확인 (단독)

```bash
python3 script/test_rtde_connection.py \
  --robot-ip <UR_IP> --config config/real_ur.yaml
```

Homing + joint 0 에 작은 sinusoid → 복귀.

### 2. 안전 최소 토크 테스트

```bash
python3 script/test_rtde_minimal.py --robot-ip <UR_IP> --mode pulse --joint 3 --amp 1.0
```

개별 joint 에 작은 pulse 만 — direct_torque 파이프라인 작동 확인용. `--mode sin` or `pulse` 선택, `--joint N` 으로 관절 선택.

### 3. 실물 통합 테스트 (ROS2 토픽 전용)

```bash
# 노드 실행 후 다른 터미널:
python3 tests/test_full_stack_real_hw.py
```

| Phase | 내용 |
|---|---|
| 0 | Setup & liveness |
| 1 | Passive stability (PAUSED 에서 drift 작음) |
| 2 | Publish rate >= 50 Hz |
| 5 | PAUSED ↔ ACTIVE 토글 |
| 6 | HOMING 수렴 |
| 7 | Reset topic |

### 4. Dummy mode 자동 테스트

```bash
python3 tests/test_full_stack_real.py
```

## Logging

실물 디버깅 시 모든 토픽 + 상태 자동 기록 (logs/ 디렉토리).

```bash
# Terminal 1: teleop
ros2 launch ur10e_teleop_real teleop_real.launch.py

# Terminal 2: logging
cd ~/colcon_ws/src/ur10e_teleop_real
bash script/logging.sh                                     # default: log/teleop_<timestamp>/
bash script/logging.sh log/my_test config/real_ur.yaml      # custom + config snapshot
# Ctrl+C to stop
```

기록 파일:
```
log/teleop_<timestamp>/
├── leader_state.csv       follower_state.csv     mode.csv
├── rosout.csv             warnings_only.csv      monitor_output.log
├── config_snapshot.yaml   system_info.txt        node_list.txt
├── topic_list.txt         hz_leader.txt          hz_follower.txt
```

실시간 모니터 (`log_monitor.py`) 가 3 초마다 요약 출력.

문제 발생 시:
```bash
zip -r teleop_logs.zip log/teleop_<timestamp>/
```

## UR Robot Prerequisites

- UR controller ON, **Remote Control** mode (pendant 모드 아님)
- Firmware >= **5.22.0** (required for `direct_torque()`); 5.25 tested
- 로봇 초기화 (브레이크 해제, 전원 on) — 현재는 수동. 자동화는 TODO
- Network: PC 와 UR 이 같은 subnet
- 포트: 30004 (RTDE), 30002 (Secondary — URScript upload), 29999 (Dashboard — identify_robots)

### URScript 자동 업로드

`URControl.connect()` 가 Secondary Interface(30002)에 아래 URScript 업로드, controller 위에서 실행:

```urscript
def rtde_torque_ctrl():
  global cmd_torque = [0, 0, 0, 0, 0, 0]
  global cmd_mode = 0

  thread torque_thread():
    while True:
      if cmd_mode == 1:
        direct_torque(cmd_torque, friction_comp=True)   # firmware friction comp
      else:
        direct_torque([0, 0, 0, 0, 0, 0], friction_comp=True)
      end
    end
  end

  run torque_thread()
  while True:
    cmd_mode = read_input_integer_register(0)
    if cmd_mode == 1:
      cmd_torque = [read_input_float_register(0..5)]
    end
    sync()
  end
end
rtde_torque_ctrl()
```

- `direct_torque` 는 별도 **real-time thread** 에서 호출 (UR 공식 driver 패턴)
- `friction_comp=True` 로 harmonic drive 관절 마찰 상쇄
- Disconnect 시 `stopj(2.0)` 로 안전 정지

## Config (real_ur.yaml) 주요 파라미터

### Torque control
- `friction_comp: true` — firmware 가 joint friction 보상 (URScript 에서 `friction_comp=True` 로 호출)
- `gravity_comp_internal: true` — firmware 가 중력 보상
- `leader_torque_limit` / `follower_torque_limit` — 각 로봇 per-joint 한계 (UR3e < UR10e)

### Bilateral coupling (leader side, ACTIVE 모드)

```yaml
KP_BI:           [100, 100, 70, 35, 35, 30]   # position spring
KD_BI:           [  0,   0,  0,  0,  0,  0]   # velocity damping = off
TAU_BI_DEADBAND: [ 10,  10, 6.5, 4.5, 4.5, 3.8]   # per-joint, in Nm
```

**제어식** (ACTIVE):
```
tau_raw = KP_BI * (peer_q - q)
tau_bi  = sign(tau_raw) * max(0, |tau_raw| - TAU_BI_DEADBAND)
tau     = tau_bi  (clipped by leader_torque_limit)
```

**의미**:
- Follower 가 leader 를 잘 따라올 때 → position error 작음 → `|tau_raw| < deadband` → `tau = 0` → 사용자는 freedrive 감
- Follower 가 막혀서 못 따라올 때 → error 가 deadband 초과 → spring 힘 loa leader 에 전달됨 → contact 감

### Follower tracking

```yaml
KP_TRACK: [300, 300, 150, 100, 100, 80]   # 공격적 tracking
KD_TRACK: [ 30,  30,  20,  10,  10, 10]
```

UR10e 의 큰 질량 / payload 극복을 위해 충분히 큰 이득.

### Home positions (bilateral-calibrated)

```yaml
leader_home:   [2.2984, -1.3313, 2.2127, -0.8611, 2.2772, 0.0376]   # UR3e
follower_home: [2.3092, -1.3226, 2.2141, -0.9007, 2.2723, 0.0357]   # UR10e
```

`calibrate_bilateral.py` 로 5 pose 측정 후 mean_offset 기반 도출.

### Joint mirroring

```yaml
joint_mirror:
  sign: [1, 1, 1, 1, 1, 1]   # 두 로봇 동일 orientation — mirror 없음
```

두 arm 이 같은 방향 mount 됨 확인. (초기 default 였던 `[-1, 1, -1, 1, -1, 1]` 은 다른 셋업용)

## Keyboard Controls (Leader, display 필요 — SSH 에서는 비활성)

| Key | Joint | Direction |
|---|---|---|
| Up / Down | 0 (shoulder_pan) | -/+ |
| Left / Right | 1 (shoulder_lift) | -/+ |
| [ / ] | 2 (elbow) | -/+ |
| R / F | 3 (wrist_1) | -/+ |
| T / G | 4 (wrist_2) | -/+ |
| Y / H | 5 (wrist_3) | -/+ |

현재 구성 (KP_USER=0) 에서는 keyboard 사용하지 않음 — pure haptic bilateral. Keyboard 쓰려면 config 의 `KP_USER`, `KD_USER` 를 0 이 아닌 값으로.

## Tuning Journal — Bilateral haptic feel

초기 naive bilateral → "자연스러운 haptic" 까지의 실험 기록. 기준: **free motion 은 freedrive 처럼 가볍고, contact 는 명확히 전달**.

### Iter-A: `KD_BI = 0`
- **변경**: viscous damping 제거 (KD_BI 5 → 0)
- **결과**: 빠른 손 motion 의 끈적함 사라짐. Contact 감 약간 약해짐 (velocity mismatch 채널 상실).
- **교훈**: KD_BI 는 viscosity 주범이지만 contact 전환의 onset 신호도 제공. 0 으로 두면 더 가볍지만 contact 반응 느림.

### Iter-C: Continuous deadband 도입
- **변경**: `tau_bi = sign(raw) * max(0, |raw| - DB)` — 작은 error 는 0 통과, 큰 error 는 threshold 빼고 전달
- **결과**: Free motion 의 residual error 가 저항으로 새지 않음. Contact 는 threshold 넘으면 명확 전달.
- **교훈**: Position-position bilateral 의 핵심 한계 (free/contact 모호) 를 해결하는 단일 지렛대.

### Iter-D: KP_BI 상향 + wrist deadband widen
- **변경**: KP_BI [40,40,25,12,12,12] → [60,60,40,20,20,20], DB wrist 0.5 → 1.2 Nm
- **결과**: Contact per 단위 error 강해짐. Wrist 가 빠른 motion 에 덜 민감.

### Iter-E: `friction_comp=True` + leader wrist torque_limit 상향
- **변경**: URScript 의 direct_torque 에 `friction_comp=True` 복원, leader wrist limit [5,3,2] → [8,6,4]
- **결과**: 이전 IP 스왑 상태에서 안 먹었던 friction_comp 가 이번엔 제대로 작동. Wrist contact 더 넓은 범위 전달.

### Iter-F: 추가 KP_BI 상향 + wrist KP_TRACK 상향
- **변경**: KP_BI 1.7x → [100,100,70,35,35,30], wrist KP_TRACK [50,50,40] → [100,100,80], leader wrist limit [9,8,6]
- **결과**: Contact 감 확실히 개선. Free motion 에서 다시 뻑뻑해짐 (KP_BI 상승분만큼 residual 이 강하게 전달).

### Iter-G: Deadband 1.6x widen
- **변경**: DB [5,5,3.5,2.1,2.1,1.8] → [8,8,5.5,3.5,3.5,3.0]
- **결과**: Free motion tolerance 가 rad 단위로도 1.6x → 부드러움 복구. Contact 강도 약간 감소하지만 여전히 OK.

### Iter-H: Shoulder/elbow DB 추가 widen
- **변경**: Shoulder DB 8 → 10, elbow 5.5 → 6.5 (wrist 유지)
- **결과**: Shoulder 특히 눈에 띄게 부드러워짐. 대비로 wrist 가 뻑뻑하게 느껴짐.

### Iter-I (최종): Wrist DB widen
- **변경**: Wrist DB [3.5,3.5,3.0] → [4.5,4.5,3.8]
- **결과**: Wrist 도 freedrive 감. Contact 약간 줄어들지만 여전히 "OK" 수준.

### 최종 수렴값

```yaml
KP_BI:           [100, 100, 70, 35, 35, 30]
KD_BI:           [  0,   0,  0,  0,  0,  0]
TAU_BI_DEADBAND: [ 10,  10, 6.5, 4.5, 4.5, 3.8]
# Tolerance (rad) = DB/KP_BI:
#   shoulder  0.100  (5.7°)
#   elbow     0.093  (5.3°)
#   wrist     0.129  (7.4°), wrist_3 0.127
```

### 핵심 교훈

1. **Contact 감은 KP_BI * effective_error 에 의해 결정** — deadband 로 바닥 slicing 해도 contact 는 deadband 위 영역에서 충분히 올라옴
2. **Free motion tolerance 는 rad 단위 (DB/KP_BI)** — 이걸 >3° 수준 보장하면 tracking jitter 가 저항으로 새지 않음
3. **Wrist 가 shoulder 보다 넓은 tolerance 필요** — 사람이 wrist 를 더 빠르게 움직이기 때문
4. **Torque_limit 이 contact 상한** — KP_BI 가 아무리 커도 limit 에서 clip. Wrist 의 leader_torque_limit 을 하드웨어 한계까지 올리면 강한 contact 가 충분히 전달됨
5. **Position-position bilateral 의 구조적 한계는 존재** — 완벽한 "freedrive + contact" 는 TCP F/T 센서 기반 pure force feedback 에서 나옴 (TODO 의 FF 분기)

## TODO

- [ ] `ur_server_dummy.py` dynamics stabilization
- [ ] F/T sensor integration for over-force detection (+ explicit contact feedback channel)
- [ ] TCP position safety limits
- [ ] Real-time kernel support (POSIX FIFO thread)
- [ ] Smooth PAUSED→ACTIVE transition on follower
      Even after calibration, follower still jerks on first ACTIVE pub.
      Residual is payload-induced gravity sag during PAUSED. Options:
      (a) payload-aware gravity model on PC, (b) per-robot soft-start
      ramp on tracking term, (c) dynamic zero-offset reference at
      transition instant.
- [ ] Auto power-on / boot / brake-release before homing
      Dashboard Server (port 29999) commands:
        `power on` → wait BOOTING → IDLE → `brake release` → wait RUNNING
      Implementation: extend `src/control.py` or add `src/dashboard_client.py`;
      leader/follower nodes call before RTDE. Opt-in via `auto_power_on: true`.
      `identify_robots.py` already uses :29999 as reference.
- [ ] Pure TCP-F/T force feedback variant (ur10e_teleop_real_ff)
      Position-position bilateral has structural ambiguity between
      free motion and contact. Plan: separate package with J^T·F from
      follower's `actual_TCP_force` replacing KP_BI path. Gives real
      freedrive in free space, clean contact signal otherwise.
