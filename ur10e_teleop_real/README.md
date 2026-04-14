# ur10e_teleop_real

Bilateral force-feedback teleoperation for real UR robots (Leader: UR3e, Follower: UR10e) via RTDE.
No external ROS2 package dependencies — standalone package.

## Package Structure

```
ur10e_teleop_real/
├── src/
│   ├── leader_real_node.py            # Leader ROS2 node (keyboard + bilateral PD)
│   ├── follower_real_node.py          # Follower ROS2 node (bilateral PD tracking)
│   ├── control.py                     # URControl — RTDE client
│   ├── rtde_connection.py             # RTDE binary protocol implementation
│   ├── dummy_control.py              # DummyControl — local dynamics for testing
│   ├── ur_server_dummy.py             # RTDE dummy server [TODO: dynamics improvement]
│   └── environment_sensing_data_emulator.py
├── config/
│   ├── real_ur.yaml                   # Config for real UR hardware
│   ├── dummy.yaml                     # Config for dummy testing
│   └── rtde_dummy.yaml               # Config for RTDE + dummy server
├── launch/
│   ├── teleop_real.launch.py          # Real UR robots (default: leader=ur3e)
│   ├── teleop_dummy.launch.py         # Dummy mode (no hardware)
│   └── teleop_rtde_dummy.launch.py    # RTDE + dummy server
├── tests/
│   ├── test_full_stack_real.py        # Dummy mode test
│   ├── test_full_stack_real_hw.py     # Real hardware test (ROS2 topics only)
│   └── test_full_stack_real_ur_server_dummy.py
├── script/
│   ├── install.sh                     # Dependency installer
│   ├── test_rtde_connection.py        # RTDE connection + motion test
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

## Run — Real UR Robots (RTDE)

Default: Leader = UR3e, Follower = UR10e.

### Two-robot bilateral teleop

```bash
ros2 launch ur10e_teleop_real teleop_real.launch.py \
  leader_ip:=192.168.1.100 \
  follower_ip:=192.168.1.101
```

### Leader is UR10e (override default)

```bash
ros2 launch ur10e_teleop_real teleop_real.launch.py \
  robot:=ur10e \
  leader_ip:=192.168.1.100 \
  follower_ip:=192.168.1.101
```

### Manual (separate terminals)

```bash
# Terminal 1: Leader
ros2 run ur10e_teleop_real leader_real_node.py \
  --client rtde --robot-ip 192.168.1.100 --config real_ur.yaml

# Terminal 2: Follower
ros2 run ur10e_teleop_real follower_real_node.py \
  --client rtde --robot-ip 192.168.1.101 --config real_ur.yaml
```

## Run — Dummy Mode (No Hardware)

```bash
ros2 launch ur10e_teleop_real teleop_dummy.launch.py
```

## Testing

### 1. RTDE Connection + Motion Test (단독, 실물 1대로)

Homing 후 joint 0에 ±0.05 rad sinusoid → 복귀. 실물 UR 연결 확인용.

```bash
python3 script/test_rtde_connection.py \
  --robot-ip 192.168.1.100 \
  --config config/real_ur.yaml
```

### 2. 실물 통합 테스트 (ROS2 토픽 전용, SSH 가능)

노드 실행 중 별도 터미널에서 실행. keyboard/X11 불필요.

```bash
# 먼저 노드 실행:
ros2 launch ur10e_teleop_real teleop_real.launch.py \
  leader_ip:=192.168.1.100 follower_ip:=192.168.1.101

# 다른 터미널:
python3 tests/test_full_stack_real_hw.py
```

테스트 항목:

| Phase | 이름 | 내용 |
|---|---|---|
| 0 | Setup & liveness | 양쪽 노드 기동 + 토픽 발행 확인 |
| 1 | Passive stability | 양팔 정지 유지 (drift < 0.05 rad / 2s) |
| 2 | Control frequency | publish rate >= 50 Hz |
| 5 | Pause toggle | PAUSED → hold 안정성 → ACTIVE 복귀 |
| 6 | Homing | HOMING mode → HOME 수렴 |
| 7 | Reset | reset topic → leader HOME 복귀 |

### 3. Dummy mode 자동 테스트

```bash
python3 tests/test_full_stack_real.py
```

## Logging (실물 디버깅용)

실물 테스트 시 모든 토픽과 상태를 자동 기록합니다. 로그는 `log/` 디렉토리에 저장됩니다.

### 사용법

```bash
# Terminal 1: teleop 실행
ros2 launch ur10e_teleop_real teleop_real.launch.py \
  leader_ip:=192.168.1.100 follower_ip:=192.168.1.101

# Terminal 2: logging 시작
cd ~/colcon_ws/src/ur10e_teleop_real
bash script/logging.sh                            # default: log/teleop_YYYYMMDD_HHMMSS/
bash script/logging.sh log/my_test config/real_ur.yaml  # custom dir + config snapshot
# → Ctrl+C로 중지
```

### 기록되는 파일

```
log/teleop_20260413_150000/
├── leader_state.csv      # Leader joint positions/velocities
├── follower_state.csv    # Follower joint positions/velocities
├── mode.csv              # Mode 전환 이력
├── rosout.csv            # 모든 노드 로그
├── warnings_only.csv     # WARN/ERROR/OVER-FORCE만 필터링
├── monitor_output.log    # 실시간 모니터 출력 기록
├── config_snapshot.yaml  # 사용된 config 복사본
├── system_info.txt       # 호스트/날짜/ROS 버전
├── node_list.txt         # 실행 중인 노드
├── topic_list.txt        # 활성 토픽
├── hz_leader.txt         # Leader publish rate
└── hz_follower.txt       # Follower publish rate
```

### 실시간 모니터 (logging 중 자동 표시)

```
   Time     Mode   Warn  TrkErr                                  Leader q[0:5]                                Follower q[0:5]
---------------------------------------------------------------------------------------------------------------------------------
     3s   ACTIVE      0  0.0012  [+2.240,-1.281,+2.160,-0.885,+2.240,+0.000]  [-2.240,-1.861,-2.160,-2.257,-2.240,+0.000]
     6s   PAUSED      1  0.0000  [+2.240,-1.281,+2.160,-0.885,+2.240,+0.000]  [-2.240,-1.861,-2.160,-2.257,-2.240,+0.000]  *** 1 NEW WARNING(S) ***
```

### 문제 발생 시

```bash
cd ~/colcon_ws/src/ur10e_teleop_real
zip -r teleop_logs.zip log/teleop_YYYYMMDD_HHMMSS/
```

이 zip과 함께 **어떤 동작을 했을 때 문제가 생겼는지** 전달.

## UR Robot Prerequisites

- UR controller ON, **Remote Control** mode (pendant 모드 아님!)
- Firmware >= **5.22.0** (required for `direct_torque()`) — 5.25 tested
- 로봇 초기화 (브레이크 해제, 전원 on)
- Network: PC와 UR이 같은 subnet (예: 192.168.1.x)
- IP 확인: UR pendant > Settings > Network
- 30002 포트 (Secondary Interface) 접근 가능해야 함 — `control.py` 가
  연결 시 URScript 토크 제어 루프를 자동 업로드하는 데 사용

### URScript 자동 업로드

`URControl.connect()` 가 UR Secondary Interface(port 30002)에 아래와 같은
스크립트를 전송해 controller 위에서 실행시킵니다:

```urscript
def rtde_torque_ctrl():
  while True:
    mode = read_input_integer_register(0)
    if mode == 1:
      tau = [read_input_float_register(0), ...]
      direct_torque(tau)        # firmware 5.22+ 토크 제어
    else:
      direct_torque([0,...,0])  # safe idle
    end
  end
end
rtde_torque_ctrl()
```

이 스크립트가 없으면 RTDE input register 에 토크값을 써도 **로봇은 1도 움직이지
않습니다** (register 만 업데이트되고 실행되지 않음). Pendant 에 Play 버튼 누를 필요
없이 PC 연결만으로 자동으로 토크 제어 모드로 진입합니다.

disconnect 시에는 `stopj(2.0)` 을 보내 안전 정지합니다.

## Config Switching

| Config | `friction_comp` | `gravity_comp_internal` | `torque_limit` | Use case |
|---|---|---|---|---|
| `real_ur.yaml` | true | true | ±10 Nm | Real UR hardware |
| `dummy.yaml` | false | false | ±330 Nm | Dummy testing |

## Joint Mirroring (reverse-mounted leader)

Leader(UR3e)가 반대 방향으로 설치되어 joints 0,2,4의 회전 방향이 반대:

```yaml
# config/real_ur.yaml
joint_mirror:
  sign: [-1, 1, -1, 1, -1, 1]   # joints 0,2,4 flipped
leader_home:  [2.24, -1.2808, 2.16, -0.8848, 2.24, 0.0]
follower_home: [-2.24, -1.8608, -2.16, -2.2568, -2.24, 0.0]
```

## Keyboard Controls (Leader, display 필요)

| Key | Joint | Direction |
|---|---|---|
| Up / Down | 0 (shoulder_pan) | -/+ |
| Left / Right | 1 (shoulder_lift) | -/+ |
| [ / ] | 2 (elbow) | -/+ |
| R / F | 3 (wrist_1) | -/+ |
| T / G | 4 (wrist_2) | -/+ |
| Y / H | 5 (wrist_3) | -/+ |

SSH 환경에서는 keyboard 자동 비활성화 (pynput unavailable 경고).

## TODO

- [ ] `ur_server_dummy.py` dynamics stabilization
- [ ] F/T sensor integration for over-force detection
- [ ] TCP position safety limits
- [ ] Real-time kernel support (POSIX FIFO thread)
