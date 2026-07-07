# openarmx_teleop — enactic openarm_teleop bilateral, ported to openarmx (Robstride)

v1.0 = enactic `openarm_teleop`의 bilateral 코드/알고리즘을 **그대로** 가져와 **openarmx(Robstride)
로봇용 파라미터 + 기구·좌표 튜닝**으로 포팅한 것. 목표 = 근본적 bilateral transparency 향상.
(원본: `~/ext_ref/openarm_teleop`, Enactic Inc. 2025, Apache-2.0. 저작권 헤더 보존.)

> ⚠️ **아직 제어 PC에서 전체 빌드·검증 안 됨(P5 미완).** dev 박스에서 openarmx **API 문법체크만 통과**
> (openarm_init.cpp OK). 제어 PC 빌드 + 아래 부호검증 필수.

## 무엇이 openarmx용으로 바뀌었나 (enactic 대비)
- 라이브러리: `openarm_can`(DM) → **`openarmx_can`(Robstride)**. `OpenArm`→`OpenArmX`,
  `MITParam{kp,kd,q,dq,tau}`→`MotionControlParam{kp,kd,position,velocity,torque}`,
  `mit_control_all`→`send_motion_control_commands`. **OpenArmX 2번째 인자=can_fd(=false)**.
- 모터: RS04(J1-2)/RS03(J3-4)/RS00(J5-7,그리퍼), CAN ID send==recv {0x01..0x07}, 그리퍼 0x08.
  (`src/openarm_constants.hpp`)
- **좌표/부호**: `joint_state_converter.hpp` OpenArmJointConverter에 **ARM_SIGN=−1** (motor↔joint;
  우리 direction_multipliers=−1). 좌우 미러는 URDF가 담당. 그리퍼 변환기는 identity(P6에서 스케일 정밀화).
- 동역학: KDL, **우리 URDF**(openarmx_description v10.urdf.xacro), 링크 `openarmx_body_link0`→
  `openarmx_<side>_hand`. **중력 G_SCALE=0.93**(붕뜸 보정, control.cpp).
- 파라미터: `config/{leader,follower}.yaml` — 우리 kp/kd·Fc/k (Fv/Fo=0 초기, P6서 식별).
- rate: `FREQUENCY=150`(USB2CAN 균일상한; enactic 1000+CAN-FD).

## 빌드 (plain CMake, colcon 아님 — enactic 방식)
```bash
# 제어 PC. openarmx_can + orocos-kdl + kdl_parser + eigen + urdfdom + yaml-cpp 필요.
cd ~/git_ws/ss_prep_260413/openarmx_teleop
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 실행
```bash
# CAN up (classic 1M)
for i in 0 1 2 3; do sudo ip link set can$i down; sudo ip link set can$i type can bitrate 1000000; sudo ip link set can$i up; done
# 오른팔: leader can0 / follower can2  (왼팔: left_arm can1 can3)
./script/launch_bilateral.sh right_arm can0 can2
```
(스크립트가 openarmx_description v10.urdf.xacro로 URDF 생성 → 바이너리 실행.)

## ⚠️ P5 첫 bring-up: 부호 검증 (필수 — 안 하면 날뜀)
motor 부호가 뒤집히면 `Kp(q_peer−q)`가 양의 피드백→발산. 켜기 전/직후:
- 각 관절을 **+ 방향**으로 손으로 움직였을 때 `openarmx_angle_limit_for_friction_test.txt` 설명과
  일치하는지 확인(예: L-J1 +=몸 뒤, L-J4 +=팔꿈치 굽힘). **불일치 관절 = 부호 반전** →
  `joint_state_converter.hpp` 해당 관절 부호 교정. (제한각은 clamp 아님, 부호 참고용.)
- 먼저 낮은 kp로 시작, 중력보상만 확인 후 커플링 게인 점진 상향.

## 상태 / 다음 (study_for_v1.md §4 로드맵)
- ✅ P0-P4 포팅(패키지·port레이어·controller·config·launch). openarmx API 문법 OK.
- ⏳ P5 제어PC 빌드 + 부호검증. P6 transparency 튜닝(마찰 Fv/Fo 식별, 관절별 kp, 자유공간 오차 로깅).
