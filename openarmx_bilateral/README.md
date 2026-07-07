# openarmx_bilateral — OpenArmX bilateral force-feedback teleop

두 openarmx 양팔로봇(leader+follower)의 **위치-위치(P-P) bilateral force-feedback** teleop.
전부 **ros2_control MIT** 인터페이스로 제어(raw CAN 아님). relay가 두 팔의 관절 위치/속도를
서로에게 중계하고, 반력은 HW MIT kp 커플링에서 발생. + 중력보상/속도FF/마찰보상/그리퍼/posture.

> 전체 배경·이유는 **history.md**, 현 상태·튜닝노브는 **STATUS.md**, 보류 부채(transparency/
> USB-CAN)는 **tech_debt.md** 참조.

---
## Quick start (실사용 명령)

**0) CAN 인터페이스** (한 번):
```bash
for i in 0 1 2 3; do sudo ip link set can$i down 2>/dev/null; sudo ip link set can$i type can bitrate 1000000; sudo ip link set can$i up; done
```

**Terminal 1 — leader 브링업** (non-namespaced):
```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  right_can_interface:=can0 left_can_interface:=can1 \
  control_mode:=mit robot_controller:=forward_position_controller
```

**Terminal 2 — follower 브링업** (`/follower`):
```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  arm_prefix:=follower right_can_interface:=can2 left_can_interface:=can3 \
  control_mode:=mit robot_controller:=forward_position_controller
```

**Terminal 3 — bilateral (이 패키지)**:
```bash
ros2 launch openarmx_bilateral bilateral.launch.py \
  arm:=both vel_ff:=true friction:=true posture:=true bilateral:=true posture_scale:=2.0 \
  kp:=35,20,15,8,8,3,1,1 kd:=3.5,5.0,2.5,0.8,0.8,0.3,0.1,0.1
```
> `kp`/`kd` = 관절별 8값(그리퍼 포함), leader+follower·좌우 동일 적용(채택 실용값 — `ENACTIC_REFERENCE_GAINS.md`).
> 안 주면 `leader_kp`(일괄) 방식으로 폴백.

### ⚠️ 브링업 필수/금지
- **`robot_controller:=forward_position_controller` 필수** — 없으면 기본값 joint_trajectory_controller가 떠서 relay 명령을 무시함(follower 안 움직임).
- **`enable_forward_effort` 주지 말 것** — 중력/마찰/effort는 이 패키지가 담당(안 그러면 leader 오염).
- follower는 **`arm_prefix:=follower`** (=/follower 네임스페이스; joint 이름은 그대로 openarmx_<side>_joint).
- CAN 매핑: leader R/L=can0/can1, follower R/L=can2/can3.

---
## bilateral.launch.py 인자
| 인자 | 기본 | 의미 |
|---|---|---|
| `arm` | left | **left / right / both** (제어할 팔 쌍) |
| `bilateral` | false | true=양방향 커플링(반력). false=unilateral(follower만 추종) |
| `vel_ff` | false | Phase 2 속도 피드포워드 (동적 추종 개선) |
| `friction` | false | Phase 3 마찰보상 (가벼워짐) |
| `friction_scale` | 0.7 | 마찰보상 배율 (↑ 가벼움, 과하면 limit cycle) |
| `posture` | false | J3 영점 자가복원 스프링 (leader) |
| `posture_scale` | 1.0 | posture 세기 배율 (bilateral에선 ↑ 필요, 예 2.0) |
| `kp` / `kd` | ''(미사용) | **관절별 MIT 게인 8값**(그리퍼 포함), leader+follower·좌우 동일 적용. 예 `kp:=240,240,240,240,24,31,25,16 kd:=3,3,3,3,0.2,0.2,0.2,0.2`. 설정 시 아래 leader_kp/follower_kp 대체 |
| `leader_kp` / `leader_kd` | 0/0 | (kp 미설정 시) 반력 세기/댐핑 일괄 (bilateral 예 10~60 / 0.5) |
| `follower_kp` / `follower_kd` | ''(HW기본) | (kp 미설정 시) follower 추종 게인 (전 관절 일괄) |
| `couple_sign` | 1.0 | leader↔follower 위치 부호 (좌·우 HW 검증 +1) |
| `g_scale` | 0.93 | 중력보상 배율 (1.0 과보상→붕뜸, 0.93에서 정지) |
| `urdf_path` | /tmp/v10_bimanual.urdf | gravity_comp용 URDF |

leader 그리퍼(joint8) 게인은 relay.launch에서 자동 `kp0.3/kd0.03` (인자 `leader_gripper_kp/kd`).

## 참고
- 단일 팔: `arm:=left` 또는 `arm:=right`. `both`는 CAN 부하 2배(rate 저하 가능 — tech_debt.md).
- 오른팔이 반대로 움직이면 `couple_sign:=-1` (좌우 동일 +1로 검증됨).
- RViz 끄기: 브링업 후 `pkill -f rviz2`.
- 진단/측정 노드: `log_node`(추종), `friction_log_node`(effort 분해), `chirp_node`(대역폭), `friction_id_node`(마찰식별), `joint_echo_node`(관절각).
