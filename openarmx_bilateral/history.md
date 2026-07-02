# openarmx_bilateral — HISTORY (무엇을·왜 했는가, 세세히)

이 패키지가 지금 형태가 된 전 과정과 이유를 기록. 깊은 세부는 자매 문서 참조:
`STATUS.md`(현 상태/명령), `FORCE_FEEDBACK_NOTES.md`(transparency 설계논의),
`tech_debt.md`(보류된 부채). 이 파일은 **전체 서사 + 하드-원 교훈**.

---
## 0. 목표와 전제
- **목표**: openarmx 양팔로봇 2대(leader+follower)로 **bilateral force-feedback teleop**. 체감 목표 = MIT Sangbae Kim 랩 데모(막히면 딱딱한 벽, 평상시 깃털).
- **방식 고정**: 전부 **ros2_control MIT 하드웨어 인터페이스**로 모터 제어(raw CAN 아님). kp/kd 런타임은 `openarmx_<side>_hardware_params` 파라미터 서비스.
- **금지(실패작)**: DOB/RTOB/RFOB, energy tank, 4-channel, wave variables. (과거 UR10e에서 실패. 다시 꺼내지 말 것.)
- **Fresh start**: 이전 oa_mit/oa_fd 시도는 폐기하고 새로 시작. (단 oa_fd_cpp의 posture 개념은 참고로 재활용 → §8.)

## 1. 제어 구조 (핵심)
**위치-위치(P-P) cross-relay + HW MIT kp.** relay 노드가 두 팔의 joint_states를 읽어 서로의 위치를 상대 팔의 forward_position_controller로 발행. 힘 반력은 HW MIT의 kp 커플링에서 발생.
- 관절별 MIT 명령: `τ = kp·(q_peer−q) + kd·(q̇_peer−q̇) + τ_ff`, `τ_ff = gravity + friction (+posture)`.
- **이 구조 = enactic openarm = MIT 2022 논문과 동일 골격**(§7에서 수식 확인). "비법 수식" 없음.

## 2. Phase 1 — 중력보상 (ADOPTED)
- `gravity_comp_node`(openarmx_gravity_comp, KDL)가 중력토크를 effort 컨트롤러로 발행. 관절 처짐(sag) 제거.
- **A/B 실측**(log_node CSV): follower 중력 ON vs OFF → 평균 |추종오차| 0.031→0.016 rad(**−48%**), J2 sag −0.053rad(3°)→~0.
- 함정: gravity_comp가 **절대 토픽**(/joint_states, /left_forward_effort_controller/commands) 사용 → follower는 반드시 **remap**(→/follower/...), leader는 non-namespaced라 그대로 도달. follower 브링업은 enable_forward_effort 없이(안 그러면 leader 오염).

## 3. Phase 2 — 속도 FF (ADOPTED)
- relay가 상대 팔의 **관절 속도**를 forward_velocity_controller로도 발행(velocity 인터페이스, position 컨트롤러와 공존 → MIT {pos,vel}). `vel_ff:=true`.
- **A/B 실측**: err-vs-speed 기울기 0.079→0.040(**−50% 동적 lag**), 이동중 오차 전 부하관절 감소(J1−24%,J4−49%,J6−42%…). 벽에서 "매끄럽다/빠르다"의 정체.

## 4. Phase 3 — 마찰보상 (ADOPTED)
- **식별**: `friction_id_node`가 한 관절씩 여러 일정속도로 왕복(kp=0, 중력 ON) → `fric = eff − grav` 로깅. **odd 대칭화 피팅**으로 중력잔차 제거. 모델 `τ_fric = Fc·tanh(0.1·k·ω)`.
- **결과**: **Coulomb 지배, 점성 Fv≈0**(생략). 로봇/개체별로 다름 → leader/follower **각각** 식별. 값(왼팔; 오른팔은 관절별 동일 재사용 — 운영자 결정):
  - LEADER Fc `[1.21,0.63,0.31,0.39,0.14,0.17,0.13]` k `[81,51,40,52,36,20,48]`
  - FOLLOWER Fc `[1.04,1.06,0.35,0.36,0.16,0.15,0.12]` k `[68,60,26,87,62,57,39]`
- **적용**: `friction_comp_node`가 gravity_comp와 effort 컨트롤러 사이에서 `out = grav + scale·Fc·tanh(0.1k·ω)` 발행(단일 발행자 유지, openarmx 미수정). `friction:=true`, `friction_scale=0.7`(과보상/limit cycle 방지, 체감튜닝).
- 검산: R² 0.4~0.85(Fc는 저속 plateau로 robust). 시작 시 정지 근처 미세 떨림(steep tanh×노이즈) 있으나 조작 중엔 무관.

## 5. Bilateral force feedback
- `bilateral:=true` → relay가 **양방향**(leader↔follower) 발행. `leader_kp`(예 5~60)가 반력 세기(force reflection). couple_sign **+1**(HW 검증, 좌·우 동일).
- 초기엔 "엄청 무거움"이었으나 P1+P2+P3로 사용 가능해짐. 무거움의 주범은 kp가 아니라 **마찰**이었음(마찰보상으로 해결).

## 6. 오른팔 / both (arm:=left|right|both)
- `bilateral.launch.py`를 **side별 빌더**(build_side)로 리팩터. `arm` 인자로 left(기본)/right/both.
- right config 추가(`right_effort/velocity_controller.yaml`), `follower_gravity.launch` side화(enable_left/right), relay 노드명 side별(both 충돌 방지), 마찰=왼팔값 재사용.
- **함정들(하드-원)**:
  - relay가 leader_cmd/follower_cmd를 **왼팔로 하드코딩** → arm:=right가 왼팔 토픽에 발행해 안 움직임. **arm_side에서 토픽 도출**로 수정.
  - 브링업 `robot_controller` 기본=**joint_trajectory_controller** → forward_position_controller가 안 떠서 relay 명령 무시(subscriber 0). **`robot_controller:=forward_position_controller` 필수.**
  - follower 네임스페이스 = 브링업 `arm_prefix:=follower` (xacro의 node_namespace로만 쓰임, **joint 이름은 openarmx_<side>_joint 유지** — relay 안 깨짐).
  - couple_sign +1 좌우 동일 확인. both = CAN 부하 2배(USB-CAN ~89Hz 캡, §9).

## 7. MIT/enactic 제어 수식 확인 (transparency 논의 중)
- **MIT 2022(SaLoutos/Stanger-Jones/Kim, IROS) Eq.4**: `τ_f=Kp(q_l−q_f)+Kd(q̇_l−q̇_f)+τ_g ; τ_l=−τ_f`. 대수전개하면 우리 대칭 P-P와 동일. **마찰보상 없음**(QDD가 투명해 불필요).
- **enactic**: 대칭 Kp(어깨240) + 중력 + **마찰(tanh)** on 양팔. 우리와 같은 SPBT 계열.
- 결론: MIT의 벽·가벼움은 수식이 아니라 **QDD 액추에이터 투명도 + 고대역폭 + 게인**. 우리 마찰보상은 부족한 HW 투명도의 대체재.

## 8. 소소 개선 3건
- **#1 g_scale (중력): 1.05→0.93.** J1/J2/J4가 가만히 둬도 붕 떴던 원인 = `g_scale=1.05`(105% **과보상**) → 위로 미는 잔여토크. 낮춰서 0.93에서 정지 확인(운영자 "딱 맞네"). **최종 0.93.** (전역 스칼라뿐 — 축별 scale은 tech_debt TODO.)
- **#3 leader 그리퍼(joint8) kp/kd = 0.3/0.03.** relay 게인루프가 j1~7만 세팅하던 것에 j8 추가(인자 `leader_gripper_kp/kd`). 5.0/0.5→2.0/0.3→1.0/0.1→**0.3/0.03**(뻑뻑함 줄이며 수렴).
- **#2 posture spring (oa_fd_cpp 참고): J3만.** 관절을 영점으로 약하게 당김(자가복원). `friction_comp_node`에 `τ += kp_post·(q_ref−q) − kd_post·q̇` 추가(position 읽음). oa_fd 값 J3 kp1.8/kd0.18(J5는 운영자 요청으로 제거). **leader에만**(follower는 커플링으로 따라옴). `posture:=true`, `posture_scale`로 배율.
  - 발견: unilateral(leader free)에선 복원 잘 됨. **bilateral에선 커플링 kp(60)이 posture(1.8)를 33배로 압도** → posture_scale 키워야 체감. PC측 스프링이라 **CAN 지연 chatter 위험**(oa_fd 교훈) → 약하게 시작.
  - oa_fd_cpp 관절별 기능표(참고): posture=J3·J5만, zone-repulsion(한계각 반발)=J1~J5(어깨 강 kp40, 나머지 약 kp5), J6·J7 없음. **zone-repulsion은 미도입**(추후 옵션).

## 9. Transparency / PreemptRT / USB-CAN 조사 → HOLD (상세: tech_debt.md)
- transparency를 MIT 수준으로 올리려 시도. **근본 병목 = USB-CAN(PEAK PCAN-USB FD) 왕복지연.**
- 제어루프가 **~89Hz로 캡**(controller_manager update_rate 500 줘도 실측 89Hz = USB-CAN per-cycle ~11ms). PreemptRT는 **transparency 개선 0**(chirp 데이터 generic=RT).
- PC N(NUC, PreemptRT) **관절 팡팡** = RT/RMW/SW 아니라 **USB 허브 2단 캐스케이드**(PC M은 1단). PC M+RT는 정상 → 기계(토폴로지) 문제 확정.
- **해결 로드맵 1순위 = 비-USB CAN(PCIe/SPI)**: rate·팡팡·RT 이득 동시 해결. **현재 보류.** 운영은 PC M(일반 커널).

## 10. 최종 파라미터 (현재 기본값)
- `g_scale=0.93`, `friction_scale=0.7`, `couple_sign=1.0`(좌우), leader gripper(j8) `kp0.3/kd0.03`.
- posture(off 기본): J3 `kp_post1.8/kd_post0.18`, q_ref 0, leader only, `posture_scale` 튜닝.
- leader_kp/kd, follower_kp/kd = 체감 튜닝(예 leader_kp 5~60/kd 0.5; follower 기본 HW).
- 토글: `arm`(left/right/both), `bilateral`, `vel_ff`, `friction`, `posture` — 전부 기본 off/left(안전).

## 11. 노드·런치·config 목록
- **노드(src/)**: `relay_node`(cross-relay pos+vel, bilateral 토글, side/그리퍼) · `friction_comp_node`(grav+friction+posture→effort) · `friction_id_node`(마찰 식별 shuttle) · `friction_log_node`(effort 분해 로깅) · `chirp_node`(사인스윕 대역폭/지연 측정, 전관절 순차) · `log_node`(cmd-vs-act) · `joint_echo_node`(임시, 관절각 출력).
- **launch/**: `bilateral.launch.py`(원샷 메인) · `relay.launch.py`(relay+게인 자동설정) · `follower_gravity.launch.py`(follower effort+중력, side) · `friction_id.launch.py`(식별, target=leader/follower).
- **config/**: `{left,right}_effort_controller.yaml`, `{left,right}_velocity_controller.yaml` (네임스페이스 yaml이 effort/velocity 컨트롤러를 선언 안 해 spawner에 -t/-p로 넘김).

## 12. 운영 명령 (요약; 상세 STATUS.md)
```
# CAN: for i in 0 1 2 3; do sudo ip link set can$i type can bitrate 1000000 up; done
# T1 leader:   ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
#   robot_controller:=forward_position_controller right_can_interface:=can0 left_can_interface:=can1 control_mode:=mit
# T2 follower:  ... arm_prefix:=follower robot_controller:=forward_position_controller \
#   right_can_interface:=can2 left_can_interface:=can3 control_mode:=mit
# T3 bilateral: ros2 launch openarmx_bilateral bilateral.launch.py arm:=both \
#   bilateral:=true vel_ff:=true friction:=true posture:=true leader_kp:=60 leader_kd:=0.5
```
브링업에 **robot_controller:=forward_position_controller 필수**, enable_forward_effort **금지**.

## 13. 하드-원 교훈 (반복 방지)
- launch에서 float/bool 인자를 노드로 넘길 때 `ParameterValue(..., value_type=)` 안 쓰면 타입불일치로 노드 즉사.
- `declare_parameter<vector<int64_t>>("x", {1,2,3})` **중괄호 초기화는 컴파일러별 쓰레기값**(빈배열→SIGSEGV, [3,4,5..]→관절 skip). 스케줄은 코드로 직접 채워라(friction_id_node).
- 컨트롤러 spawner에 `--unload-on-kill` (controller_manager가 launch보다 오래 살아 재실행 시 "already loaded").
- forward_position_controller는 **8-DOF**(7관절+finger 그리퍼) → 명령 배열 8개(그리퍼 유지) 보내야 update 에러 안 남.
- copy한 build/install엔 **시스템 .deb(openarmx-can)** 안 따라옴 — 버전 확인.
- cyclictest는 실행 중 스스로 C-state를 끔(그래서 C-state 배제 판단 가능).

## 14. 백업
git tag: `openarmx_bilateral_v1_best`(unilateral) / `v1.1`(P1+P2) / `v2.0`(P1+P2+P3) /
`v2.1`(right·both+튜닝). + `~/backup_ws/openarmx_bilateral_*` 사본. 자동싱크: ss_prep_260413.
