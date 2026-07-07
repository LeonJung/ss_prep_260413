# study_for_v1 — v1.0을 향한 공부·계획 노트

> **모드:** 이 문서가 살아있는 동안은 **공부·계획만** 한다. 운영자가 "개발하라"고 명시하기 전까진
> 코드 변경 없음. 공부한 핵심은 여기에 빠짐없이 기록.
> 현재 코드 = v0.3 (백업 `~/backup_ws/openarmx_bilateral_v0.3_69368a7`).

---

## §1. 제어 방식 비교 — openarmx_bilateral(우리) vs enactic openarm_teleop

출처(enactic): `~/ext_ref/openarm_teleop/` (Enactic, Inc. 2025, Apache-2.0) —
`control/openarm_bilateral_control.cpp`(main), `src/controller/control.cpp`(bilateral_step),
`config/{leader,follower}.yaml`, `src/periodic_timer_thread.hpp`, `src/robot_state.hpp`.

### 핵심 한 줄
**제어 법칙은 사실상 동일(대칭 P-P bilateral = SPBT). 근본 차이는 "아키텍처".**
enactic = 단일 프로세스 + 공유메모리 커플링 + 500Hz tight loop / 우리 = ros2_control 분산 노드 +
DDS relay 커플링 + 150Hz.

### 비교표
| 항목 | **openarmx_bilateral (우리, v0.3)** | **enactic openarm_teleop** |
|---|---|---|
| **제어 법칙** | 대칭 P-P: τ=Kp(q_peer−q)+Kd(q̇_peer−q̇)+τ_ff | **동일** (대칭 P-P / SPBT) |
| **MIT 명령 생성** | HW 드라이버가 pos=peer, kp/kd, effort=FF를 패킹 | control이 직접 `MITParam{Kp,Kd, pos=peer, vel=peer, eff=FF}` |
| **크로스커플링 경로** | **DDS** (relay_node: leader `/joint_states`→follower cmd 토픽, +CM) | **공유메모리** (AdminThread가 `robot_state` references를 peer 응답으로 set) — **in-process, ~0 지연** |
| **프레임워크** | ros2_control (분산 노드 + DDS 토픽) | 독립 C++ 단일 프로세스 (raw CAN, rclcpp 제어루프 밖) |
| **프로세스/스레드** | 노드 다수(relay·grav·fric·2×CM·HW) + 브링업 2개 | 단일 프로세스, 3스레드: LeaderArmThread / FollowerArmThread / AdminThread(커플링) |
| **제어 rate** | **150Hz 균일** (파이프라인 패치 후; 원래 75/86, USB2CAN 상한) | **500Hz/arm** (PeriodicTimerThread, Ts=1/500) |
| **피드포워드(FF)** | 중력 + 마찰(Coulomb) + 속도FF + posture(J3) | 중력 + 마찰(풀모델). coriolis는 계산하나 **bilateral엔 미적용**(unilateral만 ×0.1) |
| **마찰 모델** | `Fc·tanh(0.1·k·ω)` (Coulomb 위주, Fv≈0, 대충값) | `Fo + Fv·ω + Fc·tanh(k·ω)` (오프셋+점성+Coulomb, 식별값) |
| **중력보상** | 별도 노드 `gravity_comp_node`(KDL) → effort 토픽 | control 내부 `Dynamics::GetGravity` (leader/follower 각각 dynamics 객체) |
| **모터 / CAN 라이브러리** | **Robstride** / openarmx-can (namespace openarmx) | **DM(Damiao)** / openarm_can (namespace openarm) |
| **MIT KP 범위** | RS04/RS03(J1-4) [0,5000], RS00(J5-7,그리퍼) [0,500] | DM 전부 [0,500] (KD [0,5]) |
| **CAN I/O 방식** | ros2_control read/write (파이프라인 패치: read=recv만, write=send만) | `bilateral_step` 내 `mit_control_all` + `sleep 200us` + `recv_all(220us)` |
| **그리퍼** | 별도 처리(위치홀드 + 우리 마찰 패치 2줄) | **동일 MIT 루프의 한 관절**로 취급(Kp/Kd/eff 동일 파이프) |
| **채택 게인(현재)** | kp=35,20,15,8,8,3,1,1 / kd=3.5,5,2.5,0.8,0.8,0.3,0.1,0.1 | kp=240,240,240,240,24,31,25,16 / kd=3×4,0.2×4 (leader=follower) |
| **구성 범위** | bimanual (arm:=left/right/both) | 단일 arm 쌍(leader↔follower), 한 PC가 양쪽 CAN 소유 |
| **BANNED(둘 다 안 씀)** | DOB/RTOB/energy tank/4ch/wave 없음 | 없음 (순수 SPBT + FF) |

### §1.1 관찰·시사점 (v1.0 관점)
1. **법칙은 베낄 게 없다 — 이미 같다.** 둘 다 "MIT kp로 peer 위치를 당기고, 중력+마찰을 FF로 상쇄"하는
   대칭 P-P. 우리가 못 따라가는 건 알고리즘이 아니라 **구현 아키텍처**.
2. **가장 큰 구조 차이 = 커플링 경로.** enactic은 peer 위치가 **공유메모리로 ~0지연** 도달. 우리는
   **DDS relay 홉**(leader js→relay→follower cmd)을 거침. 우리 relay는 200Hz/지터 0.05ms로 깨끗하지만
   여전히 직렬화+토픽+CM 지연이 있는 층. → **in-process 단일-CM bilateral 컨트롤러**(update()에서 커플링
   직접 계산)가 enactic 구조에 대응. transparency의 "지연" 축 후보.
3. **rate: enactic 500Hz vs 우리 150Hz.** 단 이건 알고리즘이 아니라 **USB2CAN full-speed 상한** 탓
   (§LATENCY_INVESTIGATION). enactic의 DM+CAN이 우리 full-speed 동글과 다르면 500이 가능. 우리 150은
   소프트웨어 상한까지 짜낸 값. rate 더는 HS-USB/PCIe(HW) 필요.
4. **마찰 모델이 우리보다 정교(Fv 점성 + Fo 오프셋 + 식별값).** 우리는 Coulomb 대충값. **자유공간
   가벼움(transparency)** 개선 여지 = 여기. friction 재식별 + Fv/Fo 도입이 저비용 카드.
5. **coriolis**: enactic도 bilateral엔 안 씀(계산만). → 우리도 굳이 필요 없음(unilateral ×0.1 참고).
6. **gripper를 1급 관절로 통합** = enactic이 더 깔끔. 우리는 드라이버 하드코딩 우회 중.

### §1.2 열린 질문 (다음 공부 주제)
- Q1. enactic AdminThread(커플링)의 실제 rate와 leader/follower 스레드와의 동기화 방식? (지연·지터 영향)
- Q2. enactic이 실제로 500Hz를 CAN에서 달성하나? 그들의 USB/CAN 하드웨어는? (우리 150 상한과 비교)
- Q3. enactic 마찰 식별 절차(Fc/k/Fv/Fo)는 어떻게? 우리 friction_id에 Fv/Fo 확장 가능?
- Q4. "in-process 단일-CM bilateral 컨트롤러"로 갈 때 ros2_control 안에서 공유메모리 커플링을 어떻게
      구현? (양팔 단일 CM + 커스텀 controller, DDS relay 제거)
- Q5. v1.0의 정의/목표는? (rate? transparency 지표? 아키텍처 전환? 마찰 정교화?) — 계획 수립 필요.

---

## §1.3 모터에 무엇을 쏘고/받는지 + 알고리즘이 그걸 어떻게 쓰는지

### 핵심: 제어 루프는 "모터 안"에서 돈다
bilateral의 실제 힘 계산 `τ = kp·(q_des−q) + kd·(q̇_des−q̇) + τ_ff` 는 **모터 온보드 MIT 펌웨어**가
수행. **호스트는 관절당 숫자 5개(kp, kd, q_des, q̇_des, τ_ff)만 CAN으로 쏜다.** 우리든 enactic이든
이 구조는 동일(DM/Robstride 둘 다 MIT 운동제어 모드).

### 송신(host→motor): MIT 명령 5필드, 각 필드의 출처
| MIT 필드 | 의미 | **우리(openarmx)** 출처 | **enactic** 출처 |
|---|---|---|---|
| `kp` | 커플링 강성 | `kp_values_[i]` (relay가 param 설정) | `Kp_[i]` (yaml) |
| `kd` | 커플링 댐핑 | `kd_values_[i]` | `Kd_[i]` (yaml) |
| `q_des`(position) | **peer 위치** | `pos_commands_[i]` ← relay→forward_position_controller (DDS) | `ref.position` ← AdminThread가 peer 응답을 shared-mem에 복사 |
| `q̇_des`(velocity) | **peer 속도** | `vel_commands_[i]` ← relay→forward_velocity_controller **(vel_ff=true일 때만; 아니면 0)** | `ref.velocity` ← peer 속도 **(항상)** |
| `τ_ff`(torque) | 중력+마찰 FF | `tau_commands_[i]` ← gravity_comp_node + friction_comp_node (DDS→forward_effort_controller) | `gravity[i] + friction[i]` (bilateral_step 내부 계산) |

- **동일**: 둘 다 q_des=peer 위치, τ_ff=중력+마찰. 힘 계산식·모터 역할 같음.
- **차이 ① peer 데이터 경로**: 우리=DDS 다단(motor→read→broadcaster→DDS→relay→DDS→pos_controller→
  cmd iface→write). enactic=AdminThread의 shared-mem 복사 1회(~0지연). **transparency 지연축의 핵심.**
- **차이 ② 속도항**: enactic은 bilateral에서 **peer 속도를 항상** q̇_des로 → `kd(q̇_peer−q̇)`(속도추종).
  우리는 **vel_ff=false면 q̇_des=0** → `kd(0−q̇) = −kd·q̇`(순수 댐핑). **vel_ff=true라야 enactic과 동일.**
  → v1.0에선 vel_ff 기본 ON 고려(현재 우리 채택 명령엔 vel_ff:=true 있음, OK).

### 수신(motor→host): 무엇을 읽고 무엇을 쓰나
| 읽는 값 | 우리 사용처 | enactic 사용처 |
|---|---|---|
| position | relay(커플링), gravity_comp, friction 상태 | joint 변환→응답→AdminThread가 peer로 전파 |
| velocity | relay(vel_ff), friction_comp(τ_fric=f(ω)) | friction, (q̇_des로 peer에 전파) |
| **torque(측정)** | read함(`tau_states_`)→joint_states.effort로 publish하나 **제어법칙엔 미사용** | **아예 안 읽음**(`{pos,vel,0}`) |
- **결론: 둘 다 "측정 토크"를 힘피드백에 안 쓴다.** 힘반력 = 순수 위치오차×kp (SPBT). 즉 **힘센서/토크
  피드백 없는 position-position** 방식. (그래서 kp가 곧 벽강성.)

## §1.4 Coriolis(원심·코리올리) 계산부 비교
| 항목 | 우리(openarmx gravity_comp) | enactic(Dynamics) |
|---|---|---|
| 동역학 라이브러리 | **KDL** `ChainDynParam`(URDF→chain) | **KDL** `ChainDynParam`(동일) |
| 중력 | `JntToGravity` → τ_g (중력 (0,0,−9.81)) | `JntToGravity` (동일) |
| **Coriolis** | **계산 안 함** (GetGravity만 호출) | `GetCoriolis` = `JntToCoriolis(q,q̇)` → **C(q,q̇)q̇ 계산함** |
| 질량행렬 | 없음 | `JntToMass`(있으나 미사용) |
| **bilateral에서 coriolis 적용?** | 해당없음(계산 안 함) | **미적용!** effort = gravity+friction만. coriolis는 계산만 하고 버림 |
| unilateral에선? | — | `effort = gravity + friction×0.3 + coriolis×0.1` (약하게 적용) |
- **핵심 시사점: enactic조차 bilateral에선 coriolis를 안 쓴다**(훅만 남겨둠). 저속에선 C(q,q̇)q̇ 작아
  무시 가능. 고속 동작 transparency엔 이론상 도움되나 enactic도 채택 안 함 → **v1.0에서 coriolis FF는
  저우선.** 필요하면 우리도 KDL `JntToCoriolis` 한 줄 추가로 동일하게 얹을 수 있음(이미 KDL 씀).
- (Q: enactic이 unilateral엔 ×0.1로 넣은 이유 = 고속 시 관성/원심 보상해 조작감↑, 단 노이즈 증폭 위험이라
  스케일 낮춤으로 추정.)

---

## §1.5 포팅 분석 — enactic bilateral을 우리 로봇으로 가져올 때 무엇이 무엇으로 치환되나

전제: enactic `Control`(bilateral_step) + main(3스레드) + Dynamics를 **그대로 가져와**, ①드라이버 API를
우리 openarmx-can으로, ②파라미터를 우리 현재 bilateral값으로 치환.

### 큰 그림 — 아키텍처가 통째로 바뀐다
포팅하면 **우리 ros2_control 스택 전체(2 브링업 + relay + gravity/friction 노드 + CM + 컨트롤러들)를
"단일 프로세스 제어루프"가 대체**한다. enactic처럼 한 프로세스가 leader·follower CAN을 둘 다 열고,
3스레드(leader/follower/admin)로 돌린다.

### 핵심 발견: 드라이버 API가 거의 1:1 (openarmx-can = openarm_can 포크)
| enactic (openarm_can) | 우리 (openarmx-can) | 치환 난이도 |
|---|---|---|
| `openarm::can::socket::OpenArm` | `openarmx::can::socket::OpenArmX` | 이름만 |
| `MITParam{kp,kd,q,dq,tau}` | `MotionControlParam{kp,kd,position,velocity,torque}` | **필드 동일, 이름만** |
| `get_arm().mit_control_all(cmds)` | `get_arm().send_motion_control_commands(params)` | 메서드명만 |
| `recv_all(220)` | `recv_all(220)` | **동일 시그니처(timeout_us)** |
| `get_arm()/get_gripper()/get_motors()` | 동일 이름 | 그대로 |
| `get_position()/get_velocity()` | 동일 | 그대로 |
| `initialize_openarm(iface,true)` | `init_arm_motors(motor_types,...)` | **모터타입/CAN ID 주입 필요(아래)** |
→ **Control 클래스 본체(bilateral_step)는 namespace+struct명 치환 수준의 기계적 포팅.** 힘계산은 어차피
모터 펌웨어가 하므로 로직 변경 없음.

### 진짜 손봐야 할 치환(로봇 종속) — 여기가 실제 작업
| enactic 요소 | 하는 일 | 우리 것으로 치환 | 주의점 |
|---|---|---|---|
| 모터 init (DM, `initialize_openarm`) | 모터 등록 | **모터타입맵 RS04(J1-2)/RS03(J3-4)/RS00(J5-7,그리퍼) + CAN ID 1~8, 그리퍼 0x08** (v10_simple_hardware의 DEFAULT_MOTOR_TYPES) | 필수 |
| `OpenArmJointConverter`(motor↔joint) | DM 기어/부호 변환 | 우리 **direction_multipliers(−1 전부) + 그리퍼 `joint_to_motor_radians`(≈−23.8배 스케일)** | 부호·그리퍼 스케일 우리것 |
| `Dynamics`(KDL ChainDynParam)+URDF | 중력/coriolis | **동일 KDL**, 단 **우리 v10 URDF + 체인 링크명(openarmx_left_jointN, base~link7)** 주입 | URDF만 갈면 됨 |
| `ComputeFriction`(Fo+Fv·ω+Fc·tanh(k·ω)) | 마찰 | 모델 유지, **파라미터=우리 식별 Fc/k** | ⚠️ **k-scale: 우리 tanh(0.1·k) ↔ enactic tanh(k) → 우리 k를 ×10 하거나 식에 0.1 삽입** |
| Kp/Kd (yaml, [0,500]) | 게인 | **우리값 kp={35,20,15,8,8,3,1,1} kd={3.5,5,2.5,0.8,0.8,0.3,0.1,0.1}** | openarmx-can이 모터타입별 범위(RS04 [0,5000] 등) 패킹 처리 |
| 중력 scale | 없음(raw) | 우리 **g_scale=0.93** 곱 추가 | 우리 붕뜸 보정 유지하려면 |
| posture spring(J3) | 없음 | 우리 것 추가할지 선택 | enactic엔 없는 기능 |
| FREQUENCY=500Hz | 제어주기 | **150으로** (우리 USB2CAN 균일상한; 500이면 손목 starve — §LATENCY) | 필수 |
| leader can0 / follower can2 | CAN 매핑 | 우리 매핑(좌팔: leader can1 / follower can3 등) | |
| 단일 arm 쌍 | 구성 | 우리 bimanual 원하면 스레드/객체 2쌍으로 확장 | 선택 |

### 우리 스택에서 사라지는 것 (포팅 시 제거)
- `relay_node`(DDS 커플링) → **enactic AdminThread 공유메모리 커플링**으로 대체
- `gravity_comp_node`, `friction_comp_node` → **Control 내부 계산**으로 흡수
- `forward_{position,velocity,effort}_controller` + 2×`controller_manager` + 브링업 2개 → **직접 CAN 단일 프로세스**
- 우리 **드라이버 파이프라인 패치(latency lever)도 불필요** — enactic 루프는 이미 `send+sleep200us+recv_all`
  1왕복(별도 status 요청 없음) = 우리 패치와 같은 효율. (그래서 rate도 우리 150 상한에 그대로 걸림.)

### 우리 쪽에서 재사용되는 것 (그대로 남음)
- **openarmx-can 라이브러리**(저수준 CAN/모터) — 그대로 링크
- 우리 **URDF, 모터타입맵, CAN ID, 그리퍼 스케일, 부호(−1), 식별 파라미터**
- KDL 동역학(중력)은 enactic Dynamics가 우리 URDF로 그대로 씀

### 요약 판정
- **코드 포팅 자체는 쉽다**(API 1:1, 힘계산은 모터가 함). 로직 재검증 부담 낮음.
- **실제 작업 = 로봇 종속 치환 6개**: (1)모터 init 타입맵 (2)joint↔motor 변환(부호·그리퍼) (3)URDF
  (4)마찰 k-scale 정합 (5)게인/그리퍼 그대로 (6)rate 500→150.
- **가장 큰 이득 = 커플링이 DDS→공유메모리**(지연·지터 감소, transparency↑). 이게 포팅의 주된 명분.
- 리스크: 우리 안전장치(그리퍼 stall 감지 등 ros2_control 드라이버에 있던 것)를 이 프로세스에 다시
  구현해야 함. bimanual·RViz·로깅 등 ros2 편의도 재구성 필요.

---

## §2. (예정) …
> 다음 공부 주제가 생기면 여기에 계속 추가. (열린 질문: enactic 500Hz 실현 하드웨어, 마찰 Fv/Fo 식별
> 절차, in-process 단일-CM 커플링 설계, v1.0 정의.)
