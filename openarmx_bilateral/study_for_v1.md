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
| `ComputeFriction`(Fc·tanh(0.1·k·ω)+Fv·ω+Fo) | 마찰 | 모델 유지, **파라미터=우리 식별 Fc/k** | ✅ **[교정] enactic 실제 코드도 tanh(0.1·k) — 우리와 동일 계수.** k-scale 정합 불필요. 차이는 우리가 **Fv·ω+Fo 항을 뺀 것**뿐(0으로 두면 동일) |
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

## §1.6 심화 분석 ① enactic은 500Hz를 어떻게 내나 (하드웨어)
- 공식 문서(docs.openarm.dev/teleop): **"bilateral은 500Hz 이상 필요"**. CAN 설정 명령이
  `openarm-can-configure-socketcan can0 -fd` / `-4-arms -fd` → **enactic은 CAN-FD를 씀.**
- enactic 모터 = **DM(Damiao)**, CAN-FD 지원. CAN-FD(데이터구간 최대 ~5-8Mbit)로 **프레임당 CAN
  와이어 시간이 classic 1Mbps 대비 급감** → 8모터를 2ms(500Hz)에 송수신 가능.
- enactic 루프도 우리 패치와 같은 **1왕복**(mit_control_all + sleep200us + recv_all(220us), 별도
  status 요청 없음). 즉 rate 차이는 **알고리즘/루프가 아니라 CAN-FD + 어댑터**에서 나옴.
- **⚠️ 결정적: 우리는 500Hz 재현 불가.** (1) Robstride 모터 **CAN-FD 미지원**(제조사 확인), (2) 우리
  USB2CAN 동글 **full-speed**. → 코드를 포팅해도 **우리 rate 상한은 그대로 150Hz.** enactic의 500은
  하드웨어(CAN-FD) 덕이고, 그건 우리 HW를 바꿔야(HS-USB/PCIe+FD지원 모터) 가능.

## §1.6 심화 분석 ② 마찰 Fv/Fo 식별 절차
- **enactic 저장소·문서에 마찰 식별 스크립트/절차 없음.** Fc/k/Fv/Fo는 yaml에 **이미 튜닝된 상수**로만
  제공(찾음: friction/identif 관련 코드 0개). 즉 "그들의 식별법"은 공개돼 있지 않음.
- 모델은 `Fc·tanh(0.1·k·ω) + Fv·ω + Fo` (§1.5 교정). 우리는 Fc·tanh만 씀(Fv≈0, Fo=0).
- **우리가 Fv/Fo를 얻는 법(포팅 불필요, 우리 friction_id 확장):** 기존 등속 셔틀(friction_id_node)로
  관절별 여러 속도에서 `effort−gravity=friction` 측정 → **4파라미터 최소자승 적합**(Fc·tanh 홀함수 +
  Fv·ω 선형기울기 + Fo 오프셋). 현재 Coulomb만 적합하는 걸 4파라미터 적합으로 확장하면 끝. **저비용.**
- 시사점: 자유공간 가벼움(transparency)을 올릴 **가장 싸고 독립적인 카드**. 포팅과 무관하게 우리
  friction_comp에 Fv·ω+Fo만 추가하면 됨.

## §1.7 공식문서 정독 비교 (docs.openarm.dev vs openarmx.com)
- **docs.openarm.dev/teleop** 정독함: setup-guide / unilateral / bilateral / vr.
  - bilateral: 500Hz+, 6파라미터(Kp,Kd,Fc,k,Fv,Fo), 3스레드(leader/follower/admin), CAN-FD,
    zero=팔 아래로 내림, leader can0/1·follower can2/3, KDL/eigen/urdfdom 의존. 실행
    `./script/launch_bilateral.sh right_arm can0 can2`.
  - **공개 bilateral = "classical position–force"(=우리가 분석한 SPBT).**
  - **고급판 = DOB(외란관측기) 가속도제어 + sensorless force + 4채널** → **비공개(연구파트너 한정),
    미공개.** ⇒ 우리가 가져올 수 있는 "더 나은 것"은 없음. 게다가 **DOB/4채널은 우리 BANNED 목록**(UR10e
    실패작). 즉 방향도 우리가 이미 배제한 것.
- **openarmx.com(=Chengdu Changshu Robot, Robstride OEM)**: **docs.openarmx.com TLS 인증서 오류로
  전 경로 접근 불가.** 대신 설치된 **openarmx-can 헤더가 authoritative** — 모터타입(RS00/RS03/RS04),
  KP범위(RS04/03 [0,5000], RS00 [0,500]), API(§1.5)를 이미 확보. openarmx는 **openarm의 하드웨어
  변형(DM→Robstride)이고 소프트웨어(openarmx-can)는 openarm_can 포크**임이 API 평행성으로 확인됨.
  즉 openarmx 고유의 별도 bilateral 제어법은 없음(같은 openarm 계열).

## §1.8 ⭐ 의사결정 프레임 (한 판 요약)
**사실 정리:**
1. 제어 **법칙은 이미 동일**(SPBT position-force). 공개 enactic에서 베낄 알고리즘 이득 = **없음**.
2. enactic 고급판(DOB/4ch)은 **비공개 + 우리 BANNED** → 방향 자체 제외.
3. 포팅의 **유일 실이득 = 커플링 DDS→공유메모리**(지연·지터↓). rate는 **CAN-FD 의존이라 우리 못 올림
   (150 고정)**.
4. 마찰 정교화(Fv/Fo)·게인 관절별은 **포팅과 무관하게 우리 스택에서 가능**(저비용).
5. 포팅 비용 = 코드는 쉬움(API 1:1)이나 ros2_control 편의(그리퍼 stall·bimanual·RViz·안전) 재구현.

**선택지:**
| 옵션 | 내용 | 얻는 것 | 잃는 것/비용 |
|---|---|---|---|
| **1. enactic 통째 포팅** | 단일프로세스+공유메모리로 전환 | 커플링 지연↓ | ros2_control 생태계·안전장치 재구현, rate는 그대로 150 |
| **2. ros2_control 유지 + 체리픽** | (a) in-process 단일-CM 커플링 컨트롤러 (b) friction Fv/Fo (c) 관절별 kp | 커플링 지연↓ + 자유공간 가벼움, 생태계 유지 | 커스텀 컨트롤러 개발 |
| **3. 현상 + 튜닝만** | 관절별 kp + friction Fv/Fo만 | 저비용 개선 | 커플링 지연은 남음 |

**분석자 의견(참고):** 포팅(1)의 유일 이득(공유메모리 커플링)은 **옵션2의 (a)로 ros2_control 안에서도
얻을 수 있고**, rate·알고리즘 이득은 어차피 없음 → **통째 포팅의 ROI는 낮아 보임.** v1.0을 "transparency
개선"으로 정의한다면 **옵션2(친화적·점진적)**가 유력: friction Fv/Fo(자유공간) + 관절별 kp(벽강성) +
in-process 커플링(지연). 단 **v1.0의 목표 정의**가 먼저 필요(아래 Q5).

**미해결/운영자 결정 필요:**
- Q5. **v1.0의 정의는?** (예: "transparency를 체감 개선" / "아키텍처를 enactic식으로 전환" /
  "마찰·게인 완성" 중 무엇). 이게 옵션 선택을 결정.
- HW 투자(HS-USB/PCIe+FD모터)로 rate 상한을 깰 의향이 있나? (있으면 500Hz급 가능, 없으면 150 고정.)

---

## §2. 포팅 시 파일/폴더 구조 (옵션1 = enactic식 단일프로세스 전제)

> ⚠️ 이 구조는 **옵션1(통째 포팅)** 용. 옵션2(ros2_control in-process 컨트롤러)면 구조가 완전히 다름
> (ros2_controller 플러그인). v1.0 정의 후 확정.

### 원칙: "로봇/라이브러리 종속"은 port 레이어로 격리, 나머지는 upstream 최대 보존
enactic이 이미 `src/openarm_port/`로 하드웨어 종속을 격리해둠 → 우리는 **그 레이어만 openarmx-can으로
바꾸고, 나머지 로직(control/robot_state/timer)은 namespace·struct명 치환 수준으로 복사.**

### 제안: 새 패키지 `openarmx_teleop` (ss_prep_260413 안, openarmx_bilateral와 별개로 공존)
```
openarmx_teleop/
├── CMakeLists.txt          # ament_cmake, link: libopenarmx_can, orocos-kdl, eigen, urdfdom
├── package.xml
├── config/
│   ├── leader.yaml         ← 우리 파라미터 (kp=35,20,15,8,8,3,1,1 / kd=... / Fc/k/Fv/Fo / g_scale=0.93)
│   └── follower.yaml
├── control/                # 실행파일 main (= enactic control/)
│   ├── openarmx_bilateral_control.cpp    # 3스레드(leader/follower/admin), FREQUENCY=150
│   ├── openarmx_unilateral_control.cpp
│   └── openarmx_comm_test.cpp
├── src/
│   ├── controller/         # ★ 로직 — enactic에서 거의 그대로 복사
│   │   ├── control.{hpp,cpp}    # bilateral_step: MITParam→MotionControlParam, mit_control_all→send_motion_control_commands
│   │   ├── dynamics.{hpp,cpp}   # KDL 중력/coriolis — 우리 URDF/링크명 주입
│   │   └── diff.hpp             # 미분기 (그대로)
│   ├── robot_state.hpp          # 공유메모리 커플링 (그대로 — 이게 DDS relay 대체)
│   ├── periodic_timer_thread.hpp# RT 주기 루프 (그대로)
│   ├── joint_state_converter.hpp# (그대로/약간)
│   ├── yamlloader.hpp           # (그대로)
│   ├── openarmx_port/      # ★★ 치환 핵심 (= enactic src/openarm_port/)
│   │   ├── openarmx_init.{hpp,cpp}   # OpenArmX init: RS04(J1-2)/RS03(J3-4)/RS00(J5-7,그리퍼) + CAN ID 1~8, 그리퍼 0x08
│   │   └── joint_mapper.{hpp,cpp}    # 우리 부호(−1 전부) + 그리퍼 joint↔motor(≈23.8배) 스케일
│   └── openarmx_constants.hpp   # 모터타입맵/DOF/CAN ID/KP·KD 범위
├── urdf/
│   └── v10_arm.urdf             # KDL용 우리 로봇 URDF (좌/우 체인)
└── script/
    ├── launch_bilateral.sh      # CAN up(classic 1M) + 실행 (leader can1 / follower can3 등)
    ├── launch_unilateral.sh
    └── launch_grav_comp.sh
```

### 파일별 작업 유형 (포팅 부담 한눈에)
| 유형 | 파일 | 작업 |
|---|---|---|
| **거의 그대로 복사**(framework) | robot_state.hpp, periodic_timer_thread.hpp, yamlloader.hpp, controller/diff.hpp | 복사 |
| **치환 복사**(namespace/struct명) | controller/control.{hpp,cpp}, controller/dynamics.{hpp,cpp}, joint_state_converter.hpp, control/*_control.cpp | MITParam→MotionControlParam, OpenArm→OpenArmX, mit_control_all→send_motion_control_commands, FREQUENCY 500→150 |
| **우리 로봇용 새로/재작성**(port) | openarmx_port/openarmx_init, openarmx_port/joint_mapper, openarmx_constants.hpp, config/*.yaml, urdf/ | RS 타입맵·CAN ID·부호·그리퍼스케일·파라미터·URDF |

### 매핑 요약 (enactic → 우리)
- `src/openarm_port/openarm_init` → `src/openarmx_port/openarmx_init` (OpenArmX + RS 타입)
- `src/openarm_port/joint_mapper` → `src/openarmx_port/joint_mapper` (우리 부호+그리퍼)
- `src/openarm_constants.hpp` → `src/openarmx_constants.hpp`
- `control/openarm_bilateral_control.cpp` → `control/openarmx_bilateral_control.cpp` (Hz 150)
- `config/{leader,follower}.yaml` → 우리 파라미터
- controller/·framework → 복사(치환)

### 결정 의존성 / 주의
- **이 구조는 옵션1 전제.** 채택 전 §1.8의 v1.0 정의 필요.
- ros2_control 편의(그리퍼 stall 감지·bimanual·RViz·로깅·안전정지)는 **이 패키지에 재구현** 필요 → 위
  구조에 향후 `src/safety/`, bimanual용 스레드쌍 확장 여지 있음.
- **openarmx_bilateral(현 ros2_control판)은 남겨둠** — 폴백·비교·운영 지속.

## §2.1 확정된 포팅 입력값 (운영자 지시 반영)

### (a) 실행 = enactic `launch_bilateral.sh` 기준으로 포팅
enactic 스크립트 흐름: `ARM_SIDE(left/right)` + CAN 인자 → **xacro로 URDF 생성** → 바이너리에
`(leader_urdf, follower_urdf, arm_side, leader_can, follower_can)` 전달 → 실행 → temp 정리.
우리 포팅판 `launch_bilateral.sh` 치환:
| enactic | 우리 |
|---|---|
| `WS_DIR=~/openarm_ros2_ws` | `~/openarmx_ws` |
| `XACRO=openarm_description/urdf/robot/v10.urdf.xacro` | **`openarmx_description/urdf/robot/v10.urdf.xacro`** |
| `xacro ... bimanual:=true -o leader.urdf` | 동일(우리 xacro) |
| `BIN=~/openarm_teleop_tmp/build/bilateral_control` | 우리 빌드 산출물 `.../openarmx_teleop .../bilateral_control` |
| CAN 기본: right→leader can0/follower can2, left→leader can1/follower can3 | **동일**(우리 매핑과 일치) |
| 실행: `./launch_bilateral.sh right_arm can0 can2` | 동일 인터페이스 유지 |

### (b) URDF = openarmx_bilateral가 쓰는 것 (우리 모델/물성치)
- **`openarmx_ws/src/openarmx_description/urdf/robot/v10.urdf.xacro`** (bimanual:=true로 생성).
  enactic openarm과 **모델·관성·질량이 다르므로 enactic URDF 금지, 반드시 우리 것.** KDL Dynamics가
  이 URDF로 중력/coriolis 계산 → 우리 로봇 물성치 반영.
- (openarmx_bilateral의 gravity_comp도 이 xacro 유래 URDF 사용 → 일관.)

### (c) 관절 방향(부호) — 드라이버와 동일하게
- **`direction_multipliers = {−1,−1,−1,−1,−1,−1,−1}`** (7관절 전부 −1, **양팔 공통**). motor↔joint 변환:
  `joint = −1 × motor` (pos/vel/tau 모두). 그리퍼(J8)는 `motor_radians_to_joint`(≈−23.8배 스케일) 별도.
- **좌우 미러는 방향계수가 아니라 URDF 조인트축이 담당** → port의 joint_mapper는 −1 균일 적용, 미러는
  URDF에 맡김. (openarmx_port/joint_mapper에 이 규칙 이식.)

### (d) 관절 제한각 파일 = **부호(방향) 검증 기준** (clamp 아님) [운영자 확정]
- **용도: clamp/soft-limit 금지.** 이 파일은 "각 관절의 **+ 방향이 물리적으로 어느 쪽인지**"를 알려주는
  **부호 검증 레퍼런스**. 목적 = 포팅 시 **모터 부호가 뒤집혀 `kp(q_peer−q)`가 양의 피드백 → 날뛰는 것**
  을 방지. (제어부에 각도 한계를 하드코딩하지 않는다는 과거 원칙과도 일치.)
- 각 관절 + 방향(파일 원문): 예) L-J1 `+`=몸 뒤(45)/`−`=몸 앞(−135), R-J1은 미러(`+`=몸 앞/135).
  L-J4 `+`=팔꿈치 굽힘(100), L-J6 `+`=손목 바깥(45), L-J7 `+`=손목 뒤(80) … (좌우 미러).
- **적용 방법**: 방향계수는 **드라이버 authoritative = −1 균일 (§c)** 로 이식하고, **이 파일은 bring-up
  때 "각 관절을 + 방향으로 움직였을 때 파일 설명과 일치하는지" 검증**에 사용. 불일치 관절 = 부호 반전
  → 해당 관절 sign 교정(joint_mapper). 값은 참고/로깅용으로 config에 주석 보존(가능하면), **명령 clamp
  는 하지 않음.**
- (참고 라디안 환산표는 필요 시 §2.1 이력에서: L J1 −2.356~0.785 … 등. 단 제어엔 미사용.)

### 미해결(운영자 확인) — [해소됨]
- ~~제한각을 clamp?~~ → **아니오. 부호(방향) 검증용 참고만. clamp 없음.** (2026-07 확정.)

---

## §3. openarmx_bilateral(v0.3)에서 v1.0/포팅에 가져올 참고자산 리스팅

### A. 재사용 파라미터 (검증된 값 — 그대로 이식)
- **마찰 Fc/k (관절별, tanh(0.1·k·ω) — enactic 코드와 동일 모델):**
  - LEADER Fc `[1.21,0.63,0.31,0.39,0.14,0.17,0.13]` k `[81,51,40,52,36,20,48]`
  - FOLLOWER Fc `[1.04,1.06,0.35,0.36,0.16,0.15,0.12]` k `[68,60,26,87,62,57,39]`
- **채택 게인** kp `[35,20,15,8,8,3,1,1]` kd `[3.5,5,2.5,0.8,0.8,0.3,0.1,0.1]` (관절별, leader=follower).
- **g_scale=0.93**(중력 과보상 붕뜸 보정), **friction_scale=0.7**, **couple_sign=+1**(좌우 HW검증).
- 그리퍼(J8) 게인 0.3/0.03(leader). posture J3 kp1.8/kd0.18(참고, 옵션).
- (벽강성 상한 참고: enactic 근위 240 — ENACTIC_REFERENCE_GAINS.md.)

### B. 로봇 하드 사실 (port config/constants로 이식)
- 모터타입: RS04(J1-2)/RS03(J3-4)/RS00(J5-7,그리퍼), CAN ID 1~8, 그리퍼 0x08.
- MIT 범위: RS04/03 KP[0,5000]KD[0,100], RS00 KP[0,500]KD[0,5].
- 방향계수 **−1 균일(7관절)**, 그리퍼 motor_radians_to_joint(≈−23.8배). 좌우미러=URDF.
- 관절 방향/한계 파일(부호 검증용, §2.1d). URDF=openarmx_description v10.urdf.xacro.

### C. 알고리즘/로직 자산 (개념·코드 재사용)
- **friction_id_node**: 등속 셔틀(kp=0,중력ON) + **odd 대칭화 피팅** → Fc/k. **Fv/Fo 4파라미터로 확장** 시 재사용.
- **friction_comp 모델**: `grav + scale·Fc·tanh(0.1k·ω) (+posture)`.
- **relay cross-coupling**: peer pos(+vel) 중계 로직(= enactic AdminThread 개념).
- **gravity_comp**: KDL JntToGravity(우리 URDF). posture 스프링(oa_fd_cpp 유래, J3).

### D. 측정/진단 도구 (그대로 유용 — v1.0 검증 인프라)
- **rate_probe**(ROS rate/지터), **latency_capture**(candump), **chirp_node**(대역폭/지연),
  **friction_log_node**(effort 분해), **log_node**(cmd-vs-act), **joint_echo_node**(관절각).
- 분석 패턴: chirp CSV → FFT 대역폭/상호상관 지연/dt std 지터.

### E. 핵심 통찰 = v1.0 설계 지침 (★ FORCE_FEEDBACK_NOTES.md 필독)
- **"벽 = 아주 뻣뻣한 스프링"** — P-P엔 wall-detection 없음. 반력=Kp×위치오차.
- **★ 뻑뻑함의 원인 = 자유공간 추종오차이지 게인 아님.** 자유공간 추종 완벽하면 오차≈0→힘≈0(고Kp라도).
  enactic이 Kp240에도 가벼운 이유 = 추종오차≈0(마찰보상+속도FF+중력+경량HW+고속루프).
  **우리 Kp60에도 뻑뻑 = 추종충실도가 병목.**
- **방향(합의)**: "Kp 낮춰 가볍게"가 아니라 **follower 추종오차→0**을 올리면 가벼움+벽 동시 해결.
  → 객관지표 = **자유공간 leader↔follower 위치오차 로깅**(크면 그게 뻑뻑).
- **P-P 투명도 천장**: force-sensing/DOB/4ch 없이는 rigid wall 불가(→ 매우 뻣뻣한 스프링이 최선). 단 그것들은 BANNED.
- **MIT Kim 데모의 정체 = QDD 하드웨어 투명도 + 고대역폭 + 단순 P-P.** 알고리즘 아님 →
  **inertia/coriolis FF 저우선.** 우리 약점 = Robstride 마찰↑·관성↑·토크추정치 품질(블랙박스 FOC).
- 미해결: **대칭(enactic) vs 비대칭(follower를 leader보다 뻣뻣하게)** 게인 — v1.0에서 실험.

### F. 하드웨어/병목 지식 (LATENCY_INVESTIGATION.md, tech_debt.md)
- **150Hz USB2CAN 균일상한**(full-speed 동글), CAN-FD 불가(Robstride). 500Hz=CAN-FD 필요(enactic).
- 파이프라인 패치(§8d) — **포팅판엔 불필요**(enactic 루프 이미 1왕복). rate 상한은 그대로 150.
- 팡팡/지터 원인 = CM blocking + update_rate>상한 손목starve(USB tail 아님).

### G. 드라이버 패치/주의 (driver_patches/)
- 그리퍼 velocity·torque 하드코딩 → **포팅판은 우리가 직접 send_motion_control_commands라 우회됨**(이점).
- **그리퍼 stall 감지**(위치오차 큰데 안움직이면 lock) — 포팅판에 안전장치로 재구현 고려.

### H. 함정/교훈 (반복 방지 — history §13)
- 중력 과보상(g_scale>1)→관절 붕뜸. posture가 커플링 kp에 33배 압도됨.
- 마찰 시작 떨림(steep tanh×정지노이즈, J1 최악) → **vel_eps deadband/속도 LPF** 준비됨.
- couple_sign 부호 잘못→미러 반대. friction-test 각도한계 **제어부 하드코딩 금지**(부호검증만).

### I. BANNED (절대 금지 — UR10e 실패작)
- DOB/RTOB/RFOB, energy tank, 4-channel, wave variable. (enactic 고급판도 DOB/4ch=여기 해당.)

### J. 문서 인덱스 (읽는 순서)
1. **FORCE_FEEDBACK_NOTES.md** — v1.0 설계 지침(벽/뻑뻑/MIT 분석) ★최우선
2. **history.md** — 전체 서사 + 하드-원 교훈 §13
3. **ENACTIC_REFERENCE_GAINS.md** — 게인 scale 비교
4. **LATENCY_INVESTIGATION.md** / **tech_debt.md** — rate·병목·부채
5. **STATUS.md** — phase별 A/B 결과

---

## §4. (예정) …
> v1.0 정의 후: (옵션1) 위 구조로 스켈레톤 / (옵션2) ros2_control in-process 컨트롤러 설계.
