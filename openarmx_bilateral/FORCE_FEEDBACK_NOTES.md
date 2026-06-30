# Bilateral Force-Feedback — Design Discussion Notes

Running notes for tuning the bilateral force feedback (Final Phase). See STATUS.md
for the build/run/commands; this file is the *reasoning & decisions* log.

## Goal (operator's target feel)
Reference: MIT Sangbae Kim lab 3-DOF leader–follower demo the operator has used —
- **Follower blocked (wall/object) → leader becomes ~immovable** (push hard, barely
  moves) = a true "hard wall", controllability→0 at contact.
- **No contact (normal) → very smooth & fast**, near-effortless.

Current v2.0 (leader_kp≈60 uniform, kd 0.5, P1+P2+P3 on) gives, per operator:
1. **Too stiff (뻑뻑)** — wrist-fatiguing over long use.
2. **Contact feels like a SPRING, not a hard wall.**

## Key finding 1 — our method == enactic == position-position (P-P)
Verified in enactic source (openarm_teleop control.cpp + AdminThread):
```
each arm:  τ = Kp·(peer_pos − own_pos) + Kd·(peer_vel − own_vel) + (gravity + friction)
```
AdminThread sets each arm's reference = the *other* arm's response (peer). This is
identical in structure to ours: relay feeds peer position (+peer velocity via
vel_ff) into the MIT controller, with gravity+friction as torque feedforward.
There is **no wall-detection / if-logic** in enactic. → Reflected force is
`Kp × position-error` = a **spring**. A "wall" is just a *very stiff spring*.

enactic config (Damiao): **symmetric** leader=follower, per-joint
`Kp=[240,240,240,240, 24,31,25, 16]`, `Kd=[3×4, 0.2×4]`, full friction (incl Fv).
So enactic's wall = high shoulder Kp (240, ~4× ours) + per-joint + friction comp.

## Key finding 2 — "뻑뻑" is NOT caused by high Kp
`reflected force = Kp × (leader↔follower tracking error)`.
- **Free space, perfect tracking → error≈0 → force≈0 regardless of Kp.** High Kp is
  "invisible" in free motion and only "appears" at contact (where error spikes).
- enactic is smooth at Kp=240 **because its free-space tracking error ≈ 0**:
  friction comp (follower not sticky) + velocity FF + gravity comp + light hardware
  + fast loop (~500Hz–1kHz) keep follower glued to leader.
- **Our 뻑뻑 at only Kp=60 ⇒ our free-space tracking error is large** ⇒ follower
  tracking fidelity is the bottleneck, NOT the gain.

Likely error sources for us: friction comp incomplete (scale 0.7, Fv dropped,
fit R²≈0.4–0.85), follower Kp at HW default (loose), relay 200Hz, Robstride inertia.

## Reframed direction (consensus so far)
Not "lower Kp to feel light." Instead **raise follower tracking fidelity so
free-space error→0**, which:
- lightens free motion (뻑뻑 fix), AND
- lets us raise Kp → hard wall (spring fix),
simultaneously — the same thing that makes enactic both light and wall-like.

Objective metric proposed: **log free-space leader↔follower position error**; if
large, that IS the 뻑뻑. Improve comp → error shrinks → lighter.

## Levers available (within P-P; DOB / 4-channel / wave are BANNED)
- Per-joint Kp (shoulder↑ for wall+tracking, wrist↓ to de-stiffen the grip).
- Follower stiffness↑ (rigid against wall = sharper wall onset + tighter tracking).
- Friction comp completeness↑ (scale→1, add Fv, better fit) = less follower lag.
- Velocity FF / filtering, higher relay rate.
- Symmetric (enactic) vs asymmetric (follower stiffer than leader) — open question.

## Honest ceiling
Pure P-P (2-channel position-position) has a transparency limit: with finite,
stable gains you approach but never reach a *rigid* wall — it stays a *very stiff
spring*. Breaking that ceiling usually needs force-sensing / DOB / 4-channel, which
are banned here (failed UR10e approaches). So target = "much stiffer wall + much
lighter," not literally infinite stiffness.

## TODO / open
- Review MIT Sangbae Kim lab papers (operator to provide) for techniques applicable
  to our P-P + ros2_control MIT + Robstride setup; identify which parts are
  adoptable and where our control algorithm would need partial modification.
  NOTE: the Sangbae Kim 3-DOF demo reportedly gives exactly the target feel
  (hard wall at contact, very smooth/fast free motion) — understand *how*.
- Decide symmetric vs asymmetric gains; where the 뻑뻑 dominates (wrist vs whole-arm,
  motion vs static).

## MIT Sangbae Kim lab papers — applicability (analysis only, no dev)
Common root = **proprioceptive / quasi-direct-drive (QDD) actuation**: low-gear,
high-torque-density, low-inertia, backdrivable motors; force read from **motor
current** (no force sensor, no DOB). The "hard wall + smooth free motion" is mostly
a HARDWARE (actuator transparency) + BANDWIDTH result; control is simple
impedance / position-position. NOT a special algorithm, NOT DOB → no clash with bans.

Operator note: saw the demo **2019–2020**, so 2025/26 inertia/Coriolis FF was NOT in
it. ⇒ the "perfection" came from QDD transparency + high bandwidth + simple P-P,
**without advanced model compensation.** So inertia/Coriolis FF is LOW priority for us.

Per paper: 2012 ICRA / 2017 T-RO = actuator hardware design (can't adopt; confirms
principle; our Robstride = same QDD family). 2022 IROS = the closest platform to the
demo. 2025/26 telepresence (human-arm dynamics) = leader inertia comp (low priority
per above). Ramos&Kim whole-body = humanoid balance (not arm-FF; skip).

### 2022 IROS platform vs ours (v2.0) — gap table
Specs (2022): 2× QDD arms, **6-DOF**, reach 0.96m, 13.8kg; controlled over **SPI at
500Hz** by an i7 SBC; custom PCB → **CAN up to 3kHz** per actuator; proprioceptive
(current) force feedback.

| 라벨 | 구분 | MIT 2022 (Kim lab) | 우리 openarmx v2.0 | 영향 / 대응 |
|---|---|---|---|---|
| **a** | HW·액추에이터 | QDD BLDC+저감속 유성기어, 고투명·저마찰·저관성 | Robstride QDD(CAN) — 같은 계열이나 마찰 큼(어깨 Coulomb~1Nm) | 투명도 격차 → **대응불가(HW)**, 마찰보상으로 일부 회복 |
| **b** | HW·DOF/질량 | 6-DOF, 13.8kg 경량 | 7-DOF bimanual, 더 무겁/관성↑ | lag↑ → **대응불가(HW)** |
| **c** | 힘 센싱 | proprioceptive(모터 전류→토크) | proprioceptive(MIT모드: 모터가 전류로 토크 추정·보고) | **사실상 동일 패러다임** (차이=구현, 아래) |
| **d** | 통신·모터버스 | 커스텀PCB, CAN 최대 3kHz | CAN(ros2_control HW IF) | 주기 격차 가능 — *나중 논의* |
| **e** | 통신·토폴로지 | i7 SBC→SPI→PCB→CAN, 직결·최소 홉 | ROS2 DDS 토픽 다단(relay→pos ctrl→HW; grav→grav_only→fric→eff) | 지연·지터↑ — *나중* |
| **f** | 실시간성 | 거의 RT, 결정론적 | 일반 Linux+DDS, 비RT 지터 | 안정 Kp 상한↓ — *나중* |
| **g** | 제어주기 | 메인 500Hz + CAN 3kHz | controller_manager 100Hz + relay 200Hz | 최대 격차(벽 선명도·추종) — *나중* |
| **h** | 제어법+보상 | P-P/임피던스 + 전류토크, 단순보상 | P-P(relay)+MIT kp/kd+중력/마찰/속도FF | 알고리즘 동급 → 격차 아님 — *나중* |
| **i** | SW 스택 | 베어메탈/RT 펌웨어+SBC | ROS2 Jazzy, 노드/DDS, colcon | 추상화·홉 오버헤드 — *나중* |

### 항목별 논의 결정
- **a, b = 대응 불가** (하드웨어 교체 불가; a의 마찰만 SW 마찰보상으로 부분 회복).
- **c (힘 센싱) — 사실상 동일.** "MIT mode" 자체가 MIT mini-cheetah 액추에이터 명령
  프로토콜 {p,v,kp,kd,τ}에서 유래 → Robstride MIT mode = cheetah 방식 그대로. 둘 다
  **별도 force/토크 센서 없이 모터 전류로 토크 추정**(proprioceptive). 차이는 구현뿐:
  - 임피던스 루프 위치: MIT=호스트(i7)에서 토크법칙 계산+커스텀 드라이버 FOC 전류루프
    / 우리=Robstride **펌웨어 onboard(~kHz)**에서 kp·err+kd·err+τ_ff 계산, 호스트는
    setpoint만 100-200Hz 전송. → **강성 렌더링은 우리도 모터 onboard 고속**이라 100Hz에
    안 묶임(불리X).
  - 접근성: MIT=raw 전류·FOC 직접 접근/튜닝/캘리브 / 우리=Robstride 블랙박스(고정 FOC,
    보고되는 토크 추정치만 사용). → 우리 약점 = **토크 추정치 품질(노이즈/필터/Kt
    선형성)에 의존**(마찰 ID R²이 애매했던 한 원인일 수 있음). 하지만 패러다임 격차는 아님.
  - 결론: c는 동등(둘 다 proprioceptive). Robstride 펌웨어라 c 자체도 우리가 못 바꾸지만,
    **불리한 격차가 아니므로 신경 안 써도 됨.**
- **d~i = 나중에 다시 논의** (대역폭·지연·RT·제어주기·제어법·SW 스택).

### d/f/g — PreemptRT PC로 교체하면?
PreemptRT는 커널 스케줄 지터를 ms→수십µs로 bound. 라벨별:
- **f 실시간성: ~해결(핵심 타겟)**. 단 커널만으론 부족 — RT 우선순위(SCHED_FIFO),
  CPU 격리(isolcpus), 메모리 락(mlockall), RT CAN 드라이버 동반 필요.
- **g 제어주기: 상당부분**. CM 100Hz→500Hz~1kHz 안정화 가능(=MIT 500Hz 메인루프급).
  천장은 CAN(d)+사이클 연산. 액추에이터 3kHz는 g 아닌 d/HW.
- **d 모터버스: 부분(~30-50%)**. CAN I/O 지터는 개선, **대역폭/토폴로지(1Mbps,
  버스당 모터수, CAN-FD)는 불변** → 3kHz급은 CAN-FD/버스분할/드라이버 = HW·설정.
- ⚠️ 구조 함정: 양방향 커플링이 **ROS2 토픽 경로**면 그 DDS 지연은 RT가 안 없앤다.
  풀효과 = 커플링을 **controller_manager RT update() 안 커스텀 컨트롤러로 이전**해야 함.
  커널 단독 = 필요조건이지 충분조건 아님.

### e/i — 병목 = ROS2 DDS + 멀티노드 토픽 홉 구조. 대안(간단)
- (1) **단일 RT 컨트롤러 in-process**: 양팔을 한 controller_manager에 올리고(현재는
  leader/follower 별도 CM!) 커스텀 bilateral 컨트롤러가 update() 안에서 양팔 state/command
  인터페이스를 직접 읽고씀 → **루프에서 DDS 완전 제거**. 가장 MIT스럽고 효과 큼.
- (2) **intra-process / 공유메모리**: rclcpp intra-process(zero-copy), 또는 Iceoryx/
  Zenoh shared-mem RMW. 전송비↓(executor 오버헤드는 남음).
- (3) **DDS 튜닝**: RMW(Cyclone/Fast) 교체, best-effort QoS, 홉 최소화 — 완화일 뿐.
- (4) **ROS 우회**: 내부루프는 전용 C++ RT 프로세스가 CAN 직접(=MIT i7 SBC 방식), ROS는
  비-RT 감독만. 가장 근본적·가장 큰 작업.

### h — 우리 vs MIT (정정: 비교 대상은 MIT, enactic은 소스 확인된 proxy)
주의: enactic 수식은 소스로 확정했으나, **목표는 MIT**다. MIT 2022 정확한 수식은
**원문 PDF 깨짐 + 검색으로 미확정** → 아래는 확신도 표기.
- **[확실] 계열**: 양방향 4종(SPBT 위치-위치 / FRBT 힘반사 / FSBT / ABC=DOB계열).
  MIT는 force센서 없이 전류기반 토크 + 위치매핑 → **SPBT/관절임피던스**가 거의 확실.
  **ABC/DOB 아님.** 힘 핵심항 `Kp·(peer−own)+Kd·(peer_vel−own_vel)` = 우리와 동일 골격.
- **[추론] 보상**: QDD가 투명해 **명시적 중력/마찰 보상이 거의/전혀 불필요** 가능성 큼 →
  **MIT 수식이 우리보다 오히려 더 단순**할 것. 우리는 HW 마찰을 메우려 보상을 "추가"한 것.
- **[미확정]**: MIT의 정확한 수식, 좌표계(관절 PD vs task-space 임피던스).
- **결론**: "MIT의 비법 수식"을 베끼는 게 아님. 같은 골격(SPBT)에서, 부족한 HW 투명도는
  보상으로(h의 "살"), 부족한 대역폭/결정론은 RT로(d/f/g) 메우는 게 길. MIT 완벽함의 출처
  = 수식이 아니라 **QDD 투명도 + 대역폭 + 충분히 단단한 단순 P-P**.
- (참고/enactic, 소스확정) 우리=중력+Coulomb / enactic=중력+Coulomb+점성Fv+offset Fo(+일부
  coriolis), Kp 240(관절별) vs 우리 50~60(균일). enactic은 MIT와 같은 SPBT 계열의 공개 구현.
- TODO(원하면): MIT 플랫폼 관련 석사논문 찾아 정확한 제어식 확정.

### h — MIT 정확한 제어식 확보 (CONFIRMED, ar5iv Eq.4)
논문: SaLoutos, Stanger-Jones, Kim, "Fast Reflexive Grasping with a Proprioceptive
Teleoperation Platform" (IROS 2022). Eq.4 (관절별 i):
```
follower: τ_f,i = Kp(q_l,i − q_f,i) + Kd(q̇_l,i − q̇_f,i) + τ_g
leader:   τ_l,i = − τ_f,i
```
joint-space, 대응관절 미러링, 500Hz + CAN 3kHz, 중력보상 O, **마찰/관성 보상 X**,
proprioceptive(force센서 없음).
- 대수전개: τ_l = −τ_f = Kp(q_f−q_l) + Kd(q̇_f−q̇_l) − τ_g → **leader 커플링항이 우리
  대칭 P-P와 동일.** τ_l=−τ_f 는 "작용-반작용 표기"일 뿐 구조 차이 아님. **같은 골격
  확정(수식 레벨).**
- 차이: **마찰보상 없음**(QDD 투명 → 불필요; 우리는 추가=HW 부족분 대체), 중력 작용-반작용
  표기, 게인 미공개(타 출처 높음 추정), 좌표계는 우리와 같은 관절공간.
- **최종 결론**: "비법 커플링항" 없음. MIT 느낌 = 수식 아님 → **QDD 투명도 + 500Hz/3kHz
  대역폭 + (추정)높은 게인**. 우리 경로(마찰보상으로 투명도 회복 + RT/대역폭(d/f/g) +
  게인 튜닝) = 검증된 정답. enactic(대칭 P-P+마찰)도 같은 SPBT 계열 — 셋 다 동일 골격.

### 액션 → 해결/개선 항목 매핑 (무엇을 하면 a~i 중 무엇이 풀리나)
| # | 액션 (무엇을 어떻게) | 해결/개선 항목 | 비고/전제 |
|---|---|---|---|
| 1 | **PreemptRT 커널 + RT설정**(SCHED_FIFO·isolcpus·mlockall·RT CAN드라이버) | **f**(해결), **g**(가능케), **d**(부분) | 커널만으론 부족 — 설정 동반 필수 |
| 2 | **controller_manager rate ↑** (100→500~1kHz) | **g** | 1번(f) 선행해야 안정 |
| 3 | **양팔 단일 CM 통합 + 커플링을 RT update() 커스텀 컨트롤러로**(토픽 제거) | **e**(해결), **i**(개선), **g·h**(지원) | 큰 구조변경; DDS를 루프에서 제거 |
| 4 | **CAN-FD / 버스분할 / 드라이버 최적화** | **d** | HW·배선·드라이버 영역 |
| 5 | **마찰보상 완성도↑** (scale→1·Fv추가·재식별로 R²↑·vel_eps/필터로 떨림제거) | **a**(부분회복), **h**(보상 살) | 이미 쥔 카드, 빠름 |
| 6 | **관절별 게인 + 어깨 Kp↑** (대칭) | **h**(게인 살) | launch 인자, 빠름 |
| 7 | 관성/Coriolis FF 추가 | **a**(부분), **h** | 후순위(2019-20 데모도 없었음) |
| 8 | intra-process/shared-mem RMW(Iceoryx/Zenoh) or DDS 튜닝 | **e·i**(부분) | 3번의 약한 대안 |
| 9 | (범위밖) 액추에이터 교체/경량화 | **a, b** | HW 교체 불가 → 제외 |
| — | **c (힘센싱)** | 대응 불요 | 이미 MIT와 동등 |
MIT 근접 조합: **1+2+3**(대역폭·결정론·DDS제거) + **5+6**(투명도 회복·게인). 4는 d 한계 돌파용,
7은 후순위. a/b/c는 손 안 댐.

### PreemptRT 이전 시도 — PC N 팡팡 디버깅 (2026-06)
PC M(일반 우분투 커널) = bilateral 정상(팡팡 없음). PC N(PreemptRT 6.6.99-rt58, NUC11PHi7)
= **follower 팡팡(관절 튐)**, 동일 소스·빌드·파라미터.
- 격리 결과: **unilateral(기본)에서도 팡팡 / leader 정지 시엔 정상(움직일 때만) / 게인 낮춰도
  여전 / RMW(Zenoh·Cyclone 둘 다 N에서 팡팡, M은 둘 다 정상) → RMW 아님 / CAN 동일·깨끗
  (둘 다 PEAK PCAN-USB FD 1Mbps, 에러·드롭 0)**. → 유일 차이 = **PreemptRT 커널**.
- 결론(전제): **PreemptRT × USB-CAN(pcan_usb_fd) 상호작용**이 위치 샘플을 버스트/지터로 →
  follower가 튀는 명령을 추종 → 팡팡. (USB는 RT 비결정적; MIT는 USB 아닌 PCB→CAN 직결.)
- 대응: (1) USB autosuspend off, (2) xhci(USB) IRQ RT우선순위↑ + CPU격리, **(3) TODO:
  USB-CAN → PCIe/SPI-CAN 교체** (USB는 RT 결정론에 부적합; RT 이득 보려면 사실상 전제.
  MIT도 USB 안 씀). 당장 운용은 PC M(비-RT) 유지가 안전.

UPDATE(2026-06-28): 처방1(autosuspend) 무효(PCAN power/control 이미 on), 처방2(a)(xhci IRQ
FIFO90 + CM FIFO60) **0 효과**. 그리고 **PC M·N의 USB-CAN 설정 동일**(둘 다 PCAN-USB FD,
같은 config)인데 M만 정상 → **USB-CAN HW 탓 아님 확정.** 변수는 오직 **PreemptRT 커널이 같은
USB-CAN을 다르게 처리**(=RT 튜닝 미완). 이전 "USB-CAN 탓" 결론 정정.
- **방안 1 (TODO): PreemptRT 지연 튜닝.** ① `cyclictest -p90 -i200 -m -l100000` 로 Max latency
  측정(잘 튜닝 시 <100µs; 수백µs~ms면 미완=팡팡 원인). ② **깊은 C-state 억제**(NUC 1순위):
  런타임 `/dev/cpu_dma_latency`에 0 잡아두기 / 영구 GRUB `intel_idle.max_cstate=1
  processor.max_cstate=1`. ③ RT 스로틀 해제 `kernel.sched_rt_runtime_us=-1`. ④ net-RX softirq
  (ksoftirqd/napi) 우선순위↑. → 측정 후 ②부터 적용해 팡팡 재확인.
- **방안 2 (TODO): PC N의 누락된 사전세팅 점검** — openarmx 전 repo md 정독 결과:
  · **최유력: `openarmx-can_1.0.0_amd64.deb` (시스템 dpkg 설치, "driver of motors, must be
    installed before compilation").** `openarmx_can`은 소스 워크스페이스에 없음 → build/install
    복사해도 안 따라옴. PC N에 설치는 돼 있음(로봇 움직임)이나 **버전이 PC M과 같은지 확인 필요**
    (다르면 CAN 타이밍/로직 차이로 팡팡 가능). 점검: `dpkg -l|grep -i openarmx`, `lsb_release -a`,
    `echo $ROS_DISTRO` 를 PC M·N 비교. 다르면 동일 .deb 재설치.
  · 나머지(CAN 브링업 `ip link ... bitrate 1000000 up`)는 양쪽 동일 확인. openarmx 문서 전체에
    setcap/udev/sysctl/RT/latency 사전세팅 **없음**.

UPDATE(2026-06-30) — 팡팡 근본원인 = **PC N 기계(USB 토폴로지)**, RT 아님 (확정):
- 결정적: **PC M + 같은 PreemptRT 커널 → 팡팡 없음.** PC M(generic·RT 둘 다 정상) vs PC N(generic·RT
  둘 다 팡팡). → 팡팡은 RT 커널도 SW도 아니고 **PC N 하드웨어**. 방안1(RT 튜닝)은 헛다리였음.
- 유일한 HW 차이 = **USB 토폴로지**: PC M = root→7p허브→PCAN×6 (허브 1단) / PC N = root→**4p허브→7p허브**
  →PCAN×6 (**허브 2단 캐스케이드**). 펌웨어(fw v3.2.0)·드라이버(peak_usb)·속도(12M full-speed)·ID
  (0c72:0012)·can매핑 전부 동일. 6개 full-speed가 허브 TT 공유 + 단수 추가 → CAN 지터 → 팡팡(커널/RMW
  무관, 움직일 때만 표출과 일치).
- **TODO**: PC N 중간 4p허브 제거하고 7p PCAN허브를 NUC 포트 직결(PC M처럼) → 팡팡 사라지나 확인 = 토폴로지
  확정 테스트. (사라지면 토폴로지 확정 / 그대로면 가설 폐기.)
- **TODO**: 허브 **STT vs MTT** 확인 (`lsusb -v` 의 bDeviceProtocol; 01=Single TT, 02=Multi TT). PC M=MTT/
  PC N=STT 거나 단수 차이면 정확한 근거.

RT 효과 검증(별개) — "RT 깔면 transparency↑"는 **PC M에서 RT 깔아도 체감 동일** → 미확인. RT는 필요조건이지
충분조건 아님(대역폭↑·DDS제거·비USB CAN 동반 필요). **TODO**: generic vs RT 객관 비교 (PC M에서):
지표 = ①추종지연(상호상관 lag) ②동적추종오차 RMS ③오차-속도 기울기 ④대역폭+위상지연(chirp) ⑤루프 타이밍
지터(dt std/max) ⑥자유공간 경량성(eff·sign(vel)) ⑦명령 매끄러움(저크). RT 효과 기대처 = ⑤>①④.
방법 = **chirp_node**(follower 한 관절 사인 스윕 위치명령 + cmd/act/t 로깅) + 기존 **log_node**(동일 손동작).

### 결론 (격차 우선순위)
알고리즘은 같다. 격차는 ① **제어 대역폭/주기**(100·200Hz vs 500Hz+3kHz), ② **통신
지연·결정론성**(ROS2 DDS 다단 홉 vs RT 직결), ③ **HW 투명도**(Robstride 마찰 vs QDD).
2019-20 데모가 inertia/Coriolis FF 없이 완벽했으므로 → 우리도 그 방향(대역폭↑·지연↓·
마찰보상으로 투명도 회복·어깨 Kp↑)이 정답이고, 고급 모델보상은 후순위.
