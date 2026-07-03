# openarmx_bilateral — TECH DEBT (transparency / PreemptRT / USB-CAN)

> 목적: 나중에 이 debt를 해결할 시기에, 이 파일만 읽으면 **지금까지의 조사·결론·방향을
> 그대로 복원**할 수 있게 한다. (설계 논의 전반은 `FORCE_FEEDBACK_NOTES.md` 참조.)
> 상태: **HOLD (보류).** 운영은 PC M(일반 커널)에서 정상. 아래는 "더 좋게" 만들려다 막힌 것들.

## 0. 한 줄 요약
bilateral force-feedback의 **transparency(투명도)** 를 MIT Sangbae Kim 데모 수준으로
올리려 했으나 막힘. **[2026-07 실측 정정 §8c]** 예전엔 "USB-CAN이 89Hz로 캡"이라 봤으나,
candump 실측 결과 **CAN 루프는 154~173Hz**로 돌고 **`joint_states` 89Hz는 CM/broadcaster/DDS
소프트웨어 천장**임이 드러남. 두 축으로 분리: (1) **rate**는 소프트웨어(in-process CM/RT)로
하드웨어 없이 개선 가능(최우선·무료), (2) **팡팡/지터**는 USB 왕복 **tail(13~29ms 스톨)** 이
범인 → non-USB/HS-USB. PreemptRT 커널은 **transparency 개선 0(데이터 확인)**. CAN-FD는
modest(+23%, §8b).

## 1. 목표 (operator 기준)
MIT Kim 랩 3-DOF leader/follower 데모 느낌: follower가 벽에 막히면 leader도 거의 안 움직임
(딱딱한 벽), 평상시엔 깃털처럼 가볍고 빠름.

## 2. 제어 방식 (확정, 변경 불가 전제)
- 우리 = enactic = MIT 2022 = **위치-위치(P-P) 관절 임피던스 + proprioceptive(모터 전류) 토크.**
- MIT Eq.4 (SaLoutos/Stanger-Jones/Kim, IROS2022): `τ_f=Kp(q_l−q_f)+Kd(q̇_l−q̇_f)+τ_g ; τ_l=−τ_f`
  → 대수전개하면 우리 대칭 P-P와 **동일 골격.** "비법 수식" 없음.
- MIT 느낌의 출처 = 수식이 아니라 **(a) QDD 액추에이터 투명도(저마찰/저관성, 그래서 마찰보상
  조차 불필요) + (b) 고대역폭(500Hz 루프 + CAN 3kHz) + (c) 적절한 게인.**
- 우리는 (a) Robstride 마찰 → 마찰보상으로 일부 회복, (b)(c)가 부족. DOB/4채널/wave/energy-tank
  는 BANNED(실패작).

## 3. 격차 지도 (a~i) — `FORCE_FEEDBACK_NOTES.md`에 표 있음
a HW액추에이터, b HW DOF/질량 = **대응불가(HW)**. c 힘센싱 = MIT와 동등(대응불요).
h 제어법 = 골격 동일(마찰보상이 우리 추가분). **d 통신모터버스 / e 토폴로지(DDS홉) /
f 실시간성 / g 제어주기 / i SW스택** = 개선 여지. → 아래 조사가 d~g 실험.

## 4. PreemptRT / 팡팡 조사 (시간순 결론)
**증상**: PC N(PreemptRT 6.6.99-rt58, NUC11PHi7)에서 bilateral 시 follower **관절 팡팡**(튐).
PC M(일반 우분투 커널, openarmx-Default-string)에선 **정상**. 동일 소스/빌드/파라미터.

**체계적으로 배제한 것(전부 팡팡 원인 아님):**
- RMW: Zenoh·CycloneDDS·FastRTPS — PC N은 전부 팡팡, PC M은 전부 정상.
- C-state: cpu_dma_latency=0 무효 (cyclictest가 자동으로 0 설정하므로 이미 배제됨).
- SMI/펌웨어: `hwlatdetect` Max **19µs** → 정상.
- IRQ 우선순위: xhci 하드IRQ FIFO90 무효.
- softirq: ksoftirqd FIFO85 무효.
- controller_manager RT(FIFO60) 무효.
- CAN 설정: 양쪽 동일(PEAK PCAN-USB FD, 1Mbps, 에러/드롭 0).
- 사전세팅: `openarmx-can` .deb 버전·배포판·ROS 동일.
- cyclictest: avg **1µs**, max ~**900µs**(드문 스파이크, C-state 무관). 평균 스케줄 정상.

**결정적 전환점**: **PC M + 같은 PreemptRT 커널 → 팡팡 없음.** → 팡팡은 RT 커널/SW/RMW가
아니라 **PC N이라는 기계**. 일반 커널(6.8.0-124-generic)로도 PC N은 팡팡(RMW 3종 모두).

**유일한 HW 차이 = USB 토폴로지:**
- PC M: `root(480M)→7p허브→PCAN×6` (허브 **1단**).
- PC N: `root(480M)→4p허브→7p허브→PCAN×6` (허브 **2단 캐스케이드**).
- 펌웨어(fw v3.2.0)·드라이버(peak_usb)·속도(12M full-speed)·ID(0c72:0012)·can매핑 전부 동일.
- 6개 full-speed가 허브 TT 공유 + 단수 추가 → CAN 지터 → follower가 튀는 명령 추종 → 팡팡.
  (커널/RMW 무관, leader 움직일 때만 표출 — 전부 일치.)

## 5. RT의 transparency 효과 검증 (PC M, chirp, 정량) — 개선 0
chirp_node로 follower 한 관절 사인 스윕(0.2→3Hz) cmd/act 로깅. generic vs RT:
- 루프 dt: generic 평균11.73/std4.70/max25.18 ms ; RT 11.83/4.71/25.20 ms → **동일**
- 추종지연[ms] J1..7: generic 180,186,82,95,92,95,93 ; RT 175,190,84,94,83,95,94 → **동일**
- 대역폭 −3dB[Hz]: 어깨 ~1.9~2.1, 나머지 ~2.9 (양쪽 동일)
- RMS 추종오차: 양쪽 동일
→ **RT 커널 = transparency 개선 없음(데이터).** "RT 깔면 좋아진다"던 추정 반증.

## 6. 제어 rate 실험 (update_rate 100→500) — 무효, 원인 규명
- yaml 2개(`openarmx_v10_bimanual_controllers.yaml`, `..._namespaced.yaml`) 16행 `update_rate`
  를 100→500. bringup이 `/**: ros__parameters: update_rate` 를 controller_manager 노드에 전달
  (launch 108행 `control_node` parameters). **적용 확인**: `ros2 param get
  /follower/left_forward_position_controller update_rate` = 500.
- **그러나 실측 `ros2 topic hz /follower/joint_states` = 89Hz.** CAN 에러 0.
- → controller_manager가 **오버런**. 한 사이클 read(모터 전부)→update→write 가 **USB-CAN
  왕복으로 ~11ms** 걸려 500Hz(2ms) 불가, 최대 ~89Hz. (100Hz 목표 때도 89Hz였음.)
- chirp 지표(지연·대역폭·dt)도 rate500에서 무변 → 추종 대역폭은 **모터 onboard kp/kd 루프
  (~kHz)** 가 결정, host rate 무관.

## 7. 근본원인 (수렴)
**USB-CAN(PEAK PCAN-USB FD) 왕복지연이 공통 병목:**
- 제어루프를 **~89Hz로 캡** (per-cycle ~11ms). update_rate config로 못 넘음.
- PC N에선 **허브 캐스케이드**가 더해져 지터 → **팡팡**.
- transparency 한계의 큰 축(피드백 ~89Hz + dt std 4.7ms).
RT 커널은 무관(스케줄 avg 1µs로 충분). 즉 **하드웨어 통신 경로**가 문제.

## 8. 해결 로드맵 (debt 갚을 때 우선순위)
1. **[최우선] USB 왕복지연 제거** — (a) **비-USB CAN(PCIe/SPI-CAN)** 또는 (b) **HS-USB(480M)
   어댑터**(현물은 full-speed 12M — §8b). USB 왕복지연 제거 → 루프 rate↑ + PC N 팡팡 해소 +
   RT 이득 실현. MIT도 USB 안 씀(PCB→CAN 직결, CAN 3kHz). **CAN-FD는 여기 해당 안 됨(§8b 기각).**
   **이거 하나가 rate·팡팡·RT 세 개를 동시에 풀 1순위.**
2. **[싸고 빠른 확정 테스트] PC N 허브 직결**: 중간 4p허브 제거, 7p PCAN허브를 NUC 포트
   직결(PC M처럼 1단) → 팡팡 사라지나? = 토폴로지 확정.
3. **허브 STT vs MTT 확인**: `lsusb -v` `bDeviceProtocol`(01=Single,02=Multi). PC M=MTT/
   PC N=STT거나 단수 차이면 정확한 근거.
4. **RT 커널 제대로 활용** (지금 충분히 못 쓰는 중): SCHED_FIFO 우선순위 체계 + CPU 격리
   (isolcpus) + mlockall + threaded-IRQ 정렬 + **in-process RT 컨트롤러**(양팔 단일 CM,
   커플링을 update()에서 계산해 DDS 토픽 제거). 단 1·2·3 해결 후라야 의미.
5. **kp/kd 게인 튜닝**(추종 대역폭) — 마찰보상 완성도↑(scale→1·Fv추가·재식별·vel_eps로 떨림
   제거) 후 어깨 Kp↑. (h의 "살".)
6. **DDS/구조**: 양팔 단일 controller_manager + 커스텀 bilateral 컨트롤러(in-process), 또는
   intra-process/shared-mem RMW(Iceoryx/Zenoh). (e/i)

## 8b. CAN-FD 검토 결과 — 기각 (2026-07)
사양서·lsusb·제조사 답변으로 CAN-FD가 89Hz 캡을 깨는지 검토 → **기각**.
- **lsusb -t 확정:** 허브(Dev61)는 480M(HS)이지만 그 아래 **peak_usb 동글 6개는 각각 `12M`
  = full-speed** 로 enumerate. 즉 **동글 자체가 full-speed USB 장치**(허브가 HS여도 링크는 FS).
- 사양서의 "4/6채널 단일 480M HS 장치"는 **현물과 불일치**(현물=단채널 PCAN-USB FD ×6).
- 제조사: "baud rate 변경 가능, CAN-FD는 보통 5M."
- **그래도 캡 못 깸. 이유 2:**
  1. 프레임이 작음(MIT 8B). CAN-FD 이득은 큰 페이로드 데이터구간 → 8B는 arb(1Mbps) 지배,
     5M로 올려도 와이어 시간 거의 불변. (CAN 와이어는 11ms 중 ~2ms뿐.)
  2. 진짜 병목 = **full-speed USB 폴링(1ms 단위) × 채널당 8모터 순차 왕복 ≈ 8~11ms/사이클.**
     CAN-FD는 USB 트랜잭션 수·지연을 못 줄임.
- **결론:** CAN-FD는 레버 아님(11ms→~9.5ms marginal, 그나마 드라이버+모터 펌웨어 FD 필요).
  → 로드맵 1번(비-USB) 또는 **HS-USB(480M) 어댑터**가 정답.
- **[2026-07 실증 확인 — 이론 확정]** 실제로 실험함:
  1. `can_fd:=true` 브링업 → **모터 안 움직임**(토크는 걸리나 leader 밀어도 follower 무반응).
     = DM 모터 펌웨어가 classic 모드라 FD 프레임을 버림. FD 쓰려면 모터 16개 펌웨어 재설정 필요.
  2. 인터페이스만 FD(bitrate 1M/dbitrate 5M) + `can_fd` 미지정(classic 프레임) → candump
     **rate 완전 동일**(can0/1 154.5Hz, can2/3 175.6Hz — classic 1M 대비 변화 0).
     classic 프레임은 dbitrate 무시(nominal 1M). → **CAN-FD dead end, 재검토 불필요.**
  3. **[2026-07 제조사 확인 — 최종 종결]** 제조사 답변: **Robstride 모터는 CAN-FD unavailable.**
     모터가 애초에 FD를 지원 안 함 → 펌웨어 재설정 여지도 없음. **CAN-FD 라인 영구 종료.**
- **미완 확정:** `latency_capture.launch.py`(candump 타임스탬프)로 8모터 피드백이 ~1ms 순차면
  FS-USB 폴링 확정, <0.2ms 뭉치면 다른 원인. ← 최종 결정타, 아직 캡처 안 함.

## 8c. latency_capture 실측 결과 (2026-07) — 89Hz 진단 정정
`latency_capture.launch.py` candump 29s / 269k 프레임 분석. **기존 "USB-CAN이 89Hz로 캡"
서술을 정정한다.**
- **CAN 루프 = 154~173Hz (채널당 6ms), NOT 89Hz.** can0/1(leader)=154Hz, can2/3(follower)
  =173Hz. 주기 p50 5.8~6.5ms, p95 8~11ms, **max 14~16ms.**
- → **`ros2 topic hz /joint_states`=89Hz는 CAN이 아니라 controller_manager/broadcaster/DDS
  (소프트웨어) 천장.** CAN은 154Hz 주는데 ROS 레이어가 89으로 깎음.
- **프레임당 0.44ms = CAN 와이어 ~0.13ms + USB/드라이버 오버헤드 ~0.31ms.** 사이클 ~15프레임.
  USB 오버헤드가 사이클의 2/3(~4.6ms), CAN 와이어 1/3(~2ms).
- **요청(0200FDxx)→피드백(028xxxFD) 왕복 median 0.2ms (빠름).** 그러나 꼬리 p90 6ms,
  **max 13~29ms 스톨** → **이 tail이 팡팡·지터·transparency 저하의 실제 범인**(평균 아님).
- **함의 (우선순위 재정렬):**
  1. **[신규 최우선·무료] 소프트웨어 rate 천장 제거** — CAN이 154Hz 주는데 ROS 89Hz.
     in-process 단일-CM bilateral 컨트롤러 + update_rate + RT로 **하드웨어 없이 89→~150Hz 가능성.**
     CAN-FD·PCIe보다 ROI 높음. (검증: CM update_rate/오버런 카운터 확인.)
  2. **USB tail(13~29ms 스톨)** = smoothness/팡팡 → non-USB/HS-USB 정당화(단 평균 rate 아님).
  3. **CAN-FD = modest(+23%, 6→5.3ms), 변혁적 아님**(§8b). 바닥은 USB 오버헤드 4.6ms.
- 프레임 스킴: `0200FDxx`=모터xx 상태요청(77Hz), `028xxxFD`=피드백(154Hz), `017E../0180..`
  =MIT 토크명령(write).

## 8d. rate_probe 실측 — 89Hz 근본원인 = 드라이버 소프트웨어 (하드웨어 아님!)
`rate_probe.launch.py` + candump 프레임분류로 89Hz를 **완전 근본원인**까지 규명. **"PCIe 필요"
가정을 뒤집음 — 89Hz는 CAN 대역이 아니라 openarmx_hardware 드라이버의 중복 왕복 + 1ms sleep.**

**rate_probe 실측 (update_rate=100 설정):**
- **명령(write) 경로 = 200Hz, 지터 std 0.05ms (완벽).** relay_node·forward controllers 결백.
- **상태(read) 경로 = joint_state_broadcaster(=CM update 루프):**
  - leader CM **75Hz**, follower CM **86Hz** → **설정 100 미달 = 오버런.**
  - 지터 **std ~5ms, max ~25ms** → follower로 전파되어 팡팡의 SW측 원인.
- update_rate 올려도 무의미(이미 오버런).

**candump 프레임분류 (can0, 모터당):** STATUS_REQ `0200FDxx` 71Hz + MIT_CMD `017/018` 71Hz →
FEEDBACK `028xxxFD` 142Hz. **host→motor 프레임마다 피드백 1개** ⇒ MIT 명령이 이미 위치/속도/
토크 피드백을 돌려줌 ⇒ **read()의 별도 상태요청은 중복.**

**근본원인 (v10_simple_hardware.cpp) — [2026-07 정정]:** CM update 1사이클 = CAN 왕복 **2회**.
(이전 초안의 "write() 1ms sleep"은 오독 — 그 sleep_for는 shutdown용 `return_to_zero()`에 있고
per-cycle 아님. recv_all(int first_timeout_us)는 첫 프레임을 최대 timeout까지 select 대기 후
나머지는 0us 드레인.)
- `read()` (473행): `refresh_all()`[상태요청 0200FDxx 전송] + `recv_all(500us)` = 왕복①
- `write()` (566행): `send_motion_control_commands()`[MIT] + `recv_all(1000us)` = 왕복②
- MIT 명령이 이미 피드백을 돌려주므로 read()의 refresh 왕복은 **중복** → 합 ≈ 11~13ms > 10ms → 오버런.

**레버 (upstream openarmx_hardware 수정, 제어PC서 실험 필요) — LATENCY_LEVER_DRIVER_PATCH.md 참조:**
1. **[적용함, 실험 대기] 파이프라인화: 왕복 2→1.** read()=recv_all()만(직전 write의 MIT 피드백
   드레인, refresh_all 주석), write()=전송만(recv_all 주석). 사이클당 전송 절반 + recv 1회 →
   **CM ~2배(75→~150Hz) 기대.** 위험 중(상태 1사이클 stale). ← **최대 ROI, 현재 여기.**
2. (후속) read/write 별도 스레드 비동기화(CAN free-run+최신값 캐시) → CM을 CAN서 분리 → 200Hz+. 위험 높음.
→ **로드맵 재정렬: 이 SW 레버(무료)가 비-USB CAN 하드웨어보다 우선.** 팡팡/tail은 여전히 USB(§8c)지만
  rate·오버런은 여기서 대부분 해결 가능.

**테스트 프로토콜 (A/B, rate_probe):** ① 레버 전 baseline rate_probe(=leader75/follower86 확인) →
② 드라이버 패치+빌드 후 **먼저 teleop 동작·부드러움 확인**(stale 1사이클 이상 없나) → ③ rate_probe
재측정, joint_states rate가 75/86 → ~130-150 오르나 + 지터(std) 확인. 이상하면 2줄 주석해제로 즉시 롤백.

## 9. 자산 (도구·명령·위치)
- 측정 도구: `chirp_node`(반복 사인스윕+cmd/act/t 로깅, 전관절 순차), `log_node`(teleop
  cmd-vs-act), `friction_log_node`(effort 분해), `friction_id_node`(마찰 식별).
- RT 진단: `cyclictest -p90 -i200 -m -l100000`(스케줄 지연), `hwlatdetect --duration=30`(SMI),
  `candump -t d can0`(CAN 프레임 간격), `ip -s link show canX`(CAN 에러), `lsusb -t`(토폴로지).
- update_rate 위치: 위 yaml 2개 16행 (지금 100으로 되돌려둘 것 — 500은 무의미 오버런).
- PC M = 일반 커널, bilateral 정상(운영용). PC N = PreemptRT/NUC, 팡팡(디버그용).
- 분석 스크립트 패턴: chirp CSV(t,joint,freq,cmd,act,vel,eff) → 대역폭=FFT(act)/FFT(cmd) 또는
  freq-bin 게인 −3dB, 지연=cmd·act 상호상관, 지터=dt std/max.

## 10. 재개 시 첫 행동
(1) 이 파일 + `FORCE_FEEDBACK_NOTES.md` 읽기. (2) 해결 로드맵 1번(비-USB CAN) 가능 여부부터
판단(하드웨어 조달). 그 전 무료 확정은 로드맵 2번(PC N 허브 직결 테스트). (3) 비-USB CAN 확보
후에야 rate↑·in-process RT·게인 튜닝이 의미. 그 전엔 transparency는 USB-CAN ~89Hz에 묶임.

## 11. 기타 TODO
- **축별(per-joint) gravity scale 필요.** 현재 상황: openarmx `gravity_comp_node`는 **전역 스칼라
  `g_scale` 하나만** 지원(`tau = g_scale * tau_g[j]`, 모든 관절 동일). 관절별 배열 파라미터 없음.
  현재 전역 `g_scale=0.95`로 J1/J2/J4 붕 뜸을 트림 중이나, 관절마다 필요량이 달라 전역값은 절충.
  → **나중에 축별 gravity scale 가능하게 해야 함.** 구현안 (A, 권장): `friction_comp_node`에
  `grav_scale[7]`(기본 1.0) 파라미터 추가해 `out[j]=grav_scale[j]*grav[j]+friction[j]` (openarmx
  미수정). (B): openarmx_gravity_comp.cpp에 `scale_joints` 추가(패키지 수정). oa_fd_cpp엔 이미
  `scale_joints`[7] 존재(참고).
- **posture spring (#2, oa_fd_cpp 컨셉) 보류 중** — J3/J5 영점 복원. friction_comp에 per-joint
  posture FF 추가 예정. PC측 spring은 CAN 지연 chatter 위험(oa_fd 교훈) → 약한 게인+deadband.

- **[TODO] 그리퍼(joint8) 조종단 뻑뻑함 미해결.** (2026-07)
  - **완료된 것 (파이프라인 정상):** joint8 friction comp 배관 전부 연결됨 — friction_comp_node
    8관절 출력(`gripper_joint` param=finger_joint1, 출력 `max(nj,grav)`), effort yaml에 finger 추가,
    launch `grip_fc`/`grip_k` 인자화. **드라이버 2줄 패치 필수**(`GRIPPER_FRICTION_DRIVER_PATCH.md`):
    read() 그리퍼 속도(하드코딩 0 → `get_velocity()`), write() 그리퍼 토크(0 → `tau_commands_[ARM_DOF]`).
  - **실측:** effort = fric_scale(0.7)·grip_fc → 부호 정상(조일때−/풀때+), 근데 grip_fc=0.10이면 max
    ±0.07Nm로 **너무 약해 뻑뻑함 안 풀림.** grip_fc↑ 스윕 미완(0.5/1.0 등 시도 예정).
  - **미검증 가설 (다음에 볼 것):** 뻑뻑함이 (a) 기계 마찰인지 (b) **bilateral 위치 커플링 스프링**인지
    분리 안 됨. leader 그리퍼 kp=0.3(relay gain_loop) + **follower 그리퍼 kp**(HW기본, 값 미확인)로
    당겨져 뻑뻑할 수 있음. → **follower 그리퍼 kp가 0이 아니면 낮춰보기**(단 반력↓ 트레이드오프).
    먼저 확인: `ros2 param get`으로 follower `/…hardware_params` 의 `kp_joint8` 현재값. 0이면 이 가설 기각.
  - 그 외: grip_fc 과다→과보상 limit cycle(손 떼도 스르륵); grip_k↑는 저속 포화 개선. 정밀화하려면
    그리퍼 전용 friction_id 필요(현재 Fc/K는 대충값).
