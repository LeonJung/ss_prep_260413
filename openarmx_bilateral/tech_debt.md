# openarmx_bilateral — TECH DEBT (transparency / PreemptRT / USB-CAN)

> 목적: 나중에 이 debt를 해결할 시기에, 이 파일만 읽으면 **지금까지의 조사·결론·방향을
> 그대로 복원**할 수 있게 한다. (설계 논의 전반은 `FORCE_FEEDBACK_NOTES.md` 참조.)
> 상태: **HOLD (보류).** 운영은 PC M(일반 커널)에서 정상. 아래는 "더 좋게" 만들려다 막힌 것들.

## 0. 한 줄 요약
bilateral force-feedback의 **transparency(투명도)** 를 MIT Sangbae Kim 데모 수준으로
올리려 했으나, **공통 병목 = USB-CAN(PEAK PCAN-USB FD) 왕복지연**으로 막힘. 제어루프가
**~89Hz로 캡**되고, PC N(NUC)에선 USB 허브 캐스케이드로 **관절 팡팡**. PreemptRT 커널은
**transparency 개선 0(데이터 확인)**. 근본 해결 = **비-USB CAN(PCIe/SPI)** 으로 추정.

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
- **미완 확정:** `latency_capture.launch.py`(candump 타임스탬프)로 8모터 피드백이 ~1ms 순차면
  FS-USB 폴링 확정, <0.2ms 뭉치면 다른 원인. ← 최종 결정타, 아직 캡처 안 함.

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
