# Latency / transparency investigation — 실험 로그 & 결론 (2026-07)

bilateral teleop의 **transparency**(leader 입력 대비 follower 추종속도·반력 충실도)를 올리기 위한
지연/rate 조사 전 과정. **한 줄 결론: "89Hz = USB 하드웨어 병목, PCIe 필요"는 틀렸고, 진짜 원인은
드라이버가 CM 사이클마다 돌던 중복 CAN 왕복이었다. 드라이버 2줄로 하드웨어 없이 rate 2배 + 지터 12배
개선.** 상세 부채/맥락은 `tech_debt.md`, 이 파일은 실험별 결과·배움 위주.

관련 도구(이 조사에서 만든 것): `rate_probe_node`+`rate_probe.launch.py`(ROS rate/지터, 무인자),
`latency_capture.launch.py`(candump 타임스탬프, 무인자). 드라이버 패치: `LATENCY_LEVER_DRIVER_PATCH.md`.

---

## 실험 순서 · 결과 · 배운 것

### E1. 하드웨어 식별 (lsusb / lsusb -t)
- **결과:** 꽂힌 것 = **6× PEAK PCAN-USB FD (0c72:0012)** on **QinHeng USB2.0 허브**, 각 동글이
  **12M full-speed**로 enumerate (허브는 480M HS지만 동글이 FS). 사양서의 "4/6채널 단일 480M HS
  장치"와 **불일치**(그건 다른/후보 장치).
- **배움:** 현재 CAN 경로는 **full-speed USB**. 이게 이후 모든 상한의 근원.

### E2. CAN-FD 시도 → 사장(dead end)
- `can_fd:=true` 브링업 → **모터 안 움직임**(토크는 걸리나 무반응) = DM/Robstride 모터 펌웨어가
  classic 모드라 FD 프레임 거부.
- 인터페이스만 FD(`dbitrate 5M`) + classic 프레임 → candump rate **완전 동일**(154/175Hz). classic
  프레임은 dbitrate 무시(nominal 1M).
- **제조사 확인: Robstride 모터 CAN-FD unavailable.** → 라인 영구 종료.
- **배움:** CAN-FD는 이 하드웨어에서 불가. 됐어도 8B 소형 프레임+USB 오버헤드라 marginal.

### E3. candump baseline (패치 전, classic)
- **CAN 루프 = 154Hz(leader can0/1) / 173Hz(follower can2/3)** — **89Hz 아님.**
- 프레임 종류(can0, /모터): STATUS_REQ `0200FDxx` 71Hz + MIT_CMD `017/018` 71Hz →
  FEEDBACK `028xxxFD` 142Hz. **host→motor 프레임마다 피드백 1개.**
- 프레임 간격 0.44ms = CAN 와이어 ~0.13 + USB/드라이버 오버헤드 ~0.31. 왕복 median 0.2ms,
  tail max 13~29ms.
- **배움:** CAN 하드웨어는 154~173Hz 능력. **89Hz는 CAN이 아니라 상위 소프트웨어 천장**이다.

### E4. rate_probe (ROS 레이어 계측)
- **명령(write) 경로**(relay→forward controllers): **200Hz, 지터 std 0.05ms** — 완벽, 결백.
- **상태(read) 경로**(joint_state_broadcaster = CM update 루프): **leader 75Hz / follower 86Hz**,
  지터 std ~5ms, max ~25ms. 설정 update_rate(제어PC=500) **미달 = 오버런.**
- **배움:** 89Hz(75/86)은 **controller_manager update 루프 = 소프트웨어 천장.** relay는 무죄.

### E5. 드라이버 소스 근본원인 (v10_simple_hardware.cpp)
- CM update 1사이클 = CAN **blocking 왕복 2회**:
  - `read()`: `refresh_all()`[상태요청 전송] + `recv_all(500)` = 왕복①
  - `write()`: `send_motion_control_commands()`[MIT] + `recv_all(1000)` = 왕복②
- `recv_all(first_timeout_us)` = 첫 프레임을 최대 timeout까지 select 대기 후 나머지 0us 드레인.
- **MIT 명령이 이미 위치/속도/토크 피드백을 돌려주므로 read()의 별도 상태요청은 중복.**
- (초안의 "write() 1ms sleep"은 오독 — 그 sleep_for는 shutdown용 `return_to_zero()`.)
- **배움:** 사이클당 중복 왕복 2회 × 8모터 blocking = ~11~13ms > 목표 → 오버런. 왕복 하나 제거 가능.

### E6. 파이프라인 패치 (드라이버 2줄)
- `read()`: `refresh_all()` 주석 (recv_all은 유지 → **직전 write()의 MIT 피드백을 드레인**).
- `write()`: `recv_all(1000)` 주석 (피드백은 다음 read()에서 드레인).
- → 사이클당 전송 절반(상태요청 제거) + recv 1회. 상태는 1사이클 stale(150Hz면 ~6ms).

### E7. 패치 후 rate_probe (update_rate=500) — **착시**
- joint_states **500Hz**, 지터 std 0.4ms, max 3.5ms 로 보임 → 처음엔 "6배 성공"이라 판단.
- **배움(다음 단계서 뒤집힘):** joint_states rate는 **broadcaster** 출력일 뿐, 실제 모터 상태 신선도가
  아니다. candump로 검증 필수.

### E8. 패치 후 candump 모터별 — **500은 착시였다**
- update_rate 500, can0 모터별 피드백: **500,459,215,32,4,2,3,6 Hz** — **모터 1-2만 빠르고
  3-8은 굶음**(before는 균일 154→124). STATUS_REQ=0(refresh 제거 확인).
- **원인:** refresh_all(균일 라운드로빈 폴링) 제거 + update_rate 폭주 → write()가 8모터에 4000cmd/s
  쏘는데 **full-speed USB는 채널당 ~1220 프레임/s 한계** → CAN TX 큐 오버플로우로 **send 루프
  뒤쪽 모터부터 드롭.**
- **배움:** 파이프라인化는 옳지만 update_rate를 USB 상한에 맞춰야 함. rate는 broadcaster 착시 주의.

### E9. update_rate 스윕 (150~250) + 모터별 candump — 상한 확정
can0 모터별 피드백(Hz):
| update_rate | J1–5 | J6 | **J7 손목** | **J8 그리퍼** | 총합(USB) | joint_states/지터 |
|---|---|---|---|---|---|---|
| **150** | 150 | 150 | **150** | **150** | 1200 | 150Hz / std0.41 max8.0 |
| 160 | 160 | 160 | 156 | 106 | 1222 | 160Hz |
| 170 | 170 | 169 | 143 | 61 | 1224 | 170Hz |
| 180 | 180 | 173 | 118 | 30 | 1228 | 180Hz |
| 190 | 190 | 170 | 87 | 17 | 1223 | 190Hz |
| 200 | 200 | 158 | 59 | 8 | 1222 | 200Hz / std0.42 max6.5 |
| 250 | 250 | (43) | 3 | 3 | ~1220 | 250Hz / std0.43 max5.6 |
| (원본) | — | — | — | — | — | 75/86Hz / std5 max25 |
- **USB 하드 상한 ≈ 1220 프레임/s/채널** (모든 rate에서 총합 동일). 초과분은 뒤쪽 모터(J8→J7)가 흡수.
- **완전균일 상한 = 1220 ÷ 8 ≈ 152Hz.** 150=전부 균일. 160=팔 7관절 full(J7≥156) 그리퍼만 106.
  170=경계(J7 143). 180↑=손목 확실히 무너짐. 200=손목 59(나쁨). 250=붕괴.
- joint_states는 설정 update_rate를 그대로 따라감(broadcaster) — **모터별 candump로만 진짜 신선도 확인.**

---

## 배운 것 총정리 (핵심)
1. **"89Hz = USB 하드웨어 병목, PCIe 필요" 가설 기각.** 진짜 원인 = 드라이버 중복 CAN 왕복(blocking).
2. **89Hz는 CM/broadcaster 소프트웨어 천장.** CAN 자체는 154~173Hz. relay/명령경로는 200Hz 결백.
3. **read()의 상태요청(refresh_all)은 중복** — MIT 명령이 이미 피드백을 돌려줌. 이게 CAN 부하를 2배로.
4. **파이프라인化(왕복 2→1)로 CM이 설정 update_rate에 도달.** 하드웨어 불필요.
5. **하지만 full-speed USB에 하드 상한(~1220프레임/s/채널 = 8모터 균일 ~152Hz)이 있다.**
6. **joint_states rate는 broadcaster 착시** — 150 초과 설정 시 send 순서상 **뒤쪽 모터(손목 J7·그리퍼
   J8)가 굶는다.** "500Hz/200Hz"는 앞 모터만.
7. **팡팡/지터의 범인은 USB tail이 아니라 (a)CM blocking (b)update_rate>상한 시 손목 starve.** (§8c 정정)
8. **CAN-FD는 Robstride 미지원으로 불가**(제조사 확인).
9. **순이득: 75/86 → 150Hz 균일(2배) + 지터 std 5→0.42ms(12배), max 25→6.5ms** — 드라이버 2줄, 무료.
10. **더 높은 균일 rate는 HS-USB(480M) 필요**(진짜 A-1). PCIe는 필수 아님. 스레드 분리 레버는 USB가 이미
    상한이라 **불필요.**

---

## 병목 분석 — 왜 150이 상한인가 (셋 중 무엇?)
채널당 측정 상한 ≈ **2440 프레임/s** (프레임당 ~0.41ms = CAN 와이어 ~0.13 + **USB 전송 오버헤드 ~0.28**).
| 후보 | 8모터 균일 이론상한 | 현재 부하 | 병목 |
|---|---|---|---|
| CAN 버스 (1Mbps) | ~475Hz | **31%뿐**(2400프레임/s×0.131ms) | ❌ 여유 3배 |
| **USB2CAN (full-speed USB)** | **~150Hz** | **~98% 포화** | ✅ **이것** |
| 모터 (Robstride 내부루프 kHz) | kHz급 | 왕복 median 0.2ms, 즉시 응답 | ❌ |
- **병목 = PCAN-USB FD 동글의 full-speed USB 전송 오버헤드**(프레임당 ~0.28ms). CAN 버스·모터는 여유.
- 150↑ 균일하려면 병목만 교체: **HS-USB(480M) USB2CAN**(→CAN버스 상한 ~475Hz 근접, 현실적 A-1) 또는
  **PCIe/SPI CAN**. CAN-FD는 Robstride 미지원 불가(E2).

## 채택 설정 & 남은 결정
- **드라이버 파이프라인 패치 적용 (E6, `LATENCY_LEVER_DRIVER_PATCH.md`)** — 확정.
- **update_rate: 150 확정** (안전율 — 완전균일 상한 ~152 바로 아래, 8모터 전부 150Hz). yaml 2개 16행.
- **다음 검증:** 벽 느낌(transparency) 체감 / PC N 팡팡 재검증(원인이 USB tail 아니었으니 이 패치로 될 수도).
- **파킹:** 그리퍼 마찰보상(tech_debt §11), **HS-USB 조달(A-1)** — 150 이상 균일 rate 원할 때 유일한 길.

---

## E11. [2026-07] openarmx_teleop(공유메모리 포팅판)에서 150Hz 상한 **런타임 스윕으로 재확정**
새 포팅판(openarmx_teleop)에 **제어주기 런타임 인자**(`launch_bilateral.sh <side> <lcan> <fcan> <Hz>`)
+ **주기/스텝시간 로깅**([Leader/Follower ctrl] Hz·jitter·step_us)을 넣고 rate를 스윕:
| rate | 결과(운영자 체감) | 해석 |
|---|---|---|
| 100Hz | 150보다 **더 뻑뻑** | rate↓ = 지연↑ = 투명도↓ |
| **150Hz** | **최적(쓸만)** | 8모터 균일 상한 |
| 200Hz | **후방 관절(J5,6,7,8) 추종 느림** | send 루프 뒤쪽 모터 starvation 시작 |
| 250Hz | **진동 + 팡** | starvation 심화 → 발산 |
- **결론: full-speed USB2CAN의 8모터 균일 상한 = ~150Hz 재확정.** 위로 올리면 send 순서상 뒤쪽 모터
  (손목·그리퍼)가 굶고(§E8/E9와 동일 메커니즘), 아래로 내리면 뻑뻑. **150이 이 하드웨어의 스윗스팟.**
- 로그 기준(150Hz): period mean 6667us(=150Hz), min 6079 / max 7100, **jitter ~1.16ms.** 옛 ros2_control
  (75/86Hz, jitter max 25ms) 대비 rate 2배·최대지연 ~3.5배 개선 → **이 저지터가 kp120 안정의 원인.**
- **150 초과는 HW 교체만 가능**(HS-USB / PCIe-CAN / SPI-CAN). 옵션 상세 = 아래 §E12.

## E12. 150Hz 초과 옵션 (조사 2026-07)
CAN 1Mbps에서 8모터×1왕복 = 이론 **~475Hz**가 CAN 버스 상한(그 이상은 CAN-FD/고비트레이트 필요, 단
Robstride는 1Mbps classic 고정). 현재 150은 **full-speed USB의 프레임당 오버헤드**가 병목. 넘는 법:
1. **HS-USB(480M) USB-CAN 어댑터** — full-speed(1ms 프레임)→Hi-Speed(125µs 마이크로프레임) ⇒ USB
   트랜잭션/s ~8배 ⇒ CAN버스 상한(~475Hz)에 근접. **가장 저비용·최소침습**(어댑터만 교체, socketCAN 그대로).
   단 "진짜 HS(480M) 명시" 제품이어야(현물 PCAN-USB FD는 FS로 동작). ⚠ 조달 시 lsusb -t로 480M 확인.
2. **PCIe-CAN 카드** — CAN 컨트롤러가 PCIe 직결(USB 계층 제거). 전용 대역·초저지연·저지터. CAN버스
   상한(~475Hz)까지, 지터는 USB보다 훨씬↓. 단 **데스크톱 PCIe 슬롯 필요**(NUC/노트북 불가), 통합 비용↑.
3. **SPI-CAN (MCP2515/2518FD를 SBC SPI에 직결)** — USB 없이 SPI→CAN. **MIT Kim랩 플랫폼이 바로 이 방식**
   (SPI 500Hz→커스텀PCB→CAN 최대 3kHz, arxiv 2208.04487). 최저지연·최고 상한이나 **커스텀 HW/배선 +
   SBC(Jetson/Pi)** 필요 = 작업량 최대.
4. (부차) **동글을 여러 USB 호스트 컨트롤러로 분산** — 허브 TT 경합↓로 지터 약간↓뿐, per-dongle
   full-speed 상한은 그대로 → 근본 해결 아님.
- **핵심 근거(논문/자료):** USB는 시간임계 폐루프에 지연 유발 → 온보드/직결 컨트롤러 권장; QDD 고속제어
  (1kHz+)는 USB 아닌 SPI/PCIe/FPGA-DMA로 구현됨. QDD는 1:10 저감속·고backdrivability·**저마찰**(→ §Fv/Fo와 연결).
- **권장 순서: HS-USB(가성비) → PCIe(데스크톱이면) → SPI-CAN(끝까지 갈 때).**
