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

## E13. 구체 제품 후보 + 리서치 주의점 (2026-07)
### ⚠ 리서치로 드러난 핵심 주의점 (제품 고르기 전 필독)
- **gain은 rate지 jitter가 아님.** 리서치상 **Linux CAN 지연 바닥 ~0.1ms, max ~1ms(in-kernel)/~3ms(user-space RT).**
  우리 현재 jitter 1.16ms는 **이미 이 바닥 근처.** 즉 하드웨어 바꿔도 **지터는 크게 안 줄 수 있음** — 얻는 건
  주로 **처리량(rate) = 150Hz→더 높이**(full-speed→HS/PCIe로 프레임/s↑). CAN 1Mbps 상한 ~475Hz.
- **싼 slcan(serial-line CAN)은 오히려 느림.** Waveshare USB-CAN-A(STM32, COM포트/slcan) 류는 고속제어에
  부적합할 수 있음 → **네이티브 SocketCAN 커널드라이버 + Hi-Speed** 제품이라야 이득.
- **SPI-CAN(MCP2518FD)도 지터 주의.** Jetson에서 **~6ms TX 지터 보고** 사례 → 자동으로 더 좋은 게 아님.
- **결론: 확실히 빠른 건 PCIe/mini-PCIe(Kvaser 등, 1µs 타임스탬프) 또는 검증된 Hi-Speed USB(Kvaser). 후보는
  반드시 우리 로깅([Leader ctrl] Hz)으로 실측 검증 후 채택.**

### 제품 티어
| 티어 | 제품 | 폼팩터 | 특징 | 대략가 | 유통 |
|---|---|---|---|---|---|
| **A 최고성능** | **Kvaser Mini PCIe 2xHS v2 / PCIEcan HS v2** | (mini)PCIe | 1µs 타임스탬프, 초저지연, 네이티브 SocketCAN | ~$300-500 | Kvaser 대리점, phytools |
| A | **Cervoz MEC-CAN-2F02i** | **M.2 PCIe** | NUC M.2 슬롯 적합, 2ch CAN-FD, SocketCAN | 산업용 | Cervoz 대리점 |
| A(저가) | 제네릭 **Mini-PCIe 2-CH CAN** | mini-PCIe | 1Mbps, SocketCAN, 격리 | ~$50-100 | alibaba, robotshop, thepihut |
| **B 검증USB** | **Kvaser USBcan Pro 2xHS v2 / Leaf Light v2 / USBcan Light 4xHS** | USB(HS) | Hi-Speed, 네이티브 SocketCAN 드라이버, 저지연 | ~$150-600 | Kvaser 대리점 |
| B(저가 실험) | **Innomaker USB2CAN-X2** | USB | STM32F0+DMA, SocketCAN, C/Py 데모 | ~$40 | alibaba, inno-maker.com, 아마존 |
| ~~C 비추~~ | ~~Waveshare USB-CAN-A~~ | USB | slcan/COM포트, 고속제어 부적합 | ~$20 | devicemart/eleparts, aliexpress |
| C(SBC) | **Waveshare 2-CH CAN FD HAT (MCP2518FD)** | SPI(Jetson/Pi) | MIT식 SPI-CAN, 단 **지터 6ms 사례** — 신중 | ~$20-30 | devicemart/eleparts, waveshare |

### 한국 사이트 메모
- **devicemart / eleparts**: Waveshare(USB-CAN-A/B/FD, CAN HAT) 다수 취급. Kvaser/PEAK는 산업용 대리점(예:
  캔인터페이스 전문 유통사) 통해. → "CAN" "SocketCAN" "PCIe CAN" 키워드 검색.
- **coupang**: 소비자용/Waveshare 일부. **alibaba**: Innomaker·제네릭 mini-PCIe/USB-CAN 다양(가성비 실험용).
- **⚠ 우리 폼팩터 확인 필요**: 제어 PC가 데스크톱(PCIe 슬롯 有)인지 NUC(M.2/mini-PCIe만)인지에 따라 A티어 선택.
  NUC면 M.2 CAN(Cervoz) 또는 검증 Hi-Speed USB(Kvaser). 데스크톱이면 PCIe(Kvaser) 최선.
- **실측 필수**: 어떤 걸 사든 우리 rate 로깅으로 150Hz 초과·지터 확인 후 확정.

### E13a. 제어 PC = SK-M03 폼팩터 확정 (2026-07) → **Hi-Speed USB 확정**
- 제어 PC = **SK-M03**(深圳市航柏科技/Shenzhen Hangbai 미니PC). 검색으로 스펙 안 나옴(저가 심천 벤더).
- 실측 확인:
  - `product_name = Default string` (BIOS 미기입 저가 보드).
  - USB = **Intel Comet Lake PCH-LP xHCI 1개**(모바일 칩셋, 컨트롤러 단일) → 동글 분산 무의미.
  - 저장장치 `TRAN=sata`, **NVMe 없음** (M.2 M-key 비었을 수도 있으나 2.5"/M.2 SATA 구분 불가).
  - `dmidecode -t slot` = 5개 전부 `In Use`/`Opening is shared`, Bus Addr `00:01.0`(CPU PEG)+`00:1c.3~6`(PCH 루트포트).
    → **물리 카드 슬롯 아님, BIOS 보일러플레이트.** 미니PC라 실제 꽂을 PCIe/mini-PCIe 슬롯 없음.
- **결론: SK-M03엔 물리 확장슬롯 없음 → 데스크톱/mini-PCIe Kvaser 불가 → `Kvaser USBcan Pro 2xHS v2`(B티어) 확정.**
  8모터를 2ch당 4모터 또는 2~3대 분산. M.2 CAN(Cervoz)은 빈 M.2 확인+내부배선/발열 감수 시에만 차선.
  slcan(Waveshare) 금지. 설치 후 `[Leader ctrl] Hz` 로깅으로 150 초과 실측 후 확정.

### E13b. PEAK PCAN-USB Pro FD 검토 → **rate 목적이면 부적합** (2026-07)
- 질문: 현 PCAN-USB FD(`0c72:0012`) 대신 PEAK **PCAN-USB Pro FD**(IPEH-004061)로 상한 돌파 되나?
- **결론: 거의 못 깬다.** 근거:
  - PEAK 포럼: PCAN-USB FD·Pro FD **둘 다 1Mbit에서 ~8000 msg/s** → 어댑터 CAN 처리량은 병목 아님
    (우리 상한 ~1220 fr/s/ch는 그 1/6). 진짜 벽 = **USB 동기 왕복 지연(~0.8ms/트랜잭션)**.
  - Pro FD도 **같은 PEAK USB 아키텍처** → 트랜잭션 지연 동일 → 채널당 상한 그대로. 이점은 2ch/box(통합)+
    CAN-FD뿐인데 **FD는 Robstride 미지원(무용)**, 채널 수는 이미 부족치 않음(6 어댑터).
- ⚠ **"IPEH-004061 호환 가능"으로 파는 제품 = 정품 아닌 클론.** `peak_usb` 드라이버 미바인딩·지연 미검증·
  신뢰성 불명 리스크 → **금지**(slcan과 동급 경고).
- **보정된 관점**: USB2.0인 이상 어떤 어댑터든 microframe(~0.125ms)+펌웨어 지연 바닥은 못 벗어남.
  Kvaser도 "확실한 해결"이 아니라 "다른 USB 스택이라 지연 좀 낮을 수 있는 베팅"(150→잘해야 200~250, 실측 필수).
  **확실한 상한 돌파 = PCIe(SK-M03 불가) 또는 소프트웨어 파이프라이닝(send/recv 비동기).** kp/kd가 여전히 최대 실효 레버.

### E13c. IXXAT(HMS) USB-to-CAN V2 검토 → **비권장** (2026-07)
- 질문: IXXAT USB-to-CAN V2로 갈아타면?
- **결론: 우리 케이스엔 이득 없음. 두 이유:**
  1. **rate 이득 없음** — 같은 USB2.0 어댑터 → 동기 왕복지연(~0.8ms) 벽 동일, 현 PEAK보다 낮다는 근거 없음
     → 150Hz 못 깸(§E13b와 동일 논리).
  2. **SocketCAN이 out-of-tree** — IXXAT V2의 SocketCAN은 HMS가 별도 배포하는 커널모듈(아키텍처별 컴파일).
     mainline 아님 → 커널 업뎃마다 재컴파일/호환 확인 필요(Ubuntu 버그트래커에 HMS SocketCAN 이슈 다수).
     현 PEAK `peak_usb` / Kvaser `kvaser_usb`는 mainline(in-tree)이라 이 부담 없음.
- **교훈: USB 어댑터를 굳이 바꾼다면 mainline 드라이버 유지(PEAK 지속 or Kvaser). IXXAT처럼 스택 이탈 금지.**

### E13d. 기존 어댑터 정체 규명 + 정품 PEAK Pro FD 실측 (2026-07) — **§E13b 부분 정정**
- **기존 = KH-UCANFDX6-Mini (Shenzhen Kunhong/鲲弘电子)** 6채널 CANFD USB 모듈. `lsusb -t`:
  내부 480M 허브(Dev48) 뒤에 **6× `peak_usb` 12M**. 각 채널이 PEAK PCAN-USB FD id(`0c72:0012`)를
  **클론**하며 **full-speed(12M)** 로 enumerate. → §E1의 "6× PEAK FD @12M / QinHeng 허브" 정체 = 이 KH 장치.
- **새거 = 정품 PEAK PCAN-USB Pro FD (`0c72:0011`)**, `lsusb -t`에서 **480M Hi-Speed** 단독(Dev55).
- **★ §E13b 정정:** 기존(full-speed 12M) vs 새거(Hi-Speed 480M)는 **USB quantum이 근본적으로 다름**
  (FS=1ms 프레임 vs HS=125µs microframe, 8배). 우리 병목 ~0.8ms/트랜잭션이 FS quantum 언저리라
  **정품 Hi-Speed Pro FD면 지연이 실제로 줄 여지 있음.** "같은 PEAK 스택이라 무의미"는 이 비교엔 틀림.
  → 실측 가치 있음(단 rate 로깅으로 확인).
- **첫 통신 실패 & 원인 = 종단저항.** Pro FD로 왼팔 붙였더니 TX 128B/**RX 0**, candump 침묵, 전 모터 무응답.
  - PEAK 매뉴얼: **Pro FD 출고 시 종단 비활성**(솔더점퍼/외부 필요). CAN 양끝 120Ω 필수.
  - KH-Mini는 채널별 120Ω 내장으로 추정 → 그동안 외부 종단 없이 동작. KH→PEAK 교체로 어댑터측 종단 소멸.
  - 조치: DB9 pin7(CAN_H)–pin2(CAN_L) 저항 측정(∞=종단0 ★유력 / 120=1개 / 60=정상), 120Ω 추가,
    `ip -s link`로 bus-off/err 확인, 핀아웃(7=H,2=L,3=GND) 검증. **[진행중]**
