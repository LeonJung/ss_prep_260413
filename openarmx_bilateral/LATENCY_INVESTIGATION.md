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
    `ip -s link`로 bus-off/err 확인, 핀아웃(7=H,2=L,3=GND) 검증.
  - **[확정]** 루프백(`scripts/can_loopback_test.py`, 2채널 직결): **무종단 0/10 → 120Ω 10/10.**
    → 어댑터·드라이버·양 채널 정상, 첫 실패는 순수 종단 문제로 확정(Pro FD 내장종단 OFF도 실증).
    Linux mainline `peak_usb`는 종단 미제어(커널소스 확인) → 외부 120Ω 필요. 전 과정 = `PCAN_PROFD_MIGRATION.md`.

### E13e. Pro FD 실측 — **500Hz 달성** (2026-07) ★도입 목적 검증
- leader 스레드 로그: `period mean 1999.49us = 500.17Hz`, `step mean 816.7us / max 965us`, `jitter 700us`, 501 cyc.
- **KH(FS 12M) 150Hz → Pro FD(HS 480M) 500Hz ≈ 3.3배.** §E13d 가설(FS 1ms→HS 125µs quantum) 실증.
- **★ 개선 출처 = USB 전송속도(host↔어댑터 링크), CAN-FD 아님.** Robstride는 **classic CAN 1Mbps 전용**(FD 불가, §E2).
  CAN 와이어는 그대로 1Mbps classic 유지. → 병목이 CAN 버스가 아니라 **USB 왕복 quantum**이었음이 이 결과로 확정.
- step 816us → 이론상한 ≈ 1/816µs ≈ **1225Hz** → 500Hz는 왕복예산 41%만 사용, **여유 있음**(더 스윕 가능).
- 시너지: rate↑ → 高 kp 안정. KH 150Hz에서 kp50 못 넘던 제약이 여기서 풀릴 여지(enactic 240 근위 재도전 가능).
- **검증 대기**: candump로 8모터 실제 500Hz 응답(뒷joint 5~8 starvation 무) / 양 채널 다 Pro FD인지 / 실모션 안정성.

### E13f. 500Hz "간헐 발작" 원인 규명 — **제어 스레드가 비-RT(SCHED_OTHER)** (2026-07) ★확정
증상: enactic kp를 8/9(≈210)까지 올려 반력 좋아졌으나, **~10초마다 로봇이 한 번 발작**. 운영자가 로그에서
"200000µs 이상" 지연을 봤다고 함. can0=leader-left, can1=follower-left, 둘 다 **한 개 PCAN-USB Pro FD**
(`parentdev 1-3:1.0` 공유). 데이터로 하나씩 배제:
- **"200000µs"는 유령 = 로그 아티팩트.** `[Leader ctrl]`/`[Follower ctrl]` 두 스레드가 **락 없이 같은 stdout에
  동시 printf** → 숫자 엉킴(증거: `16252353`=1625+2353, `7141682`=714+1682, 한 줄에 양 스레드 박힌 줄 8개).
  **실제 최악 지표는 period max ~3.9ms / step max ~2.9ms.** 200ms짜리 진짜 지연은 로그에 없음.
- **CAN 버스 무죄(하드 카운터).** 발작 전후 `ip -s`: bus-off 0, error-pass 0, bus-errors 0, RX/TX errors 0.
  `restart-ms=0`(200ms 복구 메커니즘 자체가 없음). candump 지상검증: 와이어 최대 gap **can0 2246µs/can1 1700µs**,
  >2.5ms 0건, 뒷joint 매 사이클 다 옴(starvation 무). → **종단·노이즈·starvation 전부 배제.**
- **CPU 전원관리 무죄.** governor `powersave→performance` 바꿔도 스파이크 그대로(worst 3827→3934µs). C-state 무관.
- **전역 커널 스톨 무죄.** bilateral 부하 중 `cyclictest -p99`: **max 315µs**(안 튐). THP 파일 부재 → **khugepaged
  존재 안 함**. IRQ/SMI 계열 배제(hwlatdetect max 356µs).
- **★ 진짜 원인:** `ps -T`로 실제 제어 프로세스(`bilateral_contr`, PID≠런처) 확인 → **스레드 4개 전부 `CLS=TS`
  (SCHED_OTHER, 일반 우선순위), RTPRIO 없음, `VmLck 0`(mlock 없음).** 500Hz 하드 RT 루프가 **비-RT로 돌고 있었음.**
  그래서 ~10초마다 일반 우선순위 백그라운드(DDS discovery/커널 워커 등)에 **2~3ms preempt** → stale dt → 高 kp·kd가
  속도항 폭발로 증폭 → 발작. 스파이크가 **leader·follower 쌍으로 동시** 발생도 전역 preempt로 설명. cyclictest(RT)가
  같은 부하서 315µs인 게 "RT만 주면 잡힌다"는 증거.
- **해결(진행):** ① 제어 스레드 `SCHED_FIFO prio 80` + `mlockall(MCL_CURRENT|MCL_FUTURE)`, setcap에 `cap_ipc_lock`
  추가(`cap_sys_nice,cap_ipc_lock+ep`). 런타임 임시검증 = `chrt -f -p 80 <tid>`로 10초 스파이크 소멸 확인.
  ② 로그 두 스레드 printf에 mutex(또는 단일 라인). ③ late-cycle dt 가드(측정 dt>임계면 속도항 skip/직전값 재사용).
  IRQ 스레드 prio 50 < 80이지만 제어 스레드가 blocking read로 잠들어 USB IRQ 정상 동작 → 80 안전.

### E13g. 함정 — `-O0` 빌드가 rate를 반토막 (2026-07)
§E13f 수정 push 후 재빌드했더니 **500Hz → ~240Hz로 폭락**, RT 켜고 끄고 무관(RT on 230~250 /
off 200~220). 진단: **step mean이 ~800µs → ~4000µs로 5배** = 스케줄링·USB가 아니라 **CPU 연산시간**.
`bilateral_step` 안의 KDL 동역학(GetGravity/GetCoriolis, Eigen)이 **최적화 없이 빌드되면 5~10배 느림**.
- 원인: `build/`는 git 미추적 → pull 후 `cmake --build`가 "could not load cache" → 사용자가 fresh
  configure할 때 `-DCMAKE_BUILD_TYPE=Release`가 빠져 **`-O0`(Debug/none) 바이너리**가 나옴.
  CMakeLists가 최적화를 강제 안 했음(`-Wall`만) → BUILD_TYPE에 100% 의존. Release=`-O3 -DNDEBUG`.
- **조치:** ① 즉시 `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`로 복구.
  ② CMakeLists에 **BUILD_TYPE 미지정 시 Release 기본값** 추가(재발 방지).
- **교훈: rate가 갑자기 정수배로 떨어지고 RT 무관 + step(연산) 팽창이면 최적화 빌드부터 의심.**
  (튐은 이 -O0 상태에서 잠시 사라졌지만, 500Hz Release 복귀 시 재발 가능 → §E13f 튐 가드는 별건.)

### E13h. 함정2 — `mlockall`이 500Hz→240Hz로 떨어뜨림 (2026-07) ★§E13f 부분 롤백
§E13g(Release) 확인했는데도 여전히 **240Hz**. 판별: **USB는 480M(Hi-Speed) 정상**, **push 이전 코드는
같은 USB로 지금도 500Hz** → 원인은 순수하게 **내 코드 변경**. candump: 명령→응답 왕복이 **~1550µs**
(정상 ~200µs)로 팽창, step ~4000µs, RT on/off 무관.
- 후보 3개 중 로그 mutex(초당 1회)·`apply_rt_priority`(외부 chrt FF80은 이미 500Hz로 무죄) 소거 →
  **범인 = `mlockall(MCL_CURRENT|MCL_FUTURE)`.** 프로세스 전체 메모리 잠금이 CAN/USB 드라이버의
  per-transfer 버퍼(DMA) 경로를 느리게 만든 것으로 추정(왕복 8배↑).
- 게다가 §E13f에서 넣은 mlockall은 **이득이 애초에 0**이었음(정상운전 중 minflt flat·majflt 0 = 막을 fault 없음).
- **조치: mlockall 제거**(`openarm_bilateral_control.cpp`). RT(SCHED_FIFO)만 남김 — 그게 진짜 레버.
- **교훈: RT 튜닝에서 mlockall은 만능 아님. page fault가 실측으로 있을 때만. 없는데 넣으면 순손해(여기선 USB 처리량 반토막).**

### E13i. 외부 문헌 조사 — mlockall 성능저하 선례와 문서화된 메커니즘 (2026-08-01, 로봇 접근 불가 중 원격조사)
운영자 오더: "mlockall/pthread RT 적용으로 이런 문제가 생긴 사례가 외부에 있는지 조사". 결과 = **선례 있음,
메커니즘도 문서화되어 있음, 우리 코드가 정확히 그 패턴에 해당. §E13h 의 제거 결정은 문헌상으로도 정답.**

**1) 직접 선례 (mlockall 이 오히려 느리게 만든 보고들)**
- LKML 2000 "why does mlockall appear to make memcpy slower?" — mlockall 프로세스가 page fault **3배**
  (261 major+522 minor vs 88+266), 실행시간 **1.5배** (1.81s vs 1.19s). 스레드 결론: "curiosity, if not an
  actual bug" — mlockall'd 프로세스가 fault 를 더 많이 내는 역설이 20년 전부터 보고됨.
- LKML 2021 "Very slow unlockall()" — munlockall 이 30초+ 걸리는 보고 (cryptsetup + hardened allocator).
  **lock/unlock 북키핑(unevictable LRU 페이지 워크) 자체가 극도로 비쌀 수 있음**의 증거.

**2) 문서화된 메커니즘 (우리 240Hz 폭락과 연결)**
- **MCL_FUTURE = 이후 모든 mmap/heap 성장이 생성 시점에 populate+lock** (LWN 647728). 이 페이지 단위
  populate 오버헤드가 "significant performance penalty" 라서 커널이 **MCL_ONFAULT** 를 따로 만들었을 정도.
- **RT 표준 가이드 (linuxfoundation realtime wiki): mlockall 은 반드시
  `mallopt(M_MMAP_MAX=0, M_TRIM_THRESHOLD=-1, M_ARENA_MAX=1)` + 사전할당(pre-allocation, RT 경로에서
  malloc/free 금지) 와 세트**로 쓰라고 명시. 안 그러면 alloc/free churn 마다 heap 이 커널로 반납→재성장→
  populate+lock 이 반복되는 fault churn 발생.
- glibc 는 non-main(per-thread) arena 의 free 에서 `madvise(MADV_DONTNEED)` 로 페이지 teardown → 다음
  할당에서 refault. **스레드별 arena + 매 사이클 alloc/free = teardown/refault 순환**이 문서화된 함정.
  (MADV_DONTNEED 가 mlocked range 를 historically 거부하는 코너까지 있어 allocator 가정과 충돌.)

**3) 우리 코드가 이 패턴에 해당하는 근거 (코드 확인, 2026-08-01)**
제어 루프가 **매 사이클 heap 할당/해제**를 함: `joint_state_converter.hpp` 의 `motor_to_joint`/
`joint_to_motor` 가 호출마다 `std::vector` 생성, `robot_state.hpp` 의 `get_all_references/responses` 가
벡터 **복사 반환**, cmds `push_back`. 스레드 4개 = glibc arena 4개에서 churn. → 문헌의
"mlockall + 동적할당 churn" 저격 패턴 그대로.

**4) §E13h 추정 부분 정정**
"CAN/USB 드라이버의 per-transfer DMA 버퍼 경로가 느려짐" 가설은 **외부 근거 없음** — peak_usb 드라이버의
URB/DMA 버퍼는 커널 소유라 유저스페이스 mlockall 영향권 밖. 문헌이 지지하는 병목은 **유저스페이스
(allocator fault/lock churn) 경로**. candump 왕복 팽창 (200→1550µs) 은 "응답 수신 후 다음 명령 송신까지의
호스트 처리시간 팽창" 으로 해석하는 게 정합.

**5) 결론 & 향후 지침**
- mlockall 제거 (`ef6992c`) 유지 = 정답. Red Hat RT 문서도 "실측 fault 없으면 이득 없음, 앱 전체 잠금 대신
  RT 부분만 잠그라" — 우리 관측 (minflt flat·majflt 0) 이 정확히 '이득 0' 조건.
- 나중에 mlockall 이 다시 필요해지면 (콜드스타트 fault 등) 반드시 세트로: **MCL_ONFAULT** + mallopt 3종 +
  **per-cycle 벡터 할당 제거 (버퍼 재사용)**. per-cycle 할당 제거는 mlockall 무관하게 지터 레버이기도 함 (선택 TODO).

## E14. 500Hz에서 J7 팡팡 / J8(그리퍼) 무반응 — 원인 추정 (2026-08-01, 로봇 접근 불가 중 원격 분석)

증상 (운영자 보고): 500Hz 운전 시 (1) **J7 이 팡팡 튐**, (2) **J8(그리퍼) 는 leader 를 움직여도 follower 가
어쩌다 한 번만 따라오고 대부분 무반응**, (3) J1~J6 은 정상. 150Hz 에서는 문제 없(었)음.

### ★ 주 가설 — classic CAN 1Mbps 물리 대역폭 초과 (busload >100%), 희생자는 폴링 꼬리의 응답 프레임

산수 (검증 방법은 아래 — 실측으로 확정할 것):
- Robstride 확장(29-bit) 8B 프레임 ≈ **130~155µs @ 1Mbps** (stuffing 포함 추정).
- `bilateral_step` 은 매 사이클 **arm 7 + gripper 1 = 8 명령**, 각 명령이 STATE 응답 유발 → **16 프레임/사이클**.
- 16 × 130~155µs ≈ **2.08~2.48ms** vs 500Hz 예산 **2.00ms** → **busload 104~124%**. 8모터 cmd+resp 의
  이론 상한 rate ≈ **400~430Hz**. **USB 병목(§E13e 해결) 다음 병목이 CAN 와이어 자체.**
- enactic 이 같은 루프 구조로 500Hz 가능했던 이유: **enactic 은 CAN-FD** (`openarm-can-configure-socketcan
  can0 -fd`, study_for_v1.md §1.7) — data phase 고속. Robstride 는 classic 1Mbps 전용이라 같은 패턴이 안 담김.

왜 하필 J7/J8 인가 (증상 gradient 설명):
- CAN arbitration 은 **낮은 ID 우선**. Robstride 명령(type 1)이 응답(type 2)보다 ID 낮아 명령이 항상 이김
  → 초과분은 **응답 쪽에서 탈락**, 그중에서도 **폴링 순서 마지막 = J7, J8 의 응답**이 다음 사이클 명령 버스트와
  충돌해 만성 탈락/지연.
- leader 측 J7/J8 응답 탈락 → leader 상태 동결 → follower 의 J7/J8 **참조(reference)가 동결**:
  - **J8 = 꼬리 끝** → 참조가 거의 항상 동결 = follower 무반응. 어쩌다 응답이 뚫리면 참조가 누적분만큼 점프
    → "어쩌다 한 번 따라움직임". ✓
  - **J7 = 꼬리 둘째** → J8 보단 자주 뚫림 → 뚫릴 때마다 pos/vel 참조 step → kp·kd 킥 = **"팡팡"**. ✓
  - J1~J6 응답은 버스트 앞쪽 시간창에 전송 완료 → 무사. ✓
  - 150Hz 는 busload ≈ 31% → 전 프레임 전달 → 무사. ✓ (rate 의존성 일치)
- 참고: §E8 (구 어댑터에서 "motors 3-8 starved") 와 같은 계열의 꼬리-starvation — 그땐 USB quantum, 지금은 와이어.
- ⚠ §E13f 의 candump 스팟체크 ("뒷joint 매 사이클 다 옴") 와 상충 — 그 체크가 **gripper ID 를 필터에
  포함했는지 불명** + 짧은 창 관측이었을 가능성. §E13e 의 "검증 대기: 뒷joint 5~8 starvation 무" 가 정확히
  이 검증이었고 아직 미실시. 아래 실측으로 판가름.

### 소거된 후보 (코드 확인, 2026-08-01)
- config 인덱싱: leader/follower.yaml 의 Kp/Kd/Fc/k/Fv/Fo 모두 **8원소** ("8th = gripper") — 배열 범위 문제 없음.
- rate 무관 버그였다면 150Hz 에서도 재현됐어야 함 → 코드 로직 단독 원인 아님.

### 🟡 보조 후보 (J8 한정, rate 무관 성분이 섞였을 수 있음)
- J8 게인이 **kp=1.0 / kd=0.1** 로 극약 — 그리퍼 정지마찰을 못 이겨 참조 오차가 커질 때만 훅 움직이는
  패턴도 "어쩌다 한 번" 에 기여 가능. **분별법: 150Hz 에서 J8 추종이 멀쩡했는지 기억/재확인.** 150Hz 에서도
  뻑뻑했다면 kp_hand 상향 or 그리퍼 마찰보상 재개 (GRIPPER_FRICTION_DRIVER_PATCH.md, 파킹 중) 병행.

### 관측 데이터 (2026-08-02, 운영자)
- **200Hz (push 된 최신 버전 = Release + RT-only): J7 확실히 안 튐.** J8 은 미확인.
- 가설과 정합: 200Hz busload ≈ 16 × 130~155µs / 5ms = **42~50%** → 응답 탈락 없음 → J7 무사.
- 단, 200Hz 단독으론 "rate 의존 원인" 임을 재확인할 뿐 busload 가설 확정은 아님 — 확정은
  (a) 500Hz candump per-ID 카운트, 또는 (b) rate 스윕에서 **경계가 ~400-430Hz 부근**인지로.
  350/400/450/500 스윕에서 J7 튀기 시작하는 지점이 430 근처면 강한 실증 (candump 없이도).

### 실측 계획 (월요일 로봇 접근 시, [제어 PC])
```bash
# [제어 PC] 1) 500Hz 운전 중 per-ID 프레임 카운트 (10초) — 핵심 판정
candump -ta can0 -n 100000 > /tmp/candump_500.log   # 별도 셸에서 10s 후 Ctrl-C
# ID 별 count: 명령 8종 각각 ~5000, 응답 J1~J6 ~5000 나오는데
# ★ J7/J8 응답이 유의미하게 적으면 (예: <4000) 주 가설 확정
awk '{print $3}' /tmp/candump_500.log | sort | uniq -c | sort -rn

# [제어 PC] 2) 같은 방법으로 150Hz 대조군 → 전 ID 균일 ~1500 확인

# [제어 PC] 3) 실제 프레임 길이 실측 (back-to-back 타임스탬프 델타) → busload 정밀 계산
# 4) TX 드랍 여부: ip -s -d link show can0  (dropped / error 카운터, 운전 전후 비교)
```

### Fix 후보 (실측 확정 후 적용 순)
1. **rate 를 400Hz 이하로** — 가장 빠른 검증 겸 fix. 500→450→400 스윕에서 J7/J8 이 ~430 부근에서
   낫는지 확인 (낫는 경계 = busload 100% 지점 → 가설 실증).
2. **그리퍼 decimation** — J8 명령을 N 사이클에 1회 (예: 4:1 → 그리퍼 125Hz, 팔은 500Hz 유지).
   16 → 평균 14.5 프레임/사이클. 그리퍼는 500Hz 불필요하므로 부작용 없음. 팔 7모터만으로도
   14 × 130~155µs ≈ 1.82~2.17ms 로 **여전히 경계선** — 1번과 병행 필요할 수 있음.
3. **버스 분할 (HW)** — 채널 추가 (Pro FD 1대 더 or Kvaser 2xHS) 로 팔당 2버스 (4+4 모터)
   → 버스당 8프레임/사이클 ≈ 52~62% → 500Hz 여유. §E12/E13 의 HW 논의와 합류.
4. (해당 시) J8 kp_hand 상향 / 그리퍼 마찰보상 재개 — 보조 후보가 실증되면.
