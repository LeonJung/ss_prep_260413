# PCAN-USB Pro FD 도입 발자취 (150Hz 상한 돌파 시도)

bilateral teleop의 CAN 제어주기 **150Hz 상한**을 하드웨어로 돌파하려고 정품 PEAK **PCAN-USB Pro FD**를
도입한 전 과정 기록. **한 줄 결론(현재): 기존 어댑터가 full-speed(12M)였고 새 Pro FD는 Hi-Speed(480M)라
지연 개선 여지가 실제로 있으며, 첫 통신 실패의 원인은 순수하게 "종단저항"이었다(루프백으로 확정).**

관련: 상세 실험 로그 = `LATENCY_INVESTIGATION.md`(§E1~E13d), 루프백 도구 = `scripts/can_loopback_test.py`.

---

## 0. 배경 — 왜 USB 어댑터를 건드리나
- bilateral 제어주기가 **150Hz에서 상한**(운영자 rate 스윕: 100Hz 뻑뻑 / 150Hz 최적 / 200Hz 뒷joint 추종느림 /
  250Hz 진동). 원인 = **USB 동기 왕복 지연(~0.8ms/트랜잭션)**, CAN 버스 대역(1Mbps, ~475Hz 여유)이 아님.
- 제어 PC = **SK-M03**(深圳 航柏/Hangbai 미니PC). `dmidecode -t slot`이 전부 PCH 루트포트 보일러플레이트 →
  **물리 확장슬롯 없음** → 데스크톱/mini-PCIe CAN 카드 불가 → **Hi-Speed USB 어댑터**로 좁힘. (§E13a)

## 1. 어댑터 후보 검토 (구매 전 데스크 리서치)
- **PEAK Pro FD "IPEH-004061 호환" 클론** → ❌ 금지. `peak_usb` 미바인딩·지연 미검증 리스크(slcan 동급). (§E13b)
- **정품 PEAK Pro FD** → 당시엔 "같은 PEAK USB 스택이라 rate 이득 회의적"으로 판단. (§E13b, **뒤에서 정정됨**)
- **IXXAT USB-to-CAN V2** → ❌ 비권장. 같은 USB 벽 + SocketCAN이 out-of-tree(커널 업뎃마다 재컴파일). (§E13c)
- **Kvaser Hi-Speed USB** → mainline `kvaser_usb`, 그나마 다른 스택 베팅.
- 원칙: **USB 바꾸면 mainline 드라이버 유지**(PEAK/Kvaser), slcan(Waveshare) 금지, 얻는 건 rate지 지터 아님.

## 2. 반전 — 기존 어댑터의 정체 규명 (`lsusb -t`)
운영자가 실제 하드웨어를 확인하면서 그림이 바뀜:
- **기존 = KH-UCANFDX6-Mini** (Shenzhen Kunhong/鲲弘电子) 6채널 CANFD USB 모듈.
  - `lsusb -t`: 내부 480M 허브(Dev48) 뒤에 **6× `peak_usb` 12M**. 각 채널이 PEAK PCAN-USB FD 아이디
    (`0c72:0012`)를 **클론**하며 **full-speed(12M)** 로 enumerate.
  - → §E1의 "6× PEAK FD @12M / QinHeng 허브" 정체가 바로 이 **KH 6-in-1 장치**였음.
- **새거 = 정품 PEAK PCAN-USB Pro FD** (`0c72:0011`), `lsusb -t`에서 **480M Hi-Speed** 단독(Dev55).

**★ §E13b 정정:** 기존(FS 12M) vs 새거(HS 480M)는 **USB quantum이 근본적으로 다름**
(**full-speed = 1ms 프레임** vs **Hi-Speed = 125µs microframe**, 8배 촘촘). 우리 병목 ~0.8ms/트랜잭션이
FS quantum 언저리라 → **정품 Hi-Speed Pro FD면 지연이 실제로 줄 여지가 있음.** "같은 스택이라 무의미"는
이 비교엔 틀렸음. → **실측 가치 확실.** (단 rate 로깅으로 검증 필요.)

## 3. 첫 통신 실패
Pro FD의 한 채널을 leader 왼팔 CAN 하네스에 물리고 1Mbps로 UP:
- 증상: **TX 128B / RX 0**, `candump` 침묵, `check_motor_status.py`에서 왼팔 전 모터 **무응답**.
- LED: USB 연결 시 초록 상시, `can up` 하면 초록 점멸(주기적) — 즉 인터페이스/드라이버는 살아있음.
- 운영자 직감: "이전 KH로 쓸 땐 **종단저항을 따로 안 넣었는데** 됐다" → 여기가 핵심 단서.

## 4. 원인 = 종단저항 (KH는 내장 ON, Pro FD는 OFF)
- PEAK 매뉴얼: PCAN-USB Pro FD는 **내장 120Ω 종단이 있으나 스위처블 + 출고 시 OFF**
  ("At delivery the termination is not activated"). "이미 붙어있다"는 절반만 맞음.
- **Linux mainline `peak_usb`는 종단 제어 미구현** — 커널 소스 `pcan_usb_fd.c` 직접 확인
  (`termination`/`do_set_termination` 없음). → `ip link set canX type can termination 120` **안 먹힘**.
  - 켜려면 Windows PCAN-View / PEAK 독점 드라이버(SocketCAN 워크플로 깨질 위험) → 비실용.
- KH-UCANFDX6-Mini 같은 다채널 모듈은 **채널별 120Ω 내장(ON)** 이라 그동안 외부 종단 없이 동작.
  KH→PEAK 교체로 **어댑터측 종단이 사라져** 버스 종단 부족 → 무통신.
- **실용적 해결 = 외부 120Ω 1개** (DB9 pin7=CAN_H – pin2=CAN_L 사이), 또는 PEAK PCAN-Term 플러그.

## 5. 루프백 검증으로 원인 확정 (`scripts/can_loopback_test.py`)
로봇을 떼고 Pro FD 두 채널을 직결(CAN_H-CAN_H, CAN_L-CAN_L)해 2노드 버스로 격리 테스트:
- **종단 없음 → 0/10 실패**
- **120Ω 삽입 → 10/10 성공**
- **결론:** 어댑터·드라이버·양 채널 모두 **정상**. 첫 통신 실패는 **순수하게 종단저항 문제**로 확정.
  (Pro FD 내장 종단이 실제로 OFF임도 이 결과로 실증.)

## 6. 현재 상태 & 다음 단계
- [x] 기존/신규 어댑터 정체·속도 규명 (KH 12M / Pro FD 480M).
- [x] 첫 통신 실패 원인 = 종단저항 확정 (루프백 0/10 → 10/10).
- [ ] **로봇 버스에 120Ω 종단 넣고** Pro FD로 왼팔 8모터 통신 확인(`check_motor_status.py`).
      (로봇 반대쪽 끝에 종단 있으면 Pro FD측 1개로 60Ω 완성. 없으면 최소 KH 때처럼 1개 확보.)
- [x] **핵심 실측 [달성]**: Pro FD로 bilateral → **500Hz** (`period 1999us / step 816us / jitter 700us`).
      KH 150Hz → 500Hz ≈ 3.3배. HS quantum 가설 실증. step 816us라 이론상한 ~1225Hz(여유 있음).
      **★ 개선 출처 = USB 전송속도이지 CAN-FD 아님** — Robstride는 classic CAN 1Mbps 전용, CAN 와이어는 그대로.
- [ ] **검증**: candump로 8모터 실제 500Hz 응답(뒷joint starvation 무) / follower도 Pro FD 채널인지 / 실모션 안정.
- [ ] 500Hz에서 **enactic 근위 kp(240) 재도전** — 예전 KH 150Hz의 kp50 상한이 rate↑로 풀리는지.
- [ ] 개선 확정되면 8모터 전체를 Pro FD 계열로 확장.

## 배운 것 (요약)
1. **`lsusb -t`로 실제 enumerate 속도(12M vs 480M)를 봐야 한다** — 제품명·VID/PID만으론 KH가 PEAK id를
   클론한 걸 못 걸러냄. full-speed였다는 게 150Hz 상한의 실질 근원 후보.
2. **다채널 중국제 모듈은 종단 내장(ON)이 흔하다** — 정품 PEAK로 갈면 종단이 사라져 "멀쩡한데 무통신"이 됨.
   종단은 Linux mainline peak_usb로 못 켜니 **외부 120Ω**가 답.
3. **루프백(2채널 직결) 테스트가 어댑터 vs 로봇측 문제를 깔끔히 가른다** — 종단 유무로 0/10↔10/10이 갈려
   종단이 원인임까지 실증됨.
