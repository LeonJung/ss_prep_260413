# TODO

## 나중에 (deferred)
- [ ] **docs.openarmx.com/en/ 문서 가져와 분석에 포함.** 현재 TLS 인증서 오류(호스트명 불일치)로
      WebFetch 접근 불가 + 검색 인덱싱 안 됨. 방법: 운영자가 브라우저에서 열어 붙여넣기, 또는 세션에서
      `! curl -k https://docs.openarmx.com/en/` (인증서 검증 무시)로 받아 전달. → study_for_v1.md §1.7 갱신.
      (실질 정보는 설치된 openarmx-can 헤더에서 이미 확보; docs는 보완용.)

## 결정 대기 (운영자)
- [x] **v1.0 정의 [확정]**: enactic bilateral 코드/알고리즘 그대로 포팅(옵션1) + openarmx 파라미터·
      기구·좌표 튜닝으로 근본적 transparency 향상. → 새 패키지 openarmx_teleop. (study_for_v1.md ★/§4)
- [ ] **HW 투자 의향**: HS-USB(480M)/PCIe + FD지원 모터로 150Hz 상한 돌파할지. (있으면 500Hz급 가능.)

## openarmx_teleop(공유메모리 포팅판) 뻑뻑함 남은 레버 = 딱 2개
- [ ] **관절별 kp/kd 튜닝** — **운영자 직접 담당.** (config/{leader,follower}.yaml, 재빌드 불필요.
      어깨↑ 벽강성 / 손목↓. 옛날 J4 유독 뻑뻑했던 것 참고.)
- [ ] **USB/CAN 속도 하드웨어 교체** — 150Hz 상한 돌파용. **[확정] 제어 PC=SK-M03(심천 미니PC)은
      물리 확장슬롯 없음(§E13a 확인) → `Kvaser USBcan Pro 2xHS v2`(Hi-Speed USB).** PCIe/mini-PCIe 불가.
      상세 = LATENCY_INVESTIGATION.md §E12/§E13/§E13a. ⚠ 싼 slcan(Waveshare) 금지. gain은 rate(150↑)이지 지터 아님.
      설치 후 `[Leader ctrl] Hz` 로깅으로 150 초과 실측 후 확정.

## 마찰 (조사 완료, 결론)
- [~] friction Fv/Fo — **미확정.** 옛 데이터 재적합=Fv 0/Fo 아티팩트(위험). back-EMF는 무전원 한정,
      운전 중 드라이버가 보상. 확정하려면 넓은 속도범위 재식별 필요. (study §6) 우선순위 낮음.
- [ ] 그리퍼 마찰보상 (파킹됨, GRIPPER_FRICTION_DRIVER_PATCH.md).

## 옛 openarmx_bilateral(ros2_control) 관련 (참고용, 이제 openarmx_teleop이 주력)
- [ ] 관절별 kp 벽강성 스윕 (bilateral.launch kp/kd 인자 이미 구현됨).
