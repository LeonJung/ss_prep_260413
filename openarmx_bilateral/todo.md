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

## 저비용 개선 카드 (포팅 무관, 언제든)
- [ ] friction Fv/Fo 도입 — friction_id를 4파라미터 적합으로 확장(자유공간 가벼움↑).
- [ ] 관절별 kp 벽강성 스윕 (kp/kd 인자 이미 구현됨).
- [ ] 그리퍼 마찰보상 (파킹됨, GRIPPER_FRICTION_DRIVER_PATCH.md).
