# oa_fd_cpp — 결정 로그 (Decision Log)

각 항목: **무엇을 / 왜 / 기각한 대안**. 날짜는 대략. 새 결정은 맨 아래에 추가.
("앞으로 어떤 결정을 어떤 고민에서 왜 내렸는지 기록" — 2026-06-19 운영자 요청.)

---

## 아키텍처 / 제어 골격

### D1. 제어법 = enactic openarm bilateral 그대로 (MIT 임피던스 + 중력/마찰 FF)
- **무엇**: τ = Kp(q_ref−q)+Kd(dq_ref−dq)+g(q)+friction(q̇), 교차결합(leader_ref=follower state). Kp/Kd는 모터측 MIT, g/friction은 PC측 FF.
- **왜**: enactic openarm_teleop bilateral_step과 동일 구조(코드 대조 확인). 검증된 스킴 재사용.
- **차이**: enactic=분산(2 PC, 네트워크), 우리=단일프로세스 bimanual. 모터 Damiao→Robstride(openarmx).

### D2. 위치/속도 강성은 전부 모터측(MIT), PC측은 천천히 변하는 FF(중력)만 ★근본원리★
- **무엇**: 리밋 반발·posture 스프링을 모터측 MIT 임피던스로. PC측 토크 피드백 금지.
- **왜**: PC측은 CAN 왕복지연(2~6ms)+1kHz 이산화. **위치함수(중력)는 지연 무해, 속도함수(마찰·감쇠)는 지연되면 음의 감쇠→비수동→발작**. j2가 펜스 안 13Hz/12Nm 발작한 게 증거(저관성 자세서 PC측 감쇠가 가진기로). 모터 로컬 루프만 지연0.
- **기각**: PC측 zone 토크(3변형 다 실패), 필터속도 감쇠(지연이 에너지 주입).

### D3. 관성/코리올리 보상 안 함 (leader·follower 둘 다)
- **왜**: 속도·가속 피드백을 지연채널에 얹어 발작 악화. enactic도 bilateral선 코리올리 계산만 하고 안 씀.

---

## 중력 보상 모델

### D4. 중력 = KDL ChainDynParam::JntToGravity, Chain을 멤버로 소유
- **왜**: ChainDynParam이 Chain const-ref 보관 → dangling이면 rc=-3, g=0. 멤버로 수명 보장.

### D5. 중력벡터 = link0 프레임 Z(+9.81), Y 아님
- **무엇**: vec=(0,0,+9.81) 양팔 공통.
- **왜**: cali 실측이 Z모델과 **상관 0.99 / rawRMS 0.98**, Y는 상관≤0.5 (2026-06-19 재확인). j1 axis (0,-1,0).
- **기각**: Y(0,±9.81,0) — 한 자세선 g1(q1) 평탄해 보였으나 **실측 데이터가 Z를 강하게 지지**. 초기 "Y가 팔 날림"은 그때 fit 회귀자 버그 탓.

### D6. 모델 보정 = 질량/COM 측정·fit, **scaling 금지** ★중요 교훈★
- **무엇**: gravity cali(정지토크 측정) → fit_gravity로 per-link COM 보정. scale/scale_joints 사용 안 함.
- **왜**: leader 튜닝 때 **per-joint scaling으론 모델오차를 못 잡는다**고 결론. 그래서 grav cali를 도입한 것. (2026-06-19 운영자 재확인: "scaling 안 하기로 했잖아".)
- **기각**: gravity scale_joints 튜닝 — 폐기됨. 폭주/부정확은 scaling이 아니라 모델/데이터로 해결.

### D7. fit_gravity 회귀자 = 관절 k는 DISTAL 링크만 합산 ★버그수정★
- **왜**: 원래 전 링크(상류 포함) 합산 → 유령토크(j6/j7 최대 11Nm). "j6/j7 텔레메트리 깨짐"은 거짓이었고 실은 이 버그. 수정 후 오프라인=런타임 KDL 5e-5 일치, residual 2.5→0.28.

### D8. URDF 계보 = openarmx 기구 + enactic v2 질량/COM (openarmx_arm_v2com.urdf)
- **왜**: openarmx와 enactic description 수치 비교 후 enactic mass/COM 채택이 실기서 우월.

---

## 마찰 보상

### D9. 마찰 = tanh(Fc·tanh(kv)+Fv·v+Fo), **속도 게이트** 필수
- **왜**: FREEDRIVE는 Kp=Kd=0이라 Fc·tanh(kv)가 순수 음의 감쇠 → 밀면 폭주. 게이트로 정지 부근 보상 0 → 폭주 제거. v_full≤v_start면 게이트 off(구식).

### D10. Fo 제거, q4 점성보상 off, q2 k 낮춤(8→2)
- **Fo**: 정지 상수토크라 시작 드리프트 유발 → 0. 중력모델이 그 bias 흡수.
- **q4**: 식별이 순수 점성(Fc=0,Fv큰값)=전속도 음의 감쇠 → 리밋존서 발산 → off.
- **q2 k**: 가파른 tanh가 영속도서 bang-bang→1~3Hz 한계진동(갈갈갈). k↓로 완만하게. (단 left q2는 모터개체 문제라 못잡음, D17.)
- **마찰값 ≤ 실제마찰**(x0.75~0.85): net 항상 소산.

### D11. 마찰 속도 게이트는 per-joint, q2만 넓게(v_full 0.40)
- **왜**: q2 한계진동이 작동속도(0.5rad/s)서 나서, 그 저속대 보상을 0으로(넓은 게이트) 눌러 진동 제거하되 작동속도선 full 유지.

---

## FREEDRIVE 형상화 / 안전

### D12. q3/q5 posture 스프링 (모터측 약한 Kp→0)
- **왜**: 7-DOF 여자유도서 q3↔q5 self-motion이 팔꿈치 뒤집음. 약한 스프링으로 자연자세 복원, 손으로 이김.

### D13. 관절 리밋 반발 = 모터측, 대칭 캡 스프링 + raw-qd 감쇠(수동), 진입/탈출 비대칭 금지
- **왜**: PC측·필터감쇠·exit_scale 트릭 다 에너지 주입(발작/반동/채터). 수동요소(스프링+감쇠)만 안전. q1 left 펜스 −120~+45°, fmax 캡.

### D14. 콜드스타트 = enable시 disable→100ms→enable 사이클 + 텔레메트리 게이트
- **왜**: 첫 런치 q=0(텔레메트리 미흐름)→중력보상 조용히 죽음. 재런치가 고친 건 직전 disable 때문 → 항상 그 사이클 수행. 7/7 모터 live+enabled 확인 전 토크 거부.

### D15. 위치지령 하드 클램프 + cali pre-flight 거부 ★안전★
- **왜**: 가동범위 밖 위치지령 → Robstride fault → 토크 drop → 팔 풀림(부상). write_mit이 기계적 한계로 클램프. cali는 자세 범위 검증해 시작 거부.

---

## 좌우 미러 (오른팔)

### D16. 오른팔 = 시상면 미러, 관절축 미러맵 m=[-1,-1,-1,1,-1,1,-1] (g_right=m⊙grav(m⊙q))
- **무엇**: q1,q2,q3,q5,q7 부호반전, q4,q6 동일. 중력모델에만 적용(마찰/posture는 모터로컬이라 불필요).
- **왜**: 전역 벡터부호로는 불가(−9.81 저항/+9.81 솟구침). 모터 회전방향("q2 동일")≠공간미러부호 — q2는 미러(-1)가 맞고, 이걸 +1로 둬서 자세 안쪽충돌+q2 fly-up 났음(수정). 자세도 m⊙(검증된 좌측자세).
- **기각**: 모터 reverse(MIT 없음), Robstride 방향 param 레지스터(영구·위험).

---

## 하드웨어 한계 (회고)

### D17. left q2 갈갈갈 = 그 모터 개체 문제
- **왜**: 동일 코드/제어인데 right q2는 무진동, left q2만 진동. 마찰튜닝으로 못잡음(원인이 마찰 아님). 위치주기 토크리플=모터 토크상수 리플. TODO.

### D18. 양팔 동시 = USB 1포트/4채널 대역폭 병목 → 텔레메트리 드롭/q 점프
- **왜**: 버스 전기분리(candump 무신호)+프로세스분리해도 동시구동시 양쪽 q 물리불가 점프. 단독은 OK. timestep↓ 무효. 한 USB 엔드포인트 공유가 병목. → both leader는 나중, USB 분산 필요.

---

## per-arm 분리 (진행중)

### D19. 4팔 파라미터 완전 분리 (leader/follower × L/R 각 자체완결)
- **왜**: 운영자 요청. follower=그리퍼(무거움), leader=핸들 → 모델 다름. enactic도 dynamics_l_/dynamics_f_ 분리.
- **방식**: follower 캘리 먼저 → 점진 리팩터(①per-role URDF ②per-role friction ③4-파일). 이전 per-role URDF(77b2033) 실기 폭주 이력 → 단계별 실기검증 필수("leader처럼 실로봇 검증").
- **현황**: ①per-role URDF 완료(a3de2f6). CSV는 git 미포함(로컬 cali_data/, gitignore).

### D20. follower 중력 base = v2com + 그리퍼(hand0.35+finger2) link7에 합산
- **왜**: 질량은 fit서 pin이라 미리 넣어야. 그리퍼 수치는 openarmx_robot.urdf(enactic)서. 운영자 통찰 "follower=enactic 표준모델" 확인됨.

### D21. (미해결, 2026-06-19) follower 시작자세 폭주 = 모델 커버리지 구멍, scaling 아님
- **증상**: follower-left q1이 시작(q4≈0 곧음+q2≈0 수평)서 +펜스로 폭주. follower-right q4/q6/q7 반동.
- **진단**: cali가 q4≥0.3에서만 측정 → 시작영역(q4<0.3) 데이터 없어 외삽, 무거운 그리퍼가 q1 과보상 증폭. leader는 가벼워 견딤.
- **결정 방향**: scaling 금지(D6) → **cali 커버리지 확장(저 q4+수평 근방)으로 모델을 그 영역까지 측정**, 재fit. (자세 q4 하한 결정 대기.)
