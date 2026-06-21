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

### D19. 4팔 파라미터 완전 분리 (leader/follower × L/R 각 자체완결) — ★구현됨 48f79a9★
- **구현**: config를 ArmCfg(arm 1개 전체 파라미터)+GlobalCfg(루프 전역)로 재구성.
  노드가 4개 yaml(`config/oa_fd_{leader,follower}_{left,right}.yaml`)을 각 arm에 로드.
  Pair가 lead_cfg/foll_cfg 포인터 보유, compute_pair/friction/zone/gravity 전부 per-arm.
  oa_fd.yaml 폐기. 한 yaml에 friction 블록 쌓던 것(운영자 지적 "말이 안 됨") 해소.
- 각 파일 자체완결: can, gravity{urdf·vec·mirror·scale}, friction, Kp/Kd/home/torque,
  couple_mirror, freedrive shaping, joint_limits. 공통(timestep 등)은 중복.
- follower 파일: friction OFF로 시작(중력 먼저 검증→점진추가), q4 펜스 off(기구 스토퍼).

### D19-old. (배경) 4팔 분리 결정
- **왜**: 운영자 요청. follower=그리퍼(무거움), leader=핸들 → 모델 다름. enactic도 dynamics_l_/dynamics_f_ 분리.
- **방식**: follower 캘리 먼저 → 점진 리팩터(①per-role URDF ②per-role friction ③4-파일). 이전 per-role URDF(77b2033) 실기 폭주 이력 → 단계별 실기검증 필수("leader처럼 실로봇 검증").
- **현황**: ①per-role URDF 완료(a3de2f6). CSV는 git 미포함(로컬 cali_data/, gitignore).

### D20. follower 중력 base = v2com + 그리퍼(hand0.35+finger2) link7에 합산
- **왜**: 질량은 fit서 pin이라 미리 넣어야. 그리퍼 수치는 openarmx_robot.urdf(enactic)서. 운영자 통찰 "follower=enactic 표준모델" 확인됨.

### D21. (미해결, 2026-06-19) follower 시작자세 폭주 = 모델 커버리지 구멍, scaling 아님
- **증상**: follower-left q1이 시작(q4≈0 곧음+q2≈0 수평)서 +펜스로 폭주. follower-right q4/q6/q7 반동.
- **진단**: cali가 q4≥0.3에서만 측정 → 시작영역(q4<0.3) 데이터 없어 외삽, 무거운 그리퍼가 q1 과보상 증폭. leader는 가벼워 견딤.
- **결정 방향(초안, 철회)**: cali 커버리지 확장 — 운영자가 "절차 바꾸지 말고 기존 방식으로" 지적, 철회.
- **확정 (2026-06-19)**: cali 절차는 leader와 **이미 동일**(감사: q1/q2/q4 커버리지·도구·플래그 동일, 잔차 0.22 동일). 문제는 절차가 아니라 **full --fit-links 1-7 fit이 미결정**이라 같은 팔(link0-6)인데 leader/follower COM이 비물리적으로 다르게(q4/q5 COM-z 부호반전) 나온 것. 무거운 그리퍼가 follower 외삽을 q1 폭주로 증폭.
  → **수정**: "end tool만 다르다"는 물리 그대로 — follower URDF = **leader link0-6 동결 + link7만 fit**(그리퍼 m1.047). link0-6 동일(<0.001), link7만 다름. 6a76eaa. 실기검증 대기(수평 시작서 leader처럼 멈추는지).
  - 교훈: 같은 팔은 같은 파라미터여야. 재캘리 전에 **leader와 파라미터 직접 비교**로 비물리 차이부터 찾을 것.
- **link7-only / 무게보정 시도 실패 (2026-06-19)**: leader팔+그리퍼(link7만 fit)는 g1이 오히려↑ → follower-right까지 폭주 → 되돌림(5192e58). 그리퍼 무게 1.047→0.94(1.5배) 보정도 fit이 재흡수해 g1 거의 불변. 즉 단발 모델조작으론 안 됨.
- **불안정 평형 = 과보상 확정 (운영자 관찰)**: follower q1을 어떤 균형점서 양쪽으로 밀면 그 방향 가속(negative stiffness) = comp>실제중력. leader는 간당 안정, 무거운 follower가 임계 넘김. q4도 동일. **comp은 반드시 필요**(없으면 follower 처짐→leader에 무게=투명도↓; 과보상도 ACTIVE서 follower 위치 어긋나 leader에 힘=투명도↓). 그러니 **정확한 comp**가 답(scaling 금지).
- **q4 커버리지 가설 철회 (운영자 지적)**: leader-right도 q4[0.13,1.55] 같은 커버리지로 cali했는데 안정 → q4 커버리지는 원인 아님. (gen_cali_poses --q4-min/--q4-max 추가는 남겨두되 follower엔 불필요.)
### D30. ★UR10e bilateral은 풀스택(4CH+DOB+EnergyTank)이라 됐고, OpenArm naive는 그게 없어 안 됨 (2026-06-21)
- 운영자 질문: UR10e bilateral은 됐는데 OpenArm은 왜 진동/뻑뻑? → `ur10e_teleop_control_hybrid_cpp` 정독.
- UR10e = SOTA 풀스택(실기검증): **DynamicsModel(M,C,g) + VelocityEstimator(LPF) + DisturbanceObserver(τ̂_ext 무센서) + FourChannelController(Lawrence, Kp·e_pos+Kd·e_vel+Kf_self·τ̂_ext+Kf_peer·τ̂_ext_peer) + EnergyTank(2-layer 수동성)**, 500Hz RT, leader/follower 토픽통신. (Buamanee2025 4CH+DOB / Franken-Stramigioli EnergyTank.)
- 증상 매핑: **뻑뻑=DOB 부재**(우리는 위치지연 반사→파트너 관성/마찰까지 느낌; DOB는 외력 τ̂_ext만 반사→투명). **진동=EnergyTank 부재**(지연채널 force feedback은 비수동→에너지주입→한계진동; 탱크가 수동성 보장→지연강건). oa_mit엔 둘 다 없음(순수 Kp 스프링).
- **HW 각도**: UR10e 고감속 하모닉=고마찰/감쇠라 관대(그래도 탱크 씀). OpenArm/Robstride 저감속 quasi-DD=저마찰/backdrivable→**자연 감쇠 거의 없음**→지연피드백 에너지 흡수 못 함→더 잘 진동. **OpenArm이 UR10e보다 이 기계장치가 더 필요**.
- **결론/경로**: naive 재발명 말고 **UR10e hybrid 제어코어를 OpenArm에 포팅**. I/O만 교체(urcl/RTDE torque → openarmx_can MIT torque: kp=kd=0,torque=계산, oa_mit leader가 이미 그 방식). follower도 MIT-torque로. 상태머신/RT/leader-follower 토픽통신 재사용 → 2-PC 원격 목표와 정합(탱크가 네트워크 지연 흡수). 좋은 M(q)/C 위한 URDF 검증 필요(cali 중력URDF 보유). 주의(UR10e README): DOB 반력 "URDF 모델하 약함", apparent inertia 하한 존재, EnergyTank HW-미검증. 상세: [oa_mit/BILATERAL_NOTES.md].

### D29. ★최종목표=2-PC 원격 → 지연채널 전제 설계, 단일프로세스 폐기, oa_mit 구조 채택 (2026-06-20)
- **새 정보(운영자)**: 두 양팔로봇을 나중에 **PC 2대로 분리, 원격제어**. + couple_kp=0이 공식 Mode2와 동성능(=oa_mit 토대 OK).
- **함의**: leader/follower가 물리적 다른 PC → 단일프로세스 양팔 CAN 불가 → **D28의 (B) 단일프로세스/enactic in-memory 방식 폐기**. leader↔follower는 **영구적 지연 네트워크 채널**(지금 100Hz 토픽=네트워크지연 예고편). **oa_mit 토픽 구조가 정답**(토픽→DDS 확장).
- **올바른 구조**: 양쪽 다 자기팔 로컬 고속루프(지연0)+상대정보만 지연. oa_mit가 이미 이 모양. 문제는 지연된 상대상태에 건 결합게인뿐.
- **재계획**: ①follower→leader 상태레이트↑(JSB 100Hz가 stale→진동 주범; 전용 고속 publisher) ②상대속도 댐핑+follower위치 LPF ③effort 반력(leader kp=0 유지=자유공간 투명, follower 접촉토크만 반사) ④네트워크 passivity(wave variable/scattering) — [[project-comm-benchmark-topology]] 지연축과 직결. 1번부터.

### D28. oa_mit FF 실기결과 = 토픽지연 한계, bilateral은 신선한 양팔상태 필요 (2026-06-20, cceca85)
- 실기: couple_kp=8 → 심한 진동. kp4/kd0.2 → unilateral보다 뻑뻑, 반력 약함, 막힌자리로 leader 되돌아오나 애매.
- 진단: oa_mit는 follower 상태를 **100Hz /joint_states 토픽(지연)** 으로 받아 결합 → ①지연 폐루프(왕복~20-30ms) 高게인서 진동, ②position 결합이 **자유공간 추종지연까지 반사** → 접촉 아닌데 뻑뻑, ③안전게인 낮아 반력 약함.
- **핵심**: enactic bilateral이 부드러운 건 **단일 프로세스서 양팔 상태를 in-memory로 신선하게** 읽기 때문(AdminThread가 leader/follower state 교환, 토픽 아님). oa_mit의 토픽 홉이 병목. unilateral(couple_kp 0)이 완벽했던 건 follower→leader 피드백 불필요해서.
- 함의: 좋은 bilateral FF엔 **신선한 follower 상태**가 필수. (A) oa_mit position-FF 튜닝(필터+상대댐핑, 한계 낮음, 토픽지연 잔존) vs (B) **단일프로세스 bilateral(양팔 CAN 직접, enactic 레시피: kp~50, kd/kp~0.05, 중력만, 교차결합, 마찰FF·속도FF 없음)**. oa_fd 아키텍처(단일프로세스 양팔)는 옳았고 레시피(kp120+마찰FF+속도FF+게이트)가 틀렸던 것 — B는 oa_fd 레시피 교정 or enactic bilateral_step 포팅.

### D27. ★공식 openarmx에 "weightless leader(놓으면 멈춤)"가 이미 있음 = Mode 2 (2026-06-20)
- 운영자가 쓴 `openarmx_teleop_bimanual/teleop_bimanual.launch.py` = **Mode 1(중력보상 없음)**: README 명시 "leader motors DISABLED, manual dragging" → leader 모터 꺼서 limp로 끌고 follower만 추종(그래서 초부드러움, 제어루프 없음). 놓으면 처짐.
- bringup의 `enable_forward_effort`(gravity_comp_node)는 **bringup=follower에만** 발행(노드가 절대토픽/조인트명 하드코딩, 로봇1대 전용) → "g_scale이 follower만 효과" 정확히 설명.
- **운영자가 원하는 "손으로 옮기고 놓으면 그 자리 유지" = README Mode 2** `teleop_bimanual_with_gravitycomp.launch.py` (이미 존재). 전용 per-arm 노드가 **leader를 raw-CAN으로 직접 구동**: MIT **kp=0, kd=0, torque=g_scale·gravity**(+옵션 damp/hold) → weightless float, leader 위치를 follower forward_position_controller로 릴레이.
  - 파라미터: g_scale 0.9/0.8(우/좌, **<1.0이 기본 — 약간 가볍게**), gdir Y축 (0,∓9.81,0), kp_hold=0(순수 중력 float, "멈춤"은 중력정확도+settle로), tau_limits{10,10,5,5,2,2,2}, 300Hz.
  - 필수: `/tmp/v10_bimanual.urdf` 존재(없으면 노드 죽음=무효과 함정). follower bringup(can2/3, forward_position_controller) 별도.
- **함의**: 기능 추가 불필요 — 공식에 있음. 우리 oa_fd FREEDRIVE(kp=0+중력) 레시피가 옳았음을 공식이 재확인. raw-CAN도 **kp=kd=0+중력토크만**이면 매끈(우리 떨림은 stiff 결합+마찰FF+속도FF 탓). → 다음: 운영자가 Mode 2 실행해 leader weightless 확인 → 이후 bilateral은 이 위에 힘반사만 얹기.

### D26. ★공식 openarmx ros2_control unilateral = 같은 HW서 완벽히 부드러움 → 문제는 우리 코드/접근 (2026-06-20)
- **운영자 실측**: 공식 패키지로 unilateral teleop 실행 — `openarmx_bringup/openarmx.bimanual.launch.py robot_controller:=forward_position_controller` + `openarmx_teleop_bimanual/teleop_bimanual.launch.py`.
  - **엄청 부드러움**(bilateral 아니라 그럴 수 있으나 그 이상으로 부드러움).
  - **follower q3~7 팡팡 튀는 관절 없음** (우리 cali/teleop선 q7 stick-slip 있었음).
  - **leader/follower×L/R 4팔 전부 동시 사용해도 이상 없음** (우리는 leader L/R grav+fric comp만 해도 이상현상 → 지금 leader 1개만 씀).
  - **하드웨어 완전 동일**(제어PC·로봇·USB2CAN 다 같음).
- **함의**: "4-CAN/USB 대역", "q7 모터 특성", "팡팡은 어쩔 수 없다" 같은 HW 핑계가 **전부 반증됨**. 공식 ros2_control 스택은 같은 HW서 4팔 멀쩡. → 우리 단일프로세스 raw-CAN + MIT 직접제어 접근에 문제가 있다는 뜻. **다음: openarmx_ros2 / openarmx_description ros2_control 내부 정독·논의**(왜 부드러운지: 컨트롤러 종류·업데이트율·CAN 읽기방식·MIT 파라미터·하드웨어 인터페이스 구조). 채택/차용 검토.
- **마찰게이트 해제(806bc15) 결과**: 느낌 "비슷"(로그 미수집). 즉 게이트 해제만으론 뻑뻑/부드러움 격차 해소 안 됨 → enactic 동일법으로도 부족 → ros2_control 스택과의 구조적 차이(아래 논의)가 본질일 가능성.

### D25. ★enactic bilateral엔 DOB 없음 — 우리와 동일 제어법, 차이는 마찰게이트 (2026-06-20, 806bc15)
- **실측(enactic/openarm_teleop 소스 직독)**: `control/control.cpp bilateral_step()` = `τ=Kp(peer−q)+Kd(peer_dq−dq)+gravity+friction(tanh)`. 교차결합도 우리와 동일(leader_ref=follower_resp, 반대도). **DOB·반력관측기·가속도제어 전부 없음**(DetectVibration()까지 있음=그쪽도 진동과 싸움). → "enactic이 DOB로 투명하다"는 가설 **기각**.
- **제어 관련 유일한 핵심차이**: enactic 마찰보상 = **amp1.0 항상 full, 속도게이트 없음**. 우리는 저속 차단 게이트(FREEDRIVE 폭주방지용). ACTIVE 결합에선 Kp 스프링이 안정화 → 게이트 불필요한데 **저속 teleop서 마찰보상 꺼져 follower 끌림 = 뻑뻑함**.
- **결정**: 게이트는 freedrive-like에서만, ACTIVE(both)/PAUSED/HOMING은 full 마찰보상(806bc15). 부차차이(기록만): enactic tanh `Fc·tanh(0.1·k·v)`, 500Hz×2스레드, Fo 유지.
- **CSV 진단(bi_a~d)**: 정지 comp 정확(tau≈FF), 떨림=Kp=120 결합 4Hz 한계진동(couple_kp_scale 노브 추가), 무게=follower 관성+마찰 끌림(레버 부족한 손목이 특히). [[project-openarm-a2-bilateral]]
- 다음: 게이트해제 + couple_kp_scale 스윕 실기 재시험. 그래도 부족하면 enactic Kp/Kd 값 확인 or DOB는 정공법이나 enactic도 안 쓰므로 우선순위 낮음.

### D24. cali "팡팡" = q7 stick-slip(코드/무게 무관), 끝단 무게는 q6로 (2026-06-20)
- friction_cali 팡팡 = **q7 stick-slip**. v0(eab5a3d)·현재·historical friction_right.csv **전부 동일**(q7 stick 59~70%, q2 0.2%) → 코드 회귀 아님. 원인: cali는 마찰보상 OFF + q7 Kp=12(약함) → 정지마찰 못 뚫고 멈췄다 튐. teleop은 마찰보상+결합으로 매끈(=cali 전용 아티팩트).
- **무게 문제 아님**: q7 중력≈0(COM이 롤축 위, 토크 ~0.05Nm)이라 g_ff(무게) 기여 ~0 → 무게 알아도 q7 구동 안 변함. leader(무게 앎)도 동일하게 팡팡 = 증명.
- **끝단(link7) 무게는 q7로 측정 불가**(COM이 축 위, stick-slip은 부차적). **q6로 측정**(link7을 레버로 휘둘러 신호 ~10×). leader/follower 비는 **q1/q2/q4**(매끈)로, q7 제외.
- rejected: cali 루프에 vel FF(=MIT kd·vel 킥→lurch, 되돌림). 상세: [CALI_NOTES.md](CALI_NOTES.md). 임시도구: oa_friction_cali_v0, cmd() 루프출력(~195Hz 정상).

### D23. ★follower-right "쭉 간다" = 댐핑 부재(과보상도 마찰도 아님) (2026-06-19, bcbf803)
- **두 번의 오진 정정**: (a) "마찰 추가하면 됨"(922f771) — 틀림, 마찰보상은 드래그를 줄여 오히려 미끄러짐↑. (b) "중력 과보상"(운영자 가설에 동조) — 틀림, 데이터상 과보상 아님.
- **결정적 증거 1 (q1 teleop 시계열)**: q1이 놓는 위치마다 정지·유지(+1.21/+1.07/−0.63/+0.48/+1.22). 과보상(negative stiffness)이면 중간서 발산해야 함 → 전구간 버팀 = 중력 정확. "쭉 가는" 구간 속도 −0.7~−1.0로 거의 일정·완만감속(↑성장 아님), **위로도(중력 거슬러) 동일하게 미끄러짐** → 무게 양방향 상쇄 = 정확.
- **결정적 증거 2 (미러 평가)**: 아까 본 오른쪽 j1 −1.85/j4 −1.46 바이어스는 **평가 아티팩트** — `fit_gravity --side right`는 q/τ를 미러 안 하는데(joint limit만) teleop은 `g=m⊙gravity(m⊙q)` 미러 적용. CSV를 q→m⊙q, τ→m⊙τ로 미러 후 평가하니 **j1 −0.08 / j4 −0.10 Nm (왼쪽 −0.05급)** = 양 팔 중력모델 깨끗. scale=1, vec정상 확인.
- **정체**: single-role/gravity-only 모드는 kp=kd=0 → 댐핑 0. 이 모터 고유마찰 극소(그래서 마찰보상값도 ~0.6Nm) → 무중력+무댐핑이라 한번 밀면 관성 coast. "조금 가다 멈춤"을 원하면 **모터측 Kd**(지연0, passivity-safe D-원리)가 정답. 실제 bilateral은 이미 Kd=2라 무관(진단모드 한정 현상).
- **적용**: follower_right freedrive `posture_kd` q1/q2 = 2.0 (kp=0 유지 → 순수 감쇠, 중력 영향 0). q4/q6/q7 미는반대힘은 미보상 마찰이라 마찰보상(D22) 유지가 맞음.
- 교훈: 폭주/미끄러짐을 "중력 vs 마찰"로 단정 말 것 — **위치마다 버티는지(중력) + 속도가 성장하는지(과보상) + 양방향 대칭인지(무게상쇄)** 시계열로 가르고, 미러팔 평가는 반드시 teleop과 동일 미러로.

### D22. follower-right 마찰 투입 (2026-06-19, 922f771) — D23으로 일부정정
- **friccomp OFF 상태 실기데이터(q1/q2/q4/q6/q7 CSV) 분석**: 저속(|v|<0.05) 구간 정적 hold 토크가 위치별 매끈+저분산(q1 −0.62→−8.3 / +1.2→+13.1, std 0.03~0.4; q2 0→0 / +1.35→+13.2). 운영자 "손 놓으면 버팀"과 일치 → **중력모델 정확(D21 결론 실측 재확인)**.
- **잔여증상 전부 마찰부재**: q1/q2 밀면폭주=댐핑0 coast, q2≈0 진동=가파른구간 댐핑부재, q4/q6/q7 "미는 반대힘"=미보상 모터마찰(comp 대상 그 자체), q6/q7 처짐=이 자세 중력≈0이라 마찰. → 절차대로 **마찰 점진투입** 단계 진입.
- **결정**: `friction_follower_right.csv`를 leader와 동일 `fit_friction.py`(x0.75)로 fit → leader와 **동일 SAFE 처리**: Fo→0; q4 Fc&Fv→0(점성off, Fv=0.824는 q4 8rad/s 폭주 재발); q2 k→2(anti-chatter); q2 게이트 v_full 0.40. yaml 반영(922f771).
- **q4 미결(의도적)**: SAFE 처리로 q4 comp=0 유지 → q4는 여전히 약간 끌림(leader와 동일, 설계상). 점성(Fv) 폭주위험과 무관한 **q4 Coulomb-only(Fc=0.076, Fv=0)** 는 게이트로 유계라 안전 → 운영자가 q4 더 가볍길 원하면 후속 추가 가능(rejected 아님, 보류).

### D20. follower 중력모델 = leader 모델 (상세)
- **★확정 (84bad14): follower 중력모델 = leader 모델.** 데이터 직접검증: 실제 그리퍼 중력기여(follower측정−leader모델)가 노이즈 수준(j1 +0.05±0.63, j4 +0.14±0.27Nm)이고, **leader 모델이 follower 측정을 잔차 0.237로 설명**(follower 자체fit 0.218과 거의 동일). 즉 그리퍼는 중력에 거의 영향 없음 → follower 실제중력≈leader. follower **gripper-fit이 외삽서 없는 토크를 만들어 과보상→q1/q4 폭주**. leader 모델은 follower도 잘 맞추면서 안정. → follower URDF=leader URDF 복사. 재cali 불필요. (per-role URDF 기능[D19]은 유지하되, 현재 follower URDF는 leader와 동일 내용.)
