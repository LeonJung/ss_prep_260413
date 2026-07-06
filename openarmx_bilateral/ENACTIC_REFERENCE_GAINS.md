# Enactic OpenArm bilateral — reference gains (per joint)

Source: `~/ext_ref/openarm_teleop/config/{leader,follower}.yaml` (Enactic, Inc. 2025, Apache-2.0).
Enactic's official bilateral teleop config. Comment in-file: **"Kp/Kd are shared between
bilateral and unilateral control"** → these ARE the bilateral force-feedback gains.
**Leader and follower are identical** (only diff: follower Fc[J6]=0.093 vs leader 0.083).

Joint order = J1..J7 (arm) + J8 (gripper).

| | J1 | J2 | J3 | J4 | J5 | J6 | J7 | J8 (grip) |
|---|---|---|---|---|---|---|---|---|
| **Kp** | 240 | 240 | 240 | 240 | 24 | 31 | 25 | 16 |
| **Kd** | 3.0 | 3.0 | 3.0 | 3.0 | 0.2 | 0.2 | 0.2 | 0.2 |
| Fc (L) | 0.306 | 0.306 | 0.40 | 0.166 | 0.050 | 0.083 | 0.172 | 0.0512 |
| Fc (F) | 0.306 | 0.306 | 0.40 | 0.166 | 0.050 | **0.093** | 0.172 | 0.0512 |
| k | 28.417 | 28.417 | 29.065 | 130.038 | 151.771 | 242.287 | 7.888 | 4.000 |
| Fv | 0.063 | 0.063 | 0.604 | 0.813 | 0.029 | 0.072 | 0.084 | 0.084 |
| Fo | 0.088 | 0.088 | 0.008 | -0.058 | 0.005 | 0.009 | -0.059 | -0.050 |

Friction model (theirs): `τ_fric(ω) = Fo + Fv·ω + Fc·tanh(k·ω)`
(NB: no 0.1 factor inside tanh, unlike our `Fc·tanh(0.1·k·ω)` — so their k ≈ 10× ours in scale.
Also they include a viscous term Fv and offset Fo; we identified Fv≈0 and used Coulomb only.)

## ⚠️ Scale 비교 — enactic(DM) vs 우리(Robstride). 부분적으로만 같음
kp는 MIT 프레임에 `[kpMin,kpMax]`로 packing되어 모터에서 그대로 round-trip → **kp 값 = 실제 강성
Nm/rad**(범위는 표현 최대치만 정함). 우리 v10 관절별 모터타입(v10_simple_hardware.hpp)과 범위
(openarmx-can rs_motor_constants.hpp MOTION_CONTROL_LIMITS):
| 관절 | 우리 모터 | 우리 KP/KD 범위 | enactic(DM) 범위 | scale |
|---|---|---|---|---|
| J1–J2 | RS04(=DM8009) | **KP[0,5000] KD[0,100]** | KP[0,500] KD[0,5] | 범위 10×/20× |
| J3–J4 | RS03(=DM4340) | **KP[0,5000] KD[0,100]** | KP[0,500] KD[0,5] | 범위 10×/20× |
| J5–J7,그리퍼 | RS00(=DM4310) | KP[0,500] KD[0,5] | KP[0,500] KD[0,5] | **동일** |
- **RS00(손목/그리퍼): enactic과 범위 동일 → Kp[24,31,25,16]/Kd0.2 그대로 이식 가능(高신뢰).**
- **RS04/RS03(어깨/팔꿈치): 범위 10배 넓음** = 더 센 모터라 최대 강성 5000까지 가능. 240 Nm/rad은
  물리적으로 같지만 우리는 더 세게도 감. **240을 시작점으로 100→160→240 점진 상향(펌웨어 kp 컨벤션이
  10배 다를 가능성 배제 못하니 급투입 금지).**
- **현재 우리 relay는 leader_kp를 전 관절 UNIFORM(~10-60)로 줌** → 어깨(enactic 240 대비 ~1/4)가
  물러서 "벽 아닌 스프링" 느낌의 원인으로 추정. **관절별 kp 차등 도입이 벽느낌의 핵심.** (launch에
  관절별 배열 인자 추가 필요 — 현재 미구현.)

## 배움 / 우리 세팅과의 차이
- **Kp가 관절별로 크게 차등**: 근위 4관절(어깨·팔꿈치) **240**, 손목 3관절 **24~31**, 그리퍼 **16**.
  → 어깨는 딱딱한 벽(고 Kp), 손목은 가볍게(저 Kp). "벽 느낌 + 손목 가벼움"의 핵심으로 보임.
- **Kd도 근위 3.0 / 원위 0.2**로 차등.
- 우리(bilateral.launch.py)는 현재 `leader_kp`를 **전 관절 일괄**로 줌 → enactic처럼 **관절별 차등**
  (근위↑/원위↓)이 transparency("벽인데 안 뻑뻑")에 더 맞을 수 있음. 벽 느낌 튜닝 시 이 프로파일 참고.
- 이 Kp/Kd는 HW MIT 커플링 게인 스케일(우리 kp_joint1..8 서비스와 같은 자리)로 직접 대응됨.
