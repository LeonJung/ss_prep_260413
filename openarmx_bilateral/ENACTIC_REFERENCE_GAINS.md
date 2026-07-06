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

## 배움 / 우리 세팅과의 차이
- **Kp가 관절별로 크게 차등**: 근위 4관절(어깨·팔꿈치) **240**, 손목 3관절 **24~31**, 그리퍼 **16**.
  → 어깨는 딱딱한 벽(고 Kp), 손목은 가볍게(저 Kp). "벽 느낌 + 손목 가벼움"의 핵심으로 보임.
- **Kd도 근위 3.0 / 원위 0.2**로 차등.
- 우리(bilateral.launch.py)는 현재 `leader_kp`를 **전 관절 일괄**로 줌 → enactic처럼 **관절별 차등**
  (근위↑/원위↓)이 transparency("벽인데 안 뻑뻑")에 더 맞을 수 있음. 벽 느낌 튜닝 시 이 프로파일 참고.
- 이 Kp/Kd는 HW MIT 커플링 게인 스케일(우리 kp_joint1..8 서비스와 같은 자리)로 직접 대응됨.
