# oa_fd_py

**MuJoCo validation of the oa_fd bilateral law (OpenArm A2 / openarmx, bimanual 7-DOF).**

`oa_fd_cpp`를 실물에 올리기 전에, 같은 제어 법칙을 **openarmx 공식 MJCF 플랜트**
(openarmx_mujoco 의 control runtime — 실제 관성·joint range·damping 18·frictionloss
포함)에서 검증한다. `ur10e_teleop_mujoco` 와 같은 패턴 (leader/follower 두 MuJoCo
인스턴스 + shm 결합).

## 검증된 법칙 (oa_fd_cpp 와 동일 형태)

```
tau = Kp*(q_ref - q) + Kd*(dq_ref - dq) + g(q)        (per joint, clip ±tau_max)
  ACTIVE   : q_ref = peer 상태 (교차 결합)
  PAUSED   : 진입 시점 자세 hold
  HOMING   : quintic ramp -> HOME
  FREEDRIVE: Kp=Kd=0 (중력만)
```
g(q) = MuJoCo `qfrc_bias` (정확한 모델) — 법칙 자체를 모델오차와 분리해 검증.

## 결과 (2026-06-11, headless `tests/test_oa_fd.py` — ALL 5 PASSED)

| phase | 결과 |
|---|---|
| 1 gravity-hold (FREEDRIVE, 중력 부하 자세 3s) | drift **0.000 rad** |
| 2 paused-hold | error 0.0000 rad |
| 3 homing (quintic 2s) | settle error 0.001 rad |
| 4 tracking (leader 에 25 Nm 외력) | leader-follower gap **0.0019 rad** |
| 5 force-reflect (follower 를 40 Nm 으로 막음) | gap 0.167 rad → 반사토크 ~20 Nm |

## 실물 대비 확정된 사실 (MJCF ground truth)

- **joint1 은 중력 받는 pitch 축** (world X). j1=-1.53 에서 g[0]=-17.6 Nm.
  (oa_fd_cpp 의 URDF joint1 축 수정 방향이 옳았음을 모델이 확인)
- **오른팔 j1/j2 의 joint range 는 왼팔과 미러** (예: j2 left [-3.27,0.13] vs
  right [-0.13,3.27]) → **HOME/자세는 팔별 부호 필요**. real config 의
  `mirror` 가 바로 이 것.
- joint damping=18, frictionloss=0.15 — 실물 oa_fd 의 "뻑뻑함" 일부는 이 큰
  damping (마찰보상 FF 가 꺼져있으면 당연히 무겁게 느껴짐).
- 모터 토크 한계 = ctrlrange = RS04 120 / RS03 60 / RS00 14 Nm.

## 실행

```bash
# headless 검증 (디스플레이 불필요)
python3 tests/test_oa_fd.py

# GUI: leader/follower 창 2개 (shm 결합)
bash script/exec_sim.sh
#   SPACE=ACTIVE  P=PAUSED  H=HOMING  F=FREEDRIVE
#   leader 창에서 Ctrl+drag(더블클릭으로 body 선택 후) = 사람 조작 대용
```

## 파일

```
xml/oa_bimanual.xml(+_runtime)  openarmx MJCF 정리본 (mesh 제거·capsule 시각화·
                                position→motor 토크 액추에이터). script/build_model.py 로 재생성.
src/oa_fd_sim.py                sim 코어(OaFdSim) + viewer 프로세스(--role leader|follower)
tests/test_oa_fd.py             5-phase headless 검증
```

## 다음 단계
1. (선택) 모델오차 주입 버전 — g(q) 를 qfrc_bias 대신 URDF/KDL 로 계산해
   실물처럼 모델 불일치 상태의 강건성 확인
2. 검증된 게인/구조를 `oa_fd_cpp` 로 역반영 → 실물 재시도
3. `oa_pp_py` — 같은 플랜트에서 pos-pos(deadband) 법칙 검증
