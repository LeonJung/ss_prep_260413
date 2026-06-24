# Bilateral Force-Feedback — Design Discussion Notes

Running notes for tuning the bilateral force feedback (Final Phase). See STATUS.md
for the build/run/commands; this file is the *reasoning & decisions* log.

## Goal (operator's target feel)
Reference: MIT Sangbae Kim lab 3-DOF leader–follower demo the operator has used —
- **Follower blocked (wall/object) → leader becomes ~immovable** (push hard, barely
  moves) = a true "hard wall", controllability→0 at contact.
- **No contact (normal) → very smooth & fast**, near-effortless.

Current v2.0 (leader_kp≈60 uniform, kd 0.5, P1+P2+P3 on) gives, per operator:
1. **Too stiff (뻑뻑)** — wrist-fatiguing over long use.
2. **Contact feels like a SPRING, not a hard wall.**

## Key finding 1 — our method == enactic == position-position (P-P)
Verified in enactic source (openarm_teleop control.cpp + AdminThread):
```
each arm:  τ = Kp·(peer_pos − own_pos) + Kd·(peer_vel − own_vel) + (gravity + friction)
```
AdminThread sets each arm's reference = the *other* arm's response (peer). This is
identical in structure to ours: relay feeds peer position (+peer velocity via
vel_ff) into the MIT controller, with gravity+friction as torque feedforward.
There is **no wall-detection / if-logic** in enactic. → Reflected force is
`Kp × position-error` = a **spring**. A "wall" is just a *very stiff spring*.

enactic config (Damiao): **symmetric** leader=follower, per-joint
`Kp=[240,240,240,240, 24,31,25, 16]`, `Kd=[3×4, 0.2×4]`, full friction (incl Fv).
So enactic's wall = high shoulder Kp (240, ~4× ours) + per-joint + friction comp.

## Key finding 2 — "뻑뻑" is NOT caused by high Kp
`reflected force = Kp × (leader↔follower tracking error)`.
- **Free space, perfect tracking → error≈0 → force≈0 regardless of Kp.** High Kp is
  "invisible" in free motion and only "appears" at contact (where error spikes).
- enactic is smooth at Kp=240 **because its free-space tracking error ≈ 0**:
  friction comp (follower not sticky) + velocity FF + gravity comp + light hardware
  + fast loop (~500Hz–1kHz) keep follower glued to leader.
- **Our 뻑뻑 at only Kp=60 ⇒ our free-space tracking error is large** ⇒ follower
  tracking fidelity is the bottleneck, NOT the gain.

Likely error sources for us: friction comp incomplete (scale 0.7, Fv dropped,
fit R²≈0.4–0.85), follower Kp at HW default (loose), relay 200Hz, Robstride inertia.

## Reframed direction (consensus so far)
Not "lower Kp to feel light." Instead **raise follower tracking fidelity so
free-space error→0**, which:
- lightens free motion (뻑뻑 fix), AND
- lets us raise Kp → hard wall (spring fix),
simultaneously — the same thing that makes enactic both light and wall-like.

Objective metric proposed: **log free-space leader↔follower position error**; if
large, that IS the 뻑뻑. Improve comp → error shrinks → lighter.

## Levers available (within P-P; DOB / 4-channel / wave are BANNED)
- Per-joint Kp (shoulder↑ for wall+tracking, wrist↓ to de-stiffen the grip).
- Follower stiffness↑ (rigid against wall = sharper wall onset + tighter tracking).
- Friction comp completeness↑ (scale→1, add Fv, better fit) = less follower lag.
- Velocity FF / filtering, higher relay rate.
- Symmetric (enactic) vs asymmetric (follower stiffer than leader) — open question.

## Honest ceiling
Pure P-P (2-channel position-position) has a transparency limit: with finite,
stable gains you approach but never reach a *rigid* wall — it stays a *very stiff
spring*. Breaking that ceiling usually needs force-sensing / DOB / 4-channel, which
are banned here (failed UR10e approaches). So target = "much stiffer wall + much
lighter," not literally infinite stiffness.

## TODO / open
- Review MIT Sangbae Kim lab papers (operator to provide) for techniques applicable
  to our P-P + ros2_control MIT + Robstride setup; identify which parts are
  adoptable and where our control algorithm would need partial modification.
  NOTE: the Sangbae Kim 3-DOF demo reportedly gives exactly the target feel
  (hard wall at contact, very smooth/fast free motion) — understand *how*.
- Decide symmetric vs asymmetric gains; where the 뻑뻑 dominates (wrist vs whole-arm,
  motion vs static).

## MIT Sangbae Kim lab papers — applicability (analysis only, no dev)
Common root = **proprioceptive / quasi-direct-drive (QDD) actuation**: low-gear,
high-torque-density, low-inertia, backdrivable motors; force read from **motor
current** (no force sensor, no DOB). The "hard wall + smooth free motion" is mostly
a HARDWARE (actuator transparency) + BANDWIDTH result; control is simple
impedance / position-position. NOT a special algorithm, NOT DOB → no clash with bans.

Operator note: saw the demo **2019–2020**, so 2025/26 inertia/Coriolis FF was NOT in
it. ⇒ the "perfection" came from QDD transparency + high bandwidth + simple P-P,
**without advanced model compensation.** So inertia/Coriolis FF is LOW priority for us.

Per paper: 2012 ICRA / 2017 T-RO = actuator hardware design (can't adopt; confirms
principle; our Robstride = same QDD family). 2022 IROS = the closest platform to the
demo. 2025/26 telepresence (human-arm dynamics) = leader inertia comp (low priority
per above). Ramos&Kim whole-body = humanoid balance (not arm-FF; skip).

### 2022 IROS platform vs ours (v2.0) — gap table
Specs (2022): 2× QDD arms, **6-DOF**, reach 0.96m, 13.8kg; controlled over **SPI at
500Hz** by an i7 SBC; custom PCB → **CAN up to 3kHz** per actuator; proprioceptive
(current) force feedback.

| 구분 | MIT 2022 (Kim lab) | 우리 openarmx v2.0 | 영향 |
|---|---|---|---|
| **HW 액추에이터** | QDD BLDC + 저감속 유성기어, 고투명·저마찰·저관성 | Robstride QDD(CAN) — 같은 계열이나 마찰 큼(어깨 Coulomb~1Nm 측정) | 투명도 격차 → SW(마찰보상)로 회복 |
| **HW DOF/질량** | 6-DOF, 13.8kg 경량 | 7-DOF bimanual, 더 무겁/관성↑ | lag↑ |
| **힘 센싱** | proprioceptive(모터 전류) | proprioceptive(MIT모드 전류) | **동일** |
| **통신-모터버스** | 커스텀 PCB, CAN **최대 3kHz** | CAN(ros2_control HW IF) | 갱신주기 격차 가능 |
| **통신-토폴로지** | i7 SBC→SPI→PCB→CAN, 직결·최소 홉 | ROS2 DDS 토픽 다단(relay→pos ctrl→HW; grav→grav_only→fric→eff ctrl) | 지연·지터↑, 투명도↓ |
| **실시간성** | 거의 RT, 결정론적 | 일반 Linux+DDS, 비RT 지터 | 안정 Kp 상한↓, 벽 무름 |
| **제어주기** | 메인 **500Hz** + CAN 3kHz | controller_manager **100Hz** + relay 200Hz | **5배+ 느림 = 가장 큰 격차 (벽 선명도·추종 직접)** |
| **양방향 제어법** | P-P/임피던스 + 전류토크 | P-P(relay)+MIT kp/kd+중력/마찰/속도FF | **알고리즘 사실상 동급 → 격차 아님** |
| **모델 보상** | (2019-20) 단순, 액추에이터로 충분 | 중력+마찰+속도FF | 우리는 HW투명도 부족분을 SW로 메우는 중 |
| **SW 스택** | 베어메탈/RT 펌웨어+SBC | ROS2 Jazzy, 노드/DDS, colcon | 추상화·홉 오버헤드 |

### 결론 (격차 우선순위)
알고리즘은 같다. 격차는 ① **제어 대역폭/주기**(100·200Hz vs 500Hz+3kHz), ② **통신
지연·결정론성**(ROS2 DDS 다단 홉 vs RT 직결), ③ **HW 투명도**(Robstride 마찰 vs QDD).
2019-20 데모가 inertia/Coriolis FF 없이 완벽했으므로 → 우리도 그 방향(대역폭↑·지연↓·
마찰보상으로 투명도 회복·어깨 Kp↑)이 정답이고, 고급 모델보상은 후순위.
