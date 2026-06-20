# Cali / "팡팡" / end-link mass — investigation notes (2026-06-20)

Scope: oa_friction_cali / oa_gravity_cali smoothness ("팡팡"), and whether it
blocks measuring the END link (URDF `openarmx_link7`, operator calls it "link8")
mass and the leader↔follower per-joint output ratio.

## TL;DR
- **friction_cali "팡팡" = q7 stick-slip.** NOT a code regression, NOT a mass
  problem. Identical in v0 (eab5a3d), current, and the historical
  `friction_right.csv`.
- **End link mass is NOT measurable via q7 gravity:** q7's COM sits ~on its roll
  axis (perp offset ~8.7 mm) → gravity torque ≈ 0.05 Nm, below the friction/noise
  floor. Stick-slip makes it worse but is NOT the limiting factor — geometry is.
- **Measure end mass via q6** (wrist pitch swings link7 on a real lever, gravity
  signal ~0.5 Nm ≈ 10× q7). Measure leader/follower ratios on **q1/q2/q4**
  (smooth, large signal). **Skip q7.**
- q7 팡팡 is a **cali-only artifact** (friction-comp OFF + weak Kp). In teleop,
  friction comp + bilateral coupling smooth q7 out.

## Evidence — stick fraction (|dq|<0.05 during the sweep)
| joint | v0 (eab5a3d) | current | historical friction_right.csv | verdict |
|------|----|----|----|----|
| q2 | 0.2% | 0.4% | 0.3% | smooth (all identical) |
| q7 | 65%  | 70%  | 59%  | stick-slip (all identical) |

dq std: q2 ~0.3 (constant velocity) vs q7 ~1.1 (bimodal: stuck→spike).

## Why it's stick-slip, not mass
- cali runs **friction comp OFF** (it is *measuring* friction) → q7 stiction is
  exposed. q7 **Kp=12** (very weak): the spring can't pull q7 smoothly through
  its own stiction → position error builds until it breaks away → slip → stick.
- q2 is smooth because **Kp=110 + large gravity drive** overpower stiction.
- **Mass is irrelevant for q7:** q7 gravity ≈ 0 (COM on axis), so the gravity-FF
  term `g_ff(mass)` is ~0 for q7 *regardless of mass*. The **leader (mass known)
  stick-slips identically** to the follower → proves it is not a mass-knowledge
  issue.

## End-link (link7) mass — how to actually get it
- q7 gravity is ~0 (COM on axis) → useless for mass, with or without stick-slip.
- Use **q6**: gravity torque on q6 ∝ m7 × lever; follower/leader amplitude ratio
  → end-mass multiple. (q6 has bigger gravity drive → less stick-slip; verify.)
- Caveat: q6 mixes link7-mass and q6 motor-output; assume one to isolate the
  other (see the leader↔follower ratio plan).

## Leader ↔ follower per-joint output-ratio plan (current approach)
- Sweep one joint at a time, **leader then follower SEQUENTIALLY** (never both —
  single-USB/4-CAN bandwidth). Tool: `oa_friction_cali --joint N`.
- Use **±both directions** so +v/−v averaging cancels friction → pure gravity.
- Joints: **q1/q2/q4** (smooth, big signal) for the ratio; **q6** for end mass;
  **skip q7**.
- Estimator: `script/tmp_lf_motor_ratio.py` (follower/leader gravity-torque ratio
  + per-arm meas/model slope).

## Rejected / do-not-retry
- **Velocity feedforward in the cali move/sweep** (my attempt): MIT `kd·vel` term
  becomes an instantaneous forward kick (kd 3.5 × vel 0.9 ≈ 3 Nm) → the arm
  "lurches/teleports". Reverted. Do not re-add vel FF to the cali loops.
- **Trapezoid sweep / base-pose / position-clamp** as the 팡팡 cause: ruled out
  (operator + data). The cali motion code (Kp/Kd/SPEEDS/dt) is byte-identical to
  the smooth era; the loop runs steady ~195 Hz (healthy, sleep(4ms)+work).

## Temporary tools (keep for now; delete when done)
- `oa_friction_cali_v0` — eab5a3d baseline executable for A/B (src
  `friction_cali_v0_main.cpp`).
- `cmd()` loop-rate print inside `oa_friction_cali` — measured ~195 Hz steady.
