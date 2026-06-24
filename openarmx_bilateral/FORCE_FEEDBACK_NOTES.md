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
