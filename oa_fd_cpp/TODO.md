# oa_fd_cpp — TODO / open investigations

## q2 motor torque ripple (open, parked 2026-06-13)
**Symptom**: q2 (and likely all joints) has a regular, position-periodic
"notch / stone-root" resistance while hand-moving in FREEDRIVE; the body
shakes each time you cross a notch.

**Diagnosis so far (HW + CSV, left leader)**:
- Position-periodic, NOT time-periodic (operator confirmed).
- GONE when motors are torque-free / not controlled (backdriven by hand) ->
  it is NOT mechanical cogging or gear detent (those persist passively).
- Persists with friction comp OFF (Fc=0) but gravity comp ON -> it is NOT
  friction-comp stick-slip.
- => It is the commanded torque rippling with rotor angle = **motor torque
  ripple**: the RS04 delivers a commanded torque (here gravity comp) unevenly
  vs electrical angle. Only visible under commanded current.
- Model accuracy (mass/COM) is NOT the cause.

**Possible fix (if worth it)**: position-indexed feed-forward ripple map.
Command a slow constant velocity, log effort vs motor position, extract the
periodic component (check spatial period == RS04 electrical period / pole
pairs, and that it is repeatable pass-to-pass), then add -ripple(q) on top of
g(q). Not present in enactic. Needs /tmp/q2_nofric_slow.csv (effort vs
position) to confirm period/amplitude/repeatability first.

**Could also be a motor unit issue** — worth comparing the same joint on
another arm before investing in a ripple map.

## Per-role URDF (Task #5)
leader tip = light handle, follower tip = gripper -> separate gravity models.
First attempt (77b2033) caused a runaway and was reverted; reintroduce only
with a verifiable, staged HW test. See memory project file.

## Friction comp safe reintroduction (Task #7) — largely done
Gated, magnitude < real friction; per-joint velocity gate (q2 widened to
v_full 0.40 to kill the 1-3 Hz limit cycle). q4 viscous comp kept OFF.
