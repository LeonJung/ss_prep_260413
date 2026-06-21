# OpenArm bilateral — journey + why it's hard vs UR10e (2026-06-21)

Consolidated notes for the bilateral force-feedback effort on OpenArm (oa_mit),
and the deep comparison against the working UR10e bilateral stack.

## Journey so far (DECISIONS D26–D29)
- Official openarmx **unilateral is super smooth** on the same HW (D26): trusts
  the motor's onboard MIT loop, well-damped gains (kp50/kd2.5), torque=0.
- enactic bilateral has **no DOB** — same position-position law as us (D25).
- Operator wants "move-and-release-stays" = openarmx **Mode 2** (weightless
  gravity-comp leader: MIT kp=0,kd=0,torque=g·gravity); it already exists (D27).
- **oa_mit** = Mode 2 leader + a force-reflection coupling. couple_kp=0 ==
  Mode 2 (verified identical). 
- **oa_mit FF result (D28)**: couple_kp=8 → strong vibration; kp4 → stiff +
  weak reflection. Root cause: follower state arrives via **/joint_states at
  ~74 Hz** (CAN-bound, stale) and we feed a naive position spring on it →
  delayed-loop vibration + reflects free-space tracking lag (stiff).
- **2-PC remote goal (D29)**: the two bimanual robots will be split across two
  PCs. So leader↔follower is a **permanent delayed network channel**; single-
  process both-arm control is impossible. oa_mit's topic architecture is the
  right shape; the problem is the *control law over a delayed channel*.

## The deep question: why did UR10e bilateral work and OpenArm doesn't?

The UR10e package `ur10e_teleop_control_hybrid_cpp` is a **full SOTA bilateral
stack**, HW-verified on UR3e(leader)+UR10e(follower):

| component | what it does | oa_mit has it? |
|---|---|---|
| **DynamicsModel** M(q),C(q,q̇),g(q) (KDL) | model-based feedforward | only g(q) |
| **VelocityEstimator** (LPF on diff q) | clean q̇ for damping/DOB | no (raw motor vel) |
| **DisturbanceObserver** (Q-filter, grav-aware) | estimate τ̂_ext **sensorlessly** → transparency | **NO** |
| **FourChannelController** (Lawrence) | Kp·e_pos + Kd·e_vel + Kf_self·τ̂_ext + Kf_peer·τ̂_ext_peer | **NO** (just Kp·e_pos) |
| **EnergyTank** (two-layer passivity) | guarantee passivity → **stable under delay** | **NO** |
| 500 Hz PREEMPT_RT loop, leader/follower over ROS topics | networked, deterministic | 300 Hz, topics |

Refs the UR10e stack implements: Buamanee 2025 (sensorless 4CH via accurate
dynamics, arXiv:2507.06174); Franken/Stramigioli 2011 + Minelli 2023–24
(two-layer energy tank).

### Why those pieces matter — and map exactly onto our two symptoms
1. **Stiffness (뻑뻑)** ← we reflect *position lag*. The 4CH+**DOB** reflects the
   estimated *external* torque τ̂_ext instead: cancels the partner's own
   inertia/friction → you feel the **environment**, not the robot. → transparent.
   Our naive Kp·(q_follower−q_leader) reflects the free-space tracking lag too →
   stiff even with no contact.
2. **Vibration** ← naive force feedback over a delayed/sampled channel is
   **non-passive** (injects energy) → limit cycle. The **EnergyTank** enforces
   passivity (active torque can only spend energy the tank actually has, refilled
   from real dissipation) → **provably stable under delay/discretization**. This
   is the missing anti-vibration mechanism. oa_mit has nothing → it vibrates the
   moment the gain is useful.

### The hardware angle (makes OpenArm HARDER, not easier)
- **UR10e**: high-ratio harmonic drives → high friction/damping, non-
  backdrivable. Lots of natural energy dissipation → forgiving; masks marginal
  instability. (Even so, the team added an energy tank.)
- **OpenArm/Robstride**: low-ratio quasi-direct-drive → low friction,
  backdrivable. **Little natural dissipation** → the delayed-feedback energy has
  nothing to absorb it → oscillates readily. The very property that makes it a
  great transparent leader makes it *less forgiving* for naive bilateral.
- ⇒ OpenArm needs the passivity/DOB machinery **even more** than UR10e did.

### Communication
- Both use leader/follower over ROS topics (networked-ready). UR10e runs 500 Hz
  RT with clean RTDE state; our follower state is ~74 Hz (CAN-bound JSB). Lower
  rate + more delay = even more need for passivity handling.

## Conclusion / recommended path
The naive position-position oa_mit was always going to be stiff + vibrate: it
lacks **DOB (transparency)** and an **energy tank (delay-passivity)** — the two
things that made UR10e work — and OpenArm's low friction makes it less forgiving.

**Port the UR10e hybrid control core to OpenArm**, not reinvent:
- Reuse `DynamicsModel / VelocityEstimator / DisturbanceObserver /
  FourChannelController / EnergyTank` (+ state machine, RT, leader/follower topic
  comms) from `ur10e_teleop_control_hybrid_cpp`.
- Replace the UR I/O layer (urcl/RTDE torque write) with **openarmx_can MIT
  torque** (MIT kp=0,kd=0,torque=computed) — exactly what oa_mit's leader already
  sends. Follower likewise becomes MIT-torque (not forward_position_controller).
- Per-arm node runs on its own PC reading its own arm fresh; peer state over
  topics; energy tank absorbs the network delay → fits the 2-PC remote goal.
- Needs a good OpenArm dynamics URDF (M,C,g): we have cali gravity URDFs; verify
  M(q)/C for the DOB.

Caveats (from the UR10e README, set expectations): DOB contact reflection was
"weak under URDF model" (accuracy-limited); "apparent inertia < physical
inertia" is a hard floor; the EnergyTank was coded+selftested but **HW-untested**
on UR10e — we'd be first to HW-validate it.

Interim (if we keep tuning oa_mit while deciding): `couple_lpf` (follower-pos
EMA) to damp the 74 Hz staleness vibration; effort reflection (leader kp=0 +
reflect follower contact torque) for free-space transparency — but these are
band-aids vs the full port.
