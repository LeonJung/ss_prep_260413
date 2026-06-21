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

## enactic OpenArm bilateral — what it ACTUALLY is (source read 2026-06-21)

Read `enactic/openarm_teleop` `src/controller/control.cpp` (`bilateral_step`),
`control.hpp`, `diff.hpp`, `dynamics.cpp`. The **active** bilateral law:
```
τ = Kp·(q_peer − q) + Kd·(q̇_peer − q̇) + gravity(q) + friction(q̇)      [MIT mode]
friction = Fc·tanh(0.1·k·q̇) + Fv·q̇ + Fo          (full, no velocity gate)
```
- velocity = **motor-reported** `get_velocity()` (not a differentiator).
- peer state = exchanged **in-process** (AdminThread copies leader↔follower
  state in shared memory every tick — NOT over ROS topics).
- two arm threads ~500 Hz + the Damiao motor's onboard MIT loop.
- **No DOB, no energy tank, no 4-channel, no oblique-coords transform, no active
  vibration suppression in the loop.** Force feedback is *emergent* from the
  symmetric position spring (follower blocked → q_peer−q grows → leader feels it).

Latent-but-UNUSED code (present, never called in bilateral_step — grep-verified):
`Differentiate_w_obs` (a DOB-ish velocity obs: `acc=LPF(τ/m); v=LPF(Δq)+acc`),
`oblique_coordinates_force/position` (modal/4ch hint), `DetectVibration`
(velocity-stddev monitor, threshold 0.7 rad/s — diagnostic only, no action).
⇒ enactic *explored* these but ships **plain symmetric position-position**.

### 3-way comparison
| aspect | UR10e hybrid (HW-verified, networked) | enactic OpenArm (smooth video) | oa_mit (vibrates) |
|---|---|---|---|
| core law | 4-channel Lawrence + DOB | symmetric pos-pos + g + friction | symmetric pos-pos + g (couple_kp) |
| transparency | **DOB (τ̂_ext)** | none (light arm masks it) | none |
| delay passivity | **EnergyTank (2-layer)** | none | none |
| **peer state** | topics (delay) **+ tank** | **in-process, FRESH** | **topic ~74 Hz, DELAYED** |
| velocity | LPF estimator | motor-reported | motor-reported |
| force reflect | explicit Kf·τ̂_ext both ways | emergent (pos spring) | emergent (pos spring) |
| rate | 500 Hz RT | ~500 Hz/arm + motor loop | 300 Hz / 74 Hz follower |

### The decisive takeaways
1. **enactic ≈ oa_mit in ALGORITHM** (both naive symmetric pos-pos + gravity).
   The ONLY material difference is **fresh in-process peer state (enactic) vs
   delayed 74 Hz topic peer state (oa_mit)**. → It's the *delayed channel*, not
   the control law, that makes ours vibrate where enactic's is smooth. Confirms
   D28/D29.
2. **enactic's approach is fundamentally NOT delay-tolerant** — it has zero
   passivity machinery; it only works because the peer state is fresh (one PC,
   shared memory). Put that same law on a delayed/networked link and it
   vibrates exactly like oa_mit. So **enactic is NOT a valid model for the 2-PC
   remote goal.**
3. **UR10e's DOB + EnergyTank IS built for the delayed/networked case** (tank =
   delay passivity, DOB = transparency) → the right model for 2-PC remote.

### Staged option this opens up
- **Single-PC bilateral (intermediate, fast win)**: port enactic's simple
  `bilateral_step` (both arms one process, in-memory fresh state) → likely
  smooth like the video, validates OpenArm HW + gives a baseline. Does NOT meet
  the remote goal.
- **2-PC remote bilateral (the goal)**: port the UR10e DOB + EnergyTank stack
  (delay-passive + transparent). Harder, but the only one that survives the link.

## Dead-code verification (rigorous, 2026-06-21)
Traced the call path, not just grep. `openarm_bilateral_control.cpp`: leader &
follower threads call ONLY `bilateral_step()` in `while(keep_running)`
(AdjustPosition once at startup). Across all enactic source on hand:
`oblique_coordinates_*` = declared only, 0 assigns/reads; `Differentiate_w_obs`
= defined only, 0 calls; `DetectVibration` = defined only, 0 calls;
`differentiator_` = constructed/destructed but `Differentiate()`/`_w_obs()`
never called → the object is unused; velocity = `motor.get_velocity()` direct.
**Confirmed: enactic's active bilateral is plain symmetric pos-pos + gravity +
tanh-friction over MIT; every "advanced" piece is dead code.**

## OUR direction — exploit the MIT motor loop (don't blindly copy UR10e)

### The asset UR10e did NOT have: a motor-side MIT impedance loop
- UR10e takes torque (or servoJ); the team ran the ENTIRE controller (incl. the
  Kp·e+Kd·ė position/velocity stiffness) **PC-side at 500 Hz** → all feedback is
  PC-side, so it had to add an **energy tank** to make that PC-side, delayed
  feedback passive. They had no choice — no usable motor-side loop.
- OpenArm/Robstride **has** a delay-free, locally-passive MIT loop (kp/kd on the
  motor, closed on the fresh local encoder). This changes the whole calculus.

### Key reframe: delayed SETPOINT ≠ delayed FEEDBACK
- Our oa_mit FF vibrated because it added a **PC-side torque spring**
  `couple_kp·(q_follower−q_leader)` (MIT kp=0) — a torque computed PC-side from
  fresh-q_leader + stale-q_follower, injected straight into the actuator,
  bypassing the motor's passive servo → non-passive → vibration.
- The FOLLOWER already does the *right* thing and is **smooth**: MIT{kp=50,
  pos=q_leader(delayed)} — it tracks a **delayed setpoint** with its **motor-side
  loop on fresh local q**. A delayed *reference* into a passive local servo is
  benign (worst case: small lag), unlike a delayed-data *torque*.
- ⇒ **Make the LEADER do the same**: MIT{kp=Kf, kd=Kfd, pos=q_follower(delayed),
  tau=gravity}. Symmetric with the follower; both are motor-side position
  coupling toward the peer's (delayed) position. This is enactic's actual law,
  but delivered via MIT with delayed setpoints — and the follower already proves
  delayed-setpoint MIT is smooth. Force feedback still emerges (follower blocked
  → q_follower lags → leader's motor pulls back).

### Layered plan (each layer added only if the previous isn't enough)
0. **Backbone — motor-side MIT symmetric position coupling.** Change oa_mit
   leader from PC-side torque spring → MIT kp toward delayed follower pos +
   gravity FF. Expected: kills the vibration (passive local servo), keeps
   emergent force feedback. Tune **asymmetric** (follower stiff kp~50, leader
   soft Kf~10–20 for a lighter/transparent leader; raise Kf for stronger FF).
1. **Transparency** — gravity (have it) + optional **motor-local** friction comp
   per arm (fresh, never through the delayed channel). Addresses any residual
   free-space drag (뻑뻑) without cross-channel risk.
2. **Explicit force reflection — ONLY if needed** (drag still high, or want crisp
   contact feel). Reflect the peer's *contact* torque. Here, borrow UR10e ideas
   **surgically**: a DOB to estimate peer τ̂_ext, and a small passivity wrapper
   (energy tank / wave variable) on **that delayed force channel only** — not the
   whole controller. The local servo stays motor-side/passive.
3. **2-PC network delay** — pure position coupling is fairly delay-tolerant
   (delayed setpoint = lag, not instability), so the network leg may need little.
   Add passivity (tank/wave) only to any explicit force channel from layer 2.
   Characterize the actual link with comm_benchmark first.

### Why this beats "port UR10e wholesale"
Porting UR10e's PC-side full-torque 4CH+DOB+tank would **throw away our motor-side
passive servo**, force M(q)/C(q,q̇)+DOB+tank to run PC-side at rate, and
re-create the very PC-side delayed-feedback the tank then has to fight. We'd be
manufacturing a problem the MIT loop already solves. Use UR10e's concepts only
for the one thing the MIT loop can't give us — clean, passive **explicit force
reflection over the delayed link** — and only if layers 0–1 fall short.

### First concrete step
Modify oa_mit leader: MIT{kp=Kf, kd=Kfd, pos=mapped delayed q_follower,
tau=gravity} (was kp=0 + PC torque spring). Keep couple_lpf available. Test:
does the vibration vanish (it should, like the follower)? Then tune Kf for FF
strength vs free-space lightness.

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
