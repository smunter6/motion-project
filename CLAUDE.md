# CLAUDE.md — project context for Claude Code

This file orients a fresh Claude Code session. It records *why* things are the
way they are — decisions, gotchas, and deferrals that the code itself can't
explain. Current implementation state is derivable from the source and
`git log`; don't re-narrate it here.

## What this project is

A **learning project** (go one deliberate step at a time; explain the reasoning,
don't just emit code) building motion planning / trajectory generation for a
simple robot, initially Cartesian then integrating more complex kinematics.
The end goal is to drive **EtherCAT servo drives** (via the pure-Rust
**EtherCRAB** master + the **CiA 402** drive profile, CSP mode)
while supporting a **hardware-free simulation mode** with simple visualization.
The user does not currently have the hardware, so sim mode is a first-class
requirement, not an afterthought.

## Who I'm working with

Experienced engineer/scientist, comfortable with deep technical detail, new-ish
to Rust embedded but experienced with traditional motion control. Prefers:
understanding *why* before code, small steps with discussion between them, honest
flagging of design tradeoffs and anything not yet compiler-verified. Do not
over-produce; confirm direction at natural decision points.

## Core architectural decisions (already made — respect these)

1. **Clean seam between motion logic and backend.** The planner never knows
   whether a simulation or real EtherCAT drives are behind it. Both implement
   the same `AxisGroup` trait. Sim now, EtherCAT later, swappable.
2. **`motion-core` is I/O-free and dependency-free.** Pure math only
   (trajectory, kinematics, types). This is what we unit-test on the host and
   reuse unchanged on the Pi / potentially RP2350. Nothing touching a NIC, a
   drive, or a window goes in here. No heap allocation in the per-cycle hot
   path (hence the fixed-size, `Copy` sample structs bounded by
   `MAX_GROUP_AXES = 6`).
3. **Absolute-time trajectory vs. dt-stepping plant.** The trajectory generator
   is a *pure function of elapsed time* (exact, testable, jitter-robust). The
   *stateful, dt-stepping* model belongs to the plant layer (sim model / real
   servo). They meet at the setpoint each control cycle. See the
   `NOTE (dt seam)` comment in `motion-core/src/trajectory.rs::sample`.
4. **f64 in the core.** Real drives use integer encoder counts; that conversion
   is the backend's job at the hardware seam, not the planner's.
5. **Kinematics is pluggable** — and as of 2026-07-26 the seam is real, not
   just anticipated: `motion_core::KinematicModel`, with `IdentityKinematics`
   for Cartesian groups and `ScaraKinematics` for the 2-link arm on
   `axisGroup1`.
6. **Control rate: 250 Hz** (4 ms cycle) to start.

## Deployment context

- Develop/test in **WSL** (Linux filesystem, not /mnt/c). See the
  `deploy-to-pi` skill for the cross-compile/deploy loop.
- **Real EtherCAT bus testing happens on the Pi, not in WSL** — WSL2's
  virtualized NAT networking can't do raw L2 EtherCAT reliably. WSL is for
  code + host-side unit tests of pure logic.
- EtherCRAB is `no_std`-capable, so the same crate could later target an
  RP2350-based master with an embedded TX/RX transport.

## Design rationale — why the code looks the way it does

Read the source for *what* exists. These are the decisions behind it.

### `motion-core`

- **`StopRamp` is a separate type, but `new_with_start_velocity` extends
  `TrapezoidalProfile` in place.** These look inconsistent and aren't.
  `StopRamp` (current-velocity-to-rest, one phase, no target position) is
  genuinely simpler than a generalization would be. But when interrupting a
  move with a *new target* came up, the user's explicit correction was to
  extend `TrapezoidalProfile` itself rather than add a second type — that
  correction overrides the `StopRamp` precedent. **When a new profile variant
  comes up, ask; don't default to "add a separate type."**
- **A zero-velocity `StopRamp` reports `Done` immediately**, unlike
  `TrapezoidalProfile`'s zero-*distance* case, which reports `Pre` forever.
  Deliberate — see `TrapezoidalProfile`'s doc comment for why the two cases
  differ.
- **`new_with_start_velocity` unifies three cases through one path**, not a
  case-by-case dispatch: same-direction-with-room, start speed already above
  the *new* move's max_speed (phase 1 decelerates into cruise), and
  overshoot/reversal. Overshoot and reversal are the *same* code path,
  distinguished only by which side of the target the deceleration lands on.
  Unifying superficially-different cases has been the right default here.
- **The new move's own limits govern the entire resulting profile**, including
  any reversal preamble. No blending or carryover from the superseded move.
- **Known, accepted limitation — perpendicular velocity is discarded.** When
  redirecting a multi-axis move, `LinearMove`/`PathProfile`'s
  `new_with_start_velocity` project the actual N-dimensional velocity onto the
  new line/path tangent via a dot product. The perpendicular component is
  dropped, not reconciled — a genuine *velocity* discontinuity (not just an
  acceleration one) if the actual velocity wasn't already parallel. Only
  reachable via `app`'s `aborting` group moves; the cascade path (siblings get
  their own `StopRamp`) is fully vector-correct and unaffected.
- **`LinearMove` is deliberately not a general spline.** A straight-line
  N-axis move is one scalar `TrapezoidalProfile` over the Euclidean distance,
  composed with a fixed unit-direction vector. This "reuse the existing scalar
  profile over a scalar path coordinate" pattern is the recurring one —
  `StopRamp`, `LinearMove`, and `PathProfile` all follow it.
- **`WaypointPath` uses centripetal Catmull-Rom specifically for *local
  support*** — each segment depends only on its 4 nearest control points, so
  editing one waypoint can't ripple through the whole route. A natural cubic
  spline's *global* support would. Centripetal (not uniform/chordal)
  parameterization avoids cusps on non-uniformly-spaced waypoints (Yuksel et
  al. 2011).
- **Endpoints use reflected phantoms (`P₋₁ = 2·P₀ − P₁`), not duplication** —
  duplication zero-lengths the phantom segment and divides by zero in the
  centripetal knot-spacing formula. A 2-waypoint path degenerates to an exact
  straight line under this scheme (the natural sanity check, and it's tested).
- **Arc length has no closed form for a cubic**, so each segment carries its
  own arc-length↔parameter LUT. A query always evaluates the *exact* curve at
  the LUT-refined parameter — never interpolates stored LUT positions.
- **`tangent_at_arc_length` uses a central finite difference**, not an analytic
  Catmull-Rom derivative — arc-length parameterization already gives
  `|dP/ds| ≈ 1`, so this is accurate enough for the velocity feed-forward it
  exists for. A deliberate v1 simplification.
- **`BufferMode::Blend` is deliberately *not* an exact tangent match.** The
  insight: the reflected phantom is just an *assumption* ("the incoming
  approach was a straight line toward the first waypoint") — swap it for the
  truth when a real incoming velocity is known, and everything downstream is
  unchanged. `new_with_start_direction` places the leading phantom along the
  given direction at the same characteristic distance `|P0-P1|`. A Catmull-Rom
  tangent at `P0` blends the phantom *and* `P1`, so this matches the direction
  closely but not exactly. That was the user's explicit call — *match the
  existing spline generation's own behavior* — chosen over a mathematically
  exact Hermite tangent override. **Precedent worth following: when the user
  says "match existing behavior," that's a real technical preference for
  reusing the established mechanism, not an invitation to substitute something
  more rigorous.**
- **The path is G1, never G2 — and the straight-segment override was deleted
  rather than fixed** (2026-07-26). Catmull-Rom is a **C1** construction:
  adjacent segments agree on the tangent at a shared waypoint but not on the
  second derivative, so *curvature steps at every waypoint* (measured: ±30–50%
  on a 4-waypoint path). That's inherent to the family, not a bug.

  `SegmentKind::Line` made it much worse — a measured **22.5°** velocity
  *direction* discontinuity where a forced-line segment met a spline one. The
  previously-recorded "unbuilt fix" (feed the line's direction into the
  adjacent spline's tangent formula) **cannot work**: with centripetal knots
  the junction tangent is proportional to `L·d + v`, so moving the phantom
  along `d` rescales it without ever rotating it. That same algebra is why
  `BufferMode::Blend` is inexact — one root cause, two symptoms.

  The real fix is a different curve family — **Yuksel's C2 interpolating
  splines**, roadmap item 8, which get C2 from the formulation rather than
  from a tuned second-derivative rule, and carry exact straight segments and
  circular arcs as first-class members. Sequenced *after* jerk-limited
  profiles and curvature-limited feedrate, both worth more first. Until then
  `WaypointPath` does exactly one thing — smooth splines through every
  waypoint — and `SegmentKind` no longer exists.
- **Nothing bounds centripetal acceleration.** `PathProfile` applies one
  scalar `max_speed` over arc length; lateral acceleration is `v²κ` and is
  unchecked. Measured κ ≈ 0.23 at 50 mm/s gives ≈ 567 mm/s² against a
  `max_acceleration` of 200 — nearly 3× over, today. Curvature-limited
  feedrate is the standard fix and is the next path-quality item worth
  building. Note G2 would make that quantity *continuous*, not *bounded* —
  these are separate problems.
- **Path limits are a single scalar** max_speed/max_acceleration/
  max_deceleration over arc length. Per-axis limits projected onto the local
  path tangent is a deferred, harder follow-up.
- **Jerk limiting is done by *filtering* a trapezoid, not by deriving a
  seven-segment S-curve** (`jerk_filter.rs`, 2026-07-28). Convolving a
  trapezoidal velocity profile with a rectangular window of length
  `T = max(accel, decel) / max_jerk` gives piecewise-quadratic velocity,
  continuous acceleration, and piecewise-constant jerk. The seven-segment
  derivation is a case analysis that multiplies out once the move can start
  at a non-zero velocity — which every redirect here does — while filtering
  reuses `TrapezoidalProfile` whole and adds no branches. Same "reuse the
  existing scalar profile" pattern as `StopRamp`/`LinearMove`/`PathProfile`.

  **A trapezoid is the `max_jerk → ∞` case**, not a parallel construction:
  the window shrinks to nothing and a zero-width box filter is the identity.
  `max_jerk: None` is that limit and is asserted *bit-identical* to the
  unfiltered profile, so it delegates rather than approximating.

  Costs, both accepted: the profile is not time-optimal (it runs exactly `T`
  longer — measured, every move's duration grew by exactly the window), and
  jerk is set indirectly by the window rather than commanded.
- **Filtering needs `TrapezoidalProfile::position_integral`.** Filtered
  *velocity* is only a difference of positions, `(p(t) − p(t−T))/T`, but
  filtered *position* is the mean position over the window, which needs an
  analytic antiderivative. It is piecewise over exactly the phases `sample`
  uses, so the two cannot disagree about where a phase ends, and it is
  tested against numerical integration rather than against itself.
- **A non-zero start velocity needs a `v₀·T/2` correction, or the move
  misses.** For the filtered profile to *begin* at `v₀`, the underlying
  profile is extended backwards before `t = 0` as a straight line at that
  velocity. That extension contributes area: the filtered move overshoots by
  exactly `v₀·T/2`, independent of shape. So the inner profile is built to a
  target short by that amount, and the same constant offsets the start back
  into place. **This vanishes from rest**, so every rest-to-rest test passes
  while a redirect silently overshoots — the reason it's called out here.
- **The jerk bound is exact only when a cruise phase outlasts the window.**
  On a short move one window straddles both the accel and decel steps and
  jerk reaches `(accel + decel)/T`, up to 2× the limit. Sizing the window
  for that case would double it on *every* move to bound one where the axis
  barely moves. Deliberate, and pinned by a test so it can't drift further.
- **`kinematics` maps joint space ↔ task space, and forward is infallible
  while inverse is not.** Every joint pose puts the tool somewhere; not
  every task point is reachable, and the mapping can be locally degenerate.
  `KinematicVector` is one type for all four roles (joint/task ×
  position/velocity) because it is the same N-real-valued shape either way,
  fixed-size and `Copy` for the same no-allocator reason as
  `LinearMoveSample`.
- **The IK solution branch is an opaque `KinematicBranch(u8)` token, not an
  `Elbow` enum.** "Elbow up/down" is a SCARA concept, but the value is
  stored and forwarded by model-agnostic `app` code, and an associated type
  would break `dyn` object safety. `app` obtains one from `resolve_branch`
  and hands it back to `inverse_position`, never interpreting it.
- **`ScaraKinematics` bakes in a `q2_eff = q2 + π/2` home offset**, so raw
  `(0, 0)` — where every axis starts — is "elbow bent 90°", not the fully
  extended, singular textbook pose. **Raw `q2 = 0` does not mean "straight
  arm."** The offset exists only inside FK/IK/Jacobian; everything outside
  sees raw `q2`.
- **The singularity predicate is `|sin(q2_eff)| < 0.05` (≈2.9° from straight
  or fully folded), not `|det| < ε`.** `det = l1·l2·sin(q2_eff)` carries
  units of length², so an absolute epsilon on it silently rescales with the
  link lengths and has no natural value. Dividing `l1·l2` out leaves a
  dimensionless test with a readable geometric meaning, at the same cost.
  Understand its limit: it tests the **pose alone**. A near-singular
  Jacobian loses rank in *one* direction, so this rejects some commands that
  were realizable and — worse — accepts commands whose joint rates are
  already unrealizable short of the threshold. Bounding the joint rate
  itself needs per-axis limits, which `motion-core` must not know; that
  check belongs in `app` (roadmap 7).

- **Acceleration is computed in closed form and carried across the seam,
  never differenced** (2026-07-28). Every profile knows its own
  acceleration analytically — piecewise constant for a trapezoid, piecewise
  linear once jerk-limited — so `TrajectorySample`, `LinearMoveSample` and
  `PathSample` all report it. Differencing downstream would give a delayed,
  noisier estimate of something already known exactly.
- **A path sample's acceleration has a centripetal term, not just a
  tangential one.** `a = a_t·T̂ + v²·dT̂/ds`. Omitting the second term would
  be worse than useless as a feed-forward: on a tight curve at speed it is
  usually the *larger* of the two, and it exists even at constant speed.
  `dT̂/ds` is a central difference on the tangent (which is itself a central
  difference — see `CURVATURE_STEP`). This is also the quantity roadmap 7
  needs to bound.
- **`inverse_acceleration` needs the `J̇·q̇` term and so cannot be composed
  from `inverse_velocity`.** `q̈ = J⁻¹(ẍ − J̇q̇)`. A rotating linkage
  accelerates its own tool even at constant joint rates. Measured on the
  SCARA at moderate rates, dropping the term changes the answer by ~0.7 and
  ~1.7 rad/s² against joint accelerations of the same order — leading-order,
  not a correction. Tested two ways: a finite-difference round trip through
  forward kinematics, and an explicit check that the term moves the answer.

### `axis-backend`

- **One combined `exchange()` call**, not separate write/read. Mirrors
  EtherCAT's actual single synchronous transaction per cycle (EtherCRAB's
  `tx_rx()`), so `backend-ethercat` won't need the trait reshaped later.
- **Two state layers, deliberately.** `AxisState` (PLCopen
  `MC_ReadStatus`-flavored) is the coarse view; `Ds402State` (the real CiA 402
  power-state machine) is the detail beneath it, mirroring what PLCopen's own
  `ST_AxisStatus` exists for. `Ds402State` lives at the *trait* level, not
  inside `backend-sim`, because `backend-ethercat` will need the same shape.
- **`acceleration` crosses the seam in both directions, but means different
  things** (2026-07-28). Commanded: exact, from the planner, and consumed by
  real drives as a torque offset in CSP/CSV — which is why it belongs on the
  seam rather than being re-derived by whoever wants it. Measured: a *plant*
  quantity, and most drives have no acceleration object at all, so
  `backend-ethercat` will likely difference successive velocities and
  inherit that estimate's lag. `backend-sim` reports it honestly from the
  velocity change it applied. **Don't read the feedback one as ground truth**
  the way position and velocity are read.
- **`enabled` and `fault_reset` are two distinct cyclic fields**, not one-off
  commands — like real CiA 402 controlword bits (enable vs. the edge-triggered
  Fault Reset bit).
- **`AxisFeedback` carried `fault: Option<AxisFault>` from day one**, before
  `backend-sim` could raise any, so `app`'s fault-handling path could be built
  and exercised before real hardware exists.
- **`MC_Stop` and DS402 Quick Stop are deliberately different things.** A
  commanded stop flips only the coarser `AxisState` to `Stopping`;
  `Ds402State` is untouched and stays `OperationEnabled`, because the drive
  sees nothing but an ordinary decelerating velocity setpoint.
  `Ds402State::QuickStopActive` stays reserved — that's a distinct,
  typically emergency/safety-triggered mechanism, not implemented.

### `backend-sim`

- **Integrates velocity into position each cycle rather than passing through**
  — genuinely stateful dt-stepping (see the dt seam above). This produces a
  real (if tiny) target-vs-actual gap during accel/decel for viz to plot,
  instead of two identical lines.
- **Only integrates while `OperationEnabled`.** A disabled or faulted axis
  ignores commanded velocity entirely, matching a real drive's power stage
  being off.
- **At most one DS402 transition per `exchange()` cycle, in either direction**
  (enabling takes 3 cycles, ~12 ms at 250 Hz). Not a simplification for its own
  sake — that's how a real master/drive negotiate it, one controlword write at
  a time. See `step_ds402`'s doc comment.
- **Disabling a *moving* axis faults it** — raises
  `AxisFault::DisabledWhileMoving` and routes through `FaultReactionActive`
  (held exactly one cycle) into latched `Fault`, rather than gracefully
  stepping down. A real drive can't safely cut its power stage mid-motion the
  way it can from rest. Disabling from `StandStill` stays graceful, no fault.
  `Fault` clears only via `fault_reset` (DS402 transition 15, to
  `SwitchOnDisabled`); resetting does not itself re-enable the axis.

### `app`

- **The model is ground truth for planning; feedback is for reporting**
  (settled 2026-07-26, *reversing* an earlier "feedback is ground truth"
  rule). `AxisRuntime` carries both: `position`/`velocity` from
  `AxisFeedback`, and `commanded_position`/`commanded_velocity` — exactly
  what was sent as last cycle's `AxisSetpoint`. **Every** profile is seeded
  from commanded: a new move, an aborting redirect, a `StopRamp`, a queue
  promotion, a group or path move's start state. An idle axis holds at its
  commanded position, not wherever the servo settled, which is what keeps
  the commanded stream continuous across the gap between moves.

  Seeding from feedback injects a *step* into the commanded stream — end a
  move commanding 100.000 with the axis actually at 99.980, seed the next
  move from 99.980, and the commanded position jumps backward 0.020 mm in
  one cycle. It also destroys repeatability (identical command sequences
  produce different trajectories depending on measured error) and launders
  following error into the plan instead of leaving it visible to the drive's
  own following-error detection. Note `backend-sim` **hides** this class of
  bug: it integrates commanded velocity and never chases commanded position,
  so a position step shows up only as a target-vs-actual gap in viz.

  Accepted consequence: if an axis physically can't keep up, the model runs
  away from reality until the drive faults on following error. That fault is
  the designed detector; masking it was the old rule's real cost.
- **One resync point: non-operational → operational.** Coming back from
  `Disabled`/`ErrorStop`, commanded is set from feedback, because while the
  power stage was off the axis could have moved for reasons the model knows
  nothing about. That single hook suffices because `axis_operational()`
  gates every move/stop/promotion and both disable and fault land in one of
  those two states — nothing can be commanded again without crossing it.
  Homing will be the second such point when it exists.
- **Layering rule (settled 2026-07-21):** business logic — move/stop rejection,
  queue promotion, `enable`/`disable`/`reset` gating — checks **`AxisState`
  only**, via the `axis_operational()` helper. Never branch on `Ds402State`.
  `AxisRuntime` carries `ds402_state` purely to detect and print transitions
  for traceability. This keeps `app` backend-agnostic the same way the trait
  seam is: a `backend-ethercat` swap-in only has to report the same
  `AxisState` values, not replicate `backend-sim`'s exact DS402 timing.
- **`Command::Enable` refuses while faulted**, not just while
  `SwitchOnDisabled` — so a premature `enable` sent during the fault window
  can't silently "stick" and auto-fire the instant a later `reset` clears it.
- **Seizing or losing control clears anything queued.** Established by
  `abort_into` for single axes and extended to groups by `cascade_group_stop`.
  An axis that stops being enabled drops its *entire* queue at once, with one
  summary message.
- **Queue promotion drops a kinematically-invalid move and tries the next the
  same cycle**, rather than retrying it forever.
- **Group interruption cascades.** Stopping, disabling, or faulting *any one*
  member of an active group or path move brings every other member to its own
  independent `StopRamp` — a group move missing a member no longer means
  anything. Implemented as collect-then-apply (the borrow checker won't allow
  mutating a second `Vec` element while holding `&mut` into the first from the
  same `iter_mut()`) and guarded by `Rc::ptr_eq` so a member that's already
  moved on isn't double-fired. **Don't rebuild this pattern for a new move
  kind — extend the `ActiveGroupMove` enum instead**, which is exactly how
  path moves were folded in.
- **Group moves install atomically** (gather-then-construct-then-install), not
  via the single-axis `abort_into` helper — a group build is one fallible
  construction across all members, not N independent ones, so a rejected
  construction must leave no member half-redirected.
- **`group_pending` lives in `run_control_loop`, not `AxisRuntime`** —
  promoting a queued group move needs every member simultaneously idle, which
  doesn't fit inside any single axis's own state.
- **`MotionLimits` is one shape used by both config tables** (2026-07-28):
  `AxisConfig.limits` (joint space, per axis) and `AxisGroupDef.limits` (task
  space, per group). Same four numbers either way, so one struct stops them
  drifting apart. Which applies is a **units** question, not a preference: a
  single-axis `move axis3` is joint-space and takes the axis's limits; a
  group move is TCP-space and takes the group's, even though rotary joints
  carry it out.

  Group limits are now **per group** rather than one set of global
  `DEFAULT_CARTESIAN_*` constants — an arm's TCP capability isn't a gantry's,
  even in the same mm/s. `DEFAULT_CARTESIAN_LIMITS` remains as the shared
  starting point each group's entry copies.
- **`max_jerk` comes from config only, never from a move's command line.**
  Every other limit can be overridden per move (`[vmax] [amax] [dmax]`);
  jerk cannot. It's a property of what the machine can stand, not a per-move
  choice the way a feedrate is. `setlimits` changes it; a `move` never does.
- **Limits are resolved in the control loop, not in the parser** (added
  2026-07-28 with `setlimits`). A `Command` carries `Option<f64>` per limit
  — *what the user typed* — and `RuntimeLimits::resolve` fills the gaps from
  the loop's live table at handling time.

  This split is what makes runtime limits work at all. The parser runs on
  the stdin thread and can't see loop state, so resolving there would pin
  every command to the compile-time values and a `setlimits` would silently
  do nothing for bare moves. Resolving at handling time also means a
  `setlimits` affects the very next command rather than racing it.

  `RuntimeLimits` seeds from `AXIS_CONFIGS`/`AXIS_GROUPS` and is the only
  mutable copy; the `const` tables stay the startup values. **This is a
  deliberate softening of the "config is compile-time" decision** below —
  the tables are still the source of truth at startup, and there's still no
  persistence, so a `setlimits` lasts until exit.
- **A queued move stores its limits when accepted, not when promoted.**
  `PendingMove`/`PendingGroupMove` carry a resolved `MotionLimits`, so a
  later `setlimits` can't retroactively rewrite a move already sitting in a
  queue. Same "fail/decide where the user typed it" principle as
  `check_target_in_limits`.
- **`dmax` now falls back to the configured deceleration, not to `amax`**
  (changed 2026-07-28). The old rule pre-dated per-axis config and meant a
  configured `max_deceleration` was *silently ignored by every move* — only
  `stop` and the group cascade ever used it. Invisible until `setlimits` let
  someone set a deceleration and watch it not apply. The two now agree.
- **`setlimits` updates are partial and *named*, not positional** —
  `setlimits axis0 accel 100 jerk 500`, with `setlimits axis0` reporting and
  changing nothing. A comma form (`setlimits axis0 ,,100,500`) was the
  alternative and was rejected: miscounting a comma puts the wrong *valid*
  number into the wrong limit, silently, on a parameter governing machine
  motion. Names are also order-independent, readable back in a log, and
  survive a new limit being added — phase 2's lateral-acceleration bound
  would otherwise be a new column everyone must count past.

  Each limit takes two spellings, `speed`/`accel`/`decel`/`jerk` and
  `vmax`/`amax`/`dmax`/`jmax`, because both vocabularies already exist in
  the app (`move`'s positional args use the second in `help`).

  `LimitsUpdate::max_jerk` is `Option<Option<f64>>` and **both levels
  matter**: the outer `None` is "not named, leave it", the inner is "named
  as `none`, i.e. no jerk limit". Collapsing them would make "don't touch
  jerk" and "remove the jerk limit" the same command. Pinned by a test.

  A report always prints all four limits, not just the changed ones — after
  a partial update the useful question is what the machine will now do, not
  which words were typed.
- **Per-axis capability lives in `AXIS_CONFIGS`, not global constants**
  (added 2026-07-26). Each axis carries its own max speed/accel/decel *in its
  own units*, a display-only `units` label, and optional soft travel limits.
  Single-axis `move`/`stop` defaults come from here; Cartesian group/path
  moves keep the `DEFAULT_CARTESIAN_*` constants, since a TCP-space limit is
  mm/s whatever kind of axes carry it out.

  The motivating case wasn't the defaults, it was `cascade_group_stop`: it
  used to reuse the group move's `max_deceleration`, which is *Cartesian*,
  to build per-member `StopRamp`s in *joint* space. Identical while every
  axis is linear; a silent unit error the moment a group has non-identity
  kinematics — on the path that runs when something has already gone wrong.
  `SharedGroupMove`/`SharedPathMove` no longer carry `max_deceleration` at
  all.

  One flat struct, deliberately not an enum over axis kinds: what differs
  between a linear axis and a rotary joint is the numbers, a label, and
  whether travel is bounded — not the operations, and `motion-core` is
  unit-agnostic `f64` throughout. Add an `AxisKind` enum when some axis type
  needs different *math* (continuous-rotation shortest-path wraparound, say),
  not merely different values.
- **`position_limits` is checked at the commanded endpoint only.** A
  single-axis move target outside the range is rejected when the command is
  accepted (so a `buffered` move fails where the user typed it, not later at
  promotion). It says nothing about a path's interior, or about where a
  group's Cartesian target lands in joint space — both need whole-path
  plausibility checking, still unbuilt.
- **Every group carries a `KinematicModel`, including Cartesian ones**
  (added 2026-07-26). `axisGroup0`'s pair of linear stages uses
  `IdentityKinematics`, so there is exactly **one** code path through the
  move builders, the control loop and status/viz — never an
  `if group.is_an_arm { … }` fork — and a Cartesian group is provably
  unaffected (identity composed with identity is a no-op; verified
  value-for-value against the pre-change binary). `IdentityKinematics`
  carries its own `dof` rather than being a dof-agnostic unit struct, so
  `main()`'s arity assertion means something for the one model where it
  would be easiest to get wrong.
- **A group move's profile is Cartesian end to end.** Kinematics converts
  *into* it once at install (FK on the commanded joint state) and *out of
  it* once per group per cycle (IK to joint setpoints). `LinearMove` and
  `PathProfile` never learn that kinematics exists.
- **The IK branch is resolved once per move, from commanded joints, and
  held.** Stored on `SharedGroupMove`/`SharedPathMove`. Not per-*instance*:
  a raw single-axis jog can put the arm on the other branch, and a
  fixed-branch model would jump back to its own on the next move's first
  cycle. Not per-*cycle*: "nearest solution to last cycle" makes IK's output
  depend on its own previous output, breaking decision #3 (a sample at
  `t = 0.348` would depend on the history that got there). Held per move, IK
  stays a pure function of Cartesian position, and the branch persists
  across moves *for free* — commanded joints came out of IK on that branch,
  so re-resolving from them returns it again, until the enable-transition
  resync, which is exactly when it should change.

  **`backend-sim` cannot show you a branch bug**: it integrates commanded
  velocity and never chases position, so an off-branch jump appears only as
  a growing target-vs-actual gap. A real drive in CSP mode would fault on
  following error immediately — severity is inverted between sim and
  hardware.
- **A runtime IK failure cascades into per-joint `StopRamp`s, and the
  cascade must run *before* the setpoint pass, not after.** Holding position
  next to a singularity is exactly what doesn't recover — zero velocity
  there stays there — so every member ramps down independently in joint
  space, which is singularity-free by construction. The ordering is not
  cosmetic: each ramp starts from `commanded_velocity`, and the setpoint
  pass is what overwrites it. Cascading afterwards builds every ramp from
  the zero velocity that pass just wrote, i.e. an *instantaneous* stop —
  the exact step discontinuity the model-chain rule exists to prevent,
  delivered at the worst possible moment. Found by reading the printed ramp
  durations (`0.000s`), not by a test.
- **`cascade_group_stop` takes `except: Option<usize>` and a `reason`.**
  `None` because an IK failure has no blaming axis — every member stops.
- **`Profile::target()` is joint space for group members too** — the stored
  IK of the Cartesian target, not the matching component of it. Those
  coincide only under identity kinematics.
- **SCARA joint values are radians**, not degrees (a deliberate deviation
  from the plan, which said `units: "deg"`). `ScaraKinematics` does
  trigonometry on these values directly and `motion-core` holds no unit
  conversions, so degrees would mean a unit convention leaking into pure
  math. `AXIS_CONFIGS` says `"rad"` and `move axis3 0.5` means 0.5 rad.
- **Axis groups are hard-coded** (`AXIS_GROUPS`), not created/removed at
  runtime — deliberately simpler than PLCopen's real axis-group model. When a
  new feature needs a multi-axis target, reuse this table rather than
  reintroducing free-form per-command axis lists (the user's explicit call:
  *"that's exactly why I stopped to implement them first"*).
- **`movepath` is group-only.** A single-axis "path" is just `move ... buffered`
  chaining.
- **`BufferMode::Blend` is `movepath`-only** — `move` has no spline to lean a
  tangent into, so it keeps its own two-keyword parser.
- **`verbose` gates only the repetitive per-cycle heartbeat.** Discrete events
  (move started/finished/aborted, enable/disable, faults, DS402 transitions)
  always print. Off by default because the viz window already plots
  target-vs-actual continuously.
- **Viz taps the `AxisGroup::exchange()` seam, it does not touch the loop's
  decision logic.** `RecordingAxisGroup` wraps whatever backend it's given and
  forwards `exchange()` unchanged. The consequence to remember: viz has no
  visibility into `app`'s higher-level `Profile`/command bookkeeping, so it can
  only show a per-member rollup, never "is a synchronized group move active" —
  the terminal `status` command can, since it reads `AxisRuntime` directly.
- Viz plots a plane only for groups of **exactly 2 axes**; a 3+-axis group
  would need a projection or a 3D view (`egui_plot` is strictly 2D — this is
  why a third axis / XYZ group was declined).

## Roadmap

1. [done] One axis: trapezoidal trajectory + invariant tests.
2. **[deferred]** Two axes + coordination (finish-together time-scaling; the
   slower axis sets the move time, the faster axis is time-scaled to match).
   This is where `duration()` starts earning its keep. Explicitly on hold —
   `app` drives axes independently (no coordination) and that stays true until
   this is revisited. **Don't build toward coordinated moves as a side effect
   of other work.** **Not the same thing** as the hard-coded axis groups: a
   group move is one command building one shared trajectory for a *named,
   fixed* set of axes; this step is about auto-synchronizing the durations of
   two *independently issued* single-axis moves. Groups existing does not mean
   this snuck in via a side door.
3. [done] `AxisGroup` backend trait (`axis-backend`) + sim backend
   (`backend-sim`).
4. [done] Fixed-timestep run loop: planner → backend → feedback.
5. [done] Simple viz (egui/eframe + egui_plot), baked into `app` as
   `viz.rs` + `recording.rs` — not a separate crate, and not a second frontend
   (input stays terminal-only; viz is passive and read-only). eframe/winit
   require the GUI event loop on the main thread, which is why
   `run_control_loop` lives on a spawned thread. Viz is a dev tool, host-only,
   not deployed to the Pi. See `app/CLAUDE.md` for the WSL renderer gotcha.
5b. [done] **Kinematics seam + SCARA** (tiers T0–T2 of the kinematics plan):
   `KinematicModel`/`IdentityKinematics`/`ScaraKinematics`, install-time
   FK/IK in the move builders, per-cycle IK in the control loop, and
   `axisGroup1` = a 2-link arm on `axis2`/`axis3`. **T3 (whole-path
   reachability/singularity checking and joint-rate limiting) is not built
   — it is now folded into item 7 below. T4 (joint position limits, further
   models) is not built either** — see Known gaps.
6. **[partly done]** S-curve (jerk-limited) profiles — **done** 2026-07-28
   via `jerk_filter.rs`, wired through every move kind. Still to do: richer
   sim (second-order lag, position/following-error limits, faults).

   Worth knowing the two halves are coupled: **`backend-sim` integrates
   commanded velocity and models no mechanics, so jerk limiting is very
   nearly invisible in viz.** What it improves — excitation, following error
   during accel transients — isn't modelled, so the sim half is what makes
   the profile half observable rather than an act of faith.
7. **Unified limit scheduling** — the next phase, scoped 2026-07-27.
   Curvature-limited feedrate (bound `v²κ` instead of applying one scalar
   `max_speed` however tight the curve), joint-rate limiting, and whole-path
   reachability/singularity checking are **one problem**: a derived quantity
   evaluated along the path, bounded by a limit, with feedrate `v(s)` the only
   free variable. So they share one speed-vs-arc-length envelope and one scan,
   rather than three of each. Converts today's reactive cascade-stops into
   up-front feedrate reduction or rejection, and subsumes what the kinematics
   plan called T3.

   **The ordering cost this used to carry is gone.** The worry was that
   item 6's jerk limiting would force the envelope's forward/backward
   acceleration passes to be rebuilt. Jerk limiting landed *first*
   (2026-07-28), so those passes get built once, already knowing that
   braking distance depends on entry acceleration and not just velocity.
   Detailed design, and four open questions, are in the plan file (see
   `MEMORY.md`) — not duplicated here, since nothing is built yet.
8. **C2 interpolating splines (Yuksel's class), replacing Catmull-Rom.**
   Cem Yuksel, "A Class of C2 Interpolating Splines", *ACM TOG* 39(5) art. 160,
   July 2020 — same author as the centripetal parameterization work this
   module already rests on. Non-polynomial splines built by *trigonometric
   blending* of an interpolation function through three consecutive control
   points. Use the **circular** variant.

   Why this and not quintic Hermite (the earlier plan, superseded 2026-07-26):
   both give C2, interpolating, local support (4 points/segment), no global
   solve. But quintic Hermite needs an *invented local rule for second
   derivatives* at each waypoint, and this codebase already carries two
   documented defects that trace to exactly that kind of heuristic (see the
   `motion-core` continuity note). Yuksel's C2 falls out of the formulation
   with nothing to tune. It also adds guarantees we cannot otherwise get:
   **self-intersection-free segments** regardless of control point placement
   (the paper notes centripetal Catmull-Rom — what we run today — is still
   prone to these), plus **exact circular arcs and exact straight segments**
   as first-class members of the same family, which is how `SegmentKind`
   properly returns.

   Sequenced after 6 and 7 deliberately: geometric C2 buys little while the
   scalar profile is trapezoidal (acceleration already steps in time), and
   continuity is not boundedness. It composes with 7 — constant-curvature
   circular arcs make a curvature-limited feedrate trivial to compute.

   Cost: non-polynomial, so transcendental evaluation per sample. Irrelevant
   at 250 Hz; mildly relevant to the RP2350 aspiration, though not a new
   problem in kind — `motion-core` already calls `f64::sqrt`, which equally
   needs `libm` under `no_std`. Arc length still has no closed form, so the
   per-segment LUT machinery is unaffected.

   Separately, for *later*: Cai, Yang & You, "A Catmull-Rom Spline Based
   Analytical C3 Continuous Tool Path Smoothing Method for Robotic Machining",
   *Acta Mechanica et Automatica* 2025, DOI 10.2478/ama-2025-0088. Not an
   alternative to the above — a different layer. Yuksel replaces the *curve
   family* while keeping waypoints exactly interpolated; Cai replaces the
   *waypoint semantics* (corner smoothing within a deviation tolerance, path
   no longer passes through corners) to reach C3 jerk continuity. Revisit only
   if jerk-continuous geometry becomes the binding constraint.
9. (Hardware later) `backend-ethercat`: EtherCRAB + CiA 402 state machine +
   PDO mapping, behind the same `AxisGroup` trait.

## Known gaps / not built

- **No preemptive mid-path reachability/singularity guarding** (roadmap 7).
  Validation is install-time only — a group move's endpoint, and *every*
  explicitly-given `movepath` waypoint — plus the reactive cascade-stop
  above. Nothing checks the *interior* of a segment.
- **No joint-rate limiting** (roadmap 7). `max_speed`/`max_acceleration`
  bound Cartesian TCP motion; joint rate is `J⁻¹·ẋ`, so a perfectly legal
  Cartesian command can demand an arbitrarily large joint rate well before
  the pose-only `NearSingular` predicate trips. The *limit* comparison is
  `app`'s (it reads `AXIS_CONFIGS[axis].max_speed`, which `motion-core` must
  not know) — but note the shape changed when roadmap 7 absorbed this:
  bounding the rate *reactively* per cycle is one comparison into the
  existing `cascade_group_stop`, while bounding it *preemptively* means
  contributing a term to the speed envelope at install. The plan file takes
  the latter; the former survives only as a backstop.
- **The SCARA's equal links put a *reachable* singularity at the origin**
  (roadmap 7). Equal links collapse the inner unreachable hole to a point,
  so `r = 0` is reachable and a straight move from `(x, y)` to `(−x, −y)`
  crosses it with both endpoints validating cleanly. The user's explicit
  call, made with this understood: handle it reactively rather than
  contorting the geometry (unequal links, or an artificial `r_min`
  keep-out). Until roadmap 7, **"don't cross the origin" is a convention
  enforced by test and demo target choice, not by the code.**
- No joint *position* limits — `AXIS_CONFIGS` for the SCARA joints has
  `position_limits: None`. Distinct from rate limiting, and not part of
  roadmap 7.
- No redundant/reduced-DOF kinematic models; the design assumes
  `dof() == axes.len()`.
- No straight-segment override within a path. `SegmentKind` was removed
  (2026-07-26) rather than left broken — see the `motion-core` section. It
  returns with roadmap item 8.
- `movepath` file format has no comment syntax.
- The four real PLCopen blending variants (`BlendingLow`/`Previous`/`Next`/
  `High`) — a structurally different problem: they need a profile aware of an
  *adjacent* segment that never fully decelerates before handing off. Distinct
  from `BufferMode::Blend`, which transitions path *geometry*. No plan yet;
  confirm the shape with the user before building.
- Full control-loop integration testing (queuing, per-axis independence) —
  real `sleep()`s and `println!`-format assertions make it slow and brittle.
  Revisit only if it becomes a recurring pain point, and consider extracting
  the loop's decision logic to run on fake time first.

## How to work in this repo

Small steps. Before each step, briefly discuss what we're building and why, then
implement, then verify with `cargo test`/`cargo run`. Flag design tradeoffs
explicitly and let the user choose at decision points. Keep `motion-core`
pure. Preserve the architectural decisions above unless we explicitly revisit
one.

**Plans live in `docs/plans/`** — see its `README.md` for the index and what a
plan is for. Two are live (unified limit scheduling; telemetry), and between
them they carry nine open questions addressed to the user. **Read the relevant
plan before starting or discussing a phase**, and don't conclude from the
roadmap above that work is unplanned. Plan mode writes to `~/.claude/plans/`
under a generated slug name by default; move it into `docs/plans/` and rename
it for content once it stabilises, so plans stay versioned with the code they
describe.

**Verify interactive changes against the built binary** (`./target/debug/app`),
not `cargo run` — there's a known piped-stdin timing gotcha.

**Scripted/automated sessions run `--headless`** (no viz window) — that is the
default for anything driven by a script rather than a person, and the only
exception is a change that is specifically *about* the visualization. It isn't
just tidier: the window needs a display server, dominates startup, and keeps
the process alive after the control loop ends. Headless runs the identical
control loop (recording included — the tap is at the `AxisGroup` seam and
doesn't care whether anything draws it) on the main thread, so it verifies the
same behavior. It also needs **no startup sleep**: the loop is running before
the first piped command is drained, unlike the windowed path which wants ~2 s
first. Per-command settling sleeps are still needed — `enable` takes 3 control
cycles before a `move` will be accepted. See `scripts/demo_session.sh`.

NEVER use sed, awk, or cat to read or edit files. Always use the built-in
Read, Edit, and Write tools.
