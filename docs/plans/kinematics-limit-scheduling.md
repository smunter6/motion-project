# Kinematics → limit scheduling

Rolling plan. Phase 1 (the kinematics layer + SCARA integration) is **built
and merged**; its record is kept below in compressed form, because several
of its decisions constrain phase 2. Phase 2 is **unified limit scheduling**,
chosen 2026-07-27.

---

# Phase 1 — kinematics layer + SCARA (DONE, merged 2026-07-26)

Commits `f80bc25` (seam, models, app integration, headless mode) and
`bbdd995` (stick model, plot decimation) on `main`.

Tiers T0–T2 of the original nine-step plan all shipped: `KinematicModel` +
`IdentityKinematics` + `ScaraKinematics` in `motion-core`, a `kinematics`
field on `AxisGroupDef`, install-time FK/IK in the move builders, per-cycle
IK in the control loop, status/viz conversion, and `axisGroup1` = a 2-link
arm on `axis2`/`axis3`.

Design rationale now lives in `CLAUDE.md` (the `motion-core` and `app`
sections) and in the source. **Do not re-derive it from this file** — the
code is the authority. What follows is only what this document uniquely
records: where implementation *diverged* from the plan, and why.

## Where the plan was wrong or incomplete

1. **Step 7's cascade ordering was a real defect, not a detail.** The plan
   said to apply an IK-failure cascade *after* the setpoint pass. But each
   `StopRamp` starts from `commanded_velocity`, and the setpoint pass is
   what overwrites it — so every ramp was built from the zero velocity that
   pass had just written, making the "stop" an instantaneous step, next to
   a singularity, which is precisely the discontinuity the model-chain rule
   exists to prevent. Now applied in step 1d, *before* the setpoint pass.
   Found by reading printed ramp durations (`0.000s`), not by a test — **it
   is still untested**, and worth a test when phase 2 touches this path.

2. **`IdentityKinematics` carries its own `dof`.** The plan had a
   dof-agnostic unit struct, which leaves `main()`'s arity assertion with
   nothing to compare against — for the one model where getting arity wrong
   would be easiest.

3. **SCARA joints are radians, not the planned `"deg"`.** `ScaraKinematics`
   does trigonometry on these values directly and `motion-core` holds no
   unit conversions, so degrees would push a unit convention into pure
   math.

4. **The plan had no notion of drawing the mechanism.** Added: a `Linkage`
   type and a `KinematicModel::linkage` method returning the drawable
   polyline, defaulting to empty so a Cartesian group draws nothing. The
   elbow position exists nowhere else in the interface.

5. **Two things the plan never anticipated, both from actually running it:**
   `--headless` (scripted sessions shouldn't open a GUI window) and plot
   decimation (`MAX_PLOT_POINTS`; frame cost grew with the buffer until viz
   ran at ~9 fps). See `CLAUDE.md`.

## Phase 1 verification that has *not* been automated

Deliberately recorded, because these were checked by hand against the built
binary and would silently rot:

- `axisGroup0` value-for-value identical to pre-change behaviour.
- SCARA joint values matching hand law-of-cosines calculations.
- Origin-crossing move tripping `NearSingular` and ramping down cleanly.
- Off-branch jog starting smoothly — *confirmed discriminating* by stubbing
  `resolve_branch` to a constant and watching it jump instead.

None of these are tests. The control-loop testability problem (real
`sleep()`s, `println!`-format assertions) is the blocker, and it is the same
blocker `CLAUDE.md`'s "Known gaps" already names.

---

---

# Interim work (2026-07-27 → 28), and what it changed for phase 2

Not planned here, but it moved phase 2's starting line — three of the four
open questions below are now partly answered by code that exists.

- **Jerk-limited profiles** (`motion-core/src/jerk_filter.rs`). Roadmap 6's
  profile half, built by *filtering* a trapezoid: convolution with a
  rectangular window of `T = max(accel, decel)/max_jerk`. A trapezoid is the
  `max_jerk → ∞` case, asserted bit-identical. Wired through `LinearMove`,
  `PathProfile` and single-axis moves. See `CLAUDE.md` for the `v₀·T/2`
  correction and the 2×-jerk short-move caveat.
- **`MotionLimits` + `setlimits`.** Limits are now one struct used by both
  `AxisConfig` and `AxisGroupDef`, per-group rather than global constants,
  and editable at runtime (`RuntimeLimits`, seeded from the compile-time
  tables). Commands carry `Option<f64>` per limit and the *loop* resolves
  defaults, so a `setlimits` affects the next command. Updates are partial
  and **named** (`setlimits axis0 accel 100 jerk 500`).
- **Acceleration crosses the `AxisGroup` seam**, computed in closed form by
  the planner rather than differenced by a consumer. `TrajectorySample`,
  `LinearMoveSample`, `PathSample`, `AxisSetpoint` and `AxisFeedback` all
  carry it. Two pieces of this are directly phase-2 machinery:
  - `PathProfile::sample` now computes the **centripetal term**
    `v²·dT̂/ds` (central difference on the tangent, `CURVATURE_STEP`). That
    is exactly the quantity the curvature contributor must bound — the
    geometry work is done, only the *limit* is missing.
  - `KinematicModel::inverse_acceleration` exists, with the `J̇·q̇` term.
    So a joint-**acceleration** limit is now as reachable as a joint-rate
    one, which was not true when this phase was scoped.
- **Viz**: stick-figure linkage, plot decimation (`MAX_PLOT_POINTS`), no
  scroll-to-pan, per-axis acceleration plot fed from the seam.

# Phase 2 — unified limit scheduling

## The insight this phase rests on

Three separate deferred items are **the same problem**:

| deferred item | the quantity | where it comes from |
|---|---|---|
| roadmap 7 — curvature-limited feedrate | lateral accel `v²κ` | path geometry (Cartesian) |
| T3 — joint-rate limiting | joint rate `J⁻¹·ẋ` | kinematics + per-axis limits |
| T3 — whole-path plausibility | reachability, `sin(q2_eff)` | kinematics |

Each is *a derived quantity, evaluated along the path, that must stay within
a limit, where the only free variable is the feedrate `v(s)`*. Built
separately, each needs its own scan of the path and its own way of feeding a
result back into the profile — three times over. Built once, they are three
contributors to a single speed-vs-arc-length envelope.

This is why phase 2 is scoped this way rather than as "do T3, then roadmap
7". **Chosen by the user 2026-07-27**, with the larger scope understood.

## What is wrong today, concretely

- **Nothing bounds centripetal acceleration.** `PathProfile` applies one
  scalar `max_speed` over arc length. Measured: κ ≈ 0.23 at 50 mm/s gives
  ≈ 567 mm/s² lateral against a `max_acceleration` of 200 — nearly **3×
  over, today, on the demo path**.
- **Nothing bounds joint rate.** Cartesian limits say nothing about
  `J⁻¹·ẋ`. A perfectly legal Cartesian command can demand an arbitrarily
  large joint rate well before the pose-only `NearSingular` predicate trips.
  `backend-sim` integrates whatever it is handed and hides this; a real
  drive would fault on following error.
- **Singularity/reachability guarding is reactive.** Install-time validation
  covers the endpoint and each explicitly-given waypoint, and says nothing
  about the interior of a segment. The origin singularity is *reachable*
  (equal links), so "don't cross the origin" is currently a convention
  enforced by test and demo target choice, not by the code.

## Mechanism

A **speed envelope over arc length**, built at install, then a
time-parameterization that respects it.

```
v_limit(s) = min(
    v_commanded,                              // what the user asked for
    sqrt(a_lateral_max / κ(s)),               // curvature      [Cartesian]
    min over axes a of  q̇_max[a] / |(J⁻¹ t̂)_a| // joint rate    [kinematic]
)
```

Pointwise clamping is **not sufficient** and this is the crux of the phase:
you must be able to *decelerate into* a low-speed region. That needs the
standard two-pass sweep:

1. **Sample** the path — reuse the existing per-segment arc-length LUTs
   rather than inventing a second sampling scheme.
2. **Pointwise limits**: evaluate every contributor at each sample.
3. **Backward pass**: `v(sᵢ) ≤ sqrt(v(sᵢ₊₁)² + 2·a_dec·Δs)` — braking
   feasibility, so the profile can reach each limit having slowed in time.
4. **Forward pass**: `v(sᵢ) ≤ sqrt(v(sᵢ₋₁)² + 2·a_acc·Δs)` — acceleration
   feasibility. Seeded with the *actual* incoming speed, which is how
   `new_with_start_velocity` and `new_blended` keep working unchanged.
5. **Integrate** to get `s(t)`.

**Critical constraint this must not break** (CLAUDE.md decision #3): the
trajectory stays a **pure function of elapsed time**. With a varying
envelope `s(t)` has no closed form, so it becomes a precomputed table built
at install and interpolated by `sample(t)`. That is still a pure function of
`t` — no per-cycle state, no dependence on the previous sample. Install-time
allocation is already accepted in `motion-core` (`WaypointPath`'s LUTs); the
**per-cycle path must stay allocation-free**.

## Layering — the seam that needs deciding

Curvature is pure geometry and belongs in `motion-core`. Joint rate needs
the kinematic model *and* `AXIS_CONFIGS[axis].max_speed`, and `motion-core`
must not know per-axis limits (CLAUDE.md, `motion-core` design rule).

**Proposed:** `motion-core` owns the envelope machinery and the curvature
contributor, and accepts a caller-supplied array of additional speed limits
sampled at the same arc lengths. `app` computes the joint-rate contribution
(it has both the model and the config) and passes it in. Plain sampled
arrays, no callbacks or trait objects crossing the seam.

**Rejected alternative:** a `SpeedLimit` callback trait that `motion-core`
invokes. More flexible, but it drags model-and-config coupling into
`motion-core`'s API for no benefit at install-time-only cost.

## Reachability/singularity along the path

Same scan, same sample points: run `inverse_position` at each sample and
reject the whole move at install on failure. This is what converts the
reactive cascade-stop into an up-front rejection, and what makes the
equal-link origin singularity safe *by construction* rather than by
convention.

The per-cycle reactive cascade **stays** as a backstop — install-time
scanning is sampled, not exhaustive, and numerical edge cases will still
reach the loop.

## Open questions — resolve with the user before building

1. **Does this apply to `LinearMove` too, or only `PathProfile`?** A
   straight line has κ = 0, so curvature is moot — but **joint rate is
   not**: an arm tracing a straight Cartesian line has wildly varying joint
   rates, and near-singular ones. So a group `move` needs the joint-rate
   part even though it needs no curvature part. Options: share the envelope
   machinery between both profile types, or give `LinearMove` a reduced
   version. Still the biggest scoping question in the phase.
2. **Sampling density.** Fixed count per segment, or adaptive by curvature?
   Fixed is simpler and predictable; adaptive is what actually matters near
   a tight corner. Note the sample points must be shared with the
   reachability scan below, and `CURVATURE_STEP` already fixes a step size
   for the tangent difference — pick one story for both.
3. **What if the envelope forces `v → 0`** at a true singularity mid-path?
   Reject the move at install, or let it approach and stop? Rejection is
   more honest but forbids paths that merely *graze*.
4. **What limits does the envelope bound against?** Partly answered now:
   `MotionLimits` exists, is per-axis and per-group, and `setlimits` takes
   *named* fields, so adding one costs a struct field and a match arm — no
   positional breakage. What remains is the choice itself:
   - **Lateral acceleration** — its own limit, or share `max_acceleration`
     with the tangential term? Sharing is conservative and simple; separate
     is more expressive and more to configure.
   - **Joint rate vs. joint acceleration.** `inverse_acceleration` now
     exists, so both are available. Rate alone was the original scope;
     bounding acceleration too is what a drive actually cares about (it is
     proportional to torque). Probably both, but that is a decision.
5. **Does the jerk filter compose with a varying speed limit?** New, and
   unresolved. The filter assumes one fixed window `T` across the whole
   move. A conservative option is to build the envelope on the *unfiltered*
   trapezoid and filter afterwards, paying `T` in timing — but whether the
   filtered result still respects a *varying* pointwise limit needs
   checking, not assuming.

## Ordering: resolved 2026-07-28, in favour of the alternative

This section previously recorded an accepted cost — that doing limit
scheduling first would mean rebuilding the forward/backward passes once
jerk limits arrived. **That ordering was reversed.** Jerk-limited profiles
were built first (`motion-core/src/jerk_filter.rs`), by *filtering* a
trapezoid rather than deriving a seven-segment S-curve.

Consequence for this phase, and it is a real one: the sweeps must be built
**jerk-aware from the start**. Braking distance from a speed limit is no
longer `v² = v₀² + 2aΔs` — it depends on the *entry acceleration* too,
because acceleration can no longer step. The good news is that this is now
a known constraint being designed for once, rather than a retrofit.

Note the filtered formulation may make this easier than a seven-segment one
would: the underlying profile is still trapezoidal, so a conservative sweep
can be computed on the *unfiltered* trapezoid and the filter applied after,
at the cost of the window `T` in timing. Whether that composes correctly
with a *varying* speed limit is open question 5 below.

## What this phase retires

- CLAUDE.md's "Nothing bounds centripetal acceleration" note.
- CLAUDE.md's "No joint-rate limiting" gap.
- CLAUDE.md's "No preemptive mid-path reachability/singularity guarding".
- The "don't cross the origin is a convention, not code" caveat.
- Roadmap item 7 in full.

---

# Still deferred after phase 2

- **Joint *position* limits** (was T4). `AXIS_CONFIGS` for the SCARA joints
  has `position_limits: None`. Distinct from rate limiting.
- **Further kinematic models** (was T4). The design assumes
  `dof() == axes.len()` — no redundant or reduced-DOF models.
- **Mid-move IK branch reselection** (was T4, "and probably never").
  Reselecting per cycle would make IK's output depend on its own previous
  output, breaking CLAUDE.md decision #3.
- **Roadmap 6's sim half** — second-order lag, following-error limits. The
  profile half (S-curve) shipped 2026-07-28. Worth pulling forward at some
  point regardless of this phase: `backend-sim` is a pure integrator that
  models no mechanics, so jerk limiting is nearly invisible in viz and the
  *feedback* acceleration trace is only ever a restatement of the command.
- **Roadmap 8** — Yuksel C2 interpolating splines. Composes *well* with this
  phase: constant-curvature circular arcs make a curvature-limited feedrate
  trivial to compute. Still sequenced after 6 and 7.
- **Roadmap 2** — finish-together time-scaling for independently issued
  single-axis moves. Explicitly on hold; do not build toward it as a side
  effect.
- **Roadmap 9** — `backend-ethercat`. Blocked on hardware.
- **Control-loop integration testing.** The blocker for automating phase 1's
  hand-verification list. Consider extracting the loop's decision logic to
  run on fake time.

# Critical files for phase 2

- `motion-core/src/path_profile.rs` — where the envelope replaces the single
  scalar `TrapezoidalProfile` over arc length.
- `motion-core/src/waypoint_path.rs` — curvature evaluation; existing
  arc-length LUTs to reuse as sample points.
- `motion-core/src/linear_move.rs` — see open question 1.
- `motion-core/src/kinematics.rs` — Jacobian access for the joint-rate term.
- `app/src/main.rs` — `install_group_move`/`install_path_move_impl` supply
  the joint-rate limits; `AXIS_CONFIGS` is where `a_lateral_max` would land.
