# Add hard-coded axis groups (`axisGroup0` = X/Y Cartesian pair)

## Context

This supersedes the previous plan in this file (a full PLCopen `MC_MovePath`
spline/waypoint feature), which is shelved, not abandoned — that design
session is what surfaced the idea of a shared, `Rc`-based multi-axis
profile with group-wide interruption semantics. This plan picks up a much
narrower, simpler piece of that idea first: **hard-coded axis groups**
(not created/removed at runtime, unlike PLCopen's real model) doing plain
synchronized point-to-point moves, reusing the existing `enable`/`move`/
`stop` commands rather than inventing new ones. It's a deliberate stepping
stone — when/if the full spline `MovePath` feature is eventually built, the
`LinearMove` type here is a natural candidate to be subsumed by (or
reimplemented as a thin wrapper over) a 2-waypoint, all-straight-line path —
but this plan does not build toward that, it just doesn't foreclose it.

Confirmed requirements (via direct discussion):

1. **Groups are hard-coded**, e.g. one const table entry. To start:
   `axisGroup0` = `axis0` (X) + `axis1` (Y), a Cartesian pair.
2. **The same command verbs** (`enable`, `disable`, `reset`, `stop`, `move`)
   accept either an axis name (`axis0`) or a group name (`axisGroup0`) as
   their target. `enable`/`disable`/`reset`/`stop` on a group behave exactly
   like fanning the same per-axis logic out to each member — no new
   semantics needed there.
3. **`move axisGroup0 <x> <y> [vmax] [amax] [dmax] [aborting|buffered]`** is
   the one genuinely new behavior — confirmed via direct Q&A: *"it should be
   a straight line in xy space, the velocity should be the path velocity and
   the axis speeds should be synchronized accordingly."* I.e. one scalar
   `TrapezoidalProfile` over the Euclidean distance between the N-dimensional
   start and target points, with each axis's position/velocity derived from
   that single scalar sample via a fixed unit-direction vector — not
   independent per-axis profiles (wouldn't trace a straight line) and not
   per-axis time-rescaling (unnecessarily complex; this is simpler and
   exactly what "the velocity is the path velocity" means).
4. **Interrupting one member of an active group move stops the whole
   group** (every other member gets its own `StopRamp` from its own actual
   position/velocity) — the same principle already confirmed for the
   shelved path-following plan, carried over by the same reasoning: a group
   move missing one of its members no longer means anything.

Two research/design passes (an initial design draft, then a validation
pass) already went into the architecture below, including tracing the
actual borrow-checker mechanics and one honest correction to my own
"acceptable simplification" framing (see Known Limitation below) — trust
the corrected version, not the original framing.

## Design

### Stage 1 — `motion-core`: `LinearMove` (new file, fully unit-tested in isolation before any `app` change)

A straight-line, N-axis generalization of `TrapezoidalProfile` — deliberately
not reusing the shelved `PathProfile`/`WaypointPath` names (that Catmull-Rom
+ arc-length-LUT machinery doesn't exist yet and would be overkill for what
is fundamentally a constant unit vector times a scalar profile).

**New file `motion-core/src/linear_move.rs`:**

```rust
pub const MAX_GROUP_AXES: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LinearMoveError {
    DimensionMismatch { start_len: usize, end_len: usize },
    StartVelocityDimensionMismatch { start_len: usize, start_velocity_len: usize },
    TooManyAxes(usize),
    NonFiniteCoordinate { axis_index: usize, value: f64 },
    Speed(TrajectoryError), // propagated from the internal scalar TrapezoidalProfile
}
// Display + std::error::Error, hand-written, mirroring TrajectoryError's
// impl exactly (no thiserror — motion-core stays dependency-free). The
// Speed(_) arm delegates to the inner error's own Display.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearMoveSample {
    positions: [f64; MAX_GROUP_AXES],
    velocities: [f64; MAX_GROUP_AXES],
    len: usize,
}
impl LinearMoveSample {
    pub fn position(&self) -> &[f64];
    pub fn velocity(&self) -> &[f64];
}

#[derive(Debug, Clone, PartialEq)]
pub struct LinearMove {
    start: [f64; MAX_GROUP_AXES],
    unit_direction: [f64; MAX_GROUP_AXES],
    len: usize,
    speed: TrapezoidalProfile,
}
impl LinearMove {
    pub fn new(start: Vec<f64>, end: Vec<f64>, max_speed: f64, max_acceleration: f64, max_deceleration: f64)
        -> Result<Self, LinearMoveError>;
    pub fn new_with_start_velocity(start: Vec<f64>, start_velocity: Vec<f64>, end: Vec<f64>,
        max_speed: f64, max_acceleration: f64, max_deceleration: f64) -> Result<Self, LinearMoveError>;
    pub fn duration(&self) -> f64;
    pub fn target(&self) -> Vec<f64>;                 // one-off convenience
    pub fn phase_at(&self, t: f64) -> MotionPhase;
    pub fn sample(&self, t: f64) -> LinearMoveSample;  // zero-alloc, per-cycle hot path
    pub fn axis_count(&self) -> usize;
}
```

**Construction**: `delta[i] = end[i] - start[i]`, `length = ||delta||`
(Euclidean norm). `unit_direction[i] = delta[i] / length`, with an explicit
`length == 0.0` guard setting every `unit_direction[i] = 0.0` directly
(never computing `0.0/0.0`) — this, combined with the scalar profile's own
already-correct zero-distance handling, means the degenerate same-point case
needs no further special-casing anywhere. The scalar profile is
`TrapezoidalProfile::new(0.0, length, max_speed, max_acceleration,
max_deceleration)` (or `new_with_start_velocity(0.0, v0_along, length, ...)`)
— reused **completely unchanged**. `sample(t)`: sample the scalar profile
once, then for each axis `position[i] = start[i] + unit_direction[i] *
scalar.position`, `velocity[i] = unit_direction[i] * scalar.velocity`.

`new()` is a thin wrapper — `Self::new_with_start_velocity(start, vec![0.0;
n], end, ...)` — mirroring `TrapezoidalProfile::new`'s own precedent
exactly. Verified by hand that this reduces correctly: `v0_along =
dot(zero_vector, unit_direction) = 0.0` exactly, which is precisely the
input `TrapezoidalProfile::new_with_start_velocity`'s existing thin-wrapper
test (`start_velocity_zero_matches_plain_new`) already proves is
byte-identical to calling `new` directly.

`new_with_start_velocity`'s key trick: project the actual N-dimensional
velocity vector onto the new path's unit direction via a dot product —
`v0_along = dot(start_velocity, unit_direction)` — reducing back to the same
1-D scalar problem `TrapezoidalProfile::new_with_start_velocity` already
solves completely (same-direction-with-room, overshoot-reverse,
opposite-direction, overspeed-into-cruise — all reused for free).

**Known limitation (state this plainly, not by soft analogy)**: the
perpendicular component of the actual velocity (`start_velocity - v0_along *
unit_direction`) is discarded, not reconciled. This is a genuine **velocity
discontinuity** at the abort instant when the axis's actual velocity isn't
already parallel to the new line — one order worse than the
acceleration-only discontinuities this codebase already tolerates at
`TrapezoidalProfile`'s own phase boundaries (where velocity itself stays
continuous). Scope of the actual risk: this bites **only** an `Aborting`
`MoveGroup` redirect into a non-colinear new direction — the cascade path
(other members getting individual `StopRamp`s when one member is
interrupted) is fully vector-correct, since each `StopRamp` is built from
that axis's own actual scalar velocity with no projection involved. Accepted
as a v1 simplification; a live smoke test (below) confirms it degrades to a
visible velocity snap, not a NaN/crash.

**Tests** (mirroring `trajectory.rs`'s existing naming/style):
`linear_move_matches_hand_computed_2d_trapezoid` (a 3-4-5 triangle:
start=(0,0), end=(3,4), length=5, unit_direction=(0.6,0.8) — hand-checkable),
`degenerate_same_point_start_end_is_inert`,
`start_velocity_aligned_reduces_to_scalar_case`,
`start_velocity_perpendicular_component_is_dropped` (asserts the dropped
component is exactly zero and nothing panics/NaNs — the direct proof of the
known limitation above), `never_exceeds_max_speed_as_vector_norm`,
`dimension_mismatch_rejected`, `start_velocity_dimension_mismatch_rejected`,
`too_many_axes_rejected`, `non_finite_coordinate_rejected`,
`new_is_thin_wrapper_of_new_with_start_velocity`,
`position_is_continuous_across_all_boundaries` (a case exercising the
overshoot/reverse prefix).

Update `motion-core/src/lib.rs`:
```rust
pub mod linear_move;
pub use linear_move::{LinearMove, LinearMoveError, LinearMoveSample, MAX_GROUP_AXES};
```

Run `cargo test -p motion-core` and confirm this whole stage passes in
total isolation before touching `app` at all.

### Stage 2 — `app`: handler refactor only (no group commands yet, reviewable on its own)

Extract today's inline per-axis command-handling bodies into small named
functions, called once each from the existing single-axis arms — purely a
relocation, not a behavior change:

```rust
fn handle_enable(ax: &mut AxisRuntime, axis: usize);
fn handle_disable(ax: &mut AxisRuntime, axis: usize);
fn handle_reset(ax: &mut AxisRuntime, axis: usize);
fn handle_stop(ax: &mut AxisRuntime, axis: usize, max_deceleration: Option<f64>);
```

This is what lets Stage 3's group commands reuse the exact same logic/
messages via a simple loop, rather than duplicating it. Looping
`for &i in group_axes { handle_x(&mut axes[i], i, ...) }` is a sequential
single-index reborrow each iteration (not a live `iter_mut()`), so there's
no borrow-checker conflict — same pattern the existing command-drain loop
already uses.

While here: add the CLAUDE.md-flagged backlog item — a `#[cfg(test)] mod
tests` for `parse_command`/`parse_axis` (cheap, pure, no threading).

Verify: `cargo test -p app` (new parser tests), plus one live smoke test
rerunning today's existing single-axis scenarios (enable/move/stop/disable/
reset against the built binary) confirming output is unchanged from
pre-refactor — cheap insurance before Stage 3 lands on top.

### Stage 3 — `app`: group commands, `Profile::Group`, cascading interruption

1. Hard-coded config near `NUM_AXES`:
   ```rust
   struct AxisGroupDef { name: &'static str, axes: &'static [usize] }
   const AXIS_GROUPS: &[AxisGroupDef] = &[
       AxisGroupDef { name: "axisGroup0", axes: &[0, 1] },
   ];
   ```
   Plus a `debug_assert!` at the top of `main()` that every referenced axis
   index is `< NUM_AXES`.
2. Target resolution, so every existing command verb can take either kind
   of name:
   ```rust
   enum Target { Axis(usize), Group(usize) } // Group is an index into AXIS_GROUPS
   fn parse_target(s: &str) -> Result<Target, String> {
       if let Some(g) = AXIS_GROUPS.iter().position(|g| g.name == s) { return Ok(Target::Group(g)); }
       parse_axis(s).map(Target::Axis)
   }
   ```
   No parsing ambiguity: each group's axis count is known at parse time
   from `AXIS_GROUPS`, so the move parser deterministically consumes
   exactly that many leading numeric tokens as target coordinates (1 for an
   axis, `AXIS_GROUPS[g].axes.len()` for a group) before falling through to
   the existing up-to-3-numeric-then-optional-keyword tail logic unchanged.
3. New `Command` variants: `EnableGroup{group}`, `DisableGroup{group}`,
   `ResetGroup{group}`, `StopGroup{group, max_deceleration: Option<f64>}`,
   `MoveGroup{group, targets: Vec<f64>, max_speed, max_acceleration,
   max_deceleration, buffer_mode: BufferMode}`. `EnableGroup`/`DisableGroup`/
   `ResetGroup`/`StopGroup` are implemented as a direct loop over
   `AXIS_GROUPS[group].axes` calling the Stage 2 handler functions once per
   member — no new logic, pure fan-out.
4. Group move state:
   ```rust
   struct SharedGroupMove {
       profile: motion_core::LinearMove,
       axes: &'static [usize],  // points straight at AXIS_GROUPS[group].axes, zero-alloc
       group: usize,
       max_deceleration: f64,   // reused for sibling stops on cascade
   }
   enum Profile {
       Move(TrapezoidalProfile),
       Stop(StopRamp),
       Group { shared: Rc<SharedGroupMove>, index: usize },
   }
   ```
   `sample`/`phase_at`/`target` index into the shared `LinearMoveSample`/
   `Vec<f64>` at `index`; `is_stop()` is always `false` for `Group`.
5. **`Command::MoveGroup` handling does NOT reuse `abort_into`** — that
   helper models one fallible per-axis construction (right for the
   cascade), but installing a group move is one fallible construction
   *across all members at once*. Use a two-step, all-or-nothing shape
   instead: gather each member's actual `(position, velocity)` and attempt
   `LinearMove::new_with_start_velocity(starts, velocities, targets, ...)`
   *before* touching any axis's state; only on `Ok` does a second,
   infallible pass install `Profile::Group` on every member (clear
   `pending`, set `active`, reset `last_phase`). On `Err`, nothing is
   touched and the error is reported once (`LinearMoveError`'s `Display`).
   This keeps the group atomic — no member is left half-redirected if
   construction fails. `BufferMode::Aborting` always attempts this (busy or
   idle); `BufferMode::Buffered` requires every member simultaneously idle
   first (`active.is_none() && pending.is_empty()`) and otherwise rejects
   with a message that explains *why* group-buffering differs from
   single-axis buffering (no per-group queue exists yet — "busy, and
   buffered group moves don't queue yet; use aborting, or wait").
6. **Cascading interruption** (a single named axis being stopped/disabled/
   faulted/aborted while it's a member of an active `Profile::Group`
   cascades to every other member), via a two-pass collect-then-`abort_into`
   structure (needed because `axes.iter_mut()` can't hold `&mut` into one
   element while touching another by index):
   - **Step 4 (fault/disable, in the feedback-folding loop)**: before
     clearing `ax.active = None` for whichever axis just went
     non-operational, if its cleared profile was `Profile::Group{shared,
     ..}`, clone the `Rc` out into a `Vec<(Rc<SharedGroupMove>, usize)>`
     collected across the loop. After the loop (borrow released), a second
     pass: for each collected `(shared, faulted_axis)`, for every `other in
     shared.axes` where `other != faulted_axis`, check
     `matches!(&axes[other].active, Some(ActiveMove{profile:
     Profile::Group{shared: s, ..}, ..}) if Rc::ptr_eq(s, &shared))` before
     calling `abort_into(&mut axes[other], other, "move", |pos, vel|
     Ok(Profile::Stop(StopRamp::new(pos, vel, shared.max_deceleration)?)))`
     — the guard is what makes double-fault-in-one-cycle (both sides
     already self-cleared, guard skips both, no spurious double-stop) and
     asymmetric-disable-timing (the earlier-cleared member correctly
     cascades onto the still-active other) both come out correct.
   - **`Command::Stop{axis}` / `Command::Move{axis, buffer_mode: Aborting}`
     targeting a lone member**: clone `shared`'s `Rc` out *before* that
     axis's own handling replaces its `active`, then loop `abort_into` over
     `shared.axes.iter().filter(|&&j| j != axis)` the same way.
   No new profile-building logic for siblings anywhere — this reuses the
   existing single-axis `abort_into` + `StopRamp` mechanism unchanged, just
   invoked once per group member.
7. `print_help()`/`print_status()` text for the new commands.

**Nothing else needs to change** — `axis-backend`, `backend-sim`,
`app/src/viz.rs`, and `app/src/recording.rs` are untouched. The group lives
entirely in `motion-core` (new `LinearMove`) and `app`'s command/profile
layer; `AxisGroup::exchange` stays purely per-axis, so the existing XY plot
(already shown whenever `NUM_AXES >= 2`) traces a group move's straight line
with zero changes — a good confirmation the trait seam is doing its job.

## Explicitly out of scope for this pass

- Queued/buffered `MoveGroup` when busy (rejected, not queued — no
  per-group pending-queue type built this pass).
- Reconciling the perpendicular velocity component on an `Aborting`
  group-move redirect (documented limitation, live-tested to confirm it's
  safe, not silently fixed).
- Groups larger than 2 axes (the framework is N-generic via `Vec`/
  `MAX_GROUP_AXES`, but only `axisGroup0` = {axis0, axis1} is configured;
  no live 3+-axis smoke test this pass).
- Runtime group creation/removal — hard-coded only, per the explicit
  request that this be simpler than PLCopen's model.
- Any unification with the shelved `WaypointPath`/`PathProfile` spline
  feature (noted as a natural future direction, not built toward now).
- The tiny clock-skew between group members' independently-set
  `started_at: Instant::now()` calls, and the redundant (but cheap)
  per-member recomputation of the same shared scalar sample each cycle —
  both negligible at 250 Hz / N≤6, not worth a shared-cache mechanism now.

## Files

- `motion-core/src/linear_move.rs` — new: `LinearMoveError`, `LinearMoveSample`, `LinearMove`, `MAX_GROUP_AXES`.
- `motion-core/src/lib.rs` — new module declaration + re-exports.
- `app/src/main.rs` — `handle_enable`/`handle_disable`/`handle_reset`/`handle_stop` extraction, `AXIS_GROUPS`, `Target`/`parse_target`, new `Command::*Group` variants, `SharedGroupMove`, `Profile::Group`, cascade logic, help/status text, `parse_command`/`parse_axis` unit tests.
- `CLAUDE.md` — document each stage once built, per this project's existing habit.

## Verification

- `cargo test -p motion-core` — the full Stage 1 test list above, standalone, before Stage 2/3 begin.
- `cargo test -p app` — new `parse_command`/`parse_target` tests (Stage 2).
- **Live smoke tests against the built binary directly** (`./target/debug/app`, not `cargo run` — this project's known piped-stdin timing gotcha):
  1. `enable axisGroup0` → `move axisGroup0 <x> <y> ...` → `stop axisGroup0` — confirm fan-out messages match single-axis message text/shape, and confirm the viz XY plot traces a straight line.
  2. Aligned-velocity abort: redirect a moving group (`aborting`) to a new target that's colinear with its current velocity — confirm a smooth redirect, no discontinuity.
  3. Perpendicular-velocity abort: same, but to a target requiring a genuinely different direction — confirm the documented velocity snap is visible but produces no NaN/crash/panic.
  4. Cascading interruption: `stop axis0` while `axisGroup0`'s move is active — confirm `axis1` independently decelerates via its own `StopRamp`, with distinct per-axis messages (not a combined group message).
  5. Buffered-group-busy rejection: issue a second `move axisGroup0 ...` (default `buffered`) while the first is still active — confirm rejection (not queueing) with a message explaining why.
