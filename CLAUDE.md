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
5. **Kinematics is pluggable.** Start Cartesian (trivial), but keep the seam so
   other kinematic models (e.g., 2-link arm) can drop in later.
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
  support*** — each segment depends only on its 4 nearest control points, so a
  per-segment `SegmentKind::Line` override doesn't perturb any other segment's
  curvature. A natural cubic spline's *global* support would. Centripetal (not
  uniform/chordal) parameterization avoids cusps on non-uniformly-spaced
  waypoints (Yuksel et al. 2011).
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
- **Known, accepted limitation** at a `Line`↔`Spline` segment boundary: the
  spline side's neighbour-derived tangent may not exactly match the line's
  fixed direction — a small velocity-*direction* discontinuity (magnitude stays
  continuous, from the shared scalar profile). Unbuilt fix: make the adjacent
  spline tangent formula use the line's fixed direction.
- **Path limits are a single scalar** max_speed/max_acceleration/
  max_deceleration over arc length. Per-axis limits projected onto the local
  path tangent is a deferred, harder follow-up.

### `axis-backend`

- **One combined `exchange()` call**, not separate write/read. Mirrors
  EtherCAT's actual single synchronous transaction per cycle (EtherCRAB's
  `tx_rx()`), so `backend-ethercat` won't need the trait reshaped later.
- **Two state layers, deliberately.** `AxisState` (PLCopen
  `MC_ReadStatus`-flavored) is the coarse view; `Ds402State` (the real CiA 402
  power-state machine) is the detail beneath it, mirroring what PLCopen's own
  `ST_AxisStatus` exists for. `Ds402State` lives at the *trait* level, not
  inside `backend-sim`, because `backend-ethercat` will need the same shape.
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

- **Feedback is ground truth.** Each cycle the loop treats the backend's
  returned `AxisFeedback` — not the raw trajectory sample — as each axis's
  actual position/velocity. A new move's `start` is that actual position, never
  the last commanded one. Every move-building path follows this rule.
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
6. Richer sim: second-order lag, position/following-error limits, faults;
   S-curve (jerk-limited) profiles.
7. (Hardware later) `backend-ethercat`: EtherCRAB + CiA 402 state machine +
   PDO mapping, behind the same `AxisGroup` trait.

## Known gaps / not built

- Per-segment `SegmentKind::Line` override has no CLI syntax (the type
  supports it; no command wires it).
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

**Verify interactive changes against the built binary** (`./target/debug/app`),
not `cargo run` — there's a known piped-stdin timing gotcha.

NEVER use sed, awk, or cat to read or edit files. Always use the built-in
Read, Edit, and Write tools.
