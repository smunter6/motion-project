# Extend TrapezoidalProfile for nonzero-start-velocity moves; add PLCopen BufferMode (Aborting/Buffered)

## Context

`stop` (built this session) showed that interrupting a move works cleanly *when the new
thing has no target* — decelerate current velocity to rest. Genuine move-over-move
interruption doesn't have that luxury: it needs a real target position, starting from
wherever the axis actually is with whatever velocity it actually has. `TrapezoidalProfile`
today only knows how to plan rest-to-rest; that's exactly the gap the
`plcopen-motion-goal` memory flagged as deferred back when the online app was first being
designed.

The user's direction: don't work around that gap with a second type (the way `StopRamp`
sits beside `TrapezoidalProfile` rather than inside it) — extend `TrapezoidalProfile`
itself, and shape the command interface after PLCopen's real `BufferMode` concept (which
every buffered motion FB takes as a parameter): `Aborting`, `Buffered`, and four blending
modes (`BlendingLow`/`Previous`/`Next`/`High`).

Scope for this pass (confirmed with the user): **`Aborting` + `Buffered` only.** Blending
needs a profile that's aware of an *adjacent* segment and never fully decelerates before
handing off — a structurally different, harder problem — and gets its own design pass
once this lands.

## Design

### 1. `motion-core`: `TrapezoidalProfile` gets a nonzero-start-velocity path

Keep `TrapezoidalProfile::new(start, end, max_speed, max_acceleration, max_deceleration)`
exactly as it is — every existing call site and all 24 current motion-core tests keep
passing unchanged, since it becomes a thin wrapper (`start_velocity: 0.0`) around a new,
more general constructor:

```rust
pub fn new_with_start_velocity(
    start: f64,
    start_velocity: f64,
    end: f64,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
) -> Result<Self, TrajectoryError>
```

(Exact name TBD together — this is a placeholder, not a commitment.)

This has to handle three kinematic cases, worked through and derived together the same
way the original trapezoid math was, not dropped in as a black box:

- **(a) Same direction, room to spare** — `start_velocity` already points toward `end`
  and there's enough remaining distance to reach it without exceeding `max_speed` or
  needing a harder stop than `max_deceleration` allows. A trapezoid/triangle that starts
  partway up the ramp (or already cruising) instead of from rest.
- **(b) Same direction, not enough room** — moving toward `end`, but too fast to stop
  before reaching it at `max_deceleration`. Physically has to decelerate through the
  target and come back: decelerate to zero, then a fresh rest-to-rest segment behind.
  A real, common case for an aggressive interrupt, not an edge case to skip.
- **(c) Opposite direction** — `start_velocity` points away from `end`. Decelerate to
  zero first, then a normal rest-to-rest move from wherever that zeroing lands.

**All of it — every phase, including the decel-to-zero/reversal preamble in (b) and
(c) — uses the *new* move's `max_speed`/`max_acceleration`/`max_deceleration` exclusively.**
Once a move is aborted into a new one, the old move's kinematic limits are entirely
discarded; there's no blending or carrying over of the superseded move's rates. Within
that: "decelerating" (losing speed, whichever direction) uses the new `max_deceleration`,
"accelerating" (gaining speed toward `end`) uses the new `max_acceleration` — applied by
what the axis's speed is doing at that instant, not by which move commanded it.

(b) and (c) both reduce to "kill the existing velocity (at the new move's decel rate),
then a fresh segment (at the new move's accel/decel rates)" — conceptually close to a
`StopRamp` preamble feeding a `TrapezoidalProfile`, but must live as one cohesive type
with a single `sample`/`phase_at`/`duration`, not two objects `app` stitches together —
that's the actual ask.

`TrajectoryError` gains whatever new finite-input variant `start_velocity` needs (same
pattern as `StopRamp`'s `NonFiniteStopState`).

New tests: each case, both directions of travel, alongside confirming the existing
rest-to-rest tests are untouched.

### 2. `app`: `BufferMode` and a real queue

`BufferMode` lives in `app` next to `Command`, not in `motion-core` — it's dispatch/queuing
*policy* (which command wins, what happens to the queue), not trajectory math.
`motion-core` stays exactly "pure kinematics, no policy."

```rust
enum BufferMode {
    Aborting,
    Buffered,
}
```

(The four blending variants join later, once their profile-pairing design exists.)

- `Command::Move` gains `buffer_mode: BufferMode`. Terminal syntax: a trailing optional
  keyword after the existing three positional numeric args —
  `move <axisN> <target> [vmax] [amax] [dmax] [aborting|buffered]` — defaulting to
  `Buffered` so today's default behavior is unchanged for anyone who doesn't specify it.
- `AxisRuntime.pending: Option<PendingMove>` becomes `pending: VecDeque<PendingMove>` — a
  real FIFO queue. **This is a deliberate behavior change**, not a side effect: today,
  queuing a second move while one's already queued replaces the first ("only the most
  recent queued move is kept," per the current help text); going forward, `Buffered` moves
  queue in order and all run.
- `BufferMode::Buffered` while busy → `ax.pending.push_back(...)`.
- `BufferMode::Aborting` → takes effect immediately regardless of idle/busy: clears
  `ax.pending` (queued items don't survive an abort, same as `stop` already does today)
  and replaces `ax.active` right now, building the new profile from `ax.position`/
  `ax.velocity` (actual backend feedback, not commanded state) via
  `new_with_start_velocity`.
- Extract a small shared helper for "clear pending, build a profile from actual state,
  replace `ax.active`, report a build error the same way" — both `Command::Stop` and
  `BufferMode::Aborting` need exactly this, and duplicating it a third time isn't worth it.
  Exact shape decided during implementation.
- Step 1 of the control loop (queue promotion) moves from `.take()` to `.pop_front()`. The
  "axis not enabled" case drains and drops the *whole* queue with one summary message,
  rather than one drop-message per queued item per cycle.
- Update `print_help`/module doc comment (the "only the most recent queued move is kept"
  line is no longer true) and any `status`/print wording that assumed a single queued move.

## Files

- `motion-core/src/trajectory.rs` — new constructor + cases + tests; `TrajectoryError` variant.
- `motion-core/src/lib.rs` — export update if the new constructor needs it (likely not, same type).
- `app/src/main.rs` — `BufferMode` enum, `Command::Move` field, parser, `PendingMove` queue
  type change, step 1 promotion logic, `Command::Move`/`Command::Stop` handlers, help text.

## Verification

- `cargo test --workspace` — full suite, including new motion-core cases; confirm the
  existing 24 motion-core / 20 backend-sim / 2 axis-backend tests are all still green.
- Live smoke tests via the *built binary directly* (`./target/debug/app`, not
  `cargo run` — this session already hit `cargo run`'s startup overhead racing ahead of
  `sleep`-paced piped stdin):
  - `move` with `aborting` replacing an in-progress move, both same-direction and
    opposite-direction retarget, checking the reported position/duration math by hand.
  - `move` with `buffered` queuing 2+ moves behind a busy axis, confirming they run in
    order.
  - Confirm `stop`/`enable`/`disable`/`reset`/fault behavior is unchanged (regression).
