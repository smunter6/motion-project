# CLAUDE.md — project context for Claude Code

This file orients a fresh Claude Code session. It was carried over from an
earlier planning/design conversation. Read it before making changes.

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
   the same (future) `AxisGroup` trait. Sim now, EtherCAT later, swappable.
2. **`motion-core` is I/O-free and dependency-free.** Pure math only
   (trajectory, kinematics, types). This is what we unit-test on the host and
   reuse unchanged on the Pi / potentially RP2350. Nothing touching a NIC, a
   drive, or a window goes in here.
3. **Absolute-time trajectory vs. dt-stepping plant.** The trajectory generator
   is a *pure function of elapsed time* (exact, testable, jitter-robust). The
   *stateful, dt-stepping* model belongs to the plant layer (sim model / real
   servo), added later. They meet at the setpoint each control cycle. See the
   `NOTE (dt seam)` comment in `motion-core/src/trajectory.rs::sample`.
4. **f64 in the core.** Real drives use integer encoder counts; that conversion
   is the backend's job at the hardware seam, not the planner's.
5. **Kinematics is pluggable.** Start Cartesian (trivial), but keep the seam so
   other kinematic models (e.g., 2-link arm) can drop in later.
6. **Control rate: 250 Hz** (4 ms cycle) to start.

## Deployment context (for later steps)

- Develop/test in **WSL** (Linux filesystem, not /mnt/c). Cross-compile for a
  Raspberry Pi target (`aarch64-unknown-linux-gnu` for 64-bit Pi OS) via
  `cross` or `cargo-zigbuild`. Deploy loop: build → rsync → ssh run.
- Real EtherCAT bus testing happens on the **Pi**, not in WSL (WSL2's
  virtualized NAT networking can't do raw L2 EtherCAT reliably). WSL is for
  code + host-side unit tests of pure logic.
- EtherCRAB is `no_std`-capable, so the same crate could later target an
  RP2350-based master with an embedded TX/RX transport.

## Current state — Steps 1–5 done (2 deferred)

- `motion-core`: single-axis **trapezoidal velocity profile**,
  `TrapezoidalProfile::new(start, end, max_speed, max_acceleration,
  max_deceleration) -> Result<Self, TrajectoryError>` (fallible — validates
  finiteness and sign, doesn't panic on bad input). Accel/decel are
  independent rates. `.sample(t)`, `.duration()`, `.target()`, `.phase_at(t)`.
  `src/bin/demo_axis.rs` demos one move at 250 Hz. Also **`StopRamp`**
  (added 2026-07-20): the rest-to-rest assumption baked into
  `TrapezoidalProfile` (see `plcopen-motion-goal` memory) can't represent a
  running-start deceleration, so this is a separate, much simpler type —
  current-velocity-to-rest, one phase, no target position — rather than a
  generalization of `TrapezoidalProfile`. Same `sample`/`phase_at`/`target`
  shape; a zero-velocity ramp reports `Done` immediately (not `Pre` forever,
  unlike `TrapezoidalProfile`'s zero-distance case — see its doc comment for
  why the two cases differ). Also
  **`TrapezoidalProfile::new_with_start_velocity(start, start_velocity, end,
  max_speed, max_acceleration, max_deceleration)`** (added 2026-07-21): this
  *does* extend `TrapezoidalProfile` itself rather than adding a second type
  — the user's explicit correction, overriding the `StopRamp` precedent (see
  `plcopen-motion-goal` memory). Handles interrupting an in-flight move with
  a new target (backs `app`'s `BufferMode::Aborting`), covering three cases
  through one unified internal path rather than a case-by-case dispatch: a
  private `build_main_segment` helper generalizes the rest-to-rest
  ramp/cruise/decel math to start from an arbitrary speed (including the
  sub-case where `start_velocity` already exceeds the *new* move's
  `max_speed`, so phase 1 decelerates into cruise instead of accelerating),
  and an optional decel-to-rest "prefix" (`StopRamp`-style math) handles
  both the same-direction-overshoot and opposite-direction cases — which
  turn out to be the identical code path, distinguished only by which side
  of the target the deceleration happens to land on. The new move's own
  max_speed/max_acceleration/max_deceleration govern the *entire* resulting
  profile, including any reversal preamble — no blending or carryover from
  whatever move was superseded. `new()` is now a verified bit-identical thin
  wrapper (`start_velocity: 0.0`). 32 unit tests total.
- `axis-backend`: the `AxisGroup` trait seam (`exchange()`, one combined
  cyclic call) + `AxisSetpoint`/`AxisFeedback`/`AxisFault`/`AxisGroupError`.
  `AxisSetpoint` carries `enabled: bool` and `fault_reset: bool` — two
  distinct cyclic fields, like real CiA 402 controlword bits (enable vs. the
  edge-triggered Fault Reset bit), not one-off commands. `AxisFeedback`
  carries a motion-control state machine at two layers: `AxisState`
  (PLCopen `MC_ReadStatus`-flavored: `Disabled`/`StandStill`/
  `DiscreteMotion`/`ErrorStop`/etc.) and, one layer more detailed,
  `Ds402State` (the real CiA 402 power-state machine: `SwitchOnDisabled` ->
  `ReadyToSwitchOn` -> `SwitchedOn` -> `OperationEnabled`, plus
  `FaultReactionActive`/`Fault` — both now reachable — and reserved
  `QuickStopActive`) — `Ds402State::axis_state()` maps the former from the
  latter. Design settled 2026-07-20: the DS402 detail mirrors what
  PLCopen's own `ST_AxisStatus` field exists for (vendor/backend-specific
  detail beneath the standard bits), and lives at the trait level (not just
  inside `backend-sim`) since `backend-ethercat` will need the same shape
  later. Also carries `MotionFlags`
  (`accelerating`/`constant_velocity`/`decelerating`). `AxisFault` has one
  real category so far: `DisabledWhileMoving`. `AxisSetpoint` also carries
  `stopping: bool` (added 2026-07-20, for the `stop` command): PLCopen
  `MC_Stop` and DS402 Quick Stop are deliberately different things — a
  commanded stop only flips the coarser `AxisState` to `Stopping` (replacing
  `DiscreteMotion`), `Ds402State` is untouched and stays `OperationEnabled`,
  since the drive itself sees nothing but an ordinary decelerating velocity
  setpoint. `Ds402State::QuickStopActive` stays reserved — that's a distinct,
  typically emergency/safety-triggered mechanism, not implemented.
- `backend-sim`: `SimAxisGroup`, a software `AxisGroup` that integrates
  velocity into position each cycle (dt-stepping, not a pass-through), but
  only while `OperationEnabled` — a disabled (or faulted) axis ignores
  commanded velocity entirely, matching a real drive's power stage being
  off. Steps the DS402 machine at most one transition per `exchange()`
  cycle in either direction (enabling takes 3 cycles, ~12ms at 250 Hz), the
  same way a real master/drive negotiate it one controlword write at a
  time — not a simplification for its own sake, see `step_ds402`'s doc
  comment. Design settled 2026-07-20: disabling a *moving* axis raises
  `AxisFault::DisabledWhileMoving` and routes through
  `FaultReactionActive` (held exactly one cycle) into latched `Fault`,
  rather than gracefully stepping down — a real drive can't safely cut its
  power stage mid-motion the way it can from rest. `Fault` only clears via
  `fault_reset` (DS402 transition 15, to `SwitchOnDisabled`); resetting
  does not itself re-enable the axis. Disabling from `StandStill` (never
  actually moving) stays graceful, no fault. A commanded stop
  (`AxisSetpoint::stopping`) only ever overrides `DiscreteMotion` ->
  `Stopping` in the reported `AxisState`; `Ds402State` and the actual
  velocity-integration physics are unaffected — `backend-sim` doesn't know
  or care *why* a decelerating velocity was commanded, only whether it was.
  20 unit tests total, including dedicated coverage of the enable/disable
  sequence, the fault/reset path, and the stopping-flag override (dev-
  dependency on `motion-core` lets some tests drive a real
  `TrapezoidalProfile`'s sampled velocity through the sim rather than
  hand-crafted sequences).
- `app`: a continuously-running, multi-axis (`axis0`, `axis1`, `NUM_AXES`)
  terminal app — `move <axisN> <target> [vmax] [amax] [dmax]
  [aborting|buffered]`, `stop <axisN> [decel]`, `enable <axisN>`, `disable
  <axisN>`, `reset <axisN>`, `status`, `help`, `quit`. Fixed 250 Hz control
  loop on its own thread, decoupled from blocking stdin by a channel. Each
  cycle samples the active trajectory (or holds at rest if idle) into an
  `AxisSetpoint` per axis, calls `SimAxisGroup::exchange` once, and treats
  the returned feedback as ground truth for position/velocity — a new
  move's start is the axis's *actual* (backend) position, not the last
  commanded one. Axes are fully independent — see Step 2, deferred. Axes
  start disabled (DS402
  `SwitchOnDisabled`, matching real drive power-up); `move` on a disabled
  axis is rejected, not queued. Disabling a moving axis faults it (position
  freezes where it was, `Command::Enable` refuses outright while faulted —
  not just while `SwitchOnDisabled` — so a premature `enable` sent during
  the fault window can't silently "stick" and auto-fire the instant a later
  `reset` clears it); `reset` clears the fault but requires a fresh
  `enable` afterward, same as the backend. DS402 transitions print to the
  terminal as the backend confirms them, and `status` shows the current
  `Ds402State`.
  **`stop`** (added 2026-07-20, the first piece of interrupting a move in
  progress rather than queuing behind it — a single-axis concern, distinct
  from Step 2's multi-axis coordination; see the `plcopen-motion-goal`
  memory's long-term abort/buffer/blend goal): builds a
  `motion_core::StopRamp` from the axis's *actual*
  position/velocity and replaces whatever's active (also clearing anything
  queued). A local `Profile` enum (`Move(TrapezoidalProfile)` /
  `Stop(StopRamp)`) is what lets the rest of the loop treat "whatever's
  currently active" uniformly — completion/abort/fault messages still
  differentiate ("stopped at" vs "reached", "stop faulted" vs "move
  faulted") via `Profile::is_stop()`. Rejected on a disabled axis; a
  short-circuit for an axis already at rest. `stop` deliberately does *not*
  touch `Ds402State`/DS402 Quick Stop — see the `axis-backend` entry above.
  **`BufferMode` (added 2026-07-21, see `plcopen-motion-goal` memory)**:
  `Command::Move` takes an optional trailing `aborting`/`buffered` keyword
  (default `Buffered`, so omitting it keeps prior behavior).
  `AxisRuntime.pending` is a real `VecDeque<PendingMove>` FIFO now (was a
  single replace-only slot) — a deliberate behavior change, every buffered
  move queued eventually runs, in order, rather than only the most recent
  being kept; queue promotion pops and tries moves one at a time (a
  kinematically-invalid one is dropped and the next tried the same cycle,
  rather than retried forever), and an axis that stops being enabled drops
  its *entire* queue at once with one summary message.
  `BufferMode::Aborting` bypasses the queue entirely — active or idle,
  immediately — via a shared `abort_into` helper (clears `pending`, builds a
  fresh profile from the axis's *actual* position/velocity, replaces
  `ax.active`, reports build errors uniformly) also used by `stop`.
  `Aborting` moves are built with
  `motion_core::TrapezoidalProfile::new_with_start_velocity`, so an
  in-flight move can be redirected to a new target — same-direction or
  reversed — without waiting for it to reach rest first. Only
  `Aborting`/`Buffered` so far; the four PLCopen blending variants are a
  deferred, structurally different problem (see the memory).
  **`verbose`** toggles the periodic per-cycle position/phase heartbeat
  printed while an axis is moving (off by default — the viz window already
  plots target-vs-actual continuously, so it's usually redundant on the
  terminal). Discrete events (move started/finished/aborted, enable/
  disable, faults, DS402 transitions) always print regardless; only the
  repetitive heartbeat is gated.
  **Layering rule (settled 2026-07-21, see `feedback_axis_state_layering`
  memory)**: `app`'s business logic — move/stop rejection, queue
  promotion, `enable`/`disable`/`reset` gating — checks `AxisState` only,
  via a small `axis_operational()` helper, never `Ds402State` directly.
  `AxisRuntime` carries both fields, but `ds402_state` is read *only* to
  detect and print DS402 transitions for traceability; nothing branches on
  it. Keeps `app` backend-agnostic the same way the trait seam itself is —
  a `backend-ethercat` swap-in only has to report the same `AxisState`
  values, not replicate `backend-sim`'s exact DS402 timing.
- `app`'s viz window (`app/src/viz.rs` + `app/src/recording.rs`): an
  eframe/egui_plot window baked into the `app` binary itself, not a separate
  crate. Input stays entirely terminal-driven — viz has no controls, it only
  plots. `RecordingAxisGroup` (`recording.rs`) wraps the real backend and
  taps the `AxisGroup::exchange()` seam: it forwards every call unchanged and
  copies each cycle's setpoint/feedback into a shared, bounded `History`
  buffer. `run_control_loop` needed zero logic changes — only its backend
  construction line changed to wrap `SimAxisGroup` in `RecordingAxisGroup`.
  Shows target-vs-actual position/velocity per axis, plus an XY tool-position
  plot when `NUM_AXES >= 2`.
- Not yet built: richer sim dynamics (Step 6), `backend-ethercat` (Step 7),
  coordinated multi-axis moves (Step 2, deferred).

## Roadmap

1. [done] One axis: trapezoidal trajectory + invariant tests.
2. **[deferred]** Two axes + coordination (finish-together time-scaling; the
   slower axis sets the move time, the faster axis is time-scaled to match).
   This is where `duration()` starts earning its keep. Explicitly on hold —
   `app` currently drives axes independently (no coordination) and that's
   staying true until this is revisited. Don't build toward coordinated
   moves as a side effect of Step 3/4/5 work below.
3. **[done]** `AxisGroup` backend trait (`axis-backend` crate) + a simple
   **sim backend** (`backend-sim` crate). Design settled 2026-07-20:
   - `AxisGroup::exchange(&mut self, setpoints: &[AxisSetpoint]) ->
     Result<&[AxisFeedback], AxisGroupError>` — one combined cyclic call, not
     separate write/read. Mirrors EtherCAT's actual single synchronous
     transaction per cycle (EtherCRAB's `tx_rx()`), so `backend-ethercat`
     (Step 7) won't need the trait reshaped later.
   - Trait stays in `f64` engineering units (mm, mm/s) — integer encoder-count
     conversion is `backend-ethercat`'s private business (CLAUDE.md decision
     #4), not the trait's.
   - `AxisFeedback` carries `fault: Option<AxisFault>` from day one, even
     though `backend-sim` won't raise any yet — lets `app`'s fault-handling
     path get built/exercised before real hardware exists.
   - `backend-sim`'s first plant model integrates the velocity setpoint into
     its own position state each cycle (`position += velocity * dt`) rather
     than a pure pass-through. Still trivial/lag-free, but genuinely stateful
     dt-stepping (see the `NOTE (dt seam)` comment in `trajectory.rs`), and
     gives a real (if tiny) target-vs-actual gap during accel/decel for
     Step 5's viz to plot, instead of two identical lines until Step 6.
4. **[done]** Fixed-timestep run loop: planner -> backend -> feedback. `app`'s
   control loop now builds an `AxisSetpoint` per axis each cycle (sampling
   the active trajectory, or holding at rest if idle), calls
   `AxisGroup::exchange` once per tick against a `SimAxisGroup`, and treats
   the returned `AxisFeedback` — not the raw trajectory sample — as each
   axis's actual position/velocity. A new move's `start` is that actual
   feedback position, not the last commanded one. Confirmed live: a 0->100mm
   move now reports "reached 99.999 mm", the small Euler-integration
   following error `backend-sim`'s design was meant to surface.
5. **[done]** Simple **viz** (egui/eframe + egui_plot): target-vs-actual
   position/velocity plots per axis, plus an XY tool-position plot. Viz is a
   dev tool, runs on host, not deployed to Pi. Design settled 2026-07-20,
   resolving the loop-sharing question in favor of the smallest option:
   - **Baked into `app`, not a separate crate or shared "runtime" crate.**
     `app` gained two modules (`recording.rs`, `viz.rs`); no new workspace
     member. Terminal input is unchanged — viz is a passive, read-only
     window, not a second frontend with its own input handling.
   - **Data comes from tapping the `AxisGroup` seam, not from touching
     `run_control_loop`'s decision logic.** `RecordingAxisGroup` wraps
     whatever backend it's given, forwards `exchange()` unchanged, and
     records setpoint/feedback into a shared `History`. The loop only
     changed what it constructs the backend as.
   - **Thread layout flipped.** eframe/winit require the GUI event loop on
     the main thread, so `run_control_loop` moved onto its own spawned
     thread (alongside the existing stdin-reader thread); `main()`'s thread
     now runs `eframe::run_native`. The terminal `quit` command sets a
     shared `AtomicBool` that viz polls each frame to close its own window.
   - **Renderer: `glow` (OpenGL), not the default `wgpu`.** `wgpu` failed at
     startup in this WSL setup (`WinitEventLoop(ExitFailure(1))`) — no
     `/dev/dri` render node, no Vulkan ICD, only WSL's `/dev/dxg` GPU
     passthrough. `glow` via Mesa's GL (through `/dev/dxg`) works. Also
     dropped eframe's default `accesskit` feature (AT-SPI/D-Bus screen-reader
     integration, unneeded for a dev plotting tool, and was itself hitting a
     missing D-Bus session daemon during startup diagnosis).
   - **Environment note for a fresh WSL setup**: running a GUI app needs
     `libwayland-client0 libwayland-egl1 libwayland-cursor0 libxkbcommon0
     libegl1 libgl1` installed (`apt-get install`) even with WSLg present —
     WSLg provides the compositor, not the client-side libraries.
6. Richer sim: second-order lag, position/following-error limits, faults;
   S-curve (jerk-limited) profiles.
7. (Hardware later) `backend-ethercat`: EtherCRAB + CiA 402 state machine +
   PDO mapping, behind the same `AxisGroup` trait.

## Housekeeping notes

- [done] `app` binary crate exists (arrived early, ahead of Step 4/backend-sim —
  see `app/src/main.rs`); `Cargo.lock` is committed, `.gitignore` no longer
  excludes it.
- Workspace layout target: motion-core / axis-backend / backend-sim /
  backend-ethercat / app. (`viz` is not a separate crate — see Step 5: it's
  baked into `app` as `app/src/viz.rs` + `app/src/recording.rs`.)
- **Backlog**: unit tests for `app`'s `parse_command`/`parse_axis` (in
  `app/src/main.rs`, `#[cfg(test)] mod tests`, same pattern as
  `trajectory.rs`). Currently only manually smoke-tested via piping stdin.
  Pure parsing logic, no threading/timing involved — cheap to add whenever
  we're back in `app`. (Full control-loop integration testing — queuing,
  per-axis independence — is a separate, harder problem: real `sleep()`s and
  `println!`-format assertions make it slow/brittle; revisit only if that
  becomes a recurring pain point, and consider extracting the loop's
  decision logic to run on fake time first.)

## How to work in this repo

Small steps. Before each step, briefly discuss what we're building and why, then
implement, then verify with `cargo test`/`cargo run`. Flag design tradeoffs
explicitly and let the user choose at decision points. Keep `motion-core`
pure. Preserve the architectural decisions above unless we explicitly revisit
one.

NEVER use sed, awk, or cat to read or edit files. Always use the built-in
Read, Edit, and Write tools.
