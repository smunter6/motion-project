# CLAUDE.md — project context for Claude Code

This file orients a fresh Claude Code session. It was carried over from an
earlier planning/design conversation. Read it before making changes.

## What this project is

A **learning project** (go one deliberate step at a time; explain the reasoning,
don't just emit code) building motion planning / trajectory generation for a
simple Cartesian robot. The end goal is to drive **EtherCAT servo drives** (via
the pure-Rust **EtherCRAB** master + the **CiA 402** drive profile, CSP mode)
while supporting a **hardware-free simulation mode** with simple visualization.
The user does not currently have the hardware, so sim mode is a first-class
requirement, not an afterthought.

## Who I'm working with

Experienced engineer/scientist, comfortable with deep technical detail, new-ish
to Rust embedded/motion specifics. Prefers: understanding *why* before code,
small steps with discussion between them, honest flagging of design tradeoffs
and anything not yet compiler-verified. Do not over-produce; confirm direction
at natural decision points.

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

## Current state — Step 1 complete

`motion-core` contains the single-axis **trapezoidal velocity profile**:
- `TrapezoidalProfile::new(start, end, max_velocity, max_acceleration)`
- `.sample(t) -> TrajectorySample { position, velocity }` — pure, absolute-time,
  clamped outside `[0, duration]`.
- `.duration()`, `.target()`, `.phase_at(t) -> MotionPhase`.
- Handles both trapezoidal and (short-move) triangular cases.
- Full invariant unit tests (start/end at rest, never exceeds v_max, symmetry,
  triangular case, negative direction, phase-boundary continuity, phase_at).
- `src/bin/demo_axis.rs`: samples a 0->100mm move at 250 Hz and prints
  time/pos/vel/phase.

NOTE: this code was written in an environment WITHOUT a Rust toolchain, so it was
hand-verified, not compiler-verified. First task in Claude Code: run
`cargo test` and `cargo run -p motion-core --bin demo_axis` and fix anything the
compiler flags.

## Roadmap

1. [done] One axis: trapezoidal trajectory + invariant tests.
2. **[deferred]** Two axes + coordination (finish-together time-scaling; the
   slower axis sets the move time, the faster axis is time-scaled to match).
   This is where `duration()` starts earning its keep. Explicitly on hold —
   `app` currently drives axes independently (no coordination) and that's
   staying true until this is revisited. Don't build toward coordinated
   moves as a side effect of Step 3/4/5 work below.
3. **[next]** `AxisGroup` backend trait (new `axis-backend` crate) + a simple
   **sim backend** (new `backend-sim` crate). Design settled 2026-07-20:
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
4. Fixed-timestep run loop: planner -> backend -> feedback. `app`'s control
   loop already has this shape but currently treats the raw trajectory
   sample as ground truth; Step 3 lands by making it call
   `AxisGroup::exchange` each tick and use the returned feedback instead.
5. Simple **viz** (egui/eframe + egui_plot): 2D axis/tool view + target-vs-actual
   position/velocity plots. Viz is a dev tool, runs on host, not deployed to Pi.
   (WSLg needed to show a window from WSL; or run viz natively on Windows.)
   Open question, deliberately not decided yet: whether `app` and `viz` share
   the control-loop code (extract to a common crate) or `viz` gets its own
   copy. Revisit when viz actually starts, not before — no second consumer
   of the loop exists yet, so extracting now would be speculative.
6. Richer sim: second-order lag, position/following-error limits, faults;
   S-curve (jerk-limited) profiles.
7. (Hardware later) `backend-ethercat`: EtherCRAB + CiA 402 state machine +
   PDO mapping, behind the same `AxisGroup` trait.

## Housekeeping notes

- [done] `app` binary crate exists (arrived early, ahead of Step 4/backend-sim —
  see `app/src/main.rs`); `Cargo.lock` is committed, `.gitignore` no longer
  excludes it.
- Workspace layout target:
  motion-core / axis-backend / backend-sim / backend-ethercat / viz / app.
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
