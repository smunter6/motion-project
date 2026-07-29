# motion-project

Motion planning / trajectory generation for a small robot, built to eventually
drive **EtherCAT servo drives** (EtherCRAB + CiA 402, CSP mode) while running
hardware-free in **simulation** with live visualization. Sim mode is a
first-class target, not a stopgap — there is no hardware yet.

A learning project, built one deliberate step at a time. This file is the
*what and how*. The *why* — every design decision, tradeoff, and accepted
limitation — lives in [`CLAUDE.md`](CLAUDE.md), which is the more interesting
document and the one to read before changing anything.

## Workspace

```
motion-core/     I/O-free, dependency-free pure math: profiles, paths, kinematics
axis-backend/    the trait seam: AxisGroup::exchange(setpoints) -> feedback
backend-sim/     software plant implementing the seam (dt-stepping, DS402 model)
app/             the interactive control loop: terminal commands + egui viz
```

`backend-ethercat` (EtherCRAB + CiA 402) slots in behind the same trait when
hardware arrives. Nothing above it should need to change.

### `motion-core`

No I/O, no dependencies, no allocation in the per-cycle path — so the same code
is host-testable in WSL and reusable unchanged on a Pi (or an RP2350 later).

| Module | What it provides |
| --- | --- |
| `trajectory` | `TrapezoidalProfile` (incl. non-zero start velocity, for redirects), `StopRamp` |
| `jerk_filter` | `JerkFilteredProfile` — jerk limiting by *filtering* a trapezoid with a rectangular window, not a 7-segment S-curve |
| `linear_move` | `LinearMove`: straight-line N-axis move — one scalar profile over Euclidean distance |
| `waypoint_path` | `WaypointPath`: centripetal Catmull-Rom spline through waypoints, with per-segment arc-length LUTs |
| `path_profile` | `PathProfile`: speed profile over a path's arc length; reports centripetal as well as tangential acceleration |
| `kinematics` | `KinematicModel` seam, `IdentityKinematics`, `ScaraKinematics` (FK/IK/Jacobian, plus `inverse_acceleration` with the `J̇q̇` term) |

The recurring pattern throughout: reuse one scalar profile over a scalar path
coordinate, rather than deriving a new profile type per move kind.

### Key invariants

- **Trajectory is a pure function of elapsed time**; the *plant* is the
  stateful dt-stepping half. They meet at the setpoint each cycle.
- **`f64` everywhere in the core.** Encoder-count conversion is the backend's
  job at the hardware seam.
- **The planner plans from commanded state, never from feedback.** Feedback is
  for reporting. Seeding a profile from measured position injects a step into
  the commanded stream and launders following error into the plan.
- Control rate **250 Hz** (4 ms cycle); groups bounded by `MAX_GROUP_AXES = 6`.

## Running it

```sh
cargo test                      # host-side unit tests, the bulk in motion-core
cargo run -p motion-core --bin demo_axis   # print one axis's profile as numbers
cargo build -p app && ./target/debug/app   # interactive, with viz window
./target/debug/app --headless              # no window, loop on the main thread
./scripts/demo_session.sh                  # scripted non-interactive session
```

Two notes that will otherwise cost you an afternoon:

- **Verify interactive changes against `./target/debug/app`, not `cargo run`** —
  there's a piped-stdin timing gotcha.
- **Anything scripted runs `--headless`.** The viz window needs a display
  server, dominates startup, and outlives the control loop. Headless runs the
  identical loop (recording included) and needs no startup sleep. See
  [`app/CLAUDE.md`](app/CLAUDE.md) for the WSL renderer setup if you do want a
  window.

## The machine it simulates

Configured at compile time in `app/src/main.rs` (`AXIS_CONFIGS`, `AXIS_GROUPS`);
`setlimits` edits a runtime copy that lasts until exit.

| | Axes | Kinematics | Units |
| --- | --- | --- | --- |
| `axisGroup0` | `axis0`, `axis1` | identity (Cartesian XY stages) | mm |
| `axisGroup1` | `axis2`, `axis3` | SCARA, two 100 mm links | **rad** |

Group coordinates are always Cartesian TCP mm; a single-axis `move axisN` is
raw joint space in that axis's own units. So `move axis3 0.5` means 0.5 rad.
The SCARA bakes in a `q2 + π/2` home offset — raw `q2 = 0` is *not* a straight
arm.

## Command language

`help` prints this list live; targets are an axis (`axis0`) or a group
(`axisGroup0`).

```
move <target> <coord>... [vmax] [amax] [dmax] [aborting|buffered]
movepath <group> <n> <coord>...        [limits] [aborting|buffered|blend]
movepath <group> file <path>           [limits] [aborting|buffered|blend]
stop <target> [decel]
setlimits <target> [speed|accel|decel|jerk <value>]...
enable | disable | reset <target>
status | verbose | help | quit
```

- Limits are **named, not positional** (`setlimits axis0 accel 100 jerk 500`),
  and `jerk` accepts `none`. `setlimits <target>` alone just reports.
- `max_jerk` is config-only — a `move` can override speed/accel/decel, never
  jerk.
- `enable` takes 3 control cycles (~12 ms) before a `move` is accepted; scripted
  sessions need a short sleep between them.
- `verbose` toggles only the per-cycle heartbeat. Discrete events (moves,
  faults, DS402 transitions) always print.

## Visualization

egui/eframe + `egui_plot`, baked into `app` as `viz.rs` + `recording.rs`. It is
passive and read-only — input stays terminal-only. It taps the
`AxisGroup::exchange()` seam via `RecordingAxisGroup`, so it sees setpoints and
feedback but nothing of `app`'s higher-level bookkeeping; only `status` can tell
you whether a *synchronized group move* is active. Plane plots are drawn for
2-axis groups only (`egui_plot` is strictly 2D).

## State of play

Built: trapezoidal and jerk-limited profiles, redirect-with-start-velocity,
stop ramps, straight-line and spline path moves with buffering/blending, the
kinematics seam with a working SCARA, the sim backend with a real DS402 state
machine and faults, per-axis and per-group runtime limits, group interrupt
cascades, and viz.

Not built, and worth knowing before you trust it:

- **Nothing bounds centripetal acceleration.** One scalar `max_speed` over arc
  length; `v²κ` is unchecked and today's demo path exceeds `max_acceleration`
  by roughly 3×.
- **No joint-rate limiting and no mid-path reachability checking.** Validation
  is install-time endpoints only, plus a reactive cascade stop. These plus
  curvature-limited feedrate are one problem — the next phase (roadmap 7).
- **Paths are G1, never G2.** Curvature steps at every waypoint; inherent to
  Catmull-Rom. Fixed by moving to Yuksel C2 splines (roadmap 8), not by tuning.
- **Coordinated multi-axis time-scaling is deferred.** `app` drives
  independently issued single-axis moves without synchronizing their durations.
  Axis *groups* are a different thing and do exist.
- The SCARA's equal links put a reachable singularity at the origin. "Don't
  command a move across it" is a convention observed by tests and demo targets,
  not enforced by code.
- `backend-sim` integrates commanded velocity and models no mechanics, so it
  **hides** whole classes of bug — a commanded-position step, an off-branch IK
  jump, and most of what jerk limiting buys all show up only as a small
  target-vs-actual gap here, and as an immediate following-error fault on real
  hardware. Severity is inverted between sim and the real thing.

Full roadmap, with sequencing rationale, is in [`CLAUDE.md`](CLAUDE.md).

## Deployment

Develop and test in WSL on the Linux filesystem (not `/mnt/c`). **Real EtherCAT
bus testing happens on the Pi** — WSL2's virtualized NAT networking can't do raw
L2 EtherCAT reliably. See the `deploy-to-pi` skill for the cross-compile loop.
