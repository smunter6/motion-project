# motion-project

Motion planning and trajectory generation for a small robot, written in Rust.
Trajectories run against a software simulation of servo drives (with a CiA 402
state machine) and are plotted live.

Released as-is.

## Workspace

```
motion-core/     I/O-free, dependency-free pure math: profiles, paths, kinematics
axis-backend/    the trait seam: AxisGroup::exchange(setpoints) -> feedback
backend-sim/     software plant implementing the seam (dt-stepping, DS402 model)
app/             the interactive control loop: terminal commands + egui viz
```

### `motion-core`

No I/O, no dependencies, no allocation in the per-cycle path, so it is
host-testable and portable.

| Module | What it provides |
| --- | --- |
| `trajectory` | `TrapezoidalProfile` (incl. non-zero start velocity, for redirects), `StopRamp` |
| `jerk_filter` | `JerkFilteredProfile` — jerk limiting by *filtering* a trapezoid with a rectangular window, not a 7-segment S-curve |
| `linear_move` | `LinearMove`: straight-line N-axis move — one scalar profile over Euclidean distance |
| `waypoint_path` | `WaypointPath`: centripetal Catmull-Rom spline through waypoints, with per-segment arc-length LUTs |
| `path_profile` | `PathProfile`: speed profile over a path's arc length; reports centripetal as well as tangential acceleration |
| `kinematics` | `KinematicModel` seam, `IdentityKinematics`, `ScaraKinematics` (FK/IK/Jacobian, plus `inverse_acceleration` with the `J̇q̇` term) |

## Control loop

`app` runs a stateful control loop against the simulated plant at 250 Hz, set
by `CONTROL_RATE_HZ` in `app/src/main.rs`.

## Running it

```sh
cargo test                                  # host-side unit tests, the bulk in motion-core
cargo run -p motion-core --bin demo_axis    # print one axis's profile as numbers
cargo build -p app && ./target/debug/app    # interactive, with viz window
./target/debug/app --headless               # no window, loop on the main thread
./scripts/demo_session.sh                   # scripted session, headless
./scripts/demo_session_viz.sh               # same session with the viz window
```

Notes:

- **Drive piped input through `./target/debug/app`, not `cargo run`.**
  `cargo run`'s startup races piped stdin.
- **Scripted sessions should use `--headless`.** Headless runs the identical
  loop, including recording.

## Axis configs

Compile-time configuration, all in `app/src/main.rs`.

**Kinematic models** (`motion-core/src/kinematics.rs`)

| Model | Maps | Used by |
| --- | --- | --- |
| `IdentityKinematics` | joints are Cartesian X/Y | `axisGroup0` |
| `ScaraKinematics` | 2-link planar arm, 100 mm links; raw `q2 = 0` is a 90° elbow | `axisGroup1` |

**Axes** (`AXIS_CONFIGS`, one entry per axis)

| Axis | Units | Speed | Accel / decel | Jerk | Travel |
| --- | --- | --- | --- | --- | --- |
| `axis0`, `axis1` | mm | 50 | 200 | 4000 | ±500 |
| `axis2`, `axis3` | rad | 2 | 8 | 160 | unlimited |

**Groups** (`AXIS_GROUPS`)

| Group | Axes | Kinematics | Limits (TCP, mm) |
| --- | --- | --- | --- |
| `axisGroup0` | `axis0`, `axis1` | `IdentityKinematics` | 50 mm/s, 200 mm/s², jerk 4000 |
| `axisGroup1` | `axis2`, `axis3` | `ScaraKinematics` | same |

Group coordinates are Cartesian TCP mm; a single-axis `move axisN` is joint
space in that axis's own units, so `move axis3 0.5` means 0.5 rad.

**Adding a configuration:** append an `AxisConfig` to `AXIS_CONFIGS` and raise
`NUM_AXES`. A new model implements `KinematicModel` in
`motion-core/src/kinematics.rs`.

## Command language

`help` prints this list at runtime. Targets are an axis (`axis0`) or a group
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

- Limits are **named** (`setlimits axis0 accel 100 jerk 500`), and `jerk`
  accepts `none`. `setlimits <target>` alone reads the limits back.
- `move` can override speed/accel/decel but not `max_jerk`.
- `verbose` toggles only the per-cycle heartbeat. Discrete events (moves,
  faults, DS402 transitions) always print.

## Visualization

egui/eframe + `egui_plot`, baked into `app` as `viz.rs` + `recording.rs`. It is
passive and read-only — input stays terminal-only. It taps the
`AxisGroup::exchange()` seam via `RecordingAxisGroup`. Plane plots are drawn for
2-axis groups.

## Implemented and not implemented

Implemented:

- trapezoidal and jerk-limited profiles
- redirect with a non-zero start velocity, and stop ramps
- straight-line and spline path moves with buffering and blending
- the kinematics seam, with a working SCARA
- the sim backend, with a DS402 state machine and faults
- per-axis and per-group runtime limits
- group interrupt cascades
- visualization: per-axis plots, and plane plots for 2-axis groups

Not implemented:

- **Centripetal acceleration is unbounded.** One scalar `max_speed` applies
  over arc length; `v²κ` is unchecked and the demo path exceeds
  `max_acceleration` by roughly 3×.
- **No joint-rate limiting and no mid-path reachability checking.** Validation
  is install-time endpoints only, plus a reactive cascade stop when inverse
  kinematics fails mid-move. Singularities are also not checked mid-path.
- **Paths are G1, not G2.** Catmull-Rom splines are C1, so curvature steps at
  every waypoint.
- **No joint position limits for the SCARA** (`position_limits` is `None`), and
  limits are checked at a move's commanded endpoint only, not along its path.
- **No PLCopen blending modes** (`BlendingLow`/`Previous`/`Next`/`High`).
  `BufferMode::Blend` only changes how a new path's geometry starts.
- **No redundant or reduced-DOF kinematic models**; `dof()` must equal the
  group's axis count.
- **No control-loop integration tests.** Profiles, kinematics, parsing and the
  sim backend are unit-tested; the loop itself is exercised by running it.
- `backend-sim` integrates commanded velocity and models no mechanics.

## Utilities

`scripts/utilities/` holds a tool for drawing SVG artwork on `axisGroup0`:

- `svg_to_waypoints.rs` converts an SVG's `<path>` outlines into one
  `movepath` waypoint file per contour, plus a `session.txt`.
- `draw_svg.sh <contour-dir> [vmax]` feeds that session to `app`. It defaults to
  12 mm/s.

## License

MIT. See [`LICENSE`](LICENSE).
