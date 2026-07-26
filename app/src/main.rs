//! A continuously-running, multi-axis motion app you can drive from the
//! terminal. This is the first cut of the "online" architecture: a
//! fixed-rate control loop running on its own schedule, decoupled from
//! (blocking) terminal input by a channel.
//!
//! Each cycle, the loop samples `motion-core`'s trajectory to get this
//! cycle's commanded (position, velocity), sends it through a
//! `backend_sim::SimAxisGroup` via the `AxisGroup` seam, and treats the
//! returned feedback — not the raw sample — as the axis's actual position.
//! `SimAxisGroup` integrates velocity into its own position state rather
//! than snapping to the commanded position, so there's a small, real
//! (if currently tiny) gap between "commanded" and "actual" — the same seam
//! `backend-ethercat` will occupy later, unchanged from the loop's point of
//! view.
//!
//! Axes are independent: `move axis0 100` and `move axis1 50` run
//! concurrently on their own clocks, with no relationship between their
//! durations. Driving several axes to arrive together (finish-together
//! time-scaling) is a distinct, harder feature — deliberately deferred.
//!
//! Run from the workspace root with:
//!
//!     cargo run -p app                 # terminal + viz window
//!     cargo run -p app -- --headless   # terminal only, no window
//!
//! `--headless` skips the viz window entirely and runs the control loop on
//! the main thread. It exists for scripted, non-interactive sessions —
//! piping commands into stdin and reading the printed output back — where a
//! GUI window is pure overhead and a liability: it needs a display server,
//! it's the slowest part of startup, and it keeps the process alive after
//! the control loop has ended. Everything except the plots behaves
//! identically; see `scripts/demo_session.sh`.
//!
//! Commands (one per line on stdin):
//!     move <axisN> <target_mm> [max_speed] [max_acceleration] [max_deceleration] [aborting|buffered]
//!     stop <axisN> [max_deceleration]
//!     enable <axisN>
//!     disable <axisN>
//!     reset <axisN>
//!     verbose
//!     status
//!     help
//!     quit
//!
//! Axes start disabled (DS402's `SwitchOnDisabled`), matching real drive
//! power-up — `move` on a disabled axis is rejected, not queued. `enable`
//! steps an axis through the real DS402 sequence (`SwitchOnDisabled` ->
//! `ReadyToSwitchOn` -> `SwitchedOn` -> `OperationEnabled`), one transition
//! per control cycle, same as a real master/drive negotiate it. See
//! `axis-backend`'s `Ds402State` docs.
//!
//! Disabling an axis that's actually moving is a fault, not a graceful
//! power-down — a real drive can't safely just cut its power stage
//! mid-motion the way it safely can from rest. A faulted axis needs `reset`
//! before it'll accept `enable` again; `reset` alone doesn't re-enable it.
//!
//! `stop` interrupts whatever an axis is doing (active or queued) with a
//! controlled deceleration to rest, built from the axis's *actual* current
//! velocity (see `motion_core::StopRamp`). This is not DS402's Quick Stop
//! (a distinct, typically emergency/safety-triggered mechanism) — the
//! backend's `Ds402State` is unaffected; only the coarser `AxisState` (via
//! `AxisSetpoint::stopping`) reports `Stopping` instead of `DiscreteMotion`.
//!
//! A `move`'s trailing `aborting`/`buffered` keyword picks its buffer mode:
//! `buffered` (the default) queues behind a busy axis, FIFO, and runs once
//! earlier moves finish, same as always; `aborting` takes over immediately
//! — active or queued — building the new move from the axis's actual
//! position/velocity via
//! `motion_core::TrapezoidalProfile::new_with_start_velocity` rather than
//! waiting for it to come to rest first.
//!
//! `verbose` toggles the periodic per-cycle position/phase line printed
//! while an axis is moving — off by default. Discrete events (move
//! started/finished/aborted, enable/disable, faults, DS402 transitions)
//! always print regardless; only the repetitive heartbeat is affected. Now
//! that the viz window plots target-vs-actual continuously, that heartbeat
//! is usually redundant on the terminal.

mod recording;
mod viz;

use std::collections::VecDeque;
use std::io::{self, BufRead};
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use axis_backend::{AxisGroup, AxisSetpoint, AxisState, Ds402State};
use backend_sim::SimAxisGroup;
use motion_core::{
    LinearMove, LinearMoveError, MotionPhase, PathProfile, PathProfileError, StopRamp,
    TrajectoryError, TrapezoidalProfile,
};
use recording::{History, RecordingAxisGroup};
use viz::VizApp;

const CONTROL_RATE_HZ: f64 = 250.0;
/// Defaults for *Cartesian* (group / path) moves, which are TCP-space
/// quantities regardless of what kind of axes carry them out. Single-axis
/// commands don't use these — they use the axis's own `AxisConfig`, whose
/// units may not be mm at all. See `AXIS_CONFIGS`.
const DEFAULT_CARTESIAN_MAX_SPEED: f64 = 50.0; // mm/s
const DEFAULT_CARTESIAN_MAX_ACCELERATION: f64 = 200.0; // mm/s^2
const STATUS_PRINT_PERIOD: Duration = Duration::from_millis(250);

/// Below this speed, an axis counts as already at rest for `stop`'s
/// "nothing to do" short-circuit.
const AT_REST_EPS: f64 = 1e-6;

/// One axis's own dynamic capability and travel, in **that axis's own
/// units** — mm for a linear axis, degrees for a rotary joint. Every
/// single-axis operation reads these: a raw `move axisN`/`stop axisN`'s
/// default limits, and — the one that isn't user-visible — the per-joint
/// `StopRamp` that `cascade_group_stop` builds for each member of an
/// interrupted group.
///
/// That cascade case is why this exists rather than two global constants.
/// A group move's limits are *Cartesian* (mm/s² of TCP), but the cascade
/// applies a deceleration in **joint** space. While every axis is linear
/// and every group is Cartesian those are numerically the same thing, so
/// the confusion is invisible; the moment a group has non-identity
/// kinematics, feeding a mm/s² number to a rotary joint's ramp is a silent
/// unit error on the exact path that runs when something has already gone
/// wrong.
///
/// Deliberately one flat struct, not an enum over axis kinds. What varies
/// between a linear axis and a rotary joint is the *numbers*, a display
/// *label*, and whether travel is bounded — not the operations performed on
/// them, and `motion-core` is unit-agnostic `f64` throughout. An
/// `AxisKind` enum earns its place when some axis type needs different
/// *math* (a continuous rotary axis wanting shortest-path wraparound inside
/// `TrapezoidalProfile`, say), not merely different values.
///
/// Compile-time like `AXIS_GROUPS`, for the same reason: this is machine
/// configuration, and there's no runtime-config infrastructure to hang it
/// on yet. The shape deserializes unchanged if that ever arrives.
struct AxisConfig {
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
    /// Display only — `motion-core` never sees units. Used by `status` and
    /// the heartbeat so a rotary joint doesn't print "mm".
    units: &'static str,
    /// Soft travel limits, as `(min, max)` inclusive. `None` means
    /// unbounded — a continuous rotary axis, or an axis whose limits simply
    /// aren't modelled yet. Checked when a move target is commanded; see
    /// `check_target_in_limits`.
    position_limits: Option<(f64, f64)>,
}

/// Per-axis configuration, indexed by axis number — `main()` asserts the
/// length matches `NUM_AXES`.
const AXIS_CONFIGS: &[AxisConfig] = &[
    AxisConfig {
        max_speed: 50.0,
        max_acceleration: 200.0,
        max_deceleration: 200.0,
        units: "mm",
        position_limits: Some((-500.0, 500.0)),
    },
    AxisConfig {
        max_speed: 50.0,
        max_acceleration: 200.0,
        max_deceleration: 200.0,
        units: "mm",
        position_limits: Some((-500.0, 500.0)),
    },
    // axis2/axis3: the SCARA arm's shoulder and elbow (see
    // `SCARA_KINEMATICS`). Rotary, and in **radians** — `ScaraKinematics`
    // does trigonometry on these values directly, and `motion-core` has no
    // unit conversions in it. So `move axis3 0.5` means 0.5 rad ≈ 28.6°.
    //
    // `position_limits: None` — joint travel limits are deferred (T4); the
    // arm's real constraint today is workspace reachability, which the
    // group move's install-time IK check covers, and which says nothing
    // about a raw single-axis jog anyway.
    AxisConfig {
        max_speed: 2.0,
        max_acceleration: 8.0,
        max_deceleration: 8.0,
        units: "rad",
        position_limits: None,
    },
    AxisConfig {
        max_speed: 2.0,
        max_acceleration: 8.0,
        max_deceleration: 8.0,
        units: "rad",
        position_limits: None,
    },
];

/// Rejects a single-axis move target outside the axis's soft travel limits.
///
/// Deliberately narrow: this checks a *commanded endpoint* only. It says
/// nothing about the interior of a path, nor about where a group move's
/// Cartesian target lands once mapped through kinematics into joint space —
/// both of which need the whole-path plausibility checking that is still
/// deferred. An unlimited axis (`position_limits: None`) always passes.
fn check_target_in_limits(axis: usize, target: f64) -> Result<(), String> {
    match AXIS_CONFIGS[axis].position_limits {
        Some((min, max)) if target < min || target > max => Err(format!(
            "target {target:.3} {units} is outside travel limits [{min:.3}, {max:.3}] {units}",
            units = AXIS_CONFIGS[axis].units
        )),
        _ => Ok(()),
    }
}

/// How much target-vs-actual history the viz window keeps per axis, in
/// seconds. Older samples are dropped as new ones arrive.
const HISTORY_SECONDS: f64 = 60.0;

/// Number of independent axes the app manages, named `axis0`..`axis{N-1}`
/// on the command line. Bumping this is the only change needed to add more
/// axes — everything else is `Vec`-driven.
const NUM_AXES: usize = 4;

/// A hard-coded axis group: a named Cartesian pair/triple/etc. that can be
/// driven by the same `enable`/`disable`/`reset`/`stop`/`move` commands as a
/// single axis, just fanned out to (or coordinated across) its members.
/// Groups are **not** created or removed at runtime — a fixed,
/// known-at-compile-time table by design.
struct AxisGroupDef {
    name: &'static str,
    axes: &'static [usize],
    /// How this group's *task-space* (Cartesian) coordinates map to its
    /// members' joint positions. Every group has one, including groups of
    /// plain linear stages whose axes already are Cartesian coordinates —
    /// those use `IdentityKinematics`, so there is exactly one code path
    /// through the move builders, the control loop and status/viz rather
    /// than a kinematic/non-kinematic fork.
    ///
    /// `&'static dyn` rather than a generic parameter: the table is
    /// heterogeneous (different models per group) and the dispatch happens
    /// once per group per cycle, which is nothing next to the trig it
    /// guards.
    kinematics: &'static dyn motion_core::KinematicModel,
}

/// The pass-through model for 2-axis Cartesian groups. Carries its own DOF
/// so `main()`'s arity assertion means something for identity groups too.
static IDENTITY_KINEMATICS_2: motion_core::IdentityKinematics =
    motion_core::IdentityKinematics::new(2);

/// The 2-link planar arm driven by `axisGroup1`. Equal links, deliberately:
/// unequal links would put the fold-back singularity on the boundary of an
/// unreachable inner hole, where install-time rejection catches it for free,
/// but equal links collapse that hole to a single point **at the origin** —
/// so the singularity is reachable, and a straight move from `(x, y)` to
/// `(-x, -y)` crosses it with both endpoints validating cleanly. That is
/// handled reactively (the per-cycle `NearSingular` → cascade stop), not
/// prevented; until whole-path plausibility checking exists, "don't cross
/// the origin" is a convention that tests and demo targets observe, not a
/// property of the geometry.
static SCARA_KINEMATICS: motion_core::ScaraKinematics =
    motion_core::ScaraKinematics::new(100.0, 100.0);

/// `axisGroup0` = `axis0` (X) + `axis1` (Y), a Cartesian pair.
/// `axisGroup1` = `axis2` (shoulder) + `axis3` (elbow), a SCARA arm — same
/// commands, same Cartesian move semantics, different model. Add more
/// entries here to define more groups; `main()` asserts every referenced
/// axis index is `< NUM_AXES` and that each group's arity matches its
/// kinematic model's DOF.
const AXIS_GROUPS: &[AxisGroupDef] = &[
    AxisGroupDef {
        name: "axisGroup0",
        axes: &[0, 1],
        kinematics: &IDENTITY_KINEMATICS_2,
    },
    AxisGroupDef {
        name: "axisGroup1",
        axes: &[2, 3],
        kinematics: &SCARA_KINEMATICS,
    },
];

/// Dispatch policy for a move when the target is already busy. `Aborting`/
/// `Buffered` apply to every move target (`move`/`movegroup`/`movepath`);
/// `Blend` is `movepath`-only (see `Command::MovePath`'s docs) — a narrower,
/// `movepath`-specific thing: smoothly transitioning the path's own
/// *geometry* onto a new one, not blending across two independent move
/// segments in general.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BufferMode {
    /// Takes effect immediately regardless of idle/busy: clears anything
    /// queued and replaces whatever's active right now, built from the
    /// axis's *actual* current position/velocity (backend feedback, not a
    /// commanded setpoint) via `TrapezoidalProfile::new_with_start_velocity`.
    Aborting,
    /// Queues behind the current move if the axis is busy (FIFO — every
    /// buffered move queued eventually runs, in order); starts immediately
    /// if idle. The default, so omitting a mode keeps today's behavior.
    Buffered,
    /// `movepath` only: like `Aborting` (takes effect immediately), but
    /// built via `motion_core::PathProfile::new_blended` instead of
    /// `new_with_start_velocity` — the new path's own start tangent leans
    /// toward the group's actual incoming velocity direction (via
    /// `WaypointPath::new_with_start_direction`), instead of assuming a
    /// straight approach toward its own first waypoint, so the transition
    /// is smoother than `Aborting`'s hard redirect. Not exact velocity
    /// continuity — see `PathProfile::new_blended`'s docs.
    Blend,
}

#[derive(Debug, PartialEq)]
enum Command {
    Move {
        axis: usize,
        target: f64,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        buffer_mode: BufferMode,
    },
    Enable {
        axis: usize,
    },
    Disable {
        axis: usize,
    },
    Reset {
        axis: usize,
    },
    Stop {
        axis: usize,
        max_deceleration: Option<f64>,
    },
    MoveGroup {
        group: usize,
        targets: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        buffer_mode: BufferMode,
    },
    EnableGroup {
        group: usize,
    },
    DisableGroup {
        group: usize,
    },
    ResetGroup {
        group: usize,
    },
    StopGroup {
        group: usize,
        max_deceleration: Option<f64>,
    },
    /// A multi-waypoint move for a group, smoothly blended through every
    /// waypoint via `motion_core::PathProfile` (centripetal Catmull-Rom).
    /// `waypoints` are the points *after* the group's current position —
    /// the actual current position (and velocity, via whichever of
    /// `PathProfile::new_with_start_velocity`/`new_blended` `buffer_mode`
    /// picks) is always prepended as the path's own start (see
    /// `install_path_move_impl`), the same "start is the axes' actual
    /// state" rule every other move here follows. `buffer_mode`:
    /// `Buffered` queues FIFO behind a busy group, same as `MoveGroup`;
    /// `Aborting` and `Blend` both redirect immediately and only differ in
    /// *how* the new path's geometry is built — `Aborting` via
    /// `install_path_move` (assumes a straight approach to the new path's
    /// first waypoint), `Blend` via `install_path_move_blended` (leans the
    /// new path's own start tangent toward the actual incoming velocity
    /// direction instead, for a visibly smoother transition).
    MovePath {
        group: usize,
        waypoints: Vec<Vec<f64>>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        buffer_mode: BufferMode,
    },
    Verbose,
    Status,
    Help,
    Quit,
}

fn main() {
    debug_assert_eq!(
        AXIS_CONFIGS.len(),
        NUM_AXES,
        "AXIS_CONFIGS has {} entries but NUM_AXES is {NUM_AXES}",
        AXIS_CONFIGS.len()
    );
    for group in AXIS_GROUPS {
        for &axis in group.axes {
            debug_assert!(
                axis < NUM_AXES,
                "AXIS_GROUPS[{}] references axis{axis}, but NUM_AXES is {NUM_AXES}",
                group.name
            );
        }
        debug_assert_eq!(
            group.kinematics.dof(),
            group.axes.len(),
            "AXIS_GROUPS[{}] has {} axes but its kinematic model has {} DOF",
            group.name,
            group.axes.len(),
            group.kinematics.dof()
        );
    }

    let headless = match parse_args() {
        Ok(headless) => headless,
        Err(message) => {
            eprintln!("{message}");
            eprintln!("usage: app [--headless]");
            std::process::exit(2);
        }
    };

    print_help();

    let (tx, rx) = mpsc::channel::<Command>();
    thread::spawn(move || read_commands(tx));

    let capacity = (CONTROL_RATE_HZ * HISTORY_SECONDS) as usize;
    let history = Arc::new(Mutex::new(History::new(NUM_AXES, capacity)));

    // Headless: no window, so the control loop runs right here on the main
    // thread and the process ends when it does. `RecordingAxisGroup` still
    // wraps the backend and still records — the recording tap is at the
    // `AxisGroup` seam and has nothing to do with whether anything is
    // drawing it, and keeping it means headless and windowed runs execute
    // the identical loop, which is the point of the mode.
    if headless {
        run_control_loop(rx, history);
        return;
    }

    // Set once the control loop exits (via `quit` or stdin EOF) — the viz
    // window has no controls of its own, so this is its cue to close too.
    let shutdown = Arc::new(AtomicBool::new(false));

    {
        let history = Arc::clone(&history);
        let shutdown = Arc::clone(&shutdown);
        thread::spawn(move || {
            run_control_loop(rx, history);
            shutdown.store(true, Ordering::Relaxed);
        });
    }

    // eframe/winit need the GUI event loop on the main thread, so it runs
    // here while the control loop (moved above) and stdin reader each run
    // on their own thread. This call blocks until the viz window closes.
    let native_options = eframe::NativeOptions {
        renderer: eframe::Renderer::Glow,
        // eframe's default (unset) inner size is too short to fit the XY
        // plots + status boxes row *and* the per-axis position/velocity
        // plots row without clipping — the content also scrolls (see
        // viz.rs) so this is a starting size, not a hard requirement, but
        // one generous enough that nothing needs to scroll for the common
        // 2-axis / 1-group case.
        viewport: eframe::egui::ViewportBuilder::default().with_inner_size([1000.0, 900.0]),
        ..Default::default()
    };
    eframe::run_native(
        "motion-project viz",
        native_options,
        Box::new(move |_cc| Ok(Box::new(VizApp::new(history, shutdown, NUM_AXES)))),
    )
    .expect("failed to run viz window");
}

/// Reads the one command-line flag this app has: `--headless`. Returns
/// whether it was given, or an error message for anything else.
///
/// Hand-rolled rather than pulling in a CLI crate: it's one boolean, and
/// `app` is the only crate here with dependencies at all — adding an
/// argument parser for this would be the most-dependencies-per-feature
/// change in the workspace. Revisit if a third flag ever shows up.
///
/// Unknown arguments are an error, not ignored: a scripted session that
/// typos `--headles` should fail loudly rather than silently opening a
/// window nobody is watching and then hanging.
fn parse_args() -> Result<bool, String> {
    let mut headless = false;
    for arg in std::env::args().skip(1) {
        match arg.as_str() {
            "--headless" => headless = true,
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(headless)
}

/// Blocks on stdin, parsing one command per line and forwarding it to the
/// control loop. Runs on its own thread so a user typing (or a slow
/// terminal) never delays a control-loop tick. Parse errors are reported
/// here, directly to the terminal, since they never touch loop state.
fn read_commands(tx: Sender<Command>) {
    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let line = match line {
            Ok(line) => line,
            Err(_) => break,
        };
        match parse_command(&line) {
            Ok(Some(cmd)) => {
                if tx.send(cmd).is_err() {
                    break; // control loop has exited
                }
            }
            Ok(None) => {} // blank line
            Err(msg) => println!("  ! {msg}"),
        }
    }
    // stdin closed (EOF / Ctrl-D): shut the app down cleanly.
    let _ = tx.send(Command::Quit);
}

fn parse_command(line: &str) -> Result<Option<Command>, String> {
    let words: Vec<&str> = line.split_whitespace().collect();
    match words.as_slice() {
        [] => Ok(None),
        ["quit"] | ["exit"] => Ok(Some(Command::Quit)),
        ["verbose"] => Ok(Some(Command::Verbose)),
        ["status"] => Ok(Some(Command::Status)),
        ["help"] => Ok(Some(Command::Help)),
        ["enable", target] => match parse_target(target)? {
            Target::Axis(axis) => Ok(Some(Command::Enable { axis })),
            Target::Group(group) => Ok(Some(Command::EnableGroup { group })),
        },
        ["disable", target] => match parse_target(target)? {
            Target::Axis(axis) => Ok(Some(Command::Disable { axis })),
            Target::Group(group) => Ok(Some(Command::DisableGroup { group })),
        },
        ["reset", target] => match parse_target(target)? {
            Target::Axis(axis) => Ok(Some(Command::Reset { axis })),
            Target::Group(group) => Ok(Some(Command::ResetGroup { group })),
        },
        ["stop", target] => match parse_target(target)? {
            Target::Axis(axis) => Ok(Some(Command::Stop {
                axis,
                max_deceleration: None,
            })),
            Target::Group(group) => Ok(Some(Command::StopGroup {
                group,
                max_deceleration: None,
            })),
        },
        ["stop", target, decel] => {
            let max_deceleration = Some(parse_f64(decel)?);
            match parse_target(target)? {
                Target::Axis(axis) => Ok(Some(Command::Stop {
                    axis,
                    max_deceleration,
                })),
                Target::Group(group) => Ok(Some(Command::StopGroup {
                    group,
                    max_deceleration,
                })),
            }
        }
        ["move", target, rest @ ..] => parse_move(target, rest, line),
        ["movepath", target, rest @ ..] => parse_move_path(target, rest, line),
        _ => Err(format!(
            "unrecognized command: {line:?} (type \"help\" for usage)"
        )),
    }
}

/// Parses everything after `"move" <target>`: the target's coordinates (1
/// for a single axis, `AXIS_GROUPS[g].axes.len()` for a group — always
/// immediately after the target name), then the usual up-to-3 numeric
/// kinematic-limit args and an optional trailing `aborting`/`buffered`
/// keyword, unchanged from the single-axis-only syntax this replaces.
fn parse_move(target: &str, rest: &[&str], line: &str) -> Result<Option<Command>, String> {
    let target = parse_target(target)?;
    let coord_count = match target {
        Target::Axis(_) => 1,
        Target::Group(g) => AXIS_GROUPS[g].axes.len(),
    };
    if rest.len() < coord_count {
        return Err(format!(
            "expected {coord_count} target coordinate(s): {line:?}"
        ));
    }
    let (coords, rest) = rest.split_at(coord_count);
    let targets: Vec<f64> = coords
        .iter()
        .map(|c| parse_f64(c))
        .collect::<Result<_, _>>()?;

    // A single-axis move defaults to that axis's own limits (which may be
    // deg/s, not mm/s); a group move is Cartesian and defaults to the
    // TCP-space constants. See `AxisConfig`.
    let defaults = match target {
        Target::Axis(a) => (AXIS_CONFIGS[a].max_speed, AXIS_CONFIGS[a].max_acceleration),
        Target::Group(_) => (
            DEFAULT_CARTESIAN_MAX_SPEED,
            DEFAULT_CARTESIAN_MAX_ACCELERATION,
        ),
    };
    let (buffer_mode, max_speed, max_acceleration, max_deceleration) =
        parse_buffer_mode_and_limits(rest, line, defaults)?;

    match target {
        Target::Axis(axis) => Ok(Some(Command::Move {
            axis,
            target: targets[0],
            max_speed,
            max_acceleration,
            max_deceleration,
            buffer_mode,
        })),
        Target::Group(group) => Ok(Some(Command::MoveGroup {
            group,
            targets,
            max_speed,
            max_acceleration,
            max_deceleration,
            buffer_mode,
        })),
    }
}

/// Parses everything after `"movepath" <target>`: either the inline form
/// (a waypoint count, then that many waypoints' worth of coordinates — see
/// `parse_move_path_inline`) or, if the first token is the literal `"file"`,
/// waypoints read from a file, one per line (see `parse_move_path_file`) —
/// the inline form gets unwieldy past a handful of waypoints for an
/// interactive line-based command, so this is the same `Command::MovePath`
/// reached a second way. A single axis can't be a `movepath` target either
/// way — a one-axis "path" is just what `move ... buffered` chaining
/// already gives you.
fn parse_move_path(target: &str, rest: &[&str], line: &str) -> Result<Option<Command>, String> {
    let group = match parse_target(target)? {
        Target::Axis(_) => {
            return Err(format!(
                "movepath requires a group target, not a single axis: {line:?}"
            ));
        }
        Target::Group(g) => g,
    };
    let coord_count = AXIS_GROUPS[group].axes.len();

    match rest {
        ["file", path, rest @ ..] => parse_move_path_file(group, coord_count, path, rest, line),
        _ => parse_move_path_inline(group, coord_count, rest, line),
    }
}

/// The inline `movepath` form: a required waypoint count, then that many
/// waypoints' worth of coordinates (`coord_count` each), then the usual
/// up-to-3 numeric kinematic-limit args and an optional trailing
/// `aborting`/`buffered`/`blend` keyword — same shape `move`'s trailing
/// args have, plus the `movepath`-only `blend` option.
fn parse_move_path_inline(
    group: usize,
    coord_count: usize,
    rest: &[&str],
    line: &str,
) -> Result<Option<Command>, String> {
    let (&n_waypoints_str, rest) = rest
        .split_first()
        .ok_or_else(|| format!("expected a waypoint count: {line:?}"))?;
    let n_waypoints: usize = n_waypoints_str
        .parse()
        .map_err(|_| format!("expected a waypoint count, got {n_waypoints_str:?}"))?;
    if n_waypoints == 0 {
        return Err(format!("movepath needs at least 1 waypoint: {line:?}"));
    }

    let coord_total = n_waypoints * coord_count;
    if rest.len() < coord_total {
        return Err(format!(
            "expected {coord_total} waypoint coordinate(s) ({n_waypoints} waypoint(s) x {coord_count} axes): {line:?}"
        ));
    }
    let (coords, rest) = rest.split_at(coord_total);
    let flat: Vec<f64> = coords
        .iter()
        .map(|c| parse_f64(c))
        .collect::<Result<_, _>>()?;
    let waypoints: Vec<Vec<f64>> = flat.chunks(coord_count).map(|c| c.to_vec()).collect();

    let (buffer_mode, max_speed, max_acceleration, max_deceleration) =
        parse_movepath_buffer_mode_and_limits(rest, line)?;
    Ok(Some(Command::MovePath {
        group,
        waypoints,
        max_speed,
        max_acceleration,
        max_deceleration,
        buffer_mode,
    }))
}

/// The file `movepath` form: waypoints read from `path` (see
/// `read_waypoints_file`), followed by the usual up-to-3 numeric
/// kinematic-limit args and an optional trailing `aborting`/`buffered`/
/// `blend` keyword — everything else is identical to the inline form.
/// Reading happens here, on the stdin-reading thread (same as every other
/// parse error), not in the control loop — `Command::MovePath` ends up
/// identical either way, so nothing downstream needs to know which form
/// was used.
fn parse_move_path_file(
    group: usize,
    coord_count: usize,
    path: &str,
    rest: &[&str],
    line: &str,
) -> Result<Option<Command>, String> {
    let waypoints = read_waypoints_file(path, coord_count)?;
    let (buffer_mode, max_speed, max_acceleration, max_deceleration) =
        parse_movepath_buffer_mode_and_limits(rest, line)?;
    Ok(Some(Command::MovePath {
        group,
        waypoints,
        max_speed,
        max_acceleration,
        max_deceleration,
        buffer_mode,
    }))
}

/// Reads `path` and delegates to `parse_waypoint_lines` — split out as its
/// own thin wrapper so the actual line-parsing logic stays unit-testable
/// without touching the filesystem.
fn read_waypoints_file(path: &str, coord_count: usize) -> Result<Vec<Vec<f64>>, String> {
    let contents =
        std::fs::read_to_string(path).map_err(|e| format!("failed to read {path:?}: {e}"))?;
    parse_waypoint_lines(&contents, coord_count).map_err(|e| format!("{path}: {e}"))
}

/// One waypoint per line, `coord_count` whitespace-separated numbers each;
/// blank lines are skipped (so trailing newlines, and visual grouping in a
/// hand-written file, don't need special treatment). No comment syntax —
/// deliberately as simple as the file format needs to be for now.
fn parse_waypoint_lines(contents: &str, coord_count: usize) -> Result<Vec<Vec<f64>>, String> {
    let mut waypoints = Vec::new();
    for (i, raw_line) in contents.lines().enumerate() {
        let trimmed = raw_line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let tokens: Vec<&str> = trimmed.split_whitespace().collect();
        if tokens.len() != coord_count {
            return Err(format!(
                "line {}: expected {coord_count} coordinate(s), got {}: {trimmed:?}",
                i + 1,
                tokens.len()
            ));
        }
        let coords: Vec<f64> = tokens
            .iter()
            .map(|t| parse_f64(t))
            .collect::<Result<_, _>>()
            .map_err(|e| format!("line {}: {e}", i + 1))?;
        waypoints.push(coords);
    }
    if waypoints.is_empty() {
        return Err("no waypoints found (file is empty or all-blank)".to_string());
    }
    Ok(waypoints)
}

/// A trailing optional `aborting`/`buffered` keyword (always last,
/// regardless of how many numeric args precede it) followed by the usual
/// up-to-3 numeric kinematic-limit args — shared by `move`/`movegroup`.
/// `blend` is deliberately not recognized here — it's meaningless for a
/// straight-line move (see `parse_movepath_buffer_mode_and_limits`, which
/// both `movepath` forms use instead).
fn parse_buffer_mode_and_limits(
    rest: &[&str],
    line: &str,
    defaults: (f64, f64),
) -> Result<(BufferMode, f64, f64, f64), String> {
    let (buffer_mode, numeric_rest) = match rest.last() {
        Some(&"aborting") => (BufferMode::Aborting, &rest[..rest.len() - 1]),
        Some(&"buffered") => (BufferMode::Buffered, &rest[..rest.len() - 1]),
        _ => (BufferMode::Buffered, rest),
    };
    let (max_speed, max_acceleration, max_deceleration) =
        parse_kinematic_limits(numeric_rest, line, defaults)?;
    Ok((buffer_mode, max_speed, max_acceleration, max_deceleration))
}

/// `movepath`'s own version of `parse_buffer_mode_and_limits` — identical
/// except it also recognizes the trailing `blend` keyword
/// (`BufferMode::Blend`, movepath-only; see `Command::MovePath`'s docs).
fn parse_movepath_buffer_mode_and_limits(
    rest: &[&str],
    line: &str,
) -> Result<(BufferMode, f64, f64, f64), String> {
    let (buffer_mode, numeric_rest) = match rest.last() {
        Some(&"aborting") => (BufferMode::Aborting, &rest[..rest.len() - 1]),
        Some(&"buffered") => (BufferMode::Buffered, &rest[..rest.len() - 1]),
        Some(&"blend") => (BufferMode::Blend, &rest[..rest.len() - 1]),
        _ => (BufferMode::Buffered, rest),
    };
    let (max_speed, max_acceleration, max_deceleration) = parse_kinematic_limits(
        numeric_rest,
        line,
        (
            DEFAULT_CARTESIAN_MAX_SPEED,
            DEFAULT_CARTESIAN_MAX_ACCELERATION,
        ),
    )?;
    Ok((buffer_mode, max_speed, max_acceleration, max_deceleration))
}

/// The trailing up-to-3 numeric kinematic-limit args shared by both
/// `movepath` forms (`[vmax] [amax] [dmax]`, each defaulting the same way
/// `move`'s do: `vmax`/`amax` fall back to the global defaults, `dmax`
/// falls back to whatever `amax` resolved to).
fn parse_kinematic_limits(
    rest: &[&str],
    line: &str,
    defaults: (f64, f64),
) -> Result<(f64, f64, f64), String> {
    if rest.len() > 3 {
        return Err(format!("too many arguments: {line:?}"));
    }
    let mut numeric_rest = rest.iter();
    let max_speed = match numeric_rest.next() {
        Some(v) => parse_f64(v)?,
        None => defaults.0,
    };
    let max_acceleration = match numeric_rest.next() {
        Some(a) => parse_f64(a)?,
        None => defaults.1,
    };
    let max_deceleration = match numeric_rest.next() {
        Some(d) => parse_f64(d)?,
        None => max_acceleration,
    };
    Ok((max_speed, max_acceleration, max_deceleration))
}

/// What a command verb (`enable`/`disable`/`reset`/`stop`/`move`) is aimed
/// at: a single axis, or a hard-coded group of them.
enum Target {
    Axis(usize),
    /// Index into `AXIS_GROUPS`.
    Group(usize),
}

/// Resolves a target token as a group name first (an exact match against
/// `AXIS_GROUPS`), falling back to `parse_axis` — so `axisGroup0` and
/// `axis0` are both valid wherever a `Target` is expected, with no
/// ambiguity (group names and axis names can't collide by construction).
fn parse_target(s: &str) -> Result<Target, String> {
    if let Some(g) = AXIS_GROUPS.iter().position(|g| g.name == s) {
        return Ok(Target::Group(g));
    }
    parse_axis(s).map(Target::Axis)
}

/// Parses an axis token like `"axis0"` into its index, validating it's a
/// known axis. Axis names are `axis0`..`axis{NUM_AXES-1}`.
fn parse_axis(s: &str) -> Result<usize, String> {
    let index = s
        .strip_prefix("axis")
        .ok_or_else(|| format!("expected an axis name like \"axis0\", got {s:?}"))?
        .parse::<usize>()
        .map_err(|_| format!("expected an axis name like \"axis0\", got {s:?}"))?;
    if index >= NUM_AXES {
        return Err(format!(
            "no such axis {s:?} (valid: axis0..axis{})",
            NUM_AXES - 1
        ));
    }
    Ok(index)
}

fn parse_f64(s: &str) -> Result<f64, String> {
    s.parse::<f64>()
        .map_err(|_| format!("expected a number, got {s:?}"))
}

fn axis_label(axis: usize) -> String {
    format!("axis{axis}")
}

fn group_label(group: usize) -> &'static str {
    AXIS_GROUPS[group].name
}

/// Whether an axis is enabled and fault-free — the gate every business-logic
/// decision in this loop checks (`move`/`stop` rejection, queue promotion,
/// `enable`'s "already enabled"), instead of comparing `Ds402State` directly
/// (see `AxisRuntime::ds402_state`'s docs for why). `AxisState::Disabled`
/// covers every DS402 substate short of fully `OperationEnabled`, so this is
/// exactly equivalent to comparing `Ds402State::OperationEnabled` directly
/// — just expressed at the layer `app` is supposed to reason in.
fn axis_operational(state: AxisState) -> bool {
    !matches!(state, AxisState::Disabled | AxisState::ErrorStop)
}

/// Deliberately terse — one line per command with a short description, not
/// the detailed per-command usage notes this used to accumulate. Detailed
/// per-command help (e.g. `help move`) is a natural future addition, not
/// built yet: this just needs to stay a quick-glance list.
fn print_help() {
    println!(
        "motion-project online app — {NUM_AXES} independent axes, control rate {CONTROL_RATE_HZ} Hz"
    );
    let group_names: Vec<&str> = AXIS_GROUPS.iter().map(|g| g.name).collect();
    println!(
        "targets: axis0..axis{}{}",
        NUM_AXES - 1,
        if group_names.is_empty() {
            String::new()
        } else {
            format!(", {}", group_names.join(", "))
        }
    );
    println!("commands:");
    fn cmd_line(usage: &str, description: &str) {
        println!("  {usage:<67} — {description}");
    }
    cmd_line(
        "move <target> <coord>... [vmax] [amax] [dmax] [aborting|buffered]",
        "start or redirect a move",
    );
    cmd_line(
        "movepath <groupName> <n> <coord>... [vmax] [amax] [dmax] [aborting|buffered|blend]",
        "smooth multi-waypoint group move, waypoints inline",
    );
    cmd_line(
        "movepath <groupName> file <path> [vmax] [amax] [dmax] [aborting|buffered|blend]",
        "same, waypoints read from a file (one per line)",
    );
    cmd_line("stop <target> [decel]", "decelerate to a stop");
    cmd_line("enable <target>", "power up and enable");
    cmd_line("disable <target>", "disable (faults it if actively moving)");
    cmd_line("reset <target>", "clear a fault");
    cmd_line("verbose", "toggle the per-cycle position/phase heartbeat");
    cmd_line("status", "print position/phase for every axis and group");
    cmd_line("help", "show this message");
    cmd_line("quit", "exit");
    println!();
    // Worth saying explicitly now that a group's coordinates and its
    // members' units genuinely differ, rather than coinciding as they do
    // for a Cartesian group.
    println!("  a group move's coordinates are Cartesian (mm); a single-axis");
    println!("  move is raw joint space in that axis's own units:");
    for (axis, config) in AXIS_CONFIGS.iter().enumerate() {
        println!("    {}: {}", axis_label(axis), config.units);
    }
    println!();
}

/// State shared by every member of an active group move: the underlying
/// straight-line profile (see `motion_core::LinearMove`), which axes
/// participate (in the same order the profile's per-axis samples come out
/// in — always `AXIS_GROUPS[group].axes`, so this is a zero-alloc pointer,
/// not a clone).
///
/// Deliberately does *not* carry the group move's `max_deceleration`: it
/// used to, for `cascade_group_stop` to reuse, but that value is Cartesian
/// and the cascade's ramps are built in joint space, so each sibling now
/// decelerates at its own `AxisConfig::max_deceleration` instead.
///
/// The profile itself is entirely **Cartesian** — kinematics converts into
/// it once at install (start state) and out of it once per cycle (setpoints).
struct SharedGroupMove {
    profile: LinearMove,
    axes: &'static [usize],
    group: usize,
    /// Which IK solution branch this move runs on, resolved once from the
    /// group's *commanded* joint pose at install and then held for the
    /// move's whole duration.
    ///
    /// Per-move, not per-instance and not per-cycle. Per-instance would
    /// fight raw single-axis jogging: jog a joint into the other branch and
    /// a fixed-branch model jumps back to its own on the next move's first
    /// cycle. Per-*cycle* reselection ("nearest solution to last cycle")
    /// would make IK's output depend on its own previous output, breaking
    /// the "trajectory is a pure function of elapsed time" rule. Held here,
    /// IK stays a pure function of Cartesian position for the whole move,
    /// and the branch persists across moves for free: commanded joints came
    /// out of IK on this branch, so resolving from them next time returns
    /// it again.
    branch: motion_core::KinematicBranch,
    /// The move's endpoint in **joint** space — IK of the Cartesian target
    /// on `branch`, computed at install as the reachability check. Kept so
    /// `status` can report each member's own target instead of a Cartesian
    /// coordinate that means nothing for a rotary joint.
    joint_target: motion_core::KinematicVector,
}

/// The path-move analog of `SharedGroupMove` — same shape (a shared
/// profile, participating axes, group index), just wrapping
/// `motion_core::PathProfile` instead of
/// `LinearMove`. Kept as a distinct type (not a generalized
/// `SharedGroupMove<P>`) since the two profile types don't share a common
/// trait and nothing outside `Profile`'s own match arms needs to treat them
/// uniformly — see `ActiveGroupMove` for the one place that does.
struct SharedPathMove {
    profile: PathProfile,
    axes: &'static [usize],
    group: usize,
    /// Same role as `SharedGroupMove::branch` — see there.
    branch: motion_core::KinematicBranch,
    /// Same role as `SharedGroupMove::joint_target` — IK of the *final*
    /// waypoint. The intermediate waypoints are IK'd at install too, as a
    /// reachability check, but only the endpoint is worth keeping.
    joint_target: motion_core::KinematicVector,
}

/// Whichever motion profile is currently driving an axis: a commanded
/// move, a commanded stop, or (this axis's slice of) an active group move
/// or path move. All are pure functions of elapsed time with the same
/// shape (`sample`/`phase_at`/`target`), but aren't the same *type* —
/// `TrapezoidalProfile` always starts and ends at rest, `StopRamp` starts
/// wherever the axis actually is right now (see its own docs for why
/// that's a separate type, not a generalization of `TrapezoidalProfile`),
/// and `Group`/`Path` don't own a profile at all — they index into one
/// shared across every participating axis. This enum is what lets the rest
/// of the control loop treat "whatever's currently active" uniformly
/// without caring which one it is — except where it does care (`is_stop`),
/// for messages that should read differently for a stop than for a move.
enum Profile {
    Move(TrapezoidalProfile),
    Stop(StopRamp),
    Group {
        shared: Rc<SharedGroupMove>,
        index: usize,
    },
    Path {
        shared: Rc<SharedPathMove>,
        index: usize,
    },
}

impl Profile {
    fn sample(&self, t: f64) -> motion_core::TrajectorySample {
        match self {
            Profile::Move(p) => p.sample(t),
            Profile::Stop(p) => p.sample(t),
            Profile::Group { shared, index } => {
                let s = shared.profile.sample(t);
                motion_core::TrajectorySample {
                    position: s.position()[*index],
                    velocity: s.velocity()[*index],
                }
            }
            Profile::Path { shared, index } => {
                let s = shared.profile.sample(t);
                motion_core::TrajectorySample {
                    position: s.position()[*index],
                    velocity: s.velocity()[*index],
                }
            }
        }
    }

    fn phase_at(&self, t: f64) -> MotionPhase {
        match self {
            Profile::Move(p) => p.phase_at(t),
            Profile::Stop(p) => p.phase_at(t),
            Profile::Group { shared, .. } => shared.profile.phase_at(t),
            Profile::Path { shared, .. } => shared.profile.phase_at(t),
        }
    }

    /// This axis's own endpoint, always in **joint** space — which for a
    /// group/path member means the stored IK of the Cartesian target, not
    /// the Cartesian target's matching component. Those coincide only under
    /// identity kinematics; on an arm the Cartesian target is millimetres
    /// and this axis is a rotary joint.
    fn target(&self) -> f64 {
        match self {
            Profile::Move(p) => p.target(),
            Profile::Stop(p) => p.target(),
            Profile::Group { shared, index } => shared.joint_target.as_slice()[*index],
            Profile::Path { shared, index } => shared.joint_target.as_slice()[*index],
        }
    }

    /// `(group index, this axis's position within the group)` for a group or
    /// path move — the control loop's key into its per-group cache of
    /// IK-converted joint setpoints. `None` for a single-axis profile, which
    /// is already joint-space and needs no conversion.
    fn group_and_index(&self) -> Option<(usize, usize)> {
        match self {
            Profile::Group { shared, index } => Some((shared.group, *index)),
            Profile::Path { shared, index } => Some((shared.group, *index)),
            _ => None,
        }
    }

    fn is_stop(&self) -> bool {
        matches!(self, Profile::Stop(_))
    }

    /// The group name this profile belongs to, if it's a `Group` or `Path`
    /// — purely for `status`'s benefit, so a group/path move reads as one
    /// rather than looking like an ordinary single-axis move to a value
    /// that happens to match another axis's target.
    fn group_membership(&self) -> Option<&'static str> {
        match self {
            Profile::Group { shared, .. } => Some(group_label(shared.group)),
            Profile::Path { shared, .. } => Some(group_label(shared.group)),
            _ => None,
        }
    }
}

/// Clears any queued moves and replaces whatever's active on `ax` with a
/// fresh profile built from its current *commanded* position/velocity —
/// shared by `stop` (builds a `StopRamp`) and an `aborting` move (builds a
/// `TrapezoidalProfile` via `new_with_start_velocity`). `build` receives
/// (position, velocity), does its own success printing (it alone knows the
/// right message and has the built profile's `duration()` to hand), and
/// returns the `Profile` to install or a `TrajectoryError` to report the
/// same way regardless of which case triggered it.
///
/// Commanded, not actual: the replacement profile has to pick up the
/// commanded stream exactly where the superseded one left it, or the
/// redirect itself becomes a step input. See
/// `AxisRuntime::commanded_position`.
fn abort_into(
    ax: &mut AxisRuntime,
    axis: usize,
    action: &str,
    build: impl FnOnce(f64, f64) -> Result<Profile, TrajectoryError>,
) {
    ax.pending.clear();
    match build(ax.commanded_position, ax.commanded_velocity) {
        Ok(profile) => {
            ax.active = Some(ActiveMove {
                profile,
                started_at: Instant::now(),
            });
            ax.last_phase = None;
        }
        Err(e) => println!("  ! {}: {action} rejected: {e}", axis_label(axis)),
    }
}

/// A `Profile::Group` or `Profile::Path`'s shared state, abstracted just
/// enough for `cascade_group_stop` to treat both uniformly — the two
/// underlying profile types (`LinearMove`, `PathProfile`) don't share a
/// trait, so this is a thin enum over the two `Rc`s rather than a generic
/// `SharedGroupMove<P>`, kept private to the cascade mechanism.
enum ActiveGroupMove {
    Group(Rc<SharedGroupMove>),
    Path(Rc<SharedPathMove>),
}

impl ActiveGroupMove {
    fn axes(&self) -> &'static [usize] {
        match self {
            ActiveGroupMove::Group(s) => s.axes,
            ActiveGroupMove::Path(s) => s.axes,
        }
    }

    fn group(&self) -> usize {
        match self {
            ActiveGroupMove::Group(s) => s.group,
            ActiveGroupMove::Path(s) => s.group,
        }
    }

    /// This move's Cartesian `(position, velocity)` at `t`, plus the IK
    /// branch it was installed on — everything the control loop needs to
    /// convert one shared task-space sample into joint setpoints, from
    /// either profile type.
    ///
    /// Infallible in the vector construction: both profiles sample within
    /// `MAX_GROUP_AXES` and produce finite values from finite inputs, which
    /// the install-time construction already guaranteed.
    fn sample(
        &self,
        t: f64,
    ) -> (
        motion_core::KinematicVector,
        motion_core::KinematicVector,
        motion_core::KinematicBranch,
    ) {
        let (position, velocity, branch) = match self {
            ActiveGroupMove::Group(s) => {
                let sample = s.profile.sample(t);
                (
                    motion_core::KinematicVector::from_slice(sample.position()),
                    motion_core::KinematicVector::from_slice(sample.velocity()),
                    s.branch,
                )
            }
            ActiveGroupMove::Path(s) => {
                let sample = s.profile.sample(t);
                (
                    motion_core::KinematicVector::from_slice(sample.position()),
                    motion_core::KinematicVector::from_slice(sample.velocity()),
                    s.branch,
                )
            }
        };
        (
            position.expect("profile sample is always a valid kinematic vector"),
            velocity.expect("profile sample is always a valid kinematic vector"),
            branch,
        )
    }

    /// Whether `profile` is still actively driven by *this specific* shared
    /// move instance (identity, via `Rc::ptr_eq` within the matching
    /// variant — a `Group` can never alias a `Path`, so a variant mismatch
    /// is simply `false`, same as no active move at all).
    fn still_drives(&self, profile: &Profile) -> bool {
        match (self, profile) {
            (ActiveGroupMove::Group(target), Profile::Group { shared, .. }) => {
                Rc::ptr_eq(target, shared)
            }
            (ActiveGroupMove::Path(target), Profile::Path { shared, .. }) => {
                Rc::ptr_eq(target, shared)
            }
            _ => false,
        }
    }
}

/// If `profile` is a `Profile::Group` or `Profile::Path`, returns its
/// shared state (cloning the `Rc`, not the underlying move) — used to
/// detect group membership when a single member is about to be
/// replaced/cleared, so every other member can be cascaded into a stop too
/// (a group/path move missing one of its members no longer means
/// anything).
fn group_of(profile: &Profile) -> Option<ActiveGroupMove> {
    match profile {
        Profile::Group { shared, .. } => Some(ActiveGroupMove::Group(Rc::clone(shared))),
        Profile::Path { shared, .. } => Some(ActiveGroupMove::Path(Rc::clone(shared))),
        _ => None,
    }
}

/// Cascades a group (or path) interruption: every member of `shared` other
/// than `except` gets its own `StopRamp` from its own commanded
/// position/velocity, via the same `abort_into` mechanism a direct
/// single-axis `stop` uses — no new profile-building logic, just invoked
/// once per sibling. Guarded by `ActiveGroupMove::still_drives` (identity,
/// via `Rc::ptr_eq`) so a member that's no longer actually part of *this*
/// specific move (it already moved on to something else, including a
/// different group/path move) is left alone: this is what makes
/// double-interruption-in-the-same-cycle and asymmetric disable/fault
/// timing between members both come out correct rather than double-firing
/// or clobbering unrelated state.
/// `except` is the axis that caused the interruption and has already been
/// given its own new profile — `None` when nothing is to blame in
/// particular and *every* member should be stopped, which is what a runtime
/// IK failure does. `reason` is the parenthetical in each member's stop
/// message, so a kinematic failure doesn't report itself as an ordinary
/// group interruption.
fn cascade_group_stop(
    axes: &mut [AxisRuntime],
    group_pending: &mut [VecDeque<PendingGroupMove>],
    shared: &ActiveGroupMove,
    except: Option<usize>,
    reason: &str,
) {
    // A group/path move missing one of its members no longer means
    // anything, so neither do any of *its own* queued follow-up moves —
    // same "seizing/losing control clears anything queued" precedent as
    // `abort_into`.
    group_pending[shared.group()].clear();
    for &other in shared.axes() {
        // Each sibling decelerates at *its own* limit, not the group move's.
        // The group's `max_deceleration` is a Cartesian TCP quantity, and
        // this ramp is built in joint space — see `AxisConfig`.
        let decel = AXIS_CONFIGS[other].max_deceleration;
        let units = AXIS_CONFIGS[other].units;
        if Some(other) == except {
            continue;
        }
        let still_in_this_move = match &axes[other].active {
            Some(active) => shared.still_drives(&active.profile),
            None => false,
        };
        if !still_in_this_move {
            continue;
        }
        abort_into(
            &mut axes[other],
            other,
            "stop",
            move |position, velocity| {
                let ramp = StopRamp::new(position, velocity, decel)?;
                println!(
                    "  -> {}: stopping ({reason}): {:.3} {units}/s -> 0 (decel {decel:.3} {units}/s^2, {:.3}s)",
                    axis_label(other),
                    velocity,
                    ramp.duration()
                );
                Ok(Profile::Stop(ramp))
            },
        );
    }
}

/// Gathers one value per group member into a `KinematicVector` — the
/// commanded joint position or velocity, ready to hand to the group's
/// kinematic model. Fails only on a non-finite value or an over-long group,
/// both of which `KinematicVector::from_slice` checks.
fn commanded_vector(
    axes: &[AxisRuntime],
    members: &[usize],
    field: impl Fn(&AxisRuntime) -> f64,
) -> Result<motion_core::KinematicVector, motion_core::KinematicsError> {
    let values: Vec<f64> = members.iter().map(|&a| field(&axes[a])).collect();
    motion_core::KinematicVector::from_slice(&values)
}

/// Why a group or path move couldn't be installed: either the profile
/// construction rejected the geometry/limits, or the kinematic model
/// rejected the target (unreachable, or a dimension mismatch that means the
/// group and its model disagree on arity).
///
/// One enum over all three because the call sites only ever print whichever
/// they got — they already funnelled `LinearMoveError` and
/// `PathProfileError` through `.to_string()` for exactly that reason.
#[derive(Debug)]
enum GroupMoveError {
    LinearMove(LinearMoveError),
    Path(PathProfileError),
    Kinematics(motion_core::KinematicsError),
}

impl std::fmt::Display for GroupMoveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GroupMoveError::LinearMove(e) => write!(f, "{e}"),
            GroupMoveError::Path(e) => write!(f, "{e}"),
            GroupMoveError::Kinematics(e) => write!(f, "{e}"),
        }
    }
}

impl From<LinearMoveError> for GroupMoveError {
    fn from(e: LinearMoveError) -> Self {
        GroupMoveError::LinearMove(e)
    }
}

impl From<PathProfileError> for GroupMoveError {
    fn from(e: PathProfileError) -> Self {
        GroupMoveError::Path(e)
    }
}

impl From<motion_core::KinematicsError> for GroupMoveError {
    fn from(e: motion_core::KinematicsError) -> Self {
        GroupMoveError::Kinematics(e)
    }
}

/// Attempts to build a `LinearMove` for `group`'s `members` from their
/// current *commanded* position/velocity (see
/// `AxisRuntime::commanded_position`), mapped through the group's kinematic
/// model into Cartesian task space — the profile it builds is Cartesian
/// throughout, and the per-cycle inverse conversion back to joint setpoints
/// happens in the control loop. Only on success — including the target's
/// reachability, checked by a dry-run `inverse_position` — installs it
/// atomically as `Profile::Group` on every member (also clearing each
/// member's own individual pending queue, same as `abort_into`). On
/// failure nothing is touched, so a rejected group move never leaves one
/// member half-redirected. Returns the built profile's duration for the
/// caller's own success message — wording differs between an immediate
/// move and one just promoted off the group's queue, so this doesn't print
/// anything itself. Deliberately does **not** touch `group_pending` itself
/// (unlike `cascade_group_stop`) — the group-promotion loop below is
/// already draining that queue via `pop_front`, and clearing it here would
/// wrongly discard whatever's still queued behind the move just promoted.
fn install_group_move(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    targets: Vec<f64>,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
) -> Result<f64, GroupMoveError> {
    let model = AXIS_GROUPS[group].kinematics;
    let joint_start = commanded_vector(axes, members, |ax| ax.commanded_position)?;
    let joint_velocity = commanded_vector(axes, members, |ax| ax.commanded_velocity)?;

    // Branch first, from the commanded joint pose — everything below is
    // built on it, including the target's reachability check.
    let branch = model.resolve_branch(joint_start);
    let start = model.forward_position(joint_start);
    let start_velocity = model.forward_velocity(joint_start, joint_velocity);

    // Dry-run IK on the target before building anything: an unreachable or
    // singular endpoint is rejected here, where the user typed it, rather
    // than surfacing mid-move as a cascade stop.
    let joint_target =
        model.inverse_position(motion_core::KinematicVector::from_slice(&targets)?, branch)?;

    let profile = LinearMove::new_with_start_velocity(
        start.as_slice().to_vec(),
        start_velocity.as_slice().to_vec(),
        targets,
        max_speed,
        max_acceleration,
        max_deceleration,
    )?;
    let duration = profile.duration();
    let shared = Rc::new(SharedGroupMove {
        profile,
        axes: members,
        group,
        branch,
        joint_target,
    });
    for (index, &axis) in members.iter().enumerate() {
        let ax = &mut axes[axis];
        ax.pending.clear();
        ax.active = Some(ActiveMove {
            profile: Profile::Group {
                shared: Rc::clone(&shared),
                index,
            },
            started_at: Instant::now(),
        });
        ax.last_phase = None;
    }
    Ok(duration)
}

/// The path-move analog of `install_group_move`: attempts to build a
/// `PathProfile` for `group`'s `members`, prepending their current
/// *commanded* position as the path's own start waypoint (`waypoints` is everything
/// *after* that — see `Command::MovePath`'s docs), and — only on success —
/// installs it atomically as `Profile::Path` on every member. On failure
/// nothing is touched, same "never half-redirect a member" guarantee
/// `install_group_move` gives. Every segment is a smooth spline blending
/// through every waypoint — `motion_core::WaypointPath` no longer offers a
/// per-segment straight-line override (see its module docs).
///
/// `build` is which `PathProfile` constructor to use — always given each
/// member's current *commanded* velocity, exactly like `install_group_move`
/// always uses `LinearMove::new_with_start_velocity` (an idle group's
/// velocity is simply 0, which reduces to ordinary rest-to-rest behavior,
/// so there's no separate idle-only code path). `install_path_move` and
/// `install_path_move_blended` are both thin callers of this, differing
/// only in which constructor they pass — `Aborting` vs `Blend` is entirely
/// a choice of geometry-building strategy, not a different installation
/// mechanism.
fn install_path_move_impl(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    waypoints: Vec<Vec<f64>>,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
    build: impl FnOnce(Vec<Vec<f64>>, Vec<f64>, f64, f64, f64) -> Result<PathProfile, PathProfileError>,
) -> Result<f64, GroupMoveError> {
    let model = AXIS_GROUPS[group].kinematics;
    let joint_start = commanded_vector(axes, members, |ax| ax.commanded_position)?;
    let joint_velocity = commanded_vector(axes, members, |ax| ax.commanded_velocity)?;

    let branch = model.resolve_branch(joint_start);
    let start = model.forward_position(joint_start);
    let start_velocity = model.forward_velocity(joint_start, joint_velocity);

    // Every explicitly-given waypoint is IK'd up front, not just the final
    // target: a few extra calls, once, at install time, and an obviously
    // unreachable path is rejected before it starts rather than cascading
    // to a stop somewhere in the middle of it. All on the one resolved
    // branch — a path that would need the elbow to flip partway isn't a
    // path this can run. Says nothing about the *interior* of a segment;
    // that needs whole-path plausibility checking, still unbuilt.
    let mut joint_target = joint_start;
    for waypoint in &waypoints {
        joint_target =
            model.inverse_position(motion_core::KinematicVector::from_slice(waypoint)?, branch)?;
    }

    let mut all_waypoints = Vec::with_capacity(waypoints.len() + 1);
    all_waypoints.push(start.as_slice().to_vec());
    all_waypoints.extend(waypoints);

    let profile = build(
        all_waypoints,
        start_velocity.as_slice().to_vec(),
        max_speed,
        max_acceleration,
        max_deceleration,
    )?;
    let duration = profile.duration();
    let shared = Rc::new(SharedPathMove {
        profile,
        axes: members,
        group,
        branch,
        joint_target,
    });
    for (index, &axis) in members.iter().enumerate() {
        let ax = &mut axes[axis];
        ax.pending.clear();
        ax.active = Some(ActiveMove {
            profile: Profile::Path {
                shared: Rc::clone(&shared),
                index,
            },
            started_at: Instant::now(),
        });
        ax.last_phase = None;
    }
    Ok(duration)
}

/// `BufferMode::Aborting` (and idle/`Buffered`) path installation: built via
/// `PathProfile::new_with_start_velocity` — see `install_path_move_impl`.
fn install_path_move(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    waypoints: Vec<Vec<f64>>,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
) -> Result<f64, GroupMoveError> {
    install_path_move_impl(
        axes,
        group,
        members,
        waypoints,
        max_speed,
        max_acceleration,
        max_deceleration,
        PathProfile::new_with_start_velocity,
    )
}

/// `BufferMode::Blend` path installation: built via `PathProfile::new_blended`
/// instead — see `install_path_move_impl`.
fn install_path_move_blended(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    waypoints: Vec<Vec<f64>>,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
) -> Result<f64, GroupMoveError> {
    install_path_move_impl(
        axes,
        group,
        members,
        waypoints,
        max_speed,
        max_acceleration,
        max_deceleration,
        PathProfile::new_blended,
    )
}

/// Per-axis command handlers, extracted so a hard-coded axis group (see
/// `AXIS_GROUPS`) can fan the same logic out to each of its members with a
/// simple loop, instead of duplicating it. Called once each from the
/// single-axis `Command::Enable`/`Disable`/`Reset`/`Stop` arms below — pure
/// relocation, not a behavior change.
fn handle_enable(ax: &mut AxisRuntime, axis: usize) {
    if ax.axis_state == AxisState::ErrorStop {
        // Must refuse outright, not just leave `want_enabled` false and
        // hope — an `enable` that arrives anywhere during the fault window
        // must not "stick" and silently fire the instant a later `reset`
        // clears the fault, without a fresh `enable` after it.
        println!(
            "  ! {}: cannot enable: axis has a fault — reset it first",
            axis_label(axis)
        );
    } else if ax.want_enabled && axis_operational(ax.axis_state) {
        println!("  -> {}: already enabled", axis_label(axis));
    } else {
        ax.want_enabled = true;
        println!("  -> {}: enabling...", axis_label(axis));
    }
}

fn handle_disable(ax: &mut AxisRuntime, axis: usize) {
    if !ax.want_enabled && ax.axis_state == AxisState::Disabled {
        println!("  -> {}: already disabled", axis_label(axis));
    } else {
        ax.want_enabled = false;
        println!("  -> {}: disabling...", axis_label(axis));
    }
}

fn handle_reset(ax: &mut AxisRuntime, axis: usize) {
    if ax.axis_state == AxisState::ErrorStop {
        ax.pending_fault_reset = true;
        println!("  -> {}: resetting fault...", axis_label(axis));
    } else {
        println!("  -> {}: no fault to reset", axis_label(axis));
    }
}

fn handle_stop(ax: &mut AxisRuntime, axis: usize, max_deceleration: Option<f64>) {
    if !axis_operational(ax.axis_state) {
        println!("  ! {}: stop rejected: axis is disabled", axis_label(axis));
    } else if ax.active.is_none() && ax.commanded_velocity.abs() < AT_REST_EPS {
        // Nothing queued survives this either way (see abort_into), but
        // there's genuinely nothing to decelerate — short-circuit before
        // building a zero-duration StopRamp just to say so.
        ax.pending.clear();
        println!("  -> {}: already at rest", axis_label(axis));
    } else {
        let decel = max_deceleration.unwrap_or(AXIS_CONFIGS[axis].max_deceleration);
        let units = AXIS_CONFIGS[axis].units;
        abort_into(ax, axis, "stop", move |position, velocity| {
            let ramp = StopRamp::new(position, velocity, decel)?;
            println!(
                "  -> {}: stopping: {:.3} {units}/s -> 0 (decel {decel:.3} {units}/s^2, {:.3}s)",
                axis_label(axis),
                velocity,
                ramp.duration()
            );
            Ok(Profile::Stop(ramp))
        });
    }
}

/// An in-progress move (or stop): the profile plus the wall-clock instant it
/// began. Profiles themselves only know relative/elapsed time (see the
/// `NOTE (dt seam)` comment in motion-core) — the loop is what anchors them
/// to wall-clock time.
struct ActiveMove {
    profile: Profile,
    started_at: Instant,
}

struct PendingMove {
    target: f64,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
}

/// The group-level analog of `PendingMove` — one entry in a hard-coded
/// group's own FIFO queue (`run_control_loop`'s `group_pending`, not part
/// of `AxisRuntime`: a group's queued move needs every member
/// simultaneously idle to promote, which doesn't fit inside any single
/// axis's own state). An enum, not a single struct, so a group's queue can
/// mix `move`s and `movepath`s in the order they were issued — the
/// promotion loop matches on the variant to call `install_group_move` or
/// `install_path_move` as appropriate.
enum PendingGroupMove {
    Move {
        targets: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    },
    Path {
        waypoints: Vec<Vec<f64>>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    },
}

/// All runtime state for one axis. Axes are independent: each has its own
/// position, its own in-progress move (if any), and its own single-slot
/// pending queue — nothing here is shared across axes.
struct AxisRuntime {
    // Last-known *actual* position/velocity, from backend feedback — not
    // the raw trajectory sample. Updated once per control cycle after the
    // backend exchange. Used for *reporting only* (`status`, the heartbeat
    // print, move-completion messages) and for the enable-time resync
    // below — never to seed a profile. See `commanded_position`.
    position: f64,
    velocity: f64,
    // The *commanded* (model) position/velocity: exactly what was sent as
    // this axis's `AxisSetpoint` last cycle. This — not feedback — is what
    // every profile is seeded from: a new move, an aborting redirect, a
    // `StopRamp`, a queue promotion, a group/path move's start state.
    //
    // Why the model and not feedback: seeding from actual injects a step
    // into the commanded stream. If the previous move ended commanding
    // 100.000 while the axis actually sits at 99.980 (following error), a
    // new move seeded from 99.980 steps the commanded position backward
    // 0.020 mm in one cycle — a step input to the drive. It also destroys
    // repeatability (the same command sequence produces a different
    // trajectory depending on measured error) and quietly launders
    // following error into the plan instead of leaving it visible to the
    // drive's own following-error detection, which is the mechanism
    // designed to catch it.
    //
    // Accepted consequence: if an axis physically can't keep up (jam,
    // overload), the model runs away from reality and following error
    // grows until the drive faults. That's correct — that fault is the
    // designed detector.
    //
    // Resynced from feedback at exactly one place: the transition from
    // non-operational to operational (see the feedback fold in the control
    // loop). Every path that stops accepting moves — disable, fault —
    // leaves the axis `Disabled`/`ErrorStop`, and `axis_operational` gates
    // every move/stop/promotion, so nothing can be commanded again without
    // crossing that transition first. Homing, when it exists, will be the
    // second resync point.
    commanded_position: f64,
    commanded_velocity: f64,
    active: Option<ActiveMove>,
    // FIFO queue of buffered moves waiting for the current one (and each
    // other) to finish, in order. An `aborting` move bypasses this
    // entirely — see `BufferMode` and `abort_into`.
    pending: VecDeque<PendingMove>,
    last_status_print: Instant,
    last_phase: Option<MotionPhase>,
    // What the user last asked for via `enable`/`disable` — sent to the
    // backend every cycle as `AxisSetpoint::enabled` (see axis-backend's
    // docs on why that's a cyclic field, not a one-off command).
    want_enabled: bool,
    // This axis's coarser status, updated from feedback
    // every cycle. Every business-logic gate in this loop
    // (`move`/`stop`/queue-promotion rejection, `enable`/`disable`'s
    // "already ..." checks, fault recovery) checks *this*, not
    // `ds402_state` below — keeps `app` backend-agnostic, the same view a
    // `backend-ethercat` swap-in would still report even with different
    // internal DS402 timing/quirks than `backend-sim`'s.
    axis_state: AxisState,
    // The backend-confirmed DS402 state, updated from feedback each cycle
    // — kept *only* to detect and print transitions for traceability (the
    // "     axisN: State -> State" lines below, and `status`'s trailing
    // `[Ds402State]`). Deliberately not read by any decision in this loop;
    // see `axis_state` above for why.
    ds402_state: Ds402State,
    // A one-shot pulse: set true when `reset` is issued, sent as
    // `AxisSetpoint::fault_reset` for exactly one cycle, then cleared —
    // mirrors DS402's edge-triggered "Fault Reset" controlword bit, which
    // isn't a held/level state like `enabled`.
    pending_fault_reset: bool,
}

impl AxisRuntime {
    fn new() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            commanded_position: 0.0,
            commanded_velocity: 0.0,
            active: None,
            pending: VecDeque::new(),
            last_status_print: Instant::now(),
            last_phase: None,
            want_enabled: false,
            axis_state: AxisState::Disabled,
            ds402_state: Ds402State::SwitchOnDisabled,
            pending_fault_reset: false,
        }
    }
}

fn run_control_loop(rx: Receiver<Command>, history: Arc<Mutex<History>>) {
    let dt = Duration::from_secs_f64(1.0 / CONTROL_RATE_HZ);
    let mut axes: Vec<AxisRuntime> = (0..NUM_AXES).map(|_| AxisRuntime::new()).collect();
    // One FIFO queue per hard-coded group (see `PendingGroupMove`'s docs
    // for why this can't just live inside `AxisRuntime`).
    let mut group_pending: Vec<VecDeque<PendingGroupMove>> =
        (0..AXIS_GROUPS.len()).map(|_| VecDeque::new()).collect();
    // Recording is a passive tap at the AxisGroup seam (see recording.rs) —
    // everything below this line is unchanged from before viz existed.
    let mut backend =
        RecordingAxisGroup::new(SimAxisGroup::new(NUM_AXES, dt.as_secs_f64()), history);

    // Fixed schedule anchored to a single start instant, so ticks don't
    // drift from accumulated sleep-call overhead.
    let schedule_start = Instant::now();
    let mut cycle: u64 = 0;

    // Gates only the periodic per-cycle position/phase heartbeat (see
    // step 4 below) — every other message (move started/finished/aborted,
    // enable/disable, faults, DS402 transitions) always prints regardless.
    let mut verbose = false;

    loop {
        // 1. If idle and moves are queued, start the next one now. A
        //    rejected candidate (bad kinematic params) is dropped and the
        //    next queued item is tried in the same cycle, rather than
        //    retrying the same bad one forever; an axis that's no longer
        //    enabled drops the whole queue at once with one message,
        //    instead of one drop-message per item per cycle.
        for (i, ax) in axes.iter_mut().enumerate() {
            if ax.active.is_some() || ax.pending.is_empty() {
                continue;
            }
            if !axis_operational(ax.axis_state) {
                let dropped = ax.pending.len();
                ax.pending.clear();
                println!(
                    "  ! {}: {dropped} queued move(s) dropped: axis is not enabled",
                    axis_label(i)
                );
                continue;
            }
            while let Some(p) = ax.pending.pop_front() {
                match TrapezoidalProfile::new(
                    ax.commanded_position,
                    p.target,
                    p.max_speed,
                    p.max_acceleration,
                    p.max_deceleration,
                ) {
                    Ok(profile) => {
                        println!(
                            "  -> {}: starting queued move: {:.3} -> {:.3} {units} ({:.3}s)",
                            axis_label(i),
                            ax.commanded_position,
                            p.target,
                            profile.duration(),
                            units = AXIS_CONFIGS[i].units
                        );
                        ax.active = Some(ActiveMove {
                            profile: Profile::Move(profile),
                            started_at: Instant::now(),
                        });
                        break;
                    }
                    Err(e) => {
                        println!("  ! {}: queued move rejected: {e}", axis_label(i));
                    }
                }
            }
        }

        // 1b. Same promotion, one level up: if a *group* is idle — every
        //     member simultaneously, not just one axis — and has a queued
        //     move, start it now via install_group_move (atomic across the
        //     whole group, unlike the per-axis loop above).
        for (g, group) in AXIS_GROUPS.iter().enumerate() {
            if group_pending[g].is_empty() {
                continue;
            }
            let members = group.axes;
            let busy = members
                .iter()
                .any(|&axis| axes[axis].active.is_some() || !axes[axis].pending.is_empty());
            if busy {
                continue;
            }
            let all_operational = members
                .iter()
                .all(|&axis| axis_operational(axes[axis].axis_state));
            if !all_operational {
                let dropped = group_pending[g].len();
                group_pending[g].clear();
                println!(
                    "  ! {}: {dropped} queued move(s) dropped: not every member is enabled",
                    group.name
                );
                continue;
            }
            while let Some(p) = group_pending[g].pop_front() {
                // Both install_* return different error types (LinearMoveError
                // vs PathProfileError) — .to_string() unifies them, since the
                // caller only ever needs to print whichever it got.
                let result = match p {
                    PendingGroupMove::Move {
                        targets,
                        max_speed,
                        max_acceleration,
                        max_deceleration,
                    } => install_group_move(
                        &mut axes,
                        g,
                        members,
                        targets,
                        max_speed,
                        max_acceleration,
                        max_deceleration,
                    )
                    .map_err(|e| e.to_string()),
                    PendingGroupMove::Path {
                        waypoints,
                        max_speed,
                        max_acceleration,
                        max_deceleration,
                    } => install_path_move(
                        &mut axes,
                        g,
                        members,
                        waypoints,
                        max_speed,
                        max_acceleration,
                        max_deceleration,
                    )
                    .map_err(|e| e.to_string()),
                };
                match result {
                    Ok(duration) => {
                        println!("  -> {}: starting queued move ({duration:.3}s)", group.name);
                        break;
                    }
                    Err(e) => {
                        println!("  ! {}: queued move rejected: {e}", group.name);
                    }
                }
            }
        }

        // 1c. Kinematics, once per active group: sample the group's shared
        //     *Cartesian* profile and convert it into joint position and
        //     velocity for its members. Done here, ahead of the per-axis
        //     setpoint pass, for two reasons — the conversion is per group,
        //     not per axis (an arm's joint 1 setpoint depends on the whole
        //     Cartesian point, not on one coordinate of it), and every
        //     member is then sampled at exactly the same instant rather than
        //     each at its own `elapsed()`.
        //
        //     Under `IdentityKinematics` this is a pass-through, so a
        //     Cartesian group's setpoints are unchanged.
        //
        //     The branch comes from the move — resolved once at install,
        //     never re-resolved here. See `SharedGroupMove::branch`.
        let mut group_joint: Vec<
            Option<(motion_core::KinematicVector, motion_core::KinematicVector)>,
        > = vec![None; AXIS_GROUPS.len()];
        // A failed conversion can't cascade from inside this pass (that
        // needs `&mut` across the whole slice), so failures are collected
        // and applied after the setpoint map — the same collect-then-apply
        // shape the feedback fold below already uses.
        let mut ik_failures: Vec<(ActiveGroupMove, String)> = Vec::new();
        for (g, group) in AXIS_GROUPS.iter().enumerate() {
            // Any member driving this group's move is a valid
            // representative — they share one profile and one start
            // instant, and only one shared move per group can be live at a
            // time (installing a new one replaces every member; a cascade
            // clears every member that's left).
            let Some(shared) = group
                .axes
                .iter()
                .find_map(|&axis| match &axes[axis].active {
                    Some(mv) => match group_of(&mv.profile) {
                        Some(shared) if shared.group() == g => Some((shared, mv.started_at)),
                        _ => None,
                    },
                    None => None,
                })
            else {
                continue;
            };
            let (shared, started_at) = shared;
            let elapsed = started_at.elapsed().as_secs_f64();
            let (cartesian_position, cartesian_velocity, branch) = shared.sample(elapsed);

            let model = group.kinematics;
            let converted =
                model
                    .inverse_position(cartesian_position, branch)
                    .and_then(|joint_position| {
                        let joint_velocity =
                            model.inverse_velocity(joint_position, cartesian_velocity)?;
                        Ok((joint_position, joint_velocity))
                    });
            match converted {
                Ok(pair) => group_joint[g] = Some(pair),
                // Holding position next to a singularity is exactly what
                // doesn't recover — zero velocity there stays there. Every
                // member ramps down independently in joint space instead,
                // which is singularity-free by construction.
                Err(e) => ik_failures.push((shared, format!("kinematics failed: {e}"))),
            }
        }

        // 1d. Apply any kinematic failure from 1c, *before* this cycle's
        //     setpoints are built. Ordering matters and is not arbitrary:
        //     each member's `StopRamp` starts from its `commanded_velocity`,
        //     and the setpoint pass below is what overwrites that. Cascade
        //     after it and every ramp starts from the zero velocity the
        //     failing cycle just wrote — an instantaneous stop, which is the
        //     step discontinuity the whole model-chain rule exists to
        //     prevent, delivered at the worst possible moment. Cascading
        //     first, the ramps are installed with `started_at` = now, and
        //     the setpoint pass samples them at t ~= 0, which is exactly the
        //     velocity the group was already commanding.
        //
        //     `except: None` — nothing is to blame in particular, so every
        //     member ramps down. Double-firing is already guarded by
        //     `still_drives`.
        for (shared, reason) in ik_failures {
            println!("  ! {}: {reason}", group_label(shared.group()));
            cascade_group_stop(
                &mut axes,
                &mut group_pending,
                &shared,
                None,
                "kinematics failed",
            );
        }

        // 2. Build this cycle's commanded setpoint for every axis: sample
        //    the active trajectory if there is one, otherwise hold at the
        //    axis's last *commanded* position with zero velocity.
        //
        //    Holding at commanded rather than actual is what keeps the
        //    commanded stream continuous across the gap between moves — an
        //    idle axis must keep asking for the same place it last asked
        //    for, not drift onto wherever the servo settled. Each setpoint
        //    is written back to `commanded_position`/`commanded_velocity`
        //    here, which is the single point where the model chain
        //    advances; see `AxisRuntime::commanded_position`.
        let setpoints: Vec<AxisSetpoint> = axes
            .iter_mut()
            .map(|ax| {
                // Consume the one-shot reset pulse right here — it's sent
                // in this cycle's setpoint and must not repeat next cycle.
                let fault_reset = std::mem::take(&mut ax.pending_fault_reset);
                let setpoint = match &ax.active {
                    // A group/path member takes its component from the
                    // joint vector computed once for the whole group in
                    // step 1c, rather than sampling the shared Cartesian
                    // profile itself — that sample is task-space, and only
                    // under identity kinematics is component `index` of it
                    // also this axis's setpoint.
                    Some(mv) => match mv.profile.group_and_index() {
                        Some((g, index)) => match &group_joint[g] {
                            Some((position, velocity)) => AxisSetpoint {
                                position: position.as_slice()[index],
                                velocity: velocity.as_slice()[index],
                                enabled: ax.want_enabled,
                                fault_reset,
                                stopping: mv.profile.is_stop(),
                            },
                            // Kinematics failed for this group and the
                            // cascade in 1d couldn't even build this axis a
                            // stop ramp (so it's still pointed at the dead
                            // group move). Command what an idle axis
                            // commands: hold at commanded
                            // position, zero velocity. (Holding at
                            // *feedback* would inject a step discontinuity
                            // at the worst possible moment.)
                            None => AxisSetpoint {
                                position: ax.commanded_position,
                                velocity: 0.0,
                                enabled: ax.want_enabled,
                                fault_reset,
                                stopping: false,
                            },
                        },
                        None => {
                            let elapsed = mv.started_at.elapsed().as_secs_f64();
                            let sample = mv.profile.sample(elapsed);
                            AxisSetpoint {
                                position: sample.position,
                                velocity: sample.velocity,
                                enabled: ax.want_enabled,
                                fault_reset,
                                stopping: mv.profile.is_stop(),
                            }
                        }
                    },
                    None => AxisSetpoint {
                        position: ax.commanded_position,
                        velocity: 0.0,
                        enabled: ax.want_enabled,
                        fault_reset,
                        stopping: false,
                    },
                };
                ax.commanded_position = setpoint.position;
                ax.commanded_velocity = setpoint.velocity;
                setpoint
            })
            .collect();

        // 3. One combined cyclic exchange with the backend (sim today, real
        //    drives later — see axis-backend). setpoints.len() always equals
        //    NUM_AXES by construction above, so a count mismatch here would
        //    be a bug in this loop, not bad input — hence expect(), not a
        //    Result the caller has to handle.
        let feedback = backend
            .exchange(&setpoints)
            .expect("setpoints always match backend's axis count");

        // 4. Fold feedback into each axis's state: update actual
        //    position/velocity, report any backend fault, and detect
        //    move completion / phase changes using the *actual* feedback
        //    position (which may differ, by a tiny discretization amount,
        //    from the trajectory's idealized target).
        // Collected here (not applied until after the loop, since
        // `axes.iter_mut()` below can't hold a `&mut` into one element
        // while `cascade_group_stop` also needs to touch others by index)
        // whenever a group member goes non-operational mid-loop.
        let mut group_cascades: Vec<(ActiveGroupMove, usize)> = Vec::new();

        for (i, ax) in axes.iter_mut().enumerate() {
            let was_operational = axis_operational(ax.axis_state);
            ax.position = feedback[i].position;
            ax.velocity = feedback[i].velocity;
            ax.axis_state = feedback[i].state;

            // The one resync point: coming back from non-operational
            // (`Disabled`/`ErrorStop`) to operational. While the power
            // stage was off the axis could have moved for reasons the
            // model knows nothing about — coasting, gravity, a fault
            // reaction, or a hand — so the commanded chain is stale and
            // the only truth available is where the axis actually is.
            // Everywhere else the model leads and feedback is reporting
            // only; see `AxisRuntime::commanded_position`.
            if !was_operational && axis_operational(ax.axis_state) {
                ax.commanded_position = ax.position;
                ax.commanded_velocity = ax.velocity;
            }

            // A fault means the last enable request is void — recovery is
            // an explicit reset, then a fresh enable, not an automatic
            // re-enable once the fault clears. Checked every cycle (not
            // just on the transition into ErrorStop) rather than
            // edge-detected — simpler, and harmless to repeat while the
            // fault persists.
            if ax.axis_state == AxisState::ErrorStop {
                ax.want_enabled = false;
            }

            // Surface DS402 transitions as they're confirmed by the
            // backend — purely for traceability (see `ds402_state`'s
            // docs on `AxisRuntime`); nothing here feeds a decision. At
            // 250 Hz a full enable/disable sequence (3 transitions)
            // finishes in ~12ms, so these prints arrive essentially
            // back-to-back, not spread over noticeable time.
            if feedback[i].ds402_state != ax.ds402_state {
                let was_fault = ax.ds402_state == Ds402State::Fault;
                println!(
                    "     {}: {:?} -> {:?}",
                    axis_label(i),
                    ax.ds402_state,
                    feedback[i].ds402_state
                );
                ax.ds402_state = feedback[i].ds402_state;
                match ax.ds402_state {
                    Ds402State::OperationEnabled => {
                        println!("  -> {}: enabled", axis_label(i))
                    }
                    Ds402State::SwitchOnDisabled if was_fault => {
                        println!("  -> {}: fault reset, disabled", axis_label(i))
                    }
                    Ds402State::SwitchOnDisabled => {
                        println!("  -> {}: disabled", axis_label(i))
                    }
                    Ds402State::FaultReactionActive => {
                        if let Some(fault) = feedback[i].fault {
                            println!("  ! {}: FAULT: {fault:?}", axis_label(i));
                        }
                    }
                    _ => {}
                }
            }

            // A move can't continue on an axis that isn't fully enabled —
            // e.g. the user just disabled it, or a fault just knocked it
            // out of OperationEnabled.
            if !axis_operational(ax.axis_state) {
                if let Some(mv) = &ax.active {
                    let word = if mv.profile.is_stop() { "stop" } else { "move" };
                    if feedback[i].fault.is_some() {
                        println!("  ! {}: {word} faulted", axis_label(i));
                    } else {
                        println!(
                            "  ! {}: {word} aborted: axis is no longer enabled",
                            axis_label(i)
                        );
                    }
                    if let Some(shared) = group_of(&mv.profile) {
                        group_cascades.push((shared, i));
                    }
                    ax.active = None;
                    ax.last_phase = None;
                }
                continue;
            }

            if let Some(mv) = &ax.active {
                let elapsed = mv.started_at.elapsed().as_secs_f64();
                let phase = mv.profile.phase_at(elapsed);

                if phase == MotionPhase::Done {
                    if mv.profile.is_stop() {
                        println!(
                            "  -> {}: stopped at {:.3} {}",
                            axis_label(i),
                            ax.position,
                            AXIS_CONFIGS[i].units
                        );
                    } else {
                        println!(
                            "  -> {}: reached {:.3} {}",
                            axis_label(i),
                            ax.position,
                            AXIS_CONFIGS[i].units
                        );
                    }
                    ax.active = None;
                    ax.last_phase = None;
                } else {
                    let phase_changed = ax.last_phase != Some(phase);
                    let now = Instant::now();
                    if verbose
                        && (phase_changed
                            || now.duration_since(ax.last_status_print) >= STATUS_PRINT_PERIOD)
                    {
                        println!(
                            "     {}  t={elapsed:>6.3}s  pos={:>9.3} {units}  vel={:>8.3} {units}/s  {}",
                            axis_label(i),
                            ax.position,
                            ax.velocity,
                            phase_label(phase),
                            units = AXIS_CONFIGS[i].units
                        );
                        ax.last_status_print = now;
                    }
                    ax.last_phase = Some(phase);
                }
            }
        }

        // Apply any group cascades collected above, now that the borrow
        // from `axes.iter_mut()` has ended.
        for (shared, faulted_axis) in group_cascades {
            cascade_group_stop(
                &mut axes,
                &mut group_pending,
                &shared,
                Some(faulted_axis),
                "group interrupted",
            );
        }

        // 5. Drain any commands that arrived since the last tick.
        while let Ok(cmd) = rx.try_recv() {
            match cmd {
                Command::Move {
                    axis,
                    target,
                    max_speed,
                    max_acceleration,
                    max_deceleration,
                    buffer_mode,
                } => {
                    let ax = &mut axes[axis];
                    if !axis_operational(ax.axis_state) {
                        println!(
                            "  ! {}: move rejected: axis is disabled (enable it first)",
                            axis_label(axis)
                        );
                    } else if let Err(e) = check_target_in_limits(axis, target) {
                        // Checked when the command is accepted, so a
                        // `buffered` move is rejected at the point the user
                        // typed it rather than silently sitting in the queue
                        // until promotion.
                        println!("  ! {}: move rejected: {e}", axis_label(axis));
                    } else {
                        match buffer_mode {
                            // Blend is movepath-only and never actually
                            // produced by move's own parser (see
                            // parse_buffer_mode_and_limits) — grouped with
                            // Aborting here purely so this match stays
                            // exhaustive over BufferMode's 3 variants;
                            // genuinely unreachable via any real command.
                            BufferMode::Aborting | BufferMode::Blend => {
                                // If this axis was part of an active group
                                // move, redirecting it alone leaves that
                                // group meaning nothing — cascade the rest
                                // of it into its own stop, same as a direct
                                // `stop`/fault/disable would.
                                let previous_group =
                                    ax.active.as_ref().and_then(|mv| group_of(&mv.profile));
                                abort_into(ax, axis, "move", move |position, velocity| {
                                    let profile = TrapezoidalProfile::new_with_start_velocity(
                                        position,
                                        velocity,
                                        target,
                                        max_speed,
                                        max_acceleration,
                                        max_deceleration,
                                    )?;
                                    println!(
                                        "  -> {}: move (aborting): {:.3} -> {target:.3} {units} ({:.3}s)",
                                        axis_label(axis),
                                        position,
                                        profile.duration(),
                                        units = AXIS_CONFIGS[axis].units
                                    );
                                    Ok(Profile::Move(profile))
                                });
                                if let Some(shared) = previous_group {
                                    cascade_group_stop(
                                        &mut axes,
                                        &mut group_pending,
                                        &shared,
                                        Some(axis),
                                        "group interrupted",
                                    );
                                }
                            }
                            BufferMode::Buffered if ax.active.is_none() => {
                                match TrapezoidalProfile::new(
                                    ax.commanded_position,
                                    target,
                                    max_speed,
                                    max_acceleration,
                                    max_deceleration,
                                ) {
                                    Ok(profile) => {
                                        println!(
                                            "  -> {}: move: {:.3} -> {target:.3} {units} ({:.3}s)",
                                            axis_label(axis),
                                            ax.commanded_position,
                                            profile.duration(),
                                            units = AXIS_CONFIGS[axis].units
                                        );
                                        ax.active = Some(ActiveMove {
                                            profile: Profile::Move(profile),
                                            started_at: Instant::now(),
                                        });
                                    }
                                    Err(e) => {
                                        println!("  ! {}: move rejected: {e}", axis_label(axis));
                                    }
                                }
                            }
                            BufferMode::Buffered => {
                                println!(
                                    "  -> {}: busy: queuing move to {target:.3} {units} ({} already queued)",
                                    axis_label(axis),
                                    ax.pending.len(),
                                    units = AXIS_CONFIGS[axis].units
                                );
                                ax.pending.push_back(PendingMove {
                                    target,
                                    max_speed,
                                    max_acceleration,
                                    max_deceleration,
                                });
                            }
                        }
                    }
                }
                Command::Stop {
                    axis,
                    max_deceleration,
                } => {
                    // Same cascade reasoning as an Aborting move above: a
                    // lone member being stopped directly still needs to
                    // take the rest of its group down with it.
                    let previous_group = axes[axis]
                        .active
                        .as_ref()
                        .and_then(|mv| group_of(&mv.profile));
                    handle_stop(&mut axes[axis], axis, max_deceleration);
                    if let Some(shared) = previous_group {
                        cascade_group_stop(
                            &mut axes,
                            &mut group_pending,
                            &shared,
                            Some(axis),
                            "group interrupted",
                        );
                    }
                }
                Command::Enable { axis } => {
                    handle_enable(&mut axes[axis], axis);
                }
                Command::Disable { axis } => {
                    handle_disable(&mut axes[axis], axis);
                }
                Command::Reset { axis } => {
                    handle_reset(&mut axes[axis], axis);
                }
                Command::EnableGroup { group } => {
                    for &axis in AXIS_GROUPS[group].axes {
                        handle_enable(&mut axes[axis], axis);
                    }
                }
                Command::DisableGroup { group } => {
                    for &axis in AXIS_GROUPS[group].axes {
                        handle_disable(&mut axes[axis], axis);
                    }
                }
                Command::ResetGroup { group } => {
                    for &axis in AXIS_GROUPS[group].axes {
                        handle_reset(&mut axes[axis], axis);
                    }
                }
                Command::StopGroup {
                    group,
                    max_deceleration,
                } => {
                    // A stop cancels anything queued too, same as a
                    // single-axis stop already does.
                    group_pending[group].clear();
                    // Each member gets its own StopRamp from its own actual
                    // position/velocity, just like a direct single-axis
                    // stop — no shared "group stop ramp" is needed, a stop
                    // doesn't need to trace any particular shape.
                    for &axis in AXIS_GROUPS[group].axes {
                        handle_stop(&mut axes[axis], axis, max_deceleration);
                    }
                }
                Command::MoveGroup {
                    group,
                    targets,
                    max_speed,
                    max_acceleration,
                    max_deceleration,
                    buffer_mode,
                } => {
                    let members = AXIS_GROUPS[group].axes;
                    let all_operational = members
                        .iter()
                        .all(|&axis| axis_operational(axes[axis].axis_state));
                    let busy = members
                        .iter()
                        .any(|&axis| axes[axis].active.is_some() || !axes[axis].pending.is_empty());

                    if !all_operational {
                        println!(
                            "  ! {}: move rejected: not every member is enabled",
                            group_label(group)
                        );
                    } else if busy && buffer_mode == BufferMode::Buffered {
                        println!(
                            "  -> {}: busy: queuing move ({} already queued)",
                            group_label(group),
                            group_pending[group].len()
                        );
                        group_pending[group].push_back(PendingGroupMove::Move {
                            targets,
                            max_speed,
                            max_acceleration,
                            max_deceleration,
                        });
                    } else {
                        // Either idle, or Aborting redirecting a busy group
                        // right now — either way, seizing control clears
                        // anything this group had queued, same as
                        // `abort_into` does for a single axis.
                        group_pending[group].clear();
                        match install_group_move(
                            &mut axes,
                            group,
                            members,
                            targets,
                            max_speed,
                            max_acceleration,
                            max_deceleration,
                        ) {
                            Ok(duration) => {
                                println!("  -> {}: move ({duration:.3}s)", group_label(group));
                            }
                            Err(e) => {
                                println!("  ! {}: move rejected: {e}", group_label(group));
                            }
                        }
                    }
                }
                Command::MovePath {
                    group,
                    waypoints,
                    max_speed,
                    max_acceleration,
                    max_deceleration,
                    buffer_mode,
                } => {
                    let members = AXIS_GROUPS[group].axes;
                    let all_operational = members
                        .iter()
                        .all(|&axis| axis_operational(axes[axis].axis_state));
                    let busy = members
                        .iter()
                        .any(|&axis| axes[axis].active.is_some() || !axes[axis].pending.is_empty());
                    let n_waypoints = waypoints.len();

                    if !all_operational {
                        println!(
                            "  ! {}: movepath rejected: not every member is enabled",
                            group_label(group)
                        );
                    } else if busy && buffer_mode == BufferMode::Buffered {
                        println!(
                            "  -> {}: busy: queuing path move ({} already queued)",
                            group_label(group),
                            group_pending[group].len()
                        );
                        group_pending[group].push_back(PendingGroupMove::Path {
                            waypoints,
                            max_speed,
                            max_acceleration,
                            max_deceleration,
                        });
                    } else {
                        // Either idle, or Aborting/Blend redirecting a busy
                        // group right now (mid-flight, from wherever it
                        // actually is) — either way, seizing control clears
                        // anything this group had queued, same as
                        // MoveGroup. Only the choice of install function
                        // differs between Aborting and Blend — see
                        // install_path_move_impl's docs.
                        group_pending[group].clear();
                        let result = if buffer_mode == BufferMode::Blend {
                            install_path_move_blended(
                                &mut axes,
                                group,
                                members,
                                waypoints,
                                max_speed,
                                max_acceleration,
                                max_deceleration,
                            )
                        } else {
                            install_path_move(
                                &mut axes,
                                group,
                                members,
                                waypoints,
                                max_speed,
                                max_acceleration,
                                max_deceleration,
                            )
                        };
                        match result {
                            Ok(duration) => {
                                println!(
                                    "  -> {}: path move through {n_waypoints} waypoint(s) ({duration:.3}s)",
                                    group_label(group)
                                );
                            }
                            Err(e) => {
                                println!("  ! {}: movepath rejected: {e}", group_label(group));
                            }
                        }
                    }
                }
                Command::Verbose => {
                    verbose = !verbose;
                    println!(
                        "  -> verbose logging: {}",
                        if verbose { "on" } else { "off" }
                    );
                }
                Command::Status => print_status(&axes),
                Command::Help => print_help(),
                Command::Quit => {
                    println!("exiting.");
                    return;
                }
            }
        }

        // 6. Sleep to the next scheduled tick.
        cycle += 1;
        let next_tick = schedule_start + dt.mul_f64(cycle as f64);
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        }
    }
}

fn print_status(axes: &[AxisRuntime]) {
    for (i, ax) in axes.iter().enumerate() {
        match &ax.active {
            Some(mv) => {
                let elapsed = mv.started_at.elapsed().as_secs_f64();
                // Position/velocity come from the last backend feedback
                // (at most one cycle stale), not a fresh sample — matches
                // what the heartbeat print shows, and what's actually true
                // for the axis right now, not just what was commanded.
                let group_note = match mv.profile.group_membership() {
                    Some(name) => format!("  (group {name})"),
                    None => String::new(),
                };
                // `target()` is this axis's own joint-space endpoint — for
                // a group member that's the stored IK of the Cartesian
                // target, not one coordinate of it (see `Profile::target`).
                println!(
                    "  status: {}: pos={:.3} {units}  vel={:.3} {units}/s  {}  (target {:.3} {units})  [{:?}]{group_note}",
                    axis_label(i),
                    ax.position,
                    ax.velocity,
                    phase_label(mv.profile.phase_at(elapsed)),
                    mv.profile.target(),
                    ax.ds402_state,
                    units = AXIS_CONFIGS[i].units
                );
            }
            None => println!(
                "  status: {}: idle at {:.3} {}  [{:?}]",
                axis_label(i),
                ax.position,
                AXIS_CONFIGS[i].units,
                ax.ds402_state
            ),
        }
    }

    for (g, group) in AXIS_GROUPS.iter().enumerate() {
        // A group is "active" when one of its members is currently running
        // *this* group's shared move (a group move or a path move — any
        // member works as the representative, since they all share the
        // same profile/timing) — both `LinearMove` and `PathProfile` expose
        // the same `phase_at`/`target` shape, so this extracts a common
        // (phase, target) pair regardless of which one is actually active.
        let active = group
            .axes
            .iter()
            .find_map(|&axis| match &axes[axis].active {
                Some(mv) => {
                    let elapsed = mv.started_at.elapsed().as_secs_f64();
                    match &mv.profile {
                        Profile::Group { shared, .. } if shared.group == g => {
                            Some((shared.profile.phase_at(elapsed), shared.profile.target()))
                        }
                        Profile::Path { shared, .. } if shared.group == g => {
                            Some((shared.profile.phase_at(elapsed), shared.profile.target()))
                        }
                        _ => None,
                    }
                }
                None => None,
            });
        match active {
            Some((phase, target)) => {
                let target: Vec<String> = target.iter().map(|v| format!("{v:.3}")).collect();
                println!(
                    "  status: {}: {}  (target [{}] mm)",
                    group.name,
                    phase_label(phase),
                    target.join(", ")
                );
            }
            None => {
                // A group's position is its *TCP* position, so this is the
                // members' measured joint positions run through forward
                // kinematics — not the raw joint values, which are what the
                // per-axis lines above already report. Identical for a
                // Cartesian group; the whole point for an arm.
                let joints: Vec<f64> = group.axes.iter().map(|&axis| axes[axis].position).collect();
                let cartesian = motion_core::KinematicVector::from_slice(&joints)
                    .map(|j| group.kinematics.forward_position(j));
                match cartesian {
                    Ok(p) => {
                        let coords: Vec<String> =
                            p.as_slice().iter().map(|v| format!("{v:.3}")).collect();
                        println!(
                            "  status: {}: idle at [{}] mm",
                            group.name,
                            coords.join(", ")
                        );
                    }
                    // Only reachable if feedback itself went non-finite,
                    // which would be a backend bug — report it rather than
                    // printing a plausible-looking wrong number.
                    Err(e) => println!("  status: {}: position unavailable: {e}", group.name),
                }
            }
        }
    }
}

fn phase_label(phase: MotionPhase) -> &'static str {
    match phase {
        MotionPhase::Pre => "pre",
        MotionPhase::Accel => "accel",
        MotionPhase::Cruise => "cruise",
        MotionPhase::Decel => "decel",
        MotionPhase::Done => "done",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_axis_accepts_valid_names() {
        assert_eq!(parse_axis("axis0"), Ok(0));
        assert_eq!(parse_axis("axis1"), Ok(1));
    }

    // Phrased against NUM_AXES rather than a literal, so adding axes
    // doesn't turn this into a false failure the way it just did.
    #[test]
    fn parse_axis_rejects_out_of_range() {
        assert!(parse_axis(&format!("axis{NUM_AXES}")).is_err());
        assert!(parse_axis(&format!("axis{}", NUM_AXES + 7)).is_err());
    }

    #[test]
    fn parse_axis_rejects_malformed_names() {
        assert!(parse_axis("axisX").is_err());
        assert!(parse_axis("motor0").is_err());
        assert!(parse_axis("axis").is_err());
    }

    #[test]
    fn parse_command_blank_line_is_none() {
        assert_eq!(parse_command(""), Ok(None));
        assert_eq!(parse_command("   "), Ok(None));
    }

    #[test]
    fn parse_command_simple_verbs() {
        assert_eq!(parse_command("quit"), Ok(Some(Command::Quit)));
        assert_eq!(parse_command("exit"), Ok(Some(Command::Quit)));
        assert_eq!(parse_command("help"), Ok(Some(Command::Help)));
        assert_eq!(parse_command("status"), Ok(Some(Command::Status)));
        assert_eq!(parse_command("verbose"), Ok(Some(Command::Verbose)));
    }

    #[test]
    fn parse_command_enable_disable_reset() {
        assert_eq!(
            parse_command("enable axis0"),
            Ok(Some(Command::Enable { axis: 0 }))
        );
        assert_eq!(
            parse_command("disable axis1"),
            Ok(Some(Command::Disable { axis: 1 }))
        );
        assert_eq!(
            parse_command("reset axis0"),
            Ok(Some(Command::Reset { axis: 0 }))
        );
    }

    #[test]
    fn parse_command_stop_with_and_without_decel() {
        assert_eq!(
            parse_command("stop axis0"),
            Ok(Some(Command::Stop {
                axis: 0,
                max_deceleration: None
            }))
        );
        assert_eq!(
            parse_command("stop axis0 50"),
            Ok(Some(Command::Stop {
                axis: 0,
                max_deceleration: Some(50.0)
            }))
        );
    }

    #[test]
    fn parse_command_move_defaults() {
        assert_eq!(
            parse_command("move axis0 100"),
            Ok(Some(Command::Move {
                axis: 0,
                target: 100.0,
                max_speed: AXIS_CONFIGS[0].max_speed,
                max_acceleration: AXIS_CONFIGS[0].max_acceleration,
                max_deceleration: AXIS_CONFIGS[0].max_acceleration,
                buffer_mode: BufferMode::Buffered,
            }))
        );
    }

    #[test]
    fn target_within_travel_limits_is_accepted() {
        let (min, max) = AXIS_CONFIGS[0].position_limits.unwrap();
        assert!(check_target_in_limits(0, 0.0).is_ok());
        assert!(check_target_in_limits(0, max).is_ok(), "max is inclusive");
        assert!(check_target_in_limits(0, min).is_ok(), "min is inclusive");
    }

    #[test]
    fn target_outside_travel_limits_is_rejected() {
        let (min, max) = AXIS_CONFIGS[0].position_limits.unwrap();
        let over = check_target_in_limits(0, max + 1.0);
        assert!(over.is_err());
        // The message must name the axis's own units, not a hardcoded "mm".
        assert!(
            over.unwrap_err().contains(AXIS_CONFIGS[0].units),
            "rejection should report the axis's units"
        );
        assert!(check_target_in_limits(0, min - 1.0).is_err());
    }

    #[test]
    fn parse_command_move_explicit_kinematics() {
        assert_eq!(
            parse_command("move axis0 100 10 20 30"),
            Ok(Some(Command::Move {
                axis: 0,
                target: 100.0,
                max_speed: 10.0,
                max_acceleration: 20.0,
                max_deceleration: 30.0,
                buffer_mode: BufferMode::Buffered,
            }))
        );
    }

    #[test]
    fn parse_command_move_buffer_mode_keyword_always_trailing() {
        assert_eq!(
            parse_command("move axis0 100 10 20 30 aborting"),
            Ok(Some(Command::Move {
                axis: 0,
                target: 100.0,
                max_speed: 10.0,
                max_acceleration: 20.0,
                max_deceleration: 30.0,
                buffer_mode: BufferMode::Aborting,
            }))
        );
        // The keyword works even with no numeric args at all.
        assert_eq!(
            parse_command("move axis0 100 aborting"),
            Ok(Some(Command::Move {
                axis: 0,
                target: 100.0,
                max_speed: AXIS_CONFIGS[0].max_speed,
                max_acceleration: AXIS_CONFIGS[0].max_acceleration,
                max_deceleration: AXIS_CONFIGS[0].max_acceleration,
                buffer_mode: BufferMode::Aborting,
            }))
        );
    }

    #[test]
    fn parse_command_rejects_too_many_move_args() {
        assert!(parse_command("move axis0 100 1 2 3 4").is_err());
    }

    #[test]
    fn parse_command_rejects_bad_axis_or_number() {
        assert!(parse_command("enable axis9").is_err());
        assert!(parse_command("move axis0 not-a-number").is_err());
    }

    #[test]
    fn parse_command_rejects_unrecognized_input() {
        assert!(parse_command("frobnicate axis0").is_err());
    }

    #[test]
    fn parse_command_movepath_defaults() {
        assert_eq!(
            parse_command("movepath axisGroup0 2 10 0 10 10"),
            Ok(Some(Command::MovePath {
                group: 0,
                waypoints: vec![vec![10.0, 0.0], vec![10.0, 10.0]],
                max_speed: DEFAULT_CARTESIAN_MAX_SPEED,
                max_acceleration: DEFAULT_CARTESIAN_MAX_ACCELERATION,
                max_deceleration: DEFAULT_CARTESIAN_MAX_ACCELERATION,
                buffer_mode: BufferMode::Buffered,
            }))
        );
    }

    #[test]
    fn parse_command_movepath_explicit_kinematics() {
        assert_eq!(
            parse_command("movepath axisGroup0 1 5 5 10 20 30"),
            Ok(Some(Command::MovePath {
                group: 0,
                waypoints: vec![vec![5.0, 5.0]],
                max_speed: 10.0,
                max_acceleration: 20.0,
                max_deceleration: 30.0,
                buffer_mode: BufferMode::Buffered,
            }))
        );
    }

    #[test]
    fn parse_command_movepath_buffer_mode_keyword_always_trailing() {
        assert_eq!(
            parse_command("movepath axisGroup0 1 5 5 10 20 30 aborting"),
            Ok(Some(Command::MovePath {
                group: 0,
                waypoints: vec![vec![5.0, 5.0]],
                max_speed: 10.0,
                max_acceleration: 20.0,
                max_deceleration: 30.0,
                buffer_mode: BufferMode::Aborting,
            }))
        );
        // Works with no numeric args at all too.
        assert_eq!(
            parse_command("movepath axisGroup0 1 5 5 aborting"),
            Ok(Some(Command::MovePath {
                group: 0,
                waypoints: vec![vec![5.0, 5.0]],
                max_speed: DEFAULT_CARTESIAN_MAX_SPEED,
                max_acceleration: DEFAULT_CARTESIAN_MAX_ACCELERATION,
                max_deceleration: DEFAULT_CARTESIAN_MAX_ACCELERATION,
                buffer_mode: BufferMode::Aborting,
            }))
        );
    }

    #[test]
    fn parse_command_movepath_accepts_blend_keyword() {
        assert_eq!(
            parse_command("movepath axisGroup0 1 5 5 10 20 30 blend"),
            Ok(Some(Command::MovePath {
                group: 0,
                waypoints: vec![vec![5.0, 5.0]],
                max_speed: 10.0,
                max_acceleration: 20.0,
                max_deceleration: 30.0,
                buffer_mode: BufferMode::Blend,
            }))
        );
    }

    #[test]
    fn parse_command_move_does_not_accept_blend_keyword() {
        // "blend" isn't a recognized keyword for plain move — it falls
        // through to being parsed as a numeric kinematic-limit arg, and
        // fails as "not a number", not as a buffer mode.
        assert!(parse_command("move axis0 100 blend").is_err());
    }

    #[test]
    fn parse_command_movepath_rejects_axis_target() {
        assert!(parse_command("movepath axis0 1 5").is_err());
    }

    #[test]
    fn parse_command_movepath_rejects_zero_waypoints() {
        assert!(parse_command("movepath axisGroup0 0").is_err());
    }

    #[test]
    fn parse_command_movepath_rejects_wrong_coordinate_count() {
        // 2 waypoints x 2 axes = 4 coordinates needed, only 3 given.
        assert!(parse_command("movepath axisGroup0 2 10 0 10").is_err());
    }

    #[test]
    fn parse_command_movepath_rejects_too_many_trailing_args() {
        assert!(parse_command("movepath axisGroup0 1 5 5 10 20 30 40").is_err());
    }

    // --- parse_waypoint_lines (the file form's line-parsing, no filesystem
    // touched — see read_waypoints_file for the thin wrapper that does) ---

    #[test]
    fn parse_waypoint_lines_parses_one_waypoint_per_line() {
        assert_eq!(
            parse_waypoint_lines("10 0\n10 10\n0 20\n", 2),
            Ok(vec![vec![10.0, 0.0], vec![10.0, 10.0], vec![0.0, 20.0]])
        );
    }

    #[test]
    fn parse_waypoint_lines_skips_blank_lines() {
        assert_eq!(
            parse_waypoint_lines("\n10 0\n\n\n10 10\n\n", 2),
            Ok(vec![vec![10.0, 0.0], vec![10.0, 10.0]])
        );
    }

    #[test]
    fn parse_waypoint_lines_rejects_wrong_coordinate_count() {
        assert!(parse_waypoint_lines("10 0\n10\n", 2).is_err());
    }

    #[test]
    fn parse_waypoint_lines_rejects_non_numeric_token() {
        assert!(parse_waypoint_lines("10 abc\n", 2).is_err());
    }

    #[test]
    fn parse_waypoint_lines_rejects_empty_input() {
        assert!(parse_waypoint_lines("", 2).is_err());
        assert!(parse_waypoint_lines("\n\n  \n", 2).is_err());
    }

    #[test]
    fn parse_command_movepath_file_reads_waypoints() {
        let path = std::env::temp_dir().join(format!(
            "motion_project_movepath_test_{:?}.txt",
            std::thread::current().id()
        ));
        std::fs::write(&path, "10 0\n10 10\n").unwrap();
        let cmd = parse_command(&format!(
            "movepath axisGroup0 file {} 30 buffered",
            path.display()
        ));
        std::fs::remove_file(&path).unwrap();
        assert_eq!(
            cmd,
            Ok(Some(Command::MovePath {
                group: 0,
                waypoints: vec![vec![10.0, 0.0], vec![10.0, 10.0]],
                max_speed: 30.0,
                max_acceleration: DEFAULT_CARTESIAN_MAX_ACCELERATION,
                max_deceleration: DEFAULT_CARTESIAN_MAX_ACCELERATION,
                buffer_mode: BufferMode::Buffered,
            }))
        );
    }

    #[test]
    fn parse_command_movepath_file_missing_file_is_an_error() {
        assert!(parse_command("movepath axisGroup0 file /no/such/path.txt").is_err());
    }
}
