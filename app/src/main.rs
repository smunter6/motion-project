//! A continuously-running, multi-axis motion app driven from the terminal. A
//! fixed-rate control loop runs on its own schedule, decoupled from (blocking)
//! terminal input by a channel.
//!
//! Each cycle, the loop samples `motion-core`'s trajectory to get this cycle's
//! commanded (position, velocity, acceleration) and sends it through a
//! `backend_sim::SimAxisGroup` via the `AxisGroup` seam. Profiles are seeded
//! from the last commanded state, not from feedback (see `AxisRuntime`).
//! `SimAxisGroup` integrates velocity into its own position state, so there is
//! a small gap between "commanded" and "actual".
//!
//! Axes are independent: `move axis0 100` and `move axis1 50` run
//! concurrently on their own clocks, with no relationship between their
//! durations. Independently issued single-axis moves are not synchronized to
//! finish together.
//!
//! Run from the workspace root with:
//!
//!     cargo run -p app                 # terminal + viz window
//!     cargo run -p app -- --headless   # terminal only, no window
//!
//! `--headless` skips the viz window and runs the control loop on the main
//! thread. It is for scripted sessions (piping commands into stdin and
//! reading the output): a GUI window needs a display server, dominates
//! startup, and keeps the process alive after the control loop ends. Everything
//! except the plots behaves identically; see `scripts/demo_session.sh`.
//!
//! Commands (one per line on stdin). `help` prints the authoritative list at
//! runtime, including every configured axis and group name:
//!
//! ```text
//! move <target> <coord>... [vmax] [amax] [dmax] [aborting|buffered]
//! movepath <group> <n> <coord>...  [limits] [aborting|buffered|blend]
//! movepath <group> file <path>     [limits] [aborting|buffered|blend]
//! stop <target> [decel]
//! setlimits <target> [speed|accel|decel|jerk <value>]...
//! enable <target>
//! disable <target>
//! reset <target>
//! verbose
//! status
//! help
//! quit
//! ```
//!
//! A `<target>` is an axis (`axis0`) or a group (`axisGroup0`). Group
//! coordinates are Cartesian TCP mm; a single-axis move is raw joint space in
//! that axis's own units.
//!
//! Axes start disabled (DS402's `SwitchOnDisabled`); `move` on a disabled
//! axis is rejected, not queued. `enable` steps an axis through the DS402
//! sequence (`SwitchOnDisabled` -> `ReadyToSwitchOn` -> `SwitchedOn` ->
//! `OperationEnabled`), one transition per control cycle. See `axis-backend`'s
//! `Ds402State` docs.
//!
//! Disabling an axis that's moving is a fault, not a graceful power-down. A
//! faulted axis needs `reset` before it accepts `enable` again; `reset` alone
//! doesn't re-enable it.
//!
//! `stop` interrupts whatever an axis is doing (active or queued) with a
//! controlled deceleration to rest, built from the axis's current commanded
//! velocity (see `motion_core::StopRamp`). This is not DS402's Quick Stop: the
//! backend's `Ds402State` is unaffected, and only the coarser `AxisState` (via
//! `AxisSetpoint::stopping`) reports `Stopping` instead of `DiscreteMotion`.
//!
//! A `move`'s trailing `aborting`/`buffered` keyword picks its buffer mode.
//! `buffered` (the default) queues behind a busy axis, FIFO, and runs once
//! earlier moves finish. `aborting` takes over immediately, clearing anything
//! active or queued, and builds the new move from the axis's commanded
//! position and velocity via
//! `motion_core::TrapezoidalProfile::new_with_start_velocity` rather than
//! waiting for the axis to come to rest.
//!
//! `verbose` toggles the periodic per-cycle position/phase line printed while
//! an axis is moving (off by default). Discrete events (move
//! started/finished/aborted, enable/disable, faults, DS402 transitions) always
//! print.

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
    JerkFilteredProfile, LinearMove, LinearMoveError, MotionPhase, PathProfile, PathProfileError,
    StopRamp, TrajectoryError,
};
use recording::{History, RecordingAxisGroup};
use viz::VizApp;

const CONTROL_RATE_HZ: f64 = 250.0;

/// One thing's dynamic capability, in whatever units that thing works in —
/// mm/s for a linear axis or a Cartesian group's TCP, rad/s for a rotary
/// joint. Shared by `AxisConfig` (joint space, per axis) and `AxisGroupDef`
/// (task space, per group).
///
/// **Which one applies is a units question.** A single-axis `move axis3` is a
/// joint-space command and takes the axis's limits; a group move is a
/// TCP-space command and takes the group's, even though rotary joints carry it
/// out.
///
/// The tables below are the *startup* values. `setlimits` overrides them at
/// runtime in the control loop's own mutable copy; see `run_control_loop`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct MotionLimits {
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
    /// `None` means no jerk limit: the exact `max_jerk → ∞` case, where the
    /// profile is the plain trapezoid and nothing is filtered. See
    /// `motion_core::JerkFilteredProfile`.
    ///
    /// An `Option` rather than a large number because `motion-core` requires
    /// every limit it is given to be finite.
    max_jerk: Option<f64>,
}

/// Default limits for *Cartesian* (group / path) moves — TCP-space
/// quantities regardless of what kind of axes carry them out. Each group
/// copies this into its own entry in `AXIS_GROUPS`.
const DEFAULT_CARTESIAN_LIMITS: MotionLimits = MotionLimits {
    max_speed: 50.0,         // mm/s
    max_acceleration: 200.0, // mm/s^2
    max_deceleration: 200.0, // mm/s^2
    // 4000 mm/s^3 against 200 mm/s^2 gives a 50 ms filter window — about
    // 12 control cycles at 250 Hz, and 50 ms added to every move.
    max_jerk: Some(4000.0), // mm/s^3
};
const STATUS_PRINT_PERIOD: Duration = Duration::from_millis(250);

/// Below this speed, an axis counts as already at rest for `stop`'s
/// "nothing to do" short-circuit.
const AT_REST_EPS: f64 = 1e-6;

/// One axis's own dynamic capability and travel, in **that axis's own
/// units** — mm for a linear axis, radians for a rotary joint. Every
/// single-axis operation reads these: a raw `move axisN`/`stop axisN`'s
/// default limits, and the per-joint `StopRamp` that `cascade_group_stop`
/// builds for each member of an interrupted group.
///
/// The cascade is why these are separate from the group limits. A group move's
/// limits are *Cartesian* (mm/s² of TCP), but the cascade applies a
/// deceleration in **joint** space. With identity kinematics the two coincide;
/// with a rotary joint, feeding it a mm/s² number would be a unit error.
///
/// This is one flat struct rather than an enum over axis kinds: a linear axis
/// and a rotary joint differ in numbers, a display label, and whether travel
/// is bounded, not in the operations performed on them.
///
/// These are the **startup** values, compiled in. `setlimits` edits the
/// control loop's own copy (`RuntimeLimits`), so nothing here is mutated and a
/// change lasts until exit.
struct AxisConfig {
    /// This axis's own dynamic capability, in its own units.
    limits: MotionLimits,
    /// Display only — `motion-core` never sees units. Used by `status` and
    /// the heartbeat.
    units: &'static str,
    /// Soft travel limits, as `(min, max)` inclusive. `None` means unbounded.
    /// Checked when a move target is commanded; see `check_target_in_limits`.
    position_limits: Option<(f64, f64)>,
}

/// Per-axis configuration, indexed by axis number — `main()` asserts the
/// length matches `NUM_AXES`.
const AXIS_CONFIGS: &[AxisConfig] = &[
    AxisConfig {
        limits: MotionLimits {
            max_speed: 50.0,
            max_acceleration: 200.0,
            max_deceleration: 200.0,
            max_jerk: Some(4000.0), // 50 ms filter window
        },
        units: "mm",
        position_limits: Some((-500.0, 500.0)),
    },
    AxisConfig {
        limits: MotionLimits {
            max_speed: 50.0,
            max_acceleration: 200.0,
            max_deceleration: 200.0,
            max_jerk: Some(4000.0),
        },
        units: "mm",
        position_limits: Some((-500.0, 500.0)),
    },
    // axis2/axis3: the SCARA arm's shoulder and elbow (see
    // `SCARA_KINEMATICS`). Rotary, and in **radians** — `ScaraKinematics`
    // does trigonometry on these values directly. So `move axis3 0.5` means
    // 0.5 rad ≈ 28.6°.
    //
    // `position_limits: None`: there are no joint travel limits. The group
    // move's install-time IK check covers workspace reachability, but not a
    // raw single-axis jog.
    AxisConfig {
        limits: MotionLimits {
            max_speed: 2.0,
            max_acceleration: 8.0,
            max_deceleration: 8.0,
            // 160 rad/s^3 against 8 rad/s^2 is the same 50 ms filter window
            // the linear axes get.
            max_jerk: Some(160.0), // rad/s^3
        },
        units: "rad",
        position_limits: None,
    },
    AxisConfig {
        limits: MotionLimits {
            max_speed: 2.0,
            max_acceleration: 8.0,
            max_deceleration: 8.0,
            max_jerk: Some(160.0),
        },
        units: "rad",
        position_limits: None,
    },
];

/// One group's Cartesian sample converted into joint space for this cycle:
/// `(position, velocity, acceleration)`. Computed once per group and read per
/// member, since the conversion is a whole-vector operation.
type GroupJointSample = (
    motion_core::KinematicVector,
    motion_core::KinematicVector,
    motion_core::KinematicVector,
);

/// The control loop's live copy of every axis's and group's limits, seeded
/// from the compile-time tables and mutable by `setlimits`.
///
/// It is not shared with the parser: a command carries what the user typed
/// (`Option<f64>` per limit) and this fills the gaps at handling time, so a
/// `setlimits` affects the very next command rather than racing it. Resolving
/// in the parser, on the stdin thread, would pin every command to the
/// compile-time values.
struct RuntimeLimits {
    axes: Vec<MotionLimits>,
    groups: Vec<MotionLimits>,
}

impl RuntimeLimits {
    fn new() -> Self {
        Self {
            axes: AXIS_CONFIGS.iter().map(|c| c.limits).collect(),
            groups: AXIS_GROUPS.iter().map(|g| g.limits).collect(),
        }
    }

    /// Fill in whatever the command didn't specify.
    ///
    /// `dmax` falls back to the configured deceleration, not to whatever
    /// `amax` resolved to, matching `stop` and the group cascade.
    /// `max_jerk` has no command-line form, so it always comes from
    /// configuration.
    fn resolve(
        limits: &MotionLimits,
        given: (Option<f64>, Option<f64>, Option<f64>),
    ) -> MotionLimits {
        MotionLimits {
            max_speed: given.0.unwrap_or(limits.max_speed),
            max_acceleration: given.1.unwrap_or(limits.max_acceleration),
            max_deceleration: given.2.unwrap_or(limits.max_deceleration),
            max_jerk: limits.max_jerk,
        }
    }
}

/// Rejects a single-axis move target outside the axis's soft travel limits.
///
/// This checks a *commanded endpoint* only. It says nothing about the
/// interior of a path, or about where a group move's Cartesian target lands in
/// joint space. An unlimited axis (`position_limits: None`) always passes.
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

/// A hard-coded axis group: a named set of axes driven by the same
/// `enable`/`disable`/`reset`/`stop`/`move` commands as a single axis, fanned
/// out to (or coordinated across) its members. Groups are a fixed
/// compile-time table, not created or removed at runtime.
struct AxisGroupDef {
    name: &'static str,
    axes: &'static [usize],
    /// How this group's *task-space* (Cartesian) coordinates map to its
    /// members' joint positions. Every group has one, including groups of
    /// linear stages, which use `IdentityKinematics`, so the move builders,
    /// the control loop and status/viz have one code path.
    ///
    /// `&'static dyn` because the table is heterogeneous (different models per
    /// group).
    kinematics: &'static dyn motion_core::KinematicModel,
    /// The group's *task-space* (TCP) dynamic limits, and the defaults a
    /// `move`/`movepath` on this group uses when the command doesn't override
    /// them.
    limits: MotionLimits,
}

/// The pass-through model for 2-axis Cartesian groups.
static IDENTITY_KINEMATICS_2: motion_core::IdentityKinematics =
    motion_core::IdentityKinematics::new(2);

/// The 2-link planar arm driven by `axisGroup1`. The links are equal, which
/// collapses the inner unreachable hole to a single point **at the origin**.
/// The fold-back singularity is therefore reachable: a straight move from
/// `(x, y)` to `(-x, -y)` crosses it with both endpoints validating cleanly.
/// It is handled only reactively (the per-cycle `NearSingular` → cascade
/// stop); avoiding the origin is left to the operator.
static SCARA_KINEMATICS: motion_core::ScaraKinematics =
    motion_core::ScaraKinematics::new(100.0, 100.0);

/// `axisGroup0` = `axis0` (X) + `axis1` (Y), a Cartesian pair.
/// `axisGroup1` = `axis2` (shoulder) + `axis3` (elbow), a SCARA arm — same
/// commands, same Cartesian move semantics, different model. `main()` asserts
/// every referenced axis index is `< NUM_AXES` and that each group's arity
/// matches its kinematic model's DOF.
const AXIS_GROUPS: &[AxisGroupDef] = &[
    AxisGroupDef {
        name: "axisGroup0",
        axes: &[0, 1],
        kinematics: &IDENTITY_KINEMATICS_2,
        limits: DEFAULT_CARTESIAN_LIMITS,
    },
    AxisGroupDef {
        name: "axisGroup1",
        axes: &[2, 3],
        kinematics: &SCARA_KINEMATICS,
        limits: DEFAULT_CARTESIAN_LIMITS,
    },
];

/// Dispatch policy for a move when the target is already busy. `Aborting` and
/// `Buffered` apply to every move target; `Blend` is `movepath`-only and
/// transitions the path's own *geometry* onto a new one. It is not PLCopen's
/// blending between independent move segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BufferMode {
    /// Takes effect immediately regardless of idle/busy: clears anything
    /// queued and replaces whatever's active, built from the axis's commanded
    /// position and velocity via `TrapezoidalProfile::new_with_start_velocity`.
    Aborting,
    /// Queues behind the current move if the axis is busy (FIFO; every queued
    /// move runs, in order), and starts immediately if idle. The default.
    Buffered,
    /// `movepath` only: like `Aborting` (takes effect immediately), but built
    /// via `motion_core::PathProfile::new_blended`. The new path's start
    /// tangent leans toward the group's incoming velocity direction instead of
    /// assuming a straight approach to its first waypoint, which is smoother
    /// than `Aborting`'s hard redirect. Velocity is not exactly continuous; see
    /// `PathProfile::new_blended`.
    Blend,
}

#[derive(Debug, PartialEq)]
enum Command {
    Move {
        axis: usize,
        target: f64,
        max_speed: Option<f64>,
        max_acceleration: Option<f64>,
        max_deceleration: Option<f64>,
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
        max_speed: Option<f64>,
        max_acceleration: Option<f64>,
        max_deceleration: Option<f64>,
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
    /// `waypoints` are the points *after* the group's current position: the
    /// current commanded position (and velocity, via whichever of
    /// `PathProfile::new_with_start_velocity`/`new_blended` `buffer_mode`
    /// picks) is prepended as the path's start (see
    /// `install_path_move_impl`). `Buffered` queues FIFO behind a busy group,
    /// as `MoveGroup` does. `Aborting` and `Blend` both redirect immediately
    /// and differ in how the new path's geometry is built: `Aborting` via
    /// `install_path_move` (a straight approach to the first waypoint),
    /// `Blend` via `install_path_move_blended` (the start tangent leans toward
    /// the incoming velocity direction).
    MovePath {
        group: usize,
        waypoints: Vec<Vec<f64>>,
        max_speed: Option<f64>,
        max_acceleration: Option<f64>,
        max_deceleration: Option<f64>,
        buffer_mode: BufferMode,
    },
    /// Change some of an axis's or a group's dynamic limits at runtime, and
    /// report the result. An empty `update` reports without changing.
    ///
    /// Takes effect on the *next* move built, not on one already running: a
    /// profile is constructed once at install and is a pure function of time
    /// thereafter. Stop and re-command to apply a change immediately.
    SetLimits {
        axis: usize,
        update: LimitsUpdate,
    },
    SetGroupLimits {
        group: usize,
        update: LimitsUpdate,
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

    // Headless: no window, so the control loop runs on the main thread and the
    // process ends when it does. The loop, including its `RecordingAxisGroup`
    // tap, is identical to the windowed run.
    if headless {
        run_control_loop(rx, history);
        return;
    }

    // Set once the control loop exits (via `quit` or stdin EOF), which closes
    // the viz window.
    let shutdown = Arc::new(AtomicBool::new(false));

    {
        let history = Arc::clone(&history);
        let shutdown = Arc::clone(&shutdown);
        thread::spawn(move || {
            run_control_loop(rx, history);
            shutdown.store(true, Ordering::Relaxed);
        });
    }

    // eframe/winit need the GUI event loop on the main thread, so it runs here
    // while the control loop and stdin reader each run on their own thread.
    // This call blocks until the viz window closes.
    let native_options = eframe::NativeOptions {
        renderer: eframe::Renderer::Glow,
        // eframe's default inner size is too short to fit the XY plots and
        // status boxes row plus the per-axis plots row. The content also
        // scrolls (see viz.rs), so this is only a starting size.
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
/// Unknown arguments are an error, not ignored: a scripted session that typos
/// `--headles` should fail rather than open a window nobody is watching.
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
        // `setLimits` is accepted as an alias.
        ["setlimits" | "setLimits", rest @ ..] => parse_set_limits(rest, line),
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

/// Parses everything after `"move" <target>`: the target's coordinates (1 for
/// a single axis, `AXIS_GROUPS[g].axes.len()` for a group), then up to 3
/// numeric limit args and an optional trailing `aborting`/`buffered` keyword.
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

    // Defaults are not applied here; see `parse_kinematic_limits`.
    let (buffer_mode, (max_speed, max_acceleration, max_deceleration)) =
        parse_buffer_mode_and_limits(rest, line)?;

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

/// Parses everything after `"movepath" <target>`: either the inline form (a
/// waypoint count, then that many waypoints' worth of coordinates; see
/// `parse_move_path_inline`) or, if the first token is the literal `"file"`,
/// waypoints read from a file, one per line (see `parse_move_path_file`). Both
/// produce the same `Command::MovePath`. A single axis can't be a `movepath`
/// target; chain `move ... buffered` instead.
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
/// waypoints' worth of coordinates (`coord_count` each), then up to 3 numeric
/// limit args and an optional trailing `aborting`/`buffered`/`blend` keyword.
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

    let (buffer_mode, (max_speed, max_acceleration, max_deceleration)) =
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
/// `read_waypoints_file`), followed by the same limit args and trailing
/// keyword as the inline form. The file is read here, on the stdin thread, not
/// in the control loop.
fn parse_move_path_file(
    group: usize,
    coord_count: usize,
    path: &str,
    rest: &[&str],
    line: &str,
) -> Result<Option<Command>, String> {
    let waypoints = read_waypoints_file(path, coord_count)?;
    let (buffer_mode, (max_speed, max_acceleration, max_deceleration)) =
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

/// Reads `path` and delegates to `parse_waypoint_lines`, which is separate so
/// it can be tested without the filesystem.
fn read_waypoints_file(path: &str, coord_count: usize) -> Result<Vec<Vec<f64>>, String> {
    let contents =
        std::fs::read_to_string(path).map_err(|e| format!("failed to read {path:?}: {e}"))?;
    parse_waypoint_lines(&contents, coord_count).map_err(|e| format!("{path}: {e}"))
}

/// One waypoint per line, `coord_count` whitespace-separated numbers each.
/// Blank lines are skipped. There is no comment syntax.
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

/// Splits off a trailing optional `aborting`/`buffered` keyword (always last,
/// however many numeric args precede it) and parses the up-to-3 numeric limit
/// args before it. Shared by `move` for axes and groups. `blend` is not
/// recognized: it is meaningless for a straight-line move (see
/// `parse_movepath_buffer_mode_and_limits`).
fn parse_buffer_mode_and_limits(
    rest: &[&str],
    line: &str,
) -> Result<(BufferMode, ParsedLimits), String> {
    let (buffer_mode, numeric_rest) = match rest.last() {
        Some(&"aborting") => (BufferMode::Aborting, &rest[..rest.len() - 1]),
        Some(&"buffered") => (BufferMode::Buffered, &rest[..rest.len() - 1]),
        _ => (BufferMode::Buffered, rest),
    };
    let (max_speed, max_acceleration, max_deceleration) =
        parse_kinematic_limits(numeric_rest, line)?;
    Ok((buffer_mode, (max_speed, max_acceleration, max_deceleration)))
}

/// `movepath`'s version of `parse_buffer_mode_and_limits`; it also recognizes
/// the trailing `blend` keyword (`BufferMode::Blend`).
fn parse_movepath_buffer_mode_and_limits(
    rest: &[&str],
    line: &str,
) -> Result<(BufferMode, ParsedLimits), String> {
    let (buffer_mode, numeric_rest) = match rest.last() {
        Some(&"aborting") => (BufferMode::Aborting, &rest[..rest.len() - 1]),
        Some(&"buffered") => (BufferMode::Buffered, &rest[..rest.len() - 1]),
        Some(&"blend") => (BufferMode::Blend, &rest[..rest.len() - 1]),
        _ => (BufferMode::Buffered, rest),
    };
    let (max_speed, max_acceleration, max_deceleration) =
        parse_kinematic_limits(numeric_rest, line)?;
    Ok((buffer_mode, (max_speed, max_acceleration, max_deceleration)))
}

/// `(vmax, amax, dmax)` exactly as typed — `None` for each one omitted.
/// Jerk has no command-line form; only `setlimits` sets it.
type ParsedLimits = (Option<f64>, Option<f64>, Option<f64>);

/// The trailing up-to-3 numeric limit args shared by `move` and both
/// `movepath` forms: `[vmax] [amax] [dmax]`, each `None` when omitted.
///
/// It does not apply defaults. The control loop fills the gaps from its live
/// limits table (see `RuntimeLimits::resolve`).
fn parse_kinematic_limits(rest: &[&str], line: &str) -> Result<ParsedLimits, String> {
    if rest.len() > 3 {
        return Err(format!("too many arguments: {line:?}"));
    }
    let mut numeric_rest = rest.iter();
    let mut next =
        || -> Result<Option<f64>, String> { numeric_rest.next().map(|v| parse_f64(v)).transpose() };
    Ok((next()?, next()?, next()?))
}

/// A **partial** change to a target's limits: only the named ones move.
///
/// Limits are named rather than positional (`setlimits axis0 accel 100 jerk
/// 500`, not `setlimits axis0 ,,100,500`): miscounting a comma form puts a
/// valid number into the wrong limit without any error.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct LimitsUpdate {
    max_speed: Option<f64>,
    max_acceleration: Option<f64>,
    max_deceleration: Option<f64>,
    /// Two levels: the **outer** `None` means "not named, leave it alone",
    /// while an inner `None` means "named as `none`, i.e. no jerk limit".
    max_jerk: Option<Option<f64>>,
}

impl LimitsUpdate {
    /// Whether this names nothing — `setlimits <target>`, which reports the
    /// current limits instead of changing any.
    fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    fn apply(&self, limits: &mut MotionLimits) {
        if let Some(v) = self.max_speed {
            limits.max_speed = v;
        }
        if let Some(a) = self.max_acceleration {
            limits.max_acceleration = a;
        }
        if let Some(d) = self.max_deceleration {
            limits.max_deceleration = d;
        }
        if let Some(j) = self.max_jerk {
            limits.max_jerk = j;
        }
    }
}

/// `setlimits <target> [<name> <value>]...` — for example
/// `setlimits axis0 accel 100 jerk 500`. With no pairs at all it reports
/// the target's current limits and changes nothing.
///
/// Each limit is accepted under two names: `speed`/`accel`/`decel`/`jerk`, and
/// `vmax`/`amax`/`dmax`/`jmax`, which `move`'s positional arguments use in
/// `help`.
///
/// `jerk none` is the `max_jerk → ∞` case: no filtering, exactly the
/// trapezoidal profile.
///
/// Values are validated here, where they are typed, rather than at the next
/// move.
fn parse_set_limits(rest: &[&str], line: &str) -> Result<Option<Command>, String> {
    let Some((target, pairs)) = rest.split_first() else {
        return Err(format!(
            "expected: setlimits <target> [<name> <value>]..., got {line:?}"
        ));
    };
    let target = parse_target(target)?;

    let positive = |name: &str, text: &str| -> Result<f64, String> {
        let v = parse_f64(text)?;
        if !v.is_finite() || v <= 0.0 {
            return Err(format!("{name} must be a positive number (got {v})"));
        }
        Ok(v)
    };

    let mut update = LimitsUpdate::default();
    let mut pairs = pairs.iter();
    while let Some(name) = pairs.next() {
        let value = pairs
            .next()
            .ok_or_else(|| format!("{name:?} needs a value: {line:?}"))?;
        match *name {
            "speed" | "vmax" => update.max_speed = Some(positive("speed", value)?),
            "accel" | "amax" => update.max_acceleration = Some(positive("accel", value)?),
            "decel" | "dmax" => update.max_deceleration = Some(positive("decel", value)?),
            "jerk" | "jmax" => {
                update.max_jerk = Some(match *value {
                    "none" => None,
                    v => Some(positive("jerk", v)?),
                })
            }
            other => {
                return Err(format!(
                    "unknown limit {other:?} (expected speed, accel, decel or jerk): {line:?}"
                ));
            }
        }
    }

    Ok(Some(match target {
        Target::Axis(axis) => Command::SetLimits { axis, update },
        Target::Group(group) => Command::SetGroupLimits { group, update },
    }))
}

/// What a command verb (`enable`/`disable`/`reset`/`stop`/`move`) is aimed
/// at: a single axis, or a hard-coded group of them.
enum Target {
    Axis(usize),
    /// Index into `AXIS_GROUPS`.
    Group(usize),
}

/// Resolves a target token as a group name first (an exact match against
/// `AXIS_GROUPS`), falling back to `parse_axis`. Group names and axis names
/// can't collide.
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

/// Whether an axis is enabled and fault-free: the gate every business-logic
/// decision in this loop checks (`move`/`stop` rejection, queue promotion,
/// `enable`'s "already enabled"). `app` reasons about `AxisState` only, never
/// `Ds402State` (see `AxisRuntime::ds402_state`). `AxisState::Disabled` covers
/// every DS402 substate short of `OperationEnabled`.
fn axis_operational(state: AxisState) -> bool {
    !matches!(state, AxisState::Disabled | AxisState::ErrorStop)
}

/// One line per command with a short description.
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
    cmd_line(
        "setlimits <target> [speed|accel|decel|jerk <value>]...",
        "change some limits (jerk takes `none`); applies to the next move",
    );
    cmd_line(
        "setlimits <target>",
        "report the target's current limits, changing nothing",
    );
    cmd_line("enable <target>", "power up and enable");
    cmd_line("disable <target>", "disable (faults it if actively moving)");
    cmd_line("reset <target>", "clear a fault");
    cmd_line("verbose", "toggle the per-cycle position/phase heartbeat");
    cmd_line("status", "print position/phase for every axis and group");
    cmd_line("help", "show this message");
    cmd_line("quit", "exit");
    println!();
    println!("  a group move's coordinates are Cartesian (mm); a single-axis");
    println!("  move is raw joint space in that axis's own units:");
    for (axis, config) in AXIS_CONFIGS.iter().enumerate() {
        println!("    {}: {}", axis_label(axis), config.units);
    }
    println!();
}

/// State shared by every member of an active group move: the underlying
/// straight-line profile (see `motion_core::LinearMove`) and which axes
/// participate, in the order the profile's per-axis samples come out
/// (`AXIS_GROUPS[group].axes`).
///
/// It does not carry a deceleration for `cascade_group_stop`: the cascade's
/// ramps are in joint space, so each sibling decelerates at its own
/// `AxisConfig` deceleration.
///
/// The profile is entirely **Cartesian**. Kinematics converts into it once at
/// install (start state) and out of it once per cycle (setpoints).
struct SharedGroupMove {
    profile: LinearMove,
    axes: &'static [usize],
    group: usize,
    /// Which IK solution branch this move runs on, resolved once from the
    /// group's *commanded* joint pose at install and held for the move's whole
    /// duration.
    ///
    /// Per-move rather than per-cycle: reselecting the nearest solution each
    /// cycle would make IK's output depend on its own previous output, so a
    /// sample at a given time would depend on history. Held here, IK is a pure
    /// function of Cartesian position for the whole move. The branch persists
    /// across moves because commanded joints came out of IK on this branch, so
    /// resolving from them next time returns it again. It is not fixed per
    /// model, because a raw single-axis jog can put the arm on the other
    /// branch.
    branch: motion_core::KinematicBranch,
    /// The move's endpoint in **joint** space: IK of the Cartesian target on
    /// `branch`, computed at install as the reachability check. `status`
    /// reports it as each member's own target.
    joint_target: motion_core::KinematicVector,
}

/// The path-move analog of `SharedGroupMove`, wrapping
/// `motion_core::PathProfile` instead of `LinearMove`. The two profile types
/// share no trait; see `ActiveGroupMove` for the one place that treats them
/// uniformly.
struct SharedPathMove {
    profile: PathProfile,
    axes: &'static [usize],
    group: usize,
    /// See `SharedGroupMove::branch`.
    branch: motion_core::KinematicBranch,
    /// IK of the *final* waypoint. The intermediate waypoints are IK'd at
    /// install as a reachability check, but only the endpoint is kept.
    joint_target: motion_core::KinematicVector,
}

/// Whichever motion profile is currently driving an axis: a commanded move, a
/// commanded stop, or this axis's slice of an active group or path move. All
/// have the same shape (`sample`/`phase_at`/`target`) but are different types:
/// `TrapezoidalProfile` ends at rest, `StopRamp` has no target, and
/// `Group`/`Path` index into a profile shared across every participating
/// axis. Code that needs to distinguish a stop from a move uses `is_stop`.
enum Profile {
    Move(JerkFilteredProfile),
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
                    acceleration: s.acceleration()[*index],
                }
            }
            Profile::Path { shared, index } => {
                let s = shared.profile.sample(t);
                motion_core::TrajectorySample {
                    position: s.position()[*index],
                    velocity: s.velocity()[*index],
                    acceleration: s.acceleration()[*index],
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

    /// This axis's own endpoint, always in **joint** space. For a group/path
    /// member that is the stored IK of the Cartesian target, not the matching
    /// component of it; the two coincide only under identity kinematics.
    fn target(&self) -> f64 {
        match self {
            Profile::Move(p) => p.target(),
            Profile::Stop(p) => p.target(),
            Profile::Group { shared, index } => shared.joint_target.as_slice()[*index],
            Profile::Path { shared, index } => shared.joint_target.as_slice()[*index],
        }
    }

    /// `(group index, this axis's position within the group)` for a group or
    /// path move: the key into the control loop's per-group cache of
    /// IK-converted joint setpoints. `None` for a single-axis profile, which
    /// is already in joint space.
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

    /// The group name this profile belongs to, if it's a `Group` or `Path`.
    /// Used by `status`.
    fn group_membership(&self) -> Option<&'static str> {
        match self {
            Profile::Group { shared, .. } => Some(group_label(shared.group)),
            Profile::Path { shared, .. } => Some(group_label(shared.group)),
            _ => None,
        }
    }
}

/// Clears any queued moves and replaces whatever's active on `ax` with a
/// fresh profile built from its current *commanded* position/velocity. Used by
/// `stop` (builds a `StopRamp`) and an `aborting` move (builds a
/// `TrapezoidalProfile` via `new_with_start_velocity`). `build` receives
/// (position, velocity), prints its own success message, and returns the
/// `Profile` to install or a `TrajectoryError`, which is reported here.
///
/// The commanded state is used so the replacement picks up the commanded
/// stream where the superseded profile left it. See
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

/// A `Profile::Group` or `Profile::Path`'s shared state, so
/// `cascade_group_stop` can treat both uniformly. `LinearMove` and
/// `PathProfile` share no trait, so this is an enum over the two `Rc`s. A new
/// kind of group move extends this enum.
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

    /// This move's Cartesian `(position, velocity, acceleration)` at `t`, plus
    /// the IK branch it was installed on: what the control loop needs to
    /// convert one shared task-space sample into joint setpoints.
    ///
    /// The vector construction cannot fail: both profiles sample within
    /// `MAX_GROUP_AXES` and produce finite values from the finite inputs
    /// validated at install.
    fn sample(
        &self,
        t: f64,
    ) -> (
        motion_core::KinematicVector,
        motion_core::KinematicVector,
        motion_core::KinematicVector,
        motion_core::KinematicBranch,
    ) {
        let (position, velocity, acceleration, branch) = match self {
            ActiveGroupMove::Group(s) => {
                let sample = s.profile.sample(t);
                (
                    motion_core::KinematicVector::from_slice(sample.position()),
                    motion_core::KinematicVector::from_slice(sample.velocity()),
                    motion_core::KinematicVector::from_slice(sample.acceleration()),
                    s.branch,
                )
            }
            ActiveGroupMove::Path(s) => {
                let sample = s.profile.sample(t);
                (
                    motion_core::KinematicVector::from_slice(sample.position()),
                    motion_core::KinematicVector::from_slice(sample.velocity()),
                    motion_core::KinematicVector::from_slice(sample.acceleration()),
                    s.branch,
                )
            }
        };
        (
            position.expect("profile sample is always a valid kinematic vector"),
            velocity.expect("profile sample is always a valid kinematic vector"),
            acceleration.expect("profile sample is always a valid kinematic vector"),
            branch,
        )
    }

    /// Whether `profile` is still driven by *this specific* shared move
    /// instance (`Rc::ptr_eq` within the matching variant). A variant mismatch
    /// is `false`.
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

/// If `profile` is a `Profile::Group` or `Profile::Path`, returns its shared
/// state (cloning the `Rc`, not the move). Used to detect group membership when
/// a single member is about to be replaced or cleared, so every other member
/// can be cascaded into a stop.
fn group_of(profile: &Profile) -> Option<ActiveGroupMove> {
    match profile {
        Profile::Group { shared, .. } => Some(ActiveGroupMove::Group(Rc::clone(shared))),
        Profile::Path { shared, .. } => Some(ActiveGroupMove::Path(Rc::clone(shared))),
        _ => None,
    }
}

/// Cascades a group (or path) interruption: every member of `shared` other
/// than `except` gets its own `StopRamp` from its own commanded
/// position/velocity, via the `abort_into` mechanism a single-axis `stop`
/// uses. Guarded by `ActiveGroupMove::still_drives` (`Rc::ptr_eq`), so a member
/// that already moved on to something else is left alone. That keeps a double
/// interruption in one cycle, or asymmetric disable/fault timing between
/// members, from double-firing or clobbering unrelated state.
///
/// `except` is the axis that caused the interruption and already has its own
/// new profile; `None` stops *every* member, as a runtime IK failure does.
/// `reason` is the parenthetical in each member's stop message.
fn cascade_group_stop(
    axes: &mut [AxisRuntime],
    group_pending: &mut [VecDeque<PendingGroupMove>],
    shared: &ActiveGroupMove,
    except: Option<usize>,
    reason: &str,
) {
    // Losing control clears anything queued, as in `abort_into`: the group's
    // queued follow-up moves are dropped too.
    group_pending[shared.group()].clear();
    for &other in shared.axes() {
        // Each sibling decelerates at its own limit, not the group's: the
        // group's deceleration is a Cartesian TCP quantity and this ramp is in
        // joint space (see `AxisConfig`).
        let decel = AXIS_CONFIGS[other].limits.max_deceleration;
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

/// Gathers one value per group member into a `KinematicVector` for the
/// group's kinematic model. Fails only on a non-finite value or an over-long
/// group, both checked by `KinematicVector::from_slice`.
fn commanded_vector(
    axes: &[AxisRuntime],
    members: &[usize],
    field: impl Fn(&AxisRuntime) -> f64,
) -> Result<motion_core::KinematicVector, motion_core::KinematicsError> {
    let values: Vec<f64> = members.iter().map(|&a| field(&axes[a])).collect();
    motion_core::KinematicVector::from_slice(&values)
}

/// Why a group or path move couldn't be installed: the profile construction
/// rejected the geometry/limits, or the kinematic model rejected the target
/// (unreachable, or a group/model arity mismatch). Call sites only print it.
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
/// current *commanded* position/velocity (see `AxisRuntime::commanded_position`),
/// mapped through the group's kinematic model into Cartesian task space. The
/// profile is Cartesian throughout; the control loop converts back to joint
/// setpoints each cycle. Only on success, including the target's reachability
/// (a dry-run `inverse_position`), it installs the move atomically as
/// `Profile::Group` on every member, also clearing each member's own pending
/// queue. On failure nothing is touched, so a rejected group move never leaves
/// one member half-redirected. It is installed as a whole rather than through
/// `abort_into` because it is one fallible construction across all members.
///
/// Returns the profile's duration for the caller's success message (the
/// wording differs between an immediate move and one promoted from the queue).
/// It does **not** touch `group_pending`: the promotion loop is already
/// draining that queue, and clearing it here would discard what is queued
/// behind the move just promoted.
fn install_group_move(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    targets: Vec<f64>,
    limits: &MotionLimits,
) -> Result<f64, GroupMoveError> {
    let model = AXIS_GROUPS[group].kinematics;
    let joint_start = commanded_vector(axes, members, |ax| ax.commanded_position)?;
    let joint_velocity = commanded_vector(axes, members, |ax| ax.commanded_velocity)?;

    // Branch first, from the commanded joint pose; the target's reachability
    // check below uses it.
    let branch = model.resolve_branch(joint_start);
    let start = model.forward_position(joint_start);
    let start_velocity = model.forward_velocity(joint_start, joint_velocity);

    // Dry-run IK on the target before building anything, so an unreachable
    // endpoint is rejected here rather than surfacing mid-move as a cascade
    // stop.
    let joint_target =
        model.inverse_position(motion_core::KinematicVector::from_slice(&targets)?, branch)?;

    // `limits` is already resolved (see `RuntimeLimits::resolve`) and is
    // task-space.
    let profile = LinearMove::new_with_start_velocity(
        start.as_slice().to_vec(),
        start_velocity.as_slice().to_vec(),
        targets,
        limits.max_speed,
        limits.max_acceleration,
        limits.max_deceleration,
        limits.max_jerk,
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

/// The path-move analog of `install_group_move`: builds a `PathProfile` for
/// `group`'s `members`, prepending their current *commanded* position as the
/// path's start waypoint (`waypoints` is everything after it; see
/// `Command::MovePath`), and only on success installs it atomically as
/// `Profile::Path` on every member. On failure nothing is touched.
///
/// `build` is the `PathProfile` constructor to use. It is always given each
/// member's commanded velocity; an idle group's velocity is 0, which reduces
/// to rest-to-rest behavior. `install_path_move` and
/// `install_path_move_blended` differ only in which constructor they pass.
fn install_path_move_impl(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    waypoints: Vec<Vec<f64>>,
    limits: &MotionLimits,
    build: impl FnOnce(
        Vec<Vec<f64>>,
        Vec<f64>,
        f64,
        f64,
        f64,
        Option<f64>,
    ) -> Result<PathProfile, PathProfileError>,
) -> Result<f64, GroupMoveError> {
    let model = AXIS_GROUPS[group].kinematics;
    let joint_start = commanded_vector(axes, members, |ax| ax.commanded_position)?;
    let joint_velocity = commanded_vector(axes, members, |ax| ax.commanded_velocity)?;

    let branch = model.resolve_branch(joint_start);
    let start = model.forward_position(joint_start);
    let start_velocity = model.forward_velocity(joint_start, joint_velocity);

    // Every given waypoint is IK'd up front, not just the final target, so an
    // unreachable path is rejected before it starts rather than cascading to a
    // stop partway through. All use the one resolved branch, so a path that
    // would need the elbow to flip can't run. The interior of a segment is not
    // checked.
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
        limits.max_speed,
        limits.max_acceleration,
        limits.max_deceleration,
        limits.max_jerk,
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

/// `BufferMode::Aborting` (and idle/`Buffered`) path installation, built via
/// `PathProfile::new_with_start_velocity`.
fn install_path_move(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    waypoints: Vec<Vec<f64>>,
    limits: &MotionLimits,
) -> Result<f64, GroupMoveError> {
    install_path_move_impl(
        axes,
        group,
        members,
        waypoints,
        limits,
        PathProfile::new_with_start_velocity,
    )
}

/// `BufferMode::Blend` path installation, built via `PathProfile::new_blended`.
fn install_path_move_blended(
    axes: &mut [AxisRuntime],
    group: usize,
    members: &'static [usize],
    waypoints: Vec<Vec<f64>>,
    limits: &MotionLimits,
) -> Result<f64, GroupMoveError> {
    install_path_move_impl(
        axes,
        group,
        members,
        waypoints,
        limits,
        PathProfile::new_blended,
    )
}

/// Per-axis command handlers, shared by the single-axis commands and by the
/// group commands, which fan the same logic out to each member.
fn handle_enable(ax: &mut AxisRuntime, axis: usize) {
    if ax.axis_state == AxisState::ErrorStop {
        // Refuse outright: an `enable` that arrives during the fault window
        // must not stick and fire when a later `reset` clears the fault.
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
        // Nothing queued survives a stop (see abort_into), and there is
        // nothing to decelerate, so skip building a zero-duration StopRamp.
        ax.pending.clear();
        println!("  -> {}: already at rest", axis_label(axis));
    } else {
        let decel = max_deceleration.unwrap_or(AXIS_CONFIGS[axis].limits.max_deceleration);
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
/// began. Profiles only know elapsed time (see `NOTE (dt seam)` in
/// motion-core); the loop anchors them to wall-clock time.
struct ActiveMove {
    profile: Profile,
    started_at: Instant,
}

struct PendingMove {
    target: f64,
    /// Resolved when the command was accepted, not when it is promoted, so a
    /// later `setlimits` cannot rewrite a queued move.
    limits: MotionLimits,
}

/// The group-level analog of `PendingMove`: one entry in a group's own FIFO
/// queue (`run_control_loop`'s `group_pending`). It is not part of
/// `AxisRuntime` because promoting it needs every member idle at once. The
/// queue can mix `move`s and `movepath`s in the order issued.
enum PendingGroupMove {
    Move {
        targets: Vec<f64>,
        limits: MotionLimits,
    },
    Path {
        waypoints: Vec<Vec<f64>>,
        limits: MotionLimits,
    },
}

/// All runtime state for one axis. Axes are independent: each has its own
/// position, its own in-progress move (if any), and its own pending queue.
struct AxisRuntime {
    // Last-known *actual* position/velocity, from backend feedback, updated
    // once per control cycle after the backend exchange. Used for reporting
    // (`status`, the heartbeat, move-completion messages) and for the
    // enable-time resync below, never to seed a profile. See
    // `commanded_position`.
    position: f64,
    velocity: f64,
    /// Measured acceleration — reporting only, and an estimate (see
    /// `axis_backend::AxisFeedback::acceleration`).
    acceleration: f64,
    // The *commanded* (model) position/velocity: exactly what was sent as this
    // axis's `AxisSetpoint` last cycle. Every profile is seeded from this: a
    // new move, an aborting redirect, a `StopRamp`, a queue promotion, a
    // group/path move's start state.
    //
    // Seeding from feedback would inject a step into the commanded stream: if
    // the previous move ended commanding 100.000 while the axis sits at 99.980
    // (following error), a new move seeded from 99.980 steps the commanded
    // position back 0.020 mm in one cycle. It would also make the same command
    // sequence produce different trajectories depending on measured error, and
    // hide following error from the drive's own detection.
    //
    // If an axis physically can't keep up, the model runs away from reality
    // and following error grows until the drive faults.
    //
    // Commanded state is resynced from feedback at one place: the transition
    // from non-operational to operational (see the feedback fold in the
    // control loop). Disable and fault both leave the axis
    // `Disabled`/`ErrorStop`, and `axis_operational` gates every
    // move/stop/promotion, so nothing can be commanded again without crossing
    // that transition.
    commanded_position: f64,
    commanded_velocity: f64,
    active: Option<ActiveMove>,
    // FIFO queue of buffered moves waiting for the current one (and each
    // other) to finish. An `aborting` move bypasses it; see `BufferMode` and
    // `abort_into`.
    pending: VecDeque<PendingMove>,
    last_status_print: Instant,
    last_phase: Option<MotionPhase>,
    // What the user last asked for via `enable`/`disable`, sent to the backend
    // every cycle as `AxisSetpoint::enabled`.
    want_enabled: bool,
    // This axis's coarser status, updated from feedback every cycle. Every
    // business-logic gate in this loop (`move`/`stop`/queue-promotion
    // rejection, `enable`/`disable`'s "already ..." checks, fault recovery)
    // checks this, not `ds402_state`, so `app` depends only on the `AxisState`
    // the backend reports.
    axis_state: AxisState,
    // The backend-confirmed DS402 state, updated from feedback each cycle.
    // Used only to detect and print transitions (the "axisN: State -> State"
    // lines, and `status`'s trailing `[Ds402State]`). No decision reads it.
    ds402_state: Ds402State,
    // A one-shot pulse: set when `reset` is issued, sent as
    // `AxisSetpoint::fault_reset` for exactly one cycle, then cleared, like
    // DS402's edge-triggered "Fault Reset" controlword bit.
    pending_fault_reset: bool,
}

impl AxisRuntime {
    fn new() -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
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
    // One FIFO queue per group (see `PendingGroupMove`).
    let mut group_pending: Vec<VecDeque<PendingGroupMove>> =
        (0..AXIS_GROUPS.len()).map(|_| VecDeque::new()).collect();
    // Live limits, seeded from the compile-time tables and editable via
    // `setlimits`.
    let mut runtime_limits = RuntimeLimits::new();
    // Recording is a passive tap at the AxisGroup seam (see recording.rs).
    let mut backend =
        RecordingAxisGroup::new(SimAxisGroup::new(NUM_AXES, dt.as_secs_f64()), history);

    // Fixed schedule anchored to a single start instant, so ticks don't drift
    // from accumulated sleep overhead.
    let schedule_start = Instant::now();
    let mut cycle: u64 = 0;

    // Gates only the periodic position/phase heartbeat (see step 4 below).
    // Every other message always prints.
    let mut verbose = false;

    loop {
        // 1. If idle and moves are queued, start the next one now. A rejected
        //    candidate (bad kinematic params) is dropped and the next queued
        //    item is tried in the same cycle. An axis that's no longer enabled
        //    drops the whole queue at once with one message.
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
                match JerkFilteredProfile::new(
                    ax.commanded_position,
                    p.target,
                    p.limits.max_speed,
                    p.limits.max_acceleration,
                    p.limits.max_deceleration,
                    p.limits.max_jerk,
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

        // 1b. The same promotion for groups: if every member of a group is
        //     idle and the group has a queued move, start it via
        //     install_group_move or install_path_move, which are atomic across
        //     the whole group.
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
                let result = match p {
                    PendingGroupMove::Move { targets, limits } => {
                        install_group_move(&mut axes, g, members, targets, &limits)
                            .map_err(|e| e.to_string())
                    }
                    PendingGroupMove::Path { waypoints, limits } => {
                        install_path_move(&mut axes, g, members, waypoints, &limits)
                            .map_err(|e| e.to_string())
                    }
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
        //     *Cartesian* profile and convert it into joint position,
        //     velocity and acceleration for its members. This happens ahead of
        //     the per-axis setpoint pass because the conversion is per group (an
        //     arm's joint 1 setpoint depends on the whole Cartesian point), and
        //     so every member is sampled at the same instant.
        //
        //     Under `IdentityKinematics` this is a pass-through.
        //
        //     The branch comes from the move, resolved once at install. See
        //     `SharedGroupMove::branch`.
        let mut group_joint: Vec<Option<GroupJointSample>> = vec![None; AXIS_GROUPS.len()];
        // A failed conversion can't cascade from inside this pass (the cascade
        // needs `&mut` across the whole slice while this loop reads it), so
        // failures are collected here and applied in 1d.
        let mut ik_failures: Vec<(ActiveGroupMove, String)> = Vec::new();
        for (g, group) in AXIS_GROUPS.iter().enumerate() {
            // Any member driving this group's move is a valid representative:
            // they share one profile and one start instant, and only one
            // shared move per group can be live at a time.
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
            let (cartesian_position, cartesian_velocity, cartesian_acceleration, branch) =
                shared.sample(elapsed);

            // Position, then velocity, then acceleration; each needs the one
            // before it. Joint acceleration needs the joint velocity (the
            // `J̇·q̇` term).
            let model = group.kinematics;
            let converted =
                model
                    .inverse_position(cartesian_position, branch)
                    .and_then(|joint_position| {
                        let joint_velocity =
                            model.inverse_velocity(joint_position, cartesian_velocity)?;
                        let joint_acceleration = model.inverse_acceleration(
                            joint_position,
                            joint_velocity,
                            cartesian_acceleration,
                        )?;
                        Ok((joint_position, joint_velocity, joint_acceleration))
                    });
            match converted {
                Ok(pair) => group_joint[g] = Some(pair),
                // Holding position next to a singularity doesn't recover:
                // zero velocity there stays there. Every member ramps down
                // independently in joint space instead, which has no
                // singularities.
                Err(e) => ik_failures.push((shared, format!("kinematics failed: {e}"))),
            }
        }

        // 1d. Apply any kinematic failure from 1c *before* this cycle's
        //     setpoints are built. Each member's `StopRamp` starts from its
        //     `commanded_velocity`, which the setpoint pass below overwrites.
        //     Cascading after it would build every ramp from the zero velocity
        //     the failing cycle just wrote, an instantaneous stop. Cascading
        //     first installs the ramps with `started_at` = now, and the
        //     setpoint pass samples them at t ~= 0, the velocity the group was
        //     already commanding.
        //
        //     `except: None` because no axis is to blame, so every member
        //     ramps down.
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

        // 2. Build this cycle's commanded setpoint for every axis: sample the
        //    active trajectory if there is one, otherwise hold at the axis's
        //    last *commanded* position with zero velocity. Holding at the
        //    commanded position keeps the commanded stream continuous between
        //    moves. Each setpoint is written back to
        //    `commanded_position`/`commanded_velocity`, the one place the
        //    commanded state advances; see `AxisRuntime::commanded_position`.
        let setpoints: Vec<AxisSetpoint> = axes
            .iter_mut()
            .map(|ax| {
                // Consume the one-shot reset pulse: it is sent in this cycle's
                // setpoint and must not repeat.
                let fault_reset = std::mem::take(&mut ax.pending_fault_reset);
                let setpoint = match &ax.active {
                    // A group/path member takes its component from the joint
                    // vector computed once for the whole group in step 1c. The
                    // shared profile's own sample is task-space, and only
                    // under identity kinematics is its component `index` also
                    // this axis's setpoint.
                    Some(mv) => match mv.profile.group_and_index() {
                        Some((g, index)) => match &group_joint[g] {
                            Some((position, velocity, acceleration)) => AxisSetpoint {
                                position: position.as_slice()[index],
                                velocity: velocity.as_slice()[index],
                                acceleration: acceleration.as_slice()[index],
                                enabled: ax.want_enabled,
                                fault_reset,
                                stopping: mv.profile.is_stop(),
                            },
                            // Kinematics failed for this group and the cascade
                            // in 1d couldn't build this axis a stop ramp, so
                            // it still points at the dead group move. Hold
                            // at the commanded position with zero velocity,
                            // as an idle axis does.
                            None => AxisSetpoint {
                                position: ax.commanded_position,
                                velocity: 0.0,
                                acceleration: 0.0,
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
                                acceleration: sample.acceleration,
                                enabled: ax.want_enabled,
                                fault_reset,
                                stopping: mv.profile.is_stop(),
                            }
                        }
                    },
                    None => AxisSetpoint {
                        position: ax.commanded_position,
                        velocity: 0.0,
                        acceleration: 0.0,
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

        // 3. One combined cyclic exchange with the backend (see axis-backend).
        //    setpoints.len() always equals NUM_AXES, so a count mismatch would
        //    be a bug in this loop, hence `expect`.
        let feedback = backend
            .exchange(&setpoints)
            .expect("setpoints always match backend's axis count");

        // 4. Fold feedback into each axis's state: update actual
        //    position/velocity, report any backend fault, and detect move
        //    completion and phase changes. Completion is judged from the
        //    profile's phase; the feedback position is only reported.
        // Collected whenever a group member goes non-operational, and applied
        // after the loop: `cascade_group_stop` needs `&mut` access to other
        // elements by index while `axes.iter_mut()` holds one.
        let mut group_cascades: Vec<(ActiveGroupMove, usize)> = Vec::new();

        for (i, ax) in axes.iter_mut().enumerate() {
            let was_operational = axis_operational(ax.axis_state);
            ax.position = feedback[i].position;
            ax.velocity = feedback[i].velocity;
            ax.acceleration = feedback[i].acceleration;
            ax.axis_state = feedback[i].state;

            // The one resync point: coming back from non-operational
            // (`Disabled`/`ErrorStop`) to operational. While the power stage
            // was off the axis could have moved for reasons the model doesn't
            // know about, so the commanded state is stale and the measured
            // position is the only truth. See `AxisRuntime::commanded_position`.
            if !was_operational && axis_operational(ax.axis_state) {
                ax.commanded_position = ax.position;
                ax.commanded_velocity = ax.velocity;
            }

            // A fault voids the last enable request: recovery is an explicit
            // reset, then a fresh enable. Checked every cycle, which is
            // harmless to repeat while the fault persists.
            if ax.axis_state == AxisState::ErrorStop {
                ax.want_enabled = false;
            }

            // Print DS402 transitions as the backend confirms them. This is
            // for traceability only (see `AxisRuntime::ds402_state`). At 250 Hz
            // a full enable/disable sequence finishes in ~12 ms, so the prints
            // arrive back-to-back.
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

            // A move can't continue on an axis that isn't fully enabled, e.g.
            // after a disable or a fault.
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
                            "     {}  t={elapsed:>6.3}s  pos={:>9.3} {units}  vel={:>8.3} {units}/s  \
                             acc={:>9.3} {units}/s^2  {}",
                            axis_label(i),
                            ax.position,
                            ax.velocity,
                            ax.acceleration,
                            phase_label(phase),
                            units = AXIS_CONFIGS[i].units
                        );
                        ax.last_status_print = now;
                    }
                    ax.last_phase = Some(phase);
                }
            }
        }

        // Apply the group cascades collected above.
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
                    // Whatever the command didn't specify comes from the live
                    // limits, so a `setlimits` applies from the next move on.
                    let limits = RuntimeLimits::resolve(
                        &runtime_limits.axes[axis],
                        (max_speed, max_acceleration, max_deceleration),
                    );
                    let ax = &mut axes[axis];
                    if !axis_operational(ax.axis_state) {
                        println!(
                            "  ! {}: move rejected: axis is disabled (enable it first)",
                            axis_label(axis)
                        );
                    } else if let Err(e) = check_target_in_limits(axis, target) {
                        // Checked when the command is accepted, so a `buffered`
                        // move is rejected immediately rather than at
                        // promotion.
                        println!("  ! {}: move rejected: {e}", axis_label(axis));
                    } else {
                        match buffer_mode {
                            // `move`'s parser never produces Blend (see
                            // parse_buffer_mode_and_limits); it is grouped
                            // with Aborting only to keep the match exhaustive.
                            BufferMode::Aborting | BufferMode::Blend => {
                                // If this axis was part of an active group
                                // move, redirecting it alone cascades the rest
                                // of the group into its own stop, as a direct
                                // `stop`/fault/disable would.
                                let previous_group =
                                    ax.active.as_ref().and_then(|mv| group_of(&mv.profile));
                                abort_into(ax, axis, "move", move |position, velocity| {
                                    let profile = JerkFilteredProfile::new_with_start_velocity(
                                        position,
                                        velocity,
                                        target,
                                        limits.max_speed,
                                        limits.max_acceleration,
                                        limits.max_deceleration,
                                        limits.max_jerk,
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
                                match JerkFilteredProfile::new(
                                    ax.commanded_position,
                                    target,
                                    limits.max_speed,
                                    limits.max_acceleration,
                                    limits.max_deceleration,
                                    limits.max_jerk,
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
                                ax.pending.push_back(PendingMove { target, limits });
                            }
                        }
                    }
                }
                Command::Stop {
                    axis,
                    max_deceleration,
                } => {
                    // As with an Aborting move above, stopping a lone member
                    // takes the rest of its group down with it.
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
                    // A stop cancels anything queued, as a single-axis stop does.
                    group_pending[group].clear();
                    // Each member gets its own StopRamp from its own commanded
                    // position/velocity; a stop needs no shared shape.
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
                    // Group limits, not the members' own: a group move is a
                    // TCP-space command. See `MotionLimits`.
                    let limits = RuntimeLimits::resolve(
                        &runtime_limits.groups[group],
                        (max_speed, max_acceleration, max_deceleration),
                    );
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
                        group_pending[group].push_back(PendingGroupMove::Move { targets, limits });
                    } else {
                        // Either idle, or Aborting redirecting a busy group.
                        // Seizing control clears anything this group had
                        // queued, as `abort_into` does for a single axis.
                        group_pending[group].clear();
                        match install_group_move(&mut axes, group, members, targets, &limits) {
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
                    let limits = RuntimeLimits::resolve(
                        &runtime_limits.groups[group],
                        (max_speed, max_acceleration, max_deceleration),
                    );
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
                        group_pending[group]
                            .push_back(PendingGroupMove::Path { waypoints, limits });
                    } else {
                        // Either idle, or Aborting/Blend redirecting a busy
                        // group. Seizing control clears anything this group
                        // had queued, as in MoveGroup. Aborting and Blend
                        // differ only in the install function.
                        group_pending[group].clear();
                        let result = if buffer_mode == BufferMode::Blend {
                            install_path_move_blended(&mut axes, group, members, waypoints, &limits)
                        } else {
                            install_path_move(&mut axes, group, members, waypoints, &limits)
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
                Command::SetLimits { axis, update } => {
                    update.apply(&mut runtime_limits.axes[axis]);
                    print_limits(
                        &axis_label(axis),
                        &runtime_limits.axes[axis],
                        AXIS_CONFIGS[axis].units,
                        !update.is_empty(),
                    );
                }
                Command::SetGroupLimits { group, update } => {
                    // Group limits are Cartesian, so the units are mm, not the
                    // members'.
                    update.apply(&mut runtime_limits.groups[group]);
                    print_limits(
                        group_label(group),
                        &runtime_limits.groups[group],
                        "mm",
                        !update.is_empty(),
                    );
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
                // Position/velocity are from the last backend feedback (at
                // most one cycle stale), as in the heartbeat print.
                let group_note = match mv.profile.group_membership() {
                    Some(name) => format!("  (group {name})"),
                    None => String::new(),
                };
                // `target()` is this axis's own joint-space endpoint (see
                // `Profile::target`).
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
        // A group is "active" when one of its members is running *this*
        // group's shared move (a group move or a path move). Any member works
        // as the representative, since they share one profile. Both profile
        // types expose `phase_at`/`target`, so this extracts a common
        // (phase, target) pair.
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
                // A group's position is its *TCP* position: the members'
                // measured joint positions run through forward kinematics.
                // The per-axis lines above report the raw joint values.
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
                    // Reachable only if feedback went non-finite (a backend
                    // bug). Report it rather than print a wrong number.
                    Err(e) => println!("  status: {}: position unavailable: {e}", group.name),
                }
            }
        }
    }
}

/// Report a target's limits in its own units, after a `setlimits` changed some
/// (`changed`), or as a plain query when it named none. Always prints all four
/// limits, not just the changed ones.
fn print_limits(label: &str, limits: &MotionLimits, units: &str, changed: bool) {
    let jerk = match limits.max_jerk {
        Some(j) => format!("{j:.3} {units}/s^3"),
        None => "none (unfiltered)".to_string(),
    };
    println!(
        "  -> {label}: {}: speed {:.3} {units}/s, accel {:.3} {units}/s^2, \
         decel {:.3} {units}/s^2, jerk {jerk}",
        if changed { "limits set" } else { "limits" },
        limits.max_speed,
        limits.max_acceleration,
        limits.max_deceleration
    );
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

    /// Unspecified limits stay `None` through parsing; the control loop fills
    /// them from its live table.
    #[test]
    fn parse_command_move_leaves_unspecified_limits_open() {
        assert_eq!(
            parse_command("move axis0 100"),
            Ok(Some(Command::Move {
                axis: 0,
                target: 100.0,
                max_speed: None,
                max_acceleration: None,
                max_deceleration: None,
                buffer_mode: BufferMode::Buffered,
            }))
        );
    }

    #[test]
    fn setlimits_changes_only_what_it_names() {
        let cmd = parse_command("setlimits axis0 accel 100 jerk 500");
        let Ok(Some(Command::SetLimits { axis: 0, update })) = cmd else {
            panic!("expected a SetLimits for axis0, got {cmd:?}");
        };
        assert_eq!(update.max_acceleration, Some(100.0));
        assert_eq!(update.max_jerk, Some(Some(500.0)));
        // Untouched: speed and decel keep whatever the axis already had.
        assert_eq!(update.max_speed, None);
        assert_eq!(update.max_deceleration, None);

        let mut limits = MotionLimits {
            max_speed: 1.0,
            max_acceleration: 2.0,
            max_deceleration: 3.0,
            max_jerk: Some(4.0),
        };
        update.apply(&mut limits);
        assert_eq!(limits.max_speed, 1.0);
        assert_eq!(limits.max_acceleration, 100.0);
        assert_eq!(limits.max_deceleration, 3.0);
        assert_eq!(limits.max_jerk, Some(500.0));
    }

    /// "Don't touch jerk" and "remove the jerk limit" are different commands;
    /// the two-level Option keeps them apart.
    #[test]
    fn setlimits_distinguishes_unnamed_jerk_from_jerk_none() {
        let mut limits = MotionLimits {
            max_speed: 1.0,
            max_acceleration: 2.0,
            max_deceleration: 3.0,
            max_jerk: Some(4.0),
        };

        let Ok(Some(Command::SetLimits { update, .. })) = parse_command("setlimits axis0 speed 9")
        else {
            panic!("expected SetLimits");
        };
        update.apply(&mut limits);
        assert_eq!(limits.max_jerk, Some(4.0), "unnamed jerk is untouched");

        let Ok(Some(Command::SetLimits { update, .. })) =
            parse_command("setlimits axis0 jerk none")
        else {
            panic!("expected SetLimits");
        };
        update.apply(&mut limits);
        assert_eq!(limits.max_jerk, None, "`jerk none` removes the limit");
    }

    #[test]
    fn setlimits_with_no_pairs_is_a_query() {
        let Ok(Some(Command::SetLimits { axis: 0, update })) = parse_command("setlimits axis0")
        else {
            panic!("expected SetLimits");
        };
        assert!(update.is_empty());
    }

    #[test]
    fn setlimits_accepts_a_group_and_the_vmax_spellings() {
        let Ok(Some(Command::SetGroupLimits { group: 0, update })) =
            parse_command("setlimits axisGroup0 amax 7 jmax 8")
        else {
            panic!("expected SetGroupLimits");
        };
        assert_eq!(update.max_acceleration, Some(7.0));
        assert_eq!(update.max_jerk, Some(Some(8.0)));
    }

    #[test]
    fn setlimits_rejects_bad_names_values_and_dangling_pairs() {
        // Unknown limit name.
        assert!(parse_command("setlimits axis0 jrk 5").is_err());
        // Name with no value.
        assert!(parse_command("setlimits axis0 accel").is_err());
        // Non-positive and non-finite values.
        assert!(parse_command("setlimits axis0 accel 0").is_err());
        assert!(parse_command("setlimits axis0 speed -1").is_err());
        assert!(parse_command("setlimits axis0 jerk inf").is_err());
        // `none` is only meaningful for jerk.
        assert!(parse_command("setlimits axis0 speed none").is_err());
    }

    #[test]
    fn unspecified_limits_resolve_against_the_live_table() {
        let configured = MotionLimits {
            max_speed: 11.0,
            max_acceleration: 22.0,
            max_deceleration: 33.0,
            max_jerk: Some(44.0),
        };
        // Nothing given: every limit comes from configuration.
        let all_default = RuntimeLimits::resolve(&configured, (None, None, None));
        assert_eq!(all_default, configured);

        // Given values win, and `dmax` falls back to the *configured*
        // deceleration rather than to whatever `amax` resolved to.
        let partial = RuntimeLimits::resolve(&configured, (Some(1.0), Some(2.0), None));
        assert_eq!(partial.max_speed, 1.0);
        assert_eq!(partial.max_acceleration, 2.0);
        assert_eq!(partial.max_deceleration, 33.0);
        // Jerk has no command-line form, so it is always the configured one.
        assert_eq!(partial.max_jerk, Some(44.0));
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
                max_speed: Some(10.0),
                max_acceleration: Some(20.0),
                max_deceleration: Some(30.0),
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
                max_speed: Some(10.0),
                max_acceleration: Some(20.0),
                max_deceleration: Some(30.0),
                buffer_mode: BufferMode::Aborting,
            }))
        );
        // The keyword works even with no numeric args at all.
        assert_eq!(
            parse_command("move axis0 100 aborting"),
            Ok(Some(Command::Move {
                axis: 0,
                target: 100.0,
                max_speed: None,
                max_acceleration: None,
                max_deceleration: None,
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
                max_speed: None,
                max_acceleration: None,
                max_deceleration: None,
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
                max_speed: Some(10.0),
                max_acceleration: Some(20.0),
                max_deceleration: Some(30.0),
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
                max_speed: Some(10.0),
                max_acceleration: Some(20.0),
                max_deceleration: Some(30.0),
                buffer_mode: BufferMode::Aborting,
            }))
        );
        // Works with no numeric args at all too.
        assert_eq!(
            parse_command("movepath axisGroup0 1 5 5 aborting"),
            Ok(Some(Command::MovePath {
                group: 0,
                waypoints: vec![vec![5.0, 5.0]],
                max_speed: None,
                max_acceleration: None,
                max_deceleration: None,
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
                max_speed: Some(10.0),
                max_acceleration: Some(20.0),
                max_deceleration: Some(30.0),
                buffer_mode: BufferMode::Blend,
            }))
        );
    }

    #[test]
    fn parse_command_move_does_not_accept_blend_keyword() {
        // "blend" isn't a keyword for plain move; it is parsed as a numeric
        // limit arg and fails as "not a number".
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

    // --- parse_waypoint_lines (the file form's line parsing; no filesystem) ---

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
                max_speed: Some(30.0),
                max_acceleration: None,
                max_deceleration: None,
                buffer_mode: BufferMode::Buffered,
            }))
        );
    }

    #[test]
    fn parse_command_movepath_file_missing_file_is_an_error() {
        assert!(parse_command("movepath axisGroup0 file /no/such/path.txt").is_err());
    }
}
