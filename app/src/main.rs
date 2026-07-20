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
//! time-scaling) is a distinct, harder feature — deliberately deferred; see
//! the roadmap's Step 2.
//!
//! Run from the workspace root with:
//!
//!     cargo run -p app
//!
//! Commands (one per line on stdin):
//!     move <axisN> <target_mm> [max_speed] [max_acceleration] [max_deceleration]
//!     enable <axisN>
//!     disable <axisN>
//!     reset <axisN>
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

mod recording;
mod viz;

use std::io::{self, BufRead};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use axis_backend::{AxisGroup, AxisSetpoint, Ds402State};
use backend_sim::SimAxisGroup;
use motion_core::{MotionPhase, TrapezoidalProfile};
use recording::{History, RecordingAxisGroup};
use viz::VizApp;

const CONTROL_RATE_HZ: f64 = 250.0;
const DEFAULT_MAX_SPEED: f64 = 50.0; // mm/s
const DEFAULT_MAX_ACCELERATION: f64 = 200.0; // mm/s^2
const STATUS_PRINT_PERIOD: Duration = Duration::from_millis(250);

/// How much target-vs-actual history the viz window keeps per axis, in
/// seconds. Older samples are dropped as new ones arrive.
const HISTORY_SECONDS: f64 = 60.0;

/// Number of independent axes the app manages, named `axis0`..`axis{N-1}`
/// on the command line. Bumping this is the only change needed to add more
/// axes — everything else is `Vec`-driven.
const NUM_AXES: usize = 2;

enum Command {
    Move {
        axis: usize,
        target: f64,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
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
    Status,
    Help,
    Quit,
}

fn main() {
    print_help();

    let (tx, rx) = mpsc::channel::<Command>();
    thread::spawn(move || read_commands(tx));

    let capacity = (CONTROL_RATE_HZ * HISTORY_SECONDS) as usize;
    let history = Arc::new(Mutex::new(History::new(NUM_AXES, capacity)));
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
        ..Default::default()
    };
    eframe::run_native(
        "motion-project viz",
        native_options,
        Box::new(move |_cc| Ok(Box::new(VizApp::new(history, shutdown, NUM_AXES)))),
    )
    .expect("failed to run viz window");
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
        ["status"] => Ok(Some(Command::Status)),
        ["help"] => Ok(Some(Command::Help)),
        ["enable", axis] => Ok(Some(Command::Enable {
            axis: parse_axis(axis)?,
        })),
        ["disable", axis] => Ok(Some(Command::Disable {
            axis: parse_axis(axis)?,
        })),
        ["reset", axis] => Ok(Some(Command::Reset {
            axis: parse_axis(axis)?,
        })),
        ["move", axis, target, rest @ ..] => {
            if rest.len() > 3 {
                return Err(format!("too many arguments: {line:?}"));
            }
            let mut rest = rest.iter();
            let max_speed = match rest.next() {
                Some(v) => parse_f64(v)?,
                None => DEFAULT_MAX_SPEED,
            };
            let max_acceleration = match rest.next() {
                Some(a) => parse_f64(a)?,
                None => DEFAULT_MAX_ACCELERATION,
            };
            // Deceleration defaults to whatever acceleration resolved to
            // (default or user-specified), so a symmetric move needs no
            // extra argument.
            let max_deceleration = match rest.next() {
                Some(d) => parse_f64(d)?,
                None => max_acceleration,
            };
            Ok(Some(Command::Move {
                axis: parse_axis(axis)?,
                target: parse_f64(target)?,
                max_speed,
                max_acceleration,
                max_deceleration,
            }))
        }
        _ => Err(format!(
            "unrecognized command: {line:?} (type \"help\" for usage)"
        )),
    }
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

fn print_help() {
    println!(
        "motion-project online app — {NUM_AXES} independent axes, control rate {CONTROL_RATE_HZ} Hz"
    );
    println!("commands:");
    println!(
        "  move <axisN> <target_mm> [max_speed_mm_s] [max_acceleration_mm_s2] \
         [max_deceleration_mm_s2]"
    );
    println!("      axisN is axis0..axis{}", NUM_AXES - 1);
    println!("      (deceleration defaults to the acceleration value if omitted)");
    println!("  enable <axisN>         — power up and enable an axis (required before move)");
    println!("  disable <axisN>        — disable an axis (aborts any active move)");
    println!("  reset <axisN>          — clear a fault (doesn't re-enable — enable after)");
    println!("  status                 — print every axis's position/phase");
    println!("  help                   — show this message");
    println!("  quit                   — exit");
    println!(
        "  (axes start disabled, matching real drive power-up — move on a \
         disabled axis is rejected, not queued)"
    );
    println!(
        "  (disabling a moving axis faults it, not a graceful power-down — \
         reset, then enable again, to recover)"
    );
    println!(
        "  (a move sent to an axis that's already moving is queued and starts \
         the instant that axis's current move finishes — only the most \
         recent queued move per axis is kept; axes are otherwise independent)"
    );
    println!();
}

/// An in-progress move: the profile plus the wall-clock instant it began.
/// `TrapezoidalProfile` itself only knows relative/elapsed time (see the
/// `NOTE (dt seam)` comment in motion-core) — the loop is what anchors it
/// to wall-clock time.
struct ActiveMove {
    profile: TrapezoidalProfile,
    started_at: Instant,
}

struct PendingMove {
    target: f64,
    max_speed: f64,
    max_acceleration: f64,
    max_deceleration: f64,
}

/// All runtime state for one axis. Axes are independent: each has its own
/// position, its own in-progress move (if any), and its own single-slot
/// pending queue — nothing here is shared across axes.
struct AxisRuntime {
    // Last-known *actual* position/velocity, from backend feedback — not
    // the raw trajectory sample. Updated once per control cycle after the
    // backend exchange; `status` reads these directly rather than
    // resampling, so it reports the same "actual" value the heartbeat
    // print does.
    position: f64,
    velocity: f64,
    active: Option<ActiveMove>,
    // Single-slot queue: at most one move waits for the current one to
    // finish. This is the "block until idle" policy — deliberately simple
    // for now; see plcopen-motion-goal memory for why not to bake in
    // "always starts at rest" any harder than this in the API shape.
    pending: Option<PendingMove>,
    last_status_print: Instant,
    last_phase: Option<MotionPhase>,
    // What the user last asked for via `enable`/`disable` — sent to the
    // backend every cycle as `AxisSetpoint::enabled` (see axis-backend's
    // docs on why that's a cyclic field, not a one-off command).
    want_enabled: bool,
    // The backend-confirmed DS402 state, updated from feedback each cycle.
    // `move` gates on this, not `want_enabled` — the two can disagree for a
    // few cycles while the backend is still stepping through the enable
    // sequence.
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
            active: None,
            pending: None,
            last_status_print: Instant::now(),
            last_phase: None,
            want_enabled: false,
            ds402_state: Ds402State::SwitchOnDisabled,
            pending_fault_reset: false,
        }
    }
}

fn run_control_loop(rx: Receiver<Command>, history: Arc<Mutex<History>>) {
    let dt = Duration::from_secs_f64(1.0 / CONTROL_RATE_HZ);
    let mut axes: Vec<AxisRuntime> = (0..NUM_AXES).map(|_| AxisRuntime::new()).collect();
    // Recording is a passive tap at the AxisGroup seam (see recording.rs) —
    // everything below this line is unchanged from before viz existed.
    let mut backend = RecordingAxisGroup::new(SimAxisGroup::new(NUM_AXES, dt.as_secs_f64()), history);

    // Fixed schedule anchored to a single start instant, so ticks don't
    // drift from accumulated sleep-call overhead.
    let schedule_start = Instant::now();
    let mut cycle: u64 = 0;

    loop {
        // 1. If idle and a move is queued, start it now.
        for (i, ax) in axes.iter_mut().enumerate() {
            if ax.active.is_none() {
                if let Some(p) = ax.pending.take() {
                    if ax.ds402_state != Ds402State::OperationEnabled {
                        println!(
                            "  ! {}: queued move dropped: axis is not enabled",
                            axis_label(i)
                        );
                        continue;
                    }
                    match TrapezoidalProfile::new(
                        ax.position,
                        p.target,
                        p.max_speed,
                        p.max_acceleration,
                        p.max_deceleration,
                    ) {
                        Ok(profile) => {
                            println!(
                                "  -> {}: starting queued move: {:.3} -> {:.3} mm ({:.3}s)",
                                axis_label(i),
                                ax.position,
                                p.target,
                                profile.duration()
                            );
                            ax.active = Some(ActiveMove {
                                profile,
                                started_at: Instant::now(),
                            });
                        }
                        Err(e) => {
                            println!("  ! {}: queued move rejected: {e}", axis_label(i));
                        }
                    }
                }
            }
        }

        // 2. Build this cycle's commanded setpoint for every axis: sample
        //    the active trajectory if there is one, otherwise hold at the
        //    axis's last known actual position with zero velocity.
        let setpoints: Vec<AxisSetpoint> = axes
            .iter_mut()
            .map(|ax| {
                // Consume the one-shot reset pulse right here — it's sent
                // in this cycle's setpoint and must not repeat next cycle.
                let fault_reset = std::mem::take(&mut ax.pending_fault_reset);
                match &ax.active {
                    Some(mv) => {
                        let elapsed = mv.started_at.elapsed().as_secs_f64();
                        let sample = mv.profile.sample(elapsed);
                        AxisSetpoint {
                            position: sample.position,
                            velocity: sample.velocity,
                            enabled: ax.want_enabled,
                            fault_reset,
                        }
                    }
                    None => AxisSetpoint {
                        position: ax.position,
                        velocity: 0.0,
                        enabled: ax.want_enabled,
                        fault_reset,
                    },
                }
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
        for (i, ax) in axes.iter_mut().enumerate() {
            ax.position = feedback[i].position;
            ax.velocity = feedback[i].velocity;

            // Surface DS402 transitions as they're confirmed by the
            // backend — at 250 Hz a full enable/disable sequence (3
            // transitions) finishes in ~12ms, so these prints arrive
            // essentially back-to-back, not spread over noticeable time.
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
                        // A fault means the last request is void — recovery
                        // is an explicit reset, then a fresh enable, not an
                        // automatic re-enable once the fault clears.
                        ax.want_enabled = false;
                    }
                    _ => {}
                }
            }

            // A move can't continue on an axis that isn't fully enabled —
            // e.g. the user just disabled it, or a fault just knocked it
            // out of OperationEnabled.
            if ax.ds402_state != Ds402State::OperationEnabled {
                if ax.active.is_some() {
                    if feedback[i].fault.is_some() {
                        println!("  ! {}: move faulted", axis_label(i));
                    } else {
                        println!(
                            "  ! {}: move aborted: axis is no longer enabled",
                            axis_label(i)
                        );
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
                    println!("  -> {}: reached {:.3} mm", axis_label(i), ax.position);
                    ax.active = None;
                    ax.last_phase = None;
                } else {
                    let phase_changed = ax.last_phase != Some(phase);
                    let now = Instant::now();
                    if phase_changed
                        || now.duration_since(ax.last_status_print) >= STATUS_PRINT_PERIOD
                    {
                        println!(
                            "     {}  t={elapsed:>6.3}s  pos={:>9.3} mm  vel={:>8.3} mm/s  {}",
                            axis_label(i),
                            ax.position,
                            ax.velocity,
                            phase_label(phase)
                        );
                        ax.last_status_print = now;
                    }
                    ax.last_phase = Some(phase);
                }
            }
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
                } => {
                    let ax = &mut axes[axis];
                    if ax.ds402_state != Ds402State::OperationEnabled {
                        println!(
                            "  ! {}: move rejected: axis is disabled (enable it first)",
                            axis_label(axis)
                        );
                    } else if ax.active.is_none() {
                        match TrapezoidalProfile::new(
                            ax.position,
                            target,
                            max_speed,
                            max_acceleration,
                            max_deceleration,
                        ) {
                            Ok(profile) => {
                                println!(
                                    "  -> {}: move: {:.3} -> {target:.3} mm ({:.3}s)",
                                    axis_label(axis),
                                    ax.position,
                                    profile.duration()
                                );
                                ax.active = Some(ActiveMove {
                                    profile,
                                    started_at: Instant::now(),
                                });
                            }
                            Err(e) => {
                                println!("  ! {}: move rejected: {e}", axis_label(axis));
                            }
                        }
                    } else {
                        println!(
                            "  -> {}: busy: queuing move to {target:.3} mm",
                            axis_label(axis)
                        );
                        ax.pending = Some(PendingMove {
                            target,
                            max_speed,
                            max_acceleration,
                            max_deceleration,
                        });
                    }
                }
                Command::Enable { axis } => {
                    let ax = &mut axes[axis];
                    if matches!(
                        ax.ds402_state,
                        Ds402State::Fault | Ds402State::FaultReactionActive
                    ) {
                        // Must refuse outright, not just leave `want_enabled`
                        // false and hope — an `enable` that arrives anywhere
                        // during the fault window must not "stick" and
                        // silently fire the instant a later `reset` clears
                        // the fault, without a fresh `enable` after it.
                        println!(
                            "  ! {}: cannot enable: axis has a fault — reset it first",
                            axis_label(axis)
                        );
                    } else if ax.want_enabled && ax.ds402_state == Ds402State::OperationEnabled {
                        println!("  -> {}: already enabled", axis_label(axis));
                    } else {
                        ax.want_enabled = true;
                        println!("  -> {}: enabling...", axis_label(axis));
                    }
                }
                Command::Disable { axis } => {
                    let ax = &mut axes[axis];
                    if !ax.want_enabled && ax.ds402_state == Ds402State::SwitchOnDisabled {
                        println!("  -> {}: already disabled", axis_label(axis));
                    } else {
                        ax.want_enabled = false;
                        println!("  -> {}: disabling...", axis_label(axis));
                    }
                }
                Command::Reset { axis } => {
                    let ax = &mut axes[axis];
                    if ax.ds402_state == Ds402State::Fault {
                        ax.pending_fault_reset = true;
                        println!("  -> {}: resetting fault...", axis_label(axis));
                    } else {
                        println!("  -> {}: no fault to reset", axis_label(axis));
                    }
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
                println!(
                    "  status: {}: pos={:.3} mm  vel={:.3} mm/s  {}  (target {:.3} mm)  [{:?}]",
                    axis_label(i),
                    ax.position,
                    ax.velocity,
                    phase_label(mv.profile.phase_at(elapsed)),
                    mv.profile.target(),
                    ax.ds402_state
                );
            }
            None => println!(
                "  status: {}: idle at {:.3} mm  [{:?}]",
                axis_label(i),
                ax.position,
                ax.ds402_state
            ),
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
