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
//!     status
//!     help
//!     quit

use std::io::{self, BufRead};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant};

use axis_backend::{AxisGroup, AxisSetpoint};
use backend_sim::SimAxisGroup;
use motion_core::{MotionPhase, TrapezoidalProfile};

const CONTROL_RATE_HZ: f64 = 250.0;
const DEFAULT_MAX_SPEED: f64 = 50.0; // mm/s
const DEFAULT_MAX_ACCELERATION: f64 = 200.0; // mm/s^2
const STATUS_PRINT_PERIOD: Duration = Duration::from_millis(250);

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
    Status,
    Help,
    Quit,
}

fn main() {
    print_help();

    let (tx, rx) = mpsc::channel::<Command>();
    thread::spawn(move || read_commands(tx));

    run_control_loop(rx);
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
    println!("  status                 — print every axis's position/phase");
    println!("  help                   — show this message");
    println!("  quit                   — exit");
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
        }
    }
}

fn run_control_loop(rx: Receiver<Command>) {
    let dt = Duration::from_secs_f64(1.0 / CONTROL_RATE_HZ);
    let mut axes: Vec<AxisRuntime> = (0..NUM_AXES).map(|_| AxisRuntime::new()).collect();
    let mut backend = SimAxisGroup::new(NUM_AXES, dt.as_secs_f64());

    // Fixed schedule anchored to a single start instant, so ticks don't
    // drift from accumulated sleep-call overhead.
    let schedule_start = Instant::now();
    let mut cycle: u64 = 0;

    loop {
        // 1. If idle and a move is queued, start it now.
        for (i, ax) in axes.iter_mut().enumerate() {
            if ax.active.is_none() {
                if let Some(p) = ax.pending.take() {
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
            .iter()
            .map(|ax| match &ax.active {
                Some(mv) => {
                    let elapsed = mv.started_at.elapsed().as_secs_f64();
                    let sample = mv.profile.sample(elapsed);
                    AxisSetpoint {
                        position: sample.position,
                        velocity: sample.velocity,
                    }
                }
                None => AxisSetpoint {
                    position: ax.position,
                    velocity: 0.0,
                },
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

            if let Some(fault) = feedback[i].fault {
                println!("  ! {}: backend fault: {fault:?}", axis_label(i));
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
                    if ax.active.is_none() {
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
                    "  status: {}: pos={:.3} mm  vel={:.3} mm/s  {}  (target {:.3} mm)",
                    axis_label(i),
                    ax.position,
                    ax.velocity,
                    phase_label(mv.profile.phase_at(elapsed)),
                    mv.profile.target()
                );
            }
            None => println!("  status: {}: idle at {:.3} mm", axis_label(i), ax.position),
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
