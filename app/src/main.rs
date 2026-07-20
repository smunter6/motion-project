//! A continuously-running, single-axis motion app you can drive from the
//! terminal. This is the first cut of the "online" architecture: a
//! fixed-rate control loop running on its own schedule, decoupled from
//! (blocking) terminal input by a channel.
//!
//! There is no backend yet (Step 3 in CLAUDE.md's roadmap) — the loop
//! samples `motion-core`'s trajectory directly and treats the setpoint as
//! the actual position. Once a real/sim backend exists behind the
//! `AxisGroup` seam, this loop's "print the setpoint" step becomes "send
//! the setpoint to the backend, read actual position back".
//!
//! Run from the workspace root with:
//!
//!     cargo run -p app
//!
//! Commands (one per line on stdin):
//!     move <target_mm> [max_velocity] [max_acceleration]
//!     status
//!     help
//!     quit

use std::io::{self, BufRead};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant};

use motion_core::{MotionPhase, TrapezoidalProfile};

const CONTROL_RATE_HZ: f64 = 250.0;
const DEFAULT_MAX_VELOCITY: f64 = 50.0; // mm/s
const DEFAULT_MAX_ACCELERATION: f64 = 200.0; // mm/s^2
const STATUS_PRINT_PERIOD: Duration = Duration::from_millis(250);

enum Command {
    Move {
        target: f64,
        max_velocity: f64,
        max_acceleration: f64,
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
        ["move", target] => Ok(Some(Command::Move {
            target: parse_f64(target)?,
            max_velocity: DEFAULT_MAX_VELOCITY,
            max_acceleration: DEFAULT_MAX_ACCELERATION,
        })),
        ["move", target, vmax] => Ok(Some(Command::Move {
            target: parse_f64(target)?,
            max_velocity: parse_f64(vmax)?,
            max_acceleration: DEFAULT_MAX_ACCELERATION,
        })),
        ["move", target, vmax, amax] => Ok(Some(Command::Move {
            target: parse_f64(target)?,
            max_velocity: parse_f64(vmax)?,
            max_acceleration: parse_f64(amax)?,
        })),
        _ => Err(format!(
            "unrecognized command: {line:?} (type \"help\" for usage)"
        )),
    }
}

fn parse_f64(s: &str) -> Result<f64, String> {
    s.parse::<f64>()
        .map_err(|_| format!("expected a number, got {s:?}"))
}

fn print_help() {
    println!("motion-project online app — single axis, control rate {CONTROL_RATE_HZ} Hz");
    println!("commands:");
    println!("  move <target_mm> [max_velocity_mm_s] [max_acceleration_mm_s2]");
    println!("  status                 — print current position/phase");
    println!("  help                   — show this message");
    println!("  quit                   — exit");
    println!(
        "  (a move sent while another is in progress is queued and starts \
         the instant the current move finishes — only the most recent \
         queued move is kept)"
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
    max_velocity: f64,
    max_acceleration: f64,
}

fn run_control_loop(rx: Receiver<Command>) {
    let dt = Duration::from_secs_f64(1.0 / CONTROL_RATE_HZ);

    let mut position = 0.0_f64;
    let mut active: Option<ActiveMove> = None;
    // Single-slot queue: at most one move waits for the current one to
    // finish. This is the "block until idle" policy — deliberately simple
    // for now; see plcopen-motion-goal memory for why not to bake in
    // "always starts at rest" any harder than this in the API shape.
    let mut pending: Option<PendingMove> = None;
    let mut last_status_print = Instant::now();
    let mut last_phase: Option<MotionPhase> = None;

    // Fixed schedule anchored to a single start instant, so ticks don't
    // drift from accumulated sleep-call overhead.
    let schedule_start = Instant::now();
    let mut cycle: u64 = 0;

    loop {
        // 1. Finish/advance the in-progress move, if any.
        if let Some(mv) = &active {
            let elapsed = mv.started_at.elapsed().as_secs_f64();
            if elapsed >= mv.profile.duration() {
                position = mv.profile.target();
                println!("  -> reached {position:.3} mm");
                active = None;
                last_phase = None;
            }
        }

        // 2. If idle and a move is queued, start it now.
        if active.is_none() {
            if let Some(p) = pending.take() {
                let profile = TrapezoidalProfile::new(
                    position,
                    p.target,
                    p.max_velocity,
                    p.max_acceleration,
                );
                println!(
                    "  -> starting queued move: {position:.3} -> {:.3} mm ({:.3}s)",
                    p.target,
                    profile.duration()
                );
                active = Some(ActiveMove {
                    profile,
                    started_at: Instant::now(),
                });
            }
        }

        // 3. Drain any commands that arrived since the last tick.
        while let Ok(cmd) = rx.try_recv() {
            match cmd {
                Command::Move {
                    target,
                    max_velocity,
                    max_acceleration,
                } => {
                    if active.is_none() {
                        let profile = TrapezoidalProfile::new(
                            position,
                            target,
                            max_velocity,
                            max_acceleration,
                        );
                        println!(
                            "  -> move: {position:.3} -> {target:.3} mm ({:.3}s)",
                            profile.duration()
                        );
                        active = Some(ActiveMove {
                            profile,
                            started_at: Instant::now(),
                        });
                    } else {
                        println!("  -> busy: queuing move to {target:.3} mm");
                        pending = Some(PendingMove {
                            target,
                            max_velocity,
                            max_acceleration,
                        });
                    }
                }
                Command::Status => {
                    print_status(position, &active);
                }
                Command::Help => print_help(),
                Command::Quit => {
                    println!("exiting.");
                    return;
                }
            }
        }

        // 4. Sample the active move and print a sparing heartbeat — on
        //    phase changes always, otherwise at most every
        //    STATUS_PRINT_PERIOD (250 Hz is far too fast for a terminal).
        if let Some(mv) = &active {
            let elapsed = mv.started_at.elapsed().as_secs_f64();
            let sample = mv.profile.sample(elapsed);
            position = sample.position;
            let phase = mv.profile.phase_at(elapsed);

            let phase_changed = last_phase != Some(phase);
            let now = Instant::now();
            if phase_changed || now.duration_since(last_status_print) >= STATUS_PRINT_PERIOD {
                println!(
                    "     t={elapsed:>6.3}s  pos={:>9.3} mm  vel={:>8.3} mm/s  {}",
                    sample.position,
                    sample.velocity,
                    phase_label(phase)
                );
                last_status_print = now;
            }
            last_phase = Some(phase);
        }

        // 5. Sleep to the next scheduled tick.
        cycle += 1;
        let next_tick = schedule_start + dt.mul_f64(cycle as f64);
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        }
    }
}

fn print_status(position: f64, active: &Option<ActiveMove>) {
    match active {
        Some(mv) => {
            let elapsed = mv.started_at.elapsed().as_secs_f64();
            let sample = mv.profile.sample(elapsed);
            println!(
                "  status: pos={:.3} mm  vel={:.3} mm/s  {}  (target {:.3} mm)",
                sample.position,
                sample.velocity,
                phase_label(mv.profile.phase_at(elapsed)),
                mv.profile.target()
            );
        }
        None => println!("  status: idle at {position:.3} mm"),
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
