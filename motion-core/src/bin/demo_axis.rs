//! Watch a single-axis trapezoidal move happen, as numbers, at our control-loop
//! rate. No GUI, no hardware — just the trajectory generator being sampled the
//! way the real control loop will sample it.
//!
//! Run from the workspace root with:
//!
//!     cargo run -p motion-core --bin demo_axis
//!
//! This mimics the cyclic control loop: every cycle we compute the elapsed time
//! and ask the profile "where should the axis be right now?". At 250 Hz that's
//! one query every 4 ms. We print a subset of cycles (and always the phase
//! boundaries) so the trapezoid is visible without flooding the terminal.

use motion_core::{MotionPhase, TrapezoidalProfile};

fn main() {
    // --- Move definition ------------------------------------------------
    // Move a linear axis from 0 mm to 100 mm.
    let start = 0.0;
    let end = 100.0;
    let max_velocity = 50.0; // mm/s
    let max_acceleration = 200.0; // mm/s^2

    let profile = TrapezoidalProfile::new(start, end, max_velocity, max_acceleration);

    // --- Control-loop cadence ------------------------------------------
    let rate_hz = 250.0;
    let dt = 1.0 / rate_hz; // 4 ms per cycle
    let duration = profile.duration();

    println!("Trapezoidal move: {start} -> {end} mm");
    println!("  max velocity     = {max_velocity} mm/s");
    println!("  max acceleration = {max_acceleration} mm/s^2");
    println!("  control rate     = {rate_hz} Hz  (dt = {:.1} ms)", dt * 1000.0);
    println!("  total duration   = {:.4} s  ({} cycles)", duration, (duration / dt).ceil() as u64);
    println!();
    println!("   cycle      t (s)    pos (mm)   vel (mm/s)   phase");
    println!("  ------  ---------  ----------  -----------  --------");

    // Run a few cycles past the end so you can see it settle at the target.
    let total_cycles = (duration / dt).ceil() as u64 + 5;

    // Print roughly every Nth cycle to keep output readable, but always print
    // the first cycle, the last, and cycles near a phase change.
    let print_every = ((total_cycles as f64) / 30.0).ceil().max(1.0) as u64;

    let mut prev_phase: Option<MotionPhase> = None;
    for cycle in 0..=total_cycles {
        let t = cycle as f64 * dt;
        let sample = profile.sample(t);
        let phase = profile.phase_at(t);

        // Always print when the phase changes, so every boundary is visible.
        let phase_changed = prev_phase != Some(phase);
        prev_phase = Some(phase);

        if cycle % print_every == 0 || cycle == total_cycles || phase_changed {
            println!(
                "  {:>6}  {:>9.4}  {:>10.4}  {:>11.4}   {}",
                cycle,
                t,
                sample.position,
                sample.velocity,
                phase_label(phase)
            );
        }
    }

    println!();
    println!("Notice:");
    println!("  - velocity ramps up linearly (accel), flattens (cruise), ramps down (decel)");
    println!("  - position is the smooth integral of that velocity");
    println!("  - after t = {:.4}s the axis holds exactly at {end} mm, at rest", duration);
}

/// Short human-readable label for a phase, for the demo output only.
fn phase_label(phase: MotionPhase) -> &'static str {
    match phase {
        MotionPhase::Pre => "pre",
        MotionPhase::Accel => "accel",
        MotionPhase::Cruise => "cruise",
        MotionPhase::Decel => "decel",
        MotionPhase::Done => "done",
    }
}
