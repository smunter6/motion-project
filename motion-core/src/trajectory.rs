//! Trajectory generation: turning "go from A to B" into a stream of
//! instantaneous position setpoints.
//!
//! # Why this exists
//!
//! A servo drive running in CSP mode (Cyclic Synchronous Position) wants exactly
//! one thing from us every control cycle: the *target position for this instant*.
//! It does not want "go to B eventually" — it wants "be at 12.47 mm right now",
//! then 4 ms later "be at 12.83 mm right now", and so on. Our job is to produce
//! that stream of setpoints so the sequence adds up to smooth, physically
//! reasonable motion.
//!
//! # The model: trapezoidal velocity profile
//!
//! An axis can't teleport and can't change speed instantly (that would need
//! infinite acceleration / infinite force). So a well-behaved move has up to
//! three phases:
//!
//! 1. **Accelerate** at a constant rate until reaching a maximum velocity.
//! 2. **Cruise** at that maximum velocity.
//! 3. **Decelerate** at a constant rate, arriving exactly as velocity hits zero.
//!
//! Plotted as velocity-vs-time this is a trapezoid (ramp up, flat top, ramp
//! down). Position — the integral of velocity — comes out as a smooth S-ish
//! curve.
//!
//! # The triangular special case
//!
//! If the move is short, the axis may never reach `max_velocity`: it accelerates
//! and must *already* start decelerating to stop in time. The trapezoid loses its
//! flat top and becomes a triangle. We detect and handle this explicitly — the
//! boundary between the two cases is one of the genuinely instructive parts.
//!
//! # Time model: absolute-time query (not dt-stepping)
//!
//! This generator is a **pure function of elapsed time**: you ask "where should
//! the axis be at t = 0.348 s?" and it computes the answer from closed-form
//! kinematics. It holds no mutable state. This gives us:
//!   - no accumulated integration error (exact at every instant),
//!   - easy-to-verify invariants (test any instant independently),
//!   - robustness to loop-timing jitter (ask for the *actual* elapsed time).
//!
//! The *stateful*, dt-stepping model belongs to a different layer — the sim
//! plant model / real servo loop — which we build later. See `NOTE (dt seam)`
//! below for exactly where the two layers will meet.

/// Which phase of the trapezoidal profile a given instant falls in.
///
/// Exposed because it's genuinely useful (for display, diagnostics, and later
/// logic), and computing it from the profile's known phase-boundary times is
/// exact — far better than trying to infer it from velocity trends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionPhase {
    /// Before the move has started (t <= 0).
    Pre,
    /// Accelerating up toward cruise (or peak, if triangular).
    Accel,
    /// Cruising at constant velocity. A triangular move has no cruise phase.
    Cruise,
    /// Decelerating down to a stop at the target.
    Decel,
    /// The move is complete (t >= duration); holding at the target, at rest.
    Done,
}

/// A single-axis point-to-point move described by a trapezoidal (or, for short
/// moves, triangular) velocity profile.
///
/// Units are deliberately unspecified but must be *self-consistent*. Throughout
/// the project we treat linear axes as millimetres and seconds, so:
///   - positions in mm
///   - `max_velocity` in mm/s
///   - `max_acceleration` in mm/s^2
///
/// The core stays in `f64` for clean, readable math. Conversion to integer
/// encoder counts (what a real drive actually wants) is the job of the EtherCAT
/// backend at the hardware seam — not of this planner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidalProfile {
    start: f64,
    end: f64,
    /// Sign of travel: +1.0 if end >= start, else -1.0. Precomputed so the
    /// phase math can be written for a positive move and then re-signed.
    direction: f64,
    /// Total distance travelled (always >= 0; direction is carried separately).
    distance: f64,

    // Kinematic limits actually used for THIS move. `cruise_velocity` may be
    // below the requested max_velocity when the move is triangular.
    accel: f64,
    decel: f64,
    cruise_velocity: f64,

    // Phase timing (all relative to move start, t = 0):
    t_accel: f64, // end of acceleration phase == start of cruise
    t_cruise_end: f64, // end of cruise phase == start of deceleration
    t_total: f64, // move complete

    // Distance covered by the end of the acceleration phase (cached so the
    // cruise/decel position formulas don't recompute it every query).
    d_accel: f64,
}

/// A sample of the trajectory at one instant: the feed-forward reference the
/// control loop hands to the backend each cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrajectorySample {
    /// Target position at the queried time (same units as the profile).
    pub position: f64,
    /// Target velocity at the queried time. Cheap to compute and makes the
    /// profile's behaviour legible (you can watch it ramp up, flatten, ramp
    /// down). A real CSP drive only strictly needs position, but velocity
    /// feed-forward is useful later.
    pub velocity: f64,
}

impl TrapezoidalProfile {
    /// Build a profile for a move from `start` to `end`.
    ///
    /// `max_velocity`, `max_acceleration`, and `max_deceleration` must all be
    /// > 0. They represent the physical limits of the axis (ultimately set by
    /// motor torque / current limits on real hardware; chosen numbers in
    /// sim). Keeping them explicit means sim exercises the same limits real
    /// hardware will impose. Acceleration and deceleration are independent —
    /// many real axes (and PLCopen-style motion function blocks) allow a
    /// faster ramp-up than ramp-down, or vice versa.
    ///
    /// A zero-distance move is valid and produces a profile that simply reports
    /// `start` for all time with zero velocity (total duration 0).
    pub fn new(
        start: f64,
        end: f64,
        max_velocity: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Self {
        assert!(max_velocity > 0.0, "max_velocity must be positive");
        assert!(
            max_acceleration > 0.0,
            "max_acceleration must be positive"
        );
        assert!(
            max_deceleration > 0.0,
            "max_deceleration must be positive"
        );

        let delta = end - start;
        let distance = delta.abs();
        let direction = if delta >= 0.0 { 1.0 } else { -1.0 };

        // Degenerate move: already there. Everything is zero / start.
        if distance == 0.0 {
            return Self {
                start,
                end,
                direction,
                distance: 0.0,
                accel: max_acceleration,
                decel: max_deceleration,
                cruise_velocity: 0.0,
                t_accel: 0.0,
                t_cruise_end: 0.0,
                t_total: 0.0,
                d_accel: 0.0,
            };
        }

        // --- Decide trapezoid vs. triangle -------------------------------
        //
        // Distance needed to accelerate from 0 up to max_velocity, and
        // (independently, since accel and decel rates may differ) the
        // distance needed to decelerate from max_velocity back to 0:
        //
        //   v^2 = 2 * a * d   =>   d = v^2 / (2a)
        //
        // If accel distance + decel distance <= total distance, there's room to
        // reach cruise speed: it's a trapezoid. Otherwise we top out below
        // max_velocity: it's a triangle.
        let accel = max_acceleration;
        let decel = max_deceleration;
        let d_accel_full = (max_velocity * max_velocity) / (2.0 * accel);
        let d_decel_full = (max_velocity * max_velocity) / (2.0 * decel);
        let d_accel_plus_decel = d_accel_full + d_decel_full;

        let (cruise_velocity, t_accel, d_accel, t_cruise_end, t_total);

        if d_accel_plus_decel <= distance {
            // ---- Trapezoidal: we do reach max_velocity ----
            cruise_velocity = max_velocity;

            // Time to accelerate to cruise: v = a * t  =>  t = v / a
            t_accel = cruise_velocity / accel;
            d_accel = d_accel_full;
            let t_decel = cruise_velocity / decel;

            // Cruise covers whatever distance is left after accel + decel.
            let d_cruise = distance - d_accel_plus_decel;
            let t_cruise = d_cruise / cruise_velocity;

            t_cruise_end = t_accel + t_cruise;
            t_total = t_cruise_end + t_decel;
        } else {
            // ---- Triangular: peak velocity is below max_velocity ----
            //
            // Accelerate over d_accel, decelerate over d_decel, with
            // d_accel + d_decel == distance and a single peak velocity where
            // the two phases meet:
            //
            //   d_accel = v_peak^2 / (2*accel)
            //   d_decel = v_peak^2 / (2*decel)
            //   d_accel + d_decel = distance
            //     => v_peak = sqrt(2 * distance * accel * decel / (accel + decel))
            //
            // (reduces to the familiar sqrt(accel * distance) when
            // accel == decel.)
            cruise_velocity =
                (2.0 * distance * accel * decel / (accel + decel)).sqrt();

            t_accel = cruise_velocity / accel;
            d_accel = (cruise_velocity * cruise_velocity) / (2.0 * accel);
            let t_decel = cruise_velocity / decel;

            // No cruise phase: cruise start == cruise end.
            t_cruise_end = t_accel;
            t_total = t_accel + t_decel;
        }

        Self {
            start,
            end,
            direction,
            distance,
            accel,
            decel,
            cruise_velocity,
            t_accel,
            t_cruise_end,
            t_total,
            d_accel,
        }
    }

    /// Total duration of the move in seconds. Needed later to coordinate
    /// multiple axes (finish-together time-scaling).
    pub fn duration(&self) -> f64 {
        self.t_total
    }

    /// The commanded end position.
    pub fn target(&self) -> f64 {
        self.end
    }

    /// Which phase the move is in at absolute elapsed time `t` (seconds).
    ///
    /// Computed directly from the known phase-boundary times, so it's exact.
    /// For a triangular move the cruise window is empty (`t_accel ==
    /// t_cruise_end`), so `Cruise` is simply never returned.
    pub fn phase_at(&self, t: f64) -> MotionPhase {
        if t <= 0.0 || self.distance == 0.0 {
            MotionPhase::Pre
        } else if t >= self.t_total {
            MotionPhase::Done
        } else if t < self.t_accel {
            MotionPhase::Accel
        } else if t < self.t_cruise_end {
            MotionPhase::Cruise
        } else {
            MotionPhase::Decel
        }
    }

    /// Sample the trajectory at absolute elapsed time `t` (seconds since the
    /// move began).
    ///
    /// Times outside `[0, duration]` are clamped: `t < 0` returns the start
    /// (at rest), `t > duration` returns the end (at rest). This makes the
    /// "move already finished" case trivially correct — no completion flag to
    /// manage.
    ///
    /// NOTE (dt seam): the control loop will call this once per cycle with the
    /// *actual* elapsed time, then hand `sample.position` to the backend. The
    /// backend (sim plant model, later a real drive) is the stateful,
    /// dt-stepping layer that consumes this setpoint. This method stays pure.
    pub fn sample(&self, t: f64) -> TrajectorySample {
        // Before the move (or zero-distance move): sit at start, at rest.
        if t <= 0.0 || self.distance == 0.0 {
            return TrajectorySample {
                position: self.start,
                velocity: 0.0,
            };
        }
        // After the move: sit at end, at rest.
        if t >= self.t_total {
            return TrajectorySample {
                position: self.end,
                velocity: 0.0,
            };
        }

        // Compute position/velocity for a positive-direction move, then re-sign.
        let (pos_along, vel_along) = if t < self.t_accel {
            // --- Acceleration phase ---
            //   v(t) = a * t
            //   x(t) = 1/2 * a * t^2
            let v = self.accel * t;
            let x = 0.5 * self.accel * t * t;
            (x, v)
        } else if t < self.t_cruise_end {
            // --- Cruise phase (constant velocity) ---
            //   v(t) = cruise_velocity
            //   x(t) = d_accel + cruise_velocity * (t - t_accel)
            let v = self.cruise_velocity;
            let x = self.d_accel + self.cruise_velocity * (t - self.t_accel);
            (x, v)
        } else {
            // --- Deceleration phase ---
            // Measure time remaining until the move ends and integrate
            // backwards from a full stop at t_total, at the deceleration
            // rate (independent of the acceleration rate).
            //   td = t_total - t              (time left)
            //   v(t) = decel * td
            //   x(t) = distance - 1/2 * decel * td^2
            let td = self.t_total - t;
            let v = self.decel * td;
            let x = self.distance - 0.5 * self.decel * td * td;
            (x, v)
        };

        TrajectorySample {
            position: self.start + self.direction * pos_along,
            velocity: self.direction * vel_along,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Tolerance for floating-point comparisons.
    const EPS: f64 = 1e-9;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn starts_at_start_and_ends_at_end() {
        let p = TrapezoidalProfile::new(10.0, 110.0, 50.0, 100.0, 100.0);
        let at_start = p.sample(0.0);
        let at_end = p.sample(p.duration());
        assert!(approx(at_start.position, 10.0));
        assert!(approx(at_end.position, 110.0));
        // At rest at both ends.
        assert!(approx(at_start.velocity, 0.0));
        assert!(approx(at_end.velocity, 0.0));
    }

    #[test]
    fn clamps_before_and_after_move() {
        let p = TrapezoidalProfile::new(0.0, 100.0, 50.0, 100.0, 100.0);
        // Way before.
        let before = p.sample(-5.0);
        assert!(approx(before.position, 0.0));
        assert!(approx(before.velocity, 0.0));
        // Way after.
        let after = p.sample(p.duration() + 5.0);
        assert!(approx(after.position, 100.0));
        assert!(approx(after.velocity, 0.0));
    }

    #[test]
    fn never_exceeds_max_velocity() {
        let vmax = 50.0;
        let p = TrapezoidalProfile::new(0.0, 500.0, vmax, 100.0, 100.0);
        // Sample densely across the whole move.
        let n = 10_000;
        for i in 0..=n {
            let t = p.duration() * (i as f64) / (n as f64);
            let s = p.sample(t);
            assert!(
                s.velocity.abs() <= vmax + 1e-6,
                "velocity {} exceeded vmax {} at t={}",
                s.velocity,
                vmax,
                t
            );
        }
    }

    #[test]
    fn long_move_is_trapezoidal_and_reaches_cruise() {
        // 500 mm with vmax=50, amax=100.
        // accel distance = v^2/2a = 2500/200 = 12.5 mm; accel+decel = 25 mm.
        // 25 <= 500, so trapezoidal; cruise velocity should equal vmax.
        let p = TrapezoidalProfile::new(0.0, 500.0, 50.0, 100.0, 100.0);
        assert!(approx(p.cruise_velocity, 50.0));
        // Somewhere in the middle we should be cruising at exactly vmax.
        let mid = p.sample(p.duration() / 2.0);
        assert!(approx(mid.velocity, 50.0));
    }

    #[test]
    fn short_move_is_triangular_and_stays_below_max_velocity() {
        // 10 mm with vmax=50, amax=100.
        // accel+decel distance to reach vmax = 25 mm > 10 mm, so triangular.
        // peak velocity = sqrt(a*d) = sqrt(100*10) = sqrt(1000) ~= 31.62 mm/s.
        let p = TrapezoidalProfile::new(0.0, 10.0, 50.0, 100.0, 100.0);
        let expected_peak = (100.0_f64 * 10.0).sqrt();
        assert!(approx(p.cruise_velocity, expected_peak));
        assert!(p.cruise_velocity < 50.0);
        // No cruise phase: accel end == cruise end.
        assert!(approx(p.t_accel, p.t_cruise_end));
        // Peak occurs at the midpoint in time.
        let peak = p.sample(p.duration() / 2.0);
        assert!(approx(peak.velocity, expected_peak));
    }

    #[test]
    fn symmetric_move_is_symmetric_in_position() {
        // Position profile should be point-symmetric about the midpoint:
        // x(t) - start  ==  end - x(T - t).
        let p = TrapezoidalProfile::new(0.0, 200.0, 40.0, 80.0, 80.0);
        let dur = p.duration();
        let n = 1000;
        for i in 0..=n {
            let t = dur * (i as f64) / (n as f64);
            let a = p.sample(t).position - 0.0;
            let b = 200.0 - p.sample(dur - t).position;
            assert!(
                (a - b).abs() < 1e-6,
                "asymmetry at t={}: {} vs {}",
                t,
                a,
                b
            );
        }
    }

    #[test]
    fn negative_direction_move_works() {
        // Moving from 100 down to 0 should mirror the positive case.
        let p = TrapezoidalProfile::new(100.0, 0.0, 50.0, 100.0, 100.0);
        assert!(approx(p.sample(0.0).position, 100.0));
        assert!(approx(p.sample(p.duration()).position, 0.0));
        // Velocity should be negative during the move.
        let mid = p.sample(p.duration() / 2.0);
        assert!(mid.velocity < 0.0);
    }

    #[test]
    fn zero_distance_move_is_inert() {
        let p = TrapezoidalProfile::new(42.0, 42.0, 50.0, 100.0, 100.0);
        assert!(approx(p.duration(), 0.0));
        assert!(approx(p.sample(0.0).position, 42.0));
        assert!(approx(p.sample(1.0).position, 42.0));
        assert!(approx(p.sample(1.0).velocity, 0.0));
    }

    #[test]
    fn phase_at_reports_all_phases_for_trapezoid() {
        // distance=200, v_max=50, a_max=100 => t_accel=0.5s, t_total=4.5s, so
        // accel/decel each occupy ~11% of duration: comfortably wider than
        // the 5%-in / 5%-from-end sample points below.
        let p = TrapezoidalProfile::new(0.0, 200.0, 50.0, 100.0, 100.0);
        let dur = p.duration();
        assert_eq!(p.phase_at(-1.0), MotionPhase::Pre);
        assert_eq!(p.phase_at(0.0), MotionPhase::Pre);
        // Just inside the accel phase.
        assert_eq!(p.phase_at(dur * 0.05), MotionPhase::Accel);
        // Middle is cruise for a long move.
        assert_eq!(p.phase_at(dur * 0.5), MotionPhase::Cruise);
        // Near the end is decel.
        assert_eq!(p.phase_at(dur * 0.95), MotionPhase::Decel);
        assert_eq!(p.phase_at(dur), MotionPhase::Done);
        assert_eq!(p.phase_at(dur + 1.0), MotionPhase::Done);
    }

    #[test]
    fn phase_at_never_reports_cruise_for_triangle() {
        // Short move: triangular, so Cruise must never appear.
        let p = TrapezoidalProfile::new(0.0, 10.0, 50.0, 100.0, 100.0);
        let dur = p.duration();
        let n = 1000;
        for i in 0..=n {
            let t = dur * (i as f64) / (n as f64);
            assert_ne!(
                p.phase_at(t),
                MotionPhase::Cruise,
                "triangle should have no cruise phase (t={})",
                t
            );
        }
        // But it should still show accel then decel.
        assert_eq!(p.phase_at(dur * 0.25), MotionPhase::Accel);
        assert_eq!(p.phase_at(dur * 0.75), MotionPhase::Decel);
    }

    #[test]
    fn position_is_continuous_across_phase_boundaries() {
        // No jumps at the accel->cruise and cruise->decel seams.
        let p = TrapezoidalProfile::new(0.0, 500.0, 50.0, 100.0, 100.0);
        for &boundary in &[p.t_accel, p.t_cruise_end] {
            let just_before = p.sample(boundary - EPS).position;
            let just_after = p.sample(boundary + EPS).position;
            assert!(
                (just_before - just_after).abs() < 1e-4,
                "discontinuity at boundary {}: {} vs {}",
                boundary,
                just_before,
                just_after
            );
        }
    }

    #[test]
    fn asymmetric_trapezoid_uses_distinct_accel_and_decel_rates() {
        // distance=200, v_max=50, accel=100, decel=50 (half the accel rate).
        // d_accel = 2500/200 = 12.5, d_decel = 2500/100 = 25, sum=37.5 <= 200
        // => trapezoidal. t_accel = 50/100 = 0.5s, t_decel = 50/50 = 1.0s
        // (twice as long as accel, since decel is half the rate) — with the
        // old symmetric-only formula t_total would have been 4.25s; with
        // independent rates it's 4.75s.
        let p = TrapezoidalProfile::new(0.0, 200.0, 50.0, 100.0, 50.0);
        assert!(approx(p.duration(), 4.75));
        assert!(approx(p.t_accel, 0.5));

        // Cruise still reaches full max_velocity.
        let cruising = p.sample(2.0);
        assert!(approx(cruising.velocity, 50.0));

        // 0.75s of time remains in the decel phase at t=4.0s; decelerating
        // at 50 mm/s^2 (not 100) for that long means v = 50 * 0.75 = 37.5.
        let decelerating = p.sample(4.0);
        assert!(approx(decelerating.velocity, 37.5));

        // Still starts and ends at rest, exactly at the target.
        assert!(approx(p.sample(0.0).position, 0.0));
        let at_end = p.sample(p.duration());
        assert!(approx(at_end.position, 200.0));
        assert!(approx(at_end.velocity, 0.0));

        // Velocity invariant still holds with independent rates.
        let n = 1000;
        for i in 0..=n {
            let t = p.duration() * (i as f64) / (n as f64);
            assert!(p.sample(t).velocity <= 50.0 + 1e-6);
        }
    }

    #[test]
    fn asymmetric_triangle_peak_velocity_matches_formula() {
        // distance=10, v_max=50, accel=100, decel=25. Reaching v_max would
        // need d_accel+d_decel = 2500/200 + 2500/50 = 62.5 mm > 10 mm, so
        // triangular. Peak velocity from
        //   v_peak = sqrt(2 * distance * accel * decel / (accel + decel))
        //          = sqrt(2*10*100*25/125) = sqrt(400) = 20.
        let p = TrapezoidalProfile::new(0.0, 10.0, 50.0, 100.0, 25.0);
        assert!(approx(p.cruise_velocity, 20.0));
        assert!(p.cruise_velocity < 50.0);

        // Accel phase (0.2s) is a quarter the length of decel (0.8s), since
        // decel is a quarter the rate of accel — asymmetric in time, unlike
        // the old symmetric formula which forced them equal.
        assert!(approx(p.t_accel, 0.2));
        assert!(approx(p.duration(), 1.0));

        // Peak occurs exactly at the accel/decel boundary.
        let peak = p.sample(p.t_accel);
        assert!(approx(peak.velocity, 20.0));

        // Still starts and ends at rest, exactly at the target.
        assert!(approx(p.sample(0.0).position, 0.0));
        let at_end = p.sample(p.duration());
        assert!(approx(at_end.position, 10.0));
        assert!(approx(at_end.velocity, 0.0));
    }
}
