//! Jerk-limited (S-curve) motion, built by *filtering* a trapezoidal
//! profile rather than by deriving a seven-segment one.
//!
//! # The idea
//!
//! Convolve a trapezoidal velocity profile with a rectangular window of
//! length `T`. A box average of a piecewise-linear velocity is piecewise
//! quadratic, so acceleration becomes continuous and jerk becomes piecewise
//! constant — which is exactly a jerk-limited profile. Pick
//! `T = max(accel, decel) / max_jerk` and the jerk comes out at the limit.
//!
//! **A trapezoidal profile is the `max_jerk → ∞` case of this one**: the
//! window shrinks to nothing and a zero-width box filter is the identity.
//! `max_jerk: None` is that limit, and is bit-for-bit the unfiltered profile.
//!
//! # Filtering versus a seven-segment profile
//!
//! The classic S-curve derivation is a case analysis (cruise or no cruise,
//! `a_max` reached or not, short-move degenerate cases) whose branch count
//! multiplies when the move starts at a non-zero velocity. Filtering reuses
//! [`TrapezoidalProfile`] whole and adds no case analysis.
//!
//! The costs: the profile is not time-optimal (it runs exactly `T` longer),
//! and jerk is set indirectly by the window. See
//! [`JerkFilteredProfile::new_with_start_velocity`] for the one case where the
//! filter needs a correction to stay exact.
//!
//! # Time model
//!
//! `sample(t)` is a pure function of elapsed time. The convolution is
//! evaluated in closed form from the underlying profile's position and its
//! analytic integral; there is no running filter and no state.

use crate::trajectory::{MotionPhase, TrajectoryError, TrajectorySample, TrapezoidalProfile};

/// Windows shorter than this are treated as no filtering at all.
///
/// A numerical threshold. Filtered velocity is the difference quotient
/// `(p(t) − p(t−T)) / T`; as `T` shrinks the two positions agree to more
/// significant figures, so the subtraction keeps only rounding noise and
/// divides it by something tiny. At `T = 1e-9` a millimetre-scale profile has
/// already lost most of its velocity precision.
///
/// This guards the arithmetic only. A window shorter than one control cycle
/// cannot change what the drive sees, but this crate doesn't know the control
/// rate.
const MIN_FILTER_WINDOW: f64 = 1e-6;

/// A jerk-limited move: a [`TrapezoidalProfile`] convolved with a
/// rectangular window.
///
/// Same interface as the profile it wraps (`sample`/`phase_at`/`target`/
/// `duration`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JerkFilteredProfile {
    /// The profile being filtered. Built to a corrected target when the move
    /// starts in motion — see `new_with_start_velocity`.
    inner: TrapezoidalProfile,
    /// Filter window in seconds. `None` is the unfiltered `max_jerk → ∞`
    /// case, where every query delegates straight through.
    window: Option<f64>,
    start: f64,
    start_velocity: f64,
    /// The *true* commanded target, which is not `inner.target()` whenever
    /// the correction below applies.
    target: f64,
}

impl JerkFilteredProfile {
    /// A jerk-limited rest-to-rest move.
    ///
    /// `max_jerk: None` means no jerk limit, and reduces exactly to
    /// [`TrapezoidalProfile::new`].
    pub fn new(
        start: f64,
        end: f64,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        max_jerk: Option<f64>,
    ) -> Result<Self, TrajectoryError> {
        Self::new_with_start_velocity(
            start,
            0.0,
            end,
            max_speed,
            max_acceleration,
            max_deceleration,
            max_jerk,
        )
    }

    /// A jerk-limited move starting from a non-zero velocity.
    ///
    /// # Start-velocity correction
    ///
    /// For the filtered profile to *begin* at `start_velocity` with zero
    /// acceleration, the underlying profile is extended backwards before
    /// `t = 0` as a straight line at that velocity; the window reaches back
    /// into that history.
    ///
    /// That extension contributes area: the filtered move overshoots by
    /// exactly `start_velocity · T / 2`, independent of the profile's shape.
    /// So the inner profile is built to a target short by that amount, and the
    /// same constant offsets the start back into place. The profile then
    /// begins at `start` moving at `start_velocity` and ends at `end` at rest.
    /// The overshoot vanishes from rest, so rest-to-rest moves are unaffected.
    pub fn new_with_start_velocity(
        start: f64,
        start_velocity: f64,
        end: f64,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        max_jerk: Option<f64>,
    ) -> Result<Self, TrajectoryError> {
        let window = match max_jerk {
            None => None,
            Some(j) if !j.is_finite() || j <= 0.0 => {
                return Err(TrajectoryError::InvalidMaxJerk(j));
            }
            Some(j) => {
                // The window that yields this jerk across the steeper of the
                // two acceleration steps. Exact only when those steps don't
                // share a window; see `sample`.
                let w = max_acceleration.max(max_deceleration) / j;
                (w >= MIN_FILTER_WINDOW).then_some(w)
            }
        };

        let inner_target = match window {
            Some(w) => end - start_velocity * w / 2.0,
            None => end,
        };
        let inner = TrapezoidalProfile::new_with_start_velocity(
            start,
            start_velocity,
            inner_target,
            max_speed,
            max_acceleration,
            max_deceleration,
        )?;

        Ok(Self {
            inner,
            window,
            start,
            start_velocity,
            target: end,
        })
    }

    /// Total duration: the underlying move plus the filter window.
    pub fn duration(&self) -> f64 {
        self.inner.duration() + self.window.unwrap_or(0.0)
    }

    /// The commanded end position, not the corrected target the inner profile
    /// was built to.
    pub fn target(&self) -> f64 {
        self.target
    }

    /// Which phase the move is in.
    ///
    /// Delegates to the underlying profile, except that its final `Done`
    /// becomes `Decel` for the length of the filter tail, during which the
    /// axis is still slowing.
    pub fn phase_at(&self, t: f64) -> MotionPhase {
        let Some(_) = self.window else {
            return self.inner.phase_at(t);
        };
        if t >= self.duration() {
            return MotionPhase::Done;
        }
        match self.inner.phase_at(t) {
            MotionPhase::Done => MotionPhase::Decel,
            other => other,
        }
    }

    /// Sample the filtered trajectory at absolute elapsed time `t`.
    ///
    /// Velocity is the mean of the underlying velocity over the trailing
    /// window, which telescopes into a difference of *positions* — no
    /// integration needed:
    ///
    /// ```text
    /// v(t) = ( p(t) − p(t−T) ) / T
    /// ```
    ///
    /// Position is the mean of the underlying position over the same
    /// window, which does need the analytic integral, plus the constant
    /// from `new_with_start_velocity`'s correction.
    ///
    /// # Bound on the resulting jerk
    ///
    /// Acceleration here is `(a(t) − a(t−T)) / T` in the limit, and the
    /// underlying acceleration is piecewise constant. When the accel and
    /// decel steps are more than `T` apart the jerk is
    /// `max(accel, decel) / T`, i.e. exactly `max_jerk`. On a **short move
    /// whose cruise phase is briefer than the window**, one window straddles
    /// both steps and the jerk reaches `(accel + decel) / T` — up to twice
    /// the limit. Sizing the window for that case would double it on every
    /// move.
    pub fn sample(&self, t: f64) -> TrajectorySample {
        let Some(w) = self.window else {
            return self.inner.sample(t);
        };
        TrajectorySample {
            position: (self.position_integral(t) - self.position_integral(t - w)) / w
                + self.start_velocity * w / 2.0,
            velocity: (self.extended_position(t) - self.extended_position(t - w)) / w,
            // The same telescoping one level up: the filtered acceleration is
            // the mean of the underlying velocity over the window.
            acceleration: (self.extended_velocity(t) - self.extended_velocity(t - w)) / w,
        }
    }

    /// The underlying velocity, extended before `t = 0` as the constant
    /// `start_velocity` — the derivative of `extended_position`.
    fn extended_velocity(&self, t: f64) -> f64 {
        if t < 0.0 {
            self.start_velocity
        } else {
            self.inner.sample(t).velocity
        }
    }

    /// The underlying position, extended before `t = 0` as a straight line
    /// at `start_velocity`, as if the axis were already moving steadily.
    ///
    /// `TrapezoidalProfile::sample` holds at `start` for `t < 0` instead,
    /// which would make the filtered profile begin at rest.
    fn extended_position(&self, t: f64) -> f64 {
        if t < 0.0 {
            self.start + self.start_velocity * t
        } else {
            self.inner.sample(t).position
        }
    }

    /// `∫₀ᵗ extended_position(τ) dτ`, valid for negative `t` too (where it
    /// is negative, being a signed integral running backwards).
    fn position_integral(&self, t: f64) -> f64 {
        if t < 0.0 {
            self.start * t + self.start_velocity * t * t / 2.0
        } else {
            self.inner.position_integral(t)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    /// Numerically integrate a profile's own `sample().position`, to check
    /// the analytic integral against something that cannot share its bugs.
    fn numeric_integral(p: &TrapezoidalProfile, t: f64) -> f64 {
        let n = 20_000;
        let h = t / n as f64;
        let mut total = 0.0;
        for i in 0..n {
            // Midpoint rule: exact for the linear pieces, and second-order
            // on the quadratic ones.
            let mid = (i as f64 + 0.5) * h;
            total += p.sample(mid).position * h;
        }
        total
    }

    #[test]
    fn analytic_position_integral_matches_numeric_integration() {
        let cases = [
            TrapezoidalProfile::new(0.0, 100.0, 50.0, 200.0, 200.0).unwrap(),
            // Triangular: never reaches cruise.
            TrapezoidalProfile::new(0.0, 1.0, 50.0, 200.0, 100.0).unwrap(),
            // Negative direction.
            TrapezoidalProfile::new(10.0, -40.0, 25.0, 80.0, 120.0).unwrap(),
            // Entering with velocity, same direction.
            TrapezoidalProfile::new_with_start_velocity(0.0, 20.0, 100.0, 50.0, 200.0, 200.0)
                .unwrap(),
            // Entering with velocity that must be killed first (reversal),
            // which is the prefix branch.
            TrapezoidalProfile::new_with_start_velocity(0.0, 40.0, -50.0, 50.0, 200.0, 200.0)
                .unwrap(),
        ];
        for p in &cases {
            for frac in [0.13, 0.5, 0.87, 1.0, 1.5] {
                let t = p.duration() * frac;
                let analytic = p.position_integral(t);
                let numeric = numeric_integral(p, t);
                assert!(
                    approx(analytic, numeric, 1e-4 * (1.0 + numeric.abs())),
                    "t={t}: analytic {analytic} vs numeric {numeric}"
                );
            }
        }
    }

    #[test]
    fn no_jerk_limit_is_exactly_the_unfiltered_profile() {
        let plain = TrapezoidalProfile::new(0.0, 100.0, 50.0, 200.0, 200.0).unwrap();
        let filtered = JerkFilteredProfile::new(0.0, 100.0, 50.0, 200.0, 200.0, None).unwrap();
        assert_eq!(filtered.duration(), plain.duration());
        assert_eq!(filtered.target(), plain.target());
        for i in 0..=100 {
            let t = plain.duration() * i as f64 / 100.0;
            // Bit-identical, not merely close: None must delegate, not
            // approximate.
            assert_eq!(filtered.sample(t), plain.sample(t));
            assert_eq!(filtered.phase_at(t), plain.phase_at(t));
        }
    }

    #[test]
    fn filtered_move_starts_and_ends_exactly_where_asked() {
        for &(start, v0, end) in &[
            (0.0, 0.0, 100.0),
            (0.0, 20.0, 100.0),
            (10.0, -15.0, -60.0),
            (5.0, 30.0, 7.0), // entry speed far too high for the distance
        ] {
            let p = JerkFilteredProfile::new_with_start_velocity(
                start,
                v0,
                end,
                50.0,
                200.0,
                200.0,
                Some(2000.0),
            )
            .unwrap();

            let at_start = p.sample(0.0);
            assert!(
                approx(at_start.position, start, 1e-9),
                "start position: {} vs {start}",
                at_start.position
            );
            assert!(
                approx(at_start.velocity, v0, 1e-6),
                "start velocity: {} vs {v0}",
                at_start.velocity
            );

            let at_end = p.sample(p.duration());
            assert!(
                approx(at_end.position, end, 1e-6),
                "end position: {} vs {end}",
                at_end.position
            );
            assert!(approx(at_end.velocity, 0.0, 1e-6));
        }
    }

    #[test]
    fn filtering_removes_the_acceleration_step() {
        // The whole point: acceleration is continuous, where the unfiltered
        // profile steps from 0 to a_max instantly at t = 0.
        let max_accel = 200.0;
        let plain = TrapezoidalProfile::new(0.0, 100.0, 50.0, max_accel, max_accel).unwrap();
        let filtered =
            JerkFilteredProfile::new(0.0, 100.0, 50.0, max_accel, max_accel, Some(2000.0)).unwrap();

        let accel_of = |sample_at: &dyn Fn(f64) -> f64, t: f64| {
            let h = 1e-5;
            (sample_at(t + h) - sample_at(t - h)) / (2.0 * h)
        };
        let plain_v = |t: f64| plain.sample(t).velocity;
        let filtered_v = |t: f64| filtered.sample(t).velocity;

        // Just after the start the unfiltered profile is already at full
        // acceleration; the filtered one is still ramping into it.
        assert!(approx(accel_of(&plain_v, 0.002), max_accel, 1.0));
        assert!(accel_of(&filtered_v, 0.002) < max_accel * 0.9);

        // ...and nowhere does the filter exceed the acceleration limit.
        let n = 500;
        for i in 0..=n {
            let t = filtered.duration() * i as f64 / n as f64;
            let a = accel_of(&filtered_v, t);
            assert!(
                a.abs() <= max_accel * 1.001,
                "t={t}: |a| = {} exceeds {max_accel}",
                a.abs()
            );
        }
    }

    #[test]
    fn jerk_stays_within_the_limit_on_a_move_with_cruise() {
        let max_jerk = 2000.0;
        let p = JerkFilteredProfile::new(0.0, 200.0, 50.0, 200.0, 200.0, Some(max_jerk)).unwrap();
        // Long enough to cruise well past the window, which is the case the
        // bound is exact for (see `sample`'s docs for the short-move case).
        let h = 1e-4;
        let jerk_at = |t: f64| {
            let v = |x: f64| p.sample(x).velocity;
            (v(t + h) - 2.0 * v(t) + v(t - h)) / (h * h)
        };
        let n = 400;
        for i in 0..=n {
            let t = p.duration() * i as f64 / n as f64;
            let j = jerk_at(t);
            assert!(
                j.abs() <= max_jerk * 1.05,
                "t={t}: |j| = {} exceeds {max_jerk}",
                j.abs()
            );
        }
    }

    #[test]
    fn a_short_move_overshoots_the_jerk_limit_but_stays_within_twice_it() {
        // The documented edge case (see `sample`): with no cruise phase
        // longer than the window, one window straddles both the accel and
        // decel steps, so the jerk reaches (accel + decel)/T rather than
        // max(accel, decel)/T. This pins that it is genuinely bounded at
        // 2x and doesn't quietly run away.
        let (max_jerk, accel, decel) = (2000.0, 200.0, 200.0);
        let p = JerkFilteredProfile::new(0.0, 0.1, 50.0, accel, decel, Some(max_jerk)).unwrap();
        let window = accel.max(decel) / max_jerk;
        assert!(
            p.duration() - window < window,
            "test needs a move whose underlying duration is under one window"
        );

        let h = 1e-5;
        let jerk_at = |t: f64| {
            let v = |x: f64| p.sample(x).velocity;
            (v(t + h) - 2.0 * v(t) + v(t - h)) / (h * h)
        };
        let n = 400;
        let mut peak: f64 = 0.0;
        for i in 0..=n {
            let t = p.duration() * i as f64 / n as f64;
            peak = peak.max(jerk_at(t).abs());
        }
        assert!(
            peak <= (accel + decel) / window * 1.05,
            "peak jerk {peak} exceeds the (accel + decel)/T bound"
        );
    }

    #[test]
    fn duration_is_the_underlying_move_plus_exactly_one_window() {
        let max_jerk = 1000.0;
        let (accel, decel) = (200.0, 150.0);
        let filtered =
            JerkFilteredProfile::new(0.0, 100.0, 50.0, accel, decel, Some(max_jerk)).unwrap();
        let window = accel.max(decel) / max_jerk;
        let plain = TrapezoidalProfile::new(0.0, 100.0, 50.0, accel, decel).unwrap();
        assert!(approx(
            filtered.duration(),
            plain.duration() + window,
            1e-12
        ));
    }

    #[test]
    fn a_negligible_window_degenerates_to_unfiltered() {
        // Enormous jerk limit => window below MIN_FILTER_WINDOW => the
        // filter must switch itself off rather than divide by ~nothing.
        let plain = TrapezoidalProfile::new(0.0, 100.0, 50.0, 200.0, 200.0).unwrap();
        let filtered =
            JerkFilteredProfile::new(0.0, 100.0, 50.0, 200.0, 200.0, Some(1e12)).unwrap();
        assert_eq!(filtered.duration(), plain.duration());
        for i in 0..=50 {
            let t = plain.duration() * i as f64 / 50.0;
            assert_eq!(filtered.sample(t), plain.sample(t));
        }
    }

    #[test]
    fn rejects_a_non_finite_or_non_positive_jerk() {
        for bad in [0.0, -5.0, f64::INFINITY, f64::NAN] {
            assert!(matches!(
                JerkFilteredProfile::new(0.0, 10.0, 5.0, 20.0, 20.0, Some(bad)),
                Err(TrajectoryError::InvalidMaxJerk(_))
            ));
        }
    }
}
