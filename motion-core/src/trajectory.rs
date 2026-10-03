//! Trajectory generation: turning "go from A to B" into a stream of
//! instantaneous position setpoints.
//!
//! A servo drive needs one thing every control cycle: the *target position
//! for this instant* ("be at 12.47 mm now", then 4 ms later "be at 12.83 mm
//! now").
//!
//! # The model: trapezoidal velocity profile
//!
//! A move has up to three phases: accelerate to a max speed, cruise, decelerate
//! to zero exactly at the target. Plotted as speed-vs-time this is a trapezoid
//! (direction is constant through a move and applied separately — see
//! `direction`). Position, the integral of signed velocity, is a smooth
//! S-curve.
//!
//! If the move is too short to reach `max_speed`, the trapezoid loses its flat
//! top and becomes a triangle.
//!
//! # Time model: absolute-time query
//!
//! The generator is a **pure function of elapsed time**: asking where the axis
//! should be at t = 0.348 s computes the answer from closed-form kinematics
//! with no mutable state. A dt-stepping model belongs to the plant (sim or
//! servo loop); see `NOTE (dt seam)` below for where the two meet.

/// Which phase of the trapezoidal profile a given instant falls in.
///
/// Computed from the profile's phase-boundary times, so it is exact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionPhase {
    /// Before the move has started (t <= 0).
    Pre,
    /// Accelerating up toward cruise (or peak, if triangular).
    Accel,
    /// Cruising at constant speed. A triangular move has no cruise phase.
    Cruise,
    /// Decelerating down to a stop at the target.
    Decel,
    /// The move is complete (t >= duration); holding at the target, at rest.
    Done,
}

/// Reasons a `TrapezoidalProfile` could not be constructed.
///
/// Finiteness is checked alongside sign: `f64::INFINITY > 0.0` is `true`, so a
/// bare `> 0.0` check would let infinity through as NaN/infinity downstream.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrajectoryError {
    /// `start` or `end` is NaN or +/-infinity — not a real position.
    NonFinitePosition { start: f64, end: f64 },
    /// `max_speed` must be a finite, positive number.
    InvalidMaxSpeed(f64),
    /// `max_acceleration` must be a finite, positive number.
    InvalidMaxAcceleration(f64),
    /// `max_deceleration` must be a finite, positive number.
    InvalidMaxDeceleration(f64),
    /// `position` or `velocity` passed to [`StopRamp::new`] is NaN or
    /// +/-infinity.
    NonFiniteStopState { position: f64, velocity: f64 },
    /// `start_velocity` passed to
    /// [`TrapezoidalProfile::new_with_start_velocity`] is NaN or
    /// +/-infinity.
    NonFiniteStartVelocity(f64),
    /// `max_jerk` must be a finite, positive number *when given at all*.
    /// "No jerk limit" is `None`, not infinity — see [`JerkFilteredProfile`].
    ///
    /// [`JerkFilteredProfile`]: crate::jerk_filter::JerkFilteredProfile
    InvalidMaxJerk(f64),
}

impl std::fmt::Display for TrajectoryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrajectoryError::NonFinitePosition { start, end } => write!(
                f,
                "start ({start}) and end ({end}) must both be finite numbers"
            ),
            TrajectoryError::InvalidMaxSpeed(v) => {
                write!(f, "max_speed must be a finite, positive number (got {v})")
            }
            TrajectoryError::InvalidMaxAcceleration(a) => write!(
                f,
                "max_acceleration must be a finite, positive number (got {a})"
            ),
            TrajectoryError::InvalidMaxDeceleration(d) => write!(
                f,
                "max_deceleration must be a finite, positive number (got {d})"
            ),
            TrajectoryError::NonFiniteStopState { position, velocity } => write!(
                f,
                "position ({position}) and velocity ({velocity}) must both be finite numbers"
            ),
            TrajectoryError::NonFiniteStartVelocity(v) => {
                write!(f, "start_velocity must be a finite number (got {v})")
            }
            TrajectoryError::InvalidMaxJerk(v) => write!(
                f,
                "max_jerk must be a finite, positive number when given (got {v}); \
                 use no jerk limit rather than infinity"
            ),
        }
    }
}

impl std::error::Error for TrajectoryError {}

/// A single-axis point-to-point move described by a trapezoidal (or, for short
/// moves, triangular) velocity profile.
///
/// Units are unspecified but must be self-consistent (mm, mm/s, mm/s^2 in this
/// project). Values are `f64`; conversion to integer encoder counts belongs to
/// the backend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidalProfile {
    start: f64,
    end: f64,
    /// Sign of travel of the MAIN segment (see below): +1.0 if
    /// `end >= main_start`, else -1.0.
    direction: f64,
    /// Total distance travelled by the main segment (always >= 0; direction
    /// is carried separately).
    distance: f64,

    // Kinematic limits actually used for THIS move. `cruise_speed` may be
    // below the requested max_speed when the move is triangular.
    accel: f64,
    decel: f64,
    cruise_speed: f64,

    // --- Prefix: kills a `start_velocity` the main segment can't use
    // directly (opposite direction, or not enough room to stop before
    // `end` — see `new_with_start_velocity`). Zero-duration
    // (t_prefix_end == 0.0) whenever there's no prefix. A real prefix
    // decelerates `prefix_velocity` to rest at `prefix_decel`, exactly like
    // `StopRamp`, landing at `main_start`.
    prefix_velocity: f64,
    prefix_decel: f64,
    t_prefix_end: f64,
    /// Position where the main segment begins: `start` when there's no
    /// prefix, otherwise wherever the prefix's deceleration lands.
    main_start: f64,

    // --- Main segment: ramps from `main_start_velocity` up or down to
    // `cruise_speed`, cruises, then decelerates to rest at `end`. Phase
    // times below are ABSOLUTE (already include t_prefix_end).
    main_start_velocity: f64,
    t_accel: f64,      // end of phase 1 (accel or decel-to-cruise) == start of cruise
    t_cruise_end: f64, // end of cruise phase == start of final deceleration
    t_total: f64,      // move complete

    // Distance covered by the end of phase 1 (cached so the cruise/decel
    // position formulas don't recompute it every query).
    d_accel: f64,
}

/// The main (rest-target) segment of a profile: ramps from some starting
/// speed up or down to a cruise speed, cruises, then decelerates to rest —
/// computed independently of where it begins, so both `new()` and
/// `new_with_start_velocity` can build one and place it (directly, or after a
/// prefix) into the outer `TrapezoidalProfile`.
struct MainSegment {
    direction: f64,
    distance: f64,
    cruise_speed: f64,
    t_accel: f64,      // local: relative to the segment's own start
    t_cruise_end: f64, // local
    t_total: f64,      // local
    d_accel: f64,
}

/// A sample of the trajectory at one instant: the feed-forward reference the
/// control loop hands to the backend each cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrajectorySample {
    /// Target position at the queried time (same units as the profile).
    pub position: f64,
    /// Target velocity at the queried time. Useful as a feed-forward signal
    /// even though position alone drives the move.
    pub velocity: f64,
    /// Target acceleration at the queried time — the second feed-forward
    /// term, which a drive turns into a torque offset.
    ///
    /// Exact and closed-form, not differenced. It is piecewise *constant* for
    /// an unfiltered trapezoid (stepping at each phase boundary) and
    /// piecewise linear once jerk-limited.
    pub acceleration: f64,
}

impl TrapezoidalProfile {
    /// Build a profile for a move from `start` to `end`.
    ///
    /// `max_speed`, `max_acceleration`, and `max_deceleration` must all be
    /// finite and > 0, and `start`/`end` must both be finite; otherwise this
    /// returns `Err` rather than panicking. Acceleration and deceleration
    /// limits are independent.
    ///
    /// A zero-distance move is valid and produces a profile that simply
    /// reports `start` for all time with zero velocity (total duration 0).
    pub fn new(
        start: f64,
        end: f64,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Result<Self, TrajectoryError> {
        Self::validate_limits(start, end, max_speed, max_acceleration, max_deceleration)?;
        let seg = Self::build_main_segment(
            start,
            end,
            0.0,
            max_speed,
            max_acceleration,
            max_deceleration,
        );
        Ok(Self::from_segment(
            start,
            end,
            max_acceleration,
            max_deceleration,
            0.0,  // rest-to-rest: no start velocity
            None, // no prefix
            seg,
        ))
    }

    /// Build a profile for a move from `start` to `end`, starting from a
    /// nonzero `start_velocity` rather than the rest-to-rest assumption
    /// `new()` makes. Used to interrupt an in-flight move with a new target.
    /// The new move's own `max_speed`/`max_acceleration`/`max_deceleration`
    /// govern the entire profile, including any preamble needed to reconcile
    /// `start_velocity` with the new target; nothing carries over from a
    /// superseded move.
    ///
    /// Three kinematic cases arise from the geometry; the caller does not
    /// pick one:
    ///
    /// - **Same direction, room to spare**: ramps from `start_velocity` to a
    ///   cruise speed (accelerating, or decelerating into cruise if
    ///   `start_velocity` is already above `max_speed`), cruises, then
    ///   decelerates to rest — same shape `new()` builds, just starting
    ///   partway up (or down) the ramp instead of from rest.
    /// - **Same direction, not enough room**: too fast to stop before `end`
    ///   at `max_deceleration`. Overshoots: decelerates to rest past `end`,
    ///   then a fresh rest-to-rest move back.
    /// - **Opposite direction**: decelerates to rest first, then a normal
    ///   rest-to-rest move from wherever that lands.
    ///
    /// The last two cases share a code path: decelerate to rest, then run a
    /// fresh rest-to-rest segment. Which side of `end` the landing spot falls
    /// on distinguishes "overshoot and come back" from "reverse and go".
    pub fn new_with_start_velocity(
        start: f64,
        start_velocity: f64,
        end: f64,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Result<Self, TrajectoryError> {
        Self::validate_limits(start, end, max_speed, max_acceleration, max_deceleration)?;
        if !start_velocity.is_finite() {
            return Err(TrajectoryError::NonFiniteStartVelocity(start_velocity));
        }

        let delta = end - start;
        let direction = if delta >= 0.0 { 1.0 } else { -1.0 };
        let distance = delta.abs();
        // Component of start_velocity along the direction of travel to
        // `end`. Negative means start_velocity points away from `end`.
        let v0_along = start_velocity * direction;
        // The least distance in which v0_along can be brought to rest at
        // max_deceleration. If `end` is closer, it cannot be reached directly.
        let min_stop_distance = (v0_along * v0_along) / (2.0 * max_deceleration);

        if v0_along > 0.0 && distance >= min_stop_distance {
            // Same direction, room to spare: ride start_velocity straight
            // into the main segment, no prefix needed.
            let seg = Self::build_main_segment(
                start,
                end,
                v0_along,
                max_speed,
                max_acceleration,
                max_deceleration,
            );
            Ok(Self::from_segment(
                start,
                end,
                max_acceleration,
                max_deceleration,
                v0_along,
                None,
                seg,
            ))
        } else {
            // Not enough room, or start_velocity points the wrong way:
            // decelerate it to rest first (the same math as StopRamp), then
            // a fresh rest-to-rest segment from wherever that lands.
            let prefix_dir = start_velocity.signum();
            let t_prefix_end = start_velocity.abs() / max_deceleration;
            let prefix_distance = (start_velocity * start_velocity) / (2.0 * max_deceleration);
            let main_start = start + prefix_dir * prefix_distance;

            let seg = Self::build_main_segment(
                main_start,
                end,
                0.0,
                max_speed,
                max_acceleration,
                max_deceleration,
            );
            Ok(Self::from_segment(
                start,
                end,
                max_acceleration,
                max_deceleration,
                0.0, // main segment starts from rest, after the prefix
                Some((start_velocity, t_prefix_end, main_start)),
                seg,
            ))
        }
    }

    fn validate_limits(
        start: f64,
        end: f64,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Result<(), TrajectoryError> {
        if !start.is_finite() || !end.is_finite() {
            return Err(TrajectoryError::NonFinitePosition { start, end });
        }
        if !(max_speed.is_finite() && max_speed > 0.0) {
            return Err(TrajectoryError::InvalidMaxSpeed(max_speed));
        }
        if !(max_acceleration.is_finite() && max_acceleration > 0.0) {
            return Err(TrajectoryError::InvalidMaxAcceleration(max_acceleration));
        }
        if !(max_deceleration.is_finite() && max_deceleration > 0.0) {
            return Err(TrajectoryError::InvalidMaxDeceleration(max_deceleration));
        }
        Ok(())
    }

    /// Build the main (rest-target) segment: ramp from `start_velocity`
    /// (>= 0, along the direction of travel) to a cruise speed, cruise,
    /// decelerate to rest at `end`. The caller must have established that
    /// there is enough room to stop.
    fn build_main_segment(
        start: f64,
        end: f64,
        start_velocity: f64,
        max_speed: f64,
        accel: f64,
        decel: f64,
    ) -> MainSegment {
        let delta = end - start;
        let distance = delta.abs();
        let direction = if delta >= 0.0 { 1.0 } else { -1.0 };

        // Degenerate move: already there. Everything is zero / start.
        if distance == 0.0 {
            return MainSegment {
                direction,
                distance: 0.0,
                cruise_speed: 0.0,
                t_accel: 0.0,
                t_cruise_end: 0.0,
                t_total: 0.0,
                d_accel: 0.0,
            };
        }

        let v0 = start_velocity;

        // --- Pick the cruise speed ----------------------------------------
        //
        // If v0 already exceeds max_speed (only reachable from
        // new_with_start_velocity), cruise_speed is just max_speed — the
        // caller's room check guarantees space to decelerate through it and
        // still stop by `end`.
        //
        // Otherwise it's the trapezoid-vs-triangle decision, generalized to
        // start from v0 instead of 0: v^2 = v0^2 + 2*a*d => d = (v^2-v0^2)/(2a),
        // reducing to the v0 == 0 formulas when v0 is 0.
        let cruise_speed = if v0 <= max_speed {
            let d_accel_full = (max_speed * max_speed - v0 * v0) / (2.0 * accel);
            let d_decel_full = (max_speed * max_speed) / (2.0 * decel);
            if d_accel_full + d_decel_full <= distance {
                max_speed
            } else {
                // Triangular: peak speed where ramp-from-v0 and decel-to-0
                // meet: (v_peak^2-v0^2)/(2*accel) + v_peak^2/(2*decel) = distance
                // => v_peak = sqrt((2*accel*decel*distance + v0^2*decel)/(accel+decel)).
                ((2.0 * accel * decel * distance + v0 * v0 * decel) / (accel + decel)).sqrt()
            }
        } else {
            max_speed
        };

        // Phase 1 ramps from v0 to cruise_speed — accelerating if
        // cruise_speed >= v0, decelerating into cruise otherwise.
        let ramping_up = cruise_speed >= v0;
        let rate1 = if ramping_up { accel } else { decel };
        let t_accel = (cruise_speed - v0).abs() / rate1;
        let d_accel = (cruise_speed * cruise_speed - v0 * v0).abs() / (2.0 * rate1);

        let t_decel = cruise_speed / decel;
        let d_decel = (cruise_speed * cruise_speed) / (2.0 * decel);

        let d_cruise = distance - d_accel - d_decel;
        let t_cruise = d_cruise / cruise_speed;

        let t_cruise_end = t_accel + t_cruise;
        let t_total = t_cruise_end + t_decel;

        MainSegment {
            direction,
            distance,
            cruise_speed,
            t_accel,
            t_cruise_end,
            t_total,
            d_accel,
        }
    }

    /// Assemble a `TrapezoidalProfile` from a `MainSegment`, placed either
    /// directly at `start` (`prefix: None`) or after a prefix
    /// `(prefix_velocity, t_prefix_end, main_start)` that decelerated to
    /// rest first (`main_start_velocity` is then always 0.0). Converts the
    /// segment's local phase times into the profile's absolute ones.
    fn from_segment(
        start: f64,
        end: f64,
        accel: f64,
        decel: f64,
        main_start_velocity: f64,
        prefix: Option<(f64, f64, f64)>,
        seg: MainSegment,
    ) -> Self {
        let (prefix_velocity, t_prefix_end, main_start) = prefix.unwrap_or((0.0, 0.0, start));

        Self {
            start,
            end,
            direction: seg.direction,
            distance: seg.distance,
            accel,
            decel,
            cruise_speed: seg.cruise_speed,
            prefix_velocity,
            prefix_decel: decel,
            t_prefix_end,
            main_start,
            main_start_velocity,
            t_accel: t_prefix_end + seg.t_accel,
            t_cruise_end: t_prefix_end + seg.t_cruise_end,
            t_total: t_prefix_end + seg.t_total,
            d_accel: seg.d_accel,
        }
    }

    /// Total duration of the move in seconds.
    pub fn duration(&self) -> f64 {
        self.t_total
    }

    /// The commanded end position.
    pub fn target(&self) -> f64 {
        self.end
    }

    /// The area under the position curve: `∫₀ᵗ position(τ) dτ`.
    ///
    /// Used by [`JerkFilteredProfile`], whose box-filtered position is the
    /// *mean* position over a trailing window: a difference of this integral at
    /// two times, divided by the window. It is piecewise over the same phases
    /// [`sample`] uses.
    ///
    /// Outside the move this follows `sample`'s clamping: zero for `t <= 0`
    /// (the profile holds at `start` before it begins), and holding at `end`
    /// after. The linear backward extension a filtered profile needs at a
    /// non-zero start velocity is handled by `JerkFilteredProfile`.
    ///
    /// [`JerkFilteredProfile`]: crate::jerk_filter::JerkFilteredProfile
    /// [`sample`]: TrapezoidalProfile::sample
    pub(crate) fn position_integral(&self, t: f64) -> f64 {
        if t <= 0.0 {
            return 0.0;
        }
        let mut total = 0.0;

        // --- Prefix: p(τ) = start + pv·τ − pd·½·pdec·τ² ---
        let tp = t.min(self.t_prefix_end);
        if tp > 0.0 {
            let pd = self.prefix_velocity.signum();
            total += self.start * tp + self.prefix_velocity * tp * tp / 2.0
                - pd * self.prefix_decel * tp * tp * tp / 6.0;
        }
        if t <= self.t_prefix_end {
            return total;
        }

        // --- Main segment, in its own local time ---
        let u = t.min(self.t_total) - self.t_prefix_end;
        total += self.main_start * u + self.direction * self.main_distance_integral(u);
        if t <= self.t_total {
            return total;
        }

        // --- After the move: sitting at `end`. ---
        total + self.end * (t - self.t_total)
    }

    /// `∫₀ᵘ x(w) dw`, where `x` is the main segment's distance travelled
    /// along the direction of motion — the `pos_along` of [`sample`], with
    /// `u` local to the main segment (absolute time minus `t_prefix_end`).
    ///
    /// [`sample`]: TrapezoidalProfile::sample
    fn main_distance_integral(&self, u: f64) -> f64 {
        let t_accel = self.t_accel - self.t_prefix_end;
        let t_cruise_end = self.t_cruise_end - self.t_prefix_end;
        let t_total = self.t_total - self.t_prefix_end;

        // Same ramp direction `sample` picks.
        let ramping_up = self.cruise_speed >= self.main_start_velocity;
        let (sign, rate) = if ramping_up {
            (1.0, self.accel)
        } else {
            (-1.0, self.decel)
        };

        // --- Phase 1: x(w) = v₀·w + ½·s·r·w² ---
        let w = u.min(t_accel);
        let mut total = self.main_start_velocity * w * w / 2.0 + sign * rate * w * w * w / 6.0;
        if u <= t_accel {
            return total;
        }

        // --- Phase 2 (cruise): x(w) = d_accel + cruise·w, w from t_accel ---
        let w = u.min(t_cruise_end) - t_accel;
        total += self.d_accel * w + self.cruise_speed * w * w / 2.0;
        if u <= t_cruise_end {
            return total;
        }

        // --- Phase 3: x(w) = distance − ½·decel·(t_total − w)², so the
        // integral runs backwards from the stop. ---
        let w = u.min(t_total);
        let td_start = t_total - t_cruise_end;
        let td_end = t_total - w;
        total += self.distance * (w - t_cruise_end)
            - self.decel * (td_start * td_start * td_start - td_end * td_end * td_end) / 6.0;
        if u <= t_total {
            return total;
        }

        // Past the main segment: distance is constant at `distance`.
        total + self.distance * (u - t_total)
    }

    /// Which phase the move is in at absolute elapsed time `t` (seconds).
    ///
    /// Computed directly from the known phase-boundary times, so it's exact.
    /// For a triangular move the cruise window is empty (`t_accel ==
    /// t_cruise_end`), so `Cruise` is never returned. A `new_with_start_velocity`
    /// prefix decelerating an incompatible start velocity reports `Decel`
    /// too. Phase 1 of the main segment reports `Accel` or `Decel` depending
    /// on whether it's ramping up to cruise or down into it.
    pub fn phase_at(&self, t: f64) -> MotionPhase {
        if t <= 0.0 || self.distance == 0.0 {
            MotionPhase::Pre
        } else if t >= self.t_total {
            MotionPhase::Done
        } else if t < self.t_prefix_end {
            MotionPhase::Decel
        } else if t < self.t_accel {
            if self.cruise_speed >= self.main_start_velocity {
                MotionPhase::Accel
            } else {
                MotionPhase::Decel
            }
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
    /// (at whatever velocity the move actually began with — 0 for a plain
    /// `new()` move), `t > duration` returns the end (at rest). This makes
    /// the "move already finished" case need no completion flag.
    ///
    /// NOTE (dt seam): the control loop calls this once per cycle with the
    /// actual elapsed time, then hands the sample to the backend. The backend
    /// (sim plant model or a real drive) is the stateful, dt-stepping layer
    /// that consumes the setpoint. This method stays pure.
    pub fn sample(&self, t: f64) -> TrajectorySample {
        if t <= 0.0 {
            let velocity = if self.t_prefix_end > 0.0 {
                self.prefix_velocity
            } else {
                self.direction * self.main_start_velocity
            };
            return TrajectorySample {
                position: self.start,
                velocity,
                // Zero even when `velocity` is not, on a move entered in motion.
                acceleration: 0.0,
            };
        }
        // After the move (this also covers the zero-distance case, since
        // then start == end and t_total == t_prefix_end): sit at end, at rest.
        if t >= self.t_total {
            return TrajectorySample {
                position: self.end,
                velocity: 0.0,
                acceleration: 0.0,
            };
        }

        if t < self.t_prefix_end {
            // --- Prefix: decelerating an incompatible start_velocity to
            // rest, the same math as `StopRamp::sample`.
            let pd = self.prefix_velocity.signum();
            return TrajectorySample {
                position: self.start + self.prefix_velocity * t
                    - pd * 0.5 * self.prefix_decel * t * t,
                velocity: self.prefix_velocity - pd * self.prefix_decel * t,
                acceleration: -pd * self.prefix_decel,
            };
        }

        // Main segment: measure local time from wherever it begins
        // (main_start, at absolute time t_prefix_end).
        let t_local = t - self.t_prefix_end;
        let t_accel_local = self.t_accel - self.t_prefix_end;
        let t_cruise_end_local = self.t_cruise_end - self.t_prefix_end;
        let t_total_local = self.t_total - self.t_prefix_end;

        // Compute position/speed magnitude for a positive-direction move,
        // then re-sign into direction-aware position/velocity below.
        // Acceleration comes out of the same branches, exactly: it is the
        // rate each phase was built around, not a difference of samples.
        let (pos_along, speed_along, accel_along) = if t_local < t_accel_local {
            // --- Phase 1: ramp from main_start_velocity to cruise_speed ---
            // Accelerating if cruise_speed >= main_start_velocity,
            // decelerating into cruise otherwise.
            //   v(t) = v0 + sign * rate * t
            //   x(t) = v0 * t + 1/2 * sign * rate * t^2
            let ramping_up = self.cruise_speed >= self.main_start_velocity;
            let (sign, rate) = if ramping_up {
                (1.0, self.accel)
            } else {
                (-1.0, self.decel)
            };
            let v = self.main_start_velocity + sign * rate * t_local;
            let x = self.main_start_velocity * t_local + 0.5 * sign * rate * t_local * t_local;
            (x, v, sign * rate)
        } else if t_local < t_cruise_end_local {
            // --- Cruise phase (constant speed) ---
            //   v(t) = cruise_speed
            //   x(t) = d_accel + cruise_speed * (t - t_accel)
            let v = self.cruise_speed;
            let x = self.d_accel + self.cruise_speed * (t_local - t_accel_local);
            (x, v, 0.0)
        } else {
            // --- Deceleration phase ---
            // Measure time remaining until the move ends and integrate
            // backwards from a full stop at t_total, at the deceleration
            // rate (independent of the acceleration rate).
            //   td = t_total - t              (time left)
            //   v(t) = decel * td
            //   x(t) = distance - 1/2 * decel * td^2
            let td = t_total_local - t_local;
            let v = self.decel * td;
            let x = self.distance - 0.5 * self.decel * td * td;
            (x, v, -self.decel)
        };

        TrajectorySample {
            position: self.main_start + self.direction * pos_along,
            velocity: self.direction * speed_along,
            acceleration: self.direction * accel_along,
        }
    }
}

/// A controlled deceleration to rest from the axis's current velocity.
///
/// Used for a commanded stop: there is no target position and no accel/cruise
/// phase, just "decelerate from `start_velocity` to zero at `max_deceleration`,
/// holding wherever that lands."
///
/// Like `TrapezoidalProfile`, it is a pure function of elapsed time (see the
/// module docs' "Time model" section).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopRamp {
    start_position: f64,
    start_velocity: f64,
    decel: f64,
    /// Sign of `start_velocity`. `f64::signum()` returns +/-1.0 even for zero,
    /// which is harmless: when `start_velocity == 0.0` the ramp has zero
    /// duration and this is multiplied by a clamped `t` of `0.0`.
    direction: f64,
    /// Time to reach zero velocity: `|start_velocity| / decel`.
    t_stop: f64,
    /// Position once at rest.
    final_position: f64,
}

impl StopRamp {
    /// Build a stop ramp from the axis's current position and velocity.
    ///
    /// `max_deceleration` must be finite and > 0; `start_position` and
    /// `start_velocity` must both be finite. `start_velocity` may be any
    /// sign (or zero, in which case the ramp has zero duration — the axis
    /// was already at rest).
    pub fn new(
        start_position: f64,
        start_velocity: f64,
        max_deceleration: f64,
    ) -> Result<Self, TrajectoryError> {
        if !start_position.is_finite() || !start_velocity.is_finite() {
            return Err(TrajectoryError::NonFiniteStopState {
                position: start_position,
                velocity: start_velocity,
            });
        }
        if !(max_deceleration.is_finite() && max_deceleration > 0.0) {
            return Err(TrajectoryError::InvalidMaxDeceleration(max_deceleration));
        }

        let direction = start_velocity.signum();
        let t_stop = start_velocity.abs() / max_deceleration;
        // v^2 = 2*a*d => d = v^2/(2a), signed by direction of travel.
        let final_position = start_position
            + direction * (start_velocity.abs() * start_velocity.abs()) / (2.0 * max_deceleration);

        Ok(Self {
            start_position,
            start_velocity,
            decel: max_deceleration,
            direction,
            t_stop,
            final_position,
        })
    }

    /// Total duration of the ramp in seconds — `0.0` if the axis was already
    /// at rest.
    pub fn duration(&self) -> f64 {
        self.t_stop
    }

    /// The position the axis comes to rest at.
    pub fn target(&self) -> f64 {
        self.final_position
    }

    /// Which phase the ramp is in at elapsed time `t` (seconds since the
    /// stop was commanded).
    ///
    /// Only ever `Pre`, `Decel`, or `Done`. Unlike `TrapezoidalProfile::phase_at`
    /// (where a zero-distance move reports `Pre` forever), a zero-duration stop
    /// reports `Done` for any `t >= 0.0`: the axis was already at rest.
    pub fn phase_at(&self, t: f64) -> MotionPhase {
        if t < 0.0 {
            MotionPhase::Pre
        } else if t >= self.t_stop {
            MotionPhase::Done
        } else {
            MotionPhase::Decel
        }
    }

    /// Sample the ramp at elapsed time `t` (seconds since the stop was
    /// commanded). Clamped the same way `TrapezoidalProfile::sample` is:
    /// `t < 0` returns the starting state, `t > duration` holds at rest at
    /// [`target`](Self::target).
    pub fn sample(&self, t: f64) -> TrajectorySample {
        let clamped = t.clamp(0.0, self.t_stop);
        TrajectorySample {
            position: self.start_position + self.start_velocity * clamped
                - self.direction * 0.5 * self.decel * clamped * clamped,
            velocity: self.start_velocity - self.direction * self.decel * clamped,
            // Constant through the ramp and zero once stopped. Compared
            // against the unclamped `t` so the held-at-rest tail reports zero.
            acceleration: if t > 0.0 && t < self.t_stop {
                -self.direction * self.decel
            } else {
                0.0
            },
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
        let p = TrapezoidalProfile::new(10.0, 110.0, 50.0, 100.0, 100.0).unwrap();
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
        let p = TrapezoidalProfile::new(0.0, 100.0, 50.0, 100.0, 100.0).unwrap();
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
    fn never_exceeds_max_speed() {
        let smax = 50.0;
        let p = TrapezoidalProfile::new(0.0, 500.0, smax, 100.0, 100.0).unwrap();
        // Sample densely across the whole move.
        let n = 10_000;
        for i in 0..=n {
            let t = p.duration() * (i as f64) / (n as f64);
            let s = p.sample(t);
            assert!(
                s.velocity.abs() <= smax + 1e-6,
                "velocity {} exceeded smax {} at t={}",
                s.velocity,
                smax,
                t
            );
        }
    }

    #[test]
    fn long_move_is_trapezoidal_and_reaches_cruise() {
        // 500 mm with smax=50, amax=100.
        // accel distance = v^2/2a = 2500/200 = 12.5 mm; accel+decel = 25 mm.
        // 25 <= 500, so trapezoidal; cruise speed should equal smax.
        let p = TrapezoidalProfile::new(0.0, 500.0, 50.0, 100.0, 100.0).unwrap();
        assert!(approx(p.cruise_speed, 50.0));
        // Somewhere in the middle we should be cruising at exactly smax.
        let mid = p.sample(p.duration() / 2.0);
        assert!(approx(mid.velocity, 50.0));
    }

    #[test]
    fn short_move_is_triangular_and_stays_below_max_speed() {
        // 10 mm with smax=50, amax=100.
        // accel+decel distance to reach smax = 25 mm > 10 mm, so triangular.
        // peak speed = sqrt(a*d) = sqrt(100*10) = sqrt(1000) ~= 31.62 mm/s.
        let p = TrapezoidalProfile::new(0.0, 10.0, 50.0, 100.0, 100.0).unwrap();
        let expected_peak = (100.0_f64 * 10.0).sqrt();
        assert!(approx(p.cruise_speed, expected_peak));
        assert!(p.cruise_speed < 50.0);
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
        let p = TrapezoidalProfile::new(0.0, 200.0, 40.0, 80.0, 80.0).unwrap();
        let dur = p.duration();
        let n = 1000;
        for i in 0..=n {
            let t = dur * (i as f64) / (n as f64);
            let a = p.sample(t).position - 0.0;
            let b = 200.0 - p.sample(dur - t).position;
            assert!((a - b).abs() < 1e-6, "asymmetry at t={}: {} vs {}", t, a, b);
        }
    }

    #[test]
    fn negative_direction_move_works() {
        // Moving from 100 down to 0 should mirror the positive case.
        let p = TrapezoidalProfile::new(100.0, 0.0, 50.0, 100.0, 100.0).unwrap();
        assert!(approx(p.sample(0.0).position, 100.0));
        assert!(approx(p.sample(p.duration()).position, 0.0));
        // Velocity should be negative during the move.
        let mid = p.sample(p.duration() / 2.0);
        assert!(mid.velocity < 0.0);
    }

    #[test]
    fn zero_distance_move_is_inert() {
        let p = TrapezoidalProfile::new(42.0, 42.0, 50.0, 100.0, 100.0).unwrap();
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
        let p = TrapezoidalProfile::new(0.0, 200.0, 50.0, 100.0, 100.0).unwrap();
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
        let p = TrapezoidalProfile::new(0.0, 10.0, 50.0, 100.0, 100.0).unwrap();
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
        let p = TrapezoidalProfile::new(0.0, 500.0, 50.0, 100.0, 100.0).unwrap();
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
        // (twice as long as accel, since decel is half the rate), so
        // t_total is 4.75s. Equal rates would give 4.25s.
        let p = TrapezoidalProfile::new(0.0, 200.0, 50.0, 100.0, 50.0).unwrap();
        assert!(approx(p.duration(), 4.75));
        assert!(approx(p.t_accel, 0.5));

        // Cruise still reaches full max_speed.
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
    fn asymmetric_triangle_peak_speed_matches_formula() {
        // distance=10, v_max=50, accel=100, decel=25. Reaching v_max would
        // need d_accel+d_decel = 2500/200 + 2500/50 = 62.5 mm > 10 mm, so
        // triangular. Peak speed from
        //   v_peak = sqrt(2 * distance * accel * decel / (accel + decel))
        //          = sqrt(2*10*100*25/125) = sqrt(400) = 20.
        let p = TrapezoidalProfile::new(0.0, 10.0, 50.0, 100.0, 25.0).unwrap();
        assert!(approx(p.cruise_speed, 20.0));
        assert!(p.cruise_speed < 50.0);

        // Accel phase (0.2s) is a quarter the length of decel (0.8s), since
        // decel is a quarter the rate of accel.
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

    #[test]
    fn rejects_non_positive_max_speed() {
        assert_eq!(
            TrapezoidalProfile::new(0.0, 100.0, 0.0, 100.0, 100.0),
            Err(TrajectoryError::InvalidMaxSpeed(0.0))
        );
        assert_eq!(
            TrapezoidalProfile::new(0.0, 100.0, -5.0, 100.0, 100.0),
            Err(TrajectoryError::InvalidMaxSpeed(-5.0))
        );
    }

    #[test]
    fn rejects_non_positive_max_acceleration() {
        assert_eq!(
            TrapezoidalProfile::new(0.0, 100.0, 50.0, -10.0, 100.0),
            Err(TrajectoryError::InvalidMaxAcceleration(-10.0))
        );
    }

    #[test]
    fn rejects_non_positive_max_deceleration() {
        assert_eq!(
            TrapezoidalProfile::new(0.0, 100.0, 50.0, 100.0, -10.0),
            Err(TrajectoryError::InvalidMaxDeceleration(-10.0))
        );
    }

    #[test]
    fn rejects_non_finite_inputs() {
        // NaN positions. NaN never equals NaN (IEEE 754), so we match on the
        // variant and check `.is_nan()` rather than `assert_eq!` against a
        // NaN payload.
        match TrapezoidalProfile::new(f64::NAN, 100.0, 50.0, 100.0, 100.0) {
            Err(TrajectoryError::NonFinitePosition { start, end }) => {
                assert!(start.is_nan());
                assert!(approx(end, 100.0));
            }
            other => panic!("expected NonFinitePosition, got {other:?}"),
        }

        // Infinite max_speed would otherwise pass a bare `> 0.0` check;
        // infinity does equal infinity under IEEE 754, so assert_eq! is fine
        // here.
        assert_eq!(
            TrapezoidalProfile::new(0.0, 100.0, f64::INFINITY, 100.0, 100.0),
            Err(TrajectoryError::InvalidMaxSpeed(f64::INFINITY))
        );

        // NaN max_acceleration.
        match TrapezoidalProfile::new(0.0, 100.0, 50.0, f64::NAN, 100.0) {
            Err(TrajectoryError::InvalidMaxAcceleration(a)) => assert!(a.is_nan()),
            other => panic!("expected InvalidMaxAcceleration, got {other:?}"),
        }
    }

    #[test]
    fn stop_ramp_decelerates_positive_velocity_to_rest() {
        // v0=40, decel=100 => t_stop=0.4s, distance = v^2/(2a) = 1600/200 = 8.
        let r = StopRamp::new(10.0, 40.0, 100.0).unwrap();
        assert!(approx(r.duration(), 0.4));
        assert!(approx(r.target(), 18.0));
        assert!(approx(r.sample(0.0).position, 10.0));
        assert!(approx(r.sample(0.0).velocity, 40.0));
        assert!(approx(r.sample(r.duration()).position, 18.0));
        assert!(approx(r.sample(r.duration()).velocity, 0.0));
        // Halfway through in time, velocity should be linearly halved.
        let mid = r.sample(0.2);
        assert!(approx(mid.velocity, 20.0));
    }

    #[test]
    fn stop_ramp_decelerates_negative_velocity_to_rest() {
        // Mirror image: v0=-40, decel=100 => same duration/distance, opposite sign.
        let r = StopRamp::new(10.0, -40.0, 100.0).unwrap();
        assert!(approx(r.duration(), 0.4));
        assert!(approx(r.target(), 2.0));
        assert!(approx(r.sample(r.duration()).position, 2.0));
        assert!(approx(r.sample(r.duration()).velocity, 0.0));
        let mid = r.sample(0.2);
        assert!(approx(mid.velocity, -20.0));
    }

    #[test]
    fn stop_ramp_from_rest_is_instantly_done() {
        let r = StopRamp::new(5.0, 0.0, 100.0).unwrap();
        assert!(approx(r.duration(), 0.0));
        assert!(approx(r.target(), 5.0));
        // Unlike TrapezoidalProfile's zero-distance case (always Pre), a
        // zero-duration stop is Done for any t >= 0 — nothing pending.
        assert_eq!(r.phase_at(0.0), MotionPhase::Done);
        assert_eq!(r.phase_at(1.0), MotionPhase::Done);
        assert_eq!(r.phase_at(-1.0), MotionPhase::Pre);
    }

    #[test]
    fn stop_ramp_clamps_before_and_after() {
        let r = StopRamp::new(0.0, 40.0, 100.0).unwrap();
        let before = r.sample(-5.0);
        assert!(approx(before.position, 0.0));
        assert!(approx(before.velocity, 40.0));
        let after = r.sample(r.duration() + 5.0);
        assert!(approx(after.position, r.target()));
        assert!(approx(after.velocity, 0.0));
    }

    #[test]
    fn stop_ramp_phase_at_reports_decel_then_done() {
        let r = StopRamp::new(0.0, 40.0, 100.0).unwrap();
        assert_eq!(r.phase_at(-1.0), MotionPhase::Pre);
        assert_eq!(r.phase_at(0.0), MotionPhase::Decel);
        assert_eq!(r.phase_at(r.duration() / 2.0), MotionPhase::Decel);
        assert_eq!(r.phase_at(r.duration()), MotionPhase::Done);
        assert_eq!(r.phase_at(r.duration() + 1.0), MotionPhase::Done);
    }

    #[test]
    fn stop_ramp_rejects_non_positive_max_deceleration() {
        assert_eq!(
            StopRamp::new(0.0, 40.0, 0.0),
            Err(TrajectoryError::InvalidMaxDeceleration(0.0))
        );
        assert_eq!(
            StopRamp::new(0.0, 40.0, -10.0),
            Err(TrajectoryError::InvalidMaxDeceleration(-10.0))
        );
    }

    #[test]
    fn stop_ramp_rejects_non_finite_inputs() {
        match StopRamp::new(f64::NAN, 40.0, 100.0) {
            Err(TrajectoryError::NonFiniteStopState { position, velocity }) => {
                assert!(position.is_nan());
                assert!(approx(velocity, 40.0));
            }
            other => panic!("expected NonFiniteStopState, got {other:?}"),
        }
        match StopRamp::new(0.0, f64::INFINITY, 100.0) {
            Err(TrajectoryError::NonFiniteStopState { velocity, .. }) => {
                assert!(velocity.is_infinite());
            }
            other => panic!("expected NonFiniteStopState, got {other:?}"),
        }
    }

    // --- new_with_start_velocity ------------------------------------------

    #[test]
    fn start_velocity_zero_matches_plain_new() {
        // A thin-wrapper guarantee: start_velocity: 0.0 must reproduce
        // new()'s output exactly, for every phase field new() exposes.
        let a = TrapezoidalProfile::new(10.0, 250.0, 50.0, 100.0, 80.0).unwrap();
        let b = TrapezoidalProfile::new_with_start_velocity(10.0, 0.0, 250.0, 50.0, 100.0, 80.0)
            .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn same_direction_with_room_reaches_cruise() {
        // v0=20, distance=200, max_speed=50, accel=decel=100.
        // d_accel = (50^2-20^2)/200 = 10.5, d_decel = 2500/200 = 12.5,
        // sum=23 <= 200, so trapezoidal: cruise reaches the full max_speed.
        let p = TrapezoidalProfile::new_with_start_velocity(0.0, 20.0, 200.0, 50.0, 100.0, 100.0)
            .unwrap();
        assert!(approx(p.cruise_speed, 50.0));
        // At t=0 the profile reports the actual (nonzero) start velocity,
        // unlike a plain new() move.
        assert!(approx(p.sample(0.0).position, 0.0));
        assert!(approx(p.sample(0.0).velocity, 20.0));
        // End of phase 1: velocity reaches cruise_speed, position matches
        // the hand-derived accel distance.
        assert!(approx(p.sample(p.t_accel).velocity, 50.0));
        assert!(approx(p.sample(p.t_accel).position, 10.5));
        // Still arrives exactly at rest at the target.
        let at_end = p.sample(p.duration());
        assert!(approx(at_end.position, 200.0));
        assert!(approx(at_end.velocity, 0.0));
        // No prefix needed for this case.
        assert!(approx(p.t_prefix_end, 0.0));
    }

    #[test]
    fn same_direction_with_room_stays_triangular_below_max_speed() {
        // v0=10, distance=15, max_speed=50, accel=decel=100: reaching
        // max_speed would need d_accel+d_decel = 12+12.5 = 24.5 > 15, so
        // triangular. v_peak = sqrt((2*a*d*dist + v0^2*d)/(a+d))
        //                     = sqrt((300000+10000)/200) = sqrt(1550).
        let p = TrapezoidalProfile::new_with_start_velocity(0.0, 10.0, 15.0, 50.0, 100.0, 100.0)
            .unwrap();
        let expected_peak = 1550.0_f64.sqrt();
        assert!(approx(p.cruise_speed, expected_peak));
        assert!(p.cruise_speed < 50.0);
        assert!(p.cruise_speed > 10.0);
        // No cruise phase.
        assert!(approx(p.t_accel, p.t_cruise_end));
        let at_end = p.sample(p.duration());
        assert!(approx(at_end.position, 15.0));
        assert!(approx(at_end.velocity, 0.0));
    }

    #[test]
    fn same_direction_faster_than_new_max_speed_decelerates_into_cruise() {
        // v0=80 exceeds the new move's max_speed=50: phase 1 must
        // decelerate into cruise, not accelerate. min_stop_distance =
        // 80^2/200 = 32 <= 500, so there's room.
        let p = TrapezoidalProfile::new_with_start_velocity(0.0, 80.0, 500.0, 50.0, 100.0, 100.0)
            .unwrap();
        assert!(approx(p.cruise_speed, 50.0));
        assert!(approx(p.t_prefix_end, 0.0)); // rides straight in, no prefix
        assert!(approx(p.sample(0.0).velocity, 80.0));
        // Phase 1 is losing speed (decelerating into cruise), not gaining.
        assert_eq!(p.phase_at(p.duration() * 0.01), MotionPhase::Decel);
        let mid_phase1 = p.sample(p.t_accel / 2.0);
        assert!(mid_phase1.velocity < 80.0 && mid_phase1.velocity > 50.0);
        // Once phase 1 completes, velocity never exceeds the new max_speed.
        let n = 1000;
        for i in 0..=n {
            let t = p.t_accel + (p.duration() - p.t_accel) * (i as f64) / (n as f64);
            assert!(
                p.sample(t).velocity <= 50.0 + 1e-6,
                "velocity {} exceeded max_speed at t={}",
                p.sample(t).velocity,
                t
            );
        }
        let at_end = p.sample(p.duration());
        assert!(approx(at_end.position, 500.0));
        assert!(approx(at_end.velocity, 0.0));
    }

    #[test]
    fn same_direction_without_room_overshoots_then_reverses() {
        // v0=50, end at 5: min_stop_distance = 2500/200 = 12.5 > 5, so no
        // way to stop at the target directly. Must decelerate to rest
        // (overshooting to 12.5) then come back.
        let p = TrapezoidalProfile::new_with_start_velocity(0.0, 50.0, 5.0, 50.0, 100.0, 100.0)
            .unwrap();
        assert!(approx(p.t_prefix_end, 0.5)); // 50/100
        let prefix_end = p.sample(p.t_prefix_end);
        assert!(approx(prefix_end.position, 12.5)); // overshoot past 5.0
        assert!(approx(prefix_end.velocity, 0.0));
        assert_eq!(p.phase_at(p.t_prefix_end / 2.0), MotionPhase::Decel);
        // The fresh segment afterward reverses direction (12.5 -> 5.0).
        let just_after = p.sample(p.t_prefix_end + 1e-6);
        assert!(just_after.velocity < 0.0);
        let at_end = p.sample(p.duration());
        assert!(approx(at_end.position, 5.0));
        assert!(approx(at_end.velocity, 0.0));
    }

    #[test]
    fn opposite_direction_decelerates_then_reverses() {
        // v0=-20 points away from end=100. Must decelerate to rest first
        // (landing at -2.0, further from `end` than `start`), then a
        // normal forward move.
        let p = TrapezoidalProfile::new_with_start_velocity(0.0, -20.0, 100.0, 50.0, 100.0, 100.0)
            .unwrap();
        assert!(approx(p.t_prefix_end, 0.2)); // 20/100
        let prefix_end = p.sample(p.t_prefix_end);
        assert!(approx(prefix_end.position, -2.0));
        assert!(approx(prefix_end.velocity, 0.0));
        assert!(approx(p.sample(0.0).velocity, -20.0));
        assert_eq!(p.phase_at(0.1), MotionPhase::Decel);
        let at_end = p.sample(p.duration());
        assert!(approx(at_end.position, 100.0));
        assert!(approx(at_end.velocity, 0.0));
    }

    #[test]
    fn start_velocity_position_is_continuous_across_all_boundaries() {
        // No jumps anywhere, including the prefix/main-segment seam, for a
        // profile that exercises overshoot-and-reverse.
        let p = TrapezoidalProfile::new_with_start_velocity(0.0, 50.0, 5.0, 50.0, 100.0, 100.0)
            .unwrap();
        for &boundary in &[p.t_prefix_end, p.t_accel, p.t_cruise_end] {
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
    fn rejects_non_finite_start_velocity() {
        match TrapezoidalProfile::new_with_start_velocity(0.0, f64::NAN, 100.0, 50.0, 100.0, 100.0)
        {
            Err(TrajectoryError::NonFiniteStartVelocity(v)) => assert!(v.is_nan()),
            other => panic!("expected NonFiniteStartVelocity, got {other:?}"),
        }
        assert_eq!(
            TrapezoidalProfile::new_with_start_velocity(
                0.0,
                f64::INFINITY,
                100.0,
                50.0,
                100.0,
                100.0,
            ),
            Err(TrajectoryError::NonFiniteStartVelocity(f64::INFINITY))
        );
    }
}
