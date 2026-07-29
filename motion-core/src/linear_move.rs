//! A straight-line move across several axes at once: "move this group from
//! here to there in a straight line" rather than "move this one axis."
//!
//! # Why this exists, and why it isn't `WaypointPath`/`PathProfile`
//!
//! A multi-axis straight-line move is a special case of general path
//! following: one segment, always a line, no blending. Rather than the
//! full spline/arc-length machinery a general path-follower needs, this is
//! the minimal thing that satisfies it: one scalar [`TrapezoidalProfile`]
//! over the Euclidean distance between the start and end points, composed
//! with a fixed unit direction vector.
//!
//! Same absolute-time model as `TrapezoidalProfile` — no dt-stepping state
//! here either, `sample(t)` is a pure function of elapsed time.
//!
//! [`TrapezoidalProfile`]: crate::trajectory::TrapezoidalProfile

use crate::jerk_filter::JerkFilteredProfile;
use crate::trajectory::{MotionPhase, TrajectoryError};

/// Upper bound on how many axes a single [`LinearMove`] can span. Exists so
/// the per-cycle sample type ([`LinearMoveSample`]) can be a fixed-size,
/// `Copy`, zero-allocation struct rather than heap-allocating every control
/// cycle. Revisit only if a real need for more than 6 coordinated axes
/// arises.
pub const MAX_GROUP_AXES: usize = 6;

/// Reasons a [`LinearMove`] could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LinearMoveError {
    /// `start` and `end` have different numbers of axes.
    DimensionMismatch { start_len: usize, end_len: usize },
    /// `start` and `start_velocity` have different numbers of axes.
    StartVelocityDimensionMismatch {
        start_len: usize,
        start_velocity_len: usize,
    },
    /// More axes than [`MAX_GROUP_AXES`] were requested.
    TooManyAxes(usize),
    /// A coordinate (in `start`, `end`, or `start_velocity`) is NaN or
    /// +/-infinity.
    NonFiniteCoordinate { axis_index: usize, value: f64 },
    /// The underlying scalar [`TrapezoidalProfile`] over path distance
    /// rejected the kinematic limits or the projected start velocity.
    ///
    /// [`TrapezoidalProfile`]: crate::trajectory::TrapezoidalProfile
    Speed(TrajectoryError),
}

impl std::fmt::Display for LinearMoveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LinearMoveError::DimensionMismatch { start_len, end_len } => write!(
                f,
                "start ({start_len} axes) and end ({end_len} axes) must have the same number of axes"
            ),
            LinearMoveError::StartVelocityDimensionMismatch {
                start_len,
                start_velocity_len,
            } => write!(
                f,
                "start ({start_len} axes) and start_velocity ({start_velocity_len} axes) must have the same number of axes"
            ),
            LinearMoveError::TooManyAxes(n) => {
                write!(f, "{n} axes exceeds the maximum of {MAX_GROUP_AXES}")
            }
            LinearMoveError::NonFiniteCoordinate { axis_index, value } => write!(
                f,
                "axis {axis_index}: coordinate must be a finite number (got {value})"
            ),
            LinearMoveError::Speed(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for LinearMoveError {}

/// A sample of a [`LinearMove`] at one instant: the feed-forward reference
/// for every participating axis this control cycle.
///
/// Fixed-size and `Copy` rather than `Vec`-returning: every group member
/// samples the same shared [`LinearMove`] independently every control
/// cycle, so this is a genuine per-cycle hot path — the first one in this
/// crate to span more than one axis, and it stays allocation-free the same
/// way every other profile type here does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinearMoveSample {
    positions: [f64; MAX_GROUP_AXES],
    velocities: [f64; MAX_GROUP_AXES],
    accelerations: [f64; MAX_GROUP_AXES],
    len: usize,
}

impl LinearMoveSample {
    /// Commanded position for each participating axis, in the same order
    /// `start`/`end` were given in.
    pub fn position(&self) -> &[f64] {
        &self.positions[..self.len]
    }

    /// Commanded velocity for each participating axis.
    pub fn velocity(&self) -> &[f64] {
        &self.velocities[..self.len]
    }

    /// Commanded acceleration for each participating axis.
    ///
    /// A straight line has no curvature, so this is purely the scalar
    /// path acceleration projected onto the same fixed unit direction the
    /// other two use — no centripetal term exists to miss.
    pub fn acceleration(&self) -> &[f64] {
        &self.accelerations[..self.len]
    }
}

/// A straight-line move across `N` axes (`2 <= N <= MAX_GROUP_AXES` in
/// practice, though nothing here requires more than one), driven by a
/// single scalar [`TrapezoidalProfile`] over the Euclidean distance between
/// `start` and `end`. Every axis's velocity is that one path velocity
/// projected onto a fixed unit direction vector — a straight line traversed
/// with synchronized speed, not independent per-axis profiles (which
/// wouldn't trace a straight line at all) or per-axis time-rescaling.
///
/// [`TrapezoidalProfile`]: crate::trajectory::TrapezoidalProfile
#[derive(Debug, Clone, PartialEq)]
pub struct LinearMove {
    start: [f64; MAX_GROUP_AXES],
    unit_direction: [f64; MAX_GROUP_AXES],
    len: usize,
    speed: JerkFilteredProfile,
}

impl LinearMove {
    /// Build a straight-line move from `start` to `end` (rest-to-rest),
    /// across as many axes as `start`/`end` have coordinates.
    ///
    /// A thin wrapper — `new_with_start_velocity` with an all-zero start
    /// velocity — exactly mirroring `TrapezoidalProfile::new`'s own
    /// relationship to `new_with_start_velocity`.
    pub fn new(
        start: Vec<f64>,
        end: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        max_jerk: Option<f64>,
    ) -> Result<Self, LinearMoveError> {
        let start_velocity = vec![0.0; start.len()];
        Self::new_with_start_velocity(
            start,
            start_velocity,
            end,
            max_speed,
            max_acceleration,
            max_deceleration,
            max_jerk,
        )
    }

    /// Build a straight-line move from `start` to `end`, starting from the
    /// group's actual per-axis `start_velocity` (backend feedback, not a
    /// commanded setpoint) rather than assuming rest.
    ///
    /// The N-dimensional `start_velocity` is projected onto the new line's
    /// unit direction via a dot product, reducing to the same scalar
    /// problem `TrapezoidalProfile::new_with_start_velocity` already solves.
    /// **Known limitation**: the component of `start_velocity` perpendicular
    /// to the new line is discarded, not reconciled — if the axes' actual
    /// velocity isn't already parallel to the new line, there's a genuine
    /// velocity discontinuity at `t = 0`. Accepted as a simplification for
    /// redirecting a moving axis group onto a new line; full reconciliation
    /// would need a curved/blended transition, not a bigger version of this
    /// type.
    pub fn new_with_start_velocity(
        start: Vec<f64>,
        start_velocity: Vec<f64>,
        end: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        max_jerk: Option<f64>,
    ) -> Result<Self, LinearMoveError> {
        if start.len() != end.len() {
            return Err(LinearMoveError::DimensionMismatch {
                start_len: start.len(),
                end_len: end.len(),
            });
        }
        if start.len() != start_velocity.len() {
            return Err(LinearMoveError::StartVelocityDimensionMismatch {
                start_len: start.len(),
                start_velocity_len: start_velocity.len(),
            });
        }
        let n = start.len();
        if n > MAX_GROUP_AXES {
            return Err(LinearMoveError::TooManyAxes(n));
        }
        for i in 0..n {
            if !start[i].is_finite() {
                return Err(LinearMoveError::NonFiniteCoordinate {
                    axis_index: i,
                    value: start[i],
                });
            }
            if !end[i].is_finite() {
                return Err(LinearMoveError::NonFiniteCoordinate {
                    axis_index: i,
                    value: end[i],
                });
            }
            if !start_velocity[i].is_finite() {
                return Err(LinearMoveError::NonFiniteCoordinate {
                    axis_index: i,
                    value: start_velocity[i],
                });
            }
        }

        let mut delta = [0.0; MAX_GROUP_AXES];
        let mut sum_sq = 0.0;
        for i in 0..n {
            delta[i] = end[i] - start[i];
            sum_sq += delta[i] * delta[i];
        }
        let length = sum_sq.sqrt();

        let mut unit_direction = [0.0; MAX_GROUP_AXES];
        if length > 0.0 {
            for i in 0..n {
                unit_direction[i] = delta[i] / length;
            }
        }

        // Component of start_velocity along the new line's direction — 0.0
        // whenever length == 0.0, since unit_direction is all-zero there.
        let mut v0_along = 0.0;
        for i in 0..n {
            v0_along += start_velocity[i] * unit_direction[i];
        }

        let speed = JerkFilteredProfile::new_with_start_velocity(
            0.0,
            v0_along,
            length,
            max_speed,
            max_acceleration,
            max_deceleration,
            max_jerk,
        )
        .map_err(LinearMoveError::Speed)?;

        let mut start_arr = [0.0; MAX_GROUP_AXES];
        start_arr[..n].copy_from_slice(&start[..n]);

        Ok(Self {
            start: start_arr,
            unit_direction,
            len: n,
            speed,
        })
    }

    /// Total duration of the move in seconds.
    pub fn duration(&self) -> f64 {
        self.speed.duration()
    }

    /// The commanded end position for each axis.
    pub fn target(&self) -> Vec<f64> {
        let end_distance = self.speed.target();
        (0..self.len)
            .map(|i| self.start[i] + self.unit_direction[i] * end_distance)
            .collect()
    }

    /// Which phase the move is in at absolute elapsed time `t` (seconds) —
    /// delegates directly to the underlying scalar profile, since every
    /// axis shares the same path-progress timing.
    pub fn phase_at(&self, t: f64) -> MotionPhase {
        self.speed.phase_at(t)
    }

    /// Sample every axis's position/velocity at absolute elapsed time `t`.
    pub fn sample(&self, t: f64) -> LinearMoveSample {
        let s = self.speed.sample(t);
        let mut positions = [0.0; MAX_GROUP_AXES];
        let mut velocities = [0.0; MAX_GROUP_AXES];
        let mut accelerations = [0.0; MAX_GROUP_AXES];
        for i in 0..self.len {
            positions[i] = self.start[i] + self.unit_direction[i] * s.position;
            velocities[i] = self.unit_direction[i] * s.velocity;
            accelerations[i] = self.unit_direction[i] * s.acceleration;
        }
        LinearMoveSample {
            positions,
            velocities,
            accelerations,
            len: self.len,
        }
    }

    /// How many axes this move spans.
    pub fn axis_count(&self) -> usize {
        self.len
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Several tests check a LinearMove against the bare scalar profile it
    // reduces to when unfiltered.
    use crate::trajectory::TrapezoidalProfile;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn linear_move_matches_hand_computed_2d_trapezoid() {
        // 3-4-5 triangle: start=(0,0), end=(3,4), length=5,
        // unit_direction=(0.6, 0.8) — hand-checkable.
        let scalar = TrapezoidalProfile::new(0.0, 5.0, 2.5, 5.0, 5.0).unwrap();
        let linear = LinearMove::new(vec![0.0, 0.0], vec![3.0, 4.0], 2.5, 5.0, 5.0, None).unwrap();
        assert!(approx(linear.duration(), scalar.duration()));

        let n = 200;
        for i in 0..=n {
            let t = linear.duration() * (i as f64) / (n as f64);
            let s = scalar.sample(t);
            let l = linear.sample(t);
            assert!(approx(l.position()[0], 0.6 * s.position));
            assert!(approx(l.position()[1], 0.8 * s.position));
            assert!(approx(l.velocity()[0], 0.6 * s.velocity));
            assert!(approx(l.velocity()[1], 0.8 * s.velocity));
        }
    }

    #[test]
    fn degenerate_same_point_start_end_is_inert() {
        let linear = LinearMove::new(vec![1.0, 2.0], vec![1.0, 2.0], 5.0, 5.0, 5.0, None).unwrap();
        assert!(approx(linear.duration(), 0.0));
        for &t in &[-1.0, 0.0, 1.0, 100.0] {
            let s = linear.sample(t);
            assert!(approx(s.position()[0], 1.0));
            assert!(approx(s.position()[1], 2.0));
            assert!(approx(s.velocity()[0], 0.0));
            assert!(approx(s.velocity()[1], 0.0));
        }
    }

    #[test]
    fn start_velocity_aligned_reduces_to_scalar_case() {
        // Straight line along +X; start velocity is purely along it too.
        let scalar =
            TrapezoidalProfile::new_with_start_velocity(0.0, 3.0, 10.0, 5.0, 10.0, 10.0).unwrap();
        let linear = LinearMove::new_with_start_velocity(
            vec![0.0, 0.0],
            vec![3.0, 0.0],
            vec![10.0, 0.0],
            5.0,
            10.0,
            10.0,
            None,
        )
        .unwrap();
        assert!(approx(linear.duration(), scalar.duration()));

        let n = 200;
        for i in 0..=n {
            let t = linear.duration() * (i as f64) / (n as f64);
            let s = scalar.sample(t);
            let l = linear.sample(t);
            assert!(approx(l.position()[0], s.position));
            assert!(approx(l.position()[1], 0.0));
            assert!(approx(l.velocity()[0], s.velocity));
            assert!(approx(l.velocity()[1], 0.0));
        }
    }

    #[test]
    fn start_velocity_perpendicular_component_is_dropped() {
        // Line along +X; start_velocity = (3, 5) has a parallel component
        // (3, preserved) and a perpendicular component (5, discarded) —
        // the direct proof of this type's documented velocity-discontinuity
        // limitation.
        let linear = LinearMove::new_with_start_velocity(
            vec![0.0, 0.0],
            vec![3.0, 5.0],
            vec![10.0, 0.0],
            5.0,
            10.0,
            10.0,
            None,
        )
        .unwrap();
        let at_start = linear.sample(0.0);
        assert!(approx(at_start.velocity()[0], 3.0));
        assert!(approx(at_start.velocity()[1], 0.0));
        // No NaN/panic anywhere across the whole move.
        let n = 200;
        for i in 0..=n {
            let t = linear.duration() * (i as f64) / (n as f64);
            let s = linear.sample(t);
            assert!(s.position().iter().all(|v| v.is_finite()));
            assert!(s.velocity().iter().all(|v| v.is_finite()));
        }
    }

    #[test]
    fn never_exceeds_max_speed_as_vector_norm() {
        let max_speed = 5.0;
        let linear = LinearMove::new(
            vec![0.0, 0.0, 0.0],
            vec![100.0, -50.0, 25.0],
            max_speed,
            10.0,
            10.0,
            None,
        )
        .unwrap();
        let n = 2000;
        for i in 0..=n {
            let t = linear.duration() * (i as f64) / (n as f64);
            let s = linear.sample(t);
            let norm: f64 = s.velocity().iter().map(|v| v * v).sum::<f64>().sqrt();
            assert!(
                norm <= max_speed + 1e-6,
                "velocity norm {norm} exceeded max_speed {max_speed} at t={t}"
            );
        }
    }

    #[test]
    fn dimension_mismatch_rejected() {
        assert_eq!(
            LinearMove::new(vec![0.0, 0.0], vec![1.0, 2.0, 3.0], 5.0, 5.0, 5.0, None),
            Err(LinearMoveError::DimensionMismatch {
                start_len: 2,
                end_len: 3
            })
        );
    }

    #[test]
    fn start_velocity_dimension_mismatch_rejected() {
        assert_eq!(
            LinearMove::new_with_start_velocity(
                vec![0.0, 0.0],
                vec![1.0, 2.0, 3.0],
                vec![5.0, 5.0],
                5.0,
                5.0,
                5.0,
                None
            ),
            Err(LinearMoveError::StartVelocityDimensionMismatch {
                start_len: 2,
                start_velocity_len: 3
            })
        );
    }

    #[test]
    fn too_many_axes_rejected() {
        let start = vec![0.0; MAX_GROUP_AXES + 1];
        let end = vec![1.0; MAX_GROUP_AXES + 1];
        assert_eq!(
            LinearMove::new(start, end, 5.0, 5.0, 5.0, None),
            Err(LinearMoveError::TooManyAxes(MAX_GROUP_AXES + 1))
        );
    }

    #[test]
    fn non_finite_coordinate_rejected() {
        match LinearMove::new(vec![0.0, 0.0], vec![1.0, f64::NAN], 5.0, 5.0, 5.0, None) {
            Err(LinearMoveError::NonFiniteCoordinate { axis_index, value }) => {
                assert_eq!(axis_index, 1);
                assert!(value.is_nan());
            }
            other => panic!("expected NonFiniteCoordinate, got {other:?}"),
        }
    }

    #[test]
    fn new_is_thin_wrapper_of_new_with_start_velocity() {
        let start = vec![1.0, 2.0];
        let end = vec![11.0, -3.0];
        let a = LinearMove::new(start.clone(), end.clone(), 5.0, 10.0, 8.0, None).unwrap();
        let b =
            LinearMove::new_with_start_velocity(start, vec![0.0, 0.0], end, 5.0, 10.0, 8.0, None)
                .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn position_is_continuous_across_all_boundaries() {
        // Force the overshoot/reverse prefix: moving fast in +X but the new
        // target is very close, not enough room to stop directly.
        let linear = LinearMove::new_with_start_velocity(
            vec![0.0, 0.0],
            vec![50.0, 0.0],
            vec![1.0, 0.0],
            50.0,
            100.0,
            100.0,
            None,
        )
        .unwrap();
        let n = 5000;
        let eps = linear.duration() / (n as f64) / 2.0;
        for i in 0..n {
            let t = linear.duration() * (i as f64) / (n as f64);
            let a = linear.sample(t);
            let b = linear.sample(t + eps);
            let jump: f64 = a
                .position()
                .iter()
                .zip(b.position().iter())
                .map(|(x, y)| (x - y).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(
                jump < 1.0,
                "position jumped by {jump} between t={t} and t={}",
                t + eps
            );
        }
    }
}
