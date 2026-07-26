//! Adds time to a [`WaypointPath`]: a single scalar
//! [`TrapezoidalProfile`] over the path's total arc length drives
//! constant-progress motion along the whole route — exactly the "reuse the
//! existing scalar profile" pattern [`crate::LinearMove`] established for
//! the one-segment case, just composed with richer geometry underneath.

use crate::trajectory::{MotionPhase, TrajectoryError, TrapezoidalProfile};
use crate::waypoint_path::{WaypointPath, WaypointPathError};
use crate::linear_move::MAX_GROUP_AXES;

/// Reasons a [`PathProfile`] could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathProfileError {
    /// The waypoint geometry itself was rejected.
    Path(WaypointPathError),
    /// `start_velocity` (passed to
    /// [`PathProfile::new_with_start_velocity`]) has a different number of
    /// axes than the waypoints.
    StartVelocityDimensionMismatch { expected: usize, got: usize },
    /// The scalar speed profile over the path's total arc length was
    /// rejected (bad kinematic limits, or a non-finite start velocity).
    Speed(TrajectoryError),
}

impl std::fmt::Display for PathProfileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PathProfileError::Path(e) => write!(f, "{e}"),
            PathProfileError::StartVelocityDimensionMismatch { expected, got } => write!(
                f,
                "start_velocity has {got} axes, expected {expected} (from the waypoints)"
            ),
            PathProfileError::Speed(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for PathProfileError {}

/// A sample of a [`PathProfile`] at one instant: the feed-forward reference
/// for every participating axis this control cycle. Fixed-size and `Copy`
/// rather than `Vec`-returning, same rationale as `LinearMoveSample` — this
/// is a genuine per-cycle hot path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathSample {
    positions: [f64; MAX_GROUP_AXES],
    velocities: [f64; MAX_GROUP_AXES],
    len: usize,
}

impl PathSample {
    /// Commanded position for each participating axis, in waypoint-coordinate order.
    pub fn position(&self) -> &[f64] {
        &self.positions[..self.len]
    }

    /// Commanded velocity for each participating axis.
    pub fn velocity(&self) -> &[f64] {
        &self.velocities[..self.len]
    }
}

/// A multi-waypoint move across `N` axes, driven by a single scalar
/// [`TrapezoidalProfile`] over the path's total arc length. Every axis's
/// velocity is that one path speed projected onto the path's local tangent
/// direction at the current arc length — constant progress along the route,
/// not independent per-axis profiles.
#[derive(Debug, PartialEq)]
pub struct PathProfile {
    path: WaypointPath,
    speed: TrapezoidalProfile,
}

impl PathProfile {
    /// Build a path move through `waypoints` (rest-to-rest: the axes start
    /// and end at rest), at a single
    /// scalar `max_speed`/`max_acceleration`/`max_deceleration` applied to
    /// progress *along the path* (not to any individual axis).
    ///
    /// A thin wrapper — `new_with_start_velocity` with an all-zero start
    /// velocity — exactly mirroring `TrapezoidalProfile::new`'s and
    /// `LinearMove::new`'s own relationship to their
    /// `new_with_start_velocity` counterparts.
    pub fn new(
        waypoints: Vec<Vec<f64>>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Result<Self, PathProfileError> {
        let start_velocity = vec![0.0; waypoints.first().map_or(0, |w| w.len())];
        Self::new_with_start_velocity(
            waypoints,
            start_velocity,
            max_speed,
            max_acceleration,
            max_deceleration,
        )
    }

    /// Build a path move through `waypoints`, starting from the group's
    /// actual `start_velocity` (backend feedback, an N-dimensional vector
    /// covering the same axes as `waypoints`) rather than assuming rest —
    /// what backs redirecting an in-flight group move onto a new path.
    ///
    /// `start_velocity` is projected onto the new path's own initial
    /// tangent direction via a dot product, reducing to the same scalar
    /// problem `TrapezoidalProfile::new_with_start_velocity` already
    /// solves — the same pattern `LinearMove::new_with_start_velocity`
    /// established for a straight line, with the path's local tangent
    /// standing in for a line's fixed unit direction. **Same known
    /// limitation**: the component of `start_velocity` perpendicular to
    /// that initial tangent is discarded, not reconciled.
    pub fn new_with_start_velocity(
        waypoints: Vec<Vec<f64>>,
        start_velocity: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Result<Self, PathProfileError> {
        let path = WaypointPath::new(waypoints).map_err(PathProfileError::Path)?;
        if start_velocity.len() != path.axis_count() {
            return Err(PathProfileError::StartVelocityDimensionMismatch {
                expected: path.axis_count(),
                got: start_velocity.len(),
            });
        }
        let tangent0 = path.tangent_at_arc_length(0.0);
        let mut v0_along = 0.0;
        for i in 0..path.axis_count() {
            v0_along += start_velocity[i] * tangent0[i];
        }
        let speed = TrapezoidalProfile::new_with_start_velocity(
            0.0,
            v0_along,
            path.total_length(),
            max_speed,
            max_acceleration,
            max_deceleration,
        )
        .map_err(PathProfileError::Speed)?;
        Ok(Self { path, speed })
    }

    /// Build a path move through `waypoints`, **blending** onto it from the
    /// group's actual `start_velocity` rather than redirecting onto it —
    /// smoother than `new_with_start_velocity`'s behavior.
    ///
    /// The difference is entirely in how the geometry is built: this uses
    /// [`WaypointPath::new_with_start_direction`] (with `start_velocity`
    /// itself as the direction) instead of plain [`WaypointPath::new`], so
    /// the new path's leading phantom control point — and its start tangent
    /// — already leans toward the incoming velocity direction, instead of
    /// assuming a straight approach toward the first real waypoint.
    /// `start_velocity` is then projected onto that (now closely-matching)
    /// tangent the same way `new_with_start_velocity` does, so the composed
    /// velocity at `t = 0` ends up close to the *full* `start_velocity`
    /// vector. **Still not an exact match** — see `WaypointPath`'s module
    /// docs on why a Catmull-Rom tangent can't be pinned exactly via
    /// phantom placement alone — but a close one.
    pub fn new_blended(
        waypoints: Vec<Vec<f64>>,
        start_velocity: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Result<Self, PathProfileError> {
        let path = WaypointPath::new_with_start_direction(waypoints, start_velocity.clone())
            .map_err(PathProfileError::Path)?;
        let tangent0 = path.tangent_at_arc_length(0.0);
        let mut v0_along = 0.0;
        for i in 0..path.axis_count() {
            v0_along += start_velocity[i] * tangent0[i];
        }
        let speed = TrapezoidalProfile::new_with_start_velocity(
            0.0,
            v0_along,
            path.total_length(),
            max_speed,
            max_acceleration,
            max_deceleration,
        )
        .map_err(PathProfileError::Speed)?;
        Ok(Self { path, speed })
    }

    /// How many axes this path spans.
    pub fn axis_count(&self) -> usize {
        self.path.axis_count()
    }

    /// Total duration of the move in seconds.
    pub fn duration(&self) -> f64 {
        self.speed.duration()
    }

    /// The commanded end position for each axis (the path's last waypoint).
    pub fn target(&self) -> Vec<f64> {
        self.path.position_at_arc_length(self.path.total_length())[..self.axis_count()].to_vec()
    }

    /// Which phase the move is in at absolute elapsed time `t` — delegates
    /// directly to the underlying scalar profile, since every axis shares
    /// the same path-progress timing.
    pub fn phase_at(&self, t: f64) -> MotionPhase {
        self.speed.phase_at(t)
    }

    /// Sample every axis's position/velocity at absolute elapsed time `t`.
    pub fn sample(&self, t: f64) -> PathSample {
        let s = self.speed.sample(t);
        let position = self.path.position_at_arc_length(s.position);
        let tangent = self.path.tangent_at_arc_length(s.position);
        let mut velocities = [0.0; MAX_GROUP_AXES];
        for i in 0..self.axis_count() {
            velocities[i] = tangent[i] * s.velocity;
        }
        PathSample {
            positions: position,
            velocities,
            len: self.axis_count(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::linear_move::LinearMove;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    fn approx_eps(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn starts_and_ends_at_waypoints() {
        let p = PathProfile::new(
            vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]],
            10.0,
            50.0,
            50.0,
        )
        .unwrap();
        let at_start = p.sample(0.0);
        let at_end = p.sample(p.duration());
        assert!(approx(at_start.position()[0], 0.0) && approx(at_start.position()[1], 0.0));
        assert!(approx(at_start.velocity()[0], 0.0) && approx(at_start.velocity()[1], 0.0));
        assert!(approx_eps(at_end.position()[0], 10.0, 1e-2) && approx_eps(at_end.position()[1], 10.0, 1e-2));
        assert!(approx(at_end.velocity()[0], 0.0) && approx(at_end.velocity()[1], 0.0));
        assert!(approx_eps(p.target()[0], 10.0, 1e-2) && approx_eps(p.target()[1], 10.0, 1e-2));
    }

    #[test]
    fn two_waypoint_path_matches_linear_move() {
        // A straight, 2-waypoint PathProfile and a LinearMove over the same
        // start/end/limits should trace essentially the same kinematics —
        // a cross-check against an already-trusted type.
        let start = vec![0.0, 0.0];
        let end = vec![30.0, 40.0];
        let (max_speed, max_accel, max_decel) = (10.0, 40.0, 40.0);
        let linear = LinearMove::new(start.clone(), end.clone(), max_speed, max_accel, max_decel).unwrap();
        let path = PathProfile::new(
            vec![start, end],
            max_speed,
            max_accel,
            max_decel,
        )
        .unwrap();
        assert!(approx_eps(path.duration(), linear.duration(), 1e-6));

        let n = 200;
        for i in 0..=n {
            let t = linear.duration() * (i as f64) / (n as f64);
            let l = linear.sample(t);
            let p = path.sample(t);
            assert!(
                approx_eps(l.position()[0], p.position()[0], 1e-2)
                    && approx_eps(l.position()[1], p.position()[1], 1e-2),
                "position mismatch at t={t}: linear {:?} vs path {:?}",
                l.position(),
                p.position()
            );
            assert!(
                approx_eps(l.velocity()[0], p.velocity()[0], 1e-2)
                    && approx_eps(l.velocity()[1], p.velocity()[1], 1e-2),
                "velocity mismatch at t={t}: linear {:?} vs path {:?}",
                l.velocity(),
                p.velocity()
            );
        }
    }

    #[test]
    fn velocity_norm_never_exceeds_max_speed() {
        let max_speed = 15.0;
        let p = PathProfile::new(
            vec![
                vec![0.0, 0.0, 0.0],
                vec![20.0, 10.0, 0.0],
                vec![25.0, 25.0, 10.0],
                vec![5.0, 30.0, 15.0],
            ],
            max_speed,
            30.0,
            30.0,
        )
        .unwrap();
        let n = 2000;
        for i in 0..=n {
            let t = p.duration() * (i as f64) / (n as f64);
            let s = p.sample(t);
            let norm: f64 = s.velocity().iter().map(|v| v * v).sum::<f64>().sqrt();
            assert!(norm <= max_speed + 1e-3, "velocity norm {norm} exceeded max_speed {max_speed} at t={t}");
        }
    }

    #[test]
    fn phase_at_matches_scalar_speed_profile() {
        let p = PathProfile::new(
            vec![vec![0.0, 0.0], vec![50.0, 0.0], vec![50.0, 50.0]],
            10.0,
            20.0,
            20.0,
        )
        .unwrap();
        assert_eq!(p.phase_at(-1.0), MotionPhase::Pre);
        assert_eq!(p.phase_at(p.duration() + 1.0), MotionPhase::Done);
    }

    #[test]
    fn rejects_invalid_speed_limits() {
        match PathProfile::new(vec![vec![0.0], vec![1.0]], -1.0, 10.0, 10.0) {
            Err(PathProfileError::Speed(TrajectoryError::InvalidMaxSpeed(v))) => assert_eq!(v, -1.0),
            other => panic!("expected Speed(InvalidMaxSpeed), got {other:?}"),
        }
    }

    #[test]
    fn rejects_invalid_path_geometry() {
        match PathProfile::new(vec![vec![0.0]], 10.0, 10.0, 10.0) {
            Err(PathProfileError::Path(WaypointPathError::TooFewWaypoints(1))) => {}
            other => panic!("expected Path(TooFewWaypoints), got {other:?}"),
        }
    }

    // --- new_with_start_velocity --------------------------------------

    #[test]
    fn new_is_thin_wrapper_of_new_with_start_velocity() {
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]];
        let a = PathProfile::new(waypoints.clone(), 10.0, 50.0, 50.0).unwrap();
        let b = PathProfile::new_with_start_velocity(
            waypoints,
            vec![0.0, 0.0],
            10.0,
            50.0,
            50.0,
        )
        .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn start_velocity_aligned_with_initial_tangent_reduces_to_scalar_case() {
        // Straight line along +X; start velocity is purely along it too, so
        // this should exactly match LinearMove's own start-velocity case.
        let start_velocity = vec![3.0, 0.0];
        let linear = LinearMove::new_with_start_velocity(
            vec![0.0, 0.0],
            start_velocity.clone(),
            vec![10.0, 0.0],
            5.0,
            10.0,
            10.0,
        )
        .unwrap();
        let path = PathProfile::new_with_start_velocity(
            vec![vec![0.0, 0.0], vec![10.0, 0.0]],
            start_velocity,
            5.0,
            10.0,
            10.0,
        )
        .unwrap();
        assert!(approx_eps(path.duration(), linear.duration(), 1e-6));

        let n = 200;
        for i in 0..=n {
            let t = linear.duration() * (i as f64) / (n as f64);
            let l = linear.sample(t);
            let p = path.sample(t);
            assert!(approx_eps(l.position()[0], p.position()[0], 1e-2));
            assert!(approx_eps(l.position()[1], p.position()[1], 1e-2));
            assert!(approx_eps(l.velocity()[0], p.velocity()[0], 1e-2));
            assert!(approx_eps(l.velocity()[1], p.velocity()[1], 1e-2));
        }
    }

    #[test]
    fn start_velocity_perpendicular_component_is_dropped() {
        // Path starts along +X; start_velocity = (3, 5) has a parallel
        // component (3, preserved) and a perpendicular component (5,
        // discarded) — same documented limitation LinearMove has.
        let path = PathProfile::new_with_start_velocity(
            vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]],
            vec![3.0, 5.0],
            10.0,
            50.0,
            50.0,
        )
        .unwrap();
        let at_start = path.sample(0.0);
        assert!(approx_eps(at_start.velocity()[0], 3.0, 1e-2));
        assert!(approx_eps(at_start.velocity()[1], 0.0, 1e-2));
        // No NaN/panic anywhere across the whole move.
        let n = 200;
        for i in 0..=n {
            let t = path.duration() * (i as f64) / (n as f64);
            let s = path.sample(t);
            assert!(s.position().iter().all(|v| v.is_finite()));
            assert!(s.velocity().iter().all(|v| v.is_finite()));
        }
    }

    #[test]
    fn rejects_start_velocity_dimension_mismatch() {
        assert_eq!(
            PathProfile::new_with_start_velocity(
                vec![vec![0.0, 0.0], vec![10.0, 0.0]],
                vec![1.0, 2.0, 3.0],
                    10.0,
                10.0,
                10.0,
            ),
            Err(PathProfileError::StartVelocityDimensionMismatch { expected: 2, got: 3 })
        );
    }

    // --- new_blended ------------------------------------------------------

    #[test]
    fn blended_velocity_is_continuous_across_a_real_transition() {
        // The discriminating test: build an "old" path move, sample its
        // actual velocity mid-flight, then blend a "new" path move from
        // that exact (position, velocity) state. The composed velocity
        // vector at the instant of transition should closely match the old
        // path's velocity at that instant — both direction *and*
        // magnitude, not just speed continuity. Realistic geometry: the new
        // path's first real waypoint continues roughly the same direction
        // the old path was already heading (a gentle bend, not a hairpin)
        // — the actual use case this mode is for, and where the phantom
        // approximation is tightest (see WaypointPath's module docs: the
        // approximation blends the given direction *with* the direction to
        // the next real waypoint, so it's tightest when those roughly
        // agree, loosest when they sharply disagree).
        let old = PathProfile::new(
            vec![vec![0.0, 0.0], vec![100.0, 0.0]],
            20.0,
            40.0,
            40.0,
        )
        .unwrap();
        let t_mid = old.duration() * 0.5; // well into cruise: velocity ~= (20, 0) exactly
        let at_transition = old.sample(t_mid);
        let start_position: Vec<f64> = at_transition.position().to_vec();
        let start_velocity: Vec<f64> = at_transition.velocity().to_vec();

        let new = PathProfile::new_blended(
            vec![start_position, vec![150.0, 10.0], vec![200.0, 50.0]],
            start_velocity.clone(),
            20.0,
            40.0,
            40.0,
        )
        .unwrap();
        let at_start = new.sample(0.0);

        let old_speed: f64 = start_velocity.iter().map(|v| v * v).sum::<f64>().sqrt();
        let new_speed: f64 = at_start.velocity().iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(
            approx_eps(old_speed, new_speed, old_speed * 0.05),
            "speed discontinuity: old {old_speed} vs new {new_speed}"
        );
        let dot: f64 = start_velocity
            .iter()
            .zip(at_start.velocity().iter())
            .map(|(a, b)| a * b)
            .sum();
        let cos_similarity = dot / (old_speed * new_speed);
        assert!(
            cos_similarity > 0.98,
            "direction discontinuity: cosine similarity {cos_similarity} (old {start_velocity:?}, new {:?})",
            at_start.velocity()
        );
    }

    #[test]
    fn blended_velocity_direction_still_reasonable_on_a_sharp_mismatch() {
        // The adversarial case: the new path's first real waypoint heads
        // somewhere quite different from the incoming velocity. Blend
        // isn't exact here (see the module docs), but it should still stay
        // sane — no NaN, no direction flip, no huge speed blow-up — a real
        // improvement over Aborting's outright-dropped perpendicular
        // component, just not a tight match.
        let start_velocity = vec![20.0, 0.0]; // heading +X
        let new = PathProfile::new_blended(
            vec![vec![0.0, 0.0], vec![50.0, 50.0], vec![0.0, 100.0]], // sharp turn toward +Y
            start_velocity.clone(),
            20.0,
            40.0,
            40.0,
        )
        .unwrap();
        let at_start = new.sample(0.0);
        assert!(at_start.velocity().iter().all(|v| v.is_finite()));
        let new_speed: f64 = at_start.velocity().iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!(new_speed <= 20.0 + 1e-6, "speed grew past the incoming magnitude: {new_speed}");
        // Still leans toward the incoming +X direction rather than
        // snapping straight to the new path's own +Y-ish heading.
        assert!(at_start.velocity()[0] > 0.0);
    }

    #[test]
    fn blended_reduces_to_reflection_when_velocity_is_zero() {
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]];
        let plain = PathProfile::new(waypoints.clone(), 10.0, 50.0, 50.0).unwrap();
        let blended = PathProfile::new_blended(
            waypoints,
            vec![0.0, 0.0],
            10.0,
            50.0,
            50.0,
        )
        .unwrap();
        assert_eq!(plain, blended);
    }
}
