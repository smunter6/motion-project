//! Adds time to a [`WaypointPath`]: a single scalar
//! [`TrapezoidalProfile`] over the path's total arc length drives
//! constant-progress motion along the whole route — exactly the "reuse the
//! existing scalar profile" pattern [`crate::LinearMove`] established for
//! the one-segment case, just composed with richer geometry underneath.

use crate::trajectory::{MotionPhase, TrajectoryError, TrapezoidalProfile};
use crate::waypoint_path::{SegmentKind, WaypointPath, WaypointPathError};
use crate::linear_move::MAX_GROUP_AXES;

/// Reasons a [`PathProfile`] could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathProfileError {
    /// The waypoint geometry itself was rejected.
    Path(WaypointPathError),
    /// The scalar speed profile over the path's total arc length was
    /// rejected (bad kinematic limits).
    Speed(TrajectoryError),
}

impl std::fmt::Display for PathProfileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PathProfileError::Path(e) => write!(f, "{e}"),
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
#[derive(Debug)]
pub struct PathProfile {
    path: WaypointPath,
    speed: TrapezoidalProfile,
}

impl PathProfile {
    /// Build a path move through `waypoints` (rest-to-rest: the axes start
    /// and end at rest), shaped per-segment by `segment_kinds`, at a single
    /// scalar `max_speed`/`max_acceleration`/`max_deceleration` applied to
    /// progress *along the path* (not to any individual axis).
    pub fn new(
        waypoints: Vec<Vec<f64>>,
        segment_kinds: Vec<SegmentKind>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
    ) -> Result<Self, PathProfileError> {
        let path = WaypointPath::new(waypoints, segment_kinds).map_err(PathProfileError::Path)?;
        let speed = TrapezoidalProfile::new(0.0, path.total_length(), max_speed, max_acceleration, max_deceleration)
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
            vec![SegmentKind::Spline, SegmentKind::Spline],
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
            vec![SegmentKind::Spline],
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
            vec![SegmentKind::Spline, SegmentKind::Spline, SegmentKind::Line],
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
            vec![SegmentKind::Spline, SegmentKind::Spline],
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
        match PathProfile::new(vec![vec![0.0], vec![1.0]], vec![SegmentKind::Spline], -1.0, 10.0, 10.0) {
            Err(PathProfileError::Speed(TrajectoryError::InvalidMaxSpeed(v))) => assert_eq!(v, -1.0),
            other => panic!("expected Speed(InvalidMaxSpeed), got {other:?}"),
        }
    }

    #[test]
    fn rejects_invalid_path_geometry() {
        match PathProfile::new(vec![vec![0.0]], vec![], 10.0, 10.0, 10.0) {
            Err(PathProfileError::Path(WaypointPathError::TooFewWaypoints(1))) => {}
            other => panic!("expected Path(TooFewWaypoints), got {other:?}"),
        }
    }
}
