//! Adds time to a [`WaypointPath`]: a single scalar profile over the path's
//! total arc length drives progress along the whole route, as
//! [`crate::LinearMove`] does for one straight segment.
//!
//! One scalar `max_speed`/`max_acceleration` applies over arc length.
//! Centripetal acceleration (`v²κ`) is not bounded.

use crate::jerk_filter::JerkFilteredProfile;
use crate::linear_move::MAX_GROUP_AXES;
use crate::trajectory::{MotionPhase, TrajectoryError};
use crate::waypoint_path::{WaypointPath, WaypointPathError};

/// Arc-length step for the central difference that gives `dT̂/ds` — the
/// curvature term of a path sample's acceleration. Small enough to resolve
/// the tightest corner these splines produce, large enough to avoid
/// cancellation in the tangent, which is itself a finite difference.
const CURVATURE_STEP: f64 = 1e-3;

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
/// for every participating axis this control cycle. Fixed-size and `Copy`,
/// like `LinearMoveSample`, so the per-cycle path is allocation-free.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathSample {
    positions: [f64; MAX_GROUP_AXES],
    velocities: [f64; MAX_GROUP_AXES],
    accelerations: [f64; MAX_GROUP_AXES],
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

    /// Commanded acceleration for each participating axis — tangential
    /// *and* centripetal. See `PathProfile::sample`.
    pub fn acceleration(&self) -> &[f64] {
        &self.accelerations[..self.len]
    }
}

/// A multi-waypoint move across `N` axes, driven by a single scalar profile
/// over the path's total arc length. Every axis's velocity is that path speed
/// projected onto the path's local tangent direction at the current arc
/// length.
#[derive(Debug, PartialEq)]
pub struct PathProfile {
    path: WaypointPath,
    speed: JerkFilteredProfile,
}

impl PathProfile {
    /// Build a path move through `waypoints` (rest-to-rest), with a single
    /// scalar `max_speed`/`max_acceleration`/`max_deceleration` applied to
    /// progress *along the path*, not to any individual axis.
    ///
    /// Equivalent to `new_with_start_velocity` with an all-zero start velocity.
    pub fn new(
        waypoints: Vec<Vec<f64>>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        max_jerk: Option<f64>,
    ) -> Result<Self, PathProfileError> {
        let start_velocity = vec![0.0; waypoints.first().map_or(0, |w| w.len())];
        Self::new_with_start_velocity(
            waypoints,
            start_velocity,
            max_speed,
            max_acceleration,
            max_deceleration,
            max_jerk,
        )
    }

    /// Build a path move through `waypoints`, starting from the group's
    /// `start_velocity` (an N-dimensional vector over the same axes as
    /// `waypoints`) rather than assuming rest. Used to redirect an in-flight
    /// group move onto a new path.
    ///
    /// `start_velocity` is projected onto the new path's initial tangent via a
    /// dot product, reducing to the scalar problem
    /// `TrapezoidalProfile::new_with_start_velocity` solves, as in
    /// `LinearMove::new_with_start_velocity`. **Limitation**: the component of
    /// `start_velocity` perpendicular to that tangent is discarded.
    pub fn new_with_start_velocity(
        waypoints: Vec<Vec<f64>>,
        start_velocity: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        max_jerk: Option<f64>,
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
        let speed = JerkFilteredProfile::new_with_start_velocity(
            0.0,
            v0_along,
            path.total_length(),
            max_speed,
            max_acceleration,
            max_deceleration,
            max_jerk,
        )
        .map_err(PathProfileError::Speed)?;
        Ok(Self { path, speed })
    }

    /// Build a path move through `waypoints`, **blending** onto it from the
    /// group's `start_velocity` rather than redirecting onto it.
    ///
    /// The difference from `new_with_start_velocity` is in the geometry: this
    /// uses [`WaypointPath::new_with_start_direction`] (with `start_velocity`
    /// as the direction) instead of [`WaypointPath::new`], so the path's
    /// leading phantom control point, and therefore its start tangent, leans
    /// toward the incoming velocity direction. `start_velocity` is then
    /// projected onto that tangent as in `new_with_start_velocity`, so the
    /// velocity at `t = 0` is close to the full `start_velocity` vector. It is
    /// not an exact match; see `WaypointPath`'s module docs.
    pub fn new_blended(
        waypoints: Vec<Vec<f64>>,
        start_velocity: Vec<f64>,
        max_speed: f64,
        max_acceleration: f64,
        max_deceleration: f64,
        max_jerk: Option<f64>,
    ) -> Result<Self, PathProfileError> {
        let path = WaypointPath::new_with_start_direction(waypoints, start_velocity.clone())
            .map_err(PathProfileError::Path)?;
        let tangent0 = path.tangent_at_arc_length(0.0);
        let mut v0_along = 0.0;
        for i in 0..path.axis_count() {
            v0_along += start_velocity[i] * tangent0[i];
        }
        let speed = JerkFilteredProfile::new_with_start_velocity(
            0.0,
            v0_along,
            path.total_length(),
            max_speed,
            max_acceleration,
            max_deceleration,
            max_jerk,
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

    /// Which phase the move is in at absolute elapsed time `t`. Every axis
    /// shares the scalar profile's timing.
    pub fn phase_at(&self, t: f64) -> MotionPhase {
        self.speed.phase_at(t)
    }

    /// Sample every axis's position/velocity at absolute elapsed time `t`.
    pub fn sample(&self, t: f64) -> PathSample {
        let s = self.speed.sample(t);
        let position = self.path.position_at_arc_length(s.position);
        let tangent = self.path.tangent_at_arc_length(s.position);

        // Acceleration along a *curved* path has two terms:
        //
        //     a = a_t · T̂  +  v² · dT̂/ds
        //
        // The first is the scalar profile's own acceleration along the
        // tangent. The second is centripetal: it points across the path and
        // exists even at constant speed. On a tight curve at speed it is
        // usually the larger of the two.
        //
        // `dT̂/ds` is a central difference on the tangent, which is itself a
        // central difference (see `tangent_at_arc_length`). |v² dT̂/ds| is the
        // lateral acceleration, which nothing bounds.
        let ds = CURVATURE_STEP.min(self.path.total_length().max(f64::EPSILON));
        let before = self.path.tangent_at_arc_length(s.position - ds);
        let after = self.path.tangent_at_arc_length(s.position + ds);

        let mut velocities = [0.0; MAX_GROUP_AXES];
        let mut accelerations = [0.0; MAX_GROUP_AXES];
        for i in 0..self.axis_count() {
            velocities[i] = tangent[i] * s.velocity;
            let dtangent_ds = (after[i] - before[i]) / (2.0 * ds);
            accelerations[i] = tangent[i] * s.acceleration + dtangent_ds * s.velocity * s.velocity;
        }
        PathSample {
            positions: position,
            velocities,
            accelerations,
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
            None,
        )
        .unwrap();
        let at_start = p.sample(0.0);
        let at_end = p.sample(p.duration());
        assert!(approx(at_start.position()[0], 0.0) && approx(at_start.position()[1], 0.0));
        assert!(approx(at_start.velocity()[0], 0.0) && approx(at_start.velocity()[1], 0.0));
        assert!(
            approx_eps(at_end.position()[0], 10.0, 1e-2)
                && approx_eps(at_end.position()[1], 10.0, 1e-2)
        );
        assert!(approx(at_end.velocity()[0], 0.0) && approx(at_end.velocity()[1], 0.0));
        assert!(approx_eps(p.target()[0], 10.0, 1e-2) && approx_eps(p.target()[1], 10.0, 1e-2));
    }

    #[test]
    fn two_waypoint_path_matches_linear_move() {
        // A straight, 2-waypoint PathProfile and a LinearMove over the same
        // start/end/limits should trace essentially the same kinematics.
        let start = vec![0.0, 0.0];
        let end = vec![30.0, 40.0];
        let (max_speed, max_accel, max_decel) = (10.0, 40.0, 40.0);
        let linear = LinearMove::new(
            start.clone(),
            end.clone(),
            max_speed,
            max_accel,
            max_decel,
            None,
        )
        .unwrap();
        let path =
            PathProfile::new(vec![start, end], max_speed, max_accel, max_decel, None).unwrap();
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
            None,
        )
        .unwrap();
        let n = 2000;
        for i in 0..=n {
            let t = p.duration() * (i as f64) / (n as f64);
            let s = p.sample(t);
            let norm: f64 = s.velocity().iter().map(|v| v * v).sum::<f64>().sqrt();
            assert!(
                norm <= max_speed + 1e-3,
                "velocity norm {norm} exceeded max_speed {max_speed} at t={t}"
            );
        }
    }

    #[test]
    fn phase_at_matches_scalar_speed_profile() {
        let p = PathProfile::new(
            vec![vec![0.0, 0.0], vec![50.0, 0.0], vec![50.0, 50.0]],
            10.0,
            20.0,
            20.0,
            None,
        )
        .unwrap();
        assert_eq!(p.phase_at(-1.0), MotionPhase::Pre);
        assert_eq!(p.phase_at(p.duration() + 1.0), MotionPhase::Done);
    }

    #[test]
    fn rejects_invalid_speed_limits() {
        match PathProfile::new(vec![vec![0.0], vec![1.0]], -1.0, 10.0, 10.0, None) {
            Err(PathProfileError::Speed(TrajectoryError::InvalidMaxSpeed(v))) => {
                assert_eq!(v, -1.0)
            }
            other => panic!("expected Speed(InvalidMaxSpeed), got {other:?}"),
        }
    }

    #[test]
    fn rejects_invalid_path_geometry() {
        match PathProfile::new(vec![vec![0.0]], 10.0, 10.0, 10.0, None) {
            Err(PathProfileError::Path(WaypointPathError::TooFewWaypoints(1))) => {}
            other => panic!("expected Path(TooFewWaypoints), got {other:?}"),
        }
    }

    // --- new_with_start_velocity --------------------------------------

    #[test]
    fn new_matches_new_with_start_velocity_given_zero_velocity() {
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]];
        let a = PathProfile::new(waypoints.clone(), 10.0, 50.0, 50.0, None).unwrap();
        let b =
            PathProfile::new_with_start_velocity(waypoints, vec![0.0, 0.0], 10.0, 50.0, 50.0, None)
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
            None,
        )
        .unwrap();
        let path = PathProfile::new_with_start_velocity(
            vec![vec![0.0, 0.0], vec![10.0, 0.0]],
            start_velocity,
            5.0,
            10.0,
            10.0,
            None,
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
        // discarded), as in LinearMove.
        let path = PathProfile::new_with_start_velocity(
            vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]],
            vec![3.0, 5.0],
            10.0,
            50.0,
            50.0,
            None,
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
                None,
            ),
            Err(PathProfileError::StartVelocityDimensionMismatch {
                expected: 2,
                got: 3
            })
        );
    }

    // --- new_blended ------------------------------------------------------

    #[test]
    fn blended_velocity_is_continuous_across_a_real_transition() {
        // Build an "old" path move, sample its velocity mid-flight, then blend
        // a "new" path move from that (position, velocity) state. The new
        // velocity vector at the transition should closely match the old one
        // in both direction and magnitude. The new path continues roughly the
        // same direction (a gentle bend, not a hairpin), where the phantom
        // approximation is tightest: it blends the given direction with the
        // direction to the next waypoint, so it is loosest when they disagree.
        let old = PathProfile::new(
            vec![vec![0.0, 0.0], vec![100.0, 0.0]],
            20.0,
            40.0,
            40.0,
            None,
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
            None,
        )
        .unwrap();
        let at_start = new.sample(0.0);

        let old_speed: f64 = start_velocity.iter().map(|v| v * v).sum::<f64>().sqrt();
        let new_speed: f64 = at_start
            .velocity()
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
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
        // The new path's first waypoint heads somewhere quite different from
        // the incoming velocity. Blend isn't exact here, but it must stay
        // sane: no NaN, no direction flip, no speed blow-up.
        let start_velocity = vec![20.0, 0.0]; // heading +X
        let new = PathProfile::new_blended(
            vec![vec![0.0, 0.0], vec![50.0, 50.0], vec![0.0, 100.0]], // sharp turn toward +Y
            start_velocity.clone(),
            20.0,
            40.0,
            40.0,
            None,
        )
        .unwrap();
        let at_start = new.sample(0.0);
        assert!(at_start.velocity().iter().all(|v| v.is_finite()));
        let new_speed: f64 = at_start
            .velocity()
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            .sqrt();
        assert!(
            new_speed <= 20.0 + 1e-6,
            "speed grew past the incoming magnitude: {new_speed}"
        );
        // Still leans toward the incoming +X direction rather than the new
        // path's +Y-ish heading.
        assert!(at_start.velocity()[0] > 0.0);
    }

    #[test]
    fn blended_reduces_to_reflection_when_velocity_is_zero() {
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]];
        let plain = PathProfile::new(waypoints.clone(), 10.0, 50.0, 50.0, None).unwrap();
        let blended =
            PathProfile::new_blended(waypoints, vec![0.0, 0.0], 10.0, 50.0, 50.0, None).unwrap();
        assert_eq!(plain, blended);
    }
}
