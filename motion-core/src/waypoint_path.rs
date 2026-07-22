//! Multi-waypoint path following, geometry only (no time — see
//! [`crate::path_profile`] for the scalar speed profile composed on top).
//!
//! # Why this exists, and why it isn't just more `LinearMove`
//!
//! [`crate::LinearMove`] is a straight line between two points. A real move
//! sequence — several waypoints visited in one continuous motion, corners
//! smoothed rather than stopped at — is a different, harder geometry
//! problem: `WaypointPath` is that problem's solution, parameterized by arc
//! length so a single scalar speed profile ([`crate::path_profile::PathProfile`])
//! can still drive it, exactly the same "`TrapezoidalProfile` over a scalar
//! distance, composed with the path's geometry" pattern `LinearMove` already
//! established for the one-segment case.
//!
//! # Spline choice: centripetal Catmull-Rom
//!
//! Each interior segment is a cubic between its two endpoint waypoints,
//! shaped by the two neighboring waypoints — **local support**: a segment's
//! shape depends only on its four nearest control points, not the whole
//! path. That locality is what makes the per-segment [`SegmentKind::Line`]
//! override tractable (overriding one segment doesn't perturb any other
//! segment's curvature the way it would with a globally-solved natural
//! spline). **Centripetal** parameterization (knot spacing proportional to
//! `distance^0.5`, not `distance^0` (uniform) or `distance^1` (chordal))
//! avoids cusps and self-intersections on non-uniformly-spaced waypoints
//! (Yuksel, Schaefer, Keyser 2011).
//!
//! The first and last waypoints have no real neighbor on one side, so each
//! gets a **reflected phantom** control point (`P_-1 = 2*P0 - P1`, not a
//! duplicate of `P0` — duplication would zero-length that phantom segment
//! and divide by zero in the centripetal knot formula). A reflected phantom
//! also has a useful property: for a 2-waypoint path, the four "control
//! points" (phantom, P0, P1, phantom) are exactly colinear, so the spline
//! degenerates to precisely the straight line between them — the sanity
//! check this module's tests lean on.
//!
//! # Blending onto an in-flight path (`new_with_start_direction`)
//!
//! The reflected phantom is really just an *assumption*: "the incoming
//! direction was a straight-line approach toward P1" — reasonable when a
//! path starts from rest, wrong when it's replacing a move that was already
//! in motion. [`WaypointPath::new_with_start_direction`] swaps that
//! assumption for the truth when one is known: the leading phantom is
//! placed behind `P0` along the given direction instead of via reflection,
//! at the same characteristic distance (`|P0-P1|`) the reflection would
//! have used — same phantom-point *slot*, same Catmull-Rom evaluation, same
//! knot-spacing math, just a different formula for what fills it. This is
//! *not* an exact tangent match (a Catmull-Rom tangent at `P0` is a blend
//! of the phantom *and* `P1`, not the phantom's direction alone) — a small,
//! accepted approximation, deliberately consistent with the rest of this
//! module's tradeoffs (see `tangent_at_arc_length`'s own docs) rather than
//! a mathematically exact alternative (e.g. an explicit Hermite tangent).
//! An all-zero (or omitted) direction falls back to the ordinary reflected
//! phantom exactly as before — nothing to blend from if the incoming
//! velocity was zero.
//!
//! # Arc length has no closed form for a cubic
//!
//! so each segment gets its own lookup table, built once at construction:
//! ~200 evenly-spaced parameter samples, walked to accumulate true Euclidean
//! distance. Querying a position at a given arc length uses the LUT only to
//! *locate and refine* a parameter estimate — the returned position always
//! comes from evaluating the exact curve at that parameter, never from
//! interpolating stored LUT positions.
//!
//! # Known, accepted v1 limitation
//!
//! At a `Line`/`Spline` segment boundary, the spline side's neighbor-derived
//! tangent may not exactly match the line's fixed direction — a small
//! velocity-*direction* discontinuity there (velocity *magnitude* stays
//! continuous, since it comes from the shared scalar speed profile in
//! `PathProfile`, not from this module). Not fixed here; a future version
//! could make the adjacent spline tangent formula use the line's fixed
//! direction instead of the neighbor-derived one.

use crate::linear_move::MAX_GROUP_AXES;

/// How one segment between two consecutive waypoints is shaped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentKind {
    /// Smoothly blended via the centripetal Catmull-Rom spline (velocity
    /// direction continuous through both endpoint waypoints, assuming its
    /// neighboring segments are also splines).
    Spline,
    /// Forced straight, ignoring neighboring waypoints entirely — mirrors
    /// real PLCopen/CNC path composition, where a specific leg can be
    /// pinned to a line even inside an otherwise-smoothed path.
    Line,
}

/// Reasons a [`WaypointPath`] could not be constructed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WaypointPathError {
    /// Fewer than 2 waypoints — no path to follow.
    TooFewWaypoints(usize),
    /// A waypoint has a different number of coordinates than the first one.
    DimensionMismatch {
        waypoint_index: usize,
        expected: usize,
        got: usize,
    },
    /// More axes than [`MAX_GROUP_AXES`] were requested.
    TooManyAxes(usize),
    /// `segment_kinds.len()` must be exactly `waypoints.len() - 1`.
    SegmentKindCountMismatch { waypoints: usize, segment_kinds: usize },
    /// A coordinate is NaN or +/-infinity.
    NonFiniteCoordinate {
        waypoint_index: usize,
        axis_index: usize,
        value: f64,
    },
    /// Two consecutive waypoints are identical — a zero-length segment,
    /// which both the centripetal knot formula and a `Line` segment can't
    /// represent meaningfully.
    CoincidentWaypoints { segment_index: usize },
    /// `start_direction` (passed to
    /// [`WaypointPath::new_with_start_direction`]) has a different number
    /// of axes than the waypoints.
    StartDirectionDimensionMismatch { expected: usize, got: usize },
    /// A `start_direction` coordinate is NaN or +/-infinity.
    NonFiniteStartDirection { axis_index: usize, value: f64 },
}

impl std::fmt::Display for WaypointPathError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WaypointPathError::TooFewWaypoints(n) => {
                write!(f, "a path needs at least 2 waypoints, got {n}")
            }
            WaypointPathError::DimensionMismatch {
                waypoint_index,
                expected,
                got,
            } => write!(
                f,
                "waypoint {waypoint_index} has {got} axes, expected {expected} (from waypoint 0)"
            ),
            WaypointPathError::TooManyAxes(n) => {
                write!(f, "{n} axes exceeds the maximum of {MAX_GROUP_AXES}")
            }
            WaypointPathError::SegmentKindCountMismatch {
                waypoints,
                segment_kinds,
            } => write!(
                f,
                "{waypoints} waypoints need {} segment kinds, got {segment_kinds}",
                waypoints.saturating_sub(1)
            ),
            WaypointPathError::NonFiniteCoordinate {
                waypoint_index,
                axis_index,
                value,
            } => write!(
                f,
                "waypoint {waypoint_index}, axis {axis_index}: coordinate must be a finite number (got {value})"
            ),
            WaypointPathError::CoincidentWaypoints { segment_index } => write!(
                f,
                "waypoints {segment_index} and {} are identical (zero-length segment)",
                segment_index + 1
            ),
            WaypointPathError::StartDirectionDimensionMismatch { expected, got } => write!(
                f,
                "start_direction has {got} axes, expected {expected} (from the waypoints)"
            ),
            WaypointPathError::NonFiniteStartDirection { axis_index, value } => write!(
                f,
                "start_direction axis {axis_index}: must be a finite number (got {value})"
            ),
        }
    }
}

impl std::error::Error for WaypointPathError {}

/// How many parameter samples each segment's arc-length lookup table gets.
/// A one-time construction cost (not a per-cycle hot path — that's
/// `position_at_arc_length`/`tangent_at_arc_length`, which are O(log n)
/// lookups against this table), so generous accuracy is cheap.
const LUT_SAMPLES_PER_SEGMENT: usize = 200;

type Point = [f64; MAX_GROUP_AXES];

fn dist(a: &Point, b: &Point, axes: usize) -> f64 {
    let mut sum_sq = 0.0;
    for i in 0..axes {
        let d = a[i] - b[i];
        sum_sq += d * d;
    }
    sum_sq.sqrt()
}

fn reflect(p: &Point, q: &Point, axes: usize) -> Point {
    // 2*p - q: the phantom point that continues the p->q direction
    // backwards from p, by the same distance.
    let mut out = [0.0; MAX_GROUP_AXES];
    for i in 0..axes {
        out[i] = 2.0 * p[i] - q[i];
    }
    out
}

/// Affine combination of `pa`/`pb` at parameter `t`, given the knot values
/// `ta`/`tb` they sit at — the elementary operation the Barry-Goldman
/// Catmull-Rom recursion is built from.
fn lerp_param(pa: &Point, pb: &Point, ta: f64, tb: f64, t: f64, axes: usize) -> Point {
    let denom = tb - ta;
    let w = if denom.abs() < 1e-12 { 0.0 } else { (t - ta) / denom };
    let mut out = [0.0; MAX_GROUP_AXES];
    for i in 0..axes {
        out[i] = pa[i] + w * (pb[i] - pa[i]);
    }
    out
}

/// One arc-length lookup table entry, local to a segment: `u` (the
/// segment-local parameter, `0.0..=1.0`) paired with the true Euclidean
/// distance traveled from the segment's start to reach it.
#[derive(Debug, PartialEq)]
struct LutEntry {
    u: f64,
    local_arc_length: f64,
}

#[derive(Debug, PartialEq)]
struct Segment {
    kind: SegmentKind,
    /// Catmull-Rom control points (only meaningful for `Spline`; `p1`/`p2`
    /// are always this segment's actual start/end waypoint for both kinds,
    /// so `Line` reuses them directly and leaves `p0`/`p3`/knots unused).
    p0: Point,
    p1: Point,
    p2: Point,
    p3: Point,
    t0: f64,
    t1: f64,
    t2: f64,
    t3: f64,
    lut: Vec<LutEntry>,
    length: f64,
}

/// Evaluate a segment's exact curve at segment-local parameter `u` (`0.0` at
/// its start waypoint, `1.0` at its end waypoint) — never approximated from
/// the LUT.
fn evaluate_segment(seg: &Segment, axes: usize, u: f64) -> Point {
    match seg.kind {
        SegmentKind::Line => lerp_param(&seg.p1, &seg.p2, 0.0, 1.0, u, axes),
        SegmentKind::Spline => {
            let t = seg.t1 + u * (seg.t2 - seg.t1);
            let a1 = lerp_param(&seg.p0, &seg.p1, seg.t0, seg.t1, t, axes);
            let a2 = lerp_param(&seg.p1, &seg.p2, seg.t1, seg.t2, t, axes);
            let a3 = lerp_param(&seg.p2, &seg.p3, seg.t2, seg.t3, t, axes);
            let b1 = lerp_param(&a1, &a2, seg.t0, seg.t2, t, axes);
            let b2 = lerp_param(&a2, &a3, seg.t1, seg.t3, t, axes);
            lerp_param(&b1, &b2, seg.t1, seg.t2, t, axes)
        }
    }
}

/// A smooth (or explicitly straight, per segment) route through several
/// N-dimensional waypoints — see the module docs for the spline choice and
/// arc-length approach. Geometry only: [`crate::path_profile::PathProfile`]
/// composes this with a scalar [`crate::TrapezoidalProfile`] over arc length
/// to add time.
#[derive(Debug, PartialEq)]
pub struct WaypointPath {
    axes: usize,
    segments: Vec<Segment>,
    /// Cumulative global arc length at each waypoint (`len() == waypoints.len()`,
    /// `[0] == 0.0`, last entry `== total_length()`).
    waypoint_arc_lengths: Vec<f64>,
    total_length: f64,
}

impl WaypointPath {
    /// Build a path through `waypoints` (each an N-coordinate point, all the
    /// same N), with `segment_kinds[i]` governing the segment from
    /// `waypoints[i]` to `waypoints[i+1]` (so `segment_kinds.len()` must be
    /// exactly `waypoints.len() - 1`).
    ///
    /// A thin wrapper — `new_with_start_direction` with an all-zero
    /// direction — exactly mirroring this crate's other `new`/
    /// `new_with_start_velocity` pairs.
    pub fn new(
        waypoints: Vec<Vec<f64>>,
        segment_kinds: Vec<SegmentKind>,
    ) -> Result<Self, WaypointPathError> {
        let start_direction = vec![0.0; waypoints.first().map_or(0, |w| w.len())];
        Self::new_with_start_direction(waypoints, start_direction, segment_kinds)
    }

    /// Build a path the same way `new` does, except the path's very first
    /// phantom control point (see the module docs' "Blending" section) is
    /// placed along `start_direction` — the group's actual incoming
    /// velocity, typically — instead of via reflection, so the path's own
    /// start tangent leans toward matching an in-flight move it's
    /// replacing. An all-zero `start_direction` (including the all-zero
    /// vector `new` passes) falls back to the ordinary reflected phantom.
    pub fn new_with_start_direction(
        waypoints: Vec<Vec<f64>>,
        start_direction: Vec<f64>,
        segment_kinds: Vec<SegmentKind>,
    ) -> Result<Self, WaypointPathError> {
        let n = waypoints.len();
        if n < 2 {
            return Err(WaypointPathError::TooFewWaypoints(n));
        }
        let axes = waypoints[0].len();
        if axes > MAX_GROUP_AXES {
            return Err(WaypointPathError::TooManyAxes(axes));
        }
        for (i, wp) in waypoints.iter().enumerate() {
            if wp.len() != axes {
                return Err(WaypointPathError::DimensionMismatch {
                    waypoint_index: i,
                    expected: axes,
                    got: wp.len(),
                });
            }
            for (a, &v) in wp.iter().enumerate() {
                if !v.is_finite() {
                    return Err(WaypointPathError::NonFiniteCoordinate {
                        waypoint_index: i,
                        axis_index: a,
                        value: v,
                    });
                }
            }
        }
        let n_segments = n - 1;
        if segment_kinds.len() != n_segments {
            return Err(WaypointPathError::SegmentKindCountMismatch {
                waypoints: n,
                segment_kinds: segment_kinds.len(),
            });
        }
        if start_direction.len() != axes {
            return Err(WaypointPathError::StartDirectionDimensionMismatch {
                expected: axes,
                got: start_direction.len(),
            });
        }
        for (a, &v) in start_direction.iter().enumerate() {
            if !v.is_finite() {
                return Err(WaypointPathError::NonFiniteStartDirection {
                    axis_index: a,
                    value: v,
                });
            }
        }

        let wp_arr: Vec<Point> = waypoints
            .iter()
            .map(|w| {
                let mut a = [0.0; MAX_GROUP_AXES];
                a[..axes].copy_from_slice(w);
                a
            })
            .collect();

        for i in 0..n_segments {
            if dist(&wp_arr[i], &wp_arr[i + 1], axes) == 0.0 {
                return Err(WaypointPathError::CoincidentWaypoints { segment_index: i });
            }
        }

        // The leading phantom: placed along start_direction (at the same
        // characteristic distance a reflection would use) whenever a
        // nonzero direction was given, otherwise the ordinary reflected
        // phantom — see the module docs' "Blending" section.
        let mut dir_arr = [0.0; MAX_GROUP_AXES];
        dir_arr[..axes].copy_from_slice(&start_direction);
        let dir_norm: f64 = dir_arr[..axes].iter().map(|v| v * v).sum::<f64>().sqrt();
        let phantom_start = if dir_norm > 0.0 {
            let d = dist(&wp_arr[0], &wp_arr[1], axes);
            let mut p = [0.0; MAX_GROUP_AXES];
            for i in 0..axes {
                p[i] = wp_arr[0][i] - (dir_arr[i] / dir_norm) * d;
            }
            p
        } else {
            reflect(&wp_arr[0], &wp_arr[1], axes)
        };
        let phantom_end = reflect(&wp_arr[n - 1], &wp_arr[n - 2], axes);
        let mut extended: Vec<Point> = Vec::with_capacity(n + 2);
        extended.push(phantom_start);
        extended.extend_from_slice(&wp_arr);
        extended.push(phantom_end);

        let mut segments = Vec::with_capacity(n_segments);
        for (i, &kind) in segment_kinds.iter().enumerate() {
            let p0 = extended[i];
            let p1 = extended[i + 1];
            let p2 = extended[i + 2];
            let p3 = extended[i + 3];
            let (t0, t1, t2, t3) = match kind {
                SegmentKind::Spline => {
                    // Centripetal (alpha = 0.5) knot spacing. Reflected
                    // phantoms guarantee p0 != p1 and p2 != p3 whenever the
                    // real waypoints aren't coincident (already checked
                    // above), so these square roots are always of a
                    // positive number.
                    let t0 = 0.0;
                    let t1 = t0 + dist(&p0, &p1, axes).sqrt();
                    let t2 = t1 + dist(&p1, &p2, axes).sqrt();
                    let t3 = t2 + dist(&p2, &p3, axes).sqrt();
                    (t0, t1, t2, t3)
                }
                SegmentKind::Line => (0.0, 0.0, 1.0, 1.0), // unused
            };
            segments.push(Segment {
                kind,
                p0,
                p1,
                p2,
                p3,
                t0,
                t1,
                t2,
                t3,
                lut: Vec::new(),
                length: 0.0,
            });
        }

        let mut waypoint_arc_lengths = vec![0.0; n];
        let mut cum = 0.0;
        for (i, seg) in segments.iter_mut().enumerate() {
            let mut lut = Vec::with_capacity(LUT_SAMPLES_PER_SEGMENT + 1);
            lut.push(LutEntry {
                u: 0.0,
                local_arc_length: 0.0,
            });
            let mut prev = evaluate_segment(seg, axes, 0.0);
            let mut local_cum = 0.0;
            for k in 1..=LUT_SAMPLES_PER_SEGMENT {
                let u = k as f64 / LUT_SAMPLES_PER_SEGMENT as f64;
                let pos = evaluate_segment(seg, axes, u);
                local_cum += dist(&prev, &pos, axes);
                lut.push(LutEntry {
                    u,
                    local_arc_length: local_cum,
                });
                prev = pos;
            }
            seg.length = local_cum;
            seg.lut = lut;
            cum += local_cum;
            waypoint_arc_lengths[i + 1] = cum;
        }

        Ok(Self {
            axes,
            segments,
            waypoint_arc_lengths,
            total_length: cum,
        })
    }

    /// How many axes this path spans.
    pub fn axis_count(&self) -> usize {
        self.axes
    }

    /// Total path length (arc length of the whole route, following every
    /// segment's actual curve — a spline segment's length generally exceeds
    /// the straight-line distance between its endpoints).
    pub fn total_length(&self) -> f64 {
        self.total_length
    }

    /// Cumulative arc length at each waypoint, in waypoint order
    /// (`waypoint_arc_lengths()[0] == 0.0`, last entry `== total_length()`).
    /// Falls out of the LUT build for free.
    pub fn waypoint_arc_lengths(&self) -> &[f64] {
        &self.waypoint_arc_lengths
    }

    /// Locate segment-local (`segment_index`, `u`) for global arc length
    /// `s` (clamped to `[0, total_length()]`), refining `u` via the
    /// segment's own LUT.
    fn locate(&self, s: f64) -> (usize, f64) {
        let s = s.clamp(0.0, self.total_length);
        let seg_idx = self
            .waypoint_arc_lengths
            .partition_point(|&wal| wal <= s)
            .saturating_sub(1)
            .min(self.segments.len() - 1);
        let local_s = (s - self.waypoint_arc_lengths[seg_idx]).max(0.0);
        let lut = &self.segments[seg_idx].lut;
        let idx = lut.partition_point(|e| e.local_arc_length < local_s);
        let u = if idx == 0 {
            lut[0].u
        } else if idx >= lut.len() {
            lut[lut.len() - 1].u
        } else {
            let lo = &lut[idx - 1];
            let hi = &lut[idx];
            let span = hi.local_arc_length - lo.local_arc_length;
            let w = if span < 1e-12 {
                0.0
            } else {
                (local_s - lo.local_arc_length) / span
            };
            lo.u + w * (hi.u - lo.u)
        };
        (seg_idx, u)
    }

    /// Position at global arc length `s` along the path (clamped to
    /// `[0, total_length()]`). Always evaluated exactly at the LUT-refined
    /// parameter — never interpolated from stored LUT positions.
    pub fn position_at_arc_length(&self, s: f64) -> [f64; MAX_GROUP_AXES] {
        let (seg_idx, u) = self.locate(s);
        evaluate_segment(&self.segments[seg_idx], self.axes, u)
    }

    /// Unit tangent direction at global arc length `s`, via central finite
    /// difference in the arc-length domain (one-sided at the path's own
    /// endpoints, where the clamp in `position_at_arc_length` makes the
    /// naive central difference degrade gracefully rather than reach past
    /// the path). Not an analytic derivative of the Catmull-Rom curve — a
    /// deliberate simplification: arc-length parameterization already means
    /// `|dP/ds| ~= 1`, so a small-step finite difference plus a final
    /// re-normalize is accurate enough for the velocity feed-forward this
    /// exists for, without deriving/maintaining a separate closed-form
    /// tangent formula for both segment kinds.
    pub fn tangent_at_arc_length(&self, s: f64) -> [f64; MAX_GROUP_AXES] {
        let eps = (self.total_length * 1e-6).max(1e-9);
        let minus = self.position_at_arc_length(s - eps);
        let plus = self.position_at_arc_length(s + eps);
        let mut delta = [0.0; MAX_GROUP_AXES];
        let mut norm_sq = 0.0;
        for i in 0..self.axes {
            delta[i] = plus[i] - minus[i];
            norm_sq += delta[i] * delta[i];
        }
        let norm = norm_sq.sqrt();
        if norm > 0.0 {
            for v in delta.iter_mut().take(self.axes) {
                *v /= norm;
            }
        }
        delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    fn approx_eps(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    #[test]
    fn two_waypoint_path_is_exact_straight_line() {
        // Reflected phantoms make 4 colinear "control points" for a
        // 2-waypoint path, so the spline must degenerate to exactly the
        // straight line between them — the key sanity check for the whole
        // reflection scheme.
        let path = WaypointPath::new(
            vec![vec![0.0, 0.0], vec![30.0, 40.0]], // 3-4-5 triangle scaled by 10
            vec![SegmentKind::Spline],
        )
        .unwrap();
        assert!(approx(path.total_length(), 50.0));

        let n = 200;
        for i in 0..=n {
            let s = path.total_length() * (i as f64) / (n as f64);
            let pos = path.position_at_arc_length(s);
            let frac = s / 50.0;
            assert!(
                approx_eps(pos[0], 30.0 * frac, 1e-3),
                "x mismatch at s={s}: {} vs {}",
                pos[0],
                30.0 * frac
            );
            assert!(
                approx_eps(pos[1], 40.0 * frac, 1e-3),
                "y mismatch at s={s}: {} vs {}",
                pos[1],
                40.0 * frac
            );
        }
    }

    #[test]
    fn two_waypoint_path_tangent_is_constant_unit_direction() {
        let path =
            WaypointPath::new(vec![vec![0.0, 0.0], vec![30.0, 40.0]], vec![SegmentKind::Spline])
                .unwrap();
        let n = 50;
        for i in 1..n {
            // avoid the very ends where the one-sided difference is weakest
            let s = path.total_length() * (i as f64) / (n as f64);
            let tan = path.tangent_at_arc_length(s);
            assert!(approx_eps(tan[0], 0.6, 1e-3));
            assert!(approx_eps(tan[1], 0.8, 1e-3));
        }
    }

    #[test]
    fn waypoint_arc_lengths_hit_exact_waypoints() {
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0], vec![0.0, 10.0]];
        let path = WaypointPath::new(
            waypoints.clone(),
            vec![SegmentKind::Spline, SegmentKind::Spline, SegmentKind::Spline],
        )
        .unwrap();
        let arc_lengths = path.waypoint_arc_lengths().to_vec();
        assert_eq!(arc_lengths.len(), waypoints.len());
        assert!(approx(arc_lengths[0], 0.0));
        assert!(approx(arc_lengths[arc_lengths.len() - 1], path.total_length()));
        for (i, &s) in arc_lengths.iter().enumerate() {
            let pos = path.position_at_arc_length(s);
            assert!(
                approx_eps(pos[0], waypoints[i][0], 1e-2) && approx_eps(pos[1], waypoints[i][1], 1e-2),
                "waypoint {i} mismatch: got [{}, {}], expected {:?}",
                pos[0],
                pos[1],
                waypoints[i]
            );
        }
    }

    #[test]
    fn line_segment_override_is_exactly_linear() {
        // 3 waypoints, middle segment forced straight even though its
        // neighbor is a spline segment (local support means this doesn't
        // perturb the other segment).
        let path = WaypointPath::new(
            vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 100.0]],
            vec![SegmentKind::Line, SegmentKind::Spline],
        )
        .unwrap();
        let seg0_len = path.waypoint_arc_lengths()[1] - path.waypoint_arc_lengths()[0];
        assert!(approx(seg0_len, 10.0)); // straight segment length is exact

        let n = 100;
        for i in 0..=n {
            let s = seg0_len * (i as f64) / (n as f64);
            let pos = path.position_at_arc_length(s);
            assert!(approx_eps(pos[0], s, 1e-6));
            assert!(approx_eps(pos[1], 0.0, 1e-6));
        }
    }

    #[test]
    fn position_has_no_large_jumps_across_the_whole_path() {
        let path = WaypointPath::new(
            vec![
                vec![0.0, 0.0, 0.0],
                vec![10.0, 5.0, 0.0],
                vec![15.0, 15.0, 5.0],
                vec![5.0, 20.0, 10.0],
            ],
            vec![SegmentKind::Spline, SegmentKind::Spline, SegmentKind::Line],
        )
        .unwrap();
        let n = 2000;
        let step = path.total_length() / (n as f64);
        let mut prev = path.position_at_arc_length(0.0);
        for i in 1..=n {
            let s = step * (i as f64);
            let pos = path.position_at_arc_length(s);
            let d = dist(&prev, &pos, 3);
            assert!(
                d.is_finite() && d <= step * 1.5 + 1e-6,
                "jump of {d} at s={s} (step={step})"
            );
            prev = pos;
        }
    }

    #[test]
    fn rejects_too_few_waypoints() {
        assert_eq!(WaypointPath::new(vec![], vec![]).unwrap_err(), WaypointPathError::TooFewWaypoints(0));
        assert_eq!(
            WaypointPath::new(vec![vec![0.0, 0.0]], vec![]).unwrap_err(),
            WaypointPathError::TooFewWaypoints(1)
        );
    }

    #[test]
    fn rejects_dimension_mismatch() {
        assert_eq!(
            WaypointPath::new(vec![vec![0.0, 0.0], vec![1.0, 2.0, 3.0]], vec![SegmentKind::Spline])
                .unwrap_err(),
            WaypointPathError::DimensionMismatch {
                waypoint_index: 1,
                expected: 2,
                got: 3
            }
        );
    }

    #[test]
    fn rejects_segment_kind_count_mismatch() {
        assert_eq!(
            WaypointPath::new(
                vec![vec![0.0], vec![1.0], vec![2.0]],
                vec![SegmentKind::Spline]
            )
            .unwrap_err(),
            WaypointPathError::SegmentKindCountMismatch {
                waypoints: 3,
                segment_kinds: 1
            }
        );
    }

    #[test]
    fn rejects_coincident_adjacent_waypoints() {
        assert_eq!(
            WaypointPath::new(
                vec![vec![0.0, 0.0], vec![1.0, 1.0], vec![1.0, 1.0]],
                vec![SegmentKind::Spline, SegmentKind::Spline]
            )
            .unwrap_err(),
            WaypointPathError::CoincidentWaypoints { segment_index: 1 }
        );
    }

    #[test]
    fn rejects_non_finite_coordinate() {
        match WaypointPath::new(
            vec![vec![0.0, 0.0], vec![1.0, f64::NAN]],
            vec![SegmentKind::Spline],
        ) {
            Err(WaypointPathError::NonFiniteCoordinate {
                waypoint_index,
                axis_index,
                value,
            }) => {
                assert_eq!(waypoint_index, 1);
                assert_eq!(axis_index, 1);
                assert!(value.is_nan());
            }
            other => panic!("expected NonFiniteCoordinate, got {other:?}"),
        }
    }

    #[test]
    fn rejects_too_many_axes() {
        let axes = MAX_GROUP_AXES + 1;
        assert_eq!(
            WaypointPath::new(vec![vec![0.0; axes], vec![1.0; axes]], vec![SegmentKind::Spline])
                .unwrap_err(),
            WaypointPathError::TooManyAxes(axes)
        );
    }

    #[test]
    fn position_at_arc_length_clamps_outside_range() {
        let path =
            WaypointPath::new(vec![vec![0.0, 0.0], vec![10.0, 0.0]], vec![SegmentKind::Spline])
                .unwrap();
        let before = path.position_at_arc_length(-5.0);
        let start = path.position_at_arc_length(0.0);
        assert!(approx(before[0], start[0]) && approx(before[1], start[1]));
        let after = path.position_at_arc_length(path.total_length() + 5.0);
        let end = path.position_at_arc_length(path.total_length());
        assert!(approx(after[0], end[0]) && approx(after[1], end[1]));
    }

    // --- new_with_start_direction ("blend") -----------------------------

    #[test]
    fn new_is_thin_wrapper_of_new_with_start_direction() {
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]];
        let segment_kinds = vec![SegmentKind::Spline, SegmentKind::Spline];
        let a = WaypointPath::new(waypoints.clone(), segment_kinds.clone()).unwrap();
        let b = WaypointPath::new_with_start_direction(waypoints, vec![0.0, 0.0], segment_kinds)
            .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn start_direction_places_leading_phantom_along_given_direction() {
        // Direction (1, 0) at characteristic distance |P0-P1| = 5 should
        // place the phantom exactly 5 units in -X from P0 = (10, 20).
        let path = WaypointPath::new_with_start_direction(
            vec![vec![10.0, 20.0], vec![15.0, 20.0], vec![15.0, 30.0]],
            vec![1.0, 0.0],
            vec![SegmentKind::Spline, SegmentKind::Spline],
        )
        .unwrap();
        let phantom = path.segments[0].p0;
        assert!(approx(phantom[0], 5.0));
        assert!(approx(phantom[1], 20.0));
    }

    #[test]
    fn zero_start_direction_falls_back_to_reflection() {
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, 10.0]];
        let segment_kinds = vec![SegmentKind::Spline, SegmentKind::Spline];
        let reflected = WaypointPath::new(waypoints.clone(), segment_kinds.clone()).unwrap();
        let blended = WaypointPath::new_with_start_direction(waypoints, vec![0.0, 0.0], segment_kinds)
            .unwrap();
        assert_eq!(reflected, blended);
    }

    #[test]
    fn start_direction_tangent_is_close_to_given_direction() {
        // Not exact (see module docs), but should land much closer to the
        // requested direction than the plain reflected-phantom tangent
        // would for a path that turns sharply away from it.
        let waypoints = vec![vec![0.0, 0.0], vec![10.0, 0.0], vec![10.0, -10.0]];
        let segment_kinds = vec![SegmentKind::Spline, SegmentKind::Spline];
        // Incoming direction is +Y, even though the path immediately heads
        // toward +X then -Y — a deliberately sharp mismatch with the
        // default reflection (which would point along +X).
        let blended = WaypointPath::new_with_start_direction(
            waypoints,
            vec![0.0, 1.0],
            segment_kinds,
        )
        .unwrap();
        let tangent = blended.tangent_at_arc_length(0.0);
        // Cosine similarity to (0, 1) should be strongly positive (tangent
        // leans toward +Y), unlike the default reflection's tangent which
        // would point along +X (cosine ~= 0 against (0,1)).
        assert!(tangent[1] > 0.5, "tangent {tangent:?} doesn't lean toward the given direction");
    }

    #[test]
    fn rejects_start_direction_dimension_mismatch() {
        assert_eq!(
            WaypointPath::new_with_start_direction(
                vec![vec![0.0, 0.0], vec![1.0, 1.0]],
                vec![1.0, 2.0, 3.0],
                vec![SegmentKind::Spline],
            )
            .unwrap_err(),
            WaypointPathError::StartDirectionDimensionMismatch { expected: 2, got: 3 }
        );
    }

    #[test]
    fn rejects_non_finite_start_direction() {
        match WaypointPath::new_with_start_direction(
            vec![vec![0.0, 0.0], vec![1.0, 1.0]],
            vec![f64::NAN, 0.0],
            vec![SegmentKind::Spline],
        ) {
            Err(WaypointPathError::NonFiniteStartDirection { axis_index, value }) => {
                assert_eq!(axis_index, 0);
                assert!(value.is_nan());
            }
            other => panic!("expected NonFiniteStartDirection, got {other:?}"),
        }
    }
}
