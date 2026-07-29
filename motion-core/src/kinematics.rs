//! The seam between *task space* (where a move is commanded — Cartesian
//! X/Y for now) and *joint space* (what an axis actually is — a linear
//! stage, a rotary joint).
//!
//! # Why a trait, and why an identity instance
//!
//! Every group in `app` goes through this layer, including the ones whose
//! axes already *are* Cartesian coordinates. Those use
//! [`IdentityKinematics`], which is a pass-through in both directions. That
//! is deliberate: it means the move builders, the control loop and the
//! status/viz code have exactly one code path, never a
//! `if group.is_an_arm { .. } else { .. }` fork, and a Cartesian group is
//! provably unaffected by anything here (identity composed with identity is
//! a no-op).
//!
//! # Shape of the interface
//!
//! Forward kinematics (joint → task) is infallible: every joint pose puts
//! the tool *somewhere*. Inverse kinematics is not — a task-space point can
//! be outside the workspace, and the mapping can be locally degenerate.
//!
//! IK is also generally *not unique*: a 2-link arm reaches most points
//! elbow-up or elbow-down. Which solution to take arrives per call as an
//! opaque [`KinematicBranch`] token, obtained from [`resolve_branch`]. This
//! crate has no opinion on how long a caller holds one; `app` resolves it
//! once per move and holds it for that move's duration, which keeps IK a
//! pure function of task-space position for the whole move.
//!
//! [`resolve_branch`]: KinematicModel::resolve_branch

use crate::linear_move::MAX_GROUP_AXES;
use core::f64::consts::FRAC_PI_2;

/// A vector of N real values — a joint pose, a joint velocity, a Cartesian
/// point, or a Cartesian velocity. It is the same shape in every case, so
/// there is one type rather than four.
///
/// Fixed-size and `Copy`, exactly like [`LinearMoveSample`], and for the
/// same reason: these are built and consumed once per group per control
/// cycle. A `Vec<f64>`-returning trait method would quietly require a global
/// allocator, which this crate must not (see the crate docs).
///
/// [`LinearMoveSample`]: crate::linear_move::LinearMoveSample
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KinematicVector {
    values: [f64; MAX_GROUP_AXES],
    len: usize,
}

impl KinematicVector {
    /// Build from a slice, rejecting an over-long or non-finite input.
    ///
    /// This is the only public constructor, so a `KinematicVector` in hand
    /// is always finite and within [`MAX_GROUP_AXES`] — the models below
    /// rely on that and do not re-check.
    pub fn from_slice(values: &[f64]) -> Result<Self, KinematicsError> {
        if values.len() > MAX_GROUP_AXES {
            return Err(KinematicsError::TooManyAxes(values.len()));
        }
        let mut out = [0.0; MAX_GROUP_AXES];
        for (index, &value) in values.iter().enumerate() {
            if !value.is_finite() {
                return Err(KinematicsError::NonFiniteValue { index, value });
            }
            out[index] = value;
        }
        Ok(Self {
            values: out,
            len: values.len(),
        })
    }

    /// The values, in the order they were given.
    pub fn as_slice(&self) -> &[f64] {
        &self.values[..self.len]
    }

    /// How many values this vector holds.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether this vector holds no values at all.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Internal constructor for values a model just computed. Skips the
    /// finiteness check that [`from_slice`] does, because model output is
    /// only ever non-finite if the model divided by something it had
    /// already checked — the fallible paths below check first.
    ///
    /// [`from_slice`]: KinematicVector::from_slice
    fn from_parts(values: [f64; MAX_GROUP_AXES], len: usize) -> Self {
        Self { values, len }
    }
}

/// Reasons a kinematic conversion could not be performed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KinematicsError {
    /// More values than [`MAX_GROUP_AXES`] were given.
    TooManyAxes(usize),
    /// A value is NaN or +/-infinity.
    NonFiniteValue { index: usize, value: f64 },
    /// A vector's length doesn't match the model's degrees of freedom.
    DimensionMismatch { expected: usize, got: usize },
    /// The requested task-space point lies outside the workspace: its
    /// distance `radius` from the base is not within `[min_radius,
    /// max_radius]`.
    Unreachable {
        radius: f64,
        min_radius: f64,
        max_radius: f64,
    },
    /// The pose is at (or near) a configuration where the Jacobian loses
    /// rank, so some task-space directions would demand unbounded joint
    /// rate. `measure` is the model's dimensionless degeneracy measure —
    /// for a 2-link arm, `sin(q2_eff)`.
    NearSingular { measure: f64 },
}

impl std::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KinematicsError::TooManyAxes(n) => {
                write!(f, "{n} axes exceeds the maximum of {MAX_GROUP_AXES}")
            }
            KinematicsError::NonFiniteValue { index, value } => write!(
                f,
                "element {index}: value must be a finite number (got {value})"
            ),
            KinematicsError::DimensionMismatch { expected, got } => {
                write!(f, "kinematic model expects {expected} values, got {got}")
            }
            KinematicsError::Unreachable {
                radius,
                min_radius,
                max_radius,
            } => write!(
                f,
                "target is unreachable: radius {radius:.3} is outside the workspace \
                 [{min_radius:.3}, {max_radius:.3}]"
            ),
            KinematicsError::NearSingular { measure } => write!(
                f,
                "pose is too close to a kinematic singularity (degeneracy measure {measure:.4})"
            ),
        }
    }
}

impl std::error::Error for KinematicsError {}

/// Upper bound on the points in a [`Linkage`]: one per joint, plus the
/// base.
pub const MAX_LINKAGE_POINTS: usize = MAX_GROUP_AXES + 1;

/// The physical arrangement of a mechanism at one pose: a polyline in task
/// space running from the base outward to the tool, with one point per
/// joint origin along the way.
///
/// This exists purely so a mechanism can be *drawn*. It is the one thing a
/// viewer needs that the rest of this interface can't provide —
/// [`forward_position`] gives the tool point, and nothing gives the elbow
/// in between. Deriving it outside the model would mean re-implementing the
/// model's own geometry next to it, which is exactly the duplication this
/// trait exists to prevent.
///
/// Fixed-size and `Copy`, like everything else here. A model with nothing
/// meaningful to draw returns [`Linkage::EMPTY`].
///
/// [`forward_position`]: KinematicModel::forward_position
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Linkage {
    points: [KinematicVector; MAX_LINKAGE_POINTS],
    len: usize,
}

impl Linkage {
    /// No drawable structure — what a model returns when its "mechanism"
    /// isn't a linkage at all.
    pub const EMPTY: Self = Self {
        points: [KinematicVector {
            values: [0.0; MAX_GROUP_AXES],
            len: 0,
        }; MAX_LINKAGE_POINTS],
        len: 0,
    };

    /// The polyline, base first, tool last.
    pub fn points(&self) -> &[KinematicVector] {
        &self.points[..self.len]
    }

    /// Whether there is anything to draw.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Internal constructor — same "values a model just computed" role as
    /// [`KinematicVector::from_parts`].
    fn from_parts(points: [KinematicVector; MAX_LINKAGE_POINTS], len: usize) -> Self {
        Self { points, len }
    }
}

/// An opaque, model-defined selector for one of the several joint poses
/// that reach the same task-space point.
///
/// Deliberately not an enum: "elbow up/down" is a SCARA concept, but the
/// value is stored and forwarded by model-agnostic code in `app`, and an
/// associated type would break `dyn` object safety. Callers treat this as a
/// token obtained from [`KinematicModel::resolve_branch`] and handed back to
/// [`KinematicModel::inverse_position`]; they never interpret the contents.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KinematicBranch(u8);

/// A mapping between joint space and task space.
///
/// Implementors are pure geometry: link lengths and trigonometry, no limits
/// and no units. Anything that needs to know a *speed* limit (joint-rate
/// checking, say) belongs to the caller, which is where the per-axis
/// configuration lives.
///
/// Every vector crossing this interface must have [`dof`] elements. That is
/// a caller-side invariant, debug-asserted rather than returned as an error
/// on the infallible forward methods; `app` checks each group's arity
/// against its model once at startup.
///
/// [`dof`]: KinematicModel::dof
pub trait KinematicModel {
    /// How many joints (equivalently, how many task-space coordinates) this
    /// model maps between. This design assumes the two are equal — no
    /// redundant or reduced-DOF models.
    fn dof(&self) -> usize;

    /// Joint pose → task-space point. Infallible: every pose is somewhere.
    fn forward_position(&self, joint: KinematicVector) -> KinematicVector;

    /// Joint velocity → task-space velocity at the given pose (`J · q̇`).
    /// Infallible: the Jacobian always exists, it just isn't always
    /// invertible, and this direction never inverts it.
    fn forward_velocity(
        &self,
        joint_position: KinematicVector,
        joint_velocity: KinematicVector,
    ) -> KinematicVector;

    /// Which IK branch the given joint pose is on. Infallible — every pose
    /// is on some branch.
    ///
    /// At a pose that is *itself* singular the branches coincide, so the
    /// answer there is arbitrary: either is an equally true description of
    /// the same pose, but it decides which way the arm breaks as it leaves.
    /// Not wrong, but not predictable either.
    fn resolve_branch(&self, joint: KinematicVector) -> KinematicBranch;

    /// Task-space point → joint pose, taking the solution on `branch`.
    fn inverse_position(
        &self,
        task: KinematicVector,
        branch: KinematicBranch,
    ) -> Result<KinematicVector, KinematicsError>;

    /// Task-space velocity → joint velocity at the given pose
    /// (`J⁻¹ · ẋ`).
    ///
    /// No branch argument: `joint_position` already pins the configuration,
    /// and the Jacobian is a property of the pose alone.
    fn inverse_velocity(
        &self,
        joint_position: KinematicVector,
        task_velocity: KinematicVector,
    ) -> Result<KinematicVector, KinematicsError>;

    /// Task-space acceleration → joint acceleration:
    ///
    /// ```text
    /// q̈ = J⁻¹ · (ẍ − J̇ · q̇)
    /// ```
    ///
    /// **The `J̇·q̇` term is why this can't be composed from
    /// [`inverse_velocity`].** A rotating linkage accelerates its own tool
    /// even at constant joint rates — that is the centripetal/Coriolis
    /// content of the mapping, and dropping it gives a feed-forward that is
    /// wrong exactly when it matters most, at speed on a curve.
    ///
    /// Fails on the same near-singular poses [`inverse_velocity`] does, and
    /// for the same reason: it inverts the same Jacobian.
    ///
    /// [`inverse_velocity`]: KinematicModel::inverse_velocity
    fn inverse_acceleration(
        &self,
        joint_position: KinematicVector,
        joint_velocity: KinematicVector,
        task_acceleration: KinematicVector,
    ) -> Result<KinematicVector, KinematicsError>;

    /// Where this mechanism physically *is* at the given pose, as a
    /// drawable polyline from base to tool — see [`Linkage`].
    ///
    /// Defaults to [`Linkage::EMPTY`], because most of what this trait does
    /// has nothing to do with drawing and a model shouldn't have to answer
    /// this to exist. Overriding it is how a mechanism becomes visible.
    fn linkage(&self, _joint: KinematicVector) -> Linkage {
        Linkage::EMPTY
    }
}

/// The pass-through model: joint values *are* task-space coordinates.
///
/// This is what a group of linear stages driving Cartesian X/Y uses. It
/// carries its own `dof` so that the arity check `app` performs on every
/// group means something here too, rather than being skipped for the one
/// model where it would be most easily got wrong.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IdentityKinematics {
    dof: usize,
}

impl IdentityKinematics {
    /// A pass-through model over `dof` axes.
    pub const fn new(dof: usize) -> Self {
        Self { dof }
    }
}

/// The only branch an unambiguous model has.
const SOLE_BRANCH: KinematicBranch = KinematicBranch(0);

impl KinematicModel for IdentityKinematics {
    fn dof(&self) -> usize {
        self.dof
    }

    fn forward_position(&self, joint: KinematicVector) -> KinematicVector {
        joint
    }

    fn forward_velocity(
        &self,
        _joint_position: KinematicVector,
        joint_velocity: KinematicVector,
    ) -> KinematicVector {
        joint_velocity
    }

    /// Identity has exactly one solution everywhere, so the token is a
    /// constant and [`inverse_position`] ignores it.
    ///
    /// [`inverse_position`]: KinematicModel::inverse_position
    fn resolve_branch(&self, _joint: KinematicVector) -> KinematicBranch {
        SOLE_BRANCH
    }

    fn inverse_position(
        &self,
        task: KinematicVector,
        _branch: KinematicBranch,
    ) -> Result<KinematicVector, KinematicsError> {
        check_dimension(self.dof, task)?;
        Ok(task)
    }

    fn inverse_velocity(
        &self,
        _joint_position: KinematicVector,
        task_velocity: KinematicVector,
    ) -> Result<KinematicVector, KinematicsError> {
        check_dimension(self.dof, task_velocity)?;
        Ok(task_velocity)
    }

    /// `J` is the identity, so `J̇` is zero and this is a pass-through too.
    fn inverse_acceleration(
        &self,
        _joint_position: KinematicVector,
        _joint_velocity: KinematicVector,
        task_acceleration: KinematicVector,
    ) -> Result<KinematicVector, KinematicsError> {
        check_dimension(self.dof, task_acceleration)?;
        Ok(task_acceleration)
    }
}

fn check_dimension(expected: usize, v: KinematicVector) -> Result<(), KinematicsError> {
    if v.len() != expected {
        return Err(KinematicsError::DimensionMismatch {
            expected,
            got: v.len(),
        });
    }
    Ok(())
}

/// Elbow-up: the branch with `sin(q2_eff) >= 0`.
const ELBOW_A: KinematicBranch = KinematicBranch(0);
/// Elbow-down: the branch with `sin(q2_eff) < 0`.
const ELBOW_B: KinematicBranch = KinematicBranch(1);

/// How close to straight (or fully folded) the elbow may get before
/// [`inverse_velocity`] refuses. Dimensionless — it is a bound on
/// `|sin(q2_eff)|`, i.e. the elbow is within `asin(0.05) ≈ 2.9°` of a
/// singular configuration.
///
/// Deliberately *not* a threshold on the determinant, which carries units
/// of length² and would therefore silently rescale with the link lengths.
///
/// [`inverse_velocity`]: KinematicModel::inverse_velocity
const SINGULARITY_EPSILON: f64 = 0.05;

/// A 2-link planar arm (SCARA), two rotary joints in a plane.
///
/// # The home-offset convention — read this before using raw joint values
///
/// Textbook form has `q2` measured from link 1, so `(q1, q2) = (0, 0)` is
/// the *fully extended* arm — which is exactly a singular pose, and exactly
/// where every axis in this system starts. To keep the natural start pose
/// safe, this model applies a fixed offset internally: `q2_eff = q2 + π/2`.
///
/// So **raw joint value `q2 = 0` means "elbow bent 90°", not "straight
/// arm"**, and `(0, 0)` is a comfortably non-singular pose. Everything the
/// outside world sees — feedback, setpoints, `status`, single-axis jogging
/// — is in raw `q2`; the offset exists only inside FK, IK and the Jacobian.
///
/// # Singularities
///
/// The arm is singular where `sin(q2_eff) = 0`: `q2_eff = 0` (straight out,
/// at the outer workspace boundary) and `q2_eff = π` (folded back, at
/// radius `|l1 - l2|`). With equal links that inner one sits at the
/// *origin*, which is reachable — a straight move from `(x, y)` to
/// `(-x, -y)` passes through it with both endpoints validating cleanly. It
/// is guarded reactively, by `inverse_velocity` returning
/// [`KinematicsError::NearSingular`], not by the geometry forbidding it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScaraKinematics {
    l1: f64,
    l2: f64,
}

impl ScaraKinematics {
    /// A 2-link arm with the given link lengths (inner link first).
    pub const fn new(l1: f64, l2: f64) -> Self {
        Self { l1, l2 }
    }

    /// The effective elbow angle: the raw joint value plus the home offset
    /// described in the type docs.
    fn q2_eff(q2: f64) -> f64 {
        q2 + FRAC_PI_2
    }

    /// The 2x2 Jacobian `[[j11, j12], [j21, j22]]` mapping joint rates to
    /// task-space velocity, plus `sin(q2_eff)` — the dimensionless
    /// degeneracy measure, and the determinant divided by `l1·l2`.
    fn jacobian(&self, q1: f64, q2: f64) -> ([[f64; 2]; 2], f64) {
        let q2_eff = Self::q2_eff(q2);
        let (s1, c1) = q1.sin_cos();
        let (s12, c12) = (q1 + q2_eff).sin_cos();
        let j = [
            [-self.l1 * s1 - self.l2 * s12, -self.l2 * s12],
            [self.l1 * c1 + self.l2 * c12, self.l2 * c12],
        ];
        (j, q2_eff.sin())
    }
}

impl KinematicModel for ScaraKinematics {
    fn dof(&self) -> usize {
        2
    }

    fn forward_position(&self, joint: KinematicVector) -> KinematicVector {
        debug_assert_eq!(joint.len(), 2, "ScaraKinematics is a 2-DOF model");
        let q = joint.as_slice();
        let (q1, q2_eff) = (q[0], Self::q2_eff(q[1]));
        let mut out = [0.0; MAX_GROUP_AXES];
        out[0] = self.l1 * q1.cos() + self.l2 * (q1 + q2_eff).cos();
        out[1] = self.l1 * q1.sin() + self.l2 * (q1 + q2_eff).sin();
        KinematicVector::from_parts(out, 2)
    }

    fn forward_velocity(
        &self,
        joint_position: KinematicVector,
        joint_velocity: KinematicVector,
    ) -> KinematicVector {
        debug_assert_eq!(joint_position.len(), 2, "ScaraKinematics is a 2-DOF model");
        debug_assert_eq!(joint_velocity.len(), 2, "ScaraKinematics is a 2-DOF model");
        let q = joint_position.as_slice();
        let dq = joint_velocity.as_slice();
        let (j, _) = self.jacobian(q[0], q[1]);
        let mut out = [0.0; MAX_GROUP_AXES];
        out[0] = j[0][0] * dq[0] + j[0][1] * dq[1];
        out[1] = j[1][0] * dq[0] + j[1][1] * dq[1];
        KinematicVector::from_parts(out, 2)
    }

    fn resolve_branch(&self, joint: KinematicVector) -> KinematicBranch {
        debug_assert_eq!(joint.len(), 2, "ScaraKinematics is a 2-DOF model");
        if Self::q2_eff(joint.as_slice()[1]).sin() >= 0.0 {
            ELBOW_A
        } else {
            ELBOW_B
        }
    }

    /// Law of cosines for the elbow, `atan2` for the shoulder.
    ///
    /// The returned `q2_eff` is in `[0, π]` on the elbow-up branch and
    /// `[-π, 0]` on elbow-down (the two values `resolve_branch` returns;
    /// which is which is deliberately not part of the public API), so
    /// `inverse_position(forward_position(q), resolve_branch(q))`
    /// recovers `q` exactly for any pose whose `q2_eff` is already in
    /// `[-π, π]`, and recovers an equivalent pose modulo 2π otherwise.
    fn inverse_position(
        &self,
        task: KinematicVector,
        branch: KinematicBranch,
    ) -> Result<KinematicVector, KinematicsError> {
        check_dimension(2, task)?;
        let p = task.as_slice();
        let (x, y) = (p[0], p[1]);
        let r2 = x * x + y * y;

        let mut cos_q2_eff =
            (r2 - self.l1 * self.l1 - self.l2 * self.l2) / (2.0 * self.l1 * self.l2);
        // Tolerate landing a hair outside [-1, 1] from round-off at the
        // workspace boundary; reject anything genuinely beyond it.
        const COS_TOLERANCE: f64 = 1e-9;
        if cos_q2_eff.abs() > 1.0 + COS_TOLERANCE {
            return Err(KinematicsError::Unreachable {
                radius: r2.sqrt(),
                min_radius: (self.l1 - self.l2).abs(),
                max_radius: self.l1 + self.l2,
            });
        }
        cos_q2_eff = cos_q2_eff.clamp(-1.0, 1.0);

        let q2_eff = if branch == ELBOW_B {
            -cos_q2_eff.acos()
        } else {
            cos_q2_eff.acos()
        };
        let q1 = y.atan2(x) - (self.l2 * q2_eff.sin()).atan2(self.l1 + self.l2 * cos_q2_eff);

        let mut out = [0.0; MAX_GROUP_AXES];
        out[0] = q1;
        out[1] = q2_eff - FRAC_PI_2;
        Ok(KinematicVector::from_parts(out, 2))
    }

    /// Inverts the 2x2 Jacobian.
    ///
    /// Refuses with [`KinematicsError::NearSingular`] when the elbow is
    /// within `asin(0.05) ≈ 2.9°` of straight or fully folded. The test is on
    /// `|sin(q2_eff)|`, not on the determinant: the determinant carries units
    /// of length² and an absolute epsilon on it would silently rescale with
    /// the link lengths.
    ///
    /// Note what this does *not* catch: a near-singular Jacobian loses rank
    /// in one direction only, so motion along the surviving direction is
    /// perfectly realizable. Testing the pose alone therefore rejects some
    /// commands that were fine, and — more importantly — accepts commands
    /// whose joint rates are already unrealizable while still short of the
    /// threshold. Bounding the joint rate itself needs per-axis limits,
    /// which is the caller's business, not this crate's.
    fn inverse_velocity(
        &self,
        joint_position: KinematicVector,
        task_velocity: KinematicVector,
    ) -> Result<KinematicVector, KinematicsError> {
        check_dimension(2, joint_position)?;
        check_dimension(2, task_velocity)?;
        let q = joint_position.as_slice();
        let v = task_velocity.as_slice();
        let (j, sin_q2_eff) = self.jacobian(q[0], q[1]);
        if sin_q2_eff.abs() < SINGULARITY_EPSILON {
            return Err(KinematicsError::NearSingular {
                measure: sin_q2_eff,
            });
        }
        let det = self.l1 * self.l2 * sin_q2_eff;
        let mut out = [0.0; MAX_GROUP_AXES];
        out[0] = (j[1][1] * v[0] - j[0][1] * v[1]) / det;
        out[1] = (-j[1][0] * v[0] + j[0][0] * v[1]) / det;
        Ok(KinematicVector::from_parts(out, 2))
    }

    /// Inverts the same Jacobian [`inverse_velocity`] does, after removing
    /// the `J̇·q̇` term — the acceleration the linkage produces on its own
    /// while rotating, even at constant joint rates.
    ///
    /// `J̇` is the entry-wise time derivative of the 2x2 Jacobian, which
    /// for this arm is elementary: every entry is a sine or cosine of `q1`
    /// or `q1 + q2_eff`, so differentiating brings down `q̇1` or
    /// `q̇1 + q̇2` and swaps the trig function.
    ///
    /// [`inverse_velocity`]: KinematicModel::inverse_velocity
    fn inverse_acceleration(
        &self,
        joint_position: KinematicVector,
        joint_velocity: KinematicVector,
        task_acceleration: KinematicVector,
    ) -> Result<KinematicVector, KinematicsError> {
        check_dimension(2, joint_position)?;
        check_dimension(2, joint_velocity)?;
        check_dimension(2, task_acceleration)?;
        let q = joint_position.as_slice();
        let dq = joint_velocity.as_slice();

        let q2_eff = Self::q2_eff(q[1]);
        let (s1, c1) = q[0].sin_cos();
        let (s12, c12) = (q[0] + q2_eff).sin_cos();
        // Rates of the two angles the trig terms actually depend on.
        let w1 = dq[0];
        let w12 = dq[0] + dq[1];

        let jdot = [
            [
                -self.l1 * c1 * w1 - self.l2 * c12 * w12,
                -self.l2 * c12 * w12,
            ],
            [
                -self.l1 * s1 * w1 - self.l2 * s12 * w12,
                -self.l2 * s12 * w12,
            ],
        ];
        let a = task_acceleration.as_slice();
        // ẍ − J̇·q̇, then the same inversion `inverse_velocity` performs.
        let mut residual = [0.0; MAX_GROUP_AXES];
        residual[0] = a[0] - (jdot[0][0] * dq[0] + jdot[0][1] * dq[1]);
        residual[1] = a[1] - (jdot[1][0] * dq[0] + jdot[1][1] * dq[1]);

        self.inverse_velocity(joint_position, KinematicVector::from_parts(residual, 2))
    }

    /// Three points: the base at the origin, the elbow at the end of link
    /// 1, and the tool at the end of link 2. The tool point is
    /// [`forward_position`] itself, so the drawn arm always ends exactly
    /// where the rest of the system thinks the TCP is — they cannot drift
    /// apart, because it is the same call.
    ///
    /// [`forward_position`]: KinematicModel::forward_position
    fn linkage(&self, joint: KinematicVector) -> Linkage {
        debug_assert_eq!(joint.len(), 2, "ScaraKinematics is a 2-DOF model");
        let q1 = joint.as_slice()[0];

        let mut elbow = [0.0; MAX_GROUP_AXES];
        elbow[0] = self.l1 * q1.cos();
        elbow[1] = self.l1 * q1.sin();

        let mut points =
            [KinematicVector::from_parts([0.0; MAX_GROUP_AXES], 2); MAX_LINKAGE_POINTS];
        points[1] = KinematicVector::from_parts(elbow, 2);
        points[2] = self.forward_position(joint);
        Linkage::from_parts(points, 3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    fn v(values: &[f64]) -> KinematicVector {
        KinematicVector::from_slice(values).unwrap()
    }

    // --- KinematicVector -------------------------------------------------

    #[test]
    fn vector_rejects_non_finite_and_over_long_input() {
        // NaN != NaN, so these check the variant and index rather than
        // comparing the error value.
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            match KinematicVector::from_slice(&[1.0, bad]) {
                Err(KinematicsError::NonFiniteValue { index, .. }) => assert_eq!(index, 1),
                other => panic!("expected NonFiniteValue, got {other:?}"),
            }
        }
        assert!(matches!(
            KinematicVector::from_slice(&[0.0; MAX_GROUP_AXES + 1]),
            Err(KinematicsError::TooManyAxes(_))
        ));
    }

    // --- IdentityKinematics ----------------------------------------------

    #[test]
    fn identity_round_trips_in_several_dimensions() {
        for dof in 1..=MAX_GROUP_AXES {
            let model = IdentityKinematics::new(dof);
            let joints: Vec<f64> = (0..dof).map(|i| i as f64 * 1.5 - 2.0).collect();
            let rates: Vec<f64> = (0..dof).map(|i| i as f64 * -0.25).collect();
            let task = model.forward_position(v(&joints));
            assert_eq!(task.as_slice(), joints.as_slice());

            let branch = model.resolve_branch(v(&joints));
            let back = model.inverse_position(task, branch).unwrap();
            assert_eq!(back.as_slice(), joints.as_slice());

            let task_v = model.forward_velocity(v(&joints), v(&rates));
            assert_eq!(task_v.as_slice(), rates.as_slice());
            let back_v = model.inverse_velocity(v(&joints), task_v).unwrap();
            assert_eq!(back_v.as_slice(), rates.as_slice());
        }
    }

    #[test]
    fn identity_ignores_the_branch_token() {
        let model = IdentityKinematics::new(2);
        let task = v(&[3.0, -4.0]);
        for token in [KinematicBranch(0), KinematicBranch(1), KinematicBranch(7)] {
            assert_eq!(
                model.inverse_position(task, token).unwrap().as_slice(),
                task.as_slice()
            );
        }
    }

    #[test]
    fn identity_rejects_a_dimension_mismatch() {
        let model = IdentityKinematics::new(3);
        assert_eq!(
            model.inverse_position(v(&[1.0, 2.0]), SOLE_BRANCH),
            Err(KinematicsError::DimensionMismatch {
                expected: 3,
                got: 2
            })
        );
        assert_eq!(
            model.inverse_velocity(v(&[1.0, 2.0, 3.0]), v(&[1.0])),
            Err(KinematicsError::DimensionMismatch {
                expected: 3,
                got: 1
            })
        );
    }

    // --- ScaraKinematics -------------------------------------------------

    const L: f64 = 100.0;
    fn arm() -> ScaraKinematics {
        ScaraKinematics::new(L, L)
    }

    #[test]
    fn home_pose_is_the_elbow_bent_90_degrees_and_is_not_singular() {
        let arm = arm();
        // (0, 0) raw means q2_eff = pi/2: link 1 along +x, link 2 along +y.
        let p = arm.forward_position(v(&[0.0, 0.0]));
        assert!(approx(p.as_slice()[0], L));
        assert!(approx(p.as_slice()[1], L));

        // ...and the Jacobian is at its most invertible there.
        let (_, measure) = arm.jacobian(0.0, 0.0);
        assert!(approx(measure, 1.0));
        assert!(
            arm.inverse_velocity(v(&[0.0, 0.0]), v(&[10.0, 10.0]))
                .is_ok()
        );
    }

    #[test]
    fn forward_then_inverse_recovers_the_pose_on_either_branch() {
        // The property the whole per-move-branch design rests on: resolve
        // the branch from a pose, and IK on that branch returns the pose.
        let arm = arm();
        let mut checked = 0;
        for i in -8..=8 {
            for j in -8..=8 {
                let q1 = i as f64 * PI / 8.0;
                let q2 = j as f64 * PI / 8.0;
                // Keep q2_eff inside the principal [-pi, pi] range, where
                // IK's answer is the same representative, not merely an
                // equivalent one modulo 2*pi.
                if !(-PI..=PI).contains(&ScaraKinematics::q2_eff(q2)) {
                    continue;
                }
                let joint = v(&[q1, q2]);
                let branch = arm.resolve_branch(joint);
                let task = arm.forward_position(joint);
                let back = arm.inverse_position(task, branch).unwrap();
                // q1 comes back wrapped into atan2's range; compare the
                // resulting pose, not the raw angle.
                let round_tripped = arm.forward_position(back);
                assert!(approx(round_tripped.as_slice()[0], task.as_slice()[0]));
                assert!(approx(round_tripped.as_slice()[1], task.as_slice()[1]));
                // Away from the folded/straight poses the branches are
                // distinct, so the exact joint values must match too.
                if ScaraKinematics::q2_eff(q2).sin().abs() > 1e-6 {
                    assert!(approx(back.as_slice()[1], q2));
                }
                checked += 1;
            }
        }
        assert!(checked > 100, "expected a broad sweep, checked {checked}");
    }

    #[test]
    fn the_two_branches_are_mirror_elbows_reaching_the_same_point() {
        let arm = arm();
        let task = v(&[120.0, 40.0]);
        let up = arm.inverse_position(task, ELBOW_A).unwrap();
        let down = arm.inverse_position(task, ELBOW_B).unwrap();
        assert!(up != down);
        for solution in [up, down] {
            let p = arm.forward_position(solution);
            assert!(approx(p.as_slice()[0], 120.0));
            assert!(approx(p.as_slice()[1], 40.0));
        }
        assert_eq!(arm.resolve_branch(up), ELBOW_A);
        assert_eq!(arm.resolve_branch(down), ELBOW_B);
    }

    #[test]
    fn unreachable_targets_are_rejected() {
        let arm = arm();
        // Beyond the outer boundary (l1 + l2 = 200).
        assert!(matches!(
            arm.inverse_position(v(&[201.0, 0.0]), ELBOW_A),
            Err(KinematicsError::Unreachable { .. })
        ));
        // Just inside it is fine.
        assert!(arm.inverse_position(v(&[199.0, 0.0]), ELBOW_A).is_ok());
        // Equal links make the inner boundary a single point, so the origin
        // itself is reachable — that is the singularity this design accepts
        // and handles reactively.
        assert!(arm.inverse_position(v(&[0.0, 0.0]), ELBOW_A).is_ok());
    }

    #[test]
    fn unequal_links_reject_the_inner_hole() {
        let arm = ScaraKinematics::new(100.0, 60.0);
        assert!(matches!(
            arm.inverse_position(v(&[10.0, 0.0]), ELBOW_A),
            Err(KinematicsError::Unreachable { .. })
        ));
        assert!(arm.inverse_position(v(&[50.0, 0.0]), ELBOW_A).is_ok());
    }

    #[test]
    fn inverse_velocity_refuses_near_both_singular_poses() {
        let arm = arm();
        let task_v = v(&[10.0, 0.0]);
        // Straight out: q2_eff = 0, i.e. raw q2 = -pi/2.
        assert!(matches!(
            arm.inverse_velocity(v(&[0.3, -FRAC_PI_2]), task_v),
            Err(KinematicsError::NearSingular { .. })
        ));
        // Folded back: q2_eff = pi, i.e. raw q2 = pi/2.
        assert!(matches!(
            arm.inverse_velocity(v(&[0.3, FRAC_PI_2]), task_v),
            Err(KinematicsError::NearSingular { .. })
        ));
        // asin(0.05) is about 2.87 degrees; 5 degrees off is accepted, 1 is
        // not.
        let five_degrees = 5.0_f64.to_radians();
        assert!(
            arm.inverse_velocity(v(&[0.3, -FRAC_PI_2 + five_degrees]), task_v)
                .is_ok()
        );
        let one_degree = 1.0_f64.to_radians();
        assert!(matches!(
            arm.inverse_velocity(v(&[0.3, -FRAC_PI_2 + one_degree]), task_v),
            Err(KinematicsError::NearSingular { .. })
        ));
    }

    #[test]
    fn forward_and_inverse_velocity_round_trip_away_from_singularity() {
        let arm = arm();
        for &(q1, q2) in &[(0.0, 0.0), (0.7, -0.4), (-1.2, 0.9), (2.5, 0.3)] {
            let joint = v(&[q1, q2]);
            let rates = v(&[0.35, -0.2]);
            let task_v = arm.forward_velocity(joint, rates);
            let back = arm.inverse_velocity(joint, task_v).unwrap();
            assert!(approx(back.as_slice()[0], 0.35));
            assert!(approx(back.as_slice()[1], -0.2));
        }
    }

    #[test]
    fn analytic_jacobian_matches_a_finite_difference() {
        let arm = arm();
        let h = 1e-6;
        for &(q1, q2) in &[(0.0, 0.0), (0.7, -0.4), (-1.2, 0.9), (2.5, 0.3)] {
            let (j, _) = arm.jacobian(q1, q2);
            for (column, (dq1, dq2)) in [(h, 0.0), (0.0, h)].iter().enumerate() {
                let plus = arm.forward_position(v(&[q1 + dq1, q2 + dq2]));
                let minus = arm.forward_position(v(&[q1 - dq1, q2 - dq2]));
                for (row, j_row) in j.iter().enumerate() {
                    let numeric = (plus.as_slice()[row] - minus.as_slice()[row]) / (2.0 * h);
                    assert!(
                        (numeric - j_row[column]).abs() < 1e-4,
                        "J[{row}][{column}]: analytic {} vs numeric {numeric}",
                        j_row[column]
                    );
                }
            }
        }
    }

    /// Round-trip through the *full* acceleration mapping, checked against
    /// a numerical second derivative of forward kinematics.
    ///
    /// Integrate a constant joint acceleration forward, differentiate the
    /// resulting tool path twice to get the true task-space acceleration,
    /// then ask `inverse_acceleration` to recover the joint acceleration we
    /// started from. This is what catches a wrong or missing `J̇·q̇` term —
    /// with `q̇ ≠ 0` the two disagree substantially.
    #[test]
    fn inverse_acceleration_recovers_joint_acceleration_through_jdot() {
        let arm = arm();
        let h = 1e-5;
        for &(q1, q2, w1, w2, a1, a2) in &[
            (0.3, 0.4, 0.9, -0.7, 0.5, 1.1),
            (-1.1, 0.8, -0.6, 0.4, -0.9, 0.3),
            (2.0, -0.5, 1.3, 1.1, 0.2, -0.8),
        ] {
            let joint = v(&[q1, q2]);
            let rates = v(&[w1, w2]);

            // Tool position either side of now, under constant joint accel.
            let pose_at = |dt: f64| {
                v(&[
                    q1 + w1 * dt + 0.5 * a1 * dt * dt,
                    q2 + w2 * dt + 0.5 * a2 * dt * dt,
                ])
            };
            let p_minus = arm.forward_position(pose_at(-h));
            let p_now = arm.forward_position(pose_at(0.0));
            let p_plus = arm.forward_position(pose_at(h));

            let mut task_accel = [0.0; 2];
            for (i, slot) in task_accel.iter_mut().enumerate() {
                *slot = (p_plus.as_slice()[i] - 2.0 * p_now.as_slice()[i] + p_minus.as_slice()[i])
                    / (h * h);
            }

            let recovered = arm
                .inverse_acceleration(joint, rates, v(&task_accel))
                .unwrap();
            assert!(
                (recovered.as_slice()[0] - a1).abs() < 1e-3
                    && (recovered.as_slice()[1] - a2).abs() < 1e-3,
                "recovered {:?}, expected [{a1}, {a2}]",
                recovered.as_slice()
            );
        }
    }

    /// The `J̇·q̇` term is not decoration: with the joints already moving,
    /// treating acceleration as if it were velocity gives a materially
    /// different answer. Guards against the term being quietly dropped.
    #[test]
    fn jdot_term_matters_when_the_joints_are_moving() {
        let arm = arm();
        let joint = v(&[0.3, 0.4]);
        let task_accel = v(&[10.0, -5.0]);

        let at_rest = arm
            .inverse_acceleration(joint, v(&[0.0, 0.0]), task_accel)
            .unwrap();
        let naive = arm.inverse_velocity(joint, task_accel).unwrap();
        // At rest J̇·q̇ vanishes, so the two coincide.
        assert!(approx(at_rest.as_slice()[0], naive.as_slice()[0]));

        // Measured for this pose and rate: the terms differ by ~0.7 and
        // ~1.7 rad/s² respectively, against joint accelerations of the same
        // order — so this is a leading-order effect, not a correction.
        let moving = arm
            .inverse_acceleration(joint, v(&[1.5, -1.0]), task_accel)
            .unwrap();
        let shift = moving
            .as_slice()
            .iter()
            .zip(naive.as_slice())
            .map(|(m, n)| (m - n).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            shift > 0.5,
            "J̇ term should shift the answer substantially while moving (shift {shift})"
        );
    }

    #[test]
    fn linkage_is_base_elbow_tool_with_the_right_link_lengths() {
        let arm = arm();
        for &(q1, q2) in &[(0.0, 0.0), (0.7, -0.4), (-1.2, 0.9), (2.5, 0.3)] {
            let joint = v(&[q1, q2]);
            let linkage = arm.linkage(joint);
            let points = linkage.points();
            assert_eq!(points.len(), 3);

            // Base at the origin.
            assert!(approx(points[0].as_slice()[0], 0.0));
            assert!(approx(points[0].as_slice()[1], 0.0));

            // Each drawn segment is exactly its link's length, which is the
            // property that makes the picture a picture of *this* arm.
            let segment = |a: KinematicVector, b: KinematicVector| {
                let (a, b) = (a.as_slice(), b.as_slice());
                ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt()
            };
            assert!(approx(segment(points[0], points[1]), L));
            assert!(approx(segment(points[1], points[2]), L));

            // The tool point is forward kinematics, not a second
            // computation of it.
            let tcp = arm.forward_position(joint);
            assert_eq!(points[2], tcp);
        }
    }

    #[test]
    fn identity_has_nothing_to_draw() {
        let model = IdentityKinematics::new(2);
        assert!(model.linkage(v(&[1.0, 2.0])).is_empty());
        assert_eq!(model.linkage(v(&[1.0, 2.0])).points().len(), 0);
    }

    #[test]
    fn scara_rejects_a_dimension_mismatch() {
        let arm = arm();
        assert_eq!(
            arm.inverse_position(v(&[1.0, 2.0, 3.0]), ELBOW_A),
            Err(KinematicsError::DimensionMismatch {
                expected: 2,
                got: 3
            })
        );
    }
}
