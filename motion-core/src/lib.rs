//! # motion-core
//!
//! Pure math for trajectory generation, paths, and kinematics.
//!
//! This crate has **no I/O, no hardware knowledge, and no dependencies**. It
//! knows nothing about EtherCAT, simulation, or visualization, so its tests run
//! on the host with no bus present. Anything that touches the outside world
//! lives in a different crate, behind the `axis-backend` trait.
//!
//! ## Contents
//!
//! - [`trajectory`] — trapezoidal velocity profile and stop ramp for one axis.
//! - [`jerk_filter`] — jerk-limited profile built by filtering a trapezoid.
//! - [`linear_move`] — straight-line moves across several axes.
//! - [`waypoint_path`] — multi-waypoint spline paths.
//! - [`path_profile`] — speed profile over a [`waypoint_path::WaypointPath`].
//! - [`kinematics`] — the joint-space ↔ task-space seam.

pub mod jerk_filter;
pub mod kinematics;
pub mod linear_move;
pub mod path_profile;
pub mod trajectory;
pub mod waypoint_path;

pub use jerk_filter::JerkFilteredProfile;
pub use kinematics::{
    IdentityKinematics, KinematicBranch, KinematicModel, KinematicVector, KinematicsError, Linkage,
    MAX_LINKAGE_POINTS, ScaraKinematics,
};
pub use linear_move::{LinearMove, LinearMoveError, LinearMoveSample, MAX_GROUP_AXES};
pub use path_profile::{PathProfile, PathProfileError, PathSample};
pub use trajectory::{
    MotionPhase, StopRamp, TrajectoryError, TrajectorySample, TrapezoidalProfile,
};
pub use waypoint_path::{WaypointPath, WaypointPathError};
