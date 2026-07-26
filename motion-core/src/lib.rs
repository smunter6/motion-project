//! # motion-core
//!
//! The I/O-free heart of the motion system: pure math for trajectory
//! generation, and (later) kinematics and shared motion types.
//!
//! ## Design rule
//!
//! This crate has **no I/O, no hardware knowledge, and no dependencies**. It
//! knows nothing about EtherCAT, simulation, or visualization. That is what
//! lets us:
//!   - unit-test all the hard-to-get-right math on the host (in WSL) with no
//!     bus and no hardware present, and
//!   - reuse the exact same code unchanged on the Raspberry Pi (and, being
//!     `no_std`-friendly in spirit, potentially on an RP2350 later).
//!
//! Anything that touches the outside world (a NIC, a servo drive, a window)
//! lives in a *different* crate, on the far side of a trait seam.
//!
//! ## Contents
//!
//! - [`trajectory`] — trapezoidal velocity profile and stop ramp for one axis.
//! - [`linear_move`] — straight-line moves across several axes.
//! - [`waypoint_path`] — multi-waypoint spline paths.
//! - [`path_profile`] — speed profile over a [`waypoint_path::WaypointPath`].

pub mod linear_move;
pub mod path_profile;
pub mod trajectory;
pub mod waypoint_path;

pub use linear_move::{LinearMove, LinearMoveError, LinearMoveSample, MAX_GROUP_AXES};
pub use path_profile::{PathProfile, PathProfileError, PathSample};
pub use trajectory::{MotionPhase, StopRamp, TrajectoryError, TrajectorySample, TrapezoidalProfile};
pub use waypoint_path::{WaypointPath, WaypointPathError};
