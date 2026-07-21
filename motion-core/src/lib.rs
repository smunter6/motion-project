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
//! ## Contents so far
//!
//! - [`trajectory`] — the trapezoidal velocity profile for one axis.
//!
//! Coming in later steps: an `AxisGroup` trait (the command/feedback seam), a
//! software sim backend, simple visualization, and eventually an EtherCRAB +
//! CiA 402 backend for real drives.

pub mod linear_move;
pub mod trajectory;

pub use linear_move::{LinearMove, LinearMoveError, LinearMoveSample, MAX_GROUP_AXES};
pub use trajectory::{MotionPhase, StopRamp, TrajectoryError, TrajectorySample, TrapezoidalProfile};
