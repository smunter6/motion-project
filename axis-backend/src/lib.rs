//! `axis-backend`: the trait seam between the pure motion planner
//! (`motion-core`) and whatever moves the axes. `backend-sim` is the only
//! implementation. The run loop never knows which is behind the trait.
//!
//! # Combined `exchange()`
//!
//! A real EtherCAT cycle is one synchronous transaction: PDOs go out, the
//! bus processes them, PDOs come back (EtherCRAB's `tx_rx()`). [`AxisGroup`]
//! has the same shape: one [`exchange`](AxisGroup::exchange) call per control
//! cycle, rather than separate write and read calls.
//!
//! # Relationship to `motion_core::TrajectorySample`
//!
//! [`AxisSetpoint`] carries much the same numbers as `TrajectorySample`, but
//! they belong to different layers: `TrajectorySample` is the planner's output,
//! `AxisSetpoint` is what crosses the hardware seam. `axis-backend` does not
//! depend on `motion-core`.
//!
//! The acceleration field exists on both types, but means different things:
//! the planner's is exact and the plant's is an estimate. See
//! [`AxisFeedback::acceleration`].

use std::fmt;

/// A single control-cycle commanded setpoint for one axis.
///
/// Units are engineering units (mm, mm/s for a linear axis), matching
/// `motion-core`. A backend for real drives converts to integer encoder counts
/// on its side of the seam.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AxisSetpoint {
    /// Commanded position — the primary command in CSP mode, and the one
    /// quantity a backend must honour. Absolute, in the axis's own units.
    pub position: f64,
    /// Commanded velocity — a feed-forward term alongside `position`, not an
    /// independent command. A backend that only does CSP may ignore it.
    pub velocity: f64,
    /// Commanded acceleration — the second feed-forward term.
    ///
    /// Drives consume it: in CSP/CSV modes it becomes a torque offset. The
    /// planner knows it exactly; differencing the velocity stream downstream
    /// would give a delayed, noisier estimate.
    ///
    /// A backend with nowhere to put it may ignore it.
    pub acceleration: f64,
    /// Requested power-stage state: `true` to (request to) enable, `false`
    /// to (request to) disable. Sequencing the real multi-step DS402
    /// transitions to get there is each backend's own business. Sent every
    /// cycle, not as a one-off command.
    pub enabled: bool,
    /// Requests clearing a latched fault (see [`AxisFeedback::fault`]) — a
    /// distinct bit from `enabled`, since resetting a fault and requesting
    /// enable are different requests. Only has any effect while
    /// [`AxisFeedback::ds402_state`] is [`Ds402State::Fault`]; harmless
    /// otherwise. Resetting clears the fault but does *not* re-enable the
    /// axis — a fresh `enabled: true` afterward still has to run the normal
    /// sequence.
    pub fault_reset: bool,
    /// True while a commanded stop is decelerating this axis to rest.
    ///
    /// Not DS402's Quick Stop, which is a separate, typically
    /// emergency-triggered mechanism with its own
    /// [`Ds402State::QuickStopActive`]. To the drive a commanded stop is an
    /// ordinary decelerating velocity setpoint in `OperationEnabled`. This
    /// flag only changes what [`AxisFeedback::state`] reports (`Stopping`
    /// instead of `DiscreteMotion`); `ds402_state` is unaffected.
    pub stopping: bool,
}

/// A single control-cycle feedback reading for one axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisFeedback {
    /// Measured position, in the axis's own units. `app` uses it for
    /// reporting only; it plans from its own commanded state.
    pub position: f64,
    /// Measured velocity. Reporting only, like `position`.
    pub velocity: f64,
    /// Measured acceleration.
    ///
    /// Unlike [`AxisSetpoint::acceleration`], which the planner computes in
    /// closed form, this is a plant quantity. Most real drives have no
    /// acceleration object, so a hardware backend would derive it from
    /// successive velocities and inherit that estimate's lag and noise.
    /// `backend-sim` knows the velocity change it just applied. Use it for
    /// display and diagnostics, not as ground truth.
    pub acceleration: f64,
    /// A fault this axis is reporting, if any. Mirrors
    /// [`ds402_state`](Self::ds402_state) being [`Ds402State::FaultReactionActive`]
    /// or [`Ds402State::Fault`] — `None` the rest of the time.
    pub fault: Option<AxisFault>,
    /// This axis's motion-control state machine state. See [`AxisState`].
    pub state: AxisState,
    /// Sub-flags describing motion in progress, meaningful alongside
    /// [`AxisState::DiscreteMotion`]. See [`MotionFlags`].
    pub motion: MotionFlags,
    /// This axis's real CiA 402 (DS402) power-state, one layer more
    /// detailed than [`AxisState`]. See [`Ds402State`] for why both exist.
    pub ds402_state: Ds402State,
}

/// One axis's motion-control state — a coarse status enum with mutually
/// exclusive variants.
///
/// `backend-sim` only ever reports `Disabled`, `StandStill`,
/// `DiscreteMotion`, `Stopping`, or `ErrorStop`. `ContinuousMotion`,
/// `SynchronizedMotion` and `Homing` are never reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisState {
    /// Drive power stage off. Reported whenever the underlying
    /// [`Ds402State`] is anything short of `OperationEnabled`.
    Disabled,
    /// A fault is latched (see [`AxisFeedback::fault`]); motion is stopped.
    ErrorStop,
    /// Decelerating to a controlled stop after a commanded stop (see
    /// [`AxisSetpoint::stopping`]) — reported instead of `DiscreteMotion`
    /// while decelerating for that reason. Not the same thing as
    /// `Ds402State::QuickStopActive`; see `stopping`'s own docs for why.
    Stopping,
    /// At rest, no fault, ready to accept a move.
    StandStill,
    /// Executing a point-to-point move.
    DiscreteMotion,
    /// Executing a jog/velocity-mode move. Not raised by any backend.
    ContinuousMotion,
    /// Executing a coordinated multi-axis move. Not raised by any backend.
    SynchronizedMotion,
    /// Executing a homing sequence. Not raised by any backend.
    Homing,
}

/// One axis's real DS402 power-state machine state — the actual state a
/// servo drive's statusword reports, one layer more detailed than
/// [`AxisState`].
///
/// A real motion controller manages a drive's DS402 state machine internally
/// and exposes only a coarser view (`AxisState`) to the application. See
/// [`Ds402State::axis_state`] for that mapping.
///
/// Omits DS402's transient `Not Ready to Switch On` pseudo-state, which is
/// occupied only briefly after power-on. Backends start in `SwitchOnDisabled`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ds402State {
    /// Power stage off; not yet enabled. The startup state.
    SwitchOnDisabled,
    /// Enabled a first step: ready, but the power stage still isn't live.
    ReadyToSwitchOn,
    /// Power stage live, but not yet accepting motion commands.
    SwitchedOn,
    /// Fully enabled: power stage live and accepting motion commands. The
    /// only state in which [`AxisFeedback::motion`] can be non-default.
    OperationEnabled,
    /// Controlled stop in progress after a quick-stop request. Not raised
    /// by any backend; there is no quick-stop command.
    QuickStopActive,
    /// A fault was just detected; held for exactly one `exchange()` cycle
    /// before automatically advancing to `Fault` (DS402 transition 14).
    /// `backend-sim` has no braking dynamics to perform in this state.
    FaultReactionActive,
    /// A fault is latched; motion is stopped until
    /// [`AxisSetpoint::fault_reset`] clears it (DS402 transition 15, back to
    /// `SwitchOnDisabled`). Resetting doesn't re-enable the axis — that
    /// still needs a fresh `enabled: true` afterward.
    Fault,
}

impl Ds402State {
    /// Maps this DS402 power-state to the coarser [`AxisState`] reported at
    /// the `AxisGroup` trait level. `moving` only matters in
    /// `OperationEnabled` — that's the only DS402 state in which the axis
    /// can actually be executing motion.
    pub fn axis_state(self, moving: bool) -> AxisState {
        match self {
            Ds402State::SwitchOnDisabled | Ds402State::ReadyToSwitchOn | Ds402State::SwitchedOn => {
                AxisState::Disabled
            }
            Ds402State::OperationEnabled if moving => AxisState::DiscreteMotion,
            Ds402State::OperationEnabled => AxisState::StandStill,
            Ds402State::QuickStopActive => AxisState::Stopping,
            Ds402State::FaultReactionActive | Ds402State::Fault => AxisState::ErrorStop,
        }
    }
}

/// Sub-flags describing motion in progress, alongside [`AxisState`]. Not a
/// state machine, just "which part of the velocity profile is this axis in
/// right now."
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MotionFlags {
    /// Speeding up toward the profile's cruise velocity.
    pub accelerating: bool,
    /// In the profile's cruise phase. Never set on a triangular move, which
    /// has no cruise phase at all.
    pub constant_velocity: bool,
    /// Slowing toward rest — whether that's a move's own final phase or a
    /// commanded stop (see [`AxisSetpoint::stopping`], which distinguishes
    /// the two).
    pub decelerating: bool,
}

/// A fault reported by a backend for one axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisFault {
    /// Removed `enabled` while the axis was actively moving. A real drive
    /// can't safely cut its power stage mid-motion the way it can from rest,
    /// so `backend-sim` treats it as a fault rather than a graceful disable,
    /// requiring [`AxisSetpoint::fault_reset`] before it accepts
    /// `enabled: true` again.
    DisabledWhileMoving,
    /// A fault with no more specific category. No backend raises it.
    Unspecified,
}

/// Reasons a call to [`AxisGroup::exchange`] itself failed — as opposed to
/// an individual axis fault (see [`AxisFeedback::fault`]), this is a
/// problem with the call itself, not with any one axis's motion.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AxisGroupError {
    /// `setpoints.len()` didn't match [`AxisGroup::num_axes`].
    WrongSetpointCount {
        /// How many setpoints the group wanted — its [`AxisGroup::num_axes`].
        expected: usize,
        /// How many were actually passed.
        got: usize,
    },
}

impl fmt::Display for AxisGroupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AxisGroupError::WrongSetpointCount { expected, got } => {
                write!(f, "expected {expected} setpoints (one per axis), got {got}")
            }
        }
    }
}

impl std::error::Error for AxisGroupError {}

/// The seam between the motion planner and whatever moves the axes.
///
/// The only implementation is the software simulation in `backend-sim`.
pub trait AxisGroup {
    /// Number of axes this group manages. `setpoints` passed to
    /// [`exchange`](Self::exchange), and the [`AxisFeedback`] slice it
    /// returns, always have exactly this many elements, axis-index-ordered.
    fn num_axes(&self) -> usize;

    /// Perform one control cycle's exchange: send this cycle's setpoints to
    /// the backend, and read back this cycle's feedback.
    ///
    /// The returned slice borrows from `self`: implementations hold their
    /// feedback in a reusable buffer rather than allocating every cycle.
    fn exchange(&mut self, setpoints: &[AxisSetpoint]) -> Result<&[AxisFeedback], AxisGroupError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal in-memory `AxisGroup` that echoes setpoints back as feedback.
    struct MockAxisGroup {
        feedback: Vec<AxisFeedback>,
    }

    impl MockAxisGroup {
        fn new(num_axes: usize) -> Self {
            Self {
                feedback: vec![
                    AxisFeedback {
                        position: 0.0,
                        velocity: 0.0,
                        acceleration: 0.0,
                        fault: None,
                        state: AxisState::StandStill,
                        motion: MotionFlags::default(),
                        ds402_state: Ds402State::OperationEnabled,
                    };
                    num_axes
                ],
            }
        }
    }

    impl AxisGroup for MockAxisGroup {
        fn num_axes(&self) -> usize {
            self.feedback.len()
        }

        fn exchange(
            &mut self,
            setpoints: &[AxisSetpoint],
        ) -> Result<&[AxisFeedback], AxisGroupError> {
            if setpoints.len() != self.feedback.len() {
                return Err(AxisGroupError::WrongSetpointCount {
                    expected: self.feedback.len(),
                    got: setpoints.len(),
                });
            }
            for (fb, sp) in self.feedback.iter_mut().zip(setpoints) {
                fb.position = sp.position;
                fb.velocity = sp.velocity;
            }
            Ok(&self.feedback)
        }
    }

    #[test]
    fn exchange_echoes_setpoints_as_feedback() {
        let mut group = MockAxisGroup::new(2);
        let setpoints = [
            AxisSetpoint {
                position: 10.0,
                velocity: 1.0,
                acceleration: 0.0,
                enabled: true,
                fault_reset: false,
                stopping: false,
            },
            AxisSetpoint {
                position: 20.0,
                velocity: 2.0,
                acceleration: 0.0,
                enabled: true,
                fault_reset: false,
                stopping: false,
            },
        ];
        let feedback = group.exchange(&setpoints).unwrap();
        assert_eq!(feedback[0].position, 10.0);
        assert_eq!(feedback[1].position, 20.0);
        assert!(feedback.iter().all(|fb| fb.fault.is_none()));
    }

    #[test]
    fn exchange_rejects_wrong_setpoint_count() {
        let mut group = MockAxisGroup::new(2);
        let setpoints = [AxisSetpoint {
            position: 0.0,
            velocity: 0.0,
            enabled: true,
            ..Default::default()
        }];
        let err = group.exchange(&setpoints).unwrap_err();
        assert_eq!(
            err,
            AxisGroupError::WrongSetpointCount {
                expected: 2,
                got: 1
            }
        );
    }
}
