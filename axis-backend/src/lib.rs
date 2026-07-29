//! `axis-backend`: the trait seam between the pure motion planner
//! (`motion-core`) and whatever actually moves the axes — a software
//! simulation today (`backend-sim`), real EtherCAT servo drives later
//! (`backend-ethercat`). The run loop that drives all this never knows
//! which is behind the trait.
//!
//! # Why a combined `exchange()`, not separate write/read
//!
//! A real EtherCAT cycle is one synchronous transaction: PDOs go out, the
//! bus processes them, PDOs come back — that's what EtherCRAB's `tx_rx()`
//! does. Modeling [`AxisGroup`] the same way (one [`exchange`](AxisGroup::exchange)
//! call per control cycle) means `backend-sim` and `backend-ethercat` share
//! an identical calling convention; the run loop doesn't change shape when
//! a real backend replaces the sim one.
//!
//! # Why these types don't reuse `motion_core::TrajectorySample`
//!
//! [`AxisSetpoint`] carries much the same numbers as `TrajectorySample`,
//! but they belong to different layers: `TrajectorySample` is the planner's
//! pure-math output, `AxisSetpoint` is what crosses the hardware seam.
//! Keeping them distinct means a planner-side change or a future
//! backend-side need (say, a torque limit per cycle) doesn't ripple across
//! the seam just because one crate depended on the other's type.
//! `axis-backend` deliberately does not depend on `motion-core`.
//!
//! The acceleration field is a good example of the split earning its keep:
//! it was added to *both* types for the same reason (drives consume it as a
//! feed-forward), but they don't mean quite the same thing on each side —
//! the planner's is exact and the plant's is an estimate. See
//! [`AxisFeedback::acceleration`].

use std::fmt;

/// A single control-cycle commanded setpoint for one axis.
///
/// Units are engineering units (mm, mm/s for a linear axis), matching
/// `motion-core`. Conversion to whatever a real drive actually wants
/// (integer encoder counts) is `backend-ethercat`'s job at its side of this
/// seam, not this type's.
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
    /// Carried across the seam rather than derived by whoever wants it,
    /// because real drives *consume* it: in CSP/CSV modes it becomes a
    /// torque offset (torque being what accelerates the load), and the
    /// planner is the only layer that knows it exactly. Differencing the
    /// velocity stream downstream would give a delayed, noisier estimate of
    /// something already known in closed form.
    ///
    /// A backend that has nowhere to put it may ignore it — same as any
    /// drive that doesn't map the corresponding object.
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
    /// Deliberately **not** DS402's Quick Stop — that's a distinct,
    /// typically emergency/safety-triggered mechanism with its own
    /// [`Ds402State::QuickStopActive`], unaffected by this flag. From the
    /// drive's own point of view a commanded stop is nothing special: it's
    /// still just an ordinary decelerating velocity setpoint sent every
    /// cycle in `OperationEnabled`, same as the tail end of any move. This
    /// flag only changes what [`AxisFeedback::state`] reports at the
    /// coarser layer (`Stopping` instead of `DiscreteMotion`) —
    /// `ds402_state` is unaffected by it entirely.
    pub stopping: bool,
}

/// A single control-cycle feedback reading for one axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisFeedback {
    /// Measured position, in the axis's own units. Ground truth for
    /// *reporting*; note `app` plans from its own commanded state instead,
    /// so this is deliberately not fed back into the planner.
    pub position: f64,
    /// Measured velocity. Reporting only, for the same reason as `position`.
    pub velocity: f64,
    /// Measured acceleration.
    ///
    /// **Asymmetric with [`AxisSetpoint::acceleration`], deliberately.**
    /// The commanded value is exact — the planner computes it in closed
    /// form. This one is a *plant* quantity, and most real drives have no
    /// acceleration object to report, so `backend-ethercat` will likely
    /// have to derive it from successive velocities and will inherit that
    /// estimate's lag and noise. `backend-sim` can report it honestly
    /// because it knows the velocity change it just applied.
    ///
    /// Don't read it as ground truth the way position and velocity are
    /// read; it is for display and diagnostics.
    pub acceleration: f64,
    /// A fault this axis is reporting, if any. Mirrors
    /// [`ds402_state`](Self::ds402_state) being [`Ds402State::FaultReactionActive`]
    /// or [`Ds402State::Fault`] — `None` the rest of the time.
    pub fault: Option<AxisFault>,
    /// This axis's motion-control state machine state. See [`AxisState`].
    pub state: AxisState,
    /// Sub-flags describing motion in progress, meaningful alongside
    /// [`AxisState::DiscreteMotion`] (and, later, `ContinuousMotion`/
    /// `SynchronizedMotion`). See [`MotionFlags`].
    pub motion: MotionFlags,
    /// This axis's real CiA 402 (DS402) power-state, one layer more
    /// detailed than [`AxisState`]. See [`Ds402State`] for why both exist.
    pub ds402_state: Ds402State,
}

/// One axis's motion-control state — a coarse status enum,
/// mutually exclusive by construction, that `backend-ethercat` will
/// eventually need to report a real drive's state through as well.
///
/// Several variants aren't raised by any backend yet — `backend-sim` only
/// ever reports `Disabled`, `StandStill`, `DiscreteMotion`, `Stopping`, or
/// `ErrorStop`. They're included now so this type doesn't need reshaping
/// later, when homing, jogging, or coordinated moves (still deferred)
/// arrive.
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
    /// Executing a jog/velocity-mode move. Not raised by any backend yet.
    ContinuousMotion,
    /// Executing a coordinated multi-axis move. Not raised by any backend
    /// yet — coordinated moves are still deferred.
    SynchronizedMotion,
    /// Executing a homing sequence. Not raised by any backend yet.
    Homing,
}

/// One axis's real DS402 power-state machine state — the actual state a
/// servo drive's statusword reports, one layer more detailed than
/// [`AxisState`].
///
/// The two layers exist because a real motion controller manages a drive's
/// DS402 state machine internally and exposes only a coarser view
/// (`AxisState`) to the application; the detailed drive state is
/// backend/vendor-specific detail beneath the standard bits. See
/// [`Ds402State::axis_state`] for that mapping.
///
/// Omits DS402's transient `Not Ready to Switch On` pseudo-state (occupied
/// only for an instant right after power-on, before self-test completes) —
/// backends here start already past it, in `SwitchOnDisabled`.
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
    /// by any backend yet — no quick-stop command exists yet.
    QuickStopActive,
    /// A fault was just detected; held for exactly one `exchange()` cycle
    /// before automatically advancing to `Fault` (DS402 transition 14) —
    /// mirrors a real drive's brief fault-handling window, even though
    /// `backend-sim` has no braking dynamics to actually perform there.
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
///
/// Deliberately minimal — categories get added when a backend actually
/// needs to report them, not speculatively.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisFault {
    /// Removed `enabled` while the axis was actively moving. A real drive
    /// can't safely just cut its power stage mid-motion the way it can from
    /// rest (uncontrolled coast/stop) — `backend-sim` treats it as a fault
    /// rather than a graceful disable, requiring [`AxisSetpoint::fault_reset`]
    /// before it'll accept `enabled: true` again. A real quick-stop command
    /// (DS402 `QuickStopActive`, not implemented yet) is the controlled way
    /// to stop a moving axis without faulting.
    DisabledWhileMoving,
    /// Placeholder for fault categories no backend raises yet (following
    /// error exceeded, drive fault, not operational...).
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
/// Implementations: a software simulation (`backend-sim`), and eventually
/// real EtherCAT servo drives (`backend-ethercat`). The run loop that owns
/// an `AxisGroup` never needs to know which.
pub trait AxisGroup {
    /// Number of axes this group manages. `setpoints` passed to
    /// [`exchange`](Self::exchange), and the [`AxisFeedback`] slice it
    /// returns, always have exactly this many elements, axis-index-ordered.
    fn num_axes(&self) -> usize;

    /// Perform one control cycle's exchange: send this cycle's setpoints to
    /// the backend, and read back this cycle's feedback.
    ///
    /// A single call rather than separate write/read methods, matching how
    /// a real fieldbus cycle actually works (see the module docs). The
    /// returned slice borrows from `self` — implementations are expected to
    /// hold their feedback in a reusable buffer rather than allocating one
    /// every cycle at 250 Hz.
    fn exchange(&mut self, setpoints: &[AxisSetpoint]) -> Result<&[AxisFeedback], AxisGroupError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal in-memory `AxisGroup` used only to check the trait is
    /// actually usable end-to-end — not a stand-in for `backend-sim`, which
    /// gets a real (if still trivial) plant model of its own.
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
