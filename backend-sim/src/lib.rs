//! `backend-sim`: a software `AxisGroup` implementation — no hardware, no
//! network, just a plant model.
//!
//! # The plant model
//!
//! Each cycle, [`SimAxisGroup`] integrates the *velocity* setpoint into its
//! own position state (`position += velocity * dt`) rather than snapping to
//! the commanded position. There is no lag and no dynamics, but the model is
//! stateful and steps by `dt`. Euler-integrating a velocity profile that is a
//! closed-form quadratic during accel/decel leaves a small discretization
//! error between the planner's exact position and the sim's integrated one.
//!
//! The model only reads `AxisSetpoint::velocity`, never
//! `AxisSetpoint::position`: it dead-reckons position from commanded velocity
//! and does not track a position target. A real drive in CSP mode works the
//! other way around: position is the primary command, and the drive's servo
//! loop is where following error comes from. This is not a faithful physical
//! model.
//!
//! It has no second-order lag and no following-error limit. The one fault it
//! raises is described at `step_ds402`.

use axis_backend::{
    AxisFault, AxisFeedback, AxisGroup, AxisGroupError, AxisSetpoint, AxisState, Ds402State,
    MotionFlags,
};

/// Below this speed, an axis counts as "at rest" for [`AxisState`] purposes.
const STANDSTILL_EPS: f64 = 1e-6;

/// Below this per-cycle change in commanded speed, the axis counts as at
/// constant velocity rather than accelerating/decelerating. Real ramps change
/// by many multiples of this every cycle.
const ACCEL_EPS: f64 = 1e-9;

/// Advances a DS402 power-state by at most one standard transition per
/// call.
///
/// While no fault is in play, this steps toward `SwitchOnDisabled ->
/// ReadyToSwitchOn -> SwitchedOn -> OperationEnabled` while `enabled` is
/// `true`, or back down the same three states while it's `false`.
/// Already-at-target is a no-op either way.
///
/// Each controlword write moves a real drive at most one standard transition,
/// confirmed by the next statusword read. DS402 also allows multi-state
/// shortcuts coming down; this always steps one state at a time.
///
/// `fault_detected` (computed by the caller — see `exchange`) preempts
/// everything else, from any state. `FaultReactionActive` always advances to
/// `Fault` one cycle later, with no controlword input. `Fault` only leaves via
/// `fault_reset` (back to `SwitchOnDisabled`); toggling `enabled` alone does
/// nothing there.
///
/// `QuickStopActive` is unreachable (there is no quick-stop command) and is
/// held in place if ever reached. A *commanded* stop
/// (`AxisSetpoint::stopping`) never touches this state machine; see
/// `AxisSetpoint::stopping`.
fn step_ds402(
    current: Ds402State,
    enabled: bool,
    fault_reset: bool,
    fault_detected: bool,
) -> Ds402State {
    use Ds402State::*;

    if fault_detected {
        return FaultReactionActive;
    }

    match current {
        SwitchOnDisabled if enabled => ReadyToSwitchOn,
        SwitchOnDisabled => SwitchOnDisabled,
        ReadyToSwitchOn if enabled => SwitchedOn,
        ReadyToSwitchOn => SwitchOnDisabled,
        SwitchedOn if enabled => OperationEnabled,
        SwitchedOn => ReadyToSwitchOn,
        OperationEnabled if enabled => OperationEnabled,
        OperationEnabled => SwitchedOn,
        QuickStopActive => QuickStopActive,
        FaultReactionActive => Fault,
        Fault if fault_reset => SwitchOnDisabled,
        Fault => Fault,
    }
}

/// A software `AxisGroup`: `num_axes` independent axes, each integrating its
/// own velocity setpoint into position at a fixed cycle time `dt`.
pub struct SimAxisGroup {
    dt: f64,
    feedback: Vec<AxisFeedback>,
}

impl SimAxisGroup {
    /// `dt` is the fixed control-cycle time in seconds (e.g. `1.0 / 250.0`
    /// for a 250 Hz loop). All axes start at rest at position 0.0.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive.
    pub fn new(num_axes: usize, dt: f64) -> Self {
        assert!(dt > 0.0, "dt must be positive, got {dt}");
        Self {
            dt,
            feedback: vec![
                AxisFeedback {
                    position: 0.0,
                    velocity: 0.0,
                    acceleration: 0.0,
                    fault: None,
                    state: AxisState::Disabled,
                    motion: MotionFlags::default(),
                    ds402_state: Ds402State::SwitchOnDisabled,
                };
                num_axes
            ],
        }
    }
}

impl AxisGroup for SimAxisGroup {
    fn num_axes(&self) -> usize {
        self.feedback.len()
    }

    fn exchange(&mut self, setpoints: &[AxisSetpoint]) -> Result<&[AxisFeedback], AxisGroupError> {
        if setpoints.len() != self.feedback.len() {
            return Err(AxisGroupError::WrongSetpointCount {
                expected: self.feedback.len(),
                got: setpoints.len(),
            });
        }
        for (fb, sp) in self.feedback.iter_mut().zip(setpoints) {
            // Disabling a moving axis is a fault. `fb.velocity` still holds
            // last cycle's speed here; it is overwritten below.
            let was_moving = fb.velocity.abs() > STANDSTILL_EPS;
            let fault_detected =
                fb.ds402_state == Ds402State::OperationEnabled && !sp.enabled && was_moving;

            fb.ds402_state = step_ds402(fb.ds402_state, sp.enabled, sp.fault_reset, fault_detected);
            fb.fault = if fault_detected {
                Some(AxisFault::DisabledWhileMoving)
            } else if matches!(
                fb.ds402_state,
                Ds402State::FaultReactionActive | Ds402State::Fault
            ) {
                fb.fault // still faulted from a previous cycle — preserve it
            } else {
                None
            };

            let operational = fb.ds402_state == Ds402State::OperationEnabled;

            // No lag, so the commanded velocity is the actual speed, but only
            // while the power stage is live. A disabled or faulted axis
            // ignores motion commands.
            let prev_speed = fb.velocity;
            let speed = if operational { sp.velocity } else { 0.0 };

            if operational {
                fb.position += speed * self.dt;
            }
            // Derived from the velocity change actually applied, not copied
            // from `sp.acceleration`: `speed` is zero while not operational.
            fb.acceleration = (speed - prev_speed) / self.dt;
            fb.velocity = speed;

            let moving = speed.abs() > STANDSTILL_EPS;
            let base_state = fb.ds402_state.axis_state(moving);
            // A commanded stop changes only AxisState, never Ds402State, and
            // only replaces DiscreteMotion.
            fb.state = if sp.stopping && base_state == AxisState::DiscreteMotion {
                AxisState::Stopping
            } else {
                base_state
            };

            fb.motion = if !moving {
                MotionFlags::default()
            } else {
                let delta = speed.abs() - prev_speed.abs();
                MotionFlags {
                    accelerating: delta > ACCEL_EPS,
                    constant_velocity: delta.abs() <= ACCEL_EPS,
                    decelerating: delta < -ACCEL_EPS,
                }
            };
        }
        Ok(&self.feedback)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Steps every axis in `sim` through the 3-cycle DS402 enable sequence
    /// so it's `OperationEnabled`. Axes start `SwitchOnDisabled`, so tests
    /// that aren't about the enable sequence call this first.
    fn warm_up_enabled(sim: &mut SimAxisGroup, num_axes: usize) {
        for _ in 0..3 {
            let setpoints = vec![
                AxisSetpoint {
                    position: 0.0,
                    velocity: 0.0,
                    enabled: true,
                    ..Default::default()
                };
                num_axes
            ];
            sim.exchange(&setpoints).unwrap();
        }
    }

    #[test]
    fn integrates_velocity_into_position() {
        let mut sim = SimAxisGroup::new(1, 1.0); // dt = 1s for easy arithmetic
        warm_up_enabled(&mut sim, 1);
        let setpoints = [AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
            enabled: true,
            ..Default::default()
        }];

        let fb = sim.exchange(&setpoints).unwrap();
        assert_eq!(fb[0].position, 5.0);
        assert_eq!(fb[0].velocity, 5.0);

        // Accumulates across cycles rather than resetting each call.
        let fb = sim.exchange(&setpoints).unwrap();
        assert_eq!(fb[0].position, 10.0);
    }

    #[test]
    fn only_velocity_drives_the_plant_not_position() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        // A different commanded position is ignored; only velocity integrates.
        let setpoints = [AxisSetpoint {
            position: 999.0,
            velocity: -3.0,
            enabled: true,
            ..Default::default()
        }];
        let fb = sim.exchange(&setpoints).unwrap();
        assert_eq!(fb[0].velocity, -3.0);
        assert!((fb[0].position - (-3.0 * 0.004)).abs() < 1e-12);
    }

    #[test]
    fn axes_integrate_independently() {
        let mut sim = SimAxisGroup::new(2, 1.0);
        warm_up_enabled(&mut sim, 2);
        let setpoints = [
            AxisSetpoint {
                position: 0.0,
                velocity: 10.0,
                enabled: true,
                ..Default::default()
            },
            AxisSetpoint {
                position: 0.0,
                velocity: -4.0,
                enabled: true,
                ..Default::default()
            },
        ];
        let fb = sim.exchange(&setpoints).unwrap();
        assert_eq!(fb[0].position, 10.0);
        assert_eq!(fb[1].position, -4.0);
    }

    #[test]
    fn rejects_wrong_setpoint_count() {
        let mut sim = SimAxisGroup::new(2, 1.0);
        let err = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                ..Default::default()
            }])
            .unwrap_err();
        assert_eq!(
            err,
            AxisGroupError::WrongSetpointCount {
                expected: 2,
                got: 1
            }
        );
    }

    #[test]
    fn no_faults_reported_during_ordinary_operation() {
        let mut sim = SimAxisGroup::new(1, 1.0);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 1.0,
                enabled: true,
                ..Default::default()
            }])
            .unwrap();
        assert!(fb[0].fault.is_none());
    }

    /// Drives one axis through a hand-crafted speed sequence (ramp up,
    /// cruise, ramp down, stop) and checks `AxisState`/`MotionFlags` after
    /// every cycle, independent of any real trajectory shape.
    fn drive_and_check(
        sim: &mut SimAxisGroup,
        velocities: &[f64],
        expected: &[(AxisState, MotionFlags)],
    ) {
        assert_eq!(velocities.len(), expected.len());
        for (i, (&velocity, &(want_state, want_motion))) in
            velocities.iter().zip(expected).enumerate()
        {
            let fb = sim
                .exchange(&[AxisSetpoint {
                    position: 0.0,
                    velocity,
                    enabled: true,
                    ..Default::default()
                }])
                .unwrap();
            assert_eq!(
                fb[0].state, want_state,
                "cycle {i}: velocity={velocity}, expected state {want_state:?}, got {:?}",
                fb[0].state
            );
            assert_eq!(
                fb[0].motion, want_motion,
                "cycle {i}: velocity={velocity}, expected motion {want_motion:?}, got {:?}",
                fb[0].motion
            );
        }
    }

    #[test]
    fn state_and_motion_flags_track_a_hand_crafted_ramp_positive_direction() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        let velocities = [0.0, 2.0, 4.0, 4.0, 4.0, 2.0, 0.0];
        let accel = (
            AxisState::DiscreteMotion,
            MotionFlags {
                accelerating: true,
                constant_velocity: false,
                decelerating: false,
            },
        );
        let cruise = (
            AxisState::DiscreteMotion,
            MotionFlags {
                accelerating: false,
                constant_velocity: true,
                decelerating: false,
            },
        );
        let decel = (
            AxisState::DiscreteMotion,
            MotionFlags {
                accelerating: false,
                constant_velocity: false,
                decelerating: true,
            },
        );
        let rest = (AxisState::StandStill, MotionFlags::default());
        let expected = [rest, accel, accel, cruise, cruise, decel, rest];
        drive_and_check(&mut sim, &velocities, &expected);
    }

    #[test]
    fn state_and_motion_flags_track_a_hand_crafted_ramp_negative_direction() {
        // The flags are defined on |speed|, so this classifies identically
        // to the positive-direction case.
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        let velocities = [0.0, -2.0, -4.0, -4.0, -4.0, -2.0, 0.0];
        let accel = (
            AxisState::DiscreteMotion,
            MotionFlags {
                accelerating: true,
                constant_velocity: false,
                decelerating: false,
            },
        );
        let cruise = (
            AxisState::DiscreteMotion,
            MotionFlags {
                accelerating: false,
                constant_velocity: true,
                decelerating: false,
            },
        );
        let decel = (
            AxisState::DiscreteMotion,
            MotionFlags {
                accelerating: false,
                constant_velocity: false,
                decelerating: true,
            },
        );
        let rest = (AxisState::StandStill, MotionFlags::default());
        let expected = [rest, accel, accel, cruise, cruise, decel, rest];
        drive_and_check(&mut sim, &velocities, &expected);
    }

    /// Drives a real `motion_core::TrapezoidalProfile`'s sampled velocity
    /// through `SimAxisGroup`, cycle by cycle at the app's real 250 Hz
    /// control rate, and spot-checks state/motion flags at points well
    /// inside each phase. Cycles straddling a phase boundary are skipped:
    /// `phase_at(t)` switches at an exact continuous-time boundary, but that
    /// cycle can still show the previous phase's flag.
    fn assert_motion_flags_track_profile(start: f64, end: f64) {
        use motion_core::{MotionPhase, TrapezoidalProfile};

        let dt = 1.0 / 250.0;
        // max_speed=50, accel=decel=200 => accel phase 0..0.25s, cruise
        // 0.25..2.0s, decel 2.0..2.25s, total 2.25s duration.
        let profile = TrapezoidalProfile::new(start, end, 50.0, 200.0, 200.0).unwrap();
        let mut sim = SimAxisGroup::new(1, dt);
        warm_up_enabled(&mut sim, 1);

        // (checkpoint time, expected phase) — each picked well inside its
        // phase, at least 2 cycles' margin from any boundary.
        let checkpoints = [
            (0.10, MotionPhase::Accel),
            (1.00, MotionPhase::Cruise),
            (2.15, MotionPhase::Decel),
        ];

        let total_steps = (profile.duration() / dt).ceil() as i64 + 5;
        for step in 1..=total_steps {
            let t = step as f64 * dt;
            let sample = profile.sample(t);
            let fb = sim
                .exchange(&[AxisSetpoint {
                    position: sample.position,
                    velocity: sample.velocity,
                    enabled: true,
                    ..Default::default()
                }])
                .unwrap();

            for &(checkpoint_t, expected_phase) in &checkpoints {
                if (t - checkpoint_t).abs() > dt / 2.0 {
                    continue;
                }
                assert_eq!(
                    profile.phase_at(t),
                    expected_phase,
                    "test bug: checkpoint t={t} isn't actually in {expected_phase:?}"
                );
                assert_eq!(fb[0].state, AxisState::DiscreteMotion, "t={t}");
                let motion = fb[0].motion;
                match expected_phase {
                    MotionPhase::Accel => assert!(
                        motion.accelerating && !motion.constant_velocity && !motion.decelerating,
                        "t={t}: expected accelerating, got {motion:?}"
                    ),
                    MotionPhase::Cruise => assert!(
                        motion.constant_velocity && !motion.accelerating && !motion.decelerating,
                        "t={t}: expected constant_velocity, got {motion:?}"
                    ),
                    MotionPhase::Decel => assert!(
                        motion.decelerating && !motion.accelerating && !motion.constant_velocity,
                        "t={t}: expected decelerating, got {motion:?}"
                    ),
                    _ => unreachable!("checkpoints are only Accel/Cruise/Decel"),
                }
            }
        }

        // Once the move is done and holding at rest, the axis settles to
        // StandStill with no motion flags set.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: end,
                velocity: 0.0,
                enabled: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].state, AxisState::StandStill);
        assert_eq!(fb[0].motion, MotionFlags::default());
    }

    #[test]
    fn motion_flags_track_a_real_trapezoidal_profile_positive_direction() {
        assert_motion_flags_track_profile(0.0, 100.0);
    }

    #[test]
    fn motion_flags_track_a_real_trapezoidal_profile_negative_direction() {
        assert_motion_flags_track_profile(0.0, -100.0);
    }

    #[test]
    fn axis_starts_switch_on_disabled() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::SwitchOnDisabled);
        assert_eq!(fb[0].state, AxisState::Disabled);
    }

    #[test]
    fn enabling_steps_through_ds402_states_one_per_cycle() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        let enable = |sim: &mut SimAxisGroup| {
            sim.exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                ..Default::default()
            }])
            .unwrap()[0]
                .ds402_state
        };

        assert_eq!(enable(&mut sim), Ds402State::ReadyToSwitchOn);
        assert_eq!(enable(&mut sim), Ds402State::SwitchedOn);
        assert_eq!(enable(&mut sim), Ds402State::OperationEnabled);
        // Once there, further `enabled: true` cycles are a steady no-op.
        assert_eq!(enable(&mut sim), Ds402State::OperationEnabled);
    }

    #[test]
    fn disabling_steps_back_down_one_per_cycle() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);

        let disable = |sim: &mut SimAxisGroup| {
            sim.exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                ..Default::default()
            }])
            .unwrap()[0]
                .ds402_state
        };

        assert_eq!(disable(&mut sim), Ds402State::SwitchedOn);
        assert_eq!(disable(&mut sim), Ds402State::ReadyToSwitchOn);
        assert_eq!(disable(&mut sim), Ds402State::SwitchOnDisabled);
        // Once there, further `enabled: false` cycles are a steady no-op.
        assert_eq!(disable(&mut sim), Ds402State::SwitchOnDisabled);
    }

    #[test]
    fn axis_state_is_disabled_throughout_the_enable_sequence_until_operation_enabled() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        for _ in 0..2 {
            // First two cycles land in ReadyToSwitchOn/SwitchedOn — both
            // still map to AxisState::Disabled, not StandStill, since the
            // axis genuinely can't accept motion commands yet.
            let fb = sim
                .exchange(&[AxisSetpoint {
                    position: 0.0,
                    velocity: 0.0,
                    enabled: true,
                    ..Default::default()
                }])
                .unwrap();
            assert_eq!(fb[0].state, AxisState::Disabled);
        }
        // Third cycle reaches OperationEnabled, and only now does the
        // coarser AxisState flip to StandStill.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::OperationEnabled);
        assert_eq!(fb[0].state, AxisState::StandStill);
    }

    #[test]
    fn a_disabled_axis_cannot_move_no_matter_what_velocity_is_commanded() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        // Never enabled, yet commanding a large velocity every cycle.
        for _ in 0..10 {
            let fb = sim
                .exchange(&[AxisSetpoint {
                    position: 0.0,
                    velocity: 100.0,
                    ..Default::default()
                }])
                .unwrap();
            assert_eq!(fb[0].position, 0.0);
            assert_eq!(fb[0].velocity, 0.0);
            assert_eq!(fb[0].state, AxisState::Disabled);
            assert_eq!(fb[0].motion, MotionFlags::default());
        }
    }

    #[test]
    fn disabling_from_standstill_does_not_fault() {
        // An axis that was never moving steps down gracefully; contrast
        // `disabling_mid_motion_faults_and_freezes_position`.
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::SwitchedOn);
        assert!(fb[0].fault.is_none());
    }

    #[test]
    fn disabling_mid_motion_faults_and_freezes_position() {
        let mut sim = SimAxisGroup::new(1, 1.0); // dt = 1s for easy arithmetic
        warm_up_enabled(&mut sim, 1);

        // Move for two cycles at 5 mm/s.
        let move_setpoint = AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
            enabled: true,
            ..Default::default()
        };
        sim.exchange(&[move_setpoint]).unwrap();
        let fb = sim.exchange(&[move_setpoint]).unwrap();
        assert_eq!(fb[0].position, 10.0);

        // Disable while still commanding the same nonzero velocity.
        let disable_setpoint = AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
            ..Default::default()
        };
        let fb = sim.exchange(&[disable_setpoint]).unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::FaultReactionActive);
        assert_eq!(fb[0].fault, Some(AxisFault::DisabledWhileMoving));
        assert_eq!(fb[0].state, AxisState::ErrorStop);
        assert_eq!(fb[0].position, 10.0);
        assert_eq!(fb[0].velocity, 0.0);

        // One cycle later, FaultReactionActive automatically advances to
        // Fault — no controlword input needed.
        let fb = sim.exchange(&[disable_setpoint]).unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::Fault);
        assert_eq!(fb[0].fault, Some(AxisFault::DisabledWhileMoving));
        assert_eq!(
            fb[0].position, 10.0,
            "position must stay frozen while faulted"
        );

        // Stuck at Fault — toggling `enabled` alone doesn't get it out.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::Fault);
    }

    #[test]
    fn fault_reset_clears_fault_but_does_not_re_enable() {
        let mut sim = SimAxisGroup::new(1, 1.0);
        warm_up_enabled(&mut sim, 1);

        let move_setpoint = AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
            enabled: true,
            ..Default::default()
        };
        sim.exchange(&[move_setpoint]).unwrap();

        let disable_setpoint = AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
            ..Default::default()
        };
        sim.exchange(&[disable_setpoint]).unwrap(); // -> FaultReactionActive
        let fb = sim.exchange(&[disable_setpoint]).unwrap(); // -> Fault
        assert_eq!(fb[0].ds402_state, Ds402State::Fault);

        // Resetting while still requesting enabled=false clears the fault
        // and lands in SwitchOnDisabled — resetting does not itself
        // re-enable the axis.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                fault_reset: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::SwitchOnDisabled);
        assert_eq!(fb[0].fault, None);
        assert_eq!(fb[0].state, AxisState::Disabled);

        // A fresh enable afterward runs the normal 3-cycle sequence again,
        // same as any other enable from cold — reset alone didn't skip it.
        let enable_setpoint = AxisSetpoint {
            position: 0.0,
            velocity: 0.0,
            enabled: true,
            ..Default::default()
        };
        assert_eq!(
            sim.exchange(&[enable_setpoint]).unwrap()[0].ds402_state,
            Ds402State::ReadyToSwitchOn
        );
        assert_eq!(
            sim.exchange(&[enable_setpoint]).unwrap()[0].ds402_state,
            Ds402State::SwitchedOn
        );
        assert_eq!(
            sim.exchange(&[enable_setpoint]).unwrap()[0].ds402_state,
            Ds402State::OperationEnabled
        );
    }

    #[test]
    fn fault_reset_is_a_no_op_while_not_faulted() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                fault_reset: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::OperationEnabled);
        assert!(fb[0].fault.is_none());
    }

    #[test]
    fn stopping_flag_reports_stopping_instead_of_discrete_motion_while_moving() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);

        // Moving normally: DiscreteMotion, as always.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 10.0,
                enabled: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].state, AxisState::DiscreteMotion);
        assert_eq!(fb[0].ds402_state, Ds402State::OperationEnabled);

        // Same commanded motion, but `stopping: true` this cycle: the
        // coarser AxisState reports Stopping instead — Ds402State is
        // unaffected, still OperationEnabled, and MotionFlags still
        // reflect the real physical deceleration.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 5.0,
                enabled: true,
                stopping: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].state, AxisState::Stopping);
        assert_eq!(fb[0].ds402_state, Ds402State::OperationEnabled);
        assert!(fb[0].motion.decelerating);

        // Once at rest, StandStill — the stopping flag no longer matters.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                stopping: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].state, AxisState::StandStill);
    }

    #[test]
    fn stopping_flag_is_a_no_op_when_not_moving_or_not_operational() {
        // stopping=true while already at rest: still StandStill, not
        // Stopping — there's nothing to stop.
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                stopping: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].state, AxisState::StandStill);

        // stopping=true on a disabled axis: still Disabled, not Stopping —
        // a stop request is meaningless for an axis that was never moving
        // in the first place.
        let mut sim = SimAxisGroup::new(1, 0.004);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                stopping: true,
                ..Default::default()
            }])
            .unwrap();
        assert_eq!(fb[0].state, AxisState::Disabled);
    }
}
