//! `backend-sim`: a software `AxisGroup` implementation — no hardware, no
//! network, just a plant model you can develop and test against.
//!
//! # The plant model
//!
//! Each cycle, [`SimAxisGroup`] integrates the *velocity* setpoint into its
//! own position state (`position += velocity * dt`), rather than snapping
//! straight to the commanded position. That's still about as simple as a
//! plant model gets — no lag, no dynamics — but it's genuinely stateful,
//! dt-stepping simulation (see the `NOTE (dt seam)` comment in
//! `motion-core`'s `trajectory.rs`), not a pass-through. Euler-integrating a
//! velocity profile that's actually a closed-form quadratic during
//! accel/decel leaves a small, real discretization error between the
//! planner's exact position and the sim's integrated one — a genuine (if
//! tiny) following error to look at once viz exists, instead of two
//! identical lines.
//!
//! One consequence worth being aware of: this plant model only reads
//! `AxisSetpoint::velocity`, never `AxisSetpoint::position` — it's
//! dead-reckoning position from commanded velocity, not tracking a position
//! target. That's the opposite of how a real EtherCAT drive in CSP mode
//! actually works (position is the primary command there; the drive's own
//! internal servo loop is what supplies velocity feed-forward and is where
//! real following error comes from). `backend-ethercat`, when it exists,
//! will consume these fields the other way around. This sim is a stand-in
//! for exercising the dt-stepping seam now, not a faithful physical model —
//! that's what Step 6 ("richer sim") is for.
//!
//! Richer dynamics (second-order lag, following-error limits) are a
//! deliberately later step — see CLAUDE.md's roadmap. Faults are no longer
//! entirely deferred: see `step_ds402` for the one fault this sim can raise.

use axis_backend::{
    AxisFault, AxisFeedback, AxisGroup, AxisGroupError, AxisSetpoint, AxisState, Ds402State,
    MotionFlags,
};

/// Below this speed, an axis counts as "at rest" for [`AxisState`] purposes.
const STANDSTILL_EPS: f64 = 1e-6;

/// Below this per-cycle change in commanded speed, the axis counts as at
/// constant velocity rather than accelerating/decelerating. Real ramps move
/// by many multiples of this every cycle, so a tiny epsilon is enough to
/// absorb float noise without misclassifying an actual ramp as "constant".
const ACCEL_EPS: f64 = 1e-9;

/// Advances a DS402 power-state by at most one standard transition per
/// call.
///
/// While no fault is in play, this steps toward `SwitchOnDisabled ->
/// ReadyToSwitchOn -> SwitchedOn -> OperationEnabled` while `enabled` is
/// `true`, or back down the same three states while it's `false`.
/// Already-at-target is a no-op either way.
///
/// Real masters negotiate this the same way: each controlword write moves
/// the drive at most one standard transition, confirmed by the next
/// statusword read, rather than jumping straight there — so stepping one
/// state per `exchange()` cycle here isn't a simplification for its own
/// sake, it's what the real protocol actually does. Coming down, DS402 also
/// offers direct multi-state shortcuts (e.g. "Disable Voltage" drops
/// straight to `SwitchOnDisabled` from any of the three states above it) —
/// this always takes the granular one-state-at-a-time path instead, since
/// that's the more interesting (and more testable) case, and a real master
/// is free to choose either.
///
/// `fault_detected` (computed by the caller — see `exchange`) preempts
/// everything else, from any state, the same way a real fault does (DS402
/// transition 13). From there: `FaultReactionActive` always advances
/// automatically to `Fault` one cycle later (transition 14) — no
/// controlword input needed, mirroring a real drive's brief internal
/// fault-handling window. `Fault` only leaves via `fault_reset`
/// (transition 15, to `SwitchOnDisabled`); toggling `enabled` alone does
/// nothing there, matching a real operator having to acknowledge a fault
/// before re-enabling.
///
/// `QuickStopActive` isn't reachable yet — no quick-stop command exists —
/// held in place if ever reached.
fn step_ds402(current: Ds402State, enabled: bool, fault_reset: bool, fault_detected: bool) -> Ds402State {
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
    /// `dt` is a configuration value set by the run loop, not user input —
    /// misconfiguring it is a programming error, so this panics rather than
    /// returning `Result` (contrast `TrapezoidalProfile::new`, which
    /// validates untrusted terminal input).
    pub fn new(num_axes: usize, dt: f64) -> Self {
        assert!(dt > 0.0, "dt must be positive, got {dt}");
        Self {
            dt,
            feedback: vec![
                AxisFeedback {
                    position: 0.0,
                    velocity: 0.0,
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
            // A real drive can't safely just cut its power stage while
            // actively moving the way it safely can from rest — treat that
            // as a fault rather than a graceful disable. `fb.velocity`
            // still holds last cycle's *actual* speed here, before this
            // cycle overwrites it below.
            let was_moving = fb.velocity.abs() > STANDSTILL_EPS;
            let fault_detected = fb.ds402_state == Ds402State::OperationEnabled && !sp.enabled && was_moving;

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

            // No lag in this plant, so the commanded setpoint *is* the
            // axis's actual speed each cycle — but only while the power
            // stage is actually live (`operational`); otherwise the axis
            // physically cannot move, no matter what velocity is
            // commanded, the same way a real disabled (or faulted) drive
            // ignores motion commands entirely.
            let prev_speed = fb.velocity;
            let speed = if operational { sp.velocity } else { 0.0 };

            if operational {
                fb.position += speed * self.dt;
            }
            fb.velocity = speed;

            let moving = speed.abs() > STANDSTILL_EPS;
            fb.state = fb.ds402_state.axis_state(moving);

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
    /// so it's `OperationEnabled` before a test's real assertions begin.
    /// Axes now start `SwitchOnDisabled` (see `SimAxisGroup::new`), so any
    /// test that isn't specifically exercising the enable sequence itself
    /// needs this first — otherwise its very first `exchange()` call would
    /// still be mid-startup, not moving yet regardless of what velocity it
    /// commands. See `enabling_steps_through_ds402_states_one_per_cycle`
    /// below for a test of the sequence itself.
    fn warm_up_enabled(sim: &mut SimAxisGroup, num_axes: usize) {
        for _ in 0..3 {
            let setpoints = vec![
                AxisSetpoint {
                    position: 0.0,
                    velocity: 0.0,
                    enabled: true,
                    fault_reset: false,
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
            fault_reset: false,
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
        // A wildly different commanded position is ignored entirely; only
        // velocity integrates. See the module doc for why.
        let setpoints = [AxisSetpoint {
            position: 999.0,
            velocity: -3.0,
            enabled: true,
            fault_reset: false,
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
                fault_reset: false,
            },
            AxisSetpoint {
                position: 0.0,
                velocity: -4.0,
                enabled: true,
                fault_reset: false,
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
                fault_reset: false,
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
                fault_reset: false,
            }])
            .unwrap();
        assert!(fb[0].fault.is_none());
    }

    /// Drives one axis through a hand-crafted speed sequence (ramp up,
    /// cruise, ramp down, stop) and checks `AxisState`/`MotionFlags` after
    /// every cycle. Isolates the flag-transition logic itself, independent
    /// of any real trajectory shape (see the trapezoidal-profile tests below
    /// for that).
    fn drive_and_check(sim: &mut SimAxisGroup, velocities: &[f64], expected: &[(AxisState, MotionFlags)]) {
        assert_eq!(velocities.len(), expected.len());
        for (i, (&velocity, &(want_state, want_motion))) in
            velocities.iter().zip(expected).enumerate()
        {
            let fb = sim
                .exchange(&[AxisSetpoint {
                    position: 0.0,
                    velocity,
                    enabled: true,
                    fault_reset: false,
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
        let accel = (AxisState::DiscreteMotion, MotionFlags { accelerating: true, constant_velocity: false, decelerating: false });
        let cruise = (AxisState::DiscreteMotion, MotionFlags { accelerating: false, constant_velocity: true, decelerating: false });
        let decel = (AxisState::DiscreteMotion, MotionFlags { accelerating: false, constant_velocity: false, decelerating: true });
        let rest = (AxisState::StandStill, MotionFlags::default());
        let expected = [rest, accel, accel, cruise, cruise, decel, rest];
        drive_and_check(&mut sim, &velocities, &expected);
    }

    #[test]
    fn state_and_motion_flags_track_a_hand_crafted_ramp_negative_direction() {
        // Mirror image of the positive-direction case: the flags are
        // defined on |speed| increasing/decreasing, not signed velocity, so
        // this should classify identically to the positive case above.
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        let velocities = [0.0, -2.0, -4.0, -4.0, -4.0, -2.0, 0.0];
        let accel = (AxisState::DiscreteMotion, MotionFlags { accelerating: true, constant_velocity: false, decelerating: false });
        let cruise = (AxisState::DiscreteMotion, MotionFlags { accelerating: false, constant_velocity: true, decelerating: false });
        let decel = (AxisState::DiscreteMotion, MotionFlags { accelerating: false, constant_velocity: false, decelerating: true });
        let rest = (AxisState::StandStill, MotionFlags::default());
        let expected = [rest, accel, accel, cruise, cruise, decel, rest];
        drive_and_check(&mut sim, &velocities, &expected);
    }

    /// Drives a real `motion_core::TrapezoidalProfile`'s sampled velocity
    /// through `SimAxisGroup`, cycle by cycle at the app's real 250 Hz
    /// control rate, and spot-checks state/motion flags at points well
    /// inside each phase. Spot checks (rather than asserting every single
    /// cycle) sidestep phase-boundary ambiguity: `phase_at(t)` switches
    /// labels at an exact continuous-time boundary, but the *cycle* that
    /// straddles that boundary can legitimately still show the previous
    /// phase's flag (e.g. the first cycle `phase_at` calls "Cruise" can
    /// still have a higher speed than the cycle before it, so it's fair for
    /// `accelerating` to still be true on that one cycle).
    fn assert_motion_flags_track_profile(start: f64, end: f64) {
        use motion_core::{MotionPhase, TrapezoidalProfile};

        let dt = 1.0 / 250.0;
        // max_speed=50, accel=decel=200 => accel phase 0..0.25s, cruise
        // 0.25..2.0s, decel 2.0..2.25s, total 2.25s duration (same numbers
        // as CLAUDE.md's worked 0->100mm example).
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
                    fault_reset: false,
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
                fault_reset: false,
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
        // A fresh axis has never had `exchange()` called on it — check the
        // very first construction-time state directly, not just after some
        // `enabled: true` request.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: false,
                fault_reset: false,
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
                fault_reset: false,
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
                enabled: false,
                fault_reset: false,
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
                    fault_reset: false,
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
                fault_reset: false,
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::OperationEnabled);
        assert_eq!(fb[0].state, AxisState::StandStill);
    }

    #[test]
    fn a_disabled_axis_cannot_move_no_matter_what_velocity_is_commanded() {
        let mut sim = SimAxisGroup::new(1, 0.004);
        // Never enabled — stays SwitchOnDisabled — yet keeps commanding a
        // large velocity every cycle, matching a run loop that doesn't
        // gate its own setpoints on backend-confirmed state.
        for _ in 0..10 {
            let fb = sim
                .exchange(&[AxisSetpoint {
                    position: 0.0,
                    velocity: 100.0,
                    enabled: false,
                    fault_reset: false,
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
        // Disabling an axis that was never actually moving (commanded
        // velocity stayed 0 the whole time) is the safe, graceful case —
        // no fault, just the normal step-down sequence. Distinguishes this
        // from `disabling_mid_motion_faults_and_freezes_position` below,
        // where the axis genuinely was moving.
        let mut sim = SimAxisGroup::new(1, 0.004);
        warm_up_enabled(&mut sim, 1);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: false,
                fault_reset: false,
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
            fault_reset: false,
        };
        sim.exchange(&[move_setpoint]).unwrap();
        let fb = sim.exchange(&[move_setpoint]).unwrap();
        assert_eq!(fb[0].position, 10.0);

        // Disable while still commanding the same nonzero velocity — a real
        // drive can't safely cut power mid-motion the way it can from rest,
        // so this faults rather than gracefully disabling. Position freezes
        // immediately either way.
        let disable_setpoint = AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
            enabled: false,
            fault_reset: false,
        };
        let fb = sim.exchange(&[disable_setpoint]).unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::FaultReactionActive);
        assert_eq!(fb[0].fault, Some(AxisFault::DisabledWhileMoving));
        assert_eq!(fb[0].state, AxisState::ErrorStop);
        assert_eq!(fb[0].position, 10.0);
        assert_eq!(fb[0].velocity, 0.0);

        // One cycle later, FaultReactionActive automatically advances to
        // Fault (DS402 transition 14) — no controlword input needed.
        let fb = sim.exchange(&[disable_setpoint]).unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::Fault);
        assert_eq!(fb[0].fault, Some(AxisFault::DisabledWhileMoving));
        assert_eq!(fb[0].position, 10.0, "position must stay frozen while faulted");

        // Stuck at Fault — toggling `enabled` alone doesn't get it out.
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 0.0,
                enabled: true,
                fault_reset: false,
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
            fault_reset: false,
        };
        sim.exchange(&[move_setpoint]).unwrap();

        let disable_setpoint = AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
            enabled: false,
            fault_reset: false,
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
                enabled: false,
                fault_reset: true,
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
            fault_reset: false,
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
            }])
            .unwrap();
        assert_eq!(fb[0].ds402_state, Ds402State::OperationEnabled);
        assert!(fb[0].fault.is_none());
    }
}
