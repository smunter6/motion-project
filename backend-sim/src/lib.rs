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
//! Richer dynamics (second-order lag, following-error limits, faults) are a
//! deliberately later step — see CLAUDE.md's roadmap.

use axis_backend::{AxisFeedback, AxisGroup, AxisGroupError, AxisSetpoint, AxisState, MotionFlags};

/// Below this speed, an axis counts as "at rest" for [`AxisState`] purposes.
const STANDSTILL_EPS: f64 = 1e-6;

/// Below this per-cycle change in commanded speed, the axis counts as at
/// constant velocity rather than accelerating/decelerating. Real ramps move
/// by many multiples of this every cycle, so a tiny epsilon is enough to
/// absorb float noise without misclassifying an actual ramp as "constant".
const ACCEL_EPS: f64 = 1e-9;

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
                    state: AxisState::StandStill,
                    motion: MotionFlags::default(),
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
            // `fb.velocity` still holds last cycle's commanded speed here —
            // compare against this cycle's before overwriting it, to tell
            // whether the axis is speeding up, at constant speed, or
            // slowing down. No lag in this plant, so the commanded setpoint
            // *is* the axis's actual speed each cycle.
            let prev_speed = fb.velocity;
            let speed = sp.velocity;

            fb.position += speed * self.dt;
            fb.velocity = speed;

            let moving = speed.abs() > STANDSTILL_EPS;
            fb.state = match fb.fault {
                Some(_) => AxisState::ErrorStop,
                None if moving => AxisState::DiscreteMotion,
                None => AxisState::StandStill,
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

    #[test]
    fn integrates_velocity_into_position() {
        let mut sim = SimAxisGroup::new(1, 1.0); // dt = 1s for easy arithmetic
        let setpoints = [AxisSetpoint {
            position: 0.0,
            velocity: 5.0,
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
        // A wildly different commanded position is ignored entirely; only
        // velocity integrates. See the module doc for why.
        let setpoints = [AxisSetpoint {
            position: 999.0,
            velocity: -3.0,
        }];
        let fb = sim.exchange(&setpoints).unwrap();
        assert_eq!(fb[0].velocity, -3.0);
        assert!((fb[0].position - (-3.0 * 0.004)).abs() < 1e-12);
    }

    #[test]
    fn axes_integrate_independently() {
        let mut sim = SimAxisGroup::new(2, 1.0);
        let setpoints = [
            AxisSetpoint {
                position: 0.0,
                velocity: 10.0,
            },
            AxisSetpoint {
                position: 0.0,
                velocity: -4.0,
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
    fn no_faults_reported() {
        let mut sim = SimAxisGroup::new(1, 1.0);
        let fb = sim
            .exchange(&[AxisSetpoint {
                position: 0.0,
                velocity: 1.0,
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
}
