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

use axis_backend::{AxisFeedback, AxisGroup, AxisGroupError, AxisSetpoint};

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
            fb.position += sp.velocity * self.dt;
            fb.velocity = sp.velocity;
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
}
