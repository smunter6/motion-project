//! A passive tap on the `AxisGroup` seam, for the viz window.
//!
//! [`RecordingAxisGroup`] wraps any backend and forwards `exchange()`
//! straight through, unchanged — the control loop's behavior is identical
//! with or without it. The only extra thing it does is copy each cycle's
//! setpoint/feedback pair into a shared [`History`] buffer, which the viz
//! window reads from its own thread to plot target vs. actual. This means
//! viz needs zero changes to the control loop's decision logic: swapping
//! in this wrapper at backend-construction time is the entire integration.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use axis_backend::{AxisFeedback, AxisGroup, AxisGroupError, AxisSetpoint};

/// One control cycle's target-vs-actual data for one axis.
#[derive(Clone, Copy)]
pub struct Sample {
    pub t: f64,
    pub target_position: f64,
    pub actual_position: f64,
    pub target_velocity: f64,
    pub actual_velocity: f64,
}

/// Bounded per-axis sample history, shared between the control-loop thread
/// (writer, via [`RecordingAxisGroup`]) and the viz thread (reader). Bounded
/// so a long-running app doesn't grow this without limit — old samples are
/// dropped once an axis's buffer is full.
pub struct History {
    start: Instant,
    capacity: usize,
    axes: Vec<VecDeque<Sample>>,
}

impl History {
    pub fn new(num_axes: usize, capacity: usize) -> Self {
        Self {
            start: Instant::now(),
            capacity,
            axes: (0..num_axes)
                .map(|_| VecDeque::with_capacity(capacity))
                .collect(),
        }
    }

    pub fn axis(&self, i: usize) -> &VecDeque<Sample> {
        &self.axes[i]
    }

    fn push(&mut self, i: usize, setpoint: &AxisSetpoint, feedback: &AxisFeedback) {
        let sample = Sample {
            t: self.start.elapsed().as_secs_f64(),
            target_position: setpoint.position,
            actual_position: feedback.position,
            target_velocity: setpoint.velocity,
            actual_velocity: feedback.velocity,
        };
        let buf = &mut self.axes[i];
        if buf.len() == self.capacity {
            buf.pop_front();
        }
        buf.push_back(sample);
    }
}

pub struct RecordingAxisGroup<B: AxisGroup> {
    inner: B,
    history: Arc<Mutex<History>>,
}

impl<B: AxisGroup> RecordingAxisGroup<B> {
    pub fn new(inner: B, history: Arc<Mutex<History>>) -> Self {
        Self { inner, history }
    }
}

impl<B: AxisGroup> AxisGroup for RecordingAxisGroup<B> {
    fn num_axes(&self) -> usize {
        self.inner.num_axes()
    }

    fn exchange(&mut self, setpoints: &[AxisSetpoint]) -> Result<&[AxisFeedback], AxisGroupError> {
        let feedback = self.inner.exchange(setpoints)?;
        let mut history = self.history.lock().unwrap();
        for (i, (sp, fb)) in setpoints.iter().zip(feedback.iter()).enumerate() {
            history.push(i, sp, fb);
        }
        Ok(feedback)
    }
}
