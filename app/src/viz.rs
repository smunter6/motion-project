//! The viz window: a passive, read-only view over the shared [`History`]
//! buffer the control loop writes to via
//! [`RecordingAxisGroup`](crate::recording::RecordingAxisGroup). Input stays
//! terminal-driven — this window has no controls, it only plots.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use axis_backend::{AxisState, MotionFlags};
use eframe::egui;
use egui_plot::{Line, MarkerShape, Plot, PlotPoints, Points};

use crate::recording::History;

/// How often the window redraws (about 60 Hz). The control loop writes samples
/// at 250 Hz regardless; this is only how often they are drawn. It is a ceiling:
/// the achievable rate is set by frame cost, which [`MAX_PLOT_POINTS`] bounds.
const VIZ_REFRESH_PERIOD: std::time::Duration = std::time::Duration::from_millis(16);

/// Height of each per-axis time-series plot (position, velocity,
/// acceleration).
const PLOT_HEIGHT: f32 = 140.0;

/// The drawn mechanism: warm and opaque, so it reads as a physical object on
/// top of the two thin trace lines rather than as a third trace.
const ARM_COLOR: egui::Color32 = egui::Color32::from_rgb(230, 145, 45);
const JOINT_COLOR: egui::Color32 = egui::Color32::from_rgb(250, 250, 250);

/// A short, human-readable label for an axis's current [`AxisState`], folding
/// in [`MotionFlags`] for `DiscreteMotion`, the only state where they apply.
fn state_label(state: AxisState, motion: MotionFlags) -> &'static str {
    match state {
        AxisState::Disabled => "disabled",
        AxisState::ErrorStop => "error stop",
        AxisState::Stopping => "stopping",
        AxisState::StandStill => "standstill",
        AxisState::DiscreteMotion => {
            if motion.accelerating {
                "moving (accel)"
            } else if motion.decelerating {
                "moving (decel)"
            } else if motion.constant_velocity {
                "moving (cruise)"
            } else {
                "moving"
            }
        }
        AxisState::ContinuousMotion => "jogging",
        AxisState::SynchronizedMotion => "synchronized",
        AxisState::Homing => "homing",
    }
}

/// Roughly how many points any one plotted line carries.
///
/// The control loop records at 250 Hz and the buffer holds a minute of it, so
/// an undecimated line would be 15,000 points wide, all tessellated and
/// painted every frame across ~20 lines. Frame build time grows with the
/// buffer (6.4 ms at 2,200 samples, 24 ms at 19,700) and the frame rate falls
/// with it. At 400 points the cost no longer grows with the buffer (about
/// 5.7 ms, flat past 19,000 samples), and it is still more points than the
/// plots are pixels wide (four per row in a 1000 px window).
///
/// All samples are still recorded; this limits only how many are drawn.
///
/// Position and velocity use plain striding: they are smooth trajectories with
/// no transients for a stride to step over. Acceleration is the exception; see
/// [`in_plot_order_envelope`], which keeps up to 2 points per bucket and so
/// draws up to twice this many.
const MAX_PLOT_POINTS: usize = 400;

/// How many samples to skip between plotted points to stay under
/// [`MAX_PLOT_POINTS`]. Always at least 1 (plot everything).
fn plot_stride(len: usize) -> usize {
    (len / MAX_PLOT_POINTS).max(1)
}

/// Collects points that arrive **newest first** into plot order.
///
/// Decimation walks the buffer backwards so the stride is anchored at the live
/// end: the newest sample is always plotted, and any remainder falls off the
/// old end. Striding forwards would leave the leading tip of every trace up to
/// a stride stale, a visible lag behind the drawn arm, which uses the latest
/// pose.
fn in_plot_order(points: impl Iterator<Item = [f64; 2]>) -> PlotPoints<'static> {
    let mut points: Vec<[f64; 2]> = points.collect();
    points.reverse();
    PlotPoints::from(points)
}

/// As [`in_plot_order`], but reduces each stride-sized bucket to **both** its
/// minimum and its maximum sample rather than to one representative point.
///
/// Only the **measured** acceleration uses this. The commanded acceleration is
/// closed-form and continuous, with no one-sample transients to preserve, so
/// an envelope would only widen its smooth curve into a band; plain striding
/// is right there.
///
/// The measured trace is a first difference at the control rate
/// (`(v - v_prev)/dt`, dt = 4 ms). Any step in the velocity stream, such as a
/// redirect or an axis going non-operational (which forces speed to zero in one
/// cycle), becomes a *single-sample* spike far outside `max_acceleration`.
/// Plain striding would catch that spike in some frames and miss it in others
/// as the stride phase shifts, so an auto-fitting plot's y range would flip
/// back and forth. An envelope makes decimation phase-independent and keeps
/// the one-cycle transient visible, at the time it happened (`t` comes from the
/// chosen samples).
///
/// It keeps min *and* max rather than the largest |value|: reducing by
/// magnitude discards the sign, so a bucket straddling zero collapses to
/// whichever side is larger and consecutive buckets can pick opposite sides,
/// drawing a sign-changing trace as a rectified zigzag. Keeping both extremes
/// preserves the sign, and on a monotone stretch they are the bucket's
/// endpoints, so the output is a subsequence of the real samples in time order.
fn in_plot_order_envelope(
    points: impl Iterator<Item = [f64; 2]>,
    stride: usize,
) -> PlotPoints<'static> {
    PlotPoints::from(envelope_points(points, stride))
}

/// The body of [`in_plot_order_envelope`], before it becomes an opaque
/// `PlotPoints`, so its ordering property can be tested.
fn envelope_points(points: impl Iterator<Item = [f64; 2]>, stride: usize) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = Vec::new();
    let mut lo: Option<[f64; 2]> = None;
    let mut hi: Option<[f64; 2]> = None;
    let mut n = 0;

    // Emitted in arrival order (newest-first) so the single reverse below
    // puts both the buckets and each bucket's pair into time order.
    let flush = |out: &mut Vec<[f64; 2]>, lo: Option<[f64; 2]>, hi: Option<[f64; 2]>| match (lo, hi)
    {
        (Some(a), Some(b)) if a[0] == b[0] => out.push(a),
        (Some(a), Some(b)) if a[0] > b[0] => out.extend([a, b]),
        (Some(a), Some(b)) => out.extend([b, a]),
        _ => {}
    };

    for p in points {
        if lo.is_none_or(|best| p[1] < best[1]) {
            lo = Some(p);
        }
        if hi.is_none_or(|best| p[1] > best[1]) {
            hi = Some(p);
        }
        n += 1;
        if n == stride {
            flush(&mut out, lo.take(), hi.take());
            n = 0;
        }
    }
    // A trailing partial bucket, at the old end nobody is looking at.
    flush(&mut out, lo, hi);

    out.reverse();
    out
}

/// One time-series plot with a target and an actual trace; every per-axis plot
/// has this shape.
///
/// Scrolling is disabled on the plots because scroll-to-pan would fight the
/// page's own scroll area. Drag-to-pan and box-zoom still work.
fn time_series_plot(
    ui: &mut egui::Ui,
    id: String,
    width: f32,
    height: f32,
    target: PlotPoints<'static>,
    actual: PlotPoints<'static>,
) {
    Plot::new(id)
        .width(width)
        .height(height)
        .allow_scroll(false)
        .show(ui, |plot_ui| {
            plot_ui.line(Line::new("target", target));
            plot_ui.line(Line::new("actual", actual));
        });
}

/// The most recent *measured* joint values for a 2-axis group, or `None`
/// before any feedback has arrived. Feedback rather than setpoint, so the
/// drawn mechanism shows where the machine is.
fn latest_joints(history: &History, group: &crate::AxisGroupDef) -> Option<[f64; 2]> {
    Some([
        history.axis(group.axes[0]).back()?.feedback.position,
        history.axis(group.axes[1]).back()?.feedback.position,
    ])
}

/// A group's task-space (TCP) point from its members' joint values, via the
/// group's kinematic model. `None` if the values are rejected (non-finite
/// feedback), in which case the sample is dropped from the plot.
///
/// Two dimensions because only 2-axis groups get a plane plot (`egui_plot` is
/// 2D).
fn tcp_xy(group: &crate::AxisGroupDef, joints: [f64; 2]) -> Option<[f64; 2]> {
    let joints = motion_core::KinematicVector::from_slice(&joints).ok()?;
    let point = group.kinematics.forward_position(joints);
    Some([point.as_slice()[0], point.as_slice()[1]])
}

pub struct VizApp {
    history: Arc<Mutex<History>>,
    shutdown: Arc<AtomicBool>,
    num_axes: usize,
}

impl VizApp {
    pub fn new(history: Arc<Mutex<History>>, shutdown: Arc<AtomicBool>, num_axes: usize) -> Self {
        Self {
            history,
            shutdown,
            num_axes,
        }
    }
}

impl eframe::App for VizApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();

        // The terminal's `quit` command ends the control-loop thread, which
        // closes this window.
        if self.shutdown.load(Ordering::Relaxed) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

        egui::CentralPanel::default().show(ui, |ui| {
            let history = self.history.lock().unwrap();

            // The content (an XY plot and status box per axis group, plus
            // plots per axis) grows with the number of groups and axes, so it
            // scrolls vertically rather than clipping.
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.horizontal(|ui| {
                    // One XY plot per axis group (see `crate::AXIS_GROUPS`).
                    // Only groups of exactly 2 axes get a plane plot; a group
                    // of 3 or more would need a projection or a 3D view.
                    for group in crate::AXIS_GROUPS {
                        if group.axes.len() != 2 {
                            continue;
                        }
                        let (x_axis, y_axis) = (group.axes[0], group.axes[1]);
                        // The mechanism's current pose, drawn over the traces.
                        // None for a Cartesian group, which has no linkage.
                        let pose = latest_joints(&history, group)
                            .and_then(|joints| motion_core::KinematicVector::from_slice(&joints).ok())
                            .map(|joints| group.kinematics.linkage(joints))
                            .filter(|linkage| !linkage.is_empty());
                        ui.vertical(|ui| {
                            ui.heading(format!(
                                "{} — TCP X/Y (axis{x_axis}, axis{y_axis})",
                                group.name
                            ));
                            // The History buffer holds joint values (what
                            // crosses the AxisGroup seam), so each sample goes
                            // through forward kinematics to become a point in
                            // the plane. For an arm, raw joint angles would
                            // trace nothing meaningful.
                            let plot = Plot::new(format!("xy_plot_{}", group.name))
                                .view_aspect(1.5)
                                .height(220.0)
                                // See `time_series_plot`.
                                .allow_scroll(false);
                            // A drawn mechanism needs equal scaling on both
                            // axes or its links change length as the plot
                            // rescales. Applied only where there is something
                            // to draw; other plots auto-fit.
                            let plot = if pose.is_some() {
                                plot.data_aspect(1.0)
                            } else {
                                plot
                            };
                            plot.show(ui, |plot_ui| {
                                    // Both buffers are written in the same
                                    // cycle, so zipping them reversed pairs
                                    // each sample with its counterpart.
                                    let stride = plot_stride(history.axis(x_axis).len());
                                    let pairs = || {
                                        history
                                            .axis(x_axis)
                                            .iter()
                                            .rev()
                                            .zip(history.axis(y_axis).iter().rev())
                                            .step_by(stride)
                                    };
                                    let target = in_plot_order(pairs().filter_map(|(a, b)| {
                                        tcp_xy(group, [a.setpoint.position, b.setpoint.position])
                                    }));
                                    let actual = in_plot_order(pairs().filter_map(|(a, b)| {
                                        tcp_xy(group, [a.feedback.position, b.feedback.position])
                                    }));
                                    plot_ui.line(Line::new("target", target));
                                    plot_ui.line(Line::new("actual", actual));

                                    // Drawn last so the arm sits on top of the
                                    // traces.
                                    if let Some(linkage) = &pose {
                                        let points: Vec<[f64; 2]> = linkage
                                            .points()
                                            .iter()
                                            .map(|p| [p.as_slice()[0], p.as_slice()[1]])
                                            .collect();
                                        plot_ui.line(
                                            Line::new("arm", PlotPoints::from(points.clone()))
                                                .width(4.0)
                                                .color(ARM_COLOR),
                                        );
                                        // Base, elbow and tool as visible
                                        // pivots, so a near-straight arm
                                        // doesn't read as a single line.
                                        plot_ui.points(
                                            Points::new("joints", PlotPoints::from(points))
                                                .radius(5.0)
                                                .shape(MarkerShape::Circle)
                                                .filled(true)
                                                .color(JOINT_COLOR),
                                        );
                                    }
                                });
                        });
                    }

                    // Axes and groups are separate columns: they show
                    // different things (each axis's state vs. each group's
                    // TCP).
                    ui.vertical(|ui| {
                        ui.heading("axes");
                        for axis in 0..self.num_axes {
                            let last = history.axis(axis).back();
                            ui.group(|ui| {
                                ui.set_min_width(160.0);
                                ui.strong(format!("axis{axis}"));
                                // Each axis labels itself in its own units.
                                let units = crate::AXIS_CONFIGS[axis].units;
                                match last {
                                    Some(s) => {
                                        ui.label(format!(
                                            "position: {:8.3} {units}",
                                            s.feedback.position
                                        ));
                                        ui.label(format!(
                                            "velocity: {:8.3} {units}/s",
                                            s.feedback.velocity
                                        ));
                                        ui.label(format!(
                                            "status:   {}",
                                            state_label(s.feedback.state, s.feedback.motion)
                                        ));
                                        // Raw DS402 state alongside the coarser
                                        // status line above.
                                        ui.weak(format!("ds402:    {:?}", s.feedback.ds402_state));
                                        if let Some(fault) = s.feedback.fault {
                                            ui.colored_label(
                                                egui::Color32::RED,
                                                format!("fault: {fault:?}"),
                                            );
                                        }
                                    }
                                    None => {
                                        ui.label(format!("position:    0.000 {units}"));
                                        ui.label(format!("velocity:    0.000 {units}/s"));
                                        ui.label("status:   disabled");
                                    }
                                }
                            });
                        }
                    });

                    // One box per axis group, built from the same per-axis
                    // History feedback as the axis boxes. Viz only sees the
                    // AxisGroup::exchange() seam, not app's group-move
                    // bookkeeping, so it can't show whether a group move is
                    // active (the terminal `status` command can); this is a
                    // per-member position/velocity rollup.
                    ui.vertical(|ui| {
                        ui.heading("groups");
                        for group in crate::AXIS_GROUPS {
                            ui.group(|ui| {
                                ui.set_min_width(160.0);
                                ui.strong(group.name);
                                for &axis in group.axes {
                                    let last = history.axis(axis).back();
                                    let (position, velocity) = match last {
                                        Some(s) => (s.feedback.position, s.feedback.velocity),
                                        None => (0.0, 0.0),
                                    };
                                    let units = crate::AXIS_CONFIGS[axis].units;
                                    ui.label(format!(
                                        "axis{axis}: {position:8.3} {units}  {velocity:8.3} {units}/s"
                                    ));
                                }
                                // The same members as one TCP point.
                                if group.axes.len() == 2 {
                                    let joints = [
                                        history.axis(group.axes[0]).back().map_or(0.0, |s| {
                                            s.feedback.position
                                        }),
                                        history.axis(group.axes[1]).back().map_or(0.0, |s| {
                                            s.feedback.position
                                        }),
                                    ];
                                    if let Some([x, y]) = tcp_xy(group, joints) {
                                        ui.weak(format!("TCP:   {x:8.3}, {y:8.3} mm"));
                                    }
                                }
                            });
                        }
                    });
                });
                ui.separator();

                let col_width = ui.available_width() / self.num_axes as f32 - 8.0;
                ui.horizontal(|ui| {
                    for axis in 0..self.num_axes {
                        ui.vertical(|ui| {
                            ui.heading(format!("axis{axis}"));
                            let units = crate::AXIS_CONFIGS[axis].units;
                            let stride = plot_stride(history.axis(axis).len());
                            // Newest-first samples; every trace in this column
                            // is built from them.
                            let full = |pick: fn(&crate::recording::Sample) -> f64| {
                                history.axis(axis).iter().rev().map(move |s| [s.t, pick(s)])
                            };
                            let series =
                                |pick: fn(&crate::recording::Sample) -> f64| {
                                    full(pick).step_by(stride)
                                };

                            ui.label(format!("position ({units})"));
                            time_series_plot(
                                ui,
                                format!("pos_plot_{axis}"),
                                col_width,
                                PLOT_HEIGHT,
                                in_plot_order(series(|s| s.setpoint.position)),
                                in_plot_order(series(|s| s.feedback.position)),
                            );

                            ui.label(format!("velocity ({units}/s)"));
                            time_series_plot(
                                ui,
                                format!("vel_plot_{axis}"),
                                col_width,
                                PLOT_HEIGHT,
                                in_plot_order(series(|s| s.setpoint.velocity)),
                                in_plot_order(series(|s| s.feedback.velocity)),
                            );

                            // Both traces come straight off the seam. The target
                            // is the planner's closed-form acceleration, not a
                            // difference of plotted samples. This plot shows
                            // jerk limiting: unfiltered, the target steps
                            // between 0 and +/-a_max; filtered, it ramps.
                            //
                            // The *actual* trace is a plant estimate and looks
                            // coarser; see `AxisFeedback`.
                            ui.label(format!("acceleration ({units}/s²)"));
                            time_series_plot(
                                ui,
                                format!("acc_plot_{axis}"),
                                col_width,
                                PLOT_HEIGHT,
                                in_plot_order(series(|s| s.setpoint.acceleration)),
                                in_plot_order_envelope(full(|s| s.feedback.acceleration), stride),
                            );
                        });
                        ui.separator();
                    }
                });
            });
        });

        // Repaint on a timer rather than only on input, so the plots animate
        // as the control loop writes samples.
        ctx.request_repaint_after(VIZ_REFRESH_PERIOD);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Input arrives newest-first; output must be in time order, or the plot
    /// draws a zigzag out of smooth data.
    #[test]
    fn envelope_output_is_in_time_order() {
        // A sign-changing, non-monotone trace, like a path's centripetal
        // acceleration.
        let samples: Vec<[f64; 2]> = (0..500)
            .map(|k| {
                let t = k as f64 * 0.004;
                [t, 100.0 * (t * 2.0).sin()]
            })
            .collect();

        for stride in [1, 3, 7, 40] {
            let out = envelope_points(samples.iter().rev().copied(), stride);
            assert!(
                out.windows(2).all(|w| w[0][0] <= w[1][0]),
                "stride {stride}: output not in time order"
            );
            // Every emitted point is a real sample, not an interpolation.
            assert!(out.iter().all(|p| samples.contains(p)));
        }
    }

    /// A one-cycle spike (what a measured acceleration does when velocity
    /// steps) must survive decimation wherever it lands in a bucket, which
    /// plain striding can't guarantee.
    #[test]
    fn envelope_keeps_a_single_sample_spike_at_any_phase() {
        let stride = 10;
        for spike_at in 0..30 {
            let samples: Vec<[f64; 2]> = (0..300)
                .map(|k| {
                    let y = if k == spike_at { 12_500.0 } else { 50.0 };
                    [k as f64 * 0.004, y]
                })
                .collect();
            let out = envelope_points(samples.iter().rev().copied(), stride);
            assert!(
                out.iter().any(|p| p[1] == 12_500.0),
                "spike at index {spike_at} was decimated away"
            );
        }
    }

    /// A monotone stretch must come back as a line, not a band: the min and
    /// max of each bucket are its endpoints.
    #[test]
    fn envelope_leaves_a_ramp_alone() {
        let samples: Vec<[f64; 2]> = (0..200).map(|k| [k as f64, 2.0 * k as f64]).collect();
        let out = envelope_points(samples.iter().rev().copied(), 8);
        assert!(out.iter().all(|p| p[1] == 2.0 * p[0]));
    }
}
