//! The viz window: a passive, read-only view over the shared [`History`]
//! buffer the control loop writes to via [`RecordingAxisGroup`]. Input stays
//! terminal-driven — this window has no controls, it only plots.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use axis_backend::{AxisState, MotionFlags};
use eframe::egui;
use egui_plot::{Line, MarkerShape, Plot, PlotPoints, Points};

use crate::recording::History;

/// How often the window redraws. The control loop writes new samples at
/// `CONTROL_RATE_HZ` (250 Hz) regardless — this is purely how often that
/// data is *drawn*, so raising it costs frames, not control fidelity.
///
/// 60 Hz rather than anything higher: eframe's vsync caps the useful value
/// at the monitor refresh anyway.
///
/// Note this is a *ceiling*, not a guarantee — what the window actually
/// achieves is set by frame cost, which is why [`MAX_PLOT_POINTS`] exists
/// and matters far more than this constant does. Lowering this period is
/// almost never the fix for a window that feels slow.
const VIZ_REFRESH_PERIOD: std::time::Duration = std::time::Duration::from_millis(16);

/// The drawn mechanism: warm and opaque, so it reads as a physical object
/// on top of the two thin trace lines rather than as a third trace.
/// Height of each per-axis time-series plot. Three of them per axis now
/// (position, velocity, acceleration), so this is smaller than when there
/// were two — the row still has to fit beside its neighbours without the
/// column becoming a scroll of its own.
const PLOT_HEIGHT: f32 = 140.0;

const ARM_COLOR: egui::Color32 = egui::Color32::from_rgb(230, 145, 45);
const JOINT_COLOR: egui::Color32 = egui::Color32::from_rgb(250, 250, 250);

/// A short, human-readable label for an axis's current [`AxisState`],
/// folding in [`MotionFlags`] where they add information (only meaningful
/// alongside `DiscreteMotion` today).
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

/// Roughly how many points any one plotted line is allowed to carry.
///
/// The control loop records at 250 Hz and the buffer holds a minute of it,
/// so a line would otherwise be 15,000 points wide — around 60 per
/// horizontal pixel, all tessellated and painted every frame, across ~20
/// lines. That is what made the window feel slow, and it got worse the
/// longer it ran: **measured**, frame build time grew with the buffer
/// (6.4 ms at 2,200 samples to 24 ms at 19,700) while the window fell from
/// ~38 fps to ~9. Lock contention with the control loop was never a factor
/// at any point — 0.04 ms per frame throughout.
///
/// All samples are still *recorded*; this governs only how many are
/// *drawn*. At 400 the cost stops growing with the buffer entirely
/// (measured: ~5.7 ms build, ~40 fps, flat past 19,000 samples), and it is
/// still comfortably more points than these plots are pixels wide — they
/// sit four to a row in a 1000px window. Raising it buys nothing visible
/// and costs frames roughly linearly.
///
/// Plain striding rather than a min/max envelope per pixel bucket: these
/// are smooth trajectories, not noisy signals with transients that a stride
/// could step over.
const MAX_PLOT_POINTS: usize = 400;

/// How many samples to skip between plotted points to stay under
/// [`MAX_PLOT_POINTS`]. Always at least 1 (plot everything).
fn plot_stride(len: usize) -> usize {
    (len / MAX_PLOT_POINTS).max(1)
}

/// Collects points that arrive **newest first** into plot order.
///
/// Decimation walks the buffer backwards so the stride is anchored at the
/// live end: the newest sample is always plotted, and any remainder falls
/// off the old end where nobody is looking. Striding forwards instead would
/// leave the leading tip of every trace up to a stride stale — small, but
/// visible as a lag between the drawn arm (which uses the true latest pose)
/// and the trace it's supposed to be drawing.
fn in_plot_order(points: impl Iterator<Item = [f64; 2]>) -> PlotPoints<'static> {
    let mut points: Vec<[f64; 2]> = points.collect();
    points.reverse();
    PlotPoints::from(points)
}

/// One time-series plot with a target and an actual trace — the shape every
/// per-axis plot has.
///
/// `allow_scroll(false)` on all of them: scroll-to-pan fights the page's own
/// scroll area, so a wheel gesture aimed at reaching the plots below would
/// instead drag whichever plot the pointer happened to be over. Drag-to-pan
/// and box-zoom still work.
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
/// before any feedback has arrived. Feedback, not setpoint: the drawn
/// mechanism should show where the machine is, which is the one place in
/// viz that difference is visible as a shape rather than a gap between two
/// lines.
fn latest_joints(history: &History, group: &crate::AxisGroupDef) -> Option<[f64; 2]> {
    Some([
        history.axis(group.axes[0]).back()?.feedback.position,
        history.axis(group.axes[1]).back()?.feedback.position,
    ])
}

/// A group's task-space (TCP) point from its members' joint values, via the
/// group's own kinematic model — a pass-through for a Cartesian group, real
/// forward kinematics for an arm. `None` if the values are rejected
/// (non-finite feedback), in which case the sample is simply dropped from
/// the plot rather than drawn somewhere wrong.
///
/// Two dimensions because only 2-axis groups get a plane plot (`egui_plot`
/// is strictly 2D).
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

        // The terminal's `quit` command ends the control-loop thread; that's
        // this window's cue to close too, since it has no controls of its
        // own to drive a shutdown.
        if self.shutdown.load(Ordering::Relaxed) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

        egui::CentralPanel::default().show(ui, |ui| {
            let history = self.history.lock().unwrap();

            // The content below (an XY plot + status box per axis group,
            // plus a position/velocity plot pair per axis) keeps growing as
            // groups/axes are added — a vertical scroll area means it's
            // always reachable rather than silently clipped off the bottom
            // of the window, regardless of window size or axis count.
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.horizontal(|ui| {
                    // One XY plot per hard-coded axis group (see `crate::
                    // AXIS_GROUPS`), not a single plot hardcoded to axis0/axis1
                    // — this scales as more groups are added instead of staying
                    // pinned to whichever two axes happened to exist first.
                    // Only groups of exactly 2 axes get a plane plot; a 3+-axis
                    // group would need a projection or a 3D view, out of scope
                    // here.
                    for group in crate::AXIS_GROUPS {
                        if group.axes.len() != 2 {
                            continue;
                        }
                        let (x_axis, y_axis) = (group.axes[0], group.axes[1]);
                        // The mechanism's current pose, drawn over the
                        // traces. Empty for a Cartesian group, which has no
                        // linkage to show.
                        let pose = latest_joints(&history, group)
                            .and_then(|joints| motion_core::KinematicVector::from_slice(&joints).ok())
                            .map(|joints| group.kinematics.linkage(joints))
                            .filter(|linkage| !linkage.is_empty());
                        ui.vertical(|ui| {
                            ui.heading(format!(
                                "{} — TCP X/Y (axis{x_axis}, axis{y_axis})",
                                group.name
                            ));
                            // Both streams are joint values in the History
                            // buffer (that's what crosses the AxisGroup
                            // seam), so each sample goes through forward
                            // kinematics to become a point in the plane.
                            // Identity for a Cartesian group — the axes
                            // *are* X and Y — and the whole point for an
                            // arm, whose joint angles plotted raw would
                            // trace nothing meaningful.
                            let plot = Plot::new(format!("xy_plot_{}", group.name))
                                .view_aspect(1.5)
                                .height(220.0)
                                // Same reason as the time-series plots —
                                // see `time_series_plot`.
                                .allow_scroll(false);
                            // A drawn mechanism needs equal scaling on both
                            // axes or its links change length as the plot
                            // rescales — a rigid arm that visibly stretches
                            // is worse than no picture. Only applied where
                            // there's something to draw, so the plots that
                            // existed before this keep their auto-fit.
                            let plot = if pose.is_some() {
                                plot.data_aspect(1.0)
                            } else {
                                plot
                            };
                            plot.show(ui, |plot_ui| {
                                    // Both buffers are written the same
                                    // cycle, so zipping them reversed pairs
                                    // each sample with its own counterpart
                                    // from the newest end backwards.
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

                                    // Drawn last so the arm sits on top of
                                    // its own traces.
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
                                        // pivots — without them an arm
                                        // folded near-straight reads as a
                                        // single line.
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

                    // Two columns, axes and groups, rather than one tall
                    // stack of both: they answer different questions (what
                    // each physical axis is doing vs. where each group's
                    // TCP is), and stacked they made the row taller than
                    // the plots beside them for no reason.
                    ui.vertical(|ui| {
                        ui.heading("axes");
                        for axis in 0..self.num_axes {
                            let last = history.axis(axis).back();
                            ui.group(|ui| {
                                ui.set_min_width(160.0);
                                ui.strong(format!("axis{axis}"));
                                // Each axis labels itself in its own units —
                                // a rotary joint must not print "mm".
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
                                        // Raw DS402 detail alongside the coarser
                                        // status line above — mainly useful
                                        // while watching an enable/disable
                                        // sequence step through its 3 cycles.
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

                    // One box per hard-coded axis group, built purely from
                    // the same per-axis History feedback the boxes beside
                    // it use — viz only taps the AxisGroup::exchange() seam,
                    // it has no visibility into app's higher-level Profile::
                    // Group/command bookkeeping, so this can't show whether
                    // a synchronized group move is *currently* active the
                    // way the terminal `status` command can; it's a
                    // per-member position/velocity rollup instead.
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
                                // ...and the same members as one TCP point,
                                // which is the only line here that means
                                // the same thing across every kind of group.
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
                            // Newest-first, decimated: what every trace in
                            // this column is built from.
                            let series = |pick: fn(&crate::recording::Sample) -> f64| {
                                history
                                    .axis(axis)
                                    .iter()
                                    .rev()
                                    .step_by(stride)
                                    .map(move |s| [s.t, pick(s)])
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

                            // Both traces come straight off the seam, same
                            // as position and velocity — the target one is
                            // the planner's exact closed-form acceleration,
                            // not a difference of plotted samples. This is
                            // the plot that makes jerk limiting visible at
                            // all: unfiltered, the target steps square
                            // between 0 and +/-a_max; filtered, it ramps.
                            //
                            // The *actual* trace is a plant estimate and
                            // will look coarser — see `AxisFeedback`.
                            ui.label(format!("acceleration ({units}/s²)"));
                            time_series_plot(
                                ui,
                                format!("acc_plot_{axis}"),
                                col_width,
                                PLOT_HEIGHT,
                                in_plot_order(series(|s| s.setpoint.acceleration)),
                                in_plot_order(series(|s| s.feedback.acceleration)),
                            );
                        });
                        ui.separator();
                    }
                });
            });
        });

        // The control loop writes new samples continuously (250 Hz); repaint
        // on a timer rather than only on input so the plots keep animating.
        ctx.request_repaint_after(VIZ_REFRESH_PERIOD);
    }
}
