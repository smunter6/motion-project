//! The viz window: a passive, read-only view over the shared [`History`]
//! buffer the control loop writes to via [`RecordingAxisGroup`]. Input stays
//! terminal-driven — this window has no controls, it only plots.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use axis_backend::{AxisState, MotionFlags};
use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};

use crate::recording::History;

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
                            Plot::new(format!("xy_plot_{}", group.name))
                                .view_aspect(1.5)
                                .height(220.0)
                                .show(ui, |plot_ui| {
                                    let target: PlotPoints = history
                                        .axis(x_axis)
                                        .iter()
                                        .zip(history.axis(y_axis).iter())
                                        .filter_map(|(a, b)| {
                                            tcp_xy(
                                                group,
                                                [a.setpoint.position, b.setpoint.position],
                                            )
                                        })
                                        .collect();
                                    let actual: PlotPoints = history
                                        .axis(x_axis)
                                        .iter()
                                        .zip(history.axis(y_axis).iter())
                                        .filter_map(|(a, b)| {
                                            tcp_xy(
                                                group,
                                                [a.feedback.position, b.feedback.position],
                                            )
                                        })
                                        .collect();
                                    plot_ui.line(Line::new("target", target));
                                    plot_ui.line(Line::new("actual", actual));
                                });
                        });
                    }

                    ui.vertical(|ui| {
                        ui.heading("status");
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
                        // One box per hard-coded axis group, built purely from
                        // the same per-axis History feedback the boxes above
                        // use — viz only taps the AxisGroup::exchange() seam, it
                        // has no visibility into app's higher-level Profile::
                        // Group/command bookkeeping, so this can't show whether
                        // a synchronized group move is *currently* active the
                        // way the terminal `status` command can; it's a
                        // per-member position/velocity rollup instead.
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

                            ui.label(format!("position ({units})"));
                            Plot::new(format!("pos_plot_{axis}"))
                                .width(col_width)
                                .height(160.0)
                                .show(ui, |plot_ui| {
                                    let target: PlotPoints = history
                                        .axis(axis)
                                        .iter()
                                        .map(|s| [s.t, s.setpoint.position])
                                        .collect();
                                    let actual: PlotPoints = history
                                        .axis(axis)
                                        .iter()
                                        .map(|s| [s.t, s.feedback.position])
                                        .collect();
                                    plot_ui.line(Line::new("target", target));
                                    plot_ui.line(Line::new("actual", actual));
                                });

                            ui.label(format!("velocity ({units}/s)"));
                            Plot::new(format!("vel_plot_{axis}"))
                                .width(col_width)
                                .height(160.0)
                                .show(ui, |plot_ui| {
                                    let target: PlotPoints = history
                                        .axis(axis)
                                        .iter()
                                        .map(|s| [s.t, s.setpoint.velocity])
                                        .collect();
                                    let actual: PlotPoints = history
                                        .axis(axis)
                                        .iter()
                                        .map(|s| [s.t, s.feedback.velocity])
                                        .collect();
                                    plot_ui.line(Line::new("target", target));
                                    plot_ui.line(Line::new("actual", actual));
                                });
                        });
                        ui.separator();
                    }
                });
            });
        });

        // The control loop writes new samples continuously (250 Hz); repaint
        // on a timer rather than only on input so the plots keep animating.
        ctx.request_repaint_after(std::time::Duration::from_millis(33));
    }
}
