//! The viz window: a passive, read-only view over the shared [`History`]
//! buffer the control loop writes to via [`RecordingAxisGroup`]. Input stays
//! terminal-driven — this window has no controls, it only plots.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};

use crate::recording::History;

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

            if self.num_axes >= 2 {
                ui.heading("tool position (axis0 = X, axis1 = Y)");
                Plot::new("xy_plot")
                    .view_aspect(1.5)
                    .height(220.0)
                    .show(ui, |plot_ui| {
                        let target: PlotPoints = history
                            .axis(0)
                            .iter()
                            .zip(history.axis(1).iter())
                            .map(|(a, b)| [a.target_position, b.target_position])
                            .collect();
                        let actual: PlotPoints = history
                            .axis(0)
                            .iter()
                            .zip(history.axis(1).iter())
                            .map(|(a, b)| [a.actual_position, b.actual_position])
                            .collect();
                        plot_ui.line(Line::new("target", target));
                        plot_ui.line(Line::new("actual", actual));
                    });
                ui.separator();
            }

            for axis in 0..self.num_axes {
                ui.heading(format!("axis{axis} position (mm)"));
                Plot::new(format!("pos_plot_{axis}"))
                    .height(160.0)
                    .show(ui, |plot_ui| {
                        let target: PlotPoints = history
                            .axis(axis)
                            .iter()
                            .map(|s| [s.t, s.target_position])
                            .collect();
                        let actual: PlotPoints = history
                            .axis(axis)
                            .iter()
                            .map(|s| [s.t, s.actual_position])
                            .collect();
                        plot_ui.line(Line::new("target", target));
                        plot_ui.line(Line::new("actual", actual));
                    });

                ui.heading(format!("axis{axis} velocity (mm/s)"));
                Plot::new(format!("vel_plot_{axis}"))
                    .height(160.0)
                    .show(ui, |plot_ui| {
                        let target: PlotPoints = history
                            .axis(axis)
                            .iter()
                            .map(|s| [s.t, s.target_velocity])
                            .collect();
                        let actual: PlotPoints = history
                            .axis(axis)
                            .iter()
                            .map(|s| [s.t, s.actual_velocity])
                            .collect();
                        plot_ui.line(Line::new("target", target));
                        plot_ui.line(Line::new("actual", actual));
                    });
                ui.separator();
            }
        });

        // The control loop writes new samples continuously (250 Hz); repaint
        // on a timer rather than only on input so the plots keep animating.
        ctx.request_repaint_after(std::time::Duration::from_millis(33));
    }
}
