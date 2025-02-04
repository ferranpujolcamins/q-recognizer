#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release
#![allow(rustdoc::missing_crate_level_docs)] // it's an example

use std::fs;

use eframe::egui;
use egui::{emath, Color32, Frame, Pos2, Rect, Sense, Stroke, Ui};
use egui_file_dialog::FileDialog;
use q_recognizer::{
    gesture::Gesture,
    point::Point,
    q_point_cloud_recognizer::{self, QParameters},
};
use ron::ser::{to_string_pretty, PrettyConfig};

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([600.0, 350.0]),
        ..Default::default()
    };
    eframe::run_native(
        "My egui App",
        options,
        Box::new(|_| Ok(Box::<DemoApp>::default())),
    )
}

struct DemoApp {
    /// in 0-1 normalized coordinates
    lines: Vec<Vec<Pos2>>,
    stroke: Stroke,
    gestures: Vec<Gesture>,
    recognized_gesture: String,
    file_dialog: FileDialog,
}

impl Default for DemoApp {
    fn default() -> Self {
        Self {
            lines: Default::default(),
            stroke: Stroke::new(1.0, Color32::from_rgb(25, 200, 100)),
            gestures: Default::default(),
            recognized_gesture: Default::default(),
            file_dialog: FileDialog::new()
                .default_file_name("gestures.json")
                .resizable(false)
        }
    }
}

fn lines_to_points(lines: &[Vec<Pos2>]) -> Vec<Point> {
    lines
        .iter()
        .enumerate()
        .flat_map(|(i, line)| line.iter().map(move |p| Point::new(p.x, p.y, i as i32)))
        .collect()
}

fn points_to_lines(points: &Vec<Point>) -> Vec<Vec<Pos2>> {
    let mut lines = vec![];
    for point in points {
        while lines.len() <= point.stroke_id as usize {
            lines.push(vec![]);
        }
        lines[point.stroke_id as usize].push(Pos2::new(point.x, point.y));
    }
    lines
}

impl DemoApp {
    fn save_gesture(&mut self) {
        let name = format!("Gesture {}", self.gestures.len() + 1);
        let gesture = Gesture::new(lines_to_points(&self.lines), &name);
        self.gestures.push(gesture);
        self.lines.clear();
    }

    fn recognize_gesture(&self) -> String {
        let gesture = Gesture::new(lines_to_points(&self.lines), "Unknown");
        let result =
            q_point_cloud_recognizer::classify(&gesture, &self.gestures, &QParameters::default());
        if result.distance > 10. {
            format!("No match (distance: {:.3})", result.distance)
        } else {
            result.class
        }
    }

    pub fn ui_control(&mut self, ui: &mut egui::Ui) -> egui::Response {
        ui.horizontal(|ui| {
            if ui.button("Export Gestures").clicked() {
                self.file_dialog.save_file();
            }
            self.file_dialog.update(ui.ctx());
            if let Some(path) = self.file_dialog.picked() {
                let data = to_string_pretty(&self.gestures, PrettyConfig::default()).unwrap();
                fs::write(path, data).unwrap();
            }

            if ui.button("Save Gesture").clicked() {
                self.save_gesture()
            }
            if ui.button("Recognize Gesture").clicked() {
                self.recognized_gesture = self.recognize_gesture();
            }
            ui.separator();
            if ui.button("Clear Drawing").clicked() {
                self.lines.clear();
                self.recognized_gesture.clear();
            }
            ui.label(self.recognized_gesture.clone());
        })
        .response
    }

    pub fn ui_content(&mut self, ui: &mut Ui) -> egui::Response {
        let (mut response, painter) =
            ui.allocate_painter(ui.available_size_before_wrap(), Sense::drag());

        let to_screen = emath::RectTransform::from_to(
            Rect::from_min_size(Pos2::ZERO, response.rect.square_proportions()),
            response.rect,
        );
        let from_screen = to_screen.inverse();

        if self.lines.is_empty() {
            self.lines.push(vec![]);
        }

        let current_line = self.lines.last_mut().unwrap();

        if let Some(pointer_pos) = response.interact_pointer_pos() {
            let canvas_pos = from_screen * pointer_pos;
            if current_line.last() != Some(&canvas_pos) {
                current_line.push(canvas_pos);
                response.mark_changed();
            }
        } else if !current_line.is_empty() {
            self.lines.push(vec![]);
            response.mark_changed();
        }

        let shapes = self
            .lines
            .iter()
            .filter(|line| line.len() >= 2)
            .map(|line| {
                let points: Vec<Pos2> = line.iter().map(|p| to_screen * *p).collect();
                egui::Shape::line(points, self.stroke)
            });

        painter.extend(shapes);

        response
    }
}

impl eframe::App for DemoApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("gestures").show(ctx, |ui| {
            ui.label("Gestures:");
            for (i, gesture) in self.gestures.iter().enumerate() {
                ui.group(|ui| {
                    ui.label(format!("Gesture {}", i + 1));
                    let lines = points_to_lines(&gesture.points);
                    let desired_size = egui::vec2(80.0, 60.0);
                    let (response, painter) = ui.allocate_painter(desired_size, Sense::hover());

                    // Compute a bounding box for all points
                    let mut min = Pos2::new(f32::MAX, f32::MAX);
                    let mut max = Pos2::new(f32::MIN, f32::MIN);
                    for line in &lines {
                        for &p in line {
                            min.x = min.x.min(p.x);
                            min.y = min.y.min(p.y);
                            max.x = max.x.max(p.x);
                            max.y = max.y.max(p.y);
                        }
                    }
                    let bounds = Rect::from_min_max(min, max);
                    let transform = egui::emath::RectTransform::from_to(bounds, response.rect);

                    // Draw each line
                    for line in lines {
                        if line.len() < 2 {
                            continue;
                        }
                        let points: Vec<Pos2> = line.iter().map(|&p| transform * p).collect();
                        painter.line_segment(
                            points.windows(2).flatten().cloned().collect::<Vec<_>>()[..2].try_into().unwrap(),
                            self.stroke,
                        );
                        for w in points.windows(2).skip(1) {
                            painter.line_segment(w.try_into().unwrap(), self.stroke);
                        }
                    }
                });
            }
        });
        egui::CentralPanel::default().show(ctx, |ui| {
            self.ui_control(ui);
            ui.label("Paint with your mouse/touch!");
            Frame::canvas(ui.style()).show(ui, |ui| {
                self.ui_content(ui);
            });
        });
    }
}
