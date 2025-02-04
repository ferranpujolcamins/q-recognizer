#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release
#![allow(rustdoc::missing_crate_level_docs)] // it's an example

use eframe::egui;
use egui::{emath, Color32, Frame, Pos2, Rect, Sense, Stroke, Ui};
use q_recognizer::{gesture::Gesture, point::Point, q_point_cloud_recognizer::{self, QParameters}};

fn main() -> eframe::Result {    
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([500.0, 240.0]),
        ..Default::default()
    };
    eframe::run_native(
        "My egui App",
        options,
        Box::new(|cc| {
            // This gives us image support:
            egui_extras::install_image_loaders(&cc.egui_ctx);

            Ok(Box::<DemoApp>::default())
        }),
    )
}

struct DemoApp {
    /// in 0-1 normalized coordinates
    lines: Vec<Vec<Pos2>>,
    stroke: Stroke,
    gestures: Vec<Gesture>,
    recognized_gesture: String,
}

impl Default for DemoApp {
    fn default() -> Self {
        Self {
            lines: Default::default(),
            stroke: Stroke::new(1.0, Color32::from_rgb(25, 200, 100)),
            gestures: Default::default(),
            recognized_gesture: Default::default(),
        }
    }
}

fn lines_to_points(lines: &[Vec<Pos2>]) -> Vec<Point> {
    lines.iter().enumerate().flat_map(|(i, line)| {
        line.iter().map(move |p| {
            Point::new(p.x, p.y, i as i32)
        })
    }).collect()
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
        let result = q_point_cloud_recognizer::classify(&gesture, &self.gestures, &QParameters::default());
        if result.distance > 10. {
            format!("No match (distance: {:.3})", result.distance)
        } else {
            result.class
        }
    }

    pub fn ui_control(&mut self, ui: &mut egui::Ui) -> egui::Response {
        ui.horizontal(|ui| {
            if ui.button("Clear Drawing").clicked() {
                self.lines.clear();
                self.recognized_gesture.clear();
            }
            ui.separator();
            if ui.button("Save Gesture").clicked() {
                self.save_gesture()
            }
            ui.separator();
            if ui.button("Recognize Gesture").clicked() {
                self.recognized_gesture = self.recognize_gesture();
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
    egui::CentralPanel::default().show(ctx, |ui| {
            self.ui_control(ui);
            ui.label("Paint with your mouse/touch!");
            Frame::canvas(ui.style()).show(ui, |ui| {
                self.ui_content(ui);
            });
        });
    }
}