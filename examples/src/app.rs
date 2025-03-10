#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release
#![allow(rustdoc::missing_crate_level_docs)] // it's an example

use eframe::egui;
use egui::{emath, Color32, Frame, Pos2, Rect, Sense, Stroke, Ui};
#[cfg(target_arch = "wasm32")]
use rfd::AsyncFileDialog;
#[cfg(not(target_arch = "wasm32"))]
use rfd::FileDialog;
#[cfg(not(target_arch = "wasm32"))]
use std::fs;
use q_recognizer::{
    gesture::Gesture,
    point::Point,
    q_point_cloud_recognizer::{self, QParameters},
};
use ron::ser::{to_string_pretty, PrettyConfig};
use ron::de::from_str;
#[cfg(target_arch = "wasm32")]
use wasm_bindgen_futures::spawn_local;
#[cfg(target_arch = "wasm32")]
use std::sync::mpsc::{channel, Receiver, Sender};

pub struct DemoApp {
    /// in 0-1 normalized coordinates
    lines: Vec<Vec<Pos2>>,
    stroke: Stroke,
    gestures: Vec<Gesture>,
    recognized_gesture: String,
    #[cfg(target_arch = "wasm32")]
    gesture_receiver: Receiver<Vec<Gesture>>,
    #[cfg(target_arch = "wasm32")]
    gesture_sender: Sender<Vec<Gesture>>,
}

impl Default for DemoApp {
    fn default() -> Self {
        #[cfg(target_arch = "wasm32")]
        let (gesture_sender, gesture_receiver) = channel();
        
        Self {
            lines: Default::default(),
            stroke: Stroke::new(1.0, Color32::from_rgb(25, 200, 100)),
            gestures: Default::default(),
            recognized_gesture: Default::default(),
            #[cfg(target_arch = "wasm32")]
            gesture_receiver,
            #[cfg(target_arch = "wasm32")]
            gesture_sender,
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
    #[cfg(target_arch = "wasm32")]
    fn export_gestures(&self) {
        let content = to_string_pretty(&self.gestures, PrettyConfig::default()).unwrap();
        spawn_local(async move {
            if let Some(file_handle) = AsyncFileDialog::new()
                .set_file_name("gestures.ron")
                .save_file()
                .await
            {
                file_handle.write(content.as_bytes()).await.unwrap();
            }
        });
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn export_gestures(&self) {
        let content = to_string_pretty(&self.gestures, PrettyConfig::default()).unwrap();
        if let Some(path) = FileDialog::new()
            .set_file_name("gestures.ron")
            .save_file()
        {
            fs::write(path, content).unwrap();
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn load_gestures(&mut self) {
        let sender = self.gesture_sender.clone();
        spawn_local(async move {
            if let Some(file_handle) = AsyncFileDialog::new().pick_file().await {
                let content = file_handle.read().await;
                if let Ok(gestures) = from_str::<Vec<Gesture>>(std::str::from_utf8(&content).unwrap()) {
                    let _ = sender.send(gestures);
                }
            }
        });
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn load_gestures(&mut self) {
        if let Some(path) = FileDialog::new().pick_file() {
            let content = fs::read_to_string(path).unwrap();
            if let Ok(gestures) = from_str::<Vec<Gesture>>(&content) {
                self.gestures = gestures;
            }
        }
    }

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
            if ui.button("Load Gestures").clicked() {
                self.load_gestures();
            }
            if ui.button("Export Gestures").clicked() {
                self.export_gestures();
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
        #[cfg(target_arch = "wasm32")]
        {
            // Check for any gestures received from async operations
            if let Ok(gestures) = self.gesture_receiver.try_recv() {
                self.gestures = gestures;
            }
        }
        
        egui::SidePanel::left("gestures").show(ctx, |ui| {
            ui.label("Gestures:");
            egui::ScrollArea::vertical().show(ui, |ui| {
                for gesture in self.gestures.iter_mut() {
                    ui.group(|ui| {
                        ui.horizontal(|ui| {
                            let mut new_name = gesture.name.clone();
                            if ui.text_edit_singleline(&mut new_name).changed() {
                                gesture.name = new_name;
                            }
                        });
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
