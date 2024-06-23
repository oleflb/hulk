use std::path::PathBuf;

use eframe::egui::{
    emath::RectTransform, Color32, Image, PointerButton, Pos2, Rect, Sense, Widget,
};

use crate::polygon::paint_polygon;

pub enum Class {
    Field,
    Line,
}

pub struct Segmentator<'ui> {
    image_path: &'ui PathBuf,
    // polygons: BTreeMap<Class, Vec<Polygon>>,
    vertices: &'ui mut Vec<Pos2>,
}

impl<'ui> Segmentator<'ui> {
    pub fn new(image_path: &'ui PathBuf, vertices: &'ui mut Vec<Pos2>) -> Self {
        Segmentator {
            image_path,
            // polygons: BTreeMap::new(),
            vertices,
        }
    }
}

impl<'ui> Widget for Segmentator<'ui> {
    fn ui(self, ui: &mut eframe::egui::Ui) -> eframe::egui::Response {
        let uri = format!("file://{}", self.image_path.display());
        let image = Image::new(uri)
            .show_loading_spinner(true)
            .maintain_aspect_ratio(true)
            .shrink_to_fit()
            .sense(Sense::click());
        let response = ui.add(image);

        let transform = RectTransform::from_to(
            response.rect,
            Rect::from_min_max(Pos2::new(0.0, 0.0), Pos2::new(1.0, 1.0)),
        );

        let new_point = response
            .hover_pos()
            .map(|position| transform.transform_pos(position));

        if response.clicked_by(PointerButton::Primary) {
            self.vertices.extend(new_point);
        }

        dbg!(&new_point);

        paint_polygon(
            ui.painter(),
            self.vertices.iter().copied().chain(new_point),
            transform.inverse(),
            Color32::RED,
        );

        response
    }
}
