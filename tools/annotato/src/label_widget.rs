use std::{
    fs::{self, File},
    io::Write,
};

use crate::{
    annotation::{Annotation, AnnotationFormat},
    classes::Class,
    paths::Paths,
    utils,
    widgets::{
        bounding_box_annotator::{BoundingBoxAnnotator, CanvasState, CreationShape},
        class_selector::ClassSelector,
    },
};
use color_eyre::eyre::{ContextCompat, Result};
use eframe::{
    egui::{Color32, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Vec2},
    epaint::TextureHandle,
};

pub struct LabelWidget {
    current_paths: Option<Paths>,
    texture_handle: Option<TextureHandle>,
    image_size: Option<[f32; 2]>,
    selected_class: Class,
    goalpost_creation_shape: CreationShape,
    annotations: Vec<Annotation>,
    canvas_state: CanvasState,
    unresolved_annotations: Vec<AnnotationFormat>,
}

impl Default for LabelWidget {
    fn default() -> Self {
        Self {
            current_paths: None,
            texture_handle: None,
            image_size: None,
            selected_class: Class::Robot,
            goalpost_creation_shape: CreationShape::Box,
            annotations: Vec::new(),
            canvas_state: CanvasState::default(),
            unresolved_annotations: Vec::new(),
        }
    }
}

impl LabelWidget {
    pub fn has_paths(&self, paths: &Paths) -> bool {
        self.current_paths
            .as_ref()
            .map(|current_paths| paths.image_path == current_paths.image_path)
            .unwrap_or(false)
    }

    pub fn ui(&mut self, ui: &mut eframe::egui::Ui) {
        self.ensure_texture_loaded(ui);

        ui.vertical(|ui| {
            self.toolbar_ui(ui);
            ui.add_space(8.0);

            if self.migration_ui(ui) {
                return;
            }

            if let (Some(texture_handle), Some(image_size)) =
                (self.texture_handle.clone(), self.image_size)
            {
                let creation_shape = self.creation_shape();
                ui.add(BoundingBoxAnnotator::new(
                    texture_handle,
                    image_size,
                    &mut self.annotations,
                    &mut self.canvas_state,
                    &mut self.selected_class,
                    creation_shape,
                ));
            }
            ui.add_space(6.0);
            self.help_ui(ui);
        });
    }

    fn ensure_texture_loaded(&mut self, ui: &eframe::egui::Ui) {
        if self.texture_handle.is_some() {
            return;
        }

        let paths = self.current_paths.as_ref().expect("No image loaded");
        let handle = utils::load_image(ui, &paths.image_path).expect("failed to load image");
        let size = [handle.size()[0] as f32, handle.size()[1] as f32];
        self.image_size = Some(size);
        self.texture_handle = Some(handle);

        self.annotations = self
            .unresolved_annotations
            .drain(..)
            .map(|annotation| Annotation::from_format(annotation, size))
            .collect();
    }

    fn toolbar_ui(&mut self, ui: &mut eframe::egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            if let Some(paths) = &self.current_paths {
                let filename = paths
                    .image_path
                    .file_name()
                    .and_then(|file_name| file_name.to_str())
                    .unwrap_or("<invalid file name>");
                ui.label(RichText::new(filename).strong());
                if paths.label_present {
                    ui.colored_label(Color32::from_rgb(166, 227, 161), "saved");
                } else {
                    ui.colored_label(Color32::from_rgb(250, 179, 135), "unsaved");
                }
            }

            ui.separator();
            ui.label("Class");
            let previous_class = self.selected_class;
            ui.add(ClassSelector::new(
                "class-selector",
                &mut self.selected_class,
            ));
            if self.selected_class != previous_class {
                self.canvas_state
                    .apply_class(&mut self.annotations, self.selected_class);
            }

            if self.selected_class == Class::GoalPost {
                ui.separator();
                ui.label("Goal post");
                ui.selectable_value(&mut self.goalpost_creation_shape, CreationShape::Box, "box");
                ui.selectable_value(
                    &mut self.goalpost_creation_shape,
                    CreationShape::Point,
                    "point",
                );
            }

            ui.separator();
            ui.label(format!("{} labels", self.annotations.len()));
            if let Some(selection) = self.canvas_state.selected() {
                ui.label(format!(
                    "selected #{} {:?}",
                    selection.index() + 1,
                    selection.shape()
                ));
            }
        });
    }

    fn creation_shape(&self) -> CreationShape {
        if self.selected_class.requires_point() {
            CreationShape::Point
        } else if self.selected_class == Class::GoalPost {
            self.goalpost_creation_shape
        } else {
            CreationShape::Box
        }
    }

    fn help_ui(&self, ui: &mut eframe::egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Mouse:").strong());
            ui.label("click point");
            ui.label("drag create box");
            ui.label("drag label move");
            ui.label("drag handle resize");
            ui.label("right click delete");
            ui.separator();
            ui.label(RichText::new("Keyboard:").strong());
            ui.label("B new/commit");
            ui.label("arrows move/resize");
            ui.label("Shift faster");
            ui.label("Ctrl fine");
            ui.label("E resize box");
            ui.label("M move");
            ui.label("C corner");
            ui.label("Tab label");
            ui.label("1-8 or [] class");
            ui.label("Enter commit");
            ui.label("Esc cancel");
        });
    }

    fn migration_ui(&mut self, ui: &mut eframe::egui::Ui) -> bool {
        let Some(texture_handle) = self.texture_handle.as_ref() else {
            return false;
        };
        let Some(image_size) = self.image_size else {
            return false;
        };
        let Some((annotation_index, pending_count)) = self.pending_migration() else {
            return false;
        };
        let annotation = &self.annotations[annotation_index];
        let bounding_box = annotation
            .bounding_box
            .as_ref()
            .expect("migration annotation has bbox");
        let crop_rect = migration_crop_rect(bounding_box.rect, image_size);

        ui.vertical_centered(|ui| {
            ui.heading("Migrate point feature");
            ui.label(format!(
                "Click the exact {} location inside this legacy crop. The old box is preserved.",
                annotation.class.as_str()
            ));
            ui.label(format!(
                "{pending_count} point migrations remaining on this image"
            ));
        });
        ui.add_space(8.0);

        let available = ui.available_size_before_wrap();
        let crop_size = crop_rect.size();
        let scale = (available.x / crop_size.x.max(1.0))
            .min((available.y - 24.0).max(240.0) / crop_size.y.max(1.0))
            .max(0.1);
        let display_size = crop_size * scale;
        let (display_rect, response) = ui.allocate_exact_size(display_size, Sense::click());
        let uv = Rect::from_min_max(
            Pos2::new(
                crop_rect.left() / image_size[0],
                crop_rect.top() / image_size[1],
            ),
            Pos2::new(
                crop_rect.right() / image_size[0],
                crop_rect.bottom() / image_size[1],
            ),
        );
        let painter = ui.painter_at(display_rect);
        painter.rect_filled(display_rect, 4.0, Color32::from_rgb(17, 17, 27));
        painter.image(texture_handle.id(), display_rect, uv, Color32::WHITE);
        painter.rect_stroke(
            display_rect,
            4.0,
            Stroke::new(1.0, annotation.class.color()),
            StrokeKind::Inside,
        );

        if response.clicked()
            && let Some(pointer) = response.interact_pointer_pos()
            && let Some(point) =
                crop_click_to_image_point(pointer, display_rect, crop_rect, image_size)
        {
            self.annotations[annotation_index].point = Some(point);
        }

        true
    }

    fn pending_migration(&self) -> Option<(usize, usize)> {
        let mut first_index = None;
        let mut count = 0;

        for (index, annotation) in self.annotations.iter().enumerate() {
            if annotation.needs_point_migration() {
                first_index.get_or_insert(index);
                count += 1;
            }
        }

        first_index.map(|index| (index, count))
    }

    pub fn load_new_image_with_labels(
        &mut self,
        paths: Paths,
        model_annotations: Vec<AnnotationFormat>,
    ) -> Result<()> {
        self.annotations.clear();
        self.unresolved_annotations.clear();
        self.image_size = None;
        self.texture_handle = None;
        self.canvas_state = CanvasState::default();

        if paths.label_path.exists() {
            let existing_annotations = fs::read_to_string(&paths.label_path)?;
            self.unresolved_annotations = serde_json::from_str(&existing_annotations)?;
        } else {
            self.unresolved_annotations = model_annotations;
        }

        self.current_paths = Some(paths);

        Ok(())
    }

    pub fn save_annotation(&mut self) -> Result<()> {
        let paths = self
            .current_paths
            .as_ref()
            .wrap_err("no image loaded currently")?;
        let Some(image_size) = self.image_size else {
            return Ok(());
        };
        let annotations: Vec<AnnotationFormat> = self
            .annotations
            .iter()
            .map(|annotation| annotation.to_format(image_size))
            .collect();

        let annotations = serde_json::to_string_pretty(&annotations)?;

        let mut file = File::create(&paths.label_path)?;
        file.write_all(annotations.as_bytes())?;

        if let Some(paths) = &mut self.current_paths {
            paths.check_existence();
        }

        Ok(())
    }
}

pub fn migration_crop_rect(bounding_box: Rect, image_size: [f32; 2]) -> Rect {
    let padding = (bounding_box.width().max(bounding_box.height()) * 0.5).max(16.0);
    let image_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(image_size[0], image_size[1]));
    bounding_box.expand(padding).intersect(image_rect)
}

pub fn crop_click_to_image_point(
    pointer: Pos2,
    display_rect: Rect,
    crop_rect: Rect,
    image_size: [f32; 2],
) -> Option<Pos2> {
    if !display_rect.contains(pointer) {
        return None;
    }

    let relative = (pointer - display_rect.min) / display_rect.size();
    Some(Pos2::new(
        (crop_rect.left() + relative.x * crop_rect.width()).clamp(0.0, image_size[0]),
        (crop_rect.top() + relative.y * crop_rect.height()).clamp(0.0, image_size[1]),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn migration_crop_click_maps_to_full_image_coordinates() {
        let display = Rect::from_min_size(Pos2::new(10.0, 20.0), Vec2::new(200.0, 100.0));
        let crop = Rect::from_min_max(Pos2::new(100.0, 50.0), Pos2::new(300.0, 150.0));

        let point =
            crop_click_to_image_point(Pos2::new(110.0, 70.0), display, crop, [640.0, 480.0])
                .unwrap();

        assert_eq!(point, Pos2::new(200.0, 100.0));
    }

    #[test]
    fn migration_crop_is_clamped_to_image() {
        let crop = migration_crop_rect(
            Rect::from_min_max(Pos2::new(0.0, 0.0), Pos2::new(20.0, 20.0)),
            [100.0, 100.0],
        );

        assert_eq!(crop.min, Pos2::ZERO);
        assert!(crop.max.x <= 100.0);
        assert!(crop.max.y <= 100.0);
    }
}
