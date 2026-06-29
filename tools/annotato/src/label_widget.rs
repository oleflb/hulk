mod class_popup;

use crate::{
    annotation::AnnotationFormat,
    classes::Class,
    label_document::LabelDocument,
    paths::Paths,
    user_toml::CONFIG,
    utils,
    widgets::bounding_box_annotator::{BoundingBoxAnnotator, CanvasState, CreationShape},
};
use color_eyre::eyre::Result;
use eframe::{
    egui::{Button, Color32, Pos2, Rect, RichText, Sense, Stroke, StrokeKind, Vec2},
    epaint::TextureHandle,
};

pub struct LabelWidget {
    document: LabelDocument,
    texture_handle: Option<TextureHandle>,
    image_size: Option<[f32; 2]>,
    selected_class: Class,
    class_popup_open: bool,
    class_popup_index: usize,
    canvas_state: CanvasState,
    texture_load_error: Option<String>,
}

impl Default for LabelWidget {
    fn default() -> Self {
        Self {
            document: LabelDocument::default(),
            texture_handle: None,
            image_size: None,
            selected_class: Class::Robot,
            class_popup_open: false,
            class_popup_index: Class::ALL
                .iter()
                .position(|class| *class == Class::Robot)
                .unwrap_or(0),
            canvas_state: CanvasState::default(),
            texture_load_error: None,
        }
    }
}

impl LabelWidget {
    pub fn has_paths(&self, paths: &Paths) -> bool {
        self.document.has_paths(paths)
    }

    pub fn captures_keyboard(&self) -> bool {
        self.class_popup_open
    }

    pub fn is_dirty(&self) -> bool {
        self.document.is_dirty()
    }

    pub fn selected_class(&self) -> Class {
        self.selected_class
    }

    pub fn set_selected_class(&mut self, class: Class) {
        if self.selected_class != class {
            self.selected_class = class;
            self.canvas_state = CanvasState::default();
        }
        self.class_popup_open = false;
        self.class_popup_index = Class::ALL
            .iter()
            .position(|candidate| *candidate == class)
            .unwrap_or(0);
    }

    pub fn mark_labeled_class(&mut self, class: Class) {
        self.document.mark_labeled_class(class);
    }

    pub fn class_is_labeled(&self, class: Class) -> bool {
        self.document.class_is_labeled(class)
    }

    pub fn has_pending_migration_for_class(&self, class: Class) -> bool {
        self.document.has_pending_migration_for_class(class)
    }

    pub fn pointer_interaction_active(&self) -> bool {
        self.canvas_state.pointer_interaction_active()
    }

    pub fn ui(&mut self, ui: &mut eframe::egui::Ui, class_locked: bool) -> Result<bool> {
        self.ensure_texture_loaded(ui)?;
        self.handle_class_popup_shortcut(ui, class_locked);

        let mut annotations_changed = false;
        ui.vertical(|ui| {
            self.toolbar_ui(ui, class_locked);
            ui.add_space(8.0);

            if let Some(migration_changed) = self.migration_ui(ui) {
                annotations_changed |= migration_changed;
                return;
            }

            if let Some(error) = &self.texture_load_error {
                ui.colored_label(Color32::from_rgb(243, 139, 168), error);
            } else if let (Some(texture_handle), Some(image_size)) =
                (self.texture_handle.as_ref(), self.image_size)
            {
                let creation_shape = self.creation_shape();
                ui.add(BoundingBoxAnnotator::new(
                    texture_handle,
                    image_size,
                    self.document.annotations_mut(),
                    &mut self.canvas_state,
                    &mut self.selected_class,
                    creation_shape,
                    !self.class_popup_open,
                ));
                if self.canvas_state.take_annotations_changed() {
                    self.document.mark_dirty();
                    annotations_changed = true;
                }
            }
            ui.add_space(6.0);
            self.help_ui(ui);
        });
        if !class_locked {
            self.class_popup_ui(ui);
        }

        Ok(annotations_changed)
    }

    fn ensure_texture_loaded(&mut self, ui: &eframe::egui::Ui) -> Result<()> {
        if self.texture_handle.is_some() || self.texture_load_error.is_some() {
            return Ok(());
        }

        let paths = self.document.paths().expect("No image loaded");
        let handle = match utils::load_image(ui, &paths.image_path) {
            Ok(handle) => handle,
            Err(error) => {
                self.texture_load_error = Some(format!(
                    "failed to load image {}: {error:#}",
                    paths.image_path.display()
                ));
                return Ok(());
            }
        };
        let size = [handle.size()[0] as f32, handle.size()[1] as f32];
        self.image_size = Some(size);
        self.texture_handle = Some(handle);

        self.document.resolve_annotations(size);

        Ok(())
    }

    fn toolbar_ui(&mut self, ui: &mut eframe::egui::Ui, class_locked: bool) {
        ui.horizontal_wrapped(|ui| {
            if let Some(paths) = self.document.paths() {
                let filename = paths
                    .image_path
                    .file_name()
                    .and_then(|file_name| file_name.to_str())
                    .unwrap_or("<invalid file name>");
                ui.label(RichText::new(filename).strong());
                if self.document.is_dirty() {
                    ui.colored_label(Color32::from_rgb(249, 226, 175), "modified");
                } else if paths.label_present {
                    ui.colored_label(Color32::from_rgb(166, 227, 161), "saved");
                } else {
                    ui.colored_label(Color32::from_rgb(250, 179, 135), "unsaved");
                }
            }

            ui.separator();
            ui.label("Class");
            let config = &CONFIG.get().unwrap().keybindings;
            let class_label = if class_locked {
                format!("{}  (chunk)", self.selected_class.as_str())
            } else {
                format!(
                    "{}  ({})",
                    self.selected_class.as_str(),
                    config.class_popup.label()
                )
            };
            let class_button = Button::new(
                RichText::new(class_label)
                    .strong()
                    .size(16.0)
                    .color(Color32::from_rgb(245, 245, 245)),
            )
            .fill(self.selected_class.color().gamma_multiply(0.35))
            .stroke(Stroke::new(1.0, self.selected_class.color()));
            if ui.add_enabled(!class_locked, class_button).clicked() {
                self.open_class_popup();
            }

            ui.separator();
            ui.label(format!("{} labels", self.document.annotations().len()));
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
        } else {
            CreationShape::Box
        }
    }

    fn help_ui(&self, ui: &mut eframe::egui::Ui) {
        let config = &CONFIG.get().unwrap().keybindings;
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Mouse:").strong());
            ui.label("click point");
            ui.label("drag create box");
            ui.label("drag label move");
            ui.label("drag handle resize");
            ui.label("right click delete");
            ui.separator();
            ui.label(RichText::new("Keyboard:").strong());
            ui.label(format!("{} new/commit", config.draw.label()));
            ui.label("arrows edit active mode");
            ui.label("Shift faster");
            ui.label("Ctrl fine");
            ui.label(format!("{} resize mode", config.edit.label()));
            ui.label(format!("{} move mode", config.move_box.label()));
            ui.label(format!("{} focus", config.temporary_focus.label()));
            ui.label("Tab label");
            ui.label(format!("{} class", config.class_popup.label()));
            ui.label(format!("{} commit", config.confirm.label()));
            ui.label(format!("{} cancel", config.abort.label()));
        });
    }

    fn migration_ui(&mut self, ui: &mut eframe::egui::Ui) -> Option<bool> {
        let Some(texture_handle) = self.texture_handle.as_ref() else {
            return None;
        };
        let Some(image_size) = self.image_size else {
            return None;
        };
        let Some((annotation_index, pending_count)) =
            self.document.pending_migration(self.selected_class)
        else {
            return None;
        };
        let annotation = &self.document.annotations()[annotation_index];
        let class = annotation.class;
        let bounding_box_rect = annotation
            .bounding_box_ref()
            .expect("migration annotation has bbox")
            .rect;
        let crop_rect = migration_crop_rect(bounding_box_rect, image_size);

        let mut skip_clicked = false;
        ui.vertical_centered(|ui| {
            ui.heading("Migrate point feature");
            ui.label(format!(
                "Click the exact {} location inside this crop. The old box is preserved as a guide.",
                class.as_str()
            ));
            ui.label(format!(
                "{pending_count} point migrations remaining on this image"
            ));
            if ui.button("Skip keypoint").clicked() {
                skip_clicked = true;
            }
        });
        if skip_clicked {
            self.document.annotations_mut()[annotation_index].skip_point_migration();
            self.document.mark_dirty();
            return Some(true);
        }
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
            Stroke::new(1.0, class.color()),
            StrokeKind::Inside,
        );
        painter.rect_stroke(
            migration_box_display_rect(bounding_box_rect, crop_rect, display_rect),
            2.0,
            Stroke::new(2.0, class.color()),
            StrokeKind::Inside,
        );

        let mut annotations_changed = false;
        if response.clicked()
            && let Some(pointer) = response.interact_pointer_pos()
            && let Some(point) =
                crop_click_to_image_point(pointer, display_rect, crop_rect, image_size)
        {
            self.document.annotations_mut()[annotation_index].set_point(point);
            self.document.mark_dirty();
            annotations_changed = true;
        }

        Some(annotations_changed)
    }

    pub fn load_new_image_with_labels(
        &mut self,
        paths: Paths,
        model_annotations: &[AnnotationFormat],
    ) -> Result<()> {
        self.document
            .load_new_image_with_labels(paths, model_annotations)?;
        self.image_size = None;
        self.texture_handle = None;
        self.texture_load_error = None;
        self.canvas_state = CanvasState::default();

        Ok(())
    }

    pub fn save_annotation(&mut self) -> Result<()> {
        self.document.save(self.image_size)
    }
}

pub fn migration_crop_rect(bounding_box: Rect, image_size: [f32; 2]) -> Rect {
    let padding = (bounding_box.width().max(bounding_box.height()) * 0.7).max(16.0);
    let image_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(image_size[0], image_size[1]));
    bounding_box.expand(padding).intersect(image_rect)
}

fn migration_box_display_rect(bounding_box: Rect, crop_rect: Rect, display_rect: Rect) -> Rect {
    let min_relative = (bounding_box.min - crop_rect.min) / crop_rect.size();
    let max_relative = (bounding_box.max - crop_rect.min) / crop_rect.size();
    Rect::from_min_max(
        display_rect.min + min_relative * display_rect.size(),
        display_rect.min + max_relative * display_rect.size(),
    )
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

    #[test]
    fn migration_crop_adds_bbox_relative_context() {
        let crop = migration_crop_rect(
            Rect::from_min_max(Pos2::new(300.0, 300.0), Pos2::new(400.0, 500.0)),
            [1000.0, 1000.0],
        );

        assert_eq!(crop.min, Pos2::new(160.0, 160.0));
        assert_eq!(crop.max, Pos2::new(540.0, 640.0));
    }

    #[test]
    fn migration_bbox_guide_maps_into_display_crop() {
        let bounding_box = Rect::from_min_max(Pos2::new(50.0, 100.0), Pos2::new(150.0, 300.0));
        let crop = Rect::from_min_max(Pos2::new(10.0, 20.0), Pos2::new(190.0, 380.0));
        let display = Rect::from_min_size(Pos2::ZERO, Vec2::new(180.0, 360.0));

        let guide = migration_box_display_rect(bounding_box, crop, display);

        assert_eq!(guide.min, Pos2::new(40.0, 80.0));
        assert_eq!(guide.max, Pos2::new(140.0, 280.0));
    }
}
