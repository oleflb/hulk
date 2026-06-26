use std::{
    fs::{self, File},
    io::Write,
};

use crate::{
    annotation::AnnotationFormat,
    boundingbox::BoundingBox,
    classes::Class,
    paths::Paths,
    utils,
    widgets::{
        bounding_box_annotator::{BoundingBoxAnnotator, CanvasState},
        class_selector::ClassSelector,
    },
};
use color_eyre::eyre::{ContextCompat, Result};
use eframe::{
    egui::{Color32, RichText},
    epaint::TextureHandle,
};

pub struct LabelWidget {
    current_paths: Option<Paths>,
    texture_handle: Option<TextureHandle>,
    image_size: Option<[f32; 2]>,
    selected_class: Class,
    bounding_boxes: Vec<BoundingBox>,
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
            bounding_boxes: Vec::new(),
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
            if let (Some(texture_handle), Some(image_size)) =
                (self.texture_handle.clone(), self.image_size)
            {
                ui.add(BoundingBoxAnnotator::new(
                    texture_handle,
                    image_size,
                    &mut self.bounding_boxes,
                    &mut self.canvas_state,
                    &mut self.selected_class,
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

        self.bounding_boxes = self
            .unresolved_annotations
            .drain(..)
            .map(|annotation| BoundingBox::from_annotation(annotation, size))
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
                    .apply_class(&mut self.bounding_boxes, self.selected_class);
            }

            ui.separator();
            ui.label(format!("{} boxes", self.bounding_boxes.len()));
            if let Some(index) = self.canvas_state.selected_box() {
                ui.label(format!("selected #{}", index + 1));
            }
        });
    }

    fn help_ui(&self, ui: &mut eframe::egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Mouse:").strong());
            ui.label("drag create");
            ui.label("drag box move");
            ui.label("drag handle resize");
            ui.label("right click delete");
            ui.separator();
            ui.label(RichText::new("Keyboard:").strong());
            ui.label("B new/commit");
            ui.label("arrows move/resize");
            ui.label("Shift faster");
            ui.label("Ctrl fine");
            ui.label("E resize");
            ui.label("M move");
            ui.label("C corner");
            ui.label("Tab box");
            ui.label("1-8 or [] class");
            ui.label("Enter commit");
            ui.label("Esc cancel");
        });
    }

    pub fn load_new_image_with_labels(
        &mut self,
        paths: Paths,
        model_annotations: Vec<AnnotationFormat>,
    ) -> Result<()> {
        self.bounding_boxes.clear();
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
            .bounding_boxes
            .iter()
            .map(|bounding_box| bounding_box.to_annotation(image_size))
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
