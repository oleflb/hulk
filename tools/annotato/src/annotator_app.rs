use std::path::{Path, PathBuf};

use crate::{
    ai_assistant::ModelAnnotations, label_widget::LabelWidget, paths::Paths, user_toml::CONFIG,
    widgets::image_list::ImageList,
};
use color_eyre::{Result, eyre::Context as C};
use eframe::{
    App, CreationContext,
    egui::{CentralPanel, Context, Panel, RichText, Ui},
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AnnotationPhase {
    Labelling { current_index: usize },
    Finished,
}

pub struct AnnotatorApp {
    phase: AnnotationPhase,
    paths: Vec<Paths>,
    label_widget: LabelWidget,
    model_annotations: ModelAnnotations,
    last_error: Option<String>,
}

impl AnnotatorApp {
    pub fn convert_image_to_label_path(image_path: &Path) -> PathBuf {
        image_path.with_extension("json")
    }

    pub fn try_new(
        _: &CreationContext,
        image_paths: Vec<PathBuf>,
        predictions: Option<PathBuf>,
    ) -> Result<Self> {
        let model_annotations = predictions
            .map(ModelAnnotations::try_new)
            .transpose()?
            .unwrap_or_default();

        let paths = image_paths
            .into_iter()
            .map(|image_path| {
                let label_path = Self::convert_image_to_label_path(&image_path);
                Paths::new(image_path, label_path)
            })
            .collect::<Vec<_>>();

        Ok(AnnotatorApp {
            phase: AnnotationPhase::Labelling { current_index: 0 },
            paths,
            label_widget: LabelWidget::default(),
            model_annotations,
            last_error: None,
        })
    }

    fn current_index(&self) -> Option<usize> {
        match self.phase {
            AnnotationPhase::Labelling { current_index } => Some(current_index),
            AnnotationPhase::Finished => None,
        }
    }

    fn save_current(&mut self) -> Result<()> {
        let Some(current_index) = self.current_index() else {
            return Ok(());
        };

        self.label_widget
            .save_annotation()
            .wrap_err("failed to save annotation")?;
        if let Some(paths) = self.paths.get_mut(current_index) {
            paths.check_existence();
        }
        Ok(())
    }

    fn next(&mut self) -> Result<()> {
        let Some(current_index) = self.current_index() else {
            return Ok(());
        };
        self.save_current().wrap_err("failed to go to next image")?;

        let new_index = current_index + 1;
        if new_index < self.paths.len() {
            self.phase = AnnotationPhase::Labelling {
                current_index: new_index,
            };
        } else {
            self.phase = AnnotationPhase::Finished;
        }
        Ok(())
    }

    fn previous(&mut self) -> Result<()> {
        match self.phase {
            AnnotationPhase::Labelling { current_index } => {
                self.save_current()
                    .wrap_err("failed to go to previous image")?;
                self.phase = AnnotationPhase::Labelling {
                    current_index: current_index.saturating_sub(1),
                };
            }
            AnnotationPhase::Finished => {
                self.phase = AnnotationPhase::Labelling {
                    current_index: self.paths.len() - 1,
                };
            }
        }
        Ok(())
    }

    fn go_to(&mut self, index: usize) -> Result<()> {
        if Some(index) == self.current_index() {
            return Ok(());
        }

        self.save_current().wrap_err("failed to switch image")?;
        self.phase = AnnotationPhase::Labelling {
            current_index: index,
        };
        Ok(())
    }

    fn load_image(&mut self) -> Result<()> {
        let Some(index) = self.current_index() else {
            return Ok(());
        };

        if let Some(paths) = self.paths.get_mut(index) {
            if self.label_widget.has_paths(paths) {
                return Ok(());
            }

            let annotations = paths
                .image_path
                .file_name()
                .and_then(|file_name| file_name.to_str())
                .and_then(|file_name| self.model_annotations.for_image(file_name))
                .unwrap_or_default();

            self.label_widget
                .load_new_image_with_labels(paths.clone(), annotations)?;
        }

        self.paths.iter_mut().for_each(Paths::check_existence);

        Ok(())
    }

    fn handle_global_shortcuts(&mut self, ctx: &Context) {
        let config = &CONFIG.get().unwrap().keybindings;
        let action = ctx.input(|input| {
            if config.next.is_pressed(input) || input.key_pressed(eframe::egui::Key::N) {
                Some(GlobalAction::Next)
            } else if config.previous.is_pressed(input) || input.key_pressed(eframe::egui::Key::P) {
                Some(GlobalAction::Previous)
            } else if config.save.is_pressed(input) {
                Some(GlobalAction::Save)
            } else {
                None
            }
        });

        let result = match action {
            Some(GlobalAction::Next) => self.next(),
            Some(GlobalAction::Previous) => self.previous(),
            Some(GlobalAction::Save) => self.save_current(),
            None => Ok(()),
        };

        if let Err(error) = result {
            self.last_error = Some(error.to_string());
        }
    }

    fn show_sidebar(&mut self, ui: &mut Ui) {
        Panel::left("image-path-list")
            .default_size(280.0)
            .resizable(true)
            .show(ui, |ui| {
                let mut current_phase = self.phase.clone();
                ui.add(ImageList::new(&self.paths, &mut current_phase));

                if current_phase != self.phase
                    && let AnnotationPhase::Labelling { current_index } = current_phase
                    && let Err(error) = self.go_to(current_index)
                {
                    self.last_error = Some(error.to_string());
                }

                ui.horizontal(|ui| {
                    if ui
                        .button("Previous")
                        .on_hover_text("Shift+Space or P")
                        .clicked()
                        && let Err(error) = self.previous()
                    {
                        self.last_error = Some(error.to_string());
                    }
                    if ui.button("Next").on_hover_text("Space or N").clicked()
                        && let Err(error) = self.next()
                    {
                        self.last_error = Some(error.to_string());
                    }
                });

                if ui.button("First unlabelled").clicked() {
                    if let Some((unlabelled_index, _)) = self
                        .paths
                        .iter()
                        .enumerate()
                        .find(|(_, paths)| !paths.label_present)
                    {
                        if let Err(error) = self.go_to(unlabelled_index) {
                            self.last_error = Some(error.to_string());
                        }
                    } else {
                        self.phase = AnnotationPhase::Finished;
                    }
                }
            });
    }

    fn show_top_bar(&mut self, ui: &mut Ui) {
        Panel::top("top-bar").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("annotato").strong());
                ui.separator();
                match self.phase {
                    AnnotationPhase::Labelling { current_index } => {
                        ui.label(format!(
                            "Image {} of {}",
                            current_index + 1,
                            self.paths.len()
                        ));
                    }
                    AnnotationPhase::Finished => {
                        ui.label(format!("{} images complete", self.paths.len()));
                    }
                }
                ui.separator();
                ui.label("Space/N next");
                ui.label("Shift+Space/P previous");
                ui.label("S save");

                if let Some(error) = &self.last_error {
                    ui.separator();
                    ui.colored_label(eframe::egui::Color32::from_rgb(243, 139, 168), error);
                }
            });
        });
    }

    fn show_labelling(&mut self, ui: &mut Ui) {
        CentralPanel::default().show(ui, |ui| {
            if let Err(error) = self.load_image() {
                self.last_error = Some(error.to_string());
                return;
            }
            self.label_widget.ui(ui);
        });
    }

    fn show_finished(&mut self, ui: &mut Ui) {
        CentralPanel::default().show(ui, |ui| {
            ui.centered_and_justified(|ui| {
                ui.vertical_centered(|ui| {
                    ui.label(RichText::new("All images are labelled").size(28.0).strong());
                    if ui.button("Go back to last image").clicked()
                        && let Err(error) = self.previous()
                    {
                        self.last_error = Some(error.to_string());
                    }
                });
            });
        });
    }
}

#[derive(Debug, Clone, Copy)]
enum GlobalAction {
    Next,
    Previous,
    Save,
}

impl App for AnnotatorApp {
    fn logic(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        self.handle_global_shortcuts(ctx);
    }

    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        egui_extras::install_image_loaders(&ctx);

        self.show_top_bar(ui);
        self.show_sidebar(ui);

        match self.phase {
            AnnotationPhase::Labelling { .. } => self.show_labelling(ui),
            AnnotationPhase::Finished => self.show_finished(ui),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;

    #[test]
    fn label_path_is_sidecar_json_next_to_image() {
        assert_eq!(
            AnnotatorApp::convert_image_to_label_path(Path::new("images/frame.jpeg")),
            PathBuf::from("images/frame.json")
        );
    }
}
