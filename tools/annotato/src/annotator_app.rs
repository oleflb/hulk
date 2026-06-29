use std::{
    error::Error as StdError,
    fmt::{self, Display},
    fs,
    path::{Path, PathBuf},
};

use crate::{
    ai_assistant::ModelAnnotations,
    annotation::LabelFileFormat,
    classes::Class,
    label_widget::LabelWidget,
    paths::Paths,
    user_toml::{CONFIG, KeyBind},
    widgets::{
        image_list::{ImageList, ImageListState},
        keybind_preview::KeybindPreview,
    },
    workflow::{
        ChunkPosition, ChunkProgress, ChunkWorkflow, ClassTransition, ClassTransitionDirection,
    },
};
use color_eyre::{
    Result,
    eyre::{Context as C, Report, bail},
};
use eframe::{
    App, CreationContext,
    egui::{
        CentralPanel, Context, Grid, Key, KeyboardShortcut, Panel, ProgressBar, Response, RichText,
        Ui,
    },
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AnnotationPhase {
    Labelling { current_index: usize },
    Finished,
}

pub struct AnnotatorApp {
    phase: AnnotationPhase,
    workflow: ChunkWorkflow,
    paths: Vec<Paths>,
    label_widget: LabelWidget,
    image_list_state: ImageListState,
    model_annotations: ModelAnnotations,
    class_transition: Option<ClassTransition>,
    manual_class_edit: bool,
    manual_class_edit_changed: bool,
    next_was_down: bool,
    previous_was_down: bool,
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

        let mut app = AnnotatorApp {
            phase: AnnotationPhase::Labelling { current_index: 0 },
            workflow: ChunkWorkflow::default(),
            paths,
            label_widget: LabelWidget::default(),
            image_list_state: ImageListState::default(),
            model_annotations,
            class_transition: None,
            manual_class_edit: false,
            manual_class_edit_changed: false,
            next_was_down: false,
            previous_was_down: false,
            last_error: None,
        };
        app.move_to_first_pending();
        Ok(app)
    }

    fn current_index(&self) -> Option<usize> {
        match self.phase {
            AnnotationPhase::Labelling { current_index } => Some(current_index),
            AnnotationPhase::Finished => None,
        }
    }

    fn record_error(&mut self, error: Report) {
        self.last_error = Some(format!("{error:#}"));
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

    fn save_current_if_dirty(&mut self) -> Result<()> {
        if self.label_widget.is_dirty() {
            self.save_current()?;
        }
        Ok(())
    }

    fn active_class(&self) -> Class {
        self.workflow.active_class()
    }

    fn complete_current_for_active_class(&mut self) -> Result<()> {
        let active_class = self.active_class();
        self.load_image()?;
        if self
            .label_widget
            .has_pending_migration_for_class(active_class)
        {
            bail!(
                "finish pending {} keypoint migration before advancing",
                active_class.as_str()
            );
        }

        self.label_widget.mark_labeled_class(active_class);
        self.save_current()
    }

    fn next(&mut self) -> Result<()> {
        if self.handle_class_transition_command(ClassTransitionDirection::Next) {
            return Ok(());
        }

        let Some(current_index) = self.current_index() else {
            return Ok(());
        };
        self.complete_current_for_active_class()
            .wrap_err("failed to go to next image")?;

        if let Some(position) = self.next_pending_position_after(current_index) {
            self.apply_position_with_class_transition(position, ClassTransitionDirection::Next);
        } else {
            self.manual_class_edit = false;
            self.manual_class_edit_changed = false;
            self.phase = AnnotationPhase::Finished;
        }
        Ok(())
    }

    fn previous(&mut self) -> Result<()> {
        if self.handle_class_transition_command(ClassTransitionDirection::Previous) {
            return Ok(());
        }

        match self.phase {
            AnnotationPhase::Labelling { current_index } => {
                self.save_current_if_dirty()
                    .wrap_err("failed to go to previous image")?;
                if let Some(position) = self.previous_position_before(current_index) {
                    self.apply_position_with_class_transition(
                        position,
                        ClassTransitionDirection::Previous,
                    );
                }
            }
            AnnotationPhase::Finished => {
                let class_index = Class::ALL.len() - 1;
                self.apply_position_with_class_transition(
                    ChunkPosition::new(self.paths.len() - 1, class_index),
                    ClassTransitionDirection::Previous,
                );
            }
        }
        Ok(())
    }

    fn go_to(&mut self, index: usize) -> Result<()> {
        if self.class_transition.is_some() {
            return Ok(());
        }

        if Some(index) == self.current_index() {
            return Ok(());
        }

        self.save_current_if_dirty()
            .wrap_err("failed to switch image")?;
        self.phase = AnnotationPhase::Labelling {
            current_index: index,
        };
        self.leave_manual_class_edit();
        Ok(())
    }

    fn load_image(&mut self) -> Result<()> {
        let Some(index) = self.current_index() else {
            return Ok(());
        };
        let active_class = self.active_class();
        if !self.manual_class_edit {
            self.label_widget.set_selected_class(active_class);
        }

        if let Some(paths) = self.paths.get_mut(index) {
            if self.label_widget.has_paths(paths) {
                return Ok(());
            }

            let annotations = paths
                .image_path
                .file_name()
                .and_then(|file_name| file_name.to_str())
                .and_then(|file_name| self.model_annotations.for_image(file_name))
                .unwrap_or(Vec::new());

            self.label_widget
                .load_new_image_with_labels(paths.clone(), &annotations)?;
            paths.check_existence();
        }

        Ok(())
    }

    fn move_to_first_pending(&mut self) {
        if let Some(position) = self.first_pending_position() {
            self.apply_position(position);
        } else {
            self.manual_class_edit = false;
            self.manual_class_edit_changed = false;
            self.phase = AnnotationPhase::Finished;
        }
    }

    fn apply_position(&mut self, position: ChunkPosition) {
        self.class_transition = None;
        self.manual_class_edit = false;
        self.manual_class_edit_changed = false;
        self.workflow.apply_position(position);
        self.label_widget.set_selected_class(self.active_class());
        self.phase = AnnotationPhase::Labelling {
            current_index: position.index,
        };
    }

    fn apply_position_with_class_transition(
        &mut self,
        position: ChunkPosition,
        direction: ClassTransitionDirection,
    ) {
        if self.workflow.is_active_class(position) {
            self.apply_position(position);
        } else {
            self.leave_manual_class_edit();
            self.class_transition = Some(ClassTransition::new(position, direction));
        }
    }

    fn enter_manual_class_edit(&mut self, open_popup: bool) {
        if self.current_index().is_none() || self.class_transition.is_some() {
            return;
        }

        self.manual_class_edit = true;
        self.manual_class_edit_changed = false;
        if open_popup {
            self.label_widget.open_class_popup();
        }
    }

    fn leave_manual_class_edit(&mut self) {
        self.manual_class_edit = false;
        self.manual_class_edit_changed = false;
        self.label_widget.set_selected_class(self.active_class());
    }

    fn handle_class_transition_command(&mut self, direction: ClassTransitionDirection) -> bool {
        let Some(transition) = self.class_transition else {
            return false;
        };

        self.class_transition = None;
        if transition.direction == direction {
            self.apply_position(transition.position);
        }

        true
    }

    fn first_pending_position(&self) -> Option<ChunkPosition> {
        ChunkWorkflow::first_pending_position(self.paths.len(), |index, class| {
            self.class_is_complete_for_index(index, class)
        })
    }

    fn next_pending_position_after(&self, current_index: usize) -> Option<ChunkPosition> {
        self.workflow.next_pending_position_after(
            current_index,
            self.paths.len(),
            |index, class| self.class_is_complete_for_index(index, class),
        )
    }

    fn previous_position_before(&self, current_index: usize) -> Option<ChunkPosition> {
        self.workflow
            .previous_position_before(current_index, self.paths.len())
    }

    fn class_is_complete_for_index(&self, index: usize, class: Class) -> bool {
        let Some(label_file) = self.label_file_for_index(index) else {
            return false;
        };
        label_file.class_is_labeled(class) && !label_file.has_pending_migration_for_class(class)
    }

    fn label_file_for_index(&self, index: usize) -> Option<LabelFileFormat> {
        let label_path = &self.paths.get(index)?.label_path;
        let label_file = fs::read_to_string(label_path).ok()?;
        serde_json::from_str(&label_file).ok()
    }

    fn chunk_progress(&self) -> Option<ChunkProgress> {
        let current_index = self.current_index()?;
        Some(
            self.workflow
                .chunk_progress(current_index, self.paths.len(), |index, class| {
                    self.class_is_complete_for_index(index, class)
                }),
        )
    }

    fn handle_global_shortcuts(&mut self, ctx: &Context) {
        if self.label_widget.captures_keyboard() {
            return;
        }

        let config = &CONFIG.get().unwrap().keybindings;
        let action = ctx.input(|input| {
            let next_down = config.next.is_down(input);
            let previous_down = config.previous.is_down(input);
            let navigation_action = if next_down && !self.next_was_down {
                Some(GlobalAction::Next)
            } else if previous_down && !self.previous_was_down {
                Some(GlobalAction::Previous)
            } else if config.save.is_pressed(input) {
                Some(GlobalAction::Save)
            } else if !self.manual_class_edit && config.class_popup.is_pressed(input) {
                Some(GlobalAction::EditOtherClass)
            } else {
                None
            };

            self.next_was_down = next_down;
            self.previous_was_down = previous_down;
            navigation_action
        });

        let result = match action {
            Some(GlobalAction::Next) => self.next(),
            Some(GlobalAction::Previous) => self.previous(),
            Some(GlobalAction::Save) => self.save_current(),
            Some(GlobalAction::EditOtherClass) => {
                self.enter_manual_class_edit(false);
                Ok(())
            }
            None => Ok(()),
        };

        if let Err(error) = result {
            self.record_error(error);
        }
    }

    fn show_sidebar(&mut self, ui: &mut Ui) {
        Panel::left("image-path-list")
            .default_size(280.0)
            .resizable(true)
            .show(ui, |ui| {
                Panel::bottom("sidebar-footer").show(ui, |ui| {
                    self.show_sidebar_footer(ui);
                });

                let mut current_phase = self.phase.clone();
                CentralPanel::no_frame().show(ui, |ui| {
                    ui.add(ImageList::new(
                        &self.paths,
                        &mut current_phase,
                        &mut self.image_list_state,
                    ));
                });

                if current_phase != self.phase
                    && let AnnotationPhase::Labelling { current_index } = current_phase
                    && let Err(error) = self.go_to(current_index)
                {
                    self.record_error(error);
                }
            });
    }

    fn show_sidebar_footer(&mut self, ui: &mut Ui) -> Response {
        ui.vertical(|ui| {
            let config = &CONFIG.get().unwrap().keybindings;
            if let Some(progress) = self.chunk_progress() {
                ui.label(RichText::new("Chunk progress").strong());
                ui.add(
                    ProgressBar::new(progress.fraction())
                        .show_percentage()
                        .text(progress.label()),
                );
            }

            ui.add_space(4.0);

            let can_edit_other_class =
                self.current_index().is_some() && self.class_transition.is_none();
            if self.manual_class_edit {
                ui.label(format!(
                    "Editing {}, chunk class is {}",
                    self.label_widget.selected_class().as_str(),
                    self.active_class().as_str()
                ));
                if ui.button("Return to chunk class").clicked() {
                    self.leave_manual_class_edit();
                }
            } else if can_edit_other_class
                && ui
                    .button("Edit other class")
                    .on_hover_text(config.class_popup.label())
                    .clicked()
            {
                self.enter_manual_class_edit(true);
            }

            ui.add_space(4.0);

            ui.horizontal(|ui| {
                if ui
                    .button("Previous")
                    .on_hover_text(config.previous.label())
                    .clicked()
                    && let Err(error) = self.previous()
                {
                    self.record_error(error);
                }
                if ui
                    .button("Next")
                    .on_hover_text(config.next.label())
                    .clicked()
                    && let Err(error) = self.next()
                {
                    self.record_error(error);
                }
            });

            if ui.button("First incomplete").clicked() && self.class_transition.is_none() {
                if let Err(error) = self.save_current_if_dirty() {
                    self.record_error(error);
                } else if let Some(position) = self.first_pending_position() {
                    self.apply_position_with_class_transition(
                        position,
                        ClassTransitionDirection::Next,
                    );
                } else {
                    self.manual_class_edit = false;
                    self.manual_class_edit_changed = false;
                    self.phase = AnnotationPhase::Finished;
                }
            }

            ui.add_space(8.0);
            Self::show_keybinds(ui);
        })
        .response
    }

    fn show_keybinds(ui: &mut Ui) {
        let config = &CONFIG.get().unwrap().keybindings;
        ui.separator();
        ui.label(RichText::new("Keybinds").strong());
        let keybinds = [
            ("Next", &config.next),
            ("Previous", &config.previous),
            ("Save", &config.save),
            ("New/commit", &config.draw),
            ("Resize mode", &config.edit),
            ("Move mode", &config.move_box),
            ("Focus", &config.temporary_focus),
            ("Class/edit other", &config.class_popup),
            ("Confirm", &config.confirm),
            ("Cancel", &config.abort),
            ("Delete", &config.delete),
        ];
        Grid::new("annotato-keybinds")
            .num_columns(4)
            .show(ui, |ui| {
                for chunk in keybinds.chunks(2) {
                    for (label, keybind) in chunk {
                        Self::show_keybind_entry(ui, label, keybind);
                    }
                    if chunk.len() == 1 {
                        ui.label("");
                        ui.label("");
                    }
                    ui.end_row();
                }
            });
    }

    fn show_keybind_entry(ui: &mut Ui, label: &str, keybind: &KeyBind) {
        ui.horizontal(|ui| {
            Self::show_keybind_preview(ui, keybind, keybind.primary);
            for alternative in &keybind.alternatives {
                ui.label("/");
                Self::show_keybind_preview(ui, keybind, *alternative);
            }
        });
        ui.label(label);
    }

    fn show_keybind_preview(ui: &mut Ui, keybind: &KeyBind, key: Key) {
        ui.add(KeybindPreview(KeyboardShortcut::new(
            keybind.modifiers,
            key,
        )));
    }

    fn show_top_bar(&mut self, ui: &mut Ui) {
        Panel::top("top-bar").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("annotato").strong());
                ui.separator();
                if let Some(transition) = self.class_transition {
                    let class = transition.position.class();
                    ui.label(format!(
                        "{}: {}",
                        transition.direction.label(),
                        class.as_str()
                    ));
                } else if self.manual_class_edit {
                    ui.label(format!(
                        "Manual class edit: {}",
                        self.label_widget.selected_class().as_str()
                    ));
                } else {
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
                }

                if let Some(error) = &self.last_error {
                    ui.separator();
                    ui.colored_label(eframe::egui::Color32::from_rgb(243, 139, 168), error);
                }
            });
        });
    }

    fn show_class_transition(&mut self, ui: &mut Ui) {
        let Some(transition) = self.class_transition else {
            return;
        };
        let class = transition.position.class();
        let config = &CONFIG.get().unwrap().keybindings;
        let (confirm, cancel) = match transition.direction {
            ClassTransitionDirection::Next => (config.next.label(), config.previous.label()),
            ClassTransitionDirection::Previous => (config.previous.label(), config.next.label()),
        };

        CentralPanel::default().show(ui, |ui| {
            ui.centered_and_justified(|ui| {
                ui.vertical_centered(|ui| {
                    ui.label(
                        RichText::new(transition.direction.label())
                            .size(22.0)
                            .strong(),
                    );
                    ui.add_space(12.0);
                    ui.label(RichText::new(class.as_str()).size(48.0).strong());
                    ui.add_space(12.0);
                    ui.label(format!(
                        "Press {confirm} to continue, or {cancel} to stay here"
                    ));
                });
            });
        });
    }

    fn show_labelling(&mut self, ui: &mut Ui) {
        CentralPanel::default().show(ui, |ui| {
            if let Err(error) = self.load_image() {
                self.record_error(error);
                return;
            }
            match self.label_widget.ui(ui, !self.manual_class_edit) {
                Ok(annotations_changed) => {
                    if self.manual_class_edit && annotations_changed {
                        self.manual_class_edit_changed = true;
                    }
                    if self.manual_class_edit
                        && self.manual_class_edit_changed
                        && !self.label_widget.pointer_interaction_active()
                    {
                        self.leave_manual_class_edit();
                    }
                }
                Err(error) => self.record_error(error),
            }
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
                        self.record_error(error);
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
    EditOtherClass,
}

impl App for AnnotatorApp {
    fn logic(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        self.handle_global_shortcuts(ctx);
    }

    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        self.show_top_bar(ui);
        self.show_sidebar(ui);

        if self.class_transition.is_some() {
            self.show_class_transition(ui);
        } else {
            match self.phase {
                AnnotationPhase::Labelling { .. } => self.show_labelling(ui),
                AnnotationPhase::Finished => self.show_finished(ui),
            }
        }
    }

    fn save(&mut self, _storage: &mut dyn eframe::Storage) {
        if let Err(error) = self.save_current_if_dirty() {
            self.record_error(error);
        }
    }
}

pub fn eframe_error_to_report(error: eframe::Error) -> Report {
    match error {
        eframe::Error::AppCreation(error) => Report::new(AppCreationError(error)),
        error => Report::new(error),
    }
}

#[derive(Debug)]
struct AppCreationError(Box<dyn StdError + Send + Sync>);

impl Display for AppCreationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "app creation error")
    }
}

impl StdError for AppCreationError {
    fn source(&self) -> Option<&(dyn StdError + 'static)> {
        Some(&*self.0)
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
