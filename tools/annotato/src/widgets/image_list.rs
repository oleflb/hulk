use eframe::egui::{ProgressBar, ScrollArea, Widget};

use crate::{annotator_app::AnnotationPhase, paths::Paths};

use super::path_row::Row;

const ROW_HEIGHT: f32 = 12.0;

#[derive(Debug, Default)]
pub struct ImageListState {
    selected_index: Option<usize>,
}

impl ImageListState {
    fn scroll_target(&mut self, phase: &AnnotationPhase) -> Option<usize> {
        let selected_index = selected_index(phase);
        let changed = self.selected_index != selected_index;
        self.selected_index = selected_index;
        changed.then_some(selected_index).flatten()
    }
}

pub struct ImageList<'a> {
    paths: &'a [Paths],
    phase: &'a mut AnnotationPhase,
    state: &'a mut ImageListState,
}

impl<'a> ImageList<'a> {
    pub fn new(
        paths: &'a [Paths],
        phase: &'a mut AnnotationPhase,
        state: &'a mut ImageListState,
    ) -> Self {
        Self {
            paths,
            phase,
            state,
        }
    }
}

impl Widget for ImageList<'_> {
    fn ui(self, ui: &mut eframe::egui::Ui) -> eframe::egui::Response {
        ui.vertical(|ui| {
            let scroll_target = self.state.scroll_target(self.phase);
            ui.label("Images");
            let images_done = self
                .paths
                .iter()
                .filter(|paths| paths.label_present)
                .count();
            ui.add(
                ProgressBar::new(images_done as f32 / self.paths.len().max(1) as f32)
                    .show_percentage()
                    .text(format!("{}/{}", images_done, self.paths.len())),
            );
            ui.separator();

            let mut scroll_area = ScrollArea::vertical()
                .auto_shrink([true, false])
                .max_height(ui.available_height().max(0.0));
            if let Some(index) = scroll_target {
                scroll_area = scroll_area.vertical_scroll_offset(index as f32 * ROW_HEIGHT);
            }

            scroll_area.show_rows(ui, ROW_HEIGHT, self.paths.len(), |ui, range| {
                for index in range {
                    let paths = &self.paths[index];
                    let row = ui.add(Row::new(paths).highlight(matches!(
                        self.phase,
                        AnnotationPhase::Labelling { current_index } if *current_index == index
                    )));

                    if row.clicked() {
                        *self.phase = AnnotationPhase::Labelling {
                            current_index: index,
                        };
                    }
                    ui.separator();
                }
            });
        })
        .response
    }
}

fn selected_index(phase: &AnnotationPhase) -> Option<usize> {
    match phase {
        AnnotationPhase::Labelling { current_index } => Some(*current_index),
        AnnotationPhase::Finished => None,
    }
}
