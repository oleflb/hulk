use eframe::egui::{ProgressBar, ScrollArea, Widget};

use crate::{annotator_app::AnnotationPhase, paths::Paths};

use super::path_row::Row;
pub struct ImageList<'a> {
    paths: &'a [Paths],
    phase: &'a mut AnnotationPhase,
}

impl<'a> ImageList<'a> {
    pub fn new(paths: &'a [Paths], phase: &'a mut AnnotationPhase) -> Self {
        Self { paths, phase }
    }
}

impl Widget for ImageList<'_> {
    fn ui(self, ui: &mut eframe::egui::Ui) -> eframe::egui::Response {
        ui.vertical(|ui| {
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

            ScrollArea::vertical()
                .auto_shrink([true, false])
                .max_height(0.8 * ui.available_height())
                .show_rows(ui, 12.0, self.paths.len(), |ui, range| {
                    for index in range {
                        let paths = &self.paths[index];
                        let row = ui.add(Row::new(paths).highlight(matches!(self.phase, AnnotationPhase::Labelling { current_index } if *current_index == index)));
                        if let AnnotationPhase::Labelling { current_index } = self.phase
                            && *current_index == index
                        {
                            row.scroll_to_me(None);
                        }

                        if row.clicked() {
                            *self.phase = AnnotationPhase::Labelling {
                                current_index: index,
                            };
                        }
                        ui.separator();
                    }
                });
            ui.separator();
        })
        .response
    }
}
