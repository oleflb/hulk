use crate::panel::{Panel, PanelCreationContext, PanelUiContext};
use eframe::egui::{Slider, Ui};
use egui_material_icons::icons;

pub struct TimelinePanel;

impl Panel for TimelinePanel {
    const STORAGE_ID: &'static str = "timeline";
    const DISPLAY_NAME: &'static str = "Timeline";
    const ICON: &'static str = icons::ICON_TIMELINE.codepoint;

    fn new(_context: PanelCreationContext<'_>) -> Self {
        Self
    }

    fn ui(&mut self, ui: &mut Ui, context: PanelUiContext<'_>) {
        let mut replay = context.backend.replay();
        if let Some(error) = replay.error.clone() {
            ui.label(error);
            if ui.button("Retry").clicked() {
                replay.retry();
            }
        }
        let Some(status) = replay.status.as_ref() else {
            ui.label("Waiting for replay");
            return;
        };
        let (start, end, playing) = (status.start, status.end, status.playing);
        let mut seconds = replay.position().unwrap_or(start).saturating_sub(start) as f64 / 1e9;
        ui.add_enabled_ui(replay.error.is_none(), |ui| {
            if ui
                .add(Slider::new(&mut seconds, 0.0..=(end - start) as f64 / 1e9).suffix(" s"))
                .changed()
            {
                replay.seek(start + ((seconds * 1e9).round() as u64).min(end - start));
            }
            if ui
                .add_enabled(
                    !replay.is_seeking(),
                    eframe::egui::Button::new(if playing { "Pause" } else { "Play" }),
                )
                .clicked()
            {
                replay.set_playing(!playing);
            }
        });
    }
}
