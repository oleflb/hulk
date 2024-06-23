use std::path::PathBuf;

use eframe::egui::{
    vec2, Color32, Image, Layout, Response, ScrollArea, Sense, Stroke, Ui, Widget,
};

pub struct PreviewWidget<'ui> {
    images: &'ui [PathBuf],
    current: &'ui mut Option<usize>,
}

impl<'ui> PreviewWidget<'ui> {
    pub fn new(images: &'ui [PathBuf], current: &'ui mut Option<usize>) -> Self {
        PreviewWidget { images, current }
    }
}

impl<'ui> Widget for PreviewWidget<'ui> {
    fn ui(self, ui: &mut Ui) -> Response {
        const ASPECT_RATIO: f32 = 640.0 / 480.0;
        const IMAGE_HEIGHT: f32 = 200.0;
        const IMAGE_WIDTH: f32 = IMAGE_HEIGHT * ASPECT_RATIO;

        ScrollArea::horizontal()
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    for (index, image) in self.images.iter().enumerate() {
                        let uri = format!("file://{}", image.display());
                        let (rect, _) =
                            ui.allocate_exact_size(vec2(IMAGE_WIDTH, IMAGE_WIDTH), Sense::click());
                        if ui.is_rect_visible(rect) {
                            let mut ui = ui.child_ui(rect, Layout::default());
                            let image = Image::new(uri)
                                .show_loading_spinner(true)
                                .maintain_aspect_ratio(true)
                                .rounding(10.0)
                                .shrink_to_fit()
                                .sense(Sense::click());
                            let image_response = ui.add(image);

                            if matches!(*self.current, Some(selected) if selected == index) {
                                ui.painter().rect_stroke(
                                    image_response.rect,
                                    10.0,
                                    Stroke::new(5.0, Color32::LIGHT_BLUE),
                                );
                            }
                            if image_response.clicked() {
                                *self.current = Some(index);
                            }
                        }
                    }
                })
                .response
            })
            .inner
    }
}
