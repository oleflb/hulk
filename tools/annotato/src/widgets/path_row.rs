use eframe::{
    egui::{Response, RichText, Sense, TextStyle, TextWrapMode, Ui, Widget, WidgetText, vec2},
    epaint::Color32,
};

use crate::paths::Paths;

pub struct Row<'a> {
    paths: &'a Paths,
    highlight: bool,
}

impl<'a> Row<'a> {
    pub fn new(paths: &'a Paths) -> Self {
        Self {
            paths,
            highlight: false,
        }
    }

    pub fn highlight(mut self, highlight: bool) -> Self {
        self.highlight = highlight;
        self
    }
}

impl Widget for Row<'_> {
    fn ui(self, ui: &mut Ui) -> Response {
        let filename = self
            .paths
            .image_path
            .file_name()
            .and_then(|file_name| file_name.to_str())
            .unwrap_or("<invalid file name>")
            .to_string();
        let is_labelled = self.paths.label_present;

        let text: WidgetText = RichText::new(filename).monospace().into();
        let check_mark: WidgetText = if is_labelled {
            RichText::new("✅").color(Color32::GREEN)
        } else {
            RichText::new("❌").color(Color32::RED)
        }
        .into();

        const CHECKMARK_WIDTH: f32 = 20.0;
        const MARGIN_V: f32 = 3.0;
        let text_width = (ui.available_width() - CHECKMARK_WIDTH).max(0.0);

        let text = text.into_galley(
            ui,
            Some(TextWrapMode::Truncate),
            text_width,
            TextStyle::Button,
        );
        let check_mark = check_mark.into_galley(
            ui,
            Some(TextWrapMode::Extend),
            CHECKMARK_WIDTH,
            TextStyle::Button,
        );
        let response = ui.allocate_response(
            vec2(ui.available_width(), text.size().y + MARGIN_V),
            Sense::click(),
        );

        let painter = ui.painter_at(response.rect);
        let visuals = ui.style().interact(&response);
        if response.hovered() || self.highlight {
            painter.rect_filled(response.rect, 2.0, visuals.bg_fill);
        }
        painter.galley(
            response.rect.left_top() + vec2(0.0, MARGIN_V / 2.),
            text,
            visuals.text_color(),
        );
        painter.galley(
            response.rect.left_top() + vec2(text_width, MARGIN_V / 2.),
            check_mark,
            visuals.text_color(),
        );

        response
    }
}
