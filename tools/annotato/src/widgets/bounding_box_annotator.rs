mod drawing;
mod keyboard;
mod pointer;
mod state;
mod transform;
mod view;

use eframe::egui::{CursorIcon, Response, Sense, Ui, Vec2, Widget};
use eframe::epaint::TextureHandle;

use crate::{annotation::Annotation, classes::Class};

pub use state::{AnnotationShape, CanvasState, CreationShape, Selection};
use transform::ImageTransform;

pub(super) const HANDLE_RADIUS: f32 = 5.0;
pub(super) const HANDLE_HIT_RADIUS: f32 = 20.0;
pub(super) const POINT_RADIUS: f32 = 5.0;
pub(super) const POINT_HIT_RADIUS: f32 = 20.0;

pub struct BoundingBoxAnnotator<'a> {
    texture_handle: &'a TextureHandle,
    image_size: [f32; 2],
    selected_class: &'a mut Class,
    creation_shape: CreationShape,
    annotations: &'a mut Vec<Annotation>,
    state: &'a mut CanvasState,
    input_enabled: bool,
}

impl<'a> BoundingBoxAnnotator<'a> {
    pub fn new(
        image: &'a TextureHandle,
        image_size: [f32; 2],
        annotations: &'a mut Vec<Annotation>,
        state: &'a mut CanvasState,
        selected_class: &'a mut Class,
        creation_shape: CreationShape,
        input_enabled: bool,
    ) -> Self {
        Self {
            texture_handle: image,
            image_size,
            annotations,
            state,
            selected_class,
            creation_shape,
            input_enabled,
        }
    }

    fn handle_input(&mut self, ui: &Ui, response: &Response, transform: ImageTransform) {
        self.handle_keyboard_input(ui, response, transform);
        self.handle_pointer_input(ui, response, transform);

        if let Some(selection) = self.state.selected
            && selection.index >= self.annotations.len()
        {
            self.state.selected = None;
            self.state.clear_mode();
        }
    }
}

impl Widget for BoundingBoxAnnotator<'_> {
    fn ui(mut self, ui: &mut Ui) -> Response {
        let available_size = ui.available_size_before_wrap();
        let desired_height = available_size.y.max(360.0);
        let desired_size = Vec2::new(available_size.x.max(360.0), desired_height);
        let (rect, mut response) = ui.allocate_exact_size(desired_size, Sense::click_and_drag());
        response.widget_info(|| eframe::egui::WidgetInfo::new(eframe::egui::WidgetType::Other));

        if response.hovered() {
            response = response.on_hover_cursor(CursorIcon::Crosshair);
        }

        if self.input_enabled {
            self.handle_view_input(ui, &response);
        }
        let base_transform =
            ImageTransform::new(rect, self.image_size, self.state.zoom, self.state.pan);
        let transform = if self.input_enabled {
            self.effective_transform(ui, &response, base_transform)
        } else {
            base_transform
        };
        if self.input_enabled {
            self.handle_input(ui, &response, transform);
        }
        self.draw(ui, &response, transform);

        response
    }
}
