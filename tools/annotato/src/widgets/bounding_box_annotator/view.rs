use eframe::egui::{Event, PointerButton, Response, Ui, Vec2};

use crate::user_toml::CONFIG;

use super::{BoundingBoxAnnotator, transform::ImageTransform};

impl BoundingBoxAnnotator<'_> {
    pub(super) fn handle_view_input(&mut self, ui: &Ui, response: &Response) {
        if !response.hovered() {
            return;
        }

        let (scroll_delta, ctrl_pressed, middle_drag_delta, pointer_position) = ui.input(|input| {
            let scroll_delta = input
                .events
                .iter()
                .filter_map(|event| match event {
                    Event::MouseWheel { delta, .. } => Some(*delta),
                    _ => None,
                })
                .fold(Vec2::ZERO, |sum, delta| sum + delta);
            (
                (scroll_delta != Vec2::ZERO).then_some(scroll_delta),
                input.modifiers.ctrl,
                if input.pointer.button_down(PointerButton::Middle) {
                    input.pointer.delta()
                } else {
                    Vec2::ZERO
                },
                input.pointer.hover_pos(),
            )
        });

        if middle_drag_delta != Vec2::ZERO && self.state.zoom > 1.01 {
            self.state.pan += middle_drag_delta;
        }

        let Some(scroll_delta) = scroll_delta else {
            return;
        };

        if ctrl_pressed {
            let zoom_factor = (scroll_delta.y / 240.0).exp();
            let old_zoom = self.state.zoom.max(1.0);
            let new_zoom = (old_zoom * zoom_factor).clamp(1.0, 16.0);
            if let Some(pointer) = pointer_position {
                let old_transform =
                    ImageTransform::new(response.rect, self.image_size, old_zoom, self.state.pan);
                let image_anchor = old_transform.screen_to_image_unclamped(pointer);
                self.state.pan = ImageTransform::pan_for_anchor(
                    response.rect,
                    self.image_size,
                    new_zoom,
                    pointer,
                    image_anchor,
                );
            }
            self.state.zoom = new_zoom;
            if self.state.zoom <= 1.001 {
                self.state.pan = Vec2::ZERO;
            }
            return;
        }

        if self.state.zoom > 1.01 {
            self.state.pan += scroll_delta;
        }
    }

    pub(super) fn effective_transform(
        &self,
        ui: &Ui,
        response: &Response,
        base_transform: ImageTransform,
    ) -> ImageTransform {
        let config = &CONFIG.get().unwrap().keybindings;
        let focus = ui.input(|input| config.temporary_focus.is_down(input));
        if !focus || !response.hovered() {
            return base_transform;
        }

        let Some(pointer) = ui.input(|input| input.pointer.hover_pos()) else {
            return base_transform;
        };

        let image_anchor = base_transform.screen_to_image_clamped(pointer, self.image_size);
        let focus_zoom = (self.state.zoom * 10.0).clamp(1.0, 160.0);
        let focus_pan = ImageTransform::pan_for_anchor(
            response.rect,
            self.image_size,
            focus_zoom,
            pointer,
            image_anchor,
        );
        ImageTransform::new(response.rect, self.image_size, focus_zoom, focus_pan)
    }
}
