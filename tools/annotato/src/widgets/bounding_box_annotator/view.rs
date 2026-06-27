use eframe::egui::{PointerButton, Response, Ui, Vec2};

use crate::user_toml::CONFIG;

use super::{BoundingBoxAnnotator, state::FocusAnchor, transform::ImageTransform};

impl BoundingBoxAnnotator<'_> {
    pub(super) fn handle_view_input(&mut self, ui: &Ui, response: &Response) {
        if !response.hovered() {
            return;
        }

        let (scroll_y, middle_drag_delta, pointer_position) = ui.input(|input| {
            (
                input.smooth_scroll_delta.y,
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

        if scroll_y.abs() <= f32::EPSILON {
            return;
        }

        let old_zoom = self.state.zoom.max(1.0);
        let zoom_factor = 1.01_f32.powf(scroll_y);
        let new_zoom = (old_zoom * zoom_factor).clamp(1.0, 16.0);
        if (new_zoom - old_zoom).abs() <= f32::EPSILON {
            return;
        }

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
    }

    pub(super) fn effective_transform(
        &mut self,
        ui: &Ui,
        response: &Response,
        base_transform: ImageTransform,
    ) -> ImageTransform {
        let config = &CONFIG.get().unwrap().keybindings;
        let focus = ui.input(|input| config.temporary_focus.is_down(input));
        if !focus {
            self.state.focus_anchor = None;
            return base_transform;
        }

        if self.state.focus_anchor.is_none() {
            if !response.hovered() {
                return base_transform;
            }

            let Some(pointer) = ui.input(|input| input.pointer.hover_pos()) else {
                return base_transform;
            };

            self.state.focus_anchor = Some(FocusAnchor {
                screen_position: pointer,
                image_position: base_transform.screen_to_image_clamped(pointer, self.image_size),
            });
        }

        let Some(anchor) = self.state.focus_anchor else {
            return base_transform;
        };
        let focus_zoom = (self.state.zoom * 10.0).clamp(1.0, 160.0);
        let focus_pan = ImageTransform::pan_for_anchor(
            response.rect,
            self.image_size,
            focus_zoom,
            anchor.screen_position,
            anchor.image_position,
        );
        ImageTransform::new(response.rect, self.image_size, focus_zoom, focus_pan)
    }
}
