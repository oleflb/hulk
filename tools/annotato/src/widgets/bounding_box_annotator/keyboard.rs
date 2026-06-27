use eframe::egui::{Key, Pos2, Response, Ui, Vec2};

use crate::user_toml::CONFIG;

use super::{
    AnnotationShape, BoundingBoxAnnotator, CreationShape, Selection, state::CanvasMode,
    transform::ImageTransform,
};

impl BoundingBoxAnnotator<'_> {
    pub(super) fn handle_keyboard_input(
        &mut self,
        ui: &Ui,
        response: &Response,
        transform: ImageTransform,
    ) {
        let config = &CONFIG.get().unwrap().keybindings;

        if ui.input(|input| config.abort.is_pressed(input)) {
            if self.state.take_draft_box().is_some() || !self.state.mode.is_idle() {
                self.state.clear_mode();
            } else {
                self.state.selected = None;
            }
            return;
        }

        if ui.input(|input| config.delete.is_pressed(input) || input.key_pressed(Key::Backspace)) {
            self.delete_selected();
            return;
        }

        if ui.input(|input| config.draw.is_pressed(input)) {
            if self.state.draft_box().is_some() {
                self.commit_draft_box();
            } else if self.creation_shape == CreationShape::Point {
                self.create_point_at_pointer_or_center(response, transform);
            } else {
                self.start_keyboard_draft(response, transform);
            }
        }

        if ui.input(|input| config.confirm.is_pressed(input)) {
            if self.state.draft_box().is_some() {
                self.commit_draft_box();
            } else {
                self.state.clear_mode();
            }
        }

        if ui.input(|input| config.edit.is_pressed(input))
            && let Some(selection) = self.active_selection()
            && selection.shape == AnnotationShape::Box
            && let Some(annotation) = self.annotations.get(selection.index)
            && annotation.class.supports_boxes()
            && let Some(bounding_box) = annotation.bounding_box_ref()
        {
            let pointer = response
                .hover_pos()
                .map(|position| transform.screen_to_image_clamped(position, self.image_size))
                .unwrap_or_else(|| bounding_box.rect.center());
            self.state.mode = CanvasMode::KeyboardResizeSelected {
                corner: bounding_box.closest_corner(pointer),
            };
        }

        if ui.input(|input| config.move_box.is_pressed(input)) && self.active_selection().is_some()
        {
            self.state.mode = CanvasMode::KeyboardMoveSelected;
        }

        self.handle_tab_selection(ui);

        if let Some(delta) = keyboard_delta(ui) {
            self.apply_keyboard_delta(delta);
        }
    }

    fn active_selection(&self) -> Option<Selection> {
        let selection = self.state.selected?;
        let annotation = self.annotations.get(selection.index)?;
        (annotation.class == *self.selected_class).then_some(selection)
    }

    fn handle_tab_selection(&mut self, ui: &Ui) {
        let tab_direction = ui.input(|input| {
            if input.key_pressed(Key::Tab) {
                Some(if input.modifiers.shift { -1 } else { 1 })
            } else {
                None
            }
        });

        let Some(direction) = tab_direction else {
            return;
        };
        let selectable = self.selectable_shapes();
        if selectable.is_empty() {
            self.state.selected = None;
            return;
        }

        let current = self
            .state
            .selected
            .and_then(|selection| {
                selectable
                    .iter()
                    .position(|candidate| *candidate == selection)
            })
            .unwrap_or(0);
        let len = selectable.len();
        self.state.selected = Some(if direction > 0 {
            selectable[(current + 1) % len]
        } else {
            selectable[(current + len - 1) % len]
        });
        self.state.clear_mode();
    }

    fn apply_keyboard_delta(&mut self, delta: Vec2) {
        match self.state.mode {
            CanvasMode::KeyboardDraftBox {
                anchor,
                moving,
                pointer_position,
            } => {
                let moving = super::transform::clamp_point(moving + delta, self.image_size);
                self.state.mode = CanvasMode::KeyboardDraftBox {
                    anchor,
                    moving,
                    pointer_position,
                };
            }
            CanvasMode::KeyboardResizeSelected { corner } => {
                if let Some(selection) = self.active_selection()
                    && selection.shape == AnnotationShape::Box
                    && let Some(bounding_box) = self
                        .annotations
                        .get_mut(selection.index)
                        .and_then(|annotation| annotation.bounding_box_mut())
                {
                    let position = bounding_box.corner(corner) + delta;
                    bounding_box.set_corner(corner, position);
                    bounding_box.clip_to_image(self.image_size);
                    self.state.mark_annotations_changed();
                } else {
                    self.state.clear_mode();
                }
            }
            CanvasMode::KeyboardMoveSelected => {
                if let Some(selection) = self.active_selection() {
                    match selection.shape {
                        AnnotationShape::Box => {
                            if let Some(bounding_box) = self
                                .annotations
                                .get_mut(selection.index)
                                .and_then(|annotation| annotation.bounding_box_mut())
                            {
                                bounding_box.translate(delta, self.image_size);
                                self.state.mark_annotations_changed();
                            }
                        }
                        AnnotationShape::Point => {
                            if let Some(point) = self
                                .annotations
                                .get_mut(selection.index)
                                .and_then(|annotation| annotation.point_mut())
                            {
                                *point =
                                    super::transform::clamp_point(*point + delta, self.image_size);
                                self.state.mark_annotations_changed();
                            }
                        }
                    }
                } else {
                    self.state.clear_mode();
                }
            }
            CanvasMode::Idle
            | CanvasMode::DrawingBox { .. }
            | CanvasMode::DraggingBox { .. }
            | CanvasMode::ResizingBoxWithPointer { .. }
            | CanvasMode::DraggingPoint { .. } => {}
        }
    }

    fn start_keyboard_draft(&mut self, response: &Response, transform: ImageTransform) {
        if !self.selected_class.supports_boxes() {
            self.create_point_at_pointer_or_center(response, transform);
            return;
        }

        let pointer_position = response.hover_pos();
        let anchor = pointer_position
            .map(|position| transform.screen_to_image_clamped(position, self.image_size))
            .unwrap_or_else(|| Pos2::new(self.image_size[0] * 0.5, self.image_size[1] * 0.5));
        self.state.selected = None;
        self.state.mode = CanvasMode::KeyboardDraftBox {
            anchor,
            moving: anchor,
            pointer_position,
        };
    }
}

fn keyboard_delta(ui: &Ui) -> Option<Vec2> {
    ui.input(|input| {
        let step = if input.modifiers.ctrl {
            1.0
        } else if input.modifiers.shift {
            20.0
        } else {
            5.0
        };
        let mut delta = Vec2::ZERO;
        if input.key_pressed(Key::ArrowLeft) {
            delta.x -= step;
        }
        if input.key_pressed(Key::ArrowRight) {
            delta.x += step;
        }
        if input.key_pressed(Key::ArrowUp) {
            delta.y -= step;
        }
        if input.key_pressed(Key::ArrowDown) {
            delta.y += step;
        }

        (delta != Vec2::ZERO).then_some(delta)
    })
}
