use eframe::egui::{PointerButton, Pos2, Response, Ui};

use crate::annotation::Annotation;

use super::{
    AnnotationShape, BoundingBoxAnnotator, CreationShape, HANDLE_HIT_RADIUS, POINT_HIT_RADIUS,
    Selection,
    state::CanvasMode,
    transform::{ImageTransform, clamp_point},
};

impl BoundingBoxAnnotator<'_> {
    pub(super) fn create_point_at_pointer_or_center(
        &mut self,
        response: &Response,
        transform: ImageTransform,
    ) {
        let point = response
            .hover_pos()
            .map(|position| transform.screen_to_image_clamped(position, self.image_size))
            .unwrap_or_else(|| Pos2::new(self.image_size[0] * 0.5, self.image_size[1] * 0.5));
        self.add_point(point);
    }

    pub(super) fn add_point(&mut self, point: Pos2) {
        if !self.selected_class.supports_points() {
            return;
        }

        self.annotations.push(Annotation::point(
            *self.selected_class,
            clamp_point(point, self.image_size),
        ));
        self.state.selected = Some(Selection {
            index: self.annotations.len() - 1,
            shape: AnnotationShape::Point,
        });
        self.state.mode = CanvasMode::KeyboardMoveSelected;
        self.state.mark_annotations_changed();
    }

    pub(super) fn handle_pointer_input(
        &mut self,
        ui: &Ui,
        response: &Response,
        transform: ImageTransform,
    ) {
        let pointer_position = ui.input(|input| input.pointer.interact_pos());

        if self.update_keyboard_draft_from_pointer(response, transform) {
            return;
        }

        if response.clicked_by(PointerButton::Secondary) {
            self.delete_at(pointer_position, transform);
            return;
        }

        if response.drag_started_by(PointerButton::Primary)
            && let Some(position) = pointer_position
        {
            self.start_pointer_interaction(position, transform);
        }

        if ui.input(|input| input.pointer.button_down(PointerButton::Primary)) {
            if let Some(position) = pointer_position {
                self.update_pointer_interaction(position, transform);
            }
        } else {
            self.finish_pointer_interaction();
        }

        if response.clicked_by(PointerButton::Primary)
            && let Some(position) = pointer_position
        {
            self.click_pointer(position, transform);
        }
    }

    fn update_keyboard_draft_from_pointer(
        &mut self,
        response: &Response,
        transform: ImageTransform,
    ) -> bool {
        let CanvasMode::KeyboardDraftBox {
            anchor,
            pointer_position,
            ..
        } = self.state.mode
        else {
            return false;
        };

        if let Some(position) = response
            .hover_pos()
            .or_else(|| response.interact_pointer_pos())
            && pointer_position != Some(position)
        {
            let moving = transform.screen_to_image_clamped(position, self.image_size);
            self.state.mode = CanvasMode::KeyboardDraftBox {
                anchor,
                moving,
                pointer_position: Some(position),
            };
        }

        if response.clicked_by(PointerButton::Primary) {
            self.commit_draft_box();
        }

        true
    }

    fn click_pointer(&mut self, position: Pos2, transform: ImageTransform) {
        if let Some(selection) = self.hit_test_point(position, transform) {
            self.select(selection);
            return;
        }

        let Some(image_position) = transform.screen_to_image(position) else {
            return;
        };

        if let Some(selection) = self.hit_test_box(image_position) {
            self.select(selection);
            return;
        }

        if self.creation_shape == CreationShape::Point {
            self.add_point(image_position);
        }
    }

    fn start_pointer_interaction(&mut self, position: Pos2, transform: ImageTransform) {
        if let Some(selection) = self.hit_test_point(position, transform) {
            self.select(selection);
            self.state.mode = CanvasMode::DraggingPoint {
                index: selection.index,
            };
            return;
        }

        if let Some((index, corner)) = self.hit_test_handle(position, transform) {
            self.select(Selection {
                index,
                shape: AnnotationShape::Box,
            });
            self.state.mode = CanvasMode::ResizingBoxWithPointer { index, corner };
            return;
        }

        let Some(image_position) = transform.screen_to_image(position) else {
            return;
        };

        if let Some(selection) = self.hit_test_box(image_position) {
            self.select(selection);
            self.state.mode = CanvasMode::DraggingBox {
                index: selection.index,
                last_position: image_position,
            };
            return;
        }

        if self.creation_shape == CreationShape::Box && self.selected_class.supports_boxes() {
            self.state.selected = None;
            self.state.mode = CanvasMode::DrawingBox {
                start: image_position,
                moving: image_position,
            };
        }
    }

    fn update_pointer_interaction(&mut self, position: Pos2, transform: ImageTransform) {
        let image_position = transform.screen_to_image_clamped(position, self.image_size);
        match self.state.mode {
            CanvasMode::DrawingBox { start, .. } => {
                self.state.mode = CanvasMode::DrawingBox {
                    start,
                    moving: image_position,
                };
            }
            CanvasMode::DraggingBox {
                index,
                last_position,
            } => {
                if let Some(bounding_box) = self
                    .annotations
                    .get_mut(index)
                    .and_then(|annotation| annotation.bounding_box_mut())
                {
                    bounding_box.translate(image_position - last_position, self.image_size);
                    self.state.mode = CanvasMode::DraggingBox {
                        index,
                        last_position: image_position,
                    };
                    self.state.mark_annotations_changed();
                }
            }
            CanvasMode::ResizingBoxWithPointer { index, corner } => {
                if let Some(bounding_box) = self
                    .annotations
                    .get_mut(index)
                    .and_then(|annotation| annotation.bounding_box_mut())
                {
                    bounding_box.set_corner(corner, image_position);
                    bounding_box.clip_to_image(self.image_size);
                    self.state.mark_annotations_changed();
                }
            }
            CanvasMode::DraggingPoint { index } => {
                if let Some(point) = self
                    .annotations
                    .get_mut(index)
                    .and_then(|annotation| annotation.point_mut())
                {
                    *point = image_position;
                    self.state.mark_annotations_changed();
                }
            }
            CanvasMode::Idle
            | CanvasMode::KeyboardDraftBox { .. }
            | CanvasMode::KeyboardMoveSelected
            | CanvasMode::KeyboardResizeSelected { .. } => {}
        }
    }

    fn finish_pointer_interaction(&mut self) {
        if matches!(self.state.mode, CanvasMode::DrawingBox { .. }) {
            self.commit_draft_box();
        } else if self.state.mode.is_pointer_interaction() {
            self.state.clear_mode();
        }
    }

    fn delete_at(&mut self, pointer_position: Option<Pos2>, transform: ImageTransform) {
        if let Some(position) = pointer_position {
            if let Some(selection) = self.hit_test_point(position, transform) {
                self.delete_selection(selection);
                return;
            }

            if let Some(image_position) = transform.screen_to_image(position)
                && let Some(selection) = self.hit_test_box(image_position)
            {
                self.delete_selection(selection);
                return;
            }
        }

        self.delete_selected();
    }

    pub(super) fn delete_selected(&mut self) {
        if let Some(selection) = self.state.selected.take() {
            self.delete_selection(selection);
        }
        self.state.clear_mode();
    }

    fn delete_selection(&mut self, selection: Selection) {
        let Some(annotation) = self.annotations.get_mut(selection.index) else {
            return;
        };
        let remove_annotation = match selection.shape {
            AnnotationShape::Box => annotation.clear_bounding_box(),
            AnnotationShape::Point => annotation.clear_point(),
        };
        if remove_annotation {
            self.annotations.remove(selection.index);
        }
        self.state.selected = None;
        self.state.clear_mode();
        self.state.mark_annotations_changed();
    }

    pub(super) fn commit_draft_box(&mut self) {
        let Some(mut draft_box) = self.state.take_draft_box() else {
            return;
        };
        if !self.selected_class.supports_boxes() {
            self.add_point(draft_box.rect.center());
            return;
        }
        draft_box.clip_to_image(self.image_size);
        if draft_box.is_valid() {
            self.annotations
                .push(Annotation::bounding_box(*self.selected_class, draft_box));
            self.state.selected = Some(Selection {
                index: self.annotations.len() - 1,
                shape: AnnotationShape::Box,
            });
            self.state.mark_annotations_changed();
        }
        self.state.clear_mode();
    }

    fn select(&mut self, selection: Selection) {
        self.state.selected = Some(selection);
        self.state.clear_mode();
    }

    fn hit_test_point(
        &self,
        screen_position: Pos2,
        transform: ImageTransform,
    ) -> Option<Selection> {
        self.annotations
            .iter()
            .enumerate()
            .rev()
            .filter(|(_, annotation)| annotation.class == *self.selected_class)
            .find_map(|(index, annotation)| {
                annotation.point_position().and_then(|point| {
                    (transform.image_to_screen(point).distance(screen_position) <= POINT_HIT_RADIUS)
                        .then_some(Selection {
                            index,
                            shape: AnnotationShape::Point,
                        })
                })
            })
    }

    fn hit_test_handle(
        &self,
        screen_position: Pos2,
        transform: ImageTransform,
    ) -> Option<(usize, crate::boundingbox::Corner)> {
        self.annotations
            .iter()
            .enumerate()
            .rev()
            .filter(|(_, annotation)| {
                annotation.class == *self.selected_class && annotation.class.supports_boxes()
            })
            .find_map(|(index, annotation)| {
                annotation.bounding_box_ref().and_then(|bounding_box| {
                    crate::boundingbox::Corner::ALL
                        .into_iter()
                        .find_map(|corner| {
                            let corner_screen =
                                transform.image_to_screen(bounding_box.corner(corner));
                            (corner_screen.distance(screen_position) <= HANDLE_HIT_RADIUS)
                                .then_some((index, corner))
                        })
                })
            })
    }

    fn hit_test_box(&self, image_position: Pos2) -> Option<Selection> {
        self.annotations
            .iter()
            .enumerate()
            .filter(|(_, annotation)| {
                annotation.class == *self.selected_class && annotation.class.supports_boxes()
            })
            .filter_map(|(index, annotation)| {
                annotation
                    .bounding_box_ref()
                    .filter(|bounding_box| bounding_box.contains(image_position))
                    .map(|bounding_box| (index, bounding_box))
            })
            .min_by(|(_, left), (_, right)| left.rect.area().total_cmp(&right.rect.area()))
            .map(|(index, _)| Selection {
                index,
                shape: AnnotationShape::Box,
            })
    }

    pub(super) fn selectable_shapes(&self) -> Vec<Selection> {
        selectable_shapes(self.annotations, *self.selected_class)
    }
}

fn selectable_shapes(annotations: &[Annotation], class: crate::classes::Class) -> Vec<Selection> {
    annotations
        .iter()
        .enumerate()
        .filter(|(_, annotation)| annotation.class == class)
        .flat_map(|(index, annotation)| {
            let point = annotation.point_position().map(|_| Selection {
                index,
                shape: AnnotationShape::Point,
            });
            let bounding_box = (annotation.bounding_box_ref().is_some()
                && annotation.class.supports_boxes())
            .then_some(Selection {
                index,
                shape: AnnotationShape::Box,
            });
            [point, bounding_box].into_iter().flatten()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use eframe::egui::Pos2;

    use super::*;
    use crate::{boundingbox::BoundingBox, classes::Class};

    #[test]
    fn affected_classes_do_not_offer_bbox_selection() {
        let annotations = vec![Annotation::bounding_box(
            Class::LSpot,
            BoundingBox::new(Pos2::ZERO, Pos2::new(10.0, 10.0)),
        )];

        assert!(selectable_shapes(&annotations, Class::LSpot).is_empty());
    }
}
