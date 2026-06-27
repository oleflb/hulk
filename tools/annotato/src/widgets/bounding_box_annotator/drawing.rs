use std::borrow::Cow;

use eframe::egui::{Align2, Color32, FontId, Pos2, Rect, Response, Stroke, StrokeKind, Ui, Vec2};

use crate::{annotation::Annotation, boundingbox::BoundingBox, classes::Class};

use super::{
    AnnotationShape, BoundingBoxAnnotator, HANDLE_RADIUS, POINT_RADIUS, Selection,
    transform::ImageTransform,
};

impl BoundingBoxAnnotator<'_> {
    pub(super) fn draw(&self, ui: &Ui, response: &Response, transform: ImageTransform) {
        let painter = ui.painter_at(response.rect);
        painter.rect_filled(response.rect, 8.0, Color32::from_rgb(17, 17, 27));
        painter.rect_filled(transform.image_rect, 4.0, Color32::from_rgb(24, 24, 37));
        painter.image(
            self.texture_handle.id(),
            transform.image_rect,
            Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
            Color32::WHITE,
        );

        for (index, annotation) in self.annotations.iter().enumerate() {
            if let Some(bounding_box) = annotation.bounding_box_ref() {
                self.draw_box(
                    ui,
                    transform,
                    annotation.class,
                    bounding_box,
                    BoxDrawOptions {
                        active: annotation.class == *self.selected_class,
                        selected: self.state.selected
                            == Some(Selection {
                                index,
                                shape: AnnotationShape::Box,
                            }),
                        draft: false,
                    },
                );
            }
        }

        if let Some(draft_box) = self.state.draft_box() {
            self.draw_box(
                ui,
                transform,
                *self.selected_class,
                &draft_box,
                BoxDrawOptions {
                    active: true,
                    selected: true,
                    draft: true,
                },
            );
        }

        for (index, annotation) in self.annotations.iter().enumerate() {
            if let Some(point) = annotation.point_position() {
                self.draw_point(
                    ui,
                    transform,
                    annotation,
                    point,
                    annotation.class == *self.selected_class,
                    self.state.selected
                        == Some(Selection {
                            index,
                            shape: AnnotationShape::Point,
                        }),
                );
            }
        }
    }

    fn draw_box(
        &self,
        ui: &Ui,
        transform: ImageTransform,
        class: Class,
        bounding_box: &BoundingBox,
        options: BoxDrawOptions,
    ) {
        if !bounding_box.is_valid() && !options.draft {
            return;
        }

        let painter = ui.painter();
        let rect = transform.image_rect_to_screen(bounding_box.rect);
        let editable = options.active && (class.supports_boxes() || options.draft);
        let color = if options.active || options.draft {
            class.color()
        } else {
            class.color().gamma_multiply(0.35)
        };
        let stroke_width = if options.selected { 2.5 } else { 1.5 };
        painter.rect_filled(
            rect,
            2.0,
            color.gamma_multiply(if options.selected {
                0.18
            } else if options.active {
                0.10
            } else {
                0.05
            }),
        );
        painter.rect_stroke(
            rect,
            2.0,
            Stroke::new(stroke_width, color),
            StrokeKind::Inside,
        );

        let label: Cow<'_, str> = if options.draft {
            Cow::Owned(format!("new {}", class.as_str()))
        } else if editable || !options.active {
            Cow::Borrowed(class.as_str())
        } else {
            Cow::Owned(format!("legacy {} box", class.as_str()))
        };
        painter.text(
            rect.left_top() + Vec2::new(4.0, 4.0),
            Align2::LEFT_TOP,
            label.as_ref(),
            FontId::proportional(18.0),
            if options.active {
                Color32::from_rgb(205, 214, 244)
            } else {
                Color32::from_rgb(127, 132, 156)
            },
        );

        if editable && options.active && (options.selected || options.draft) {
            for corner in crate::boundingbox::Corner::ALL {
                let corner_screen = transform.image_to_screen(bounding_box.corner(corner));
                painter.circle_filled(corner_screen, HANDLE_RADIUS, color);
                painter.circle_stroke(
                    corner_screen,
                    HANDLE_RADIUS + 1.5,
                    Stroke::new(1.0, Color32::from_rgb(205, 214, 244)),
                );
            }
        }
    }

    fn draw_point(
        &self,
        ui: &Ui,
        transform: ImageTransform,
        annotation: &Annotation,
        point: Pos2,
        active: bool,
        selected: bool,
    ) {
        let painter = ui.painter();
        let position = transform.image_to_screen(point);
        let color = if active {
            annotation.class.color()
        } else {
            annotation.class.color().gamma_multiply(0.35)
        };
        let radius = if selected {
            POINT_RADIUS + 2.0
        } else {
            POINT_RADIUS
        };

        painter.circle_filled(position, radius, color);
        painter.circle_stroke(
            position,
            radius + 2.0,
            Stroke::new(1.5, Color32::from_rgb(205, 214, 244)),
        );
        painter.line_segment(
            [
                position - Vec2::new(9.0, 0.0),
                position + Vec2::new(9.0, 0.0),
            ],
            Stroke::new(1.5, color),
        );
        painter.line_segment(
            [
                position - Vec2::new(0.0, 9.0),
                position + Vec2::new(0.0, 9.0),
            ],
            Stroke::new(1.5, color),
        );
        painter.text(
            position + Vec2::new(8.0, -8.0),
            Align2::LEFT_BOTTOM,
            annotation.class.as_str(),
            FontId::proportional(18.0),
            if active {
                Color32::from_rgb(205, 214, 244)
            } else {
                Color32::from_rgb(127, 132, 156)
            },
        );
    }
}

#[derive(Debug, Clone, Copy)]
struct BoxDrawOptions {
    active: bool,
    selected: bool,
    draft: bool,
}
