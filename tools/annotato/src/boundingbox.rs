use crate::{annotation::AnnotationFormat, classes::Class};
use eframe::egui::{Pos2, Rect, Vec2};

#[derive(Debug, Clone, PartialEq)]
pub struct BoundingBox {
    pub rect: Rect,
    pub class: Class,
}

impl BoundingBox {
    pub fn new(corner: Pos2, opposing_corner: Pos2, class: Class) -> Self {
        BoundingBox {
            rect: Rect::from_two_pos(corner, opposing_corner),
            class,
        }
    }

    pub fn from_rect(rect: Rect, class: Class) -> Self {
        BoundingBox { rect, class }
    }

    pub fn set_corner(&mut self, corner: Corner, position: Pos2) {
        let mut minimum = self.rect.min;
        let mut maximum = self.rect.max;
        match corner {
            Corner::TopLeft => minimum = position,
            Corner::TopRight => {
                maximum.x = position.x;
                minimum.y = position.y;
            }
            Corner::BottomRight => maximum = position,
            Corner::BottomLeft => {
                minimum.x = position.x;
                maximum.y = position.y;
            }
        }
        self.rect = Rect::from_two_pos(minimum, maximum);
    }

    pub fn corner(&self, corner: Corner) -> Pos2 {
        match corner {
            Corner::TopLeft => self.rect.left_top(),
            Corner::TopRight => self.rect.right_top(),
            Corner::BottomRight => self.rect.right_bottom(),
            Corner::BottomLeft => self.rect.left_bottom(),
        }
    }

    pub fn contains(&self, image_position: Pos2) -> bool {
        self.rect.contains(image_position)
    }

    pub fn clip_to_image(&mut self, image_size: [f32; 2]) {
        let clamp = |position: Pos2| {
            Pos2::new(
                position.x.clamp(0.0, image_size[0]),
                position.y.clamp(0.0, image_size[1]),
            )
        };
        self.rect = Rect::from_two_pos(clamp(self.rect.min), clamp(self.rect.max));
    }

    pub fn translate(&mut self, delta: Vec2, image_size: [f32; 2]) {
        self.rect = self.rect.translate(delta);

        let image_rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(image_size[0], image_size[1]));
        let mut correction = Vec2::ZERO;
        if self.rect.left() < image_rect.left() {
            correction.x = image_rect.left() - self.rect.left();
        }
        if self.rect.right() > image_rect.right() {
            correction.x = image_rect.right() - self.rect.right();
        }
        if self.rect.top() < image_rect.top() {
            correction.y = image_rect.top() - self.rect.top();
        }
        if self.rect.bottom() > image_rect.bottom() {
            correction.y = image_rect.bottom() - self.rect.bottom();
        }
        self.rect = self.rect.translate(correction);
    }

    pub fn is_valid(&self) -> bool {
        self.rect.area() >= 4.0
    }

    pub fn iou(&self, other: &BoundingBox) -> f32 {
        let intersection = self.rect.intersect(other.rect).area();
        let union = self.rect.area() + other.rect.area() - intersection;

        intersection / union
    }

    pub fn closest_corner(&self, position: Pos2) -> Corner {
        Corner::ALL
            .into_iter()
            .min_by(|left, right| {
                self.corner(*left)
                    .distance_sq(position)
                    .total_cmp(&self.corner(*right).distance_sq(position))
            })
            .unwrap_or(Corner::BottomRight)
    }

    pub fn to_annotation(&self, image_size: [f32; 2]) -> AnnotationFormat {
        let width = image_size[0];
        let height = image_size[1];

        AnnotationFormat {
            points: [
                [
                    clamp_normalized(self.rect.left() / width),
                    clamp_normalized(self.rect.top() / height),
                ],
                [
                    clamp_normalized(self.rect.right() / width),
                    clamp_normalized(self.rect.bottom() / height),
                ],
            ],
            class: self.class,
        }
    }

    pub fn from_annotation(annotation: AnnotationFormat, image_size: [f32; 2]) -> Self {
        let [[minimum_x, minimum_y], [maximum_x, maximum_y]] = annotation.points;
        let width = image_size[0];
        let height = image_size[1];

        let corner = Pos2::new(minimum_x * width, minimum_y * height);
        let opposing_corner = Pos2::new(maximum_x * width, maximum_y * height);

        Self::new(corner, opposing_corner, annotation.class)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Corner {
    TopLeft,
    TopRight,
    BottomRight,
    BottomLeft,
}

impl Corner {
    pub const ALL: [Corner; 4] = [
        Corner::TopLeft,
        Corner::TopRight,
        Corner::BottomRight,
        Corner::BottomLeft,
    ];

    pub fn next(self) -> Self {
        match self {
            Corner::TopLeft => Corner::TopRight,
            Corner::TopRight => Corner::BottomRight,
            Corner::BottomRight => Corner::BottomLeft,
            Corner::BottomLeft => Corner::TopLeft,
        }
    }
}

fn clamp_normalized(value: f32) -> f32 {
    value.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn annotations_roundtrip_through_image_size() {
        let annotation = AnnotationFormat {
            points: [[0.25, 0.2], [0.75, 0.8]],
            class: Class::Ball,
        };

        let bounding_box = BoundingBox::from_annotation(annotation.clone(), [800.0, 400.0]);
        let serialized = bounding_box.to_annotation([800.0, 400.0]);

        assert_eq!(serialized.class, annotation.class);
        assert_eq!(serialized.points, annotation.points);
    }

    #[test]
    fn annotations_are_normalized_for_different_image_sizes() {
        let bounding_box = BoundingBox::new(
            Pos2::new(100.0, 50.0),
            Pos2::new(300.0, 250.0),
            Class::Robot,
        );

        assert_eq!(
            bounding_box.to_annotation([400.0, 500.0]).points,
            [[0.25, 0.1], [0.75, 0.5]]
        );
    }
}
