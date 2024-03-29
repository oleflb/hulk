use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use linear_algebra::{point, Point2, Vector2};

#[derive(
    Clone,
    Copy,
    Debug,
    Deserialize,
    PartialEq,
    PathDeserialize,
    PathIntrospect,
    PathSerialize,
    Serialize,
)]
pub struct Rectangle<Frame> {
    pub min: Point2<Frame>,
    pub max: Point2<Frame>,
}

impl<Frame> Rectangle<Frame> {
    pub fn from_cxcywh(center_x: f32, center_y: f32, width: f32, height: f32) -> Self {
        Self {
            min: point![center_x - width / 2.0, center_y - height / 2.0],
            max: point![center_x + width / 2.0, center_y + height / 2.0],
        }
    }

    pub fn new_with_center_and_size(center: Point2<Frame>, size: Vector2<Frame>) -> Self {
        Self {
            min: center - size / 2.0,
            max: center + size / 2.0,
        }
    }
    pub fn rectangle_intersection(self, other: Rectangle<Frame>) -> f32 {
        let intersection_x = f32::max(
            0.0,
            f32::min(self.max.x(), other.max.x()) - f32::max(self.min.x(), other.min.x()),
        );
        let intersection_y = f32::max(
            0.0,
            f32::min(self.max.y(), other.max.y()) - f32::max(self.min.y(), other.min.y()),
        );
        intersection_x * intersection_y
    }

    pub fn area(self) -> f32 {
        let dimensions = self.dimensions();
        dimensions.x() * dimensions.y()
    }

    pub fn dimensions(&self) -> Vector2<Frame> {
        self.max.coords() - self.min.coords()
    }
}
