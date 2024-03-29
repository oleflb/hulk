use geometry::rectangle::Rectangle;
use serde::{Deserialize, Serialize};
use linear_algebra::{Point2, vector};
use serialize_hierarchy::SerializeHierarchy;

use coordinate_systems::Pixel;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Serialize, Deserialize, SerializeHierarchy)]
pub enum DetectedObject {
    Robot,
}

impl DetectedObject {
    pub fn from_u8(index: u8) -> Option<DetectedObject> {
        match index {
            0 => Some(DetectedObject::Robot),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, SerializeHierarchy)]
pub struct BoundingBox {
    pub bounding_box: Rectangle<Pixel>,
    pub class: DetectedObject,
    pub score: f32,
}

impl BoundingBox {
    pub const fn new(class: DetectedObject, score: f32, bounding_box: Rectangle<Pixel>) -> Self {
        Self {
            bounding_box,
            class,
            score,
        }
    }

    pub fn bottom_center(&self) -> Point2<Pixel> {
        self.bounding_box.max - vector![self.bounding_box.dimensions().x() / 2.0, 0.0]
    }

    pub fn iou(&self, other: &Self) -> f32 {
        let intersection = self.bounding_box.rectangle_intersection(other.bounding_box);
        let union = self.bounding_box.area() + other.bounding_box.area();

        intersection / (union - intersection)
    }
}
