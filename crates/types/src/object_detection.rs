use geometry::rectangle::Rectangle;
use linear_algebra::{vector, Point2};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use coordinate_systems::Pixel;

#[derive(
    Debug,
    Clone,
    Copy,
    Serialize,
    Deserialize,
    PathSerialize,
    PathDeserialize,
    PathIntrospect,
    PartialEq,
    Eq,
)]
pub enum DetectedObject {
    Ball,
    Robot,
    GoalPost,
    PenaltySpot,
}

impl DetectedObject {
    pub fn from_index(index: usize) -> Option<Self> {
        match index {
            0 => Some(Self::Ball),
            1 => Some(Self::Robot),
            2 => Some(Self::GoalPost),
            3 => Some(Self::PenaltySpot),
            _ => None,
        }
    }
}

#[derive(
    Debug, Clone, Copy, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct BoundingBox {
    pub bounding_box: Rectangle<Pixel>,
    pub score: f32,
    pub class: DetectedObject,
}

impl BoundingBox {
    pub const fn new(class: DetectedObject, score: f32, bounding_box: Rectangle<Pixel>) -> Self {
        Self {
            bounding_box,
            score,
            class,
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
