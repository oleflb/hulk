use std::collections::HashSet;

use coordinate_systems::{Ground, Pixel};
use geometry::line::Line2;
use linear_algebra::Point2;
use serde::{Deserialize, Serialize};
use serialize_hierarchy::SerializeHierarchy;

#[derive(Clone, Default, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub struct LineData {
    pub lines: Vec<Line2<Ground>>,
    pub used_segments: HashSet<Point2<Pixel, u16>>,
}

#[derive(Clone, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub enum LineDiscardReason {
    TooFewPoints,
    LineTooShort,
    LineTooLong,
    TooFarAway,
}
