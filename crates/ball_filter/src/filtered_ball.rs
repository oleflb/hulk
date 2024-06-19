use std::time::SystemTime;

use coordinate_systems::Ground;
use linear_algebra::{Point2, Vector2};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use crate::hypothesis::{moving::MovingHypothesis, resting::RestingHypothesis};

#[derive(
    Debug, Clone, Copy, PathDeserialize, PathSerialize, PathIntrospect, Serialize, Deserialize,
)]
pub struct BallPosition<Frame> {
    pub position: Point2<Frame>,
    pub velocity: Vector2<Frame>,
    pub last_seen: SystemTime,
}

impl From<MovingHypothesis> for BallPosition<Ground> {
    fn from(hypothesis: MovingHypothesis) -> Self {
        Self {
            position: hypothesis.position(),
            velocity: hypothesis.velocity(),
            last_seen: SystemTime::now(),
        }
    }
}

impl From<RestingHypothesis> for BallPosition<Ground> {
    fn from(hypothesis: RestingHypothesis) -> Self {
        Self {
            position: hypothesis.position(),
            velocity: Vector2::zeros(),
            last_seen: SystemTime::now(),
        }
    }
}
