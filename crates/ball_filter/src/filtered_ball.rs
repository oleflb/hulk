use std::time::SystemTime;

use coordinate_systems::Ground;
use linear_algebra::{Point2, Vector2};

pub struct FilteredBall {
    pub position: Point2<Ground>,
    pub velocity: Vector2<Ground>,
    pub last_seen: SystemTime,
}
