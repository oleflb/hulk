use std::time::SystemTime;

use coordinate_systems::Ground;
use linear_algebra::{vector, Point2};
use serde::{Deserialize, Serialize};

use serialize_hierarchy::SerializeHierarchy;

use crate::{
    ball_position::BallPosition, multivariate_normal_distribution::MultivariateNormalDistribution,
};

#[derive(Clone, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub struct Hypothesis {
    pub state: MultivariateNormalDistribution<4>,

    pub validity: f32,
    pub last_update: SystemTime,
}

impl Hypothesis {
    pub fn position(&self) -> BallPosition<Ground> {
        BallPosition {
            position: Point2::from(self.state.mean.xy()),
            velocity: vector![self.state.mean.z, self.state.mean.w],
            last_seen: self.last_update,
        }
    }
}
