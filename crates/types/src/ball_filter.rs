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
    pub rolling_state: MultivariateNormalDistribution<4>,
    pub resting_state: MultivariateNormalDistribution<2>,

    pub validity: f32,
    pub last_update: SystemTime,
}

impl Hypothesis {
    pub fn position(&self) -> BallPosition<Ground> {
        BallPosition {
            position: Point2::from(self.resting_state.mean.xy()),
            velocity: vector![0.0, 0.0],
            last_seen: self.last_update,
        }
    }
}
