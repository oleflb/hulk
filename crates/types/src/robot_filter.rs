use std::time::SystemTime;

use serde::{Deserialize, Serialize};
use serialize_hierarchy::SerializeHierarchy;

use linear_algebra::Point2;
use coordinate_systems::Ground;

use crate::multivariate_normal_distribution::MultivariateNormalDistribution;

#[derive(Clone, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub struct Hypothesis {
    // [ground_x, ground_y, velocity_x, velocity_y]
    pub robot_state: MultivariateNormalDistribution<4>,

    pub validity: f32,
    pub last_update: SystemTime,
}

#[derive(Clone, Debug, Serialize, Deserialize, SerializeHierarchy)]
pub struct Measurement {
    pub location: Point2<Ground>,
    pub score: f32,
}
