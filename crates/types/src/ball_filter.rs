use std::time::SystemTime;

use coordinate_systems::Ground;
use linear_algebra::{vector, Point};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use crate::{
    ball_position::BallPosition, multivariate_normal_distribution::MultivariateNormalDistribution,
};

#[derive(Clone, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect)]
pub struct Hypothesis {
    pub moving_state: MultivariateNormalDistribution<4>,
    pub resting_state: MultivariateNormalDistribution<2>,

    pub validity: f32,
    pub last_update: SystemTime,
}

impl Hypothesis {
    pub fn is_resting(&self, velocity_threshold: f32) -> bool {
        self.moving_state.mean.rows(2, 2).norm() < velocity_threshold
    }

    pub fn selected_ball_position(&self, velocity_threshold: f32) -> BallPosition<Ground> {
        let (position, velocity) = if self.is_resting(velocity_threshold) {
            (Point::from(self.resting_state.mean.xy()), vector![0.0, 0.0])
        } else {
            (
                Point::from(self.moving_state.mean.xy()),
                vector![self.moving_state.mean.z, self.moving_state.mean.w],
            )
        };

        BallPosition {
            position,
            velocity,
            last_seen: self.last_update,
        }
    }
}
