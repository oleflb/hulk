use std::time::{Duration, SystemTime};

use nalgebra::{Matrix2, Matrix4};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use coordinate_systems::Ground;
use linear_algebra::{Isometry2, Point2};

use moving::MovingHypothesis;
use resting::RestingHypothesis;

use crate::filtered_ball::BallPosition;

pub mod moving;
pub mod resting;

#[derive(Clone, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect)]
pub struct BallHypothesis {
    moving: MovingHypothesis,
    resting: RestingHypothesis,
    last_seen: SystemTime,
    pub validity: f32,
}

impl BallHypothesis {
    pub fn new(
        moving_hypothesis: MovingHypothesis,
        resting_hypothesis: RestingHypothesis,
        last_seen: SystemTime,
    ) -> Self {
        Self {
            moving: moving_hypothesis,
            resting: resting_hypothesis,
            last_seen,
            validity: 1.0,
        }
    }

    pub fn choose_ball(&self, velocity_threshold: f32) -> BallPosition<Ground> {
        if self.moving.velocity().norm() < velocity_threshold {
            return self.resting.into();
        };
        self.moving.into()
    }

    pub fn predict(
        &mut self,
        delta_time: Duration,
        last_to_current_odometry: Isometry2<Ground, Ground>,
        velocity_decay: f32,
        moving_process_noise: Matrix4<f32>,
        resting_process_noise: Matrix2<f32>,
        velocity_threshold: f32,
    ) {
        self.moving.predict(
            delta_time,
            last_to_current_odometry,
            velocity_decay,
            moving_process_noise,
        );
        self.resting
            .predict(last_to_current_odometry, resting_process_noise);

        if self.moving.velocity().norm() < velocity_threshold {
            self.resting.reset(self.moving.position());
        }
    }

    pub fn update(
        &mut self,
        detection_time: SystemTime,
        measurement: Point2<Ground>,
        noise: Matrix2<f32>,
    ) {
        self.last_seen = detection_time;
        self.moving.update(measurement, noise);
        self.resting.update(measurement, noise);
        self.validity += 1.0;
    }

    pub fn merge(&mut self, other: BallHypothesis) {
        self.moving.merge(other.moving);
        self.resting.merge(other.resting);
    }
}
