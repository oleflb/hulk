use std::time::{Duration, SystemTime};

use nalgebra::{Matrix2, Matrix4};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

use coordinate_systems::Ground;
use linear_algebra::{Isometry2, Point2, Vector2};

use moving::MovingHypothesis;
use resting::RestingHypothesis;

use crate::filtered_ball::FilteredBall;

pub mod moving;
pub mod resting;

#[derive(Clone, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect)]
pub struct BallHypothesis {
    moving_hypothesis: MovingHypothesis,
    resting_hypothesis: RestingHypothesis,
    last_seen: SystemTime,
    validity: f32,
}

impl BallHypothesis {
    pub fn new(
        moving_hypothesis: MovingHypothesis,
        resting_hypothesis: RestingHypothesis,
        last_seen: SystemTime,
    ) -> Self {
        Self {
            moving_hypothesis,
            resting_hypothesis,
            last_seen,
            validity: 1.0,
        }
    }

    pub fn validity(&self) -> f32 {
        self.validity
    }

    pub fn decay_validity(&mut self, decay_factor: f32) {
        self.validity *= decay_factor;
    }

    pub fn choose_ball(&self, velocity_threshold: f32) -> FilteredBall<Ground> {
        if self.moving_hypothesis.velocity().norm() < velocity_threshold {
            return FilteredBall {
                position: self.resting_hypothesis.position(),
                velocity: Vector2::zeros(),
                last_seen: self.last_seen,
            };
        };
        FilteredBall {
            position: self.moving_hypothesis.position(),
            velocity: self.moving_hypothesis.velocity(),
            last_seen: self.last_seen,
        }
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
        self.moving_hypothesis.predict(
            delta_time,
            last_to_current_odometry,
            velocity_decay,
            moving_process_noise,
        );
        self.resting_hypothesis
            .predict(last_to_current_odometry, resting_process_noise);

        if self.moving_hypothesis.velocity().norm() < velocity_threshold {
            self.resting_hypothesis
                .move_to(self.moving_hypothesis.position());
        }
    }

    pub fn update(
        &mut self,
        detection_time: SystemTime,
        measurement: Point2<Ground>,
        noise: Matrix2<f32>,
    ) {
        self.last_seen = detection_time;
        self.moving_hypothesis.update(measurement, noise);
        self.resting_hypothesis.update(measurement, noise);
        self.validity += 1.0;
    }

    pub fn merge(&mut self, other: BallHypothesis) {
        self.moving_hypothesis.merge(other.moving_hypothesis);
        self.resting_hypothesis.merge(other.resting_hypothesis);
    }
}
