use std::time::SystemTime;

use coordinate_systems::Ground;
use linear_algebra::Point2;
use nalgebra::{Matrix2, Matrix4};

use crate::{
    hypothesis::{moving::MovingHypothesis, resting::RestingHypothesis},
    BallFilter, BallHypothesis,
};

pub trait HypothesisSpawner {
    fn spawn(
        &mut self,
        detection_time: SystemTime,
        measurement: Point2<Ground>,
        initial_moving_covariance: Matrix4<f32>,
        initial_resting_covariance: Matrix2<f32>,
    );
}

impl HypothesisSpawner for BallFilter {
    fn spawn(
        &mut self,
        detection_time: SystemTime,
        measurement: Point2<Ground>,
        initial_moving_covariance: Matrix4<f32>,
        initial_resting_covariance: Matrix2<f32>,
    ) {
        let initial_state = nalgebra::vector![measurement.x(), measurement.y(), 0.0, 0.0];

        let moving_hypothesis = MovingHypothesis::new(initial_state, initial_moving_covariance);
        let resting_hypothesis =
            RestingHypothesis::new(initial_state.xy(), initial_resting_covariance);

        self.hypotheses.push(BallHypothesis::new(
            moving_hypothesis,
            resting_hypothesis,
            detection_time,
        ))
    }
}
