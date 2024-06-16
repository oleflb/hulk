use std::time::SystemTime;

use coordinate_systems::Ground;
use linear_algebra::Point2;
use nalgebra::Matrix2;

use crate::{BallFilter, BallHypothesis};

pub struct MatchingHypotheses<'a>(Vec<&'a mut BallHypothesis>);

impl<'a> MatchingHypotheses<'a> {
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

pub trait HypothesisUpdater {
    fn find_matching_hypotheses(
        &mut self,
        criterion: impl Fn(&BallHypothesis) -> bool,
    ) -> MatchingHypotheses<'_>;

    fn update(
        hypotheses: MatchingHypotheses<'_>,
        detection_time: SystemTime,
        measurement: Point2<Ground>,
        noise: Matrix2<f32>,
    );
}

impl HypothesisUpdater for BallFilter {
    fn find_matching_hypotheses(
        &mut self,
        criterion: impl Fn(&BallHypothesis) -> bool,
    ) -> MatchingHypotheses<'_> {
        MatchingHypotheses(
            self.hypotheses
                .iter_mut()
                .filter(|hypothesis| criterion(hypothesis as &BallHypothesis))
                .collect(),
        )
    }

    fn update(
        hypotheses: MatchingHypotheses<'_>,
        detection_time: SystemTime,
        measurement: Point2<Ground>,
        noise: Matrix2<f32>,
    ) {
        for hypothesis in hypotheses.0 {
            hypothesis.update(detection_time, measurement, noise)
        }
    }
}
