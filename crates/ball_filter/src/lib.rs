use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};

mod filtered_ball;
mod hypothesis;
mod hypothesis_decayer;
mod hypothesis_merger;
mod hypothesis_predicter;
mod hypothesis_spawner;
mod hypothesis_updater;

pub use filtered_ball::FilteredBall;
pub use hypothesis::BallHypothesis;
pub use hypothesis_decayer::HypothesisDecayer;
pub use hypothesis_merger::{HypothesisMerger, RemovedHypotheses, ValidHypotheses};
pub use hypothesis_predicter::HypothesisPredicter;
pub use hypothesis_spawner::HypothesisSpawner;
pub use hypothesis_updater::HypothesisUpdater;

#[derive(
    Debug, Default, Clone, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct BallFilter {
    hypotheses: Vec<BallHypothesis>,
}

impl BallFilter {
    pub fn best_hypothesis(&self, validity_threshold: f32) -> Option<&BallHypothesis> {
        self.hypotheses
            .iter()
            .filter(|hypothesis| hypothesis.validity() >= validity_threshold)
            .max_by(|a, b| a.validity().partial_cmp(&b.validity()).unwrap())
    }

    pub fn hypotheses(&self) -> &Vec<BallHypothesis> {
        &self.hypotheses
    }
}
