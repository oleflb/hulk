use crate::{hypothesis::BallHypothesis, BallFilter};

pub struct RemovedHypotheses(pub(crate) Vec<BallHypothesis>);

impl RemovedHypotheses {
    pub fn inner(self) -> Vec<BallHypothesis> {
        self.0
    }
}

pub struct ValidHypotheses(Vec<BallHypothesis>);

pub trait HypothesisMerger {
    fn partition(
        &mut self,
        is_valid: impl Fn(&BallHypothesis) -> bool,
    ) -> (ValidHypotheses, RemovedHypotheses);
    fn merge(
        &mut self,
        valid_hypotheses: ValidHypotheses,
        merge_criterion: impl Fn(&BallHypothesis, &BallHypothesis) -> bool,
    );
}

impl HypothesisMerger for BallFilter {
    fn partition(
        &mut self,
        is_valid: impl Fn(&BallHypothesis) -> bool,
    ) -> (ValidHypotheses, RemovedHypotheses) {
        let (valid, removed) = self.hypotheses.drain(..).partition(is_valid);
        (ValidHypotheses(valid), RemovedHypotheses(removed))
    }

    fn merge(
        &mut self,
        mut valid_hypotheses: ValidHypotheses,
        merge_criterion: impl Fn(&BallHypothesis, &BallHypothesis) -> bool,
    ) {
        self.hypotheses =
            valid_hypotheses
                .0
                .drain(..)
                .fold(vec![], |mut deduplicated, hypothesis| {
                    let mergeable_hypothesis =
                        deduplicated.iter_mut().find(|existing_hypothesis| {
                            merge_criterion(existing_hypothesis, &hypothesis)
                        });

                    if let Some(mergeable_hypothesis) = mergeable_hypothesis {
                        mergeable_hypothesis.merge(hypothesis)
                    } else {
                        deduplicated.push(hypothesis);
                    }

                    deduplicated
                })
    }
}
