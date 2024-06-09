use crate::hypothesis::{self, BallHypothesis};

pub struct RemovedHypotheses(Vec<BallHypothesis>);

pub fn remove_hypotheses(
    hypotheses: &mut Vec<BallHypothesis>,
    hypothesis_filter: impl Fn(&BallHypothesis) -> bool,
    should_merge_hypothesis: impl Fn(&BallHypothesis, &BallHypothesis) -> bool,
) -> RemovedHypotheses {
    let (retained, removed): (Vec<_>, Vec<_>) = hypotheses
        .drain(..)
        .partition(|hypothesis| hypothesis_filter(hypothesis));

    let mut deduplicated = Vec::new();
    for hypothesis in retained {
        let mergeable_hypothesis = deduplicated.iter_mut().find(|existing_hypothesis| {
            should_merge_hypothesis(existing_hypothesis, &hypothesis)
        });

        if let Some(mergeable_hypothesis) = mergeable_hypothesis {
            mergeable_hypothesis.merge(hypothesis);
        } else {
            deduplicated.push(hypothesis);
        }
    }

    todo!();
} 
