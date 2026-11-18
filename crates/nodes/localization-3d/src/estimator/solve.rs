use std::time::{Duration, Instant};

use color_eyre::{Result, eyre::eyre};
use fagra::{BlockId, OptimizeOptions, SolverError, StateKey};
use localization_fagra::variables::{CameraIntrinsics, FieldAlignment, ImuBias, PoseControl};
use types::localization::LocalizationEstimate;

use super::{Graph, Localization, covariance::EstimateCovariance};
use crate::{diagnostics::SolveDiagnostics, heading::HeadingReference};

pub(crate) struct SolveResult {
    pub estimate: Option<LocalizationEstimate>,
    pub diagnostics: SolveDiagnostics,
    pub converged: bool,
    pub visual_rejected: bool,
}

/// LM rejects individual trials; application validation can still reject the whole update.
struct StateSnapshot {
    controls: Vec<(StateKey<PoseControl>, PoseControl)>,
    alignment: Option<(StateKey<FieldAlignment>, FieldAlignment)>,
    intrinsics: (StateKey<CameraIntrinsics>, CameraIntrinsics),
    biases: Vec<(StateKey<ImuBias>, ImuBias)>,
}

impl StateSnapshot {
    fn capture(estimator: &Localization) -> Result<Self> {
        let graph = &estimator.graph;
        Ok(Self {
            controls: estimator
                .controls
                .values()
                .map(|&key| Ok((key, graph.get(key)?.clone())))
                .collect::<Result<_>>()?,
            alignment: estimator
                .alignment
                .map(|key| Ok::<_, color_eyre::Report>((key, graph.get(key)?.clone())))
                .transpose()?,
            intrinsics: (
                estimator.intrinsics,
                graph.get(estimator.intrinsics)?.clone(),
            ),
            biases: estimator
                .biases
                .values()
                .map(|&key| Ok((key, graph.get(key)?.clone())))
                .collect::<Result<_>>()?,
        })
    }

    fn restore(self, graph: &mut Graph) -> Result<()> {
        for (key, value) in self.controls {
            graph.set(key, value)?;
        }
        if let Some((key, value)) = self.alignment {
            graph.set(key, value)?;
        }
        graph.set(self.intrinsics.0, self.intrinsics.1)?;
        for (key, value) in self.biases {
            graph.set(key, value)?;
        }
        Ok(())
    }
}

impl Localization {
    fn trace_factor_costs(&self, stage: &str, total: Option<f64>) {
        if self.pending_visuals.is_empty() || !tracing::enabled!(tracing::Level::TRACE) {
            return;
        }
        tracing::trace!(time = ?self.latest_time, stage, ?total,
            imu_costs = ?self.imu_factor_costs(),
            pending_visual_cost = ?self.pending_visual_cost(),
            "localization factor costs");
    }

    pub(super) fn solve_options(&self) -> OptimizeOptions<f64> {
        let solver = &self.parameters.solver;
        OptimizeOptions {
            max_iterations: solver.max_iterations,
            gradient_tolerance: solver.gradient_tolerance,
            step_tolerance: solver.step_tolerance,
            cost_tolerance: solver.cost_tolerance,
        }
    }

    #[cfg(test)]
    pub(crate) fn solve_motion(&mut self) -> SolveResult {
        self.solve_with_heading(None)
    }

    pub(crate) fn solve_with_heading(&mut self, heading: Option<&HeadingReference>) -> SolveResult {
        let had_visual_update = !self.pending_visuals.is_empty();
        let mut visual_rejected = false;
        let start = Instant::now();
        let mut diagnostics = SolveDiagnostics {
            time: self.latest_time,
            epoch: self.epoch,
            duration: Duration::ZERO,
            estimation_duration: Duration::ZERO,
            ingestion_duration: Duration::ZERO,
            iterations: None,
            lm_attempts: 0,
            lm_rejected_steps: 0,
            gradient_norm: None,
            motion_rebuilt: false,
            initial_cost: None,
            final_cost: None,
            termination: "failed".into(),
            state_count: 0,
            measurement_count: 0,
            failure: None,
            imu_bias: None,
        };
        let result = (|| -> Result<_> {
            let mut current_yaw = None;
            let result = self
                .prepare_preintegration()
                .and_then(|()| self.prepare_attitude())
                .and_then(|yaw| {
                    current_yaw = yaw;
                    let mut result = self.solve_graph(&mut diagnostics, heading);
                    if result.is_err() && had_visual_update {
                        self.discard_visuals()?;
                        visual_rejected = true;
                        result = self.solve_graph(&mut diagnostics, heading);
                    }
                    result
                });
            let result = if result.is_ok() {
                self.accept_visuals();
                result
            } else {
                self.discard_visuals().and(result)
            };
            if let Some(key) = current_yaw {
                self.graph.remove_factor(key)?;
            }
            // Retire the restored graph after rejection too; continuous motion
            // must not grow an unbounded graph while field updates are withheld.
            self.retire_old_segments()?;
            result
        })();
        let (estimate, converged) = match result {
            Ok((estimate, converged)) => (Some(estimate), converged),
            Err(error) => {
                diagnostics.failure = Some(format!("{error:#}"));
                (None, false)
            }
        };
        if converged {
            self.last_converged_time = Some(self.latest_time);
        }
        if estimate.is_none() {
            diagnostics.imu_bias = None;
        }
        diagnostics.duration = start.elapsed();
        diagnostics.state_count =
            self.controls.len() + self.biases.len() + 1 + usize::from(self.alignment.is_some());
        diagnostics.measurement_count = self.segment_factors.values().map(|f| f.measurements).sum();
        SolveResult {
            estimate,
            diagnostics,
            converged,
            visual_rejected,
        }
    }

    fn solve_graph(
        &mut self,
        diagnostics: &mut SolveDiagnostics,
        heading: Option<&HeadingReference>,
    ) -> Result<(LocalizationEstimate, bool)> {
        let (segment, tau) = self.segment_and_tau(self.latest_time)?;
        let controls = self.ensure_segment(segment)?;
        self.ensure_biases(self.latest_time)?;
        let snapshot = StateSnapshot::capture(self)?;
        let blocks = self.estimate_covariance_blocks(controls)?;
        let checked = (|| -> Result<_> {
            self.trace_factor_costs("before_seed", None);
            self.apply_recovery_seed()?;
            self.trace_factor_costs("seed", None);
            let (covariance, converged) = self.optimize(&blocks, diagnostics)?;
            if let Some(heading) = heading
                && self.alignment.is_some()
                && !self.pending_visuals.is_empty()
            {
                self.validate_heading(heading)?;
            }
            if !converged && !self.pending_visuals.is_empty() {
                return Err(eyre!("visual update did not converge"));
            }
            self.validate_pending_visuals()?;
            self.validate_tilt()?;
            let covariance = match covariance {
                Some(covariance) => covariance,
                None => {
                    // Accepted partial motion still needs a fresh, undamped
                    // covariance. Failed optimization has no cached-query API.
                    let covariance = self.graph.joint_covariance(&blocks)?;
                    EstimateCovariance::from_fn(|r, c| {
                        if r < covariance.nrows() && c < covariance.ncols() {
                            covariance[(r, c)]
                        } else {
                            0.0
                        }
                    })
                }
            };
            let mut estimate =
                self.estimate_from_covariance(controls, tau, &covariance, diagnostics)?;
            if heading.is_some_and(|heading| self.validate_heading(heading).is_err()) {
                // Field inconsistency cannot erase motion evidence or stop local odometry.
                estimate.robot_to_field = None;
            }
            Ok((estimate, converged))
        })();
        if let Err(error) = &checked {
            tracing::debug!(time = ?self.latest_time, %error, visual_frames = self.pending_visuals.len(), final_cost = ?diagnostics.final_cost, "rolling back inconsistent trajectory update");
            snapshot.restore(&mut self.graph)?;
        }
        checked
    }

    fn optimize(
        &mut self,
        blocks: &[BlockId],
        diagnostics: &mut SolveDiagnostics,
    ) -> Result<(Option<EstimateCovariance>, bool)> {
        let options = self.solve_options();
        // LM's final gradient check already built the undamped model. Extract the
        // joint marginal from that model instead of visiting every factor again.
        let result = self
            .optimizer
            .solve_batch_with_covariance(&mut self.graph, &options, blocks, &Default::default())
            .map(|(report, covariance)| {
                let joint = EstimateCovariance::from_fn(|r, c| {
                    if r < covariance.nrows() && c < covariance.ncols() {
                        covariance[(r, c)]
                    } else {
                        0.0
                    }
                });
                (report, joint)
            });
        let statistics = self.optimizer.statistics();
        diagnostics.lm_attempts = statistics.attempts;
        diagnostics.lm_rejected_steps = statistics.rejected_steps;
        diagnostics.gradient_norm = statistics.gradient_norm;
        let converged = result.is_ok();
        diagnostics.iterations = statistics.cost.map(|_| statistics.accepted_steps);
        diagnostics.final_cost = statistics.cost;
        self.trace_factor_costs("trial", statistics.cost);
        // This fagra version returns the initial cost only in a successful report.
        // On failure after accepted steps it is unknown, not a reason to rebuild H.
        diagnostics.initial_cost = result
            .as_ref()
            .ok()
            .map(|(report, _)| report.initial_cost)
            .or_else(|| statistics.cost.filter(|_| statistics.accepted_steps == 0));
        diagnostics.termination = match &result {
            Ok((report, _)) => format!("{:?}", report.termination),
            Err(SolverError::NoProgress) => "NoProgress".into(),
            Err(SolverError::NoConvergence) => "MaxIterations".into(),
            // A converged numerical solve can still fail covariance rank checks.
            Err(_)
                if statistics
                    .gradient_norm
                    .is_some_and(|g| g <= options.gradient_tolerance) =>
            {
                "GradientTolerance".into()
            }
            Err(_) => "failed".into(),
        };
        self.trace_visual_audit("trial");
        let covariance = match result {
            Ok((_, covariance)) => Some(covariance),
            // LM only commits cost-decreasing steps. Its recorded accepted
            // cost replaces the old evaluation-only solve on these paths.
            Err(SolverError::NoConvergence | SolverError::NoProgress)
                if statistics.cost.is_some() =>
            {
                None
            }
            Err(error) => return Err(error.into()),
        };
        Ok((covariance, converged))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fagra::Variable;
    use linear_algebra::IntoTransform;

    #[test]
    fn snapshot_restores_all_optimized_state_families() -> Result<()> {
        let mut estimator = super::super::tests::estimator();
        let alignment = estimator.graph.add(FieldAlignment::identity());
        estimator.alignment = Some(alignment);
        let snapshot = StateSnapshot::capture(&estimator)?;
        for &(key, ref value) in &snapshot.controls {
            let mut changed = value.clone();
            changed.pose.inner.translation.x += 1.0;
            estimator.graph.set(key, changed)?;
        }
        estimator.graph.set(
            alignment,
            FieldAlignment {
                local_to_field: nalgebra::Isometry2::translation(1.0, 2.0).framed_transform(),
            },
        )?;
        let mut changed = snapshot.intrinsics.1.clone();
        changed.focal_lengths.inner.x += 1.0;
        estimator.graph.set(snapshot.intrinsics.0, changed)?;
        for &(key, ref value) in &snapshot.biases {
            let mut changed = value.clone();
            changed.gyroscope.inner.x += 1.0;
            estimator.graph.set(key, changed)?;
        }
        let expected = StateSnapshot::capture(&super::super::tests::estimator())?;
        let expected_intrinsics = snapshot.intrinsics.1.clone();
        snapshot.restore(&mut estimator.graph)?;
        for ((_, expected), &key) in expected.controls.iter().zip(estimator.controls.values()) {
            assert_eq!(estimator.graph.get(key)?.pose, expected.pose);
        }
        assert_eq!(
            estimator.graph.get(alignment)?.local_to_field,
            FieldAlignment::<f64>::identity().local_to_field
        );
        assert_eq!(
            estimator.graph.get(estimator.intrinsics)?.focal_lengths,
            expected_intrinsics.focal_lengths
        );
        for ((_, expected), &key) in expected.biases.iter().zip(estimator.biases.values()) {
            let actual = estimator.graph.get(key)?;
            assert_eq!(actual.gyroscope, expected.gyroscope);
            assert_eq!(actual.accelerometer, expected.accelerometer);
        }
        Ok(())
    }
}
