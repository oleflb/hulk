use std::time::{Duration, Instant};

use color_eyre::Result;
use fagra::{BlockId, OptimizeOptions, SolverError, StateKey};
use localization_fagra::variables::{ImuBias, PoseControl};
use types::localization::LocalizationEstimate;

use super::{Localization, covariance::EstimateCovariance};
use crate::{diagnostics::SolveDiagnostics, heading::HeadingReference};

pub(crate) struct SolveResult {
    pub estimate: Option<LocalizationEstimate>,
    pub diagnostics: SolveDiagnostics,
    pub converged: bool,
    pub visual_rejected: bool,
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
        let mut diagnostics = self.solve_diagnostics();
        let result = (|| -> Result<_> {
            self.expire_supported_legs(heading)?;
            let mut current_yaw = None;
            let mut support_snapshot = None;
            let result = (|| {
                self.prepare_preintegration()?;
                current_yaw = self.prepare_attitude()?;
                if self.supported_legs.is_some() {
                    let segment = self.segment_and_tau(self.latest_time)?.0;
                    self.ensure_segment(segment)?;
                    self.ensure_biases(self.latest_time)?;
                    support_snapshot = Some(self.graph.checkpoint_states());
                }
                let mut solve = |estimator: &mut Self| -> Result<_> {
                    let result = estimator.solve_graph(&mut diagnostics, heading);
                    if result.is_err() && had_visual_update {
                        estimator.discard_visuals()?;
                        visual_rejected = true;
                        estimator.solve_graph(&mut diagnostics, heading)
                    } else {
                        result
                    }
                };
                let mut result = solve(self);
                if result.is_ok() && self.resolve_supported_legs(heading)? {
                    result = solve(self);
                }
                result
            })();
            let result = if result.is_ok() {
                self.accept_visuals();
                if had_visual_update && !visual_rejected {
                    self.renew_supported_legs(heading);
                }
                result
            } else {
                if let Some(snapshot) = support_snapshot {
                    self.graph.restore_states(snapshot)?;
                }
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
        if result.is_ok() {
            self.last_converged_time = Some(self.latest_time);
        }
        self.finish_solve(result, diagnostics, start, visual_rejected)
    }

    pub(super) fn solve_diagnostics(&self) -> SolveDiagnostics {
        SolveDiagnostics {
            time: self.latest_time,
            epoch: self.epoch,
            duration: Duration::ZERO,
            estimation_duration: Duration::ZERO,
            ingestion_duration: Duration::ZERO,
            iterations: None,
            lm_attempts: 0,
            lm_rejected_steps: 0,
            gradient_norm: None,
            initial_cost: None,
            final_cost: None,
            termination: "failed".into(),
            state_count: 0,
            measurement_count: 0,
            failure: None,
            imu_bias: None,
            motion_rebuilt: false,
        }
    }

    pub(super) fn finish_solve(
        &self,
        result: Result<LocalizationEstimate>,
        mut diagnostics: SolveDiagnostics,
        start: Instant,
        visual_rejected: bool,
    ) -> SolveResult {
        let estimate = result
            .inspect_err(|error| diagnostics.failure = Some(format!("{error:#}")))
            .ok();
        if estimate.is_none() {
            diagnostics.imu_bias = None;
        }
        diagnostics.duration = start.elapsed();
        diagnostics.state_count =
            self.controls.len() + self.biases.len() + 1 + usize::from(self.alignment.is_some());
        diagnostics.measurement_count = self.segment_factors.values().map(|f| f.measurements).sum();
        SolveResult {
            converged: estimate.is_some(),
            estimate,
            diagnostics,
            visual_rejected,
        }
    }

    pub(super) fn solve_graph(
        &mut self,
        diagnostics: &mut SolveDiagnostics,
        heading: Option<&HeadingReference>,
    ) -> Result<LocalizationEstimate> {
        let (segment, tau) = self.segment_and_tau(self.latest_time)?;
        let controls = self.ensure_segment(segment)?;
        self.ensure_biases(self.latest_time)?;
        let snapshot = self.graph.checkpoint_states();
        let checked = (|| -> Result<_> {
            self.trace_factor_costs("before_seed", None);
            self.apply_recovery_seed()?;
            self.trace_factor_costs("seed", None);
            let covariance = self.optimize(controls, diagnostics)?;
            if let Some(heading) = heading
                && self.alignment.is_some()
                && !self.pending_visuals.is_empty()
            {
                self.validate_heading(heading)?;
            }
            self.validate_pending_visuals()?;
            self.validate_tilt()?;
            let mut estimate =
                self.estimate_from_covariance(controls, tau, &covariance, diagnostics)?;
            color_eyre::eyre::ensure!(
                self.pending_visuals.is_empty() || estimate.robot_to_field.is_some(),
                "visual update has invalid field pose or covariance"
            );
            if heading.is_some_and(|heading| self.validate_heading(heading).is_err()) {
                // Field inconsistency cannot erase motion evidence or stop local odometry.
                estimate.robot_to_field = None;
            }
            Ok(estimate)
        })();
        if let Err(error) = &checked {
            tracing::debug!(time = ?self.latest_time, %error, visual_frames = self.pending_visuals.len(), final_cost = ?diagnostics.final_cost, "rolling back inconsistent trajectory update");
            self.graph.restore_states(snapshot)?;
        }
        checked
    }

    fn optimize(
        &mut self,
        controls: [StateKey<PoseControl>; 4],
        diagnostics: &mut SolveDiagnostics,
    ) -> Result<EstimateCovariance> {
        let blocks = self.estimate_covariance_blocks(controls)?;
        let (biases, fraction) = self.ensure_biases(self.latest_time)?;
        let mut options = self.solve_options();
        let mut result = self.optimize_trial(&blocks, biases, fraction, &options);
        let mut statistics = self.optimizer.statistics();
        // Failed reports do not expose the initial cost after accepted steps.
        diagnostics.initial_cost = match &result {
            Ok((report, _)) => Some(report.initial_cost),
            Err(_) => statistics.cost.filter(|_| statistics.accepted_steps == 0),
        };
        if matches!(result, Err(SolverError::NoProgress))
            && statistics.accepted_steps > 0
            && statistics.accepted_steps < options.max_iterations
            && statistics
                .step_norm
                .is_some_and(|step| step <= options.step_tolerance)
        {
            // Small steps are not convergence: use the remaining budget to reach
            // the gradient tolerance before rejecting a numerically stalled solve.
            options.max_iterations -= statistics.accepted_steps;
            options.step_tolerance = 0.0;
            result = self.optimize_trial(&blocks, biases, fraction, &options);
            let continued = self.optimizer.statistics();
            let accepted_steps = statistics.accepted_steps + continued.accepted_steps;
            let attempts = statistics.attempts + continued.attempts;
            let rejected_steps = statistics.rejected_steps + continued.rejected_steps;
            statistics = continued;
            statistics.accepted_steps = accepted_steps;
            statistics.attempts = attempts;
            statistics.rejected_steps = rejected_steps;
        }
        diagnostics.lm_attempts += statistics.attempts;
        diagnostics.lm_rejected_steps += statistics.rejected_steps;
        diagnostics.gradient_norm = statistics.gradient_norm;
        diagnostics.iterations = statistics.cost.map(|_| statistics.accepted_steps);
        diagnostics.final_cost = statistics.cost;
        self.trace_factor_costs("trial", statistics.cost);
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
        // Cost-decreasing steps alone are not safe to publish or marginalize.
        // Propagate every failure so solve_graph restores the pre-solve state.
        result.map(|(_, covariance)| covariance).map_err(Into::into)
    }

    fn optimize_trial(
        &mut self,
        blocks: &[BlockId],
        biases: [StateKey<ImuBias>; 2],
        fraction: f64,
        options: &OptimizeOptions<f64>,
    ) -> std::result::Result<(fagra::OptimizeReport<f64>, EstimateCovariance), SolverError> {
        // LM's final gradient check already built the undamped model. Extract the
        // joint marginal from that model instead of visiting every factor again.
        let pose_dimension = if self.alignment.is_some() { 27 } else { 24 };
        let (report, covariance) = self.optimizer.solve_batch_with_covariance_view(
            &mut self.graph,
            options,
            blocks,
            &Default::default(),
        )?;
        let pose = nalgebra::SMatrix::from_fn(|r, c| {
            if r < pose_dimension && c < pose_dimension {
                covariance.matrix()[(r, c)]
            } else {
                0.0
            }
        });
        let weights = [1.0 - fraction, fraction];
        let mut bias = nalgebra::SMatrix::<f64, 6, 6>::zeros();
        for (i, &row) in biases.iter().enumerate() {
            for (j, &column) in biases.iter().enumerate() {
                let block = covariance.block(row, column)?;
                bias += nalgebra::SMatrix::<f64, 6, 6>::from_fn(|r, c| {
                    weights[i] * weights[j] * block[(r, c)]
                });
            }
        }
        Ok((report, EstimateCovariance { pose, bias }))
    }
}
