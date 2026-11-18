use color_eyre::{Result, eyre::eyre};

use crate::{
    diagnostics::SolveDiagnostics, heading::HeadingReference, parameters::Localization3dParameters,
};
use coordinate_systems::{Local, Robot};
use linear_algebra::Isometry3;
use ros_z::time::Time;
use types::camera_geometry::CameraGeometry;
use types::{
    field_dimensions::FieldDimensions,
    localization::{LocalizationEstimate, LocalizationState, LocalizationStatus},
    time_wrapper::TimeWrapper,
    visual_localization::{VisualAssociationSource, VisualLocalizationFrame},
};

pub struct SolveOutput {
    pub estimate: Option<LocalizationEstimate>,
    pub diagnostics: SolveDiagnostics,
}

#[cfg(test)]
mod recording_tests;
#[cfg(test)]
mod recovery_tests;

/// Survives validated startup graph replacement without copying graph state.
pub(super) struct Lifecycle {
    time: Time,
    state: LocalizationState,
    latest_visual: Option<Time>,
    pending_visual: Option<Time>,
    last_solve: Option<Time>,
    heading_reference: Option<HeadingReference>,
    pending_bootstrap: Option<TimeWrapper<VisualLocalizationFrame>>,
}

impl Lifecycle {
    pub(super) fn new(time: Time) -> Self {
        Self {
            time,
            state: LocalizationState::Startup,
            latest_visual: None,
            pending_visual: None,
            last_solve: None,
            heading_reference: None,
            pending_bootstrap: None,
        }
    }
}

pub use crate::estimator::Localization;

impl Localization {
    pub(crate) fn has_measurement_gap(&self, time: Time) -> bool {
        time > self
            .latest_time()
            .saturating_add(self.parameters.timing.optimization_window)
    }
    pub fn new(
        time: Time,
        epoch: u64,
        parameters: &Localization3dParameters,
        field: &FieldDimensions,
        camera: &CameraGeometry,
        initial_pose: Isometry3<Robot, Local>,
    ) -> Result<Self> {
        parameters.validate().map_err(|message| eyre!(message))?;
        if !camera.intrinsics.is_valid()
            || !initial_pose
                .inner
                .to_homogeneous()
                .iter()
                .all(|v| v.is_finite())
        {
            return Err(eyre!("invalid localization initialization geometry"));
        }
        Self::new_motion(
            time,
            epoch,
            initial_pose,
            &camera.intrinsics,
            parameters.clone(),
            field,
        )
    }

    pub fn status(&self) -> LocalizationStatus {
        LocalizationStatus {
            heading: self
                .lifecycle
                .heading_reference
                .map(|heading| heading.snapshot(self.parameters.max_heading_error)),
            time: self.lifecycle.time,
            epoch: self.epoch,
            generation: self.generation,
            state: self.lifecycle.state,
        }
    }
    pub(crate) fn max_camera_gap(&self) -> std::time::Duration {
        self.parameters.timing.max_camera_gap
    }

    pub fn deadline(&self) -> Option<Time> {
        if self.lifecycle.state != LocalizationState::Tracking {
            return None;
        }
        Some(
            self.lifecycle
                .last_solve?
                .saturating_add(self.parameters.tracking_timeout)
                .min(
                    self.lifecycle
                        .latest_visual?
                        .saturating_add(self.parameters.visual_tracking_timeout),
                ),
        )
    }

    pub fn advance_time(&mut self, now: Time) {
        if self.deadline().is_some_and(|deadline| deadline <= now) {
            if let Some(heading) = self.lifecycle.heading_reference.as_mut() {
                heading.interrupt_tracking();
            }
            self.lifecycle.time = now;
            self.lifecycle.state = LocalizationState::LostTrack;
        }
    }

    pub fn ingest_visual_localization_frame(
        &mut self,
        frame: TimeWrapper<VisualLocalizationFrame>,
    ) -> Result<bool> {
        let time = frame.time;
        if frame.inner.epoch != self.epoch
            || frame.inner.generation != self.generation
            || self
                .lifecycle
                .latest_visual
                .is_some_and(|accepted| time <= accepted)
        {
            return Ok(false);
        }
        // Recovery requires evidence acquired after loss, not delayed pre-loss work.
        if self.lifecycle.state == LocalizationState::LostTrack && time <= self.lifecycle.time {
            return Ok(false);
        }
        if self.lifecycle.state == LocalizationState::Startup
            || frame.inner.source == VisualAssociationSource::Global
        {
            if (self.lifecycle.state != LocalizationState::Startup
                && self.lifecycle.heading_reference.is_none())
                || !crate::alignment::valid_visual_frame(&frame.inner, &self.parameters.visual)
                || self
                    .lifecycle
                    .pending_bootstrap
                    .as_ref()
                    .is_some_and(|old| old.time >= time)
            {
                return Ok(false);
            }
            self.lifecycle.pending_bootstrap = Some(frame);
            return Ok(true);
        }
        let inserted = self.ingest_visual(frame)?;
        if inserted {
            self.lifecycle.pending_visual = Some(
                self.lifecycle
                    .pending_visual
                    .map_or(time, |old| old.max(time)),
            );
        }
        Ok(inserted)
    }

    pub fn solve(&mut self, now: Time) -> SolveOutput {
        let started = std::time::Instant::now();
        self.advance_time(now);
        // Incorporate this batch's motion before carrying its trajectory/velocity
        // into a bootstrap candidate, especially during moving startup.
        let heading = self.lifecycle.heading_reference;
        let solved = self.solve_with_heading(heading.as_ref());
        if solved.visual_rejected {
            self.lifecycle.pending_visual = None;
        }
        // Retain the motion-conditioned posterior through visual loss.
        // Global matches are observations, not permission to replace that history.
        let solved = self.try_bootstrap(now).unwrap_or(solved);
        if solved.estimate.is_none() {
            self.lifecycle.pending_visual = None;
        }
        let estimate = solved.estimate.map(|mut estimate| {
            let visual_time = self
                .lifecycle
                .pending_visual
                .or(self.lifecycle.latest_visual);
            let visual_valid = visual_time.is_some_and(|time| {
                time.saturating_add(self.parameters.visual_tracking_timeout) > now
                    && (self.lifecycle.state != LocalizationState::LostTrack
                        || time > self.lifecycle.time)
            });
            // An initialization candidate is not field localization until the
            // optimized frame agrees with its actual pixel observations.
            let field_valid = estimate.robot_to_field.is_some()
                && crate::alignment::valid_visual_rms(
                    self.accepted_visual_rms(),
                    &self.parameters.visual,
                )
                && visual_time.is_some_and(|time| self.attitude_at(time).is_some());
            if solved.converged && visual_valid && field_valid {
                let continuing_tracking = self.lifecycle.state == LocalizationState::Tracking;
                if let Some(time) = self.lifecycle.pending_visual
                    && let Some((field, imu)) =
                        self.field_heading_at(time).zip(self.attitude_at(time))
                {
                    match self.lifecycle.heading_reference.as_mut() {
                        None => {
                            self.lifecycle.heading_reference =
                                Some(HeadingReference::new(time, field, imu))
                        }
                        Some(reference) if continuing_tracking => reference.observe_tracking(
                            time,
                            field,
                            imu,
                            self.parameters.max_heading_reference_drift_per_second,
                        ),
                        Some(reference) => reference.interrupt_tracking(),
                    }
                }
                self.lifecycle.latest_visual = visual_time;
                self.lifecycle.pending_visual = None;
                if self.lifecycle.state != LocalizationState::Tracking {
                    self.lifecycle.time = now;
                    self.lifecycle.state = LocalizationState::Tracking;
                }
            }
            if self.lifecycle.state == LocalizationState::Startup {
                estimate.robot_to_field = None;
            }
            if solved.converged {
                self.lifecycle.last_solve = Some(estimate.time);
            }
            estimate
        });
        self.advance_time(now);
        let mut diagnostics = solved.diagnostics;
        diagnostics.estimation_duration = started.elapsed();
        SolveOutput {
            estimate,
            diagnostics,
        }
    }

    fn try_bootstrap(&mut self, now: Time) -> Option<crate::estimator::SolveResult> {
        let frame = self.lifecycle.pending_bootstrap.as_ref()?;
        if (self.lifecycle.state == LocalizationState::LostTrack
            && frame.time <= self.lifecycle.time)
            || self
                .lifecycle
                .pending_visual
                .or(self.lifecycle.latest_visual)
                .is_some_and(|time| frame.time <= time)
            || frame.time > now
            || frame
                .time
                .saturating_add(self.parameters.visual_tracking_timeout)
                <= now
        {
            self.lifecycle.pending_bootstrap = None;
            return None;
        }
        // Source streams arrive independently. Keep the newest recovery image until
        // its IMU bracket is available (or the normal freshness limit expires).
        self.attitude_at(frame.time)?;
        let frame = self.lifecycle.pending_bootstrap.take()?;
        let time = frame.time;
        let heading = self.lifecycle.heading_reference;
        if self.lifecycle.state != LocalizationState::Startup
            && let Some(heading) = heading.as_ref()
        {
            if !self
                .ingest_recovery(frame, heading)
                .inspect_err(|error| tracing::warn!(%error, "discarding recovery observations"))
                .ok()?
            {
                return None;
            }
            let solved = self.solve_with_heading(Some(heading));
            if !solved.visual_rejected && solved.converged {
                self.lifecycle.pending_visual = Some(time);
            }
            return Some(solved);
        }
        let candidate = self.bootstrap_candidate(frame, heading.as_ref());
        let predicted_height = self.height_prediction(time);
        let mut candidate = candidate
            .inspect_err(|error| tracing::warn!(%error, "discarding recovery candidate"))
            .ok()??;
        let solved = candidate.solve_with_heading(heading.as_ref());
        if !solved.converged
            || solved
                .estimate
                .as_ref()
                .is_none_or(|estimate| estimate.robot_to_field.is_none())
            || !crate::alignment::valid_visual_rms(
                candidate.accepted_visual_rms(),
                &self.parameters.visual,
            )
        {
            tracing::debug!(time = ?time, converged = solved.converged, failure = ?solved.diagnostics.failure, termination = %solved.diagnostics.termination, rms = ?candidate.accepted_visual_rms(), "discarding unsolved recovery");
            return None;
        }
        if let Some(prediction) = predicted_height {
            let candidate_height = candidate.height_prediction(time)?;
            if !prediction.agrees_with(&candidate_height, self.parameters.recovery_height_gate) {
                tracing::warn!(
                    predicted_height = prediction.height,
                    recovered_height = candidate_height.height,
                    "discarding height-inconsistent recovery"
                );
                return None;
            }
        }
        // Publish only the validated graph, evaluated through the latest motion sample.
        candidate.lifecycle = std::mem::replace(&mut self.lifecycle, Lifecycle::new(now));
        *self = candidate;
        self.lifecycle.pending_visual = Some(time);
        Some(solved)
    }
}
