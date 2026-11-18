use std::{collections::BTreeMap, time::Duration};

use color_eyre::{Result, eyre::eyre};
use coordinate_systems::{Field, ImuReference, Robot};
use linear_algebra::{IntoTransform, Isometry3, Orientation3, Point3, Vector3};
use localization_fagra::variables::{FieldAlignment, PoseControl, TrajectoryState};
use nalgebra::{Matrix2, SMatrix, UnitQuaternion};
use projection::intrinsic::Intrinsic;
use ros_z::time::Time;
use types::{time_wrapper::TimeWrapper, visual_localization::VisualLocalizationFrame};

use super::{Localization, control_keys};
use crate::{
    alignment::{seed_alignment, seed_recovery_alignment, valid_visual_frame},
    heading::HeadingReference,
};

pub(crate) enum ConflictRecovery {
    Rejected,
    AwaitingMotion,
    Recovered(Box<super::SolveResult>),
}

pub(super) struct SupportedLegs {
    // Renewed only by an accepted, independently well-conditioned image.
    visual_time: Time,
    // At most 64 records / six knots: three for evidence, two for image age, one solve margin.
    pending: Vec<MotionRecord>,
}

/// Source observations and admission cursors survive reconstruction; graph handles do not.
#[derive(Clone)]
pub(super) struct MotionHistory {
    pub records: Vec<MotionRecord>,
    pub start: Time,
    pub attitudes: BTreeMap<Time, Orientation3<ImuReference, f64>>,
    pub landmarks: BTreeMap<Time, Vec<Point3<Field>>>,
    pub latest_vo_epoch: Option<u64>,
    pub latest_kinematic_time: Option<Time>,
}

impl MotionHistory {
    pub(super) fn new(start: Time) -> Self {
        Self {
            records: Vec::new(),
            start,
            attitudes: BTreeMap::new(),
            landmarks: BTreeMap::new(),
            latest_vo_epoch: None,
            latest_kinematic_time: None,
        }
    }

    fn push(&mut self, record: MotionRecord, start: Time) {
        if start > self.start {
            self.records.retain(|record| record.start() >= start);
            // Preserve the source sample needed to interpolate the left boundary.
            if let Some((&before, _)) = self.attitudes.range(..=start).next_back() {
                self.attitudes.retain(|time, _| *time >= before);
            }
            self.start = start;
        }
        self.records.push(record);
    }

    pub(super) fn attitude_at(
        &self,
        time: Time,
        max_gap: Duration,
    ) -> Option<Orientation3<ImuReference, f64>> {
        let (&before, a) = self.attitudes.range(..=time).next_back()?;
        if before == time {
            return Some(*a);
        }
        let (&after, b) = self.attitudes.range(time..).next()?;
        if after.duration_since(before) > max_gap {
            return None;
        }
        let fraction = (time.as_nanos() - before.as_nanos()) as f64
            / (after.as_nanos() - before.as_nanos()) as f64;
        Some(a.slerp(*b, fraction))
    }
}

/// Accepted, normalized observations. Replay bypasses stream admission, not factor construction.
#[derive(Clone)]
pub(super) enum MotionRecord {
    Imu {
        time: Time,
        angular_velocity: Vector3<Robot, f64>,
        attitude: linear_algebra::Orientation3<coordinate_systems::ImuReference, f64>,
        /// Raw specific force; calibration is applied when rebuilding intervals.
        force: Vector3<Robot, f64>,
    },
    Feet {
        time: Time,
        vertices: [Point3<Robot, f64>; 2],
    },
    Kinematic {
        previous_time: Time,
        time: Time,
        translation: linear_algebra::Vector2<coordinate_systems::Ground, f64>,
        information_root: Matrix2<f64>,
    },
    Visual {
        previous_time: Time,
        time: Time,
        transform: Isometry3<Robot, Robot, f64>,
    },
}

impl MotionRecord {
    fn start(&self) -> Time {
        match self {
            Self::Imu { time, .. } | Self::Feet { time, .. } => *time,
            Self::Kinematic { previous_time, .. } | Self::Visual { previous_time, .. } => {
                *previous_time
            }
        }
    }

    fn end(&self) -> Time {
        match self {
            Self::Imu { time, .. }
            | Self::Feet { time, .. }
            | Self::Kinematic { time, .. }
            | Self::Visual { time, .. } => *time,
        }
    }

    fn insert(self, estimator: &mut Localization) -> Result<()> {
        match self {
            Self::Imu {
                time,
                angular_velocity,
                attitude,
                force,
            } => estimator.insert_imu(time, angular_velocity, attitude, force),
            Self::Feet { time, vertices } => estimator.insert_feet(time, vertices),
            Self::Kinematic {
                previous_time,
                time,
                translation,
                information_root,
            } => estimator.insert_kinematic_odometry(
                previous_time,
                time,
                translation,
                information_root,
            ),
            Self::Visual {
                previous_time,
                time,
                transform,
            } => estimator.insert_visual_odometry(previous_time, time, transform),
        }
    }
}

impl Localization {
    /// Global frames must have passed association's ambiguity checks; motion agreement is not uniqueness.
    /// Both intervals must contradict live leg factors; no evidence already in a prior is removable.
    pub(crate) fn recover_conflict(
        &mut self,
        mut first: TimeWrapper<VisualLocalizationFrame>,
        mut second: TimeWrapper<VisualLocalizationFrame>,
        heading: &HeadingReference,
    ) -> Result<ConflictRecovery> {
        let Some(mut anchor) = self.latest_visual_frame.clone() else {
            return Ok(ConflictRecovery::Rejected);
        };
        if !self.pending_visuals.is_empty()
            || self.supported_legs.is_some()
            || self.last_converged_time != Some(self.latest_time)
            || anchor.time >= first.time
            || first.time >= second.time
            || second.time > self.latest_time
            || [first.inner.source, second.inner.source]
                .iter()
                .any(|source| {
                    *source != types::visual_localization::VisualAssociationSource::Global
                })
        {
            return Ok(ConflictRecovery::Rejected);
        }
        tracing::debug!(source_time = ?second.time, first_time = ?first.time,
            live_leg_count = self.kinematic_factors.len(), "checking kinematic conflict evidence");
        let Some(anchor_fit) = self.conflict_fit(&mut anchor, heading) else {
            return Ok(ConflictRecovery::Rejected);
        };
        let Some(first_fit) = self.conflict_fit(&mut first, heading) else {
            return Ok(ConflictRecovery::Rejected);
        };
        let Some(second_fit) = self.conflict_fit(&mut second, heading) else {
            return Ok(ConflictRecovery::Rejected);
        };
        if !anchor_fit.corroborates(&first_fit, &second_fit) {
            return Ok(ConflictRecovery::Rejected);
        }
        let Some(before) = self.contradicted_leg_interval(&anchor_fit, &first_fit, heading) else {
            return Ok(ConflictRecovery::Rejected);
        };
        let Some(after) = self.contradicted_leg_interval(&first_fit, &second_fit, heading) else {
            return Ok(ConflictRecovery::AwaitingMotion);
        };
        let contradicted = |record: &MotionRecord| {
            // The middle exposure is not an evidence boundary: both adjacent spans corroborate it.
            matches!(record,
            MotionRecord::Kinematic { previous_time, time, .. }
            if *previous_time >= before.start && *time <= after.end)
        };
        let can_defer = self.parameters.accelerometer.is_some()
            && self.parameters.timing.optimization_window
                >= self.parameters.timing.trajectory_spacing * 7
            && self.latest_time.duration_since(second.time)
                <= self.parameters.timing.trajectory_spacing * 2
            && self.has_continuous_force(after.end, self.latest_time);
        // Delivery-time legs are not yet visual evidence. Hold them transactionally, not discard them.
        let deferred = |record: &MotionRecord| {
            can_defer
                && matches!(record, MotionRecord::Kinematic { previous_time, .. }
                if *previous_time >= after.end)
        };
        let tail: Vec<_> = self
            .history
            .records
            .iter()
            .filter(|record| deferred(record))
            .cloned()
            .collect();
        if tail.len() > 64
            || tail.windows(2).any(|pair| pair[0].end() != pair[1].start())
            || tail.first().is_some_and(|first| {
                self.latest_time.duration_since(first.start())
                    > self.parameters.timing.trajectory_spacing * 6
            })
        {
            return Ok(ConflictRecovery::Rejected);
        }
        let records: Vec<_> = self
            .history
            .records
            .iter()
            .filter(|record| contradicted(record) || deferred(record))
            .cloned()
            .collect();
        // Prepare cached IMU factors before the transaction; conflict solves only
        // edit visual/kinematic factors, so their rollback cannot stale this cache.
        self.prepare_preintegration()?;
        let checkpoint = self.graph.checkpoint();
        let segments = self.segment_factors.clone();
        let kinematics = self.kinematic_factors.clone();
        let source_time = second.time;
        let factor_count = records.len();
        tracing::debug!(?source_time, first_time = ?first.time, anchor_time = ?anchor.time,
            solve_time = ?self.latest_time, factor_count, deferred_factor_count = tail.len(),
            "attempting kinematic conflict recovery");
        let old_seed = self.recovery_seed.take();
        let trial = (|| -> Result<Option<super::SolveResult>> {
            for record in records {
                let start = record.start();
                let (_, factor) = self
                    .kinematic_factors
                    .get(&start)
                    .copied()
                    .ok_or_else(|| eyre!("missing live kinematic factor"))?;
                factor.remove(&mut self.graph)?;
                self.kinematic_factors.remove(&start);
                let segment = self.segment_and_tau(start)?.0;
                self.segment_factors
                    .entry(segment)
                    .or_default()
                    .measurements -= 1;
            }
            self.recovery_seed = Some((second.time, second_fit.pose));
            if !self.insert_visual(first, false)? || !self.insert_visual(second, false)? {
                return Ok(None);
            }
            let start = std::time::Instant::now();
            let mut diagnostics = self.solve_diagnostics();
            let result = (|| {
                let current_yaw = self.prepare_attitude()?;
                let result = self.solve_graph(&mut diagnostics, Some(heading));
                if let Some(key) = current_yaw {
                    self.graph.remove_factor(key)?;
                }
                result
            })();
            let solved = self.finish_solve(result, diagnostics, start, false);
            tracing::debug!(?source_time, converged = solved.estimate.is_some(),
                anchor_rms = ?self.frame_rms(&anchor),
                failure = ?solved.diagnostics.failure, termination = %solved.diagnostics.termination,
                "validated kinematic conflict trial");
            if !crate::alignment::valid_visual_rms(self.frame_rms(&anchor), &self.parameters.visual)
                || solved
                    .estimate
                    .as_ref()
                    .is_none_or(|estimate| estimate.robot_to_field.is_none())
            {
                return Ok(None);
            }
            Ok(Some(solved))
        })();
        self.recovery_seed = old_seed;
        if matches!(trial, Ok(Some(_))) {
            self.history
                .records
                .retain(|record| !contradicted(record) && !deferred(record));
            self.accept_visuals();
            self.last_converged_time = Some(self.latest_time);
            // Leave enough live graph behind the bounded admission buffer for restoration.
            if can_defer {
                self.supported_legs = Some(SupportedLegs {
                    visual_time: source_time,
                    pending: tail,
                });
            }
            tracing::debug!(
                ?source_time,
                factor_count,
                "accepted kinematic conflict recovery"
            );
        } else {
            self.graph.restore(checkpoint)?;
            self.pending_visuals.clear();
            self.segment_factors = segments;
            self.kinematic_factors = kinematics;
            tracing::debug!(?source_time, factor_count, error = ?trial.as_ref().err(),
                "restored kinematic conflict recovery");
        }
        Ok(trial?.map_or(ConflictRecovery::Rejected, |solved| {
            ConflictRecovery::Recovered(Box::new(solved))
        }))
    }

    fn conflict_fit(
        &self,
        frame: &mut TimeWrapper<VisualLocalizationFrame>,
        heading: &HeadingReference,
    ) -> Option<ConflictFit> {
        if frame.inner.epoch != self.epoch
            || frame.inner.generation != self.generation
            || !valid_visual_frame(&frame.inner, &self.parameters.visual)
            || !crate::alignment::informative_landmarks(
                &frame
                    .inner
                    .associations
                    .iter()
                    .map(|a| a.field_point)
                    .collect::<Vec<_>>(),
                &self.parameters.visual,
            )
        {
            return None;
        }
        let attitude = self.attitude_at(frame.time)?;
        let (roll, pitch, _) = attitude.euler_angles();
        let tilt = linear_algebra::Rotation3::from_euler_angles(roll as f32, pitch as f32, 0.0);
        let (pose, alignment) = seed_recovery_alignment(
            &mut frame.inner,
            tilt,
            Some(types::localization::HeadingConstraint {
                expected: heading.expected(attitude),
                max_error: self.parameters.max_heading_error,
            }),
            &self.parameters.visual,
        )?;
        let pose = (alignment.to_3d() * pose).inner.cast::<f64>();
        let camera_to_robot = frame.inner.robot_to_camera.inner.cast::<f64>().inverse();
        let camera_pose = pose * camera_to_robot;
        let camera = localization_fagra::alignment::GravityCamera::new(
            (tilt.inner.cast::<f64>() * camera_to_robot.rotation)
                .to_rotation_matrix()
                .into_inner(),
            frame.inner.camera_intrinsic.focals.cast(),
            frame
                .inner
                .camera_intrinsic
                .optical_center
                .inner
                .coords
                .cast(),
            self.parameters.visual.min_reprojection_depth,
        )?;
        let ground = localization_fagra::alignment::GroundPose {
            position: camera_pose.translation.vector.xy(),
            yaw: pose.rotation.euler_angles().2,
            height: camera_pose.translation.z,
        };
        // Marginalize shared tilt uncertainty instead of counting it once per landmark.
        let mut information = SMatrix::<f64, 6, 6>::zeros();
        for association in &frame.inner.associations {
            let projection =
                camera.project(ground, association.field_point.inner.coords.xy().cast())?;
            let mut jacobian = SMatrix::<f64, 2, 6>::zeros();
            jacobian
                .fixed_view_mut::<2, 4>(0, 0)
                .copy_from(&projection.jacobian);
            jacobian
                .fixed_view_mut::<2, 2>(0, 4)
                .copy_from(&projection.tilt_jacobian);
            information +=
                jacobian.transpose() * jacobian / self.parameters.visual_feature_noise_variance;
        }
        for index in 4..6 {
            information[(index, index)] += self.parameters.imu_preintegration.tilt_sigma.powi(-2);
        }
        let covariance = information.cholesky()?.inverse();
        // Bound camera-to-body lever-arm uncertainty, including its correlation with position.
        let orientation_variance = covariance[(2, 2)] + covariance[(4, 4)] + covariance[(5, 5)];
        let position_covariance = covariance.fixed_view::<2, 2>(0, 0) * 2.0
            + Matrix2::identity()
                * (2.0 * camera_to_robot.translation.vector.norm_squared() * orientation_variance);
        if !covariance.iter().all(|value| value.is_finite())
            || position_covariance.symmetric_eigen().eigenvalues.max() > 1.0
            || covariance[(2, 2)] > self.parameters.max_heading_error.powi(2)
            || covariance[(3, 3)] > 1.0
        {
            return None;
        }
        Some(ConflictFit {
            time: frame.time,
            pose,
            covariance: position_covariance,
        })
    }

    fn contradicted_leg_interval(
        &self,
        first: &ConflictFit,
        second: &ConflictFit,
        heading: &HeadingReference,
    ) -> Option<std::ops::Range<Time>> {
        let (start, end) = (first.time, second.time);
        let mut cursor = None;
        let mut interval = None::<std::ops::Range<Time>>;
        let mut displacement = nalgebra::Vector2::zeros();
        let mut covariance = Matrix2::zeros();
        for record in &self.history.records {
            let MotionRecord::Kinematic {
                previous_time,
                time,
                translation,
                information_root,
            } = record
            else {
                continue;
            };
            if *time <= start || *previous_time >= end {
                continue;
            }
            if cursor.map_or(*previous_time > start, |cursor| *previous_time != cursor)
                || !self
                    .kinematic_factors
                    .get(previous_time)
                    .is_some_and(|(live_end, _)| live_end == time)
            {
                return None;
            }
            cursor = Some(*time);
            // Preserve factors crossing exposure boundaries rather than discarding outside evidence.
            if *previous_time < start || *time > end {
                continue;
            }
            interval.get_or_insert(*previous_time..*time).end = *time;
            let attitude = self.attitude_at(*previous_time)?;
            let rotation = heading.expected(attitude).inner.to_rotation_matrix();
            displacement += rotation * translation.inner;
            let noise = (information_root.transpose() * information_root).try_inverse()?;
            covariance += rotation.matrix() * noise * rotation.matrix().transpose();
        }
        if cursor.is_none_or(|cursor| cursor < end) {
            return None;
        }
        let interval = interval?;
        let fraction = interval.end.duration_since(interval.start).as_secs_f64()
            / end.duration_since(start).as_secs_f64();
        covariance += (first.covariance + second.covariance) * fraction.powi(2);
        let heading_error = nalgebra::Vector2::new(-displacement.y, displacement.x);
        covariance +=
            heading_error * heading_error.transpose() * self.parameters.max_heading_error.powi(2);
        let error = (second.pose.translation.vector.xy() - first.pose.translation.vector.xy())
            * fraction
            - displacement;
        let root = covariance.cholesky()?;
        let distance = error.dot(&root.solve(&error));
        // The same exp(-9) chi-square tail bound used by visual innovation admission.
        (distance.is_finite() && distance > 2.0 + 2.0 * 18.0_f64.sqrt() + 18.0).then_some(interval)
    }

    pub(crate) fn informative_visual_window(&mut self, since: Option<Time>) -> bool {
        let cutoff = self.latest_time - self.parameters.timing.optimization_window;
        self.history.landmarks.retain(|time, _| *time >= cutoff);
        let points = self
            .history
            .landmarks
            .iter()
            .filter(|(time, _)| since.is_none_or(|since| **time > since))
            .flat_map(|(_, points)| points.iter().copied())
            .collect::<Vec<_>>();
        crate::alignment::informative_landmarks(&points, &self.parameters.visual)
    }

    /// Resolve the global match's half-turn convention, then update the retained graph.
    /// Its marginalized history, alignment and motion factors must explain recovery.
    pub(crate) fn ingest_recovery(
        &mut self,
        mut frame: TimeWrapper<VisualLocalizationFrame>,
        heading: &HeadingReference,
    ) -> Result<bool> {
        let Some(attitude) = self.attitude_at(frame.time) else {
            return Ok(false);
        };
        let (roll, pitch, _) = attitude.euler_angles();
        let Some((pose, alignment)) = seed_recovery_alignment(
            &mut frame.inner,
            linear_algebra::Rotation3::from_euler_angles(roll as f32, pitch as f32, 0.0),
            types::localization::HeadingConstraint {
                expected: heading.expected(attitude),
                max_error: self.parameters.max_heading_error,
            },
            &self.parameters.visual,
        ) else {
            return Ok(false);
        };
        let time = frame.time;
        let inserted = self.ingest_visual(frame)?;
        if inserted {
            self.recovery_seed = Some((time, (alignment.to_3d() * pose).inner.cast()));
        }
        Ok(inserted)
    }

    pub(super) fn apply_recovery_seed(&mut self) -> Result<()> {
        let Some((time, field_pose)) = self.recovery_seed.take() else {
            return Ok(());
        };
        let Some(alignment) = self.alignment else {
            return Ok(());
        };
        let (segment, tau) = self.segment_and_tau(time)?;
        let old = self
            .spline(control_keys(&self.controls, segment)?)?
            .pose(tau)?
            .inner;
        let local_pose = self
            .graph
            .get(alignment)?
            .local_to_field
            .to_3d()
            .inner
            .inverse()
            * field_pose;
        let correction = local_pose * old.inverse();
        let first = *self.controls.first_key_value().unwrap().0;
        // This only seeds optimization; every prior and motion factor remains in force.
        for (&index, &key) in &self.controls {
            let weight = ((index - first) as f64 / (segment + 1 - first) as f64).clamp(0.0, 1.0);
            let mut control = self.graph.get(key)?.clone();
            let target = correction * control.pose.inner;
            control.pose.inner.translation.vector = control
                .pose
                .inner
                .translation
                .vector
                .lerp(&target.translation.vector, weight);
            control.pose.inner.rotation =
                control.pose.inner.rotation.slerp(&target.rotation, weight);
            self.graph.set(key, control)?;
        }
        Ok(())
    }

    pub(super) fn accept_motion(&mut self, record: MotionRecord) -> Result<bool> {
        record.clone().insert(self)?;
        self.commit_time(record.end());
        let first = (self.oldest_window_segment() - 1).max(0);
        let start =
            Time::from_nanos(self.origin.as_nanos() + first * self.parameters.timing.knot_ns());
        self.history.push(record, start);
        Ok(true)
    }

    fn has_continuous_force(&self, start: Time, end: Time) -> bool {
        let Some((&before, _)) = self.history.attitudes.range(..=start).next_back() else {
            return false;
        };
        let mut cursor = before;
        for (&time, _) in self.history.attitudes.range(start..=end) {
            if time.duration_since(cursor) > self.parameters.timing.max_imu_gap {
                return false;
            }
            cursor = time;
        }
        end.duration_since(cursor) <= self.parameters.timing.max_imu_gap
            && !self.history.records.iter().any(|record| {
                matches!(record,
                MotionRecord::Imu { time, force, .. } if *time >= start && *time <= end
                    && !force.inner.iter().all(|value| value.is_finite()))
            })
    }

    /// Rebootstrap recent motion without carrying field-conditioned marginal priors.
    /// The active estimator is untouched until Localization validates this candidate.
    pub(crate) fn bootstrap_candidate(
        &self,
        mut frame: TimeWrapper<VisualLocalizationFrame>,
        heading: Option<&HeadingReference>,
    ) -> Result<Option<Self>> {
        let Some((segment, tau)) = self.check_time(frame.time, "global recovery")? else {
            return Ok(None);
        };
        if frame.inner.epoch != self.epoch
            || frame.inner.generation != self.generation
            || !valid_visual_frame(&frame.inner, &self.parameters.visual)
            || frame.time > self.latest_time
        {
            return Ok(None);
        }
        let Some(attitude) = self.attitude_at(frame.time) else {
            return Ok(None);
        };
        let old = self
            .spline(control_keys(&self.controls, segment)?)?
            .state(tau)?;
        // IMU tilt at exposure, with the existing Local heading convention.
        let (roll, pitch, _) = attitude.euler_angles();
        let yaw = old.pose.inner.rotation.euler_angles().2;
        let robot_to_local = linear_algebra::Rotation3::wrap(
            UnitQuaternion::from_euler_angles(roll, pitch, yaw).cast(),
        );
        let seed = match heading {
            Some(heading) => seed_recovery_alignment(
                &mut frame.inner,
                robot_to_local,
                types::localization::HeadingConstraint {
                    expected: heading.expected(attitude),
                    max_error: self.parameters.max_heading_error,
                },
                &self.parameters.visual,
            ),
            None => seed_alignment(
                robot_to_local,
                frame.inner.robot_to_camera,
                frame.inner.camera_intrinsic,
                &mut frame.inner.associations,
                true,
                &self.parameters.visual,
            ),
        };
        let Some((pose, seed)) = seed else {
            return Ok(None);
        };
        let alignment = FieldAlignment {
            local_to_field: seed.inner.cast().framed_transform(),
        };
        let correction = pose.inner.cast::<f64>() * old.pose.inner.inverse();
        let mut candidate = self.rebuild_motion(
            frame.time,
            TrajectoryState {
                pose: linear_algebra::Framed::wrap(pose.inner.cast()),
                velocity: Vector3::wrap(correction.rotation * old.velocity.inner),
            },
            &frame.inner.camera_intrinsic,
            None,
        )?;
        candidate.generation = self.generation.wrapping_add(1);
        candidate.alignment = Some(candidate.graph.add(alignment));
        for segment in candidate.segments() {
            candidate.add_containment(segment)?;
        }
        frame.inner.generation = candidate.generation;
        if !candidate.ingest_visual(frame)? {
            return Ok(None);
        }
        Ok(Some(candidate))
    }

    fn rebuild_motion(
        &self,
        time: Time,
        anchor: TrajectoryState,
        intrinsics: &Intrinsic,
        height_sigma: Option<f64>,
    ) -> Result<Self> {
        let mut candidate = Self::empty(
            self.origin,
            self.epoch,
            intrinsics,
            self.parameters.clone(),
            self.field_half_extents,
        )?;
        candidate.generation = self.generation;
        candidate.initialize_biases(self.history.start, self.bias_at(self.history.start)?)?;
        let first = self.segment_and_tau(self.history.start)?.0;
        let last = self.segment_and_tau(self.latest_time)?.0;
        let reference_attitude = self
            .attitude_at(time)
            .ok_or_else(|| color_eyre::eyre::eyre!("missing anchor IMU attitude"))?;
        let yaw_offset =
            anchor.pose.inner.rotation.euler_angles().2 - reference_attitude.euler_angles().2;
        for index in first - 1..=last + 2 {
            let stamp = self.origin.as_nanos() + index * self.parameters.timing.knot_ns();
            let sample_time = Time::from_nanos(stamp);
            let attitude = self.attitude_at(sample_time).unwrap_or_else(|| {
                self.history
                    .attitudes
                    .range(..=sample_time)
                    .next_back()
                    .or_else(|| self.history.attitudes.first_key_value())
                    .map_or(reference_attitude, |(_, a)| *a)
            });
            let (roll, pitch, yaw) = attitude.euler_angles();
            // Controls are initial guesses, not interpolation samples. Refit the
            // recorded motion instead of copying a possibly corrupted spline tail.
            let position = anchor.pose.inner.translation.vector
                + anchor.velocity.inner * ((stamp - time.as_nanos()) as f64 * 1e-9);
            let control = PoseControl {
                pose: linear_algebra::Framed::wrap(nalgebra::Isometry3::from_parts(
                    position.into(),
                    UnitQuaternion::from_euler_angles(roll, pitch, yaw + yaw_offset),
                )),
            };
            candidate
                .controls
                .insert(index, candidate.graph.add(control));
        }
        candidate.add_anchor(time, anchor, height_sigma)?;
        for segment in first..=last {
            candidate.add_motion_prior(segment, false)?;
        }
        for record in self.history.records.iter().cloned() {
            record.insert(&mut candidate)?;
        }
        candidate.preintegration.restore_boundary(
            self.origin,
            self.history.start,
            &self.preintegration,
        );
        candidate.history.clone_from(&self.history);
        candidate.latest_time = self.latest_time;
        Ok(candidate)
    }
}

struct ConflictFit {
    time: Time,
    pose: nalgebra::Isometry3<f64>,
    covariance: Matrix2<f64>,
}

impl ConflictFit {
    fn corroborates(&self, first: &Self, second: &Self) -> bool {
        let before = first.time.duration_since(self.time).as_secs_f64();
        let after = second.time.duration_since(first.time).as_secs_f64();
        let velocity_before =
            (first.pose.translation.vector.xy() - self.pose.translation.vector.xy()) / before;
        let velocity_after =
            (second.pose.translation.vector.xy() - first.pose.translation.vector.xy()) / after;
        let covariance = self.covariance / before.powi(2)
            + first.covariance * (before.recip() + after.recip()).powi(2)
            + second.covariance / after.powi(2);
        let Some(root) = covariance.cholesky() else {
            return false;
        };
        let error = velocity_after - velocity_before;
        let distance = error.dot(&root.solve(&error));
        // Fail closed on changing translation: a single displaced image must not explain away legs.
        distance.is_finite() && distance <= 2.0 + 2.0 * 18.0_f64.sqrt() + 18.0
    }
}
