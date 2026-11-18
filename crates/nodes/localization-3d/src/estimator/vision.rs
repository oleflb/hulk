use super::{Localization, control_keys, recovery::MotionRecord};
use crate::alignment::{reprojection_rms, valid_visual_frame, valid_visual_rms};
use crate::heading::HeadingReference;
use color_eyre::{Result, eyre::eyre};
use coordinate_systems::{Field, Robot};
use linear_algebra::{IntoTransform, Isometry3, Orientation2, Point2, Point3};
use localization_fagra::factors::{
    AdjacentVisualOdometry, FrameReprojections, ReprojectionObservation, VisualOdometry,
    VisualOdometryObservation,
};
use nalgebra::SMatrix;
use ros_z::time::Time;
use types::camera_geometry::CameraGeometry;
use types::{
    time_wrapper::TimeWrapper, visual_localization::VisualLocalizationFrame,
    visual_odometry::VisualOdometer,
};

pub(super) struct PendingVisual {
    segment: i64,
    batch: fagra::BatchKey<FrameReprojections>,
    factors: Vec<fagra::FactorKey<ReprojectionObservation>>,
    frame: TimeWrapper<VisualLocalizationFrame>,
}

impl Localization {
    pub(super) fn admits_visual_innovation(
        &mut self,
        time: Time,
        frame: &VisualLocalizationFrame,
    ) -> Result<bool> {
        let Some(alignment) = self.alignment else {
            return Ok(true);
        };
        // A newly bootstrapped graph has no independent field prediction yet.
        if self.latest_visual_frame.is_none() {
            return Ok(true);
        }
        let (segment, tau) = self.segment_and_tau(time)?;
        let controls = self.ensure_segment(segment)?;
        let mut blocks: Vec<_> = controls.iter().map(|key| key.block_id()).collect();
        blocks.push(alignment.block_id());
        let covariance = match self.graph.joint_covariance(&blocks) {
            Ok(covariance) => covariance,
            Err(
                error @ (fagra::SolverError::SingularInformation
                | fagra::SolverError::InvalidInformation
                | fagra::SolverError::LinearSolveFailed
                | fagra::SolverError::Evaluation(fagra::EvaluationError::InvalidEvaluation)),
            ) => {
                tracing::warn!(?time, source = ?frame.source, %error,
                    "discarding visual frame: motion innovation covariance unavailable");
                return Ok(false);
            }
            Err(error) => return Err(error.into()),
        };
        let covariance = SMatrix::<f64, 27, 27>::from_fn(|r, c| covariance[(r, c)]);
        let sample = self.spline(controls)?.linearize()?.pose(tau)?;
        let pose = self.graph.get(alignment)?.local_to_field.to_3d().inner * sample.pose.inner;
        let admitted = bearing_innovation_agrees(
            frame,
            pose,
            sample.field_covariance(&covariance),
            self.parameters.visual_feature_noise_variance,
            self.parameters.model.min_landmark_range,
        );
        if !admitted {
            tracing::warn!(?time, source = ?frame.source, "discarding visual motion innovation");
        }
        Ok(admitted)
    }

    pub(super) fn pending_visual_cost(&self) -> Result<f64> {
        let mut cost = 0.0;
        for pending in &self.pending_visuals {
            for &key in &pending.factors {
                cost += self.graph.factor_cost(key)?;
            }
        }
        Ok(cost)
    }

    pub(super) fn trace_visual_audit(&self, stage: &str) {
        if !tracing::enabled!(tracing::Level::DEBUG) {
            return;
        }
        for pending in &self.pending_visuals {
            let time = pending.frame.time;
            let pose = self.field_pose_at(time);
            tracing::debug!(?time, stage, source = ?pending.frame.inner.source,
                features = pending.frame.inner.associations.len(),
                rms_px = ?self.frame_rms(&pending.frame), ?pose,
                "visual correction audit");
        }
    }

    pub(crate) fn field_heading_at(&self, time: Time) -> Option<Orientation2<Field, f64>> {
        Some(Orientation2::new(
            self.field_pose_at(time)?.rotation.euler_angles().2,
        ))
    }

    #[cfg(test)]
    pub(crate) fn field_velocity_at(&self, time: Time) -> Option<nalgebra::Vector3<f64>> {
        let (segment, tau) = self.segment_and_tau(time).ok()?;
        let velocity = self
            .spline(control_keys(&self.controls, segment).ok()?)
            .ok()?
            .velocity(tau)
            .ok()?;
        let alignment = self.graph.get(self.alignment?).ok()?;
        Some(alignment.local_to_field.to_3d().inner.rotation * velocity.inner)
    }

    fn field_pose_at(&self, time: Time) -> Option<nalgebra::Isometry3<f64>> {
        let (segment, tau) = self.segment_and_tau(time).ok()?;
        let pose = self
            .spline(control_keys(&self.controls, segment).ok()?)
            .ok()?
            .pose(tau)
            .ok()?;
        let alignment = self.graph.get(self.alignment?).ok()?;
        Some(alignment.local_to_field.to_3d().inner * pose.inner)
    }

    pub(crate) fn accepted_visual_rms(&self) -> Option<f64> {
        self.frame_rms(self.latest_visual_frame.as_ref()?)
    }

    pub(super) fn frame_rms(&self, frame: &TimeWrapper<VisualLocalizationFrame>) -> Option<f64> {
        let pose = self.field_pose_at(frame.time)?;
        let camera = self.graph.get(self.intrinsics).ok()?;
        let transform = frame.inner.robot_to_camera.inner.cast::<f64>() * pose.inverse();
        reprojection_rms(
            &frame.inner.associations,
            &transform,
            camera.focal_lengths.inner,
            camera.optical_center.inner,
            &self.parameters.visual,
        )
    }

    /// Validate before either publication or marginalization can retain a bad field update.
    pub(super) fn validate_heading(&self, reference: &HeadingReference) -> Result<()> {
        let (&time, _) = self
            .history
            .attitudes
            .range(..=self.latest_time)
            .next_back()
            .ok_or_else(|| eyre!("missing IMU heading"))?;
        if self.latest_time.duration_since(time) > self.parameters.timing.max_imu_gap {
            return Err(eyre!("stale IMU heading"));
        }
        for time in std::iter::once(time).chain(self.pending_visuals.iter().map(|p| p.frame.time)) {
            let expected = reference.expected(
                self.attitude_at(time)
                    .ok_or_else(|| eyre!("missing exposure IMU heading"))?,
            );
            let field = self
                .field_heading_at(time)
                .ok_or_else(|| eyre!("missing field heading"))?;
            if expected.rotation_to(field).inner.angle().abs() > self.parameters.max_heading_error {
                return Err(eyre!("field heading disagrees with propagated IMU heading"));
            }
        }
        Ok(())
    }

    pub(super) fn validate_tilt(&self) -> Result<()> {
        let Some((&time, &attitude)) = self
            .history
            .attitudes
            .range(..=self.latest_time)
            .next_back()
        else {
            return Ok(());
        };
        let (segment, tau) = self.segment_and_tau(time)?;
        let pose = self
            .spline(control_keys(&self.controls, segment)?)?
            .pose(tau)?;
        let predicted = pose.inner.rotation.inverse() * nalgebra::Vector3::z();
        let measured = attitude.inner.inverse() * nalgebra::Vector3::z();
        let error = predicted
            .cross(&measured)
            .norm()
            .atan2(predicted.dot(&measured));
        if error > self.parameters.max_tilt_error {
            return Err(eyre!("estimated tilt disagrees with IMU attitude"));
        }
        Ok(())
    }

    pub(super) fn validate_pending_visuals(&self) -> Result<()> {
        if self.pending_visuals.iter().any(|pending| {
            !valid_visual_frame(&pending.frame.inner, &self.parameters.visual)
                || !valid_visual_rms(self.frame_rms(&pending.frame), &self.parameters.visual)
        }) {
            return Err(eyre!("visual update failed pixel validation"));
        }
        Ok(())
    }

    pub(super) fn accept_visuals(&mut self) {
        self.trace_visual_audit("accepted");
        for pending in self.pending_visuals.drain(..) {
            let frame = &pending.frame;
            self.history.landmarks.insert(
                frame.time,
                frame
                    .inner
                    .associations
                    .iter()
                    .map(|a| a.field_point)
                    .collect(),
            );
            let segment = self.segment_factors.entry(pending.segment).or_default();
            segment.reprojections.push(pending.batch);
            if self
                .latest_visual_frame
                .as_ref()
                .is_none_or(|old| pending.frame.time >= old.time)
            {
                self.latest_visual_frame = Some(pending.frame);
            }
        }
    }

    pub(super) fn discard_visuals(&mut self) -> Result<()> {
        for pending in self.pending_visuals.drain(..) {
            for key in &pending.factors {
                self.graph.remove_factor(*key)?;
            }
            self.graph.remove_batch(pending.batch)?;
            self.segment_factors
                .entry(pending.segment)
                .or_default()
                .measurements -= pending.factors.len();
        }
        Ok(())
    }

    pub(crate) fn ingest_visual(
        &mut self,
        frame: TimeWrapper<VisualLocalizationFrame>,
    ) -> Result<bool> {
        self.insert_visual(frame, true)
    }

    pub(super) fn insert_visual(
        &mut self,
        frame: TimeWrapper<VisualLocalizationFrame>,
        gate_innovation: bool,
    ) -> Result<bool> {
        let time = frame.time;
        let Some((segment, tau)) = self.check_time(time, "visual localization")? else {
            return Ok(false);
        };
        let frame = frame.inner;
        if frame.epoch != self.epoch
            || frame.generation != self.generation
            || !valid_visual_frame(&frame, &self.parameters.visual)
        {
            return Ok(false);
        }
        let Some(alignment_key) = self.alignment else {
            return Ok(false);
        };
        if gate_innovation && !self.admits_visual_innovation(time, &frame)? {
            return Ok(false);
        }
        let controls = self.ensure_segment(segment)?;
        let pose = self.spline(controls)?.pose(tau)?;
        let alignment = self.graph.get(alignment_key)?;
        tracing::debug!(
            ?time,
            source = ?frame.source,
            predicted_field_pose = ?(alignment.local_to_field.to_3d().inner * pose.inner),
            associations = ?frame.associations,
            "admitting visual observations"
        );
        let field_to_camera = frame.robot_to_camera.inner.cast::<f64>()
            * pose.inner.inverse()
            * alignment.local_to_field.to_3d().inner.inverse();
        if frame.associations.iter().any(|a| {
            let p = field_to_camera * a.field_point.inner.cast::<f64>();
            !p.iter().all(|v| v.is_finite())
                || p.coords.norm() <= self.parameters.model.min_landmark_range
        }) {
            tracing::warn!(?time, "discarding visual frame with invalid landmark range");
            return Ok(false);
        }
        let batch = self
            .graph
            .add_batch(self.visual_model(&frame, controls, alignment_key, tau));
        let mut factors = Vec::with_capacity(frame.associations.len());
        for association in &frame.associations {
            factors.push(self.graph.add_factor_to(
                batch,
                ReprojectionObservation {
                    field_point: Point3::wrap(association.field_point.inner.cast()),
                    detection: Point2::wrap(association.detection.inner.cast()),
                },
            )?);
            self.segment_factors
                .entry(segment)
                .or_default()
                .measurements += 1;
        }
        self.pending_visuals.push(PendingVisual {
            segment,
            batch,
            factors,
            frame: TimeWrapper { time, inner: frame },
        });
        self.trace_visual_audit("before");
        self.commit_time(time);
        Ok(true)
    }

    fn visual_model(
        &self,
        frame: &VisualLocalizationFrame,
        controls: [fagra::StateKey<localization_fagra::variables::PoseControl>; 4],
        alignment_key: fagra::StateKey<localization_fagra::variables::FieldAlignment>,
        tau: f64,
    ) -> FrameReprojections {
        FrameReprojections {
            controls,
            alignment: alignment_key,
            intrinsics: self.intrinsics,
            duration: self.parameters.timing.trajectory_spacing.as_secs_f64(),
            tau,
            robot_to_camera: frame.robot_to_camera.inner.cast().framed_transform(),
            information_root: (f64::from(frame.camera_intrinsic.focals.x)
                * f64::from(frame.camera_intrinsic.focals.y))
            .sqrt()
                / self.parameters.visual_feature_noise_variance.sqrt(),
            huber_threshold: self.parameters.model.huber_threshold,
            min_range: self.parameters.model.min_landmark_range,
        }
    }

    pub fn ingest_visual_odometry(
        &mut self,
        sample: VisualOdometer,
        previous_camera: Option<&CameraGeometry>,
        current_camera: Option<&CameraGeometry>,
    ) -> Result<bool> {
        let Some((current_segment, _)) = self.check_time(sample.time, "visual odometry")? else {
            return Ok(false);
        };
        if self
            .history
            .latest_vo_epoch
            .is_some_and(|epoch| sample.epoch < epoch)
        {
            tracing::warn!(epoch = sample.epoch, "discarding old VO epoch");
            return Ok(false);
        }
        self.history.latest_vo_epoch = Some(sample.epoch);
        let Some(delta) = sample.delta else {
            return Ok(false);
        };
        if !delta
            .current_left_camera_to_previous_left_camera
            .to_homogeneous()
            .iter()
            .all(|v| v.is_finite())
        {
            tracing::warn!("discarding invalid VO timestamp or transform");
            return Ok(false);
        }
        let (Some(previous_camera), Some(current_camera)) = (previous_camera, current_camera)
        else {
            tracing::warn!("discarding visual odometry without endpoint camera geometry");
            return Ok(false);
        };
        let Some((previous_segment, _)) =
            self.check_time(delta.previous_time, "visual odometry")?
        else {
            return Ok(false);
        };
        if sample.time <= delta.previous_time || current_segment - previous_segment > 1 {
            tracing::warn!("discarding unsupported visual odometry interval");
            return Ok(false);
        }
        if delta.previous_time.as_nanos()
            < self
                .latest_time
                .max(sample.time)
                .as_nanos()
                .saturating_sub(self.parameters.timing.window_ns())
        {
            tracing::warn!("discarding visual odometry outside optimization window");
            return Ok(false);
        }
        let transform = previous_camera
            .robot_to_camera
            .inner
            .cast::<f64>()
            .inverse()
            * delta
                .current_left_camera_to_previous_left_camera
                .cast::<f64>()
            * current_camera.robot_to_camera.inner.cast::<f64>();
        self.accept_motion(MotionRecord::Visual {
            previous_time: delta.previous_time,
            time: sample.time,
            transform: transform.framed_transform(),
        })
    }

    pub(super) fn insert_visual_odometry(
        &mut self,
        previous_time: Time,
        time: Time,
        transform: Isometry3<Robot, Robot, f64>,
    ) -> Result<()> {
        let (previous_segment, previous_tau) = self.segment_and_tau(previous_time)?;
        let (current_segment, current_tau) = self.segment_and_tau(time)?;
        let observation = VisualOdometryObservation {
            previous_tau,
            current_tau,
            current_to_previous: transform,
        };
        let information_root = SMatrix::identity() / self.parameters.model.visual_odometry_sigma;
        if previous_segment == current_segment {
            let controls = self.ensure_segment(current_segment)?;
            let batch = *self
                .segment_factors
                .entry(current_segment)
                .or_default()
                .odometry
                .get_or_insert_with(|| {
                    self.graph.add_batch(VisualOdometry {
                        controls,
                        duration: self.parameters.timing.trajectory_spacing.as_secs_f64(),
                        information_root,
                        huber_threshold: self.parameters.model.huber_threshold,
                    })
                });
            self.graph.add_factor_to(batch, observation)?;
        } else {
            let a = self.ensure_segment(previous_segment)?;
            let b = self.ensure_segment(current_segment)?;
            self.graph.add_factor(AdjacentVisualOdometry {
                controls: [a[0], a[1], a[2], a[3], b[3]],
                duration: self.parameters.timing.trajectory_spacing.as_secs_f64(),
                observation,
                information_root,
                huber_threshold: self.parameters.model.huber_threshold,
            })?;
        }
        self.segment_factors
            .entry(previous_segment)
            .or_default()
            .measurements += 1;
        Ok(())
    }
}

fn bearing_innovation_agrees(
    frame: &VisualLocalizationFrame,
    robot_to_field: nalgebra::Isometry3<f64>,
    covariance: nalgebra::Matrix6<f64>,
    pixel_variance: f64,
    min_range: f64,
) -> bool {
    use nalgebra::{Matrix3, SMatrix};

    // Same fixed angular noise as FrameReprojections, without robust downweighting.
    let variance = pixel_variance
        / (f64::from(frame.camera_intrinsic.focals.x) * f64::from(frame.camera_intrinsic.focals.y));
    // Marginal gates avoid treating correlated landmarks as independent evidence.
    // Chi-square tail bound with a union bound over the frame, P(reject) <= exp(-9).
    let tail = 9.0 + (frame.associations.len().max(1) as f64).ln();
    let gate = 3.0 + 2.0 * (3.0 * tail).sqrt() + 2.0 * tail;
    let camera = frame.robot_to_camera.inner.cast::<f64>();
    let rotation = camera.rotation.to_rotation_matrix();
    for association in &frame.associations {
        let robot = robot_to_field.inverse() * association.field_point.inner.cast::<f64>();
        let point = camera * robot;
        let range = point.coords.norm();
        if !range.is_finite() || range <= min_range {
            return false;
        }
        let predicted = point.coords / range;
        let observed = frame
            .camera_intrinsic
            .bearing(association.detection)
            .inner
            .cast::<f64>();
        let observed = observed.normalize();
        // Angular tangent residual stays meaningful for uncertain global corrections;
        // a bearing chord would spuriously penalize its unobservable radial component.
        let cosine = predicted.dot(&observed).clamp(-1.0, 1.0);
        let tangent = observed - predicted * cosine;
        let sine = tangent.norm();
        if sine <= f64::EPSILON && cosine < 0.0 {
            return false;
        }
        let error = if sine > f64::EPSILON {
            tangent * (sine.atan2(cosine) / sine)
        } else {
            tangent
        };
        let mut inverse_pose = SMatrix::<f64, 3, 6>::zeros();
        inverse_pose
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&robot.coords.cross_matrix());
        inverse_pose
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&-Matrix3::identity());
        let projection = (Matrix3::identity() - predicted * predicted.transpose()) / range;
        let jacobian = projection * rotation.matrix() * inverse_pose;
        let innovation =
            jacobian * covariance * jacobian.transpose() + Matrix3::identity() * variance;
        let Some(cholesky) = innovation.cholesky() else {
            return false;
        };
        let squared_error = error.dot(&cholesky.solve(&error));
        if !squared_error.is_finite() || squared_error < 0.0 || squared_error > gate {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use linear_algebra::{point, vector};
    use nalgebra::{Matrix6, Translation3, UnitQuaternion};
    use projection::intrinsic::Intrinsic;
    use types::visual_localization::{FieldMarkAssociation, VisualAssociationSource};

    #[test]
    fn uncertain_motion_allows_large_global_correction() {
        let camera = nalgebra::Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_euler_angles(std::f32::consts::PI, 0.0, 0.0),
        );
        let intrinsic = Intrinsic::new(nalgebra::vector![300.0, 300.0], point![160.0, 120.0]);
        let mut frame = VisualLocalizationFrame {
            epoch: 0,
            generation: 0,
            source: VisualAssociationSource::Global,
            robot_to_camera: camera.framed_transform(),
            camera_intrinsic: intrinsic,
            associations: vec![],
        };
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)] {
            frame.associations.push(FieldMarkAssociation {
                field_point: point![x + 5.7, y, 0.0],
                detection: intrinsic.project(vector![x, -y, 0.55]),
            });
        }
        let pose = nalgebra::Isometry3::translation(0.0, 0.0, 0.55);
        assert!(!bearing_innovation_agrees(
            &frame,
            pose,
            Matrix6::identity() * 1e-6,
            1.0,
            0.01
        ));
        let mut uncertain = Matrix6::identity() * 1e-6;
        uncertain
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(nalgebra::Matrix3::identity() * 100.0));
        assert!(bearing_innovation_agrees(
            &frame, pose, uncertain, 1.0, 0.01
        ));
        uncertain[(0, 0)] = f64::NAN;
        assert!(!bearing_innovation_agrees(
            &frame, pose, uncertain, 1.0, 0.01
        ));
    }
}
