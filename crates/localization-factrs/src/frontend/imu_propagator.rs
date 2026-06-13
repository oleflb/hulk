use std::{collections::VecDeque, time::Duration, time::SystemTime};

use factrs::{
    core::SO3,
    traits::Variable,
    variables::{MatrixLieGroup, SE23},
};
use nalgebra::Vector3;

use super::{OptimizationResult, se23_to_isometry3_and_velocity};
use crate::{
    backend::OptimizationResult as BackendOptimizationResult, measurements::ImuMeasurement,
};

const DEFAULT_BACKEND_REJECTION_POSITION_INNOVATION_METERS: f64 = 0.5;
const DEFAULT_BACKEND_REJECTION_ORIENTATION_INNOVATION_RADIANS: f64 =
    10.0 * std::f64::consts::PI / 180.0;
const DEFAULT_BACKEND_MAX_POSITION_CORRECTION_METERS: f64 = 2.0;
const DEFAULT_BACKEND_MAX_ORIENTATION_CORRECTION_RADIANS: f64 = 2.75 * std::f64::consts::PI / 180.0;
const DEFAULT_IMU_PROPAGATION_HISTORY: Duration = Duration::from_secs(1);
// The live IMU normally arrives at roughly 500 Hz. A 100 ms gap means the
// frontend missed many samples, so dead-reckoning through it is not trustworthy.
const DEFAULT_MAX_IMU_INTEGRATION_DT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy)]
pub struct FrontendConfiguration {
    /// Controls how delayed backend updates are reconciled with live IMU propagation.
    /// Set to `None` to trust backend poses directly.
    pub backend_correction: Option<BackendCorrectionConfiguration>,
    /// Retained IMU horizon used to replay propagation when delayed backend results arrive.
    pub imu_propagation_history: Duration,
    /// Largest gap that the frontend is allowed to integrate through.
    pub max_imu_integration_dt: Duration,
}

impl Default for FrontendConfiguration {
    fn default() -> Self {
        Self {
            backend_correction: Some(BackendCorrectionConfiguration::default()),
            imu_propagation_history: DEFAULT_IMU_PROPAGATION_HISTORY,
            max_imu_integration_dt: DEFAULT_MAX_IMU_INTEGRATION_DT,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BackendCorrectionConfiguration {
    /// Position innovation threshold for rejecting updates that are also orientation outliers.
    pub rejection_position_innovation: f64,
    /// Orientation innovation threshold for rejecting updates that are also position outliers.
    pub rejection_orientation_innovation: f64,
    /// Maximum position snap applied when accepting a delayed backend update.
    pub max_position_correction: f64,
    /// Maximum orientation snap applied when accepting a delayed backend update.
    pub max_orientation_correction: f64,
}

impl Default for BackendCorrectionConfiguration {
    fn default() -> Self {
        Self {
            rejection_position_innovation: DEFAULT_BACKEND_REJECTION_POSITION_INNOVATION_METERS,
            rejection_orientation_innovation:
                DEFAULT_BACKEND_REJECTION_ORIENTATION_INNOVATION_RADIANS,
            max_position_correction: DEFAULT_BACKEND_MAX_POSITION_CORRECTION_METERS,
            max_orientation_correction: DEFAULT_BACKEND_MAX_ORIENTATION_CORRECTION_RADIANS,
        }
    }
}

#[derive(Debug, Clone)]
struct PropagatedState {
    time: SystemTime,
    pose: SE23,
    last_imu: Option<ImuMeasurement>,
    backend_result: BackendOptimizationResult,
}

#[derive(Debug, Clone)]
pub(super) struct ImuPropagator {
    gravity: Vector3<f64>,
    config: FrontendConfiguration,
    imu_samples: VecDeque<ImuMeasurement>,
    state: Option<PropagatedState>,
}

impl ImuPropagator {
    pub(super) fn new(gravity: Vector3<f64>, config: FrontendConfiguration) -> Self {
        Self {
            gravity,
            config,
            imu_samples: VecDeque::new(),
            state: None,
        }
    }

    pub(super) fn push_imu(&mut self, measurement: ImuMeasurement) {
        let insertion_index = self
            .imu_samples
            .iter()
            .position(|sample| sample.time > measurement.time)
            .unwrap_or(self.imu_samples.len());
        self.imu_samples.insert(insertion_index, measurement);
    }

    pub(super) fn needs_backend_result(&self, backend_result: &BackendOptimizationResult) -> bool {
        self.state
            .as_ref()
            .is_none_or(|state| !backend_results_equal(&state.backend_result, backend_result))
    }

    pub(super) fn observe_backend_result(&mut self, backend_result: &BackendOptimizationResult) {
        if let Some(pose) = self.corrected_backend_pose(backend_result) {
            self.reset_from_backend_pose(backend_result, pose);
        } else {
            log::warn!("rejecting backend localization update inconsistent with IMU propagation");
            self.mark_backend_result_seen(backend_result);
        }
    }

    pub(super) fn propagate_to_latest_imu(&mut self) {
        let Some(state) = self.state.clone() else {
            return;
        };

        self.state = self.propagate_state_until(state, None);
        self.prune_samples_before_state();
    }

    pub(super) fn optimization_result(&self) -> Option<OptimizationResult> {
        let state = self.state.as_ref()?;
        let (transform, velocity) = se23_to_isometry3_and_velocity(state.pose.clone());
        Some(OptimizationResult {
            time: state.time,
            transform,
            velocity,
            camera_intrinsics: state.backend_result.camera_intrinsics.clone(),
        })
    }

    fn reset_from_backend_pose(&mut self, backend_result: &BackendOptimizationResult, pose: SE23) {
        let last_imu = self.latest_imu_at_or_before(backend_result.time);
        self.state = Some(PropagatedState {
            time: backend_result.time,
            pose,
            last_imu,
            backend_result: backend_result.clone(),
        });
        self.prune_samples_before_state();
    }

    fn mark_backend_result_seen(&mut self, backend_result: &BackendOptimizationResult) {
        if let Some(state) = self.state.as_mut() {
            state.backend_result = backend_result.clone();
        }
    }

    fn corrected_backend_pose(&self, backend_result: &BackendOptimizationResult) -> Option<SE23> {
        let Some(correction) = self.config.backend_correction else {
            return Some(backend_result.latest_pose.clone());
        };
        let Some(predicted_state) = self.predicted_state_at(backend_result.time) else {
            return Some(backend_result.latest_pose.clone());
        };

        if predicted_state.time != backend_result.time {
            return Some(backend_result.latest_pose.clone());
        }

        let innovation = BackendInnovation::between(&predicted_state.pose, backend_result);
        log::debug!(
            "backend localization innovation: position={:.3}m orientation={:.3}deg",
            innovation.position,
            innovation.orientation.to_degrees()
        );

        if innovation.exceeds_rejection_thresholds(correction) {
            return None;
        }

        Some(SE23::from_rot_vel_trans(
            capped_rotation_correction(
                predicted_state.pose.rot(),
                backend_result.latest_pose.rot(),
                correction.max_orientation_correction,
            ),
            corrected_velocity(
                &predicted_state.pose,
                &backend_result.latest_pose,
                correction,
            ),
            capped_position_correction(
                &predicted_state.pose,
                &backend_result.latest_pose,
                correction.max_position_correction,
            ),
        ))
    }

    fn predicted_state_at(&self, time: SystemTime) -> Option<PropagatedState> {
        let state = self.state.as_ref()?;
        if time < state.time {
            return None;
        }
        self.propagate_state_until(state.clone(), Some(time))
    }

    fn propagate_state_until(
        &self,
        mut state: PropagatedState,
        end_time: Option<SystemTime>,
    ) -> Option<PropagatedState> {
        let start_time = state.time;
        for next_imu in self
            .imu_samples
            .iter()
            .skip_while(|measurement| measurement.time <= start_time)
        {
            if end_time.is_some_and(|time| next_imu.time > time) {
                break;
            }
            propagate_state_to_imu(
                self.gravity,
                self.config.max_imu_integration_dt,
                &mut state,
                next_imu.clone(),
            )?;
        }

        Some(state)
    }

    fn latest_imu_at_or_before(&self, time: SystemTime) -> Option<ImuMeasurement> {
        self.imu_samples
            .iter()
            .take_while(|measurement| measurement.time <= time)
            .last()
            .cloned()
    }

    fn prune_samples_before_state(&mut self) {
        let Some(state_time) = self.state.as_ref().map(|state| state.time) else {
            return;
        };
        let Some(prune_before) = state_time.checked_sub(self.config.imu_propagation_history) else {
            return;
        };

        while self
            .imu_samples
            .front()
            .is_some_and(|measurement| measurement.time < prune_before)
        {
            self.imu_samples.pop_front();
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct BackendInnovation {
    position: f64,
    orientation: f64,
}

impl BackendInnovation {
    fn between(predicted_pose: &SE23, backend_result: &BackendOptimizationResult) -> Self {
        Self {
            position: (predicted_pose.xyz() - backend_result.latest_pose.xyz()).norm(),
            orientation: rotation_error_radians(
                predicted_pose.rot(),
                backend_result.latest_pose.rot(),
            ),
        }
    }

    fn exceeds_rejection_thresholds(self, config: BackendCorrectionConfiguration) -> bool {
        self.position > config.rejection_position_innovation
            && self.orientation > config.rejection_orientation_innovation
    }
}

fn propagate_state_to_imu(
    gravity: Vector3<f64>,
    max_imu_integration_dt: Duration,
    state: &mut PropagatedState,
    next_imu: ImuMeasurement,
) -> Option<()> {
    let dt_duration = next_imu
        .time
        .duration_since(state.time)
        .expect("IMU samples must be time ordered");
    if dt_duration > max_imu_integration_dt {
        return None;
    }

    let dt = dt_duration.as_secs_f64();

    if dt == 0.0 {
        state.last_imu = Some(next_imu);
        return Some(());
    }

    let previous_imu = state.last_imu.as_ref().unwrap_or(&next_imu);
    let previous_gyro = previous_imu.state.angular_velocity.inner.cast::<f64>();
    let next_gyro = next_imu.state.angular_velocity.inner.cast::<f64>();
    let average_gyro = (previous_gyro + next_gyro) * 0.5;

    let previous_rotation = state.pose.rot().clone();
    let next_rotation = previous_rotation.compose(&SO3::exp((average_gyro * dt).as_view()));

    let previous_accel = previous_imu.state.linear_acceleration.inner.cast::<f64>();
    let next_accel = next_imu.state.linear_acceleration.inner.cast::<f64>();
    let previous_accel_global = previous_rotation.apply(previous_accel.as_view()) - gravity;
    let next_accel_global = next_rotation.apply(next_accel.as_view()) - gravity;
    let average_accel_global = (previous_accel_global + next_accel_global) * 0.5;

    let next_velocity = state.pose.uvw() + average_accel_global * dt;
    let next_position =
        state.pose.xyz() + state.pose.uvw() * dt + average_accel_global * (0.5 * dt * dt);

    state.pose = SE23::from_rot_vel_trans(next_rotation, next_velocity, next_position);
    state.time = next_imu.time;
    state.last_imu = Some(next_imu);
    Some(())
}

pub(super) fn rotation_error_radians(left: &SO3, right: &SO3) -> f64 {
    left.inverse().compose(right).log().norm()
}

fn capped_rotation_correction(predicted: &SO3, backend: &SO3, max_correction: f64) -> SO3 {
    let correction = predicted.inverse().compose(backend).log();
    let correction_norm = correction.norm();
    if correction_norm <= max_correction {
        return backend.clone();
    }

    predicted.compose(&SO3::exp(
        (correction * (max_correction / correction_norm)).as_view(),
    ))
}

fn corrected_velocity(
    predicted: &SE23,
    backend: &SE23,
    config: BackendCorrectionConfiguration,
) -> nalgebra::Vector3<f64> {
    if position_correction_norm(predicted, backend) <= config.max_position_correction {
        backend.uvw().into_owned()
    } else {
        predicted.uvw().into_owned()
    }
}

fn capped_position_correction(
    predicted: &SE23,
    backend: &SE23,
    max_correction: f64,
) -> nalgebra::Vector3<f64> {
    let correction = backend.xyz() - predicted.xyz();
    let correction_norm = correction.norm();
    if correction_norm <= max_correction {
        return backend.xyz().into_owned();
    }

    predicted.xyz() + correction * (max_correction / correction_norm)
}

fn position_correction_norm(predicted: &SE23, backend: &SE23) -> f64 {
    (backend.xyz() - predicted.xyz()).norm()
}

fn backend_results_equal(
    left: &BackendOptimizationResult,
    right: &BackendOptimizationResult,
) -> bool {
    left.time == right.time
        && se23_equal(&left.latest_pose, &right.latest_pose)
        && camera_intrinsics_equal(&left.camera_intrinsics, &right.camera_intrinsics)
}

fn camera_intrinsics_equal(
    left: &crate::camera_intrinsics::CameraIntrinsics<f64>,
    right: &crate::camera_intrinsics::CameraIntrinsics<f64>,
) -> bool {
    left.focals()
        .iter()
        .zip(right.focals().iter())
        .all(|(left, right)| left == right)
        && left
            .optical_center()
            .iter()
            .zip(right.optical_center().iter())
            .all(|(left, right)| left == right)
}

fn se23_equal(left: &SE23, right: &SE23) -> bool {
    left.rot().w() == right.rot().w()
        && left.rot().x() == right.rot().x()
        && left.rot().y() == right.rot().y()
        && left.rot().z() == right.rot().z()
        && left
            .uvw()
            .iter()
            .zip(right.uvw().iter())
            .all(|(left, right)| left == right)
        && left
            .xyz()
            .iter()
            .zip(right.xyz().iter())
            .all(|(left, right)| left == right)
}
