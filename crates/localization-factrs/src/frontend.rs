use std::time::SystemTime;

use booster::ImuState;
use factrs::{
    core::{SE3, SO3},
    traits::Variable,
    variables::{MatrixLieGroup, SE23},
};
use nalgebra::{Point2, Point3, Vector3};
use thiserror::Error;
use tokio::sync::{mpsc::UnboundedSender, watch};

mod imu_propagator;

pub use imu_propagator::{BackendCorrectionConfiguration, FrontendConfiguration};

use crate::backend::OptimizationResult as BackendOptimizationResult;
use crate::camera_intrinsics::CameraIntrinsics;
use crate::measurements::{
    ImuMeasurement, LandmarkAssociationCosts, SensorMeasurement, VisualClassMeasurement,
    VisualMeasurement,
};
use imu_propagator::ImuPropagator;

pub struct VinsFrontend {
    measurement_sender: UnboundedSender<SensorMeasurement>,
    result_receiver: watch::Receiver<Option<BackendOptimizationResult>>,
    imu_propagator: ImuPropagator,
}

#[derive(Debug, Clone)]
pub struct OptimizationResult {
    pub time: SystemTime,
    pub transform: nalgebra::Isometry3<f64>,
    pub velocity: nalgebra::Vector3<f64>,
    pub camera_intrinsics: CameraIntrinsics<f64>,
}

impl VinsFrontend {
    pub fn new(
        measurement_sender: UnboundedSender<SensorMeasurement>,
        result_receiver: watch::Receiver<Option<BackendOptimizationResult>>,
        gravity: Vector3<f64>,
    ) -> Self {
        Self::with_config(
            measurement_sender,
            result_receiver,
            gravity,
            FrontendConfiguration::default(),
        )
    }

    pub fn with_config(
        measurement_sender: UnboundedSender<SensorMeasurement>,
        result_receiver: watch::Receiver<Option<BackendOptimizationResult>>,
        gravity: Vector3<f64>,
        frontend_config: FrontendConfiguration,
    ) -> Self {
        Self {
            measurement_sender,
            result_receiver,
            imu_propagator: ImuPropagator::new(gravity, frontend_config),
        }
    }

    pub async fn wait_for_optimization_result(&mut self) -> Result<(), VinsFrontendError> {
        self.result_receiver
            .changed()
            .await
            .map_err(|_| VinsFrontendError::BackendDisconnected)?;

        if let Some(backend_result) = self.result_receiver.borrow_and_update().clone() {
            self.imu_propagator.observe_backend_result(&backend_result);
            self.imu_propagator.propagate_to_latest_imu();
        }

        Ok(())
    }

    pub async fn wait_for_backend_optimization_result(&mut self) -> Result<(), VinsFrontendError> {
        self.result_receiver
            .changed()
            .await
            .map_err(|_| VinsFrontendError::BackendDisconnected)
    }

    pub fn last_optimization_result(&mut self) -> Option<OptimizationResult> {
        let backend_result_changed = self.result_receiver.has_changed().unwrap_or(false);
        let backend_result = self.result_receiver.borrow_and_update().clone()?;
        if backend_result_changed || self.imu_propagator.needs_backend_result(&backend_result) {
            self.imu_propagator.observe_backend_result(&backend_result);
        }
        self.imu_propagator.propagate_to_latest_imu();
        self.imu_propagator.optimization_result()
    }

    pub fn last_backend_optimization_result(&mut self) -> Option<OptimizationResult> {
        let backend_result = self.result_receiver.borrow_and_update().clone()?;
        Some(optimization_result_from_backend_result(backend_result))
    }

    /// Adds an IMU measurement to the optimization pipeline.
    pub fn ingest_imu(
        &mut self,
        time: SystemTime,
        state: ImuState,
    ) -> Result<(), VinsFrontendError> {
        let measurement = ImuMeasurement { time, state };
        self.measurement_sender
            .send(SensorMeasurement::Imu(measurement.clone()))
            .map_err(|_| VinsFrontendError::BackendDisconnected)?;
        self.imu_propagator.push_imu(measurement);
        Ok(())
    }

    /// Adds a visual measurement to the optimization pipeline.
    pub fn ingest_visual(
        &mut self,
        time: SystemTime,
        detections: Vec<Point2<f64>>,
        candidates: Vec<Point3<f64>>,
        robot_to_camera: nalgebra::Isometry3<f32>,
    ) -> Result<(), VinsFrontendError> {
        self.ingest_visual_with_association_costs(
            time,
            detections,
            candidates,
            robot_to_camera,
            None,
        )
    }

    /// Adds a visual measurement with per-frame association costs.
    pub fn ingest_visual_with_association_costs(
        &mut self,
        time: SystemTime,
        detections: Vec<Point2<f64>>,
        candidates: Vec<Point3<f64>>,
        robot_to_camera: nalgebra::Isometry3<f32>,
        association_costs: Option<LandmarkAssociationCosts>,
    ) -> Result<(), VinsFrontendError> {
        self.ingest_visual_classes(
            time,
            vec![VisualClassMeasurement {
                detections,
                candidates,
                association_costs,
            }],
            robot_to_camera,
        )
    }

    /// Adds visual measurements for multiple semantic classes in one image frame.
    pub fn ingest_visual_classes(
        &mut self,
        time: SystemTime,
        classes: Vec<VisualClassMeasurement>,
        robot_to_camera: nalgebra::Isometry3<f32>,
    ) -> Result<(), VinsFrontendError> {
        let robot_to_camera = isometry3_to_se3(robot_to_camera);
        let measurements = classes
            .into_iter()
            .filter(|class| !class.detections.is_empty() && !class.candidates.is_empty())
            .map(|class| VisualMeasurement {
                time,
                detections: class.detections,
                candidates: class.candidates,
                robot_to_camera: robot_to_camera.clone(),
                association_costs: class.association_costs,
            })
            .collect::<Vec<_>>();

        if measurements.is_empty() {
            return Ok(());
        }

        self.measurement_sender
            .send(SensorMeasurement::Visual(measurements))
            .map_err(|_| VinsFrontendError::BackendDisconnected)
    }
}

#[derive(Debug, Error)]
pub enum VinsFrontendError {
    #[error("the localization backend is disconnected")]
    BackendDisconnected,
}

fn isometry3_to_se3(isometry: nalgebra::Isometry3<f32>) -> SE3 {
    let rotation = isometry.rotation.quaternion();
    let translation = isometry.translation.vector.cast::<f64>();

    SE3::from_rot_trans(
        SO3::from_xyzw(
            rotation.i as f64,
            rotation.j as f64,
            rotation.k as f64,
            rotation.w as f64,
        ),
        translation,
    )
}

fn optimization_result_from_backend_result(
    backend_result: BackendOptimizationResult,
) -> OptimizationResult {
    let (transform, velocity) = se23_to_isometry3_and_velocity(backend_result.latest_pose);
    OptimizationResult {
        time: backend_result.time,
        transform,
        velocity,
        camera_intrinsics: backend_result.camera_intrinsics,
    }
}

pub(crate) fn se23_to_isometry3_and_velocity(
    se23: SE23,
) -> (nalgebra::Isometry3<f64>, nalgebra::Vector3<f64>) {
    let rotation = se23.rot();
    let local_velocity = rotation.inverse().apply(se23.uvw());

    let isometry = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::from(se23.xyz().into_owned()),
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            rotation.w(),
            rotation.x(),
            rotation.y(),
            rotation.z(),
        )),
    );

    (isometry, local_velocity)
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, SystemTime};

    use factrs::{traits::Variable, variables::SE23};
    use linear_algebra::IntoFramed;
    use nalgebra::{Vector3, vector};
    use tokio::sync::{mpsc, watch};

    use super::imu_propagator::rotation_error_radians;
    use super::*;

    fn backend_result(time: SystemTime, latest_pose: SE23) -> BackendOptimizationResult {
        BackendOptimizationResult {
            time,
            latest_pose,
            camera_intrinsics: CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
        }
    }

    fn frontend_with_backend_result(
        time: SystemTime,
        pose: SE23,
        gravity: Vector3<f64>,
    ) -> (
        VinsFrontend,
        watch::Sender<Option<BackendOptimizationResult>>,
        mpsc::UnboundedReceiver<SensorMeasurement>,
    ) {
        let (measurement_sender, measurement_receiver) = mpsc::unbounded_channel();
        let (result_sender, result_receiver) = watch::channel(Some(backend_result(time, pose)));

        (
            VinsFrontend::with_config(
                measurement_sender,
                result_receiver,
                gravity,
                FrontendConfiguration {
                    max_imu_integration_dt: Duration::from_secs(10),
                    ..FrontendConfiguration::default()
                },
            ),
            result_sender,
            measurement_receiver,
        )
    }

    fn imu(angular_velocity: Vector3<f32>, linear_acceleration: Vector3<f32>) -> ImuState {
        ImuState {
            roll_pitch_yaw: Vector3::zeros().framed(),
            angular_velocity: angular_velocity.framed(),
            linear_acceleration: linear_acceleration.framed(),
        }
    }

    #[test]
    fn large_imu_gap_invalidates_frontend_until_backend_reset() {
        let start = SystemTime::UNIX_EPOCH;
        let gap_time = start + Duration::from_millis(11);
        let (measurement_sender, _measurement_receiver) = mpsc::unbounded_channel();
        let (result_sender, result_receiver) =
            watch::channel(Some(backend_result(start, SE23::identity())));
        let mut frontend = VinsFrontend::with_config(
            measurement_sender,
            result_receiver,
            Vector3::zeros(),
            FrontendConfiguration {
                max_imu_integration_dt: Duration::from_millis(10),
                ..FrontendConfiguration::default()
            },
        );

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), Vector3::zeros()))
            .expect("IMU send should succeed");
        assert!(frontend.last_optimization_result().is_some());

        frontend
            .ingest_imu(gap_time, imu(Vector3::zeros(), Vector3::zeros()))
            .expect("IMU send should succeed");
        assert!(frontend.last_optimization_result().is_none());

        let backend_pose =
            SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), vector![1.0, 2.0, 3.0]);
        result_sender
            .send(Some(backend_result(gap_time, backend_pose)))
            .expect("backend result should send");

        let result = frontend
            .last_optimization_result()
            .expect("new backend result should reset frontend after gap");
        assert_eq!(result.time, gap_time);
        assert_eq!(result.transform.translation.vector, vector![1.0, 2.0, 3.0]);
    }

    #[test]
    fn stationary_imu_keeps_latest_pose_fixed() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, _result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), vector![0.0, 0.0, 9.81]);

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![0.0, 0.0, 9.81]))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![0.0, 0.0, 9.81]),
            )
            .expect("IMU send should succeed");

        let result = frontend
            .last_optimization_result()
            .expect("backend result should be available");

        assert_eq!(result.time, start + Duration::from_secs(1));
        assert!(result.transform.translation.vector.norm() < 1.0e-6);
        assert!(result.velocity.norm() < 1.0e-6);
    }

    #[test]
    fn constant_acceleration_propagates_position_and_velocity() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, _result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            )
            .expect("IMU send should succeed");

        let result = frontend
            .last_optimization_result()
            .expect("backend result should be available");

        assert_eq!(result.time, start + Duration::from_secs(1));
        assert!((result.transform.translation.vector.x - 0.5).abs() < 1.0e-9);
        assert!(result.transform.translation.vector.y.abs() < 1.0e-9);
        assert!(result.transform.translation.vector.z.abs() < 1.0e-9);
        assert!((result.velocity.x - 1.0).abs() < 1.0e-9);
        assert!(result.velocity.y.abs() < 1.0e-9);
        assert!(result.velocity.z.abs() < 1.0e-9);
    }

    #[test]
    fn backend_result_resets_propagated_state() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            )
            .expect("IMU send should succeed");

        let propagated = frontend
            .last_optimization_result()
            .expect("backend result should be available");
        assert!((propagated.transform.translation.vector.x - 0.5).abs() < 1.0e-9);

        let corrected_pose =
            SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), vector![0.25, 0.0, 0.0]);
        result_sender
            .send(Some(backend_result(
                start + Duration::from_secs(1),
                corrected_pose,
            )))
            .expect("backend result should send");

        let corrected = frontend
            .last_optimization_result()
            .expect("corrected backend result should be available");

        assert_eq!(corrected.time, start + Duration::from_secs(1));
        assert!((corrected.transform.translation.vector.x - 0.25).abs() < 1.0e-9);
        assert!(corrected.velocity.norm() < 1.0e-9);
    }

    #[tokio::test]
    async fn waited_backend_result_resets_propagated_state() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            )
            .expect("IMU send should succeed");

        let propagated = frontend
            .last_optimization_result()
            .expect("backend result should be available");
        assert!((propagated.transform.translation.vector.x - 0.5).abs() < 1.0e-9);

        let corrected_pose =
            SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), vector![0.25, 0.0, 0.0]);
        result_sender
            .send(Some(backend_result(
                start + Duration::from_secs(1),
                corrected_pose,
            )))
            .expect("backend result should send");

        frontend
            .wait_for_optimization_result()
            .await
            .expect("backend result should be observed");
        let corrected = frontend
            .last_optimization_result()
            .expect("corrected backend result should be available");

        assert_eq!(corrected.time, start + Duration::from_secs(1));
        assert!((corrected.transform.translation.vector.x - 0.25).abs() < 1.0e-9);
        assert!(corrected.velocity.norm() < 1.0e-9);
    }

    #[tokio::test]
    async fn last_result_uses_current_backend_result_after_seen_watch_update() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            )
            .expect("IMU send should succeed");

        let propagated = frontend
            .last_optimization_result()
            .expect("backend result should be available");
        assert!((propagated.transform.translation.vector.x - 0.5).abs() < 1.0e-9);

        let corrected_pose =
            SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), vector![0.25, 0.0, 0.0]);
        result_sender
            .send(Some(backend_result(
                start + Duration::from_secs(1),
                corrected_pose,
            )))
            .expect("backend result should send");
        frontend
            .result_receiver
            .changed()
            .await
            .expect("backend result should be observed");

        let corrected = frontend
            .last_optimization_result()
            .expect("corrected backend result should be available");

        assert_eq!(corrected.time, start + Duration::from_secs(1));
        assert!((corrected.transform.translation.vector.x - 0.25).abs() < 1.0e-9);
        assert!(corrected.velocity.norm() < 1.0e-9);
    }

    #[test]
    fn implausible_backend_result_is_rejected() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            )
            .expect("IMU send should succeed");

        let propagated = frontend
            .last_optimization_result()
            .expect("backend result should be available");
        assert!((propagated.transform.translation.vector.x - 0.5).abs() < 1.0e-9);

        let implausible_pose = SE23::from_rot_vel_trans(
            SO3::exp(vector![0.0, 0.0, 20.0_f64.to_radians()].as_view()),
            Vector3::zeros(),
            vector![10.0, 0.0, 0.0],
        );
        result_sender
            .send(Some(backend_result(
                start + Duration::from_secs(1),
                implausible_pose,
            )))
            .expect("backend result should send");

        let result = frontend
            .last_optimization_result()
            .expect("localization result should remain available");

        assert_eq!(result.time, start + Duration::from_secs(1));
        assert!((result.transform.translation.vector.x - 0.5).abs() < 1.0e-9);
        assert!((result.velocity.x - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn large_backend_position_correction_is_capped() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            )
            .expect("IMU send should succeed");

        let propagated = frontend
            .last_optimization_result()
            .expect("backend result should be available");
        assert!((propagated.transform.translation.vector.x - 0.5).abs() < 1.0e-9);

        let position_outlier =
            SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), vector![10.0, 0.0, 0.0]);
        result_sender
            .send(Some(backend_result(
                start + Duration::from_secs(1),
                position_outlier,
            )))
            .expect("backend result should send");

        let result = frontend
            .last_optimization_result()
            .expect("localization result should remain available");
        let max_position_correction =
            BackendCorrectionConfiguration::default().max_position_correction;

        assert_eq!(result.time, start + Duration::from_secs(1));
        assert!(
            (result.transform.translation.vector.x - (0.5 + max_position_correction)).abs()
                < 1.0e-9
        );
        assert!((result.velocity.x - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn large_backend_rotation_correction_is_capped() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(start, imu(Vector3::zeros(), Vector3::zeros()))
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), Vector3::zeros()),
            )
            .expect("IMU send should succeed");

        frontend
            .last_optimization_result()
            .expect("backend result should be available");

        let backend_rotation = SO3::exp(vector![0.0, 0.0, 10.0_f64.to_radians()].as_view());
        let backend_pose =
            SE23::from_rot_vel_trans(backend_rotation, Vector3::zeros(), vector![0.1, 0.0, 0.0]);
        result_sender
            .send(Some(backend_result(
                start + Duration::from_secs(1),
                backend_pose,
            )))
            .expect("backend result should send");

        let result = frontend
            .last_optimization_result()
            .expect("localization result should remain available");
        let capped_rotation = SO3::from_xyzw(
            result.transform.rotation.i,
            result.transform.rotation.j,
            result.transform.rotation.k,
            result.transform.rotation.w,
        );

        assert!((result.transform.translation.vector.x - 0.1).abs() < 1.0e-9);
        assert!(
            (rotation_error_radians(&SO3::identity(), &capped_rotation)
                - BackendCorrectionConfiguration::default().max_orientation_correction)
                .abs()
                < 1.0e-9
        );
    }

    #[test]
    fn out_of_order_imu_samples_are_propagated_by_timestamp() {
        let start = SystemTime::UNIX_EPOCH;
        let (mut frontend, _result_sender, _measurement_receiver) =
            frontend_with_backend_result(start, SE23::identity(), Vector3::zeros());

        frontend
            .ingest_imu(
                start + Duration::from_secs(1),
                imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            )
            .expect("IMU send should succeed");
        frontend
            .ingest_imu(start, imu(Vector3::zeros(), vector![1.0, 0.0, 0.0]))
            .expect("IMU send should succeed");

        let result = frontend
            .last_optimization_result()
            .expect("backend result should be available");

        assert_eq!(result.time, start + Duration::from_secs(1));
        assert!((result.transform.translation.vector.x - 0.5).abs() < 1.0e-9);
        assert!((result.velocity.x - 1.0).abs() < 1.0e-9);
    }
}
