use std::time::SystemTime;

use booster::ImuState;
use factrs::{
    core::{SE3, SO3},
    traits::Variable,
    variables::{MatrixLieGroup, SE23},
};
use nalgebra::{Point2, Point3};
use thiserror::Error;
use tokio::sync::{mpsc::UnboundedSender, watch};

use crate::backend::OptimizationResult as BackendOptimizationResult;
use crate::camera_intrinsics::CameraIntrinsics;
use crate::foot_above_ground_factor::FootHeightMeasurement;
use crate::measurements::{
    ImuMeasurement, LandmarkAssociationCosts, SensorMeasurement, VisualClassMeasurement,
    VisualMeasurement,
};
use crate::visual_odometry_factors::VisualOdometrySample;

pub struct VinsFrontend {
    measurement_sender: UnboundedSender<SensorMeasurement>,
    result_receiver: watch::Receiver<Option<BackendOptimizationResult>>,
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
    ) -> Self {
        Self {
            measurement_sender,
            result_receiver,
        }
    }

    pub async fn wait_for_optimization_result(&mut self) -> Result<(), VinsFrontendError> {
        self.result_receiver
            .changed()
            .await
            .map_err(|_| VinsFrontendError::BackendDisconnected)
    }

    pub fn last_optimization_result(&mut self) -> Option<OptimizationResult> {
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
            .send(SensorMeasurement::Imu(measurement))
            .map_err(|_| VinsFrontendError::BackendDisconnected)
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

    /// Adds an accumulated visual odometry pose to the optimization pipeline.
    pub fn ingest_visual_odometry(
        &mut self,
        time: SystemTime,
        robot_to_left_camera: nalgebra::Isometry3<f32>,
        odometer: nalgebra::Isometry3<f32>,
    ) -> Result<(), VinsFrontendError> {
        let measurement = VisualOdometrySample {
            robot_to_left_camera: isometry3_to_se3(robot_to_left_camera),
            odometer: isometry3_to_se3(odometer),
            timestamp: time,
        };

        self.measurement_sender
            .send(SensorMeasurement::VisualOdometry(measurement))
            .map_err(|_| VinsFrontendError::BackendDisconnected)
    }

    /// Adds sole positions to keep the estimated feet above the ground plane.
    pub fn ingest_foot_heights(
        &mut self,
        time: SystemTime,
        left_sole_in_robot: Point3<f64>,
        right_sole_in_robot: Point3<f64>,
    ) -> Result<(), VinsFrontendError> {
        let measurement = FootHeightMeasurement {
            time,
            left_sole_in_robot,
            right_sole_in_robot,
        };

        self.measurement_sender
            .send(SensorMeasurement::FootHeights(measurement))
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
