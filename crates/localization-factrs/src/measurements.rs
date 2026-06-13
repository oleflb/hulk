use std::time::SystemTime;

use booster::ImuState;
use factrs::core::SE3;
use nalgebra::{Point2, Point3};

use crate::visual_odometry_factors::Measurement as VisualOdometryMeasurement;

#[derive(Debug, Clone)]
pub enum SensorMeasurement {
    Imu(ImuMeasurement),
    Visual(Vec<VisualMeasurement>),
    VisualOdometry(VisualOdometryMeasurement),
}

impl SensorMeasurement {
    pub fn time(&self) -> SystemTime {
        match self {
            SensorMeasurement::Imu(imu) => imu.time,
            SensorMeasurement::Visual(visual) => {
                visual
                    .first()
                    .expect("visual frames must contain at least one measurement")
                    .time
            }
            SensorMeasurement::VisualOdometry(visual_odometry) => visual_odometry.timestamp,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ImuMeasurement {
    pub time: SystemTime,
    pub state: ImuState,
}

#[derive(Debug, Clone)]
pub struct VisualMeasurement {
    /// Time of the detection
    pub time: SystemTime,
    /// The detected features in image space
    pub detections: Vec<Point2<f64>>,
    /// Candidate 3d global correspondences
    pub candidates: Vec<Point3<f64>>,
    /// Transformation from the robot frame to the camera frame
    pub robot_to_camera: SE3<f64>,
    /// Optional per-frame association costs overriding the landmark factor default.
    pub association_costs: Option<LandmarkAssociationCosts>,
}

#[derive(Debug, Clone)]
pub struct VisualClassMeasurement {
    /// The detected features in image space for one semantic class.
    pub detections: Vec<Point2<f64>>,
    /// Candidate 3d global correspondences for the same semantic class.
    pub candidates: Vec<Point3<f64>>,
    /// Optional per-class association costs overriding the landmark factor default.
    pub association_costs: Option<LandmarkAssociationCosts>,
}

#[derive(Debug, Clone, Copy)]
pub struct LandmarkAssociationCosts {
    pub unmatched_landmark: f64,
    pub unmatched_detection: f64,
}
