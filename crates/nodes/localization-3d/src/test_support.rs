use std::time::Duration;

use linear_algebra::{IntoTransform, point};
use projection::intrinsic::Intrinsic;
use types::camera_geometry::CameraGeometry;

use crate::Localization3dParameters;

pub(crate) fn parameters() -> Localization3dParameters {
    Localization3dParameters {
        timing: Default::default(),
        model: Default::default(),
        solver: Default::default(),
        visual: Default::default(),
        inputs: Default::default(),
        imu_preintegration: Default::default(),
        imu_bias: Default::default(),
        kinematic_odometry_noise: Some(Default::default()),
        accelerometer: None,
        initial_height_sigma: 1.0,
        initial_velocity_sigma: 5.0,
        recovery_height_gate: 9.0,
        max_tilt_error: 20.0_f64.to_radians(),
        accelerometer_process_noise_variance: 10.0,
        visual_feature_noise_variance: 100.0,
        field_containment_sigma: 1.0,
        max_heading_error: 20.0_f64.to_radians(),
        max_heading_reference_drift_per_second: 0.5_f64.to_radians(),
        tracking_timeout: Duration::from_secs(2),
        visual_tracking_timeout: Duration::from_secs(2),
    }
}

pub(crate) fn camera() -> CameraGeometry {
    CameraGeometry {
        robot_to_camera: nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(0.12, -0.04, 0.2),
            nalgebra::UnitQuaternion::from_euler_angles(std::f32::consts::PI, 0.0, 0.0),
        )
        .inverse()
        .framed_transform(),
        intrinsics: Intrinsic::new(nalgebra::vector![300.0, 300.0], point![160.0, 120.0]),
    }
}
