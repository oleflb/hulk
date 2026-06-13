use std::time::SystemTime;

use factrs::{
    core::SE3,
    linalg::{ForwardProp, Numeric, VectorX},
    traits::{Residual, Variable},
    variables::SE23,
};
use nalgebra::{SMatrix, SVector};

use crate::{
    SE23Spline,
    utils::{interval_dt, tau},
};

#[derive(Debug, Clone)]
pub struct Measurement {
    /// Transformation from the robot frame to the left camera frame.
    pub robot_to_left_camera: SE3,
    /// Accumulated visual odometry pose from the current left camera frame to
    /// the visual odometer frame.
    pub odometer: SE3,
    pub timestamp: SystemTime,
}

#[derive(Debug, Clone)]
pub struct VisualOdometryFactor {
    measurements: Vec<DeltaMeasurement>,
    last_measurement: Option<Measurement>,
    information_root: SMatrix<f64, 6, 6>,
    start_time: SystemTime,
    end_time: SystemTime,
    duration: f64,
}

#[derive(Debug, Clone)]
struct DeltaMeasurement {
    start_tau: f64,
    end_tau: f64,
    robot_delta: SE3,
}

#[factrs::mark]
impl Residual for VisualOdometryFactor {
    type Input = (SE23, SE23);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        self.measurements.len() * 6
    }

    fn residual<T: Numeric>(&self, (start, end): (SE23<T>, SE23<T>)) -> VectorX<T> {
        self.residuals_on_spline(start, end)
    }
}

impl VisualOdometryFactor {
    pub fn new(
        measurements: Vec<Measurement>,
        visual_odometry_noise: SMatrix<f64, 6, 6>,
        start_time: SystemTime,
        end_time: SystemTime,
    ) -> Self {
        let information_root = visual_odometry_noise
            .cholesky()
            .expect("visual odometry covariance must be positive definite")
            .l()
            .try_inverse()
            .expect("visual odometry covariance Cholesky factor must be invertible");

        let duration = interval_dt::<f64>(start_time, end_time);
        let last_measurement = measurements.last().cloned();
        let measurements = delta_measurements(measurements, start_time, end_time);

        Self {
            measurements,
            last_measurement,
            information_root,
            start_time,
            end_time,
            duration,
        }
    }

    pub fn extend_measurements(&mut self, measurements: impl IntoIterator<Item = Measurement>) {
        for measurement in measurements {
            if let Some(previous) = &self.last_measurement {
                self.measurements.push(DeltaMeasurement::new(
                    previous,
                    &measurement,
                    self.start_time,
                    self.end_time,
                ));
            }
            self.last_measurement = Some(measurement);
        }
    }

    fn residuals_on_spline<T: Numeric>(&self, start: SE23<T>, end: SE23<T>) -> VectorX<T> {
        let mut residuals = VectorX::<T>::zeros(self.dim_out());
        if self.measurements.is_empty() {
            return residuals;
        }

        let spline = SE23Spline::new(start, end, T::from(self.duration));
        let information_root = self.information_root.cast::<T>();

        for (index, measurement) in self.measurements.iter().enumerate() {
            let start_pose = se23_pose_to_se3(spline.evaluate(T::from(measurement.start_tau)));
            let end_pose = se23_pose_to_se3(spline.evaluate(T::from(measurement.end_tau)));

            let predicted_delta = start_pose.inverse().compose(&end_pose);
            let measured_delta = measurement.robot_delta.cast::<T>();

            let raw_error = predicted_delta.ominus(&measured_delta);
            let raw_error = SVector::<T, 6>::from_column_slice(raw_error.as_slice());
            let whitened_error = information_root * raw_error;

            residuals
                .fixed_view_mut::<6, 1>(index * 6, 0)
                .copy_from(&whitened_error);
        }

        residuals
    }
}

impl DeltaMeasurement {
    fn new(
        previous: &Measurement,
        current: &Measurement,
        start_time: SystemTime,
        end_time: SystemTime,
    ) -> Self {
        let camera_delta = previous.odometer.inverse().compose(&current.odometer);
        let robot_delta = previous
            .robot_to_left_camera
            .inverse()
            .compose(&camera_delta)
            .compose(&current.robot_to_left_camera);

        Self {
            start_tau: tau::<f64>(start_time, end_time, previous.timestamp),
            end_tau: tau::<f64>(start_time, end_time, current.timestamp),
            robot_delta,
        }
    }
}

fn delta_measurements(
    measurements: Vec<Measurement>,
    start_time: SystemTime,
    end_time: SystemTime,
) -> Vec<DeltaMeasurement> {
    measurements
        .windows(2)
        .map(|pair| DeltaMeasurement::new(&pair[0], &pair[1], start_time, end_time))
        .collect()
}

fn se23_pose_to_se3<T: Numeric>(pose: SE23<T>) -> SE3<T> {
    SE3::from_rot_trans(pose.rot().clone(), pose.xyz().into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use factrs::core::{SO3, Vector3};
    use nalgebra::vector;
    use std::time::Duration;

    fn state(position: Vector3, velocity: Vector3) -> SE23 {
        SE23::from_rot_vel_trans(SO3::identity(), velocity, position)
    }

    fn state_with_rotation(rotation: SO3, position: Vector3, velocity: Vector3) -> SE23 {
        SE23::from_rot_vel_trans(rotation, velocity, position)
    }

    fn yaw_90() -> SO3 {
        SO3::from_xyzw(
            0.0,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        )
    }

    fn translation(x: f64, y: f64, z: f64) -> SE3 {
        SE3::from_rot_trans(SO3::identity(), vector![x, y, z])
    }

    fn transform(rotation: SO3, x: f64, y: f64, z: f64) -> SE3 {
        SE3::from_rot_trans(rotation, vector![x, y, z])
    }

    fn measurement(timestamp: SystemTime, robot_to_left_camera: SE3, odometer: SE3) -> Measurement {
        Measurement {
            robot_to_left_camera,
            odometer,
            timestamp,
        }
    }

    #[test]
    fn single_measurement_has_empty_residual() {
        let start_time = SystemTime::UNIX_EPOCH;
        let factor = VisualOdometryFactor::new(
            vec![measurement(start_time, SE3::identity(), SE3::identity())],
            SMatrix::<f64, 6, 6>::identity(),
            start_time,
            start_time + Duration::from_secs(1),
        );

        let residual = factor.residuals_on_spline(
            state(Vector3::zeros(), Vector3::zeros()),
            state(Vector3::zeros(), Vector3::zeros()),
        );

        assert_eq!(residual.len(), 0);
    }

    #[test]
    fn residual_is_zero_for_matching_camera_motion() {
        let start_time = SystemTime::UNIX_EPOCH;
        let factor = VisualOdometryFactor::new(
            vec![
                measurement(start_time, SE3::identity(), SE3::identity()),
                measurement(
                    start_time + Duration::from_secs(1),
                    SE3::identity(),
                    translation(1.0, 0.0, 0.0),
                ),
            ],
            SMatrix::<f64, 6, 6>::identity(),
            start_time,
            start_time + Duration::from_secs(1),
        );

        let residual = factor.residuals_on_spline(
            state(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            state(vector![1.0, 0.0, 0.0], vector![1.0, 0.0, 0.0]),
        );

        assert!(
            residual.iter().all(|value| value.abs() < 1e-9),
            "expected zero residual, got {residual:?}"
        );
    }

    #[test]
    fn residual_uses_robot_to_left_camera_extrinsics() {
        let start_time = SystemTime::UNIX_EPOCH;
        let robot_to_left_camera = translation(1.0, 0.0, 0.0);
        let factor = VisualOdometryFactor::new(
            vec![
                measurement(start_time, robot_to_left_camera.clone(), SE3::identity()),
                measurement(
                    start_time + Duration::from_secs(1),
                    robot_to_left_camera,
                    transform(yaw_90(), 1.0, -1.0, 0.0),
                ),
            ],
            SMatrix::<f64, 6, 6>::identity(),
            start_time,
            start_time + Duration::from_secs(1),
        );

        let residual = factor.residuals_on_spline(
            state(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            state_with_rotation(yaw_90(), Vector3::zeros(), vector![1.0, 0.0, 0.0]),
        );

        assert!(
            residual.iter().all(|value| value.abs() < 1e-9),
            "expected zero residual, got {residual:?}"
        );
    }

    #[test]
    fn residual_is_nonzero_for_mismatching_odometer_motion() {
        let start_time = SystemTime::UNIX_EPOCH;
        let factor = VisualOdometryFactor::new(
            vec![
                measurement(start_time, SE3::identity(), SE3::identity()),
                measurement(
                    start_time + Duration::from_secs(1),
                    SE3::identity(),
                    translation(0.5, 0.0, 0.0),
                ),
            ],
            SMatrix::<f64, 6, 6>::identity(),
            start_time,
            start_time + Duration::from_secs(1),
        );

        let residual = factor.residuals_on_spline(
            state(Vector3::zeros(), vector![1.0, 0.0, 0.0]),
            state(vector![1.0, 0.0, 0.0], vector![1.0, 0.0, 0.0]),
        );

        assert!(
            residual.norm() > 0.1,
            "expected nonzero residual, got {residual:?}"
        );
    }
}
