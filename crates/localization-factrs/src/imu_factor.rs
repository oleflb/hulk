use std::time::SystemTime;

use factrs::{
    core::{SO3, Vector2, Vector3},
    linalg::{ForwardProp, Matrix3, Numeric, VectorX},
    traits::Residual,
    variables::SE23,
};

use crate::{
    SE23Spline,
    measurements::ImuMeasurement,
    utils::{interval_dt, tau},
};

#[derive(Debug, Clone)]
pub struct IntervalGaussianProcessImuFactor {
    measurements: Vec<ImuMeasurement>,
    measurement_taus: Vec<f64>,
    roll_pitch_yaw_information_root: Matrix3<f64>,
    start_time: SystemTime,
    end_time: SystemTime,
    duration: f64,
}

#[factrs::mark]
impl Residual for IntervalGaussianProcessImuFactor {
    type Input = (SE23, SE23);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        self.measurements.len() * 2 + self.measurements.len().saturating_sub(1)
    }

    fn residual<T: Numeric>(&self, (start, end): (SE23<T>, SE23<T>)) -> VectorX<T> {
        self.residuals_on_spline(start, end)
    }
}

impl IntervalGaussianProcessImuFactor {
    pub fn new(
        measurements: Vec<ImuMeasurement>,
        roll_pitch_yaw_noise: Matrix3<f64>,
        start_time: SystemTime,
        end_time: SystemTime,
    ) -> Self {
        // The inverse of the lower Cholesky factor is required to whiten the residuals
        // such that the resulting error vectors have a covariance of the identity matrix.
        let roll_pitch_yaw_information_root = roll_pitch_yaw_noise
            .cholesky()
            .expect("roll/pitch noise covariance must be positive definite")
            .l()
            .try_inverse()
            .expect("roll/pitch lower triangular matrix must be invertible");

        let duration = interval_dt::<f64>(start_time, end_time);
        let measurement_taus = measurements
            .iter()
            .map(|measurement| tau::<f64>(start_time, end_time, measurement.time))
            .collect();

        Self {
            measurements,
            measurement_taus,
            roll_pitch_yaw_information_root,
            start_time,
            end_time,
            duration,
        }
    }

    pub fn extend_measurements(&mut self, measurements: impl IntoIterator<Item = ImuMeasurement>) {
        for measurement in measurements {
            self.measurement_taus.push(tau::<f64>(
                self.start_time,
                self.end_time,
                measurement.time,
            ));
            self.measurements.push(measurement);
        }
    }

    fn residuals_on_spline<T: Numeric>(
        &self,
        pose_start: SE23<T>,
        pose_end: SE23<T>,
    ) -> VectorX<T> {
        let dt = T::from(self.duration);
        let spline = SE23Spline::new(pose_start, pose_end, dt);

        assert_eq!(self.measurements.len(), self.measurement_taus.len());

        let mut residual = VectorX::<T>::zeros(
            2 * self.measurements.len() + self.measurements.len().saturating_sub(1),
        );

        let mut previous_predicted_yaw = T::zero();
        let mut previous_measurement_yaw = T::zero();

        for (i, (measurement, measurement_tau)) in self
            .measurements
            .iter()
            .zip(self.measurement_taus.iter())
            .enumerate()
        {
            let measurement_tau = T::from(*measurement_tau);
            let current_pose = spline.evaluate(measurement_tau);

            let predicted_rpy = roll_pitch_yaw_from_so3(current_pose.rot());
            let measurement_rpy = measurement.state.roll_pitch_yaw.inner.cast::<T>();

            let roll_residual = angle_difference(predicted_rpy.x, measurement_rpy.x);
            let pitch_residual = angle_difference(predicted_rpy.y, measurement_rpy.y);

            if i == 0 {
                previous_predicted_yaw = predicted_rpy.z;
                previous_measurement_yaw = measurement_rpy.z;

                let whitened_roll_pitch_error = self
                    .roll_pitch_yaw_information_root
                    .cast::<T>()
                    .fixed_view::<2, 2>(0, 0)
                    * Vector2::new(roll_residual, pitch_residual);
                residual
                    .fixed_view_mut::<2, 1>(0, 0)
                    .copy_from(&whitened_roll_pitch_error);
                continue;
            }

            let predicted_yaw_delta = angle_difference(predicted_rpy.z, previous_predicted_yaw);
            let measurement_yaw_delta =
                angle_difference(measurement_rpy.z, previous_measurement_yaw);

            let yaw_residual = angle_difference(predicted_yaw_delta, measurement_yaw_delta);

            let roll_pitch_yaw_residual = Vector3::new(roll_residual, pitch_residual, yaw_residual);
            let whitened_roll_pitch_yaw_error =
                self.roll_pitch_yaw_information_root.cast::<T>() * roll_pitch_yaw_residual;

            previous_predicted_yaw = predicted_rpy.z;
            previous_measurement_yaw = measurement_rpy.z;

            residual
                .fixed_view_mut::<3, 1>(3 * i - 1, 0)
                .copy_from(&whitened_roll_pitch_yaw_error);
        }

        residual
    }
}

fn roll_pitch_yaw_from_so3<T: Numeric>(rotation: &SO3<T>) -> Vector3<T> {
    let one = T::from(1.0);
    let two = T::from(2.0);

    let w = rotation.w();
    let x = rotation.x();
    let y = rotation.y();
    let z = rotation.z();

    let roll = (two * (w * x + y * z)).atan2(one - two * (x * x + y * y));
    let pitch = (two * (w * y - z * x)).asin();
    let yaw = (two * (w * z + x * y)).atan2(one - two * (y * y + z * z));

    Vector3::new(roll, pitch, yaw)
}

fn angle_difference<T: Numeric>(left: T, right: T) -> T {
    let difference = left - right;
    difference.sin().atan2(difference.cos())
}
#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use booster::ImuState;
    use factrs::{core::SO3, traits::Variable};
    use linear_algebra::IntoFramed;
    use nalgebra::{UnitQuaternion, Vector3, vector};

    #[test]
    fn test_imu_factor_residual_matches_roll_pitch_and_yaw_deltas() {
        let now = SystemTime::now();
        let duration = Duration::from_secs(1);
        let sample_dt = Duration::from_secs_f64(1.0 / 500.0);

        let start_rotation = SO3::exp(Vector3::new(0.1, -0.2, 0.3).as_view());
        let end_rotation = SO3::exp(Vector3::new(0.2, -0.1, 0.8).as_view());
        let start = SE23::from_rot_vel_trans(start_rotation, Vector3::zeros(), Vector3::zeros());
        let end = SE23::from_rot_vel_trans(end_rotation, Vector3::zeros(), Vector3::zeros());
        let spline = SE23Spline::new(start.clone(), end.clone(), duration.as_secs_f64());

        let mut measurements = Vec::new();
        for i in 0..500 {
            let time = now + Duration::from_secs_f64(i as f64 * sample_dt.as_secs_f64());
            let sample_tau = tau::<f64>(now, now + duration, time);
            let roll_pitch_yaw = roll_pitch_yaw_from_so3(spline.evaluate(sample_tau).rot());
            measurements.push(ImuMeasurement {
                time,
                state: ImuState {
                    roll_pitch_yaw: roll_pitch_yaw.cast::<f32>().framed(),
                    angular_velocity: Vector3::zeros().framed(),
                    linear_acceleration: Vector3::zeros().framed(),
                },
            });
        }
        let imu_factor = IntervalGaussianProcessImuFactor::new(
            measurements,
            Matrix3::identity(),
            now,
            now + duration,
        );

        let residual = imu_factor.residuals_on_spline(start, end);

        assert_eq!(residual.len(), 3 * 500 - 1);
        assert!(residual.norm() < 1.0e-5);
    }

    #[test]
    fn test_imu_factor_wraps_yaw_delta_residual() {
        let now = SystemTime::now();
        let start_yaw = 179.0_f64.to_radians();
        let measured_end_yaw = -179.0_f64.to_radians();

        let pose = SE23::from_rot_vel_trans(
            so3_from_euler_angles(0.0, 0.0, start_yaw),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let measurements = vec![
            ImuMeasurement {
                time: now,
                state: ImuState {
                    roll_pitch_yaw: vector![0.0, 0.0, start_yaw as f32].framed(),
                    angular_velocity: Vector3::zeros().framed(),
                    linear_acceleration: Vector3::zeros().framed(),
                },
            },
            ImuMeasurement {
                time: now + Duration::from_secs(1),
                state: ImuState {
                    roll_pitch_yaw: vector![0.0, 0.0, measured_end_yaw as f32].framed(),
                    angular_velocity: Vector3::zeros().framed(),
                    linear_acceleration: Vector3::zeros().framed(),
                },
            },
        ];
        let imu_factor = IntervalGaussianProcessImuFactor::new(
            measurements,
            Matrix3::identity(),
            now,
            now + Duration::from_secs(1),
        );

        let residual = imu_factor.residuals_on_spline(pose.clone(), pose);

        assert_eq!(residual.len(), 5);
        assert!(residual.fixed_rows::<4>(0).norm() < 1.0e-6);
        assert!((residual[4] + 2.0_f64.to_radians()).abs() < 1.0e-6);
    }

    fn so3_from_euler_angles(roll: f64, pitch: f64, yaw: f64) -> SO3 {
        let quaternion = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        SO3::from_xyzw(quaternion.i, quaternion.j, quaternion.k, quaternion.w)
    }
}
