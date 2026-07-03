use std::time::SystemTime;

use factrs::{
    core::SE3,
    linalg::{ForwardProp, Numeric, VectorX},
    traits::{Residual, Variable},
    variables::SE23,
};
use nalgebra::{SMatrix, SVector};

use crate::{SE23Spline, measurements::GlobalPoseMeasurement, utils::tau};

#[derive(Debug, Clone)]
pub struct GlobalPoseFactor {
    measurements: Vec<GlobalPoseMeasurement>,
    measurement_taus: Vec<f64>,
    information_root: SMatrix<f64, 6, 6>,
    start_time: SystemTime,
    end_time: SystemTime,
    duration: f64,
}

#[factrs::mark]
impl Residual for GlobalPoseFactor {
    type Input = (SE23, SE23);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        self.measurements.len() * 6
    }

    fn residual<T: Numeric>(&self, (start, end): (SE23<T>, SE23<T>)) -> VectorX<T> {
        self.residuals_on_spline(start, end)
    }
}

impl GlobalPoseFactor {
    pub fn new(
        start_time: SystemTime,
        end_time: SystemTime,
        measurements: impl IntoIterator<Item = GlobalPoseMeasurement>,
        global_pose_noise: SMatrix<f64, 6, 6>,
    ) -> Self {
        let information_root = global_pose_noise
            .cholesky()
            .expect("global pose covariance must be positive definite")
            .l()
            .try_inverse()
            .expect("global pose covariance Cholesky factor must be invertible");
        let duration = crate::utils::interval_dt::<f64>(start_time, end_time);
        let mut factor = Self {
            measurements: Vec::new(),
            measurement_taus: Vec::new(),
            information_root,
            start_time,
            end_time,
            duration,
        };
        factor.extend_measurements(measurements);
        factor
    }

    pub fn extend_measurements(
        &mut self,
        measurements: impl IntoIterator<Item = GlobalPoseMeasurement>,
    ) {
        for measurement in measurements {
            self.measurement_taus.push(tau::<f64>(
                self.start_time,
                self.end_time,
                measurement.time,
            ));
            self.measurements.push(measurement);
        }
    }

    fn residuals_on_spline<T: Numeric>(&self, start: SE23<T>, end: SE23<T>) -> VectorX<T> {
        assert_eq!(self.measurements.len(), self.measurement_taus.len());

        let spline = SE23Spline::new(start, end, T::from(self.duration));
        let information_root = self.information_root.cast::<T>();
        let mut residuals = VectorX::<T>::zeros(self.dim_out());

        for (index, (measurement, measurement_tau)) in self
            .measurements
            .iter()
            .zip(self.measurement_taus.iter())
            .enumerate()
        {
            let predicted = se23_pose_to_se3(spline.evaluate(T::from(*measurement_tau)));
            let observed = se23_pose_to_se3(measurement.robot_to_field.clone().cast::<T>());
            let raw_error = predicted.ominus(&observed);
            let raw_error = SVector::<T, 6>::from_column_slice(raw_error.as_slice());
            let whitened = information_root * raw_error;

            residuals
                .fixed_view_mut::<6, 1>(index * 6, 0)
                .copy_from(&whitened);
        }

        residuals
    }
}

fn se23_pose_to_se3<T: Numeric>(pose: SE23<T>) -> SE3<T> {
    SE3::from_rot_trans(pose.rot().clone(), pose.xyz().into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use factrs::core::{SO3, Vector3};

    fn state(x: f64) -> SE23 {
        SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), Vector3::new(x, 0.0, 0.0))
    }

    fn measurement(time: SystemTime, x: f64) -> GlobalPoseMeasurement {
        GlobalPoseMeasurement {
            time,
            robot_to_field: state(x),
        }
    }

    #[test]
    fn residual_is_zero_for_matching_pose() {
        let time = SystemTime::UNIX_EPOCH;
        let factor = GlobalPoseFactor::new(
            time,
            time + std::time::Duration::from_secs(1),
            [measurement(time, 1.0)],
            SMatrix::<f64, 6, 6>::identity(),
        );

        let residual = factor.residuals_on_spline(state(1.0), state(1.0));

        assert!(residual.norm() < 1.0e-9);
    }

    #[test]
    fn residual_changes_with_pose_error() {
        let time = SystemTime::UNIX_EPOCH;
        let factor = GlobalPoseFactor::new(
            time,
            time + std::time::Duration::from_secs(1),
            [measurement(time, 1.0)],
            SMatrix::<f64, 6, 6>::identity(),
        );

        let residual = factor.residuals_on_spline(state(0.0), state(0.0));

        assert!(residual.norm() > 0.5);
    }
}
