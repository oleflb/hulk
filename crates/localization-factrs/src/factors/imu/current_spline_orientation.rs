use std::time::SystemTime;

use factrs::{
    core::{SO3, Vector2},
    linalg::{ForwardProp, Matrix3, Numeric, VectorX},
    traits::Residual,
    variables::SE23,
};
use nalgebra::Matrix2;

use crate::{
    SE23Spline,
    measurements::ImuMeasurement,
    utils::{interval_dt, tau},
};

use super::orientation::{
    angle_difference, orientation_from_measurement, relative_yaw_information_root,
    roll_pitch_information_root, roll_pitch_yaw_from_so3,
};

#[derive(Debug, Clone)]
pub(crate) struct CurrentSplineOrientationFactor {
    measurement_tau: f64,
    measured_roll_pitch: Vector2<f64>,
    measured_yaw_delta_from_start: f64,
    roll_pitch_information_root: Matrix2<f64>,
    yaw_information_root: f64,
    duration: f64,
}

#[factrs::mark]
impl Residual for CurrentSplineOrientationFactor {
    type Input = (SE23, SE23);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        3
    }

    fn residual<T: Numeric>(&self, (start, end): (SE23<T>, SE23<T>)) -> VectorX<T> {
        let spline = SE23Spline::new(start.clone(), end, T::from(self.duration));
        let current_pose = spline.evaluate(T::from(self.measurement_tau));

        let start_rpy = roll_pitch_yaw_from_so3(start.rot());
        let current_rpy = roll_pitch_yaw_from_so3(current_pose.rot());
        let measured_roll_pitch = self.measured_roll_pitch.cast::<T>();

        let roll_pitch_error = Vector2::new(
            angle_difference(current_rpy.x, measured_roll_pitch.x),
            angle_difference(current_rpy.y, measured_roll_pitch.y),
        );
        let whitened_roll_pitch = self.roll_pitch_information_root.cast::<T>() * roll_pitch_error;

        let predicted_yaw_delta_from_start = angle_difference(current_rpy.z, start_rpy.z);
        let yaw_error = angle_difference(
            predicted_yaw_delta_from_start,
            T::from(self.measured_yaw_delta_from_start),
        );

        let mut residual = VectorX::<T>::zeros(3);
        residual
            .fixed_view_mut::<2, 1>(0, 0)
            .copy_from(&whitened_roll_pitch);
        residual[2] = T::from(self.yaw_information_root) * yaw_error;
        residual
    }
}

impl CurrentSplineOrientationFactor {
    pub(crate) fn new(
        measured_start_orientation: SO3,
        current_measurement: &ImuMeasurement,
        roll_pitch_yaw_noise: Matrix3<f64>,
        start_time: SystemTime,
        end_time: SystemTime,
    ) -> Self {
        let measured_current_orientation = orientation_from_measurement(current_measurement);
        let measured_current_rpy = roll_pitch_yaw_from_so3(&measured_current_orientation);
        let measured_start_yaw = roll_pitch_yaw_from_so3(&measured_start_orientation).z;

        Self {
            measurement_tau: tau::<f64>(start_time, end_time, current_measurement.time),
            measured_roll_pitch: measured_current_rpy.fixed_rows::<2>(0).into_owned(),
            measured_yaw_delta_from_start: angle_difference(
                measured_current_rpy.z,
                measured_start_yaw,
            ),
            roll_pitch_information_root: roll_pitch_information_root(roll_pitch_yaw_noise),
            yaw_information_root: relative_yaw_information_root(roll_pitch_yaw_noise),
            duration: interval_dt::<f64>(start_time, end_time),
        }
    }
}
