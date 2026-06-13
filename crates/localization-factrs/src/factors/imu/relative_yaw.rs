use factrs::{
    core::SO3,
    linalg::{ForwardProp, Matrix3, Numeric, VectorX},
    traits::Residual,
    variables::SE23,
};

use super::orientation::{
    angle_difference, relative_yaw_information_root, roll_pitch_yaw_from_so3,
};

#[derive(Debug, Clone)]
pub(crate) struct RelativeYawFactor {
    measured_yaw_delta: f64,
    information_root: f64,
}

#[factrs::mark]
impl Residual for RelativeYawFactor {
    type Input = (SE23, SE23);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        1
    }

    fn residual<T: Numeric>(&self, (start, end): (SE23<T>, SE23<T>)) -> VectorX<T> {
        let start_yaw = roll_pitch_yaw_from_so3(start.rot()).z;
        let end_yaw = roll_pitch_yaw_from_so3(end.rot()).z;
        let predicted_yaw_delta = angle_difference(end_yaw, start_yaw);
        let raw_error = angle_difference(predicted_yaw_delta, T::from(self.measured_yaw_delta));

        VectorX::<T>::from_element(1, T::from(self.information_root) * raw_error)
    }
}

impl RelativeYawFactor {
    pub(crate) fn new(
        measured_start_orientation: SO3,
        measured_end_orientation: SO3,
        roll_pitch_yaw_noise: Matrix3<f64>,
    ) -> Self {
        let start_yaw = roll_pitch_yaw_from_so3(&measured_start_orientation).z;
        let end_yaw = roll_pitch_yaw_from_so3(&measured_end_orientation).z;

        Self {
            measured_yaw_delta: angle_difference(end_yaw, start_yaw),
            information_root: relative_yaw_information_root(roll_pitch_yaw_noise),
        }
    }
}

#[cfg(test)]
mod tests {
    use factrs::{core::Vector3, traits::Residual, variables::SE23};
    use nalgebra::Matrix3;

    use super::*;
    use crate::factors::imu::orientation::so3_from_euler_angles;

    #[test]
    fn relative_yaw_factor_ignores_constant_yaw_offset() {
        let measured_start = so3_from_euler_angles(0.0, 0.0, 0.1);
        let measured_end = so3_from_euler_angles(0.0, 0.0, 0.6);
        let predicted_start = SE23::from_rot_vel_trans(
            so3_from_euler_angles(0.0, 0.0, 1.2),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let predicted_end = SE23::from_rot_vel_trans(
            so3_from_euler_angles(0.0, 0.0, 1.7),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let factor = RelativeYawFactor::new(measured_start, measured_end, Matrix3::identity());

        let residual = factor.residual((predicted_start, predicted_end));

        assert!(residual.norm() < 1.0e-12);
    }

    #[test]
    fn relative_yaw_factor_wraps_yaw_delta_residual() {
        let measured_start = so3_from_euler_angles(0.0, 0.0, 179.0_f64.to_radians());
        let measured_end = so3_from_euler_angles(0.0, 0.0, -179.0_f64.to_radians());
        let predicted = SE23::from_rot_vel_trans(
            so3_from_euler_angles(0.0, 0.0, 179.0_f64.to_radians()),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let factor = RelativeYawFactor::new(measured_start, measured_end, Matrix3::identity());

        let residual = factor.residual((predicted.clone(), predicted));

        assert!((residual[0] + 2.0_f64.to_radians() / 2.0_f64.sqrt()).abs() < 1.0e-12);
    }
}
