use std::time::SystemTime;

use factrs::{
    linalg::{ForwardProp, Numeric, VectorX},
    traits::{Residual, Variable},
    variables::{MatrixLieGroup, SE23},
};
use nalgebra::Matrix2;

use crate::{
    SE23Spline,
    camera_intrinsics::CameraIntrinsics,
    measurements::VisualReprojectionMeasurement,
    utils::{interval_dt, tau},
};

#[derive(Debug, Clone)]
pub struct VisualReprojectionFactor {
    measurements: Vec<VisualReprojectionMeasurement>,
    measurement_taus: Vec<f64>,
    pixel_information_root: Matrix2<f64>,
    start_time: SystemTime,
    end_time: SystemTime,
    duration: f64,
}

#[derive(Debug, Clone)]
pub struct PoseHintVisualReprojectionFactor {
    inner: VisualReprojectionFactor,
}

#[factrs::mark]
impl Residual for VisualReprojectionFactor {
    type Input = (SE23, SE23, CameraIntrinsics);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        self.measurements.len() * 2
    }

    fn residual<T: Numeric>(
        &self,
        (start, end, camera_intrinsics): (SE23<T>, SE23<T>, CameraIntrinsics<T>),
    ) -> VectorX<T> {
        self.residuals_on_spline(start, end, camera_intrinsics)
    }
}

impl VisualReprojectionFactor {
    pub fn new(
        start_time: SystemTime,
        end_time: SystemTime,
        frames: Vec<Vec<VisualReprojectionMeasurement>>,
        visual_feature_noise: Matrix2<f64>,
    ) -> Self {
        let pixel_information_root = visual_feature_noise
            .cholesky()
            .expect("visual feature covariance must be positive definite")
            .l()
            .try_inverse()
            .expect("visual feature covariance Cholesky factor must be invertible");
        let duration = interval_dt::<f64>(start_time, end_time);
        let mut factor = Self {
            measurements: Vec::new(),
            measurement_taus: Vec::new(),
            pixel_information_root,
            start_time,
            end_time,
            duration,
        };
        factor.extend_frames(frames);
        factor
    }

    pub fn extend_frames(
        &mut self,
        frames: impl IntoIterator<Item = Vec<VisualReprojectionMeasurement>>,
    ) {
        for measurement in frames.into_iter().flatten() {
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
        start: SE23<T>,
        end: SE23<T>,
        intrinsics: CameraIntrinsics<T>,
    ) -> VectorX<T> {
        assert_eq!(self.measurements.len(), self.measurement_taus.len());

        let spline = SE23Spline::new(start, end, T::from(self.duration));
        let pixel_information_root = self.pixel_information_root.cast::<T>();
        let mut residuals = VectorX::<T>::zeros(self.dim_out());

        for (index, (measurement, measurement_tau)) in self
            .measurements
            .iter()
            .zip(self.measurement_taus.iter())
            .enumerate()
        {
            let robot_to_field = spline.evaluate(T::from(*measurement_tau));
            let field_to_robot = robot_to_field.inverse();
            let robot_to_camera = measurement.robot_to_camera.cast::<T>();
            let field_point = measurement.field_point.coords.cast::<T>();
            let point_robot = field_to_robot.apply(field_point.as_view());
            let point_camera = robot_to_camera.apply(point_robot.as_view());
            let reprojection = intrinsics.project(point_camera.as_view())
                - measurement.detection.coords.cast::<T>();
            let whitened = pixel_information_root * reprojection;

            residuals
                .fixed_view_mut::<2, 1>(index * 2, 0)
                .copy_from(&whitened);
        }

        residuals
    }
}

#[factrs::mark]
impl Residual for PoseHintVisualReprojectionFactor {
    type Input = (SE23, SE23, CameraIntrinsics);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        self.inner.dim_out()
    }

    fn residual<T: Numeric>(&self, input: (SE23<T>, SE23<T>, CameraIntrinsics<T>)) -> VectorX<T> {
        self.inner.residual(input)
    }
}

impl PoseHintVisualReprojectionFactor {
    pub fn new(
        start_time: SystemTime,
        end_time: SystemTime,
        measurement: VisualReprojectionMeasurement,
        visual_feature_noise: Matrix2<f64>,
    ) -> Self {
        Self {
            inner: VisualReprojectionFactor::new(
                start_time,
                end_time,
                vec![vec![measurement]],
                visual_feature_noise,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, SystemTime};

    use factrs::{
        core::{SE3, SO3, Vector3},
        linalg::VectorX,
        traits::{Residual, Variable},
        variables::SE23,
    };
    use nalgebra::{Matrix2, Point2, Point3, vector};

    use super::*;

    fn state(position: Vector3) -> SE23 {
        SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), position)
    }

    #[test]
    fn residual_is_zero_for_exact_reprojection() {
        let time = SystemTime::UNIX_EPOCH;
        let factor = VisualReprojectionFactor::new(
            time,
            time + Duration::from_secs(1),
            vec![vec![VisualReprojectionMeasurement {
                time,
                detection: Point2::new(0.0, 0.0),
                field_point: Point3::new(0.0, 0.0, 2.0),
                robot_to_camera: SE3::from_rot_trans(SO3::identity(), Vector3::zeros()),
            }]],
            Matrix2::identity(),
        );
        let intrinsics = CameraIntrinsics::new(vector![100.0, 100.0], vector![0.0, 0.0]);

        let residual =
            factor.residual((state(Vector3::zeros()), state(Vector3::zeros()), intrinsics));

        assert!(residual.norm() < 1.0e-9);
    }

    #[test]
    fn residual_changes_with_pose_error() {
        let time = SystemTime::UNIX_EPOCH;
        let factor = VisualReprojectionFactor::new(
            time,
            time + Duration::from_secs(1),
            vec![vec![VisualReprojectionMeasurement {
                time,
                detection: Point2::new(0.0, 0.0),
                field_point: Point3::new(0.0, 0.0, 2.0),
                robot_to_camera: SE3::from_rot_trans(SO3::identity(), Vector3::zeros()),
            }]],
            Matrix2::identity(),
        );
        let intrinsics = CameraIntrinsics::new(vector![100.0, 100.0], vector![0.0, 0.0]);

        let residual = factor.residual((
            state(vector![0.1, 0.0, 0.0]),
            state(vector![0.1, 0.0, 0.0]),
            intrinsics,
        ));

        assert!(residual.norm() > 1.0);
    }

    #[test]
    fn translation_jacobian_pulls_pose_toward_observation() {
        let time = SystemTime::UNIX_EPOCH;
        let factor = VisualReprojectionFactor::new(
            time,
            time + Duration::from_secs(1),
            vec![vec![VisualReprojectionMeasurement {
                time,
                detection: Point2::new(0.0, 0.0),
                field_point: Point3::new(0.0, 0.0, 2.0),
                robot_to_camera: SE3::from_rot_trans(SO3::identity(), Vector3::zeros()),
            }]],
            Matrix2::identity(),
        );
        let intrinsics = CameraIntrinsics::new(vector![100.0, 100.0], vector![0.0, 0.0]);
        let pose = state(vector![0.1, -0.2, 0.0]);

        let linearized = factor.residual_jacobian((pose.clone(), pose.clone(), intrinsics.clone()));

        assert_close(linearized.value[0], -5.0, 1.0e-9);
        assert_close(linearized.value[1], 10.0, 1.0e-9);

        // SE23 tangent order is [rot_x, rot_y, rot_z, vel_x, vel_y, vel_z, x, y, z].
        let start_x_column = 6;
        let start_y_column = 7;
        assert_close(linearized.diff[(0, start_x_column)], -50.0, 1.0e-9);
        assert_close(linearized.diff[(1, start_y_column)], -50.0, 1.0e-9);
        assert_close(linearized.diff[(0, start_y_column)], 0.0, 1.0e-9);
        assert_close(linearized.diff[(1, start_x_column)], 0.0, 1.0e-9);

        let mut translation_step = VectorX::zeros(9);
        translation_step[start_x_column] =
            -linearized.value[0] / linearized.diff[(0, start_x_column)];
        translation_step[start_y_column] =
            -linearized.value[1] / linearized.diff[(1, start_y_column)];

        assert_close(translation_step[start_x_column], -0.1, 1.0e-9);
        assert_close(translation_step[start_y_column], 0.2, 1.0e-9);

        let corrected_pose = pose.oplus(translation_step.as_view());
        let corrected_residual = factor.residual((corrected_pose, pose, intrinsics));

        assert!(corrected_residual.norm() < 1.0e-9);
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}"
        );
    }
}
