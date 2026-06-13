use factrs::{
    linalg::Numeric,
    variables::{MatrixLieGroup, SE23, Variable},
};
use nalgebra::{Matrix2, Vector2, Vector3};

use crate::{camera_intrinsics::CameraIntrinsics, measurements::VisualMeasurement};

use super::LandmarkAssociationConfig;

pub(super) struct CandidateProjectionCache<T: Numeric> {
    projections: Vec<Vector2<T>>,
    depth_residuals: Vec<T>,
}

#[derive(Clone)]
pub(super) struct PairResidual<T: Numeric> {
    reprojection: Vector2<T>,
    depth: T,
}

impl<T: Numeric> CandidateProjectionCache<T> {
    pub(super) fn for_frame(
        frame: &VisualMeasurement,
        state_inverse: &SE23<T>,
        intrinsics: &CameraIntrinsics<T>,
        config: &LandmarkAssociationConfig,
    ) -> Self {
        let camera_from_robot = frame.robot_to_camera.cast::<T>();
        let depth_floor = T::from(config.depth_floor);
        let depth_softness = T::from(config.depth_softness);
        let depth_sigma = T::from(config.depth_sigma);

        let (projections, depth_residuals) = frame
            .candidates
            .iter()
            .map(|global_point| {
                let point_robot = state_inverse.apply(global_point.cast::<T>().coords.as_view());
                let point_camera = camera_from_robot.apply(point_robot.as_view());

                let depth_violation = smooth_hinge(depth_floor - point_camera.z, depth_softness);
                let z_safe = point_camera.z + depth_violation;
                let point_camera_safe = Vector3::new(point_camera.x, point_camera.y, z_safe);

                (
                    intrinsics.project(point_camera_safe.as_view()),
                    depth_violation / depth_sigma,
                )
            })
            .unzip();

        Self {
            projections,
            depth_residuals,
        }
    }

    pub(super) fn pair_residuals(
        &self,
        frame: &VisualMeasurement,
        pixel_information_root: &Matrix2<T>,
    ) -> Vec<PairResidual<T>> {
        frame
            .detections
            .iter()
            .flat_map(|detection| {
                let detection = detection.cast::<T>().coords;
                self.projections
                    .iter()
                    .zip(self.depth_residuals.iter())
                    .map(move |(projection, depth_residual)| PairResidual {
                        reprojection: pixel_information_root * (*projection - detection),
                        depth: *depth_residual,
                    })
            })
            .collect()
    }
}

impl<T: Numeric> PairResidual<T> {
    pub(super) fn cost(&self) -> T {
        T::from(0.5) * (self.reprojection.norm_squared() + self.depth * self.depth)
    }

    pub(super) fn weighted(self, assignment_sqrt: T) -> Vector3<T> {
        Vector3::new(
            self.reprojection.x * assignment_sqrt,
            self.reprojection.y * assignment_sqrt,
            self.depth * assignment_sqrt,
        )
    }
}

pub(super) fn smooth_hinge<T: Numeric>(x: T, softness: T) -> T {
    T::from(0.5) * (x + (x * x + softness * softness).sqrt())
}
