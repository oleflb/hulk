use std::time::SystemTime;

use factrs::{
    linalg::{Diff, DiffResult, ForwardProp, MatrixX, NumericalDiff, VectorX},
    traits::Residual,
    variables::SE23,
};
use nalgebra::Matrix2;

use crate::{
    camera_intrinsics::CameraIntrinsics,
    measurements::{LandmarkAssociationCosts, VisualMeasurement},
    utils::{interval_dt, tau as normalized_time},
};

mod association;
mod projection;
mod residual;

#[cfg(test)]
mod tests;

#[derive(Debug, Clone)]
pub struct LandmarkFactor {
    start_time: SystemTime,
    end_time: SystemTime,
    duration: f64,
    frames: Vec<LandmarkFrame>,
    residual_dim: usize,
    pixel_information_root: Matrix2<f64>,
    config: LandmarkAssociationConfig,
}

#[derive(Debug, Clone)]
pub(super) struct LandmarkFrame {
    measurements: Vec<VisualMeasurement>,
    tau: f64,
}

impl LandmarkFrame {
    fn new(
        start_time: SystemTime,
        end_time: SystemTime,
        measurements: Vec<VisualMeasurement>,
    ) -> Self {
        let time = measurements
            .first()
            .expect("landmark frames must contain at least one visual measurement")
            .time;
        debug_assert!(
            measurements
                .iter()
                .all(|measurement| measurement.time == time)
        );

        let tau = normalized_time::<f64>(start_time, end_time, time);
        Self { measurements, tau }
    }

    fn residual_dim(&self) -> usize {
        self.measurements
            .iter()
            .map(|measurement| 3 * measurement.detections.len() * measurement.candidates.len())
            .sum()
    }

    pub(super) fn association_config_for_measurement(
        &self,
        measurement: &VisualMeasurement,
        base_config: LandmarkAssociationConfig,
    ) -> LandmarkAssociationConfig {
        if let Some(LandmarkAssociationCosts {
            unmatched_landmark,
            unmatched_detection,
        }) = measurement.association_costs
        {
            LandmarkAssociationConfig {
                unmatched_landmark_cost: unmatched_landmark,
                unmatched_detection_cost: unmatched_detection,
                ..base_config
            }
        } else {
            base_config
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LandmarkAssociationConfig {
    /// Number of fixed Sinkhorn row/column normalization iterations.
    /// Keep this fixed so the residual remains a deterministic smooth function
    /// of the graph variables.
    pub sinkhorn_iterations: usize,
    /// Entropic association temperature. Larger values make the association
    /// softer. Smaller values approach hard assignment.
    pub temperature: f64,
    /// Cost used by dummy detection rows to absorb unmatched landmarks.
    /// Use zero when an unmatched landmark should not penalize the pose.
    pub unmatched_landmark_cost: f64,
    /// Cost used by dummy landmark columns to absorb unmatched detections.
    /// Use zero when an unmatched detection should not penalize the pose.
    pub unmatched_detection_cost: f64,
    /// Minimum positive depth used by the smooth projection model.
    pub depth_floor: f64,
    /// Smoothness of the positive-depth hinge. Larger values make the depth
    /// transition smoother but perturb valid positive-depth projections more.
    pub depth_softness: f64,
    /// Standard deviation for the smooth behind-camera depth residual.
    pub depth_sigma: f64,
}

impl Default for LandmarkAssociationConfig {
    fn default() -> Self {
        Self {
            sinkhorn_iterations: 10,
            temperature: 5.0,
            unmatched_landmark_cost: 0.0,
            unmatched_detection_cost: 25.0,
            depth_floor: 1e-3,
            depth_softness: 1e-3,
            depth_sigma: 0.1,
        }
    }
}

#[factrs::mark]
impl Residual for LandmarkFactor {
    type Input = (SE23, SE23, CameraIntrinsics);
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        self.residual_dim()
    }

    fn residual<T: factrs::linalg::Numeric>(
        &self,
        (start, end, camera_intrinsics): (SE23<T>, SE23<T>, CameraIntrinsics<T>),
    ) -> VectorX<T> {
        self.residuals_on_spline(start, end, camera_intrinsics)
    }

    fn residual_jacobian(
        &self,
        input: (SE23, SE23, CameraIntrinsics),
    ) -> DiffResult<VectorX, MatrixX> {
        let assignment_sqrts = self.assignment_sqrts(&input.0, &input.1, &input.2);
        NumericalDiff::<6>::jacobian(
            |(start, end, camera_intrinsics)| {
                self.residuals_on_spline_with_assignment_sqrts(
                    start,
                    end,
                    camera_intrinsics,
                    &assignment_sqrts,
                )
            },
            &input,
        )
    }
}

impl LandmarkFactor {
    pub fn new(
        start_time: SystemTime,
        end_time: SystemTime,
        frames: Vec<Vec<VisualMeasurement>>,
        visual_feature_noise: Matrix2<f64>,
    ) -> Self {
        let pixel_information_root = visual_feature_noise
            .cholesky()
            .expect("pixel noise covariance must be positive definite")
            .l()
            .try_inverse()
            .expect("pixel noise lower triangular matrix must be invertible");

        let duration = interval_dt::<f64>(start_time, end_time);
        let frames = frames
            .into_iter()
            .map(|frame| LandmarkFrame::new(start_time, end_time, frame))
            .collect::<Vec<_>>();
        let residual_dim = frames.iter().map(LandmarkFrame::residual_dim).sum();

        Self {
            start_time,
            end_time,
            duration,
            frames,
            residual_dim,
            pixel_information_root,
            config: LandmarkAssociationConfig::default(),
        }
    }

    pub fn with_unmatched_costs(
        mut self,
        unmatched_landmark_cost: f64,
        unmatched_detection_cost: f64,
    ) -> Self {
        self.config.unmatched_landmark_cost = unmatched_landmark_cost;
        self.config.unmatched_detection_cost = unmatched_detection_cost;
        self
    }

    pub fn extend_frames(&mut self, frames: impl IntoIterator<Item = Vec<VisualMeasurement>>) {
        let frames = frames
            .into_iter()
            .map(|frame| LandmarkFrame::new(self.start_time, self.end_time, frame))
            .collect::<Vec<_>>();

        self.residual_dim += frames
            .iter()
            .map(LandmarkFrame::residual_dim)
            .sum::<usize>();
        self.frames.extend(frames);
    }

    fn residual_dim(&self) -> usize {
        self.residual_dim
    }
}
