use std::slice;

use factrs::{
    linalg::{Numeric, VectorX},
    variables::{SE23, Variable},
};
use nalgebra::{Matrix2, Vector3};

use crate::{camera_intrinsics::CameraIntrinsics, splines::SE23Spline};

use super::{
    LandmarkAssociationConfig, LandmarkFactor, LandmarkFrame,
    association::rectangular_log_sinkhorn_log_assignments,
    projection::{CandidateProjectionCache, PairResidual},
};

use crate::measurements::VisualMeasurement;

enum AssignmentMode<'a> {
    Dynamic,
    Frozen(&'a [f64]),
}

impl LandmarkFactor {
    pub(crate) fn residuals_on_spline<T: Numeric>(
        &self,
        pose_start: SE23<T>,
        pose_end: SE23<T>,
        intrinsics: CameraIntrinsics<T>,
    ) -> VectorX<T> {
        self.residuals_on_spline_impl(pose_start, pose_end, intrinsics, AssignmentMode::Dynamic)
    }

    pub(super) fn residuals_on_spline_with_assignment_sqrts<T: Numeric>(
        &self,
        pose_start: SE23<T>,
        pose_end: SE23<T>,
        intrinsics: CameraIntrinsics<T>,
        assignment_sqrts: &[f64],
    ) -> VectorX<T> {
        self.residuals_on_spline_impl(
            pose_start,
            pose_end,
            intrinsics,
            AssignmentMode::Frozen(assignment_sqrts),
        )
    }

    pub(super) fn assignment_sqrts(
        &self,
        pose_start: &SE23<f64>,
        pose_end: &SE23<f64>,
        intrinsics: &CameraIntrinsics<f64>,
    ) -> Vec<f64> {
        let spline = SE23Spline::new(pose_start.clone(), pose_end.clone(), self.duration);
        let half = 0.5;

        self.frames
            .iter()
            .flat_map(|frame| {
                frame
                    .measurements
                    .iter()
                    .filter(has_associations)
                    .flat_map(|measurement| {
                        let config =
                            frame.association_config_for_measurement(measurement, self.config);
                        let pair_residuals = self.measurement_pair_residuals(
                            frame,
                            measurement,
                            &spline,
                            intrinsics,
                            &self.pixel_information_root,
                        );
                        let costs = pair_residuals
                            .iter()
                            .map(PairResidual::cost)
                            .collect::<Vec<_>>();

                        rectangular_log_sinkhorn_log_assignments(
                            &config,
                            &costs,
                            measurement.detections.len(),
                            measurement.candidates.len(),
                        )
                        .into_iter()
                        .map(move |log_assignment| (half * log_assignment).exp())
                    })
            })
            .collect()
    }

    fn residuals_on_spline_impl<T: Numeric>(
        &self,
        pose_start: SE23<T>,
        pose_end: SE23<T>,
        intrinsics: CameraIntrinsics<T>,
        assignment_mode: AssignmentMode<'_>,
    ) -> VectorX<T> {
        let dt = T::from(self.duration);
        let spline = SE23Spline::new(pose_start, pose_end, dt);
        let pixel_information_root = self.pixel_information_root.cast::<T>();

        let mut residuals = VectorX::<T>::zeros(self.residual_dim());
        let mut residual_offset = 0;
        let mut frozen_assignments = match assignment_mode {
            AssignmentMode::Dynamic => None,
            AssignmentMode::Frozen(assignment_sqrts) => {
                assert_eq!(assignment_sqrts.len(), self.residual_dim() / 3);
                Some(assignment_sqrts.iter())
            }
        };

        for frame in &self.frames {
            for measurement in frame.measurements.iter().filter(has_associations) {
                let config = frame.association_config_for_measurement(measurement, self.config);
                let pair_residuals = self.measurement_pair_residuals(
                    frame,
                    measurement,
                    &spline,
                    &intrinsics,
                    &pixel_information_root,
                );

                match frozen_assignments.as_mut() {
                    Some(assignment_sqrts) => write_frozen_assignment_residuals(
                        &mut residuals,
                        &mut residual_offset,
                        pair_residuals,
                        assignment_sqrts,
                    ),
                    None => write_dynamic_assignment_residuals(
                        &config,
                        &mut residuals,
                        &mut residual_offset,
                        pair_residuals,
                        measurement.detections.len(),
                        measurement.candidates.len(),
                    ),
                }
            }
        }

        if let Some(assignment_sqrts) = frozen_assignments {
            assert_eq!(assignment_sqrts.len(), 0);
        }
        assert_eq!(residual_offset, self.residual_dim());

        residuals
    }

    fn measurement_pair_residuals<T: Numeric>(
        &self,
        frame: &LandmarkFrame,
        measurement: &VisualMeasurement,
        spline: &SE23Spline<T>,
        intrinsics: &CameraIntrinsics<T>,
        pixel_information_root: &Matrix2<T>,
    ) -> Vec<PairResidual<T>> {
        let state = spline.evaluate(T::from(frame.tau));
        let state_inverse = state.inverse();
        let projection_cache = CandidateProjectionCache::for_frame(
            measurement,
            &state_inverse,
            intrinsics,
            &self.config,
        );

        projection_cache.pair_residuals(measurement, pixel_information_root)
    }
}

fn has_associations(measurement: &&VisualMeasurement) -> bool {
    !measurement.detections.is_empty() && !measurement.candidates.is_empty()
}

fn write_dynamic_assignment_residuals<T: Numeric>(
    config: &LandmarkAssociationConfig,
    residuals: &mut VectorX<T>,
    residual_offset: &mut usize,
    pair_residuals: Vec<PairResidual<T>>,
    num_detections: usize,
    num_candidates: usize,
) {
    let costs = pair_residuals
        .iter()
        .map(PairResidual::cost)
        .collect::<Vec<_>>();
    let log_assignments =
        rectangular_log_sinkhorn_log_assignments(config, &costs, num_detections, num_candidates);

    pair_residuals
        .into_iter()
        .zip(log_assignments)
        .for_each(|(pair_residual, log_assignment)| {
            let assignment_sqrt = (T::from(0.5) * log_assignment).exp();
            write_pair_residual(residuals, residual_offset, pair_residual, assignment_sqrt);
        });
}

fn write_frozen_assignment_residuals<T: Numeric>(
    residuals: &mut VectorX<T>,
    residual_offset: &mut usize,
    pair_residuals: Vec<PairResidual<T>>,
    assignment_sqrts: &mut slice::Iter<'_, f64>,
) {
    pair_residuals.into_iter().for_each(|pair_residual| {
        let assignment_sqrt = assignment_sqrts
            .next()
            .expect("frozen assignment weights must match residual pairs");
        write_pair_residual(
            residuals,
            residual_offset,
            pair_residual,
            T::from(*assignment_sqrt),
        );
    });
}

fn write_pair_residual<T: Numeric>(
    residuals: &mut VectorX<T>,
    residual_offset: &mut usize,
    pair_residual: PairResidual<T>,
    assignment_sqrt: T,
) {
    let weighted = pair_residual.weighted(assignment_sqrt);
    write_weighted_residual(residuals, residual_offset, weighted);
}

fn write_weighted_residual<T: Numeric>(
    residuals: &mut VectorX<T>,
    residual_offset: &mut usize,
    weighted: Vector3<T>,
) {
    residuals
        .fixed_view_mut::<3, 1>(*residual_offset, 0)
        .copy_from(&weighted);
    *residual_offset += 3;
}
