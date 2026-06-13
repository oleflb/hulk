use std::time::{Duration, SystemTime};

use factrs::{
    core::{SO3, Values, Vector3},
    traits::Variable,
    variables::{SE3, SE23},
};
use nalgebra::{Matrix2, Point2, Point3, vector};

use crate::{
    camera_intrinsics::CameraIntrinsics,
    measurements::VisualMeasurement,
    symbols::{CameraIntrinsics as CameraIntrinsicsSymbol, State},
};

use super::{LandmarkFactor, projection::smooth_hinge};

fn identity_state() -> SE23 {
    SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), Vector3::zeros())
}

fn translated_state(translation: Vector3) -> SE23 {
    SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), translation)
}

fn force_real_assignments(factor: &mut LandmarkFactor) {
    factor.config.sinkhorn_iterations = 100;
    factor.config.temperature = 0.001;
    factor.config.unmatched_landmark_cost = 100.0;
    factor.config.unmatched_detection_cost = 100.0;
}

#[test]
fn selects_candidate_with_smallest_reprojection_error() {
    let now = SystemTime::UNIX_EPOCH;
    let feature = Point2::new(1.1, -0.2);
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![feature],
            candidates: vec![Point3::new(0.0, 0.0, 1.0), Point3::new(1.0, 0.0, 1.0)],
            robot_to_camera: SE3::identity(),
            association_costs: None,
        }]],
        Matrix2::identity() * 5.0,
    );
    force_real_assignments(&mut factor);
    factor.config.unmatched_landmark_cost = 0.0;

    let residual = factor.residuals_on_spline(
        identity_state(),
        identity_state(),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    let expected_feature = Point2::new(1.0, 0.0);
    let expected_residual = expected_feature - feature;
    assert_eq!(residual.len(), 6);
    assert!(residual.fixed_rows::<3>(0).norm() < 1e-2);
    assert!((residual[3] - expected_residual.x / 5.0_f64.sqrt()).abs() < 1e-3);
    assert!((residual[4] - expected_residual.y / 5.0_f64.sqrt()).abs() < 1e-3);
}

#[test]
fn residual_uses_graph_values_for_pose_extrinsics_and_intrinsics() {
    let now = SystemTime::UNIX_EPOCH;
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![Point2::new(4.0, 0.0)],
            candidates: vec![Point3::new(1.0, 0.0, 1.0)],
            robot_to_camera: SE3::from_rot_trans(SO3::identity(), Vector3::new(2.0, 0.0, 0.0)),
            association_costs: None,
        }]],
        Matrix2::identity() * 5.0,
    );
    force_real_assignments(&mut factor);

    let mut values = Values::new();
    values.insert(State(0), translated_state(vector![1.0, 0.0, 0.0]));
    values.insert(State(1), translated_state(vector![1.0, 0.0, 0.0]));
    values.insert(
        CameraIntrinsicsSymbol(0),
        CameraIntrinsics::new(vector![2.0, 2.0], vector![0.0, 0.0]),
    );

    let keys = [
        State(0).into(),
        State(1).into(),
        CameraIntrinsicsSymbol(0).into(),
    ];
    let residual = factrs::residuals::ErasedResidual::residual(&factor, &values, &keys)
        .expect("factor should evaluate");

    assert!(residual[0].abs() < 1e-4);
    assert!(residual[1].abs() < 1e-9);
    assert!(residual[2].is_finite());
    assert!(residual[2].abs() < 1e-4);
}

#[test]
fn same_frame_classes_are_associated_independently() {
    let now = SystemTime::UNIX_EPOCH;
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![
            VisualMeasurement {
                time: now,
                detections: vec![Point2::new(10.0, 0.0)],
                candidates: vec![Point3::new(0.0, 0.0, 1.0)],
                robot_to_camera: SE3::identity(),
                association_costs: None,
            },
            VisualMeasurement {
                time: now,
                detections: vec![Point2::new(0.0, 0.0)],
                candidates: vec![Point3::new(10.0, 0.0, 1.0)],
                robot_to_camera: SE3::identity(),
                association_costs: None,
            },
        ]],
        Matrix2::identity(),
    );
    force_real_assignments(&mut factor);

    let residual = factor.residuals_on_spline(
        identity_state(),
        identity_state(),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    assert_eq!(residual.len(), 6);
    assert!(residual[0] < -1.0);
    assert!(residual[1].abs() < 1.0e-9);
    assert!(residual[3] > 1.0);
    assert!(residual[4].abs() < 1.0e-9);
}

#[test]
fn unmatched_landmarks_do_not_force_non_visible_candidates() {
    let now = SystemTime::UNIX_EPOCH;
    let feature = Point2::new(0.0, 0.0);
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![feature],
            candidates: vec![Point3::new(0.0, 0.0, 1.0), Point3::new(10.0, 0.0, 1.0)],
            robot_to_camera: SE3::identity(),
            association_costs: Some(crate::measurements::LandmarkAssociationCosts {
                unmatched_landmark: 0.0,
                unmatched_detection: 100.0,
            }),
        }]],
        Matrix2::identity(),
    );
    force_real_assignments(&mut factor);

    let residual = factor.residuals_on_spline(
        identity_state(),
        identity_state(),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    assert_eq!(residual.len(), 6);
    assert!(residual.fixed_rows::<3>(0).norm() < 1.0e-5);
    assert!(residual.fixed_rows::<3>(3).norm() < 1.0e-5);
}

#[test]
fn sparse_goalpost_detection_only_constrains_best_candidate() {
    let now = SystemTime::UNIX_EPOCH;
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![Point2::new(0.0, 0.0)],
            candidates: vec![
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(20.0, 0.0, 1.0),
                Point3::new(0.0, 20.0, 1.0),
                Point3::new(-20.0, 0.0, 1.0),
            ],
            robot_to_camera: SE3::identity(),
            association_costs: Some(crate::measurements::LandmarkAssociationCosts {
                unmatched_landmark: 0.0,
                unmatched_detection: 25.0,
            }),
        }]],
        Matrix2::identity(),
    );
    factor.config.sinkhorn_iterations = 10;
    factor.config.temperature = 5.0;

    let residual = factor.residuals_on_spline(
        identity_state(),
        identity_state(),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    assert_eq!(residual.len(), 12);
    assert!(residual.fixed_rows::<3>(0).norm() < 1.0e-5);
    assert!(residual.fixed_rows::<3>(3).norm() < 1.0e-5);
    assert!(residual.fixed_rows::<3>(6).norm() < 1.0e-5);
    assert!(residual.fixed_rows::<3>(9).norm() < 1.0e-5);
}

#[test]
fn jacobian_has_expected_shape_with_intrinsics_key() {
    let now = SystemTime::UNIX_EPOCH;
    let factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![Point2::new(0.0, 0.0)],
            candidates: vec![Point3::new(0.0, 0.0, 1.0)],
            robot_to_camera: SE3::identity(),
            association_costs: None,
        }]],
        Matrix2::identity() * 5.0,
    );

    let mut values = Values::new();
    values.insert(State(0), identity_state());
    values.insert(State(1), identity_state());
    values.insert(
        CameraIntrinsicsSymbol(0),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    let keys = [
        State(0).into(),
        State(1).into(),
        CameraIntrinsicsSymbol(0).into(),
    ];
    let jacobian = factrs::residuals::ErasedResidual::residual_jacobian(&factor, &values, &keys)
        .expect("factor should linearize")
        .diff;

    assert_eq!(jacobian.nrows(), 3);
    assert_eq!(jacobian.ncols(), 22);
}

#[test]
fn residual_is_whitened_by_pixel_noise() {
    let now = SystemTime::UNIX_EPOCH;
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![Point2::new(2.0, -4.0)],
            candidates: vec![Point3::new(0.0, 0.0, 1.0)],
            robot_to_camera: SE3::identity(),
            association_costs: None,
        }]],
        Matrix2::identity() * 4.0,
    );
    force_real_assignments(&mut factor);

    let residual = factor.residuals_on_spline(
        identity_state(),
        identity_state(),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    assert!((residual[0] + 1.0).abs() < 1e-9);
    assert!((residual[1] - 2.0).abs() < 1e-9);
}

#[test]
fn jacobian_value_matches_residual_with_fixed_assignments() {
    let now = SystemTime::UNIX_EPOCH;
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![Point2::new(2.0, -4.0)],
            candidates: vec![Point3::new(0.0, 0.0, 1.0)],
            robot_to_camera: SE3::identity(),
            association_costs: None,
        }]],
        Matrix2::identity() * 4.0,
    );
    force_real_assignments(&mut factor);
    let start = identity_state();
    let end = identity_state();
    let intrinsics = CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]);

    let expected = factor.residuals_on_spline(start.clone(), end.clone(), intrinsics.clone());
    let actual =
        factrs::traits::Residual::residual_jacobian(&factor, (start, end, intrinsics)).value;

    assert!((actual - expected).norm() < 1e-9);
}

#[test]
fn returns_large_constant_residual_if_all_candidates_are_behind_camera() {
    let now = SystemTime::UNIX_EPOCH;
    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![Point2::new(0.0, 0.0)],
            candidates: vec![Point3::new(0.0, 0.0, -1.0)],
            robot_to_camera: SE3::identity(),
            association_costs: None,
        }]],
        Matrix2::identity(),
    );
    force_real_assignments(&mut factor);

    let residual = factor.residuals_on_spline(
        identity_state(),
        identity_state(),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    let expected_depth_violation = smooth_hinge(
        1.0 + factor.config.depth_floor,
        factor.config.depth_softness,
    );
    let expected_depth_residual = expected_depth_violation / factor.config.depth_sigma;
    assert!(residual[0].abs() < 1e-9);
    assert!(residual[1].abs() < 1e-9);
    assert!((residual[2] - expected_depth_residual).abs() < 1e-9);
}

#[test]
fn behind_camera_projection_uses_positive_safe_depth() {
    let now = SystemTime::UNIX_EPOCH;
    let config = super::LandmarkAssociationConfig::default();
    let expected_depth_violation = smooth_hinge(1.0 + config.depth_floor, config.depth_softness);
    let safe_depth = -1.0 + expected_depth_violation;
    let expected_detection = Point2::new(1.0 / safe_depth, 0.0);

    let mut factor = LandmarkFactor::new(
        now,
        now + Duration::from_secs(1),
        vec![vec![VisualMeasurement {
            time: now,
            detections: vec![expected_detection],
            candidates: vec![Point3::new(1.0, 0.0, -1.0)],
            robot_to_camera: SE3::identity(),
            association_costs: None,
        }]],
        Matrix2::identity(),
    );
    force_real_assignments(&mut factor);

    let residual = factor.residuals_on_spline(
        identity_state(),
        identity_state(),
        CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]),
    );

    let expected_depth_residual = expected_depth_violation / factor.config.depth_sigma;
    assert!(residual[0].abs() < 1e-9);
    assert!(residual[1].abs() < 1e-9);
    assert!((residual[2] - expected_depth_residual).abs() < 1e-9);
}
