use crate::heading::HeadingReference;
use booster::ImuState;
use fagra::OptimizeOptions;
use std::time::Duration;

use linear_algebra::{IntoTransform, point};
use projection::intrinsic::Intrinsic;
use types::camera_geometry::CameraGeometry;

use super::*;

pub(super) fn estimator() -> Localization {
    let camera = CameraGeometry {
        intrinsics: Intrinsic::new(nalgebra::vector![300.0, 300.0], point![160.0, 120.0]),
        ..Default::default()
    };
    Localization::new_motion(
        Time::from_nanos(1_000_000_000),
        7,
        nalgebra::Isometry3::identity().framed_transform(),
        &camera.intrinsics,
        Localization3dParameters {
            timing: Default::default(),
            model: Default::default(),
            solver: Default::default(),
            visual: Default::default(),
            inputs: Default::default(),
            imu_preintegration: Default::default(),
            imu_bias: Default::default(),
            kinematic_odometry_noise: Some(Default::default()),
            accelerometer: None,
            initial_height_sigma: 1.0,
            initial_velocity_sigma: 5.0,
            recovery_height_gate: 9.0,
            max_tilt_error: 20.0_f64.to_radians(),
            accelerometer_process_noise_variance: 10.0,
            visual_feature_noise_variance: 100.0,
            field_containment_sigma: 1.0,
            max_heading_error: 20.0_f64.to_radians(),
            max_heading_reference_drift_per_second: 0.5_f64.to_radians(),
            tracking_timeout: Duration::from_secs(2),
            visual_tracking_timeout: Duration::from_secs(2),
        },
        &FieldDimensions::SPL_2025,
    )
    .unwrap()
}

#[test]
fn configured_foot_noise_changes_measurement_weight() {
    let template = estimator();
    let costs = [0.01, 0.04].map(|sigma| {
        let mut parameters = template.parameters.clone();
        parameters.model.foot_sigma = sigma;
        let mut estimator = Localization::new_motion(
            template.origin,
            template.epoch,
            Isometry3::identity(),
            &Intrinsic::default(),
            parameters,
            &FieldDimensions::SPL_2025,
        )
        .unwrap();
        estimator
            .insert_feet(
                estimator.origin,
                point![0.0, 0.0, -0.02],
                point![0.0, 0.0, -0.02],
            )
            .unwrap();
        let evaluate = OptimizeOptions {
            gradient_tolerance: f64::MAX,
            ..estimator.solve_options()
        };
        estimator
            .optimizer
            .solve_batch(&mut estimator.graph, &evaluate)
            .unwrap()
            .initial_cost
    });
    // Two 2-cm penetrations: quadrupling sigma divides their likelihood cost by 16.
    assert!((costs[0] - 4.0).abs() < 1.0e-10);
    assert!((costs[0] / costs[1] - 16.0).abs() < 1.0e-10);
}

#[test]
fn sustained_fast_turn_preserves_off_grid_camera_estimates() {
    let mut estimator = estimator();
    let mut max_error = 0.0_f64;
    for index in 0..=1500 {
        let t = index as f64 * 0.002;
        estimator
            .ingest_imu(
                estimator.origin + Duration::from_millis(index * 2),
                ImuState {
                    roll_pitch_yaw: vector![0.0, 0.0, (4.0 * t) as f32],
                    angular_velocity: vector![0.0, 0.0, 4.0],
                    ..Default::default()
                },
            )
            .unwrap();
        if index == 0 || index % 25 != 0 {
            continue;
        }
        let solved = estimator.solve_motion();
        assert!(solved.estimate.is_some(), "{:?}", solved.diagnostics);
        let exposure = estimator.latest_time - Duration::from_millis(37);
        let (segment, tau) = estimator.segment_and_tau(exposure).unwrap();
        let controls = control_keys(&estimator.controls, segment).unwrap();
        let spline = estimator.spline(controls).unwrap();
        let sample = spline.linearize().unwrap().pose(tau).unwrap();
        let expected = nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 4.0 * (t - 0.037));
        max_error = max_error.max(sample.pose.inner.rotation.angle_to(&expected));
        let covariance = estimator
            .graph
            .joint_covariance(&controls.map(|key| key.block_id()))
            .unwrap();
        let covariance = SMatrix::<f64, 24, 24>::from_fn(|r, c| covariance[(r, c)]);
        assert!(sample.covariance(&covariance).cholesky().is_some());
    }
    eprintln!(
        "off-grid camera-time fast-turn max error: {:.6} deg",
        max_error.to_degrees()
    );
    // The unchanged zero-rotation process prior biases sustained blind turns:
    // the raw-IMU baseline reaches 6.351317 degrees in this same experiment.
    assert!(max_error < 6.5_f64.to_radians());
}

#[test]
fn stationary_imu_produces_timestamped_local_estimate() {
    let mut estimator = estimator();
    // The initial graph is underconstrained: a completed numerical report
    // must survive covariance failure and must not leak into the next attempt.
    let failed = estimator.solve_motion();
    assert!(failed.estimate.is_none());
    assert!(failed.diagnostics.failure.is_some());
    assert_eq!(failed.diagnostics.initial_cost, Some(0.0));
    assert_eq!(failed.diagnostics.final_cost, Some(0.0));
    assert_eq!(failed.diagnostics.iterations, Some(0));
    for index in 0..100 {
        let time = Time::from_nanos(1_000_000_000 + index * 2_000_000);
        assert!(estimator.ingest_imu(time, ImuState::default(),).unwrap());
    }
    let result = estimator.solve_motion();
    assert!(
        result.diagnostics.failure.is_none(),
        "{:?}",
        result.diagnostics
    );
    let estimate = result.estimate.unwrap();
    assert_eq!(estimate.time, Time::from_nanos(1_198_000_000));
    assert_eq!(estimate.epoch, 7);
    assert!(estimate.robot_to_field.is_none());
    assert!(
        estimate
            .robot_to_local
            .covariance
            .iter()
            .all(|value| value.is_finite())
    );
    // A second solve must not retain the previous temporary yaw constraint.
    let repeated = estimator.solve_motion().estimate.unwrap();
    assert!(
        (estimate.robot_to_local.covariance - repeated.robot_to_local.covariance).amax() < 1.0e-10
    );
    // Crossing the knot boundary must retain observability.
    let boundary = Time::from_nanos(1_200_000_000);
    estimator.ingest_imu(boundary, ImuState::default()).unwrap();
    assert_eq!(estimator.solve_motion().estimate.unwrap().time, boundary);
}

#[test]
fn partial_motion_solve_rolls_back_and_later_solve_converges() -> Result<()> {
    let mut estimator = estimator();
    for index in 0..100 {
        estimator.ingest_imu(
            estimator.origin + Duration::from_millis(index * 2),
            ImuState {
                roll_pitch_yaw: vector![0.02, -0.01, 0.0],
                ..Default::default()
            },
        )?;
    }
    let accepted = estimator.solve_motion();
    assert!(accepted.estimate.is_some(), "{:?}", accepted.diagnostics);
    let accepted_time = estimator.last_converged_time;
    estimator.ingest_imu(
        estimator.latest_time + Duration::from_millis(2),
        ImuState {
            roll_pitch_yaw: vector![0.03, -0.02, 0.0],
            ..Default::default()
        },
    )?;
    let controls = estimator
        .controls
        .values()
        .map(|&key| Ok((key, estimator.graph.get(key)?.pose)))
        .collect::<Result<Vec<_>>>()?;
    let solver = estimator.parameters.solver.clone();
    // Force an accepted LM step that does not reach convergence.
    estimator.parameters.solver.max_iterations = 1;
    estimator.parameters.solver.gradient_tolerance = 1e-30;
    let partial = estimator.solve_motion();
    assert!(partial.estimate.is_none(), "{:?}", partial.diagnostics);
    assert!(partial.diagnostics.failure.is_some());
    assert_eq!(partial.diagnostics.termination, "MaxIterations");
    assert_eq!(partial.diagnostics.iterations, Some(1));
    assert!(partial.diagnostics.final_cost.is_some());
    assert!(partial.diagnostics.initial_cost.is_none());
    assert_eq!(estimator.last_converged_time, accepted_time);
    for (key, pose) in controls {
        assert_eq!(estimator.graph.get(key)?.pose, pose);
    }

    estimator.parameters.solver = solver;
    // A premature small-step stop must still meet the gradient criterion
    // within the remaining budget, rather than publish a partial fit.
    estimator.parameters.solver.step_tolerance = f64::MAX;
    estimator.parameters.solver.cost_tolerance = 0.0;
    let recovered = estimator.solve_motion();
    assert!(recovered.estimate.is_some(), "{:?}", recovered.diagnostics);
    assert!(
        recovered
            .diagnostics
            .iterations
            .is_some_and(|steps| steps > 1 && steps <= estimator.parameters.solver.max_iterations)
    );
    assert!(
        recovered
            .diagnostics
            .gradient_norm
            .is_some_and(|gradient| gradient <= estimator.parameters.solver.gradient_tolerance)
    );
    assert_eq!(estimator.last_converged_time, Some(estimator.latest_time));
    Ok(())
}

#[test]
fn late_imu_rebuild_preserves_cost_and_information() {
    let build = |late: bool| {
        let mut estimator = estimator();
        estimator.parameters.accelerometer = Some(Default::default());
        let mut indices: Vec<_> = (0..100).collect();
        if late {
            indices.swap(24, 70);
        }
        for index in indices {
            estimator
                .ingest_imu(
                    estimator.origin + Duration::from_millis(index * 2),
                    ImuState {
                        angular_velocity: vector![0.1, -0.2, index as f32 * 0.001],
                        linear_acceleration: vector![0.2, -0.1, 9.81],
                        ..Default::default()
                    },
                )
                .unwrap();
            if late && index == 70 {
                estimator.prepare_preintegration().unwrap();
            }
        }
        estimator.prepare_preintegration().unwrap();
        estimator.prepare_attitude().unwrap();
        estimator
    };
    let mut grouped = build(false);
    let mut singletons = build(true);
    let evaluate = OptimizeOptions {
        gradient_tolerance: f64::MAX,
        ..grouped.solve_options()
    };
    let grouped_cost = grouped
        .optimizer
        .solve_batch(&mut grouped.graph, &evaluate)
        .unwrap()
        .initial_cost;
    let singleton_cost = singletons
        .optimizer
        .solve_batch(&mut singletons.graph, &evaluate)
        .unwrap()
        .initial_cost;
    assert!((grouped_cost - singleton_cost).abs() < 1e-12);
    let grouped_controls = control_keys(&grouped.controls, 0).unwrap();
    let singleton_controls = control_keys(&singletons.controls, 0).unwrap();
    let grouped_blocks = grouped
        .estimate_covariance_blocks(grouped_controls)
        .unwrap();
    let singleton_blocks = singletons
        .estimate_covariance_blocks(singleton_controls)
        .unwrap();
    let a = grouped.graph.joint_covariance(&grouped_blocks).unwrap();
    let b = singletons
        .graph
        .joint_covariance(&singleton_blocks)
        .unwrap();
    for r in 0..a.nrows() {
        for c in 0..a.ncols() {
            assert!((a[(r, c)] - b[(r, c)]).abs() < 1e-9 * (1.0 + a[(r, c)].abs()));
        }
    }
}

#[test]
fn kinematic_odometry_stops_blind_motion_and_survives_marginalization() {
    use types::odometry::KinematicOdometryDelta;
    let run = |observe_stop: bool| {
        let mut estimator = estimator();
        let origin = estimator.origin;
        let mut last_delta = None;
        for index in 0..=80 {
            let time = origin + Duration::from_millis(index * 50);
            estimator.ingest_imu(time, ImuState::default()).unwrap();
            if index > 0 && (index <= 20 || observe_stop) {
                let delta = KinematicOdometryDelta {
                    previous_time: time - Duration::from_millis(50),
                    time,
                    current_to_previous: linear_algebra::Isometry2::from_parts(
                        vector![if index <= 20 { 0.02 } else { 0.0 }, 0.0],
                        0.0,
                    ),
                };
                assert!(estimator.ingest_kinematic_odometry(delta).unwrap());
                last_delta = Some(delta);
            }
            if index == 0 {
                continue;
            }
            let solved = estimator.solve_motion();
            assert!(
                solved.diagnostics.failure.is_none(),
                "{:?}",
                solved.diagnostics
            );
        }
        assert!(
            !estimator
                .ingest_kinematic_odometry(last_delta.unwrap())
                .unwrap()
        );
        let (segment, tau) = estimator.segment_and_tau(estimator.latest_time).unwrap();
        let controls = control_keys(&estimator.controls, segment).unwrap();
        estimator.spline(controls).unwrap().state(tau).unwrap()
    };
    let stopped = run(true);
    assert!(stopped.velocity.norm() < 0.02, "{stopped:?}");
    assert!(
        (stopped.pose.inner.translation.vector.x - 0.4).abs() < 0.05,
        "{stopped:?}"
    );
    let unobserved = run(false);
    assert!(unobserved.velocity.x() > 0.2, "{unobserved:?}");
    assert!(
        unobserved.pose.inner.translation.vector.x > 1.0,
        "{unobserved:?}"
    );
}

#[test]
fn head_motion_odometry_does_not_move_stationary_body_without_ground_geometry() {
    use types::visual_odometry::{VisualOdometer, VisualOdometryDelta};

    let mut estimator = estimator();
    let camera = |angle| CameraGeometry {
        robot_to_camera: Isometry3::wrap(
            nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::new(0.05, 0.0, 0.25),
                nalgebra::UnitQuaternion::from_euler_angles(0.0, angle, 0.0),
            )
            .inverse(),
        ),
        ..Default::default()
    };
    let mut previous = camera(0.0);
    for index in 0..100 {
        let time = Time::from_nanos(1_000_000_000 + index * 2_000_000);
        estimator.ingest_imu(time, ImuState::default()).unwrap();
        if index > 0 {
            let current = camera(index as f32 * 0.005);
            let delta = previous.robot_to_camera.inner * current.robot_to_camera.inner.inverse();
            assert!(
                estimator
                    .ingest_visual_odometry(
                        VisualOdometer {
                            time,
                            epoch: 0,
                            delta: Some(VisualOdometryDelta {
                                previous_time: time - Duration::from_millis(2),
                                current_left_camera_to_previous_left_camera: delta,
                            }),
                            current_left_camera_to_visual_odometer: current
                                .robot_to_camera
                                .inner
                                .inverse(),
                        },
                        Some(&previous),
                        Some(&current),
                    )
                    .unwrap()
            );
            previous = current;
        }
    }
    let result = estimator.solve_motion();
    assert!(
        result.diagnostics.failure.is_none(),
        "{:?}",
        result.diagnostics
    );
    let pose = result.estimate.unwrap().robot_to_local.pose.inner;
    assert!(pose.translation.vector.norm() < 1e-5, "{pose:?}");
    assert!(pose.rotation.angle() < 1e-5, "{pose:?}");
}

#[test]
fn solve_retires_controls_outside_two_second_window() {
    let mut estimator = estimator();
    for index in 0..=110 {
        estimator
            .ingest_imu(
                Time::from_nanos(1_000_000_000 + index * 20_000_000),
                ImuState::default(),
            )
            .unwrap();
    }
    estimator.solve_motion().estimate.unwrap();
    assert_eq!(estimator.controls.first_key_value().unwrap().0, &0);
    assert_eq!(estimator.segments(), 1..12);
    // Late data inside the retained window changes the graph, not the output time.
    assert!(
        estimator
            .ingest_imu(Time::from_nanos(2_810_000_000), ImuState::default())
            .unwrap()
    );
    assert_eq!(
        estimator.solve_motion().estimate.unwrap().time,
        Time::from_nanos(3_200_000_000)
    );
}

#[test]
fn imu_bias_learns_from_motion_evidence_and_stays_bounded_across_retirement() {
    use types::visual_odometry::{VisualOdometer, VisualOdometryDelta};
    let mut estimator = estimator();
    estimator.parameters.accelerometer = Some(Default::default());
    estimator.parameters.kinematic_odometry_noise = None;
    let camera = CameraGeometry::default();
    let origin = estimator.origin;
    let bias_at = |t: f64| ImuBias {
        gyroscope: vector![0.003 + 0.00005 * t, -0.004, 0.006],
        accelerometer: vector![0.025 + 0.0001 * t, -0.03, 0.015],
    };
    let mut last_bias = None;
    let mut last_pose = None;
    for index in 0..=2000 {
        let t = index as f64 * 0.01;
        let time = origin + Duration::from_millis(index * 10);
        let truth_bias = bias_at(t);
        estimator
            .ingest_imu(
                time,
                ImuState {
                    angular_velocity: Vector3::wrap(truth_bias.gyroscope.inner.cast()),
                    linear_acceleration: Vector3::wrap(
                        (truth_bias.accelerometer.inner + nalgebra::vector![0.1, 0.0, 9.81]).cast(),
                    ),
                    ..Default::default()
                },
            )
            .unwrap();
        if index > 0 && index % 2 == 0 {
            estimator
                .ingest_visual_odometry(
                    VisualOdometer {
                        time,
                        epoch: 0,
                        delta: Some(VisualOdometryDelta {
                            previous_time: time - Duration::from_millis(20),
                            current_left_camera_to_previous_left_camera:
                                nalgebra::Isometry3::translation(
                                    (0.05 * (t * t - (t - 0.02).powi(2))) as f32,
                                    0.0,
                                    0.0,
                                ),
                        }),
                        current_left_camera_to_visual_odometer: nalgebra::Isometry3::identity(),
                    },
                    Some(&camera),
                    Some(&camera),
                )
                .unwrap();
        }
        if index > 0 && index % 5 == 0 {
            let solved = estimator.solve_motion();
            let estimate = solved.estimate.expect("biased but observable motion");
            last_pose = Some(estimate.robot_to_local.pose);
            last_bias = solved.diagnostics.imu_bias;
            assert!(
                estimator.biases.len() <= 3,
                "coarse calibration must retire with the window"
            );
            assert!(estimator.preintegration.interval_count() <= 22);
        }
    }
    let bias = last_bias.unwrap();
    assert!(
        (bias.accelerometer - bias_at(20.0).accelerometer).norm() < 0.012,
        "{bias:?}"
    );
    assert!(
        (bias.gyroscope - bias_at(20.0).gyroscope).norm() < 0.001,
        "{bias:?}"
    );
    // Real acceleration stays in the trajectory, not in the learned bias.
    assert!((last_pose.unwrap().translation().x() - 20.0).abs() < 0.2);
    assert!(estimator.biases.first_key_value().unwrap().0 >= &3);
    assert!(bias.covariance.iter().flatten().all(|v| v.is_finite()));
}

#[test]
fn rejected_visual_frame_does_not_advance_output_time() {
    let mut estimator = estimator();
    assert!(
        !estimator
            .ingest_visual(TimeWrapper {
                time: Time::from_nanos(2_000_000_000),
                inner: VisualLocalizationFrame {
                    epoch: 7,
                    source: types::visual_localization::VisualAssociationSource::Tracking,
                    robot_to_camera: nalgebra::Isometry3::identity().framed_transform(),
                    generation: 0,
                    camera_intrinsic: Intrinsic::new(
                        nalgebra::vector![300.0, 300.0],
                        point![160.0, 120.0],
                    ),
                    associations: Vec::new(),
                },
            })
            .unwrap()
    );
    assert_eq!(estimator.latest_time, Time::from_nanos(1_000_000_000));
}

fn recovery_camera() -> CameraGeometry {
    CameraGeometry {
        robot_to_camera: Isometry3::wrap(
            nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::new(0.12, -0.04, 0.2),
                nalgebra::UnitQuaternion::from_euler_angles(std::f32::consts::PI, 0.0, 0.0),
            )
            .inverse(),
        ),
        intrinsics: Intrinsic::new(nalgebra::vector![300.0, 300.0], point![160.0, 120.0]),
    }
}

#[test]
fn startup_replaces_drifted_height_prior_with_landmark_fit() {
    use types::visual_localization::FieldMarkAssociation;
    let camera = recovery_camera();
    let truth = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(-2.0, 1.0, 0.55),
        nalgebra::UnitQuaternion::from_euler_angles(0.1, -0.05, 0.4),
    );
    let associations: Vec<_> = [
        point![-3.0, 0.0, 0.0],
        point![-1.0, 0.0, 0.0],
        point![-2.0, 2.0, 0.0],
    ]
    .into_iter()
    .map(|field_point| {
        let p = camera.robot_to_camera.inner * truth.inverse() * field_point.inner;
        FieldMarkAssociation {
            field_point,
            detection: camera.intrinsics.project(Vector3::wrap(p.coords)),
        }
    })
    .collect();
    for wrong_height in [0.55, 4.0] {
        let mut estimator = estimator();
        let wrong_pose = Isometry3::wrap(nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(100.0, -50.0, wrong_height),
            nalgebra::UnitQuaternion::from_euler_angles(0.1, -0.05, 0.0),
        ));
        estimator = Localization::new_motion(
            estimator.origin,
            estimator.epoch,
            wrong_pose,
            &camera.intrinsics,
            estimator.parameters.clone(),
            &FieldDimensions::SPL_2025,
        )
        .unwrap();
        let time = Time::from_nanos(1_100_000_000);
        for index in 0..=400 {
            estimator
                .ingest_imu(
                    estimator.origin + Duration::from_millis(index * 2),
                    ImuState {
                        roll_pitch_yaw: vector![0.1, -0.05, 0.0],
                        ..Default::default()
                    },
                )
                .unwrap();
        }
        let origin = estimator.origin;
        estimator = estimator
            .bootstrap_candidate(
                TimeWrapper {
                    time,
                    inner: VisualLocalizationFrame {
                        epoch: 7,
                        source: types::visual_localization::VisualAssociationSource::Tracking,
                        generation: 0,
                        robot_to_camera: camera.robot_to_camera,
                        camera_intrinsic: camera.intrinsics,
                        associations: associations.clone(),
                    },
                },
                None,
            )
            .unwrap()
            .unwrap();
        assert_eq!(estimator.origin, origin);
        assert!(
            estimator
                .segment_factors
                .values()
                .all(|f| f.reprojections.is_empty())
        );
        let controls = control_keys(&estimator.controls, 0).unwrap();
        let seeded = estimator.spline(controls).unwrap().pose(0.0).unwrap();
        assert!((seeded.inner.translation.vector.z - 0.55).abs() < 1e-5);
        let solved = estimator.solve_motion();
        assert!(
            solved.diagnostics.failure.is_none(),
            "{:?}",
            solved.diagnostics
        );
        let field = solved.estimate.unwrap().robot_to_field.unwrap().pose.inner;
        assert!((field.translation.vector - truth.translation.vector.cast::<f64>()).norm() < 1e-4);
        assert!(field.rotation.angle_to(&truth.rotation.cast::<f64>()) < 1e-4);
        assert!(estimator.accepted_visual_rms().unwrap() < 0.01);
        assert!(estimator.pending_visuals.is_empty());
        assert_eq!(
            estimator
                .segment_factors
                .values()
                .map(|f| f.reprojections.len())
                .sum::<usize>(),
            1
        );
    }
}

fn recovery_fixture() -> (
    Localization,
    TimeWrapper<VisualLocalizationFrame>,
    HeadingReference,
) {
    recovery_fixture_with_timing(Default::default())
}

fn recovery_fixture_with_timing(
    timing: crate::TimingParameters,
) -> (
    Localization,
    TimeWrapper<VisualLocalizationFrame>,
    HeadingReference,
) {
    use types::visual_localization::{FieldMarkAssociation, VisualAssociationSource};
    let camera = recovery_camera();
    let truth = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(-2.0, 1.0, 0.55),
        nalgebra::UnitQuaternion::from_euler_angles(0.1, -0.05, 0.4),
    );
    let field_points: [linear_algebra::Point3<coordinate_systems::Field>; 3] = [
        point![-3.0, 0.0, 0.0],
        point![-1.0, 0.0, 0.0],
        point![-2.0, 2.0, 0.0],
    ];
    let associations = field_points
        .into_iter()
        .map(|field_point| {
            let p = camera.robot_to_camera.inner * truth.inverse() * field_point.inner;
            // The global matcher is free to emit the half-turned representative.
            FieldMarkAssociation {
                field_point: point![-field_point.x(), -field_point.y(), 0.0],
                detection: camera.intrinsics.project(Vector3::wrap(p.coords)),
            }
        })
        .collect();
    let mut estimator = estimator();
    let wrong_pose = Isometry3::wrap(nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(10.0, -5.0, 4.0),
        nalgebra::UnitQuaternion::from_euler_angles(0.1, -0.05, 2.0),
    ));
    estimator.parameters.timing = timing;
    estimator = Localization::new_motion(
        estimator.origin,
        7,
        wrong_pose,
        &camera.intrinsics,
        estimator.parameters.clone(),
        &FieldDimensions::SPL_2025,
    )
    .unwrap();
    let time = Time::from_nanos(4_375_000_000);
    let frame = TimeWrapper {
        time,
        inner: VisualLocalizationFrame {
            epoch: 7,
            source: VisualAssociationSource::Global,
            generation: 0,
            robot_to_camera: camera.robot_to_camera,
            camera_intrinsic: camera.intrinsics,
            associations,
        },
    };
    for index in 0..=1800 {
        estimator
            .ingest_imu(
                estimator.origin + Duration::from_millis(index * 2),
                ImuState {
                    roll_pitch_yaw: Vector3::wrap(nalgebra::Vector3::new(0.1, -0.05, 2.0)),
                    linear_acceleration: Vector3::wrap(
                        wrong_pose.inner.rotation.inverse() * nalgebra::vector![0.0, 0.0, 9.81],
                    ),
                    ..Default::default()
                },
            )
            .unwrap();
        if index > 0 && index % 100 == 0 {
            estimator.solve_motion().estimate.unwrap();
        }
    }
    // This whole interval spans exposure and adjacent knots. Moving the knot
    // origin to exposure would make it unsupported or truncate its motion.
    let delta = types::odometry::KinematicOdometryDelta {
        previous_time: Time::from_nanos(4_210_000_000),
        time: Time::from_nanos(4_590_000_000),
        current_to_previous: linear_algebra::Isometry2::identity(),
    };
    assert!(estimator.ingest_kinematic_odometry(delta).unwrap());
    assert!(
        estimator
            .ingest_visual_odometry(
                types::visual_odometry::VisualOdometer {
                    time: delta.time,
                    epoch: 4,
                    delta: Some(types::visual_odometry::VisualOdometryDelta {
                        previous_time: delta.previous_time,
                        current_left_camera_to_previous_left_camera: nalgebra::Isometry3::identity(
                        ),
                    }),
                    current_left_camera_to_visual_odometer: nalgebra::Isometry3::identity(),
                },
                Some(&camera),
                Some(&camera)
            )
            .unwrap()
    );
    // Epoch resets without motion must also survive reconstruction.
    assert!(
        !estimator
            .ingest_visual_odometry(
                types::visual_odometry::VisualOdometer {
                    time: delta.time,
                    epoch: 9,
                    delta: None,
                    current_left_camera_to_visual_odometer: nalgebra::Isometry3::identity(),
                },
                None,
                None
            )
            .unwrap()
    );
    let mut feet = kinematics::robot_kinematics::RobotKinematics::default();
    let offset = wrong_pose.inner.rotation.inverse() * nalgebra::Vector3::new(0.0, 0.0, -0.55);
    feet.left_leg.sole_to_robot.inner.translation.vector = offset;
    feet.right_leg.sole_to_robot.inner.translation.vector = offset;
    assert!(
        estimator
            .ingest_kinematics(TimeWrapper {
                time: delta.time,
                inner: feet
            })
            .unwrap()
    );
    let heading = HeadingReference::new(
        estimator.origin,
        linear_algebra::Orientation2::new(0.4),
        linear_algebra::Orientation3::from_euler_angles(0.1, -0.05, 2.0),
    );
    (estimator, frame, heading)
}

#[test]
fn nondefault_grids_survive_bias_retirement_and_recovery() {
    let timing = crate::TimingParameters {
        trajectory_spacing: Duration::from_millis(400),
        bias_spacing: Duration::from_millis(1200),
        preintegration_interval: Duration::from_millis(50),
        optimization_window: Duration::from_millis(2400),
        ..Default::default()
    };
    let (estimator, frame, heading) = recovery_fixture_with_timing(timing.clone());
    let origin = estimator.origin;
    let latest = estimator.latest_time;
    let (segment, tau) = estimator.segment_and_tau(frame.time).unwrap();
    assert_eq!(segment, 8);
    assert!((tau - 0.4375).abs() < 1e-12);
    assert_eq!(*estimator.controls.first_key_value().unwrap().0, 2);
    assert_eq!(*estimator.biases.first_key_value().unwrap().0, 1);
    assert_eq!(estimator.biases.len(), 4);
    assert!(estimator.preintegration.interval_count() <= 49);
    let bias = estimator.bias_at(frame.time).unwrap();
    let mut candidate = estimator
        .bootstrap_candidate(frame, Some(&heading))
        .unwrap()
        .unwrap();
    assert_eq!(candidate.origin, origin);
    assert_eq!(candidate.parameters.timing, timing);
    assert_eq!(candidate.segment_and_tau(latest).unwrap().0, 9);
    assert!((candidate.bias_at(latest).unwrap().gyroscope - bias.gyroscope).norm() < 1e-6);
    let solved = candidate.solve_with_heading(Some(&heading));
    assert!(solved.estimate.is_some(), "{:?}", solved.diagnostics);
    assert!(candidate.accepted_visual_rms().unwrap() < 0.01);
    // Recovery replay and another retirement must continue on the original grids.
    for millis in (3602..=4000).step_by(2) {
        candidate
            .ingest_imu(
                origin + Duration::from_millis(millis),
                ImuState {
                    roll_pitch_yaw: vector![0.1, -0.05, 2.0],
                    ..Default::default()
                },
            )
            .unwrap();
    }
    let solved = candidate.solve_with_heading(Some(&heading));
    assert!(solved.estimate.is_some(), "{:?}", solved.diagnostics);
    assert_eq!(*candidate.controls.first_key_value().unwrap().0, 3);
    assert!(candidate.preintegration.interval_count() <= 49);
}

#[test]
fn fractional_window_retains_support_for_late_samples_and_recovery() {
    let timing = crate::TimingParameters {
        trajectory_spacing: Duration::from_millis(400),
        bias_spacing: Duration::from_millis(1200),
        preintegration_interval: Duration::from_millis(50),
        optimization_window: Duration::from_millis(2500),
        ..Default::default()
    };
    let (mut estimator, frame, heading) = recovery_fixture_with_timing(timing.clone());
    let latest = estimator.latest_time;
    let cutoff = latest - timing.optimization_window;
    let imu = ImuState {
        roll_pitch_yaw: vector![0.1, -0.05, 2.0],
        ..Default::default()
    };
    assert!(
        !estimator
            .ingest_imu(cutoff - Duration::from_nanos(1), imu)
            .unwrap()
    );
    assert!(estimator.ingest_imu(cutoff, imu).unwrap());
    let solved = estimator.solve_motion();
    assert_eq!(
        solved
            .estimate
            .expect("late sample inside window must retain spline support")
            .time,
        latest
    );
    let mut candidate = estimator
        .bootstrap_candidate(frame, Some(&heading))
        .unwrap()
        .unwrap();
    assert!(candidate.ingest_imu(cutoff, imu).unwrap());
    let solved = candidate.solve_with_heading(Some(&heading));
    assert!(solved.estimate.is_some(), "{:?}", solved.diagnostics);
    assert!(candidate.accepted_visual_rms().unwrap() < 0.01);
}

#[test]
fn imu_preparation_failure_does_not_retry_a_stale_graph_without_visuals() {
    let (estimator, frame, heading) = recovery_fixture();
    let mut candidate = estimator
        .bootstrap_candidate(frame, Some(&heading))
        .unwrap()
        .unwrap();
    assert!(!candidate.pending_visuals.is_empty());
    let mut parameters = candidate.parameters.clone();
    parameters.accelerometer = Some(crate::parameters::AccelerometerParameters {
        scale: nalgebra::vector![1e154, 1.0, 1.0],
        ..Default::default()
    });
    assert!(parameters.validate().is_ok());
    // Deliberate fault injection bypasses the public restart-required update guard.
    candidate.parameters = parameters;
    let start = candidate.latest_time;
    // Finite accepted readings can still overflow covariance propagation.
    for millis in [2, 4, 6] {
        candidate
            .ingest_imu(
                start + Duration::from_millis(millis),
                ImuState {
                    linear_acceleration: vector![f32::MAX, 0.0, 9.81],
                    ..Default::default()
                },
            )
            .unwrap();
    }
    let result = candidate.solve_motion();
    assert!(result.estimate.is_none());
    assert!(!result.visual_rejected);
    assert_eq!(result.diagnostics.iterations, None);
    assert!(
        result
            .diagnostics
            .failure
            .as_deref()
            .unwrap()
            .contains("IMU propagation"),
        "{:?}",
        result.diagnostics
    );
    assert!(candidate.pending_visuals.is_empty());
}

#[test]
fn reconstruction_preserves_motion_and_admission_state() {
    let (mut estimator, frame, heading) = recovery_fixture();
    estimator.parameters.accelerometer = Some(Default::default());
    for nanos in [4_490_000_000, 4_590_000_000] {
        let time = Time::from_nanos(nanos);
        let force = estimator
            .attitude_at(time)
            .unwrap()
            .rotation::<Robot>()
            .inverse()
            * vector![0.0, 0.0, 9.81];
        estimator
            .accept_motion(recovery::MotionRecord::Imu {
                time,
                force,
                angular_velocity: Vector3::zeros(),
                attitude: estimator.attitude_at(time).unwrap(),
            })
            .unwrap();
    }
    let delta = types::odometry::KinematicOdometryDelta {
        previous_time: Time::from_nanos(4_210_000_000),
        time: Time::from_nanos(4_590_000_000),
        current_to_previous: linear_algebra::Isometry2::identity(),
    };
    assert!(estimator.history.records.len() < 1200);
    assert!(estimator.controls.len() < 20);
    let mut candidate = estimator
        .bootstrap_candidate(frame, Some(&heading))
        .unwrap()
        .unwrap();
    assert_eq!(candidate.latest_time, estimator.latest_time);
    candidate.prepare_preintegration().unwrap();
    assert!(candidate.preintegration.interval_count() > 0);
    assert_eq!(candidate.origin, estimator.origin);
    assert_eq!(
        candidate.history.records.len(),
        estimator.history.records.len()
    );
    assert_eq!(candidate.history.latest_vo_epoch, Some(9));
    assert_eq!(candidate.history.latest_kinematic_time, Some(delta.time));
    assert!(!candidate.ingest_kinematic_odometry(delta).unwrap());
    assert_eq!(
        candidate.segment_and_tau(delta.previous_time).unwrap().0 + 1,
        candidate.segment_and_tau(delta.time).unwrap().0
    );
    let recovered = candidate
        .solve_motion()
        .estimate
        .unwrap()
        .robot_to_field
        .unwrap()
        .pose
        .inner;
    assert!((recovered.translation.vector - nalgebra::vector![-2.0, 1.0, 0.55]).norm() < 1e-4);
    assert!(
        recovered
            .rotation
            .angle_to(&nalgebra::UnitQuaternion::from_euler_angles(
                0.1, -0.05, 0.4
            ))
            < 1e-4
    );
}

#[test]
fn bootstrap_height_is_not_a_millimetre_prior() {
    let (estimator, frame, heading) = recovery_fixture();
    let mut candidate = estimator
        .bootstrap_candidate(frame, Some(&heading))
        .unwrap()
        .unwrap();
    let evaluate = OptimizeOptions {
        gradient_tolerance: f64::MAX,
        ..candidate.solve_options()
    };
    let before = candidate
        .optimizer
        .solve_batch(&mut candidate.graph, &evaluate)
        .unwrap()
        .initial_cost;
    for key in candidate.controls.values() {
        let mut control = candidate.graph.get(*key).unwrap().clone();
        control.pose.inner.translation.z += 1.0;
        candidate.graph.set(*key, control).unwrap();
    }
    let after = candidate
        .optimizer
        .solve_batch(&mut candidate.graph, &evaluate)
        .unwrap()
        .initial_cost;
    // Reprojections disagree, but there is no extra 500,000-cost height pseudo-observation.
    // Feet above the floor remain legal: this must not impose a contact equality.
    assert!(
        after > before && after - before < 1000.0,
        "{before} -> {after}"
    );
}

#[test]
fn repeated_heading_rejection_keeps_the_window_bounded() {
    let (estimator, frame, heading) = recovery_fixture();
    let mut candidate = estimator
        .bootstrap_candidate(frame, Some(&heading))
        .unwrap()
        .unwrap();
    candidate.solve_motion().estimate.unwrap();
    let wrong_heading = HeadingReference::new(
        candidate.latest_time,
        linear_algebra::Orientation2::new(-2.0),
        linear_algebra::Orientation3::from_euler_angles(0.1, -0.05, 2.0),
    );
    let mut pending = candidate.latest_visual_frame.clone().unwrap();
    pending.time = candidate.latest_time;
    assert!(candidate.ingest_visual(pending).unwrap());
    for index in 1..=150 {
        candidate
            .ingest_imu(
                Time::from_nanos(4_600_000_000) + Duration::from_millis(index * 20),
                ImuState {
                    roll_pitch_yaw: vector![0.1, -0.05, 2.0],
                    angular_velocity: vector![if index == 1 { 0.03 } else { 0.0 }, 0.0, 0.0],
                    ..Default::default()
                },
            )
            .unwrap();
        let rejected = candidate.solve_with_heading(Some(&wrong_heading));
        assert!(rejected.estimate.unwrap().robot_to_field.is_none());
        assert!(
            candidate.alignment.is_some(),
            "field history must survive rejection"
        );
        assert_eq!(rejected.visual_rejected, index == 1);
        assert!(
            candidate.controls.len() < 20,
            "rejection must still retire the restored graph"
        );
    }
    let recovered = candidate.solve_with_heading(Some(&wrong_heading));
    let estimate = recovered.estimate.expect("retained motion remains usable");
    assert!(estimate.robot_to_field.is_none());
    assert!((estimate.robot_to_local.pose.inner.translation.z - 0.55).abs() < 0.01);
}

#[test]
fn visual_ingestion_accepts_behind_camera_predictions_but_not_zero_range() {
    use types::visual_localization::FieldMarkAssociation;
    let mut estimator = estimator();
    estimator.alignment = Some(estimator.graph.add(FieldAlignment {
        local_to_field: nalgebra::Isometry2::identity().framed_transform(),
    }));
    let time = Time::from_nanos(1_000_000_000);
    let mut frame = TimeWrapper {
        time,
        inner: VisualLocalizationFrame {
            epoch: 7,
            source: types::visual_localization::VisualAssociationSource::Tracking,
            generation: 0,
            robot_to_camera: Isometry3::from_translation(0.0, 0.0, -1.0),
            camera_intrinsic: Intrinsic::new(nalgebra::vector![300.0, 300.0], point![160.0, 120.0]),
            associations: [
                (point![-1.0, -1.0, 0.0], point![150.0, 110.0]),
                (point![1.0, -1.0, 0.0], point![170.0, 110.0]),
                (point![0.0, 0.0, 0.0], point![160.0, 130.0]),
            ]
            .map(|(field_point, detection)| FieldMarkAssociation {
                field_point,
                detection,
            })
            .to_vec(),
        },
    };
    assert!(estimator.ingest_visual(frame.clone()).unwrap());
    // A finite bearing objective does not authorize a physically invalid tracking result.
    assert!(estimator.validate_pending_visuals().is_err());
    assert!(
        estimator
            .segment_factors
            .values()
            .all(|f| f.reprojections.is_empty())
    );
    frame.inner.robot_to_camera = Isometry3::identity();
    assert!(!estimator.ingest_visual(frame).unwrap());
    estimator.discard_visuals().unwrap();
    assert!(estimator.pending_visuals.is_empty());
    assert!(
        estimator
            .segment_factors
            .values()
            .all(|f| f.reprojections.is_empty())
    );
    assert_eq!(
        estimator
            .segment_factors
            .values()
            .map(|f| f.measurements)
            .sum::<usize>(),
        0
    );
}
