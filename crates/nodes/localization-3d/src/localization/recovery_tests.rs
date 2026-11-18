use super::*;
use booster::ImuState;
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::{IntoTransform, Vector3, point, vector};
use nalgebra::{Translation3, UnitQuaternion};
use std::time::Duration;
use types::{
    odometry::KinematicOdometryDelta,
    visual_localization::FieldMarkAssociation,
    visual_odometry::{VisualOdometer, VisualOdometryDelta},
};

use crate::test_support::{camera, parameters};
fn turn(t: f64) -> (f64, f64) {
    let u = ((t - 3.0) / 0.6).clamp(0.0, 1.0);
    (0.6 * u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u))
}

fn truth(t: f64) -> nalgebra::Isometry3<f64> {
    nalgebra::Isometry3::from_parts(
        Translation3::new(-2.0 + 0.9 * t, 1.0, 0.55),
        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.4 + turn(t).0),
    )
}

fn frame(t: f64, half_turn: bool) -> TimeWrapper<VisualLocalizationFrame> {
    let camera = camera();
    TimeWrapper {
        time: Time::from_nanos(1_000_000_000 + (t * 1e9).round() as i64),
        inner: VisualLocalizationFrame {
            epoch: 7,
            source: VisualAssociationSource::Global,
            generation: if t == 0.0 { 0 } else { 1 },
            robot_to_camera: camera.robot_to_camera,
            camera_intrinsic: camera.intrinsics,
            associations: [
                point![-3.0, 0.0, 0.0],
                point![3.0, 0.0, 0.0],
                point![0.0, 3.0, 0.0],
            ]
            .map(|field_point| {
                let p = camera.robot_to_camera.inner
                    * truth(t).cast::<f32>().inverse()
                    * field_point.inner;
                FieldMarkAssociation {
                    field_point: if half_turn {
                        point![-field_point.x(), -field_point.y(), 0.0]
                    } else {
                        field_point
                    },
                    detection: camera.intrinsics.project(Vector3::wrap(p.coords)),
                }
            })
            .to_vec(),
        },
    }
}

fn ingest_motion(localization: &mut Localization, index: u64, with_vo: bool) {
    let t = index as f64 * 0.01;
    let time = Time::from_nanos(1_000_000_000) + Duration::from_millis(index * 10);
    let (yaw, rate) = turn(t);
    localization
        .ingest_imu(
            time,
            ImuState {
                // Deliberately different IMU and Local yaw zeros; only changes are observations.
                roll_pitch_yaw: vector![0.0, 0.0, (2.7 + yaw) as f32],
                angular_velocity: vector![0.0, 0.0, rate as f32],
                ..Default::default()
            },
        )
        .unwrap();
    if index == 0 {
        return;
    }
    let previous_time = time - Duration::from_millis(10);
    let previous = truth(t - 0.01);
    let delta = previous.inverse() * truth(t);
    assert!(
        localization
            .ingest_kinematic_odometry(KinematicOdometryDelta {
                previous_time,
                time,
                current_to_previous: nalgebra::Isometry2::new(
                    delta.translation.vector.xy().cast(),
                    0.0
                )
                .framed_transform(),
            })
            .unwrap()
    );
    if with_vo {
        let camera = camera();
        let camera_delta = camera.robot_to_camera.inner
            * delta.cast::<f32>()
            * camera.robot_to_camera.inner.inverse();
        assert!(
            localization
                .ingest_visual_odometry(
                    VisualOdometer {
                        time,
                        epoch: 4,
                        delta: Some(VisualOdometryDelta {
                            previous_time,
                            current_left_camera_to_previous_left_camera: camera_delta
                        }),
                        current_left_camera_to_visual_odometer: nalgebra::Isometry3::identity(),
                    },
                    Some(&camera),
                    Some(&camera)
                )
                .unwrap()
        );
    }
}

fn new_localization() -> Localization {
    new_localization_with_parameters(&parameters())
}

fn new_localization_with_parameters(parameters: &Localization3dParameters) -> Localization {
    let initial = frame(0.0, false);
    Localization::new(
        initial.time,
        7,
        parameters,
        &FieldDimensions::SPL_2025,
        &camera(),
        nalgebra::Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.55),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 2.0),
        )
        .framed_transform(),
    )
    .unwrap()
}

fn lost_localization(with_vo: bool) -> (Localization, LocalizationEstimate) {
    let mut localization = new_localization();
    let mut latest = None;
    assert!(
        localization
            .ingest_visual_localization_frame(frame(0.0, false))
            .unwrap()
    );
    for index in 0..=360 {
        ingest_motion(&mut localization, index, with_vo);
        if index == 20 {
            let mut confirmation = frame(0.2, false);
            confirmation.inner.source = VisualAssociationSource::Tracking;
            assert!(
                localization
                    .ingest_visual_localization_frame(confirmation)
                    .is_ok_and(|inserted| inserted)
            );
        }
        if index > 0 && index % 5 == 0 {
            let time = Time::from_nanos(1_000_000_000) + Duration::from_millis(index * 10);
            let solved = localization.solve(time);
            assert!(solved.estimate.is_some(), "{:?}", solved.diagnostics);
            latest = solved.estimate;
            if index == 20 {
                assert_eq!(localization.lifecycle.state, LocalizationState::Tracking);
            }
        }
    }
    assert_eq!(localization.lifecycle.state, LocalizationState::LostTrack);
    (localization, latest.unwrap())
}

#[test]
fn startup_preserves_delayed_imu_brackets_and_waits_for_missing_samples() -> Result<()> {
    for delayed in [false, true] {
        let mut localization = new_localization();
        let last = if delayed { 20 } else { 5 };
        for index in 0..=last {
            ingest_motion(&mut localization, index, false);
        }
        let mut startup = frame(0.055, false);
        startup.inner.generation = 0;
        assert!(localization.ingest_visual_localization_frame(startup)?);
        if !delayed {
            localization.solve(Time::from_nanos(1_055_000_000));
            assert_eq!(localization.lifecycle.state, LocalizationState::Startup);
            assert!(localization.lifecycle.pending_bootstrap.is_some());
            for index in 6..=20 {
                ingest_motion(&mut localization, index, false);
            }
        }
        let output = localization.solve(Time::from_nanos(1_200_000_000));
        assert_eq!(
            localization.lifecycle.state,
            LocalizationState::Tracking,
            "{:?}",
            output.diagnostics
        );
        let estimate = output.estimate.unwrap();
        assert_eq!(estimate.time, Time::from_nanos(1_200_000_000));
        let pose = estimate.robot_to_field.unwrap().pose.inner;
        assert!((pose.translation.vector - truth(0.2).translation.vector).norm() < 0.03);
        assert!(
            localization.status().heading.is_none(),
            "initialization alone cannot constrain recovery heading"
        );
    }
    Ok(())
}

#[test]
fn global_frames_confirm_startup_after_alignment() -> Result<()> {
    let mut localization = new_localization();
    for index in 0..=20 {
        ingest_motion(&mut localization, index, false);
    }
    let now = Time::from_nanos(1_200_000_000);
    localization.solve(now);
    for (exposure, expected) in [
        (0.0, LocalizationState::Startup),
        (0.2, LocalizationState::Tracking),
    ] {
        let mut source = frame(exposure, false);
        source.inner.generation = localization.status().generation;
        assert!(localization.ingest_visual_localization_frame(source)?);
        let output = localization.solve(now);
        assert_eq!(localization.status().generation, 1);
        assert_eq!(
            localization.status().state,
            expected,
            "{:?}",
            output.diagnostics
        );
    }
    Ok(())
}

#[test]
fn global_confirmation_retains_the_graph_and_corroborates_heading() -> Result<()> {
    let mut localization = new_localization();
    for index in 0..=20 {
        ingest_motion(&mut localization, index, false);
    }
    localization.solve(Time::from_nanos(1_200_000_000));
    assert_eq!(localization.status().state, LocalizationState::Startup);
    let prediction = localization
        .height_prediction(Time::from_nanos(1_200_000_000))
        .ok_or_else(|| eyre!("startup local height must be available without field alignment"))?;
    assert!((prediction.height - 0.55).abs() < 0.01);
    assert!(prediction.variance.is_finite() && prediction.variance > 0.0);
    assert!(localization.ingest_visual_localization_frame(frame(0.0, false))?);
    localization.solve(Time::from_nanos(1_200_000_000));
    assert_eq!(localization.status().state, LocalizationState::Startup);
    assert_eq!(localization.status().generation, 1);
    let mut initial = frame(0.2, false);
    initial.inner.generation = localization.status().generation;
    assert!(localization.ingest_visual_localization_frame(initial)?);
    localization.solve(Time::from_nanos(1_200_000_000));
    assert_eq!(localization.status().state, LocalizationState::Tracking);
    assert!(localization.status().heading.is_none());

    for (end, count) in [(25, 1), (30, 3)] {
        for index in end - 4..=end {
            ingest_motion(&mut localization, index, false);
        }
        let mut observation = frame(end as f64 * 0.01, false);
        observation.inner.source = if count == 1 {
            VisualAssociationSource::Tracking
        } else {
            VisualAssociationSource::Global
        };
        observation.inner.associations.truncate(count);
        let time = observation.time;
        assert!(localization.ingest_visual_localization_frame(observation)?);
        let output = localization.solve(time);
        assert_eq!(
            localization.status().heading.is_some(),
            count == 3,
            "{:?}, latest={:?}",
            output.diagnostics,
            localization.lifecycle.latest_visual
        );
        assert_eq!(localization.status().generation, 1);
    }
    Ok(())
}

#[test]
fn sparse_frames_restore_tracking_only_after_non_collinear_window_evidence() -> Result<()> {
    for batched in [false, true] {
        let (mut localization, _) = lost_localization(false);
        for (start, end, landmark) in [(361, 365, 0), (366, 370, 1), (371, 375, 2)] {
            for index in start..=end {
                ingest_motion(&mut localization, index, false);
            }
            let mut sparse = frame(end as f64 * 0.01, false);
            sparse.inner.source = VisualAssociationSource::Tracking;
            sparse.inner.associations = vec![sparse.inner.associations[landmark]];
            let time = sparse.time;
            assert!(localization.ingest_visual_localization_frame(sparse)?);
            if !batched || landmark == 2 {
                localization.solve(time);
            }
            assert_eq!(
                localization.status().state == LocalizationState::Tracking,
                landmark == 2
            );
        }
    }
    Ok(())
}

#[test]
fn delayed_recovery_preserves_motion_and_publishes_only_after_acceptance() {
    for with_vo in [false, true] {
        let (mut localization, before) = lost_localization(with_vo);
        // Between IMU samples, and inside both odometry intervals. The turn ends
        // before delivery, so later zero-rate IMU readings cannot reconstruct it.
        let recovery = frame(3.375, true);
        assert!(
            localization
                .ingest_visual_localization_frame(recovery.clone())
                .unwrap()
        );
        assert_eq!(localization.lifecycle.state, LocalizationState::LostTrack);
        assert_eq!(localization.latest_time, before.time);
        assert_eq!(localization.status().generation, before.generation);
        ingest_motion(&mut localization, 361, with_vo);
        let now = Time::from_nanos(4_610_000_000);
        let solved = localization.solve(now);
        assert_eq!(
            localization.lifecycle.state,
            LocalizationState::Tracking,
            "{:?}",
            solved.diagnostics
        );
        let estimate = solved.estimate.unwrap();
        assert_eq!(estimate.time, now);
        assert_eq!(estimate.generation, before.generation);
        assert_eq!(estimate.generation, localization.status().generation);
        let pose = estimate.robot_to_field.unwrap().pose.inner;
        assert!(
            (pose.translation.vector - truth(3.61).translation.vector).norm() < 0.03,
            "{pose:?}"
        );
        assert!(
            pose.rotation.angle_to(&truth(3.61).rotation) < 0.03,
            "{pose:?}"
        );
        assert!(
            pose.translation.vector.x > 0.0,
            "recovery changed field half"
        );
        let mut stale_tracking = recovery.clone();
        stale_tracking.inner.source = VisualAssociationSource::Tracking;
        assert!(
            !localization
                .ingest_visual_localization_frame(stale_tracking)
                .unwrap()
        );
        assert!(
            !localization
                .ingest_visual_localization_frame(recovery)
                .unwrap()
        );

        // Recovery updates the same trajectory, including its marginalized history.
        for index in 362..=370 {
            ingest_motion(&mut localization, index, with_vo);
        }
        let mut tracking = frame(3.7, false);
        tracking.inner.generation = localization.generation;
        tracking.inner.source = VisualAssociationSource::Tracking;
        assert!(
            localization
                .ingest_visual_localization_frame(tracking)
                .unwrap()
        );
        let solved = localization.solve(Time::from_nanos(4_700_000_000));
        let pose = solved.estimate.unwrap().robot_to_field.unwrap().pose.inner;
        assert!((pose.translation.vector - truth(3.7).translation.vector).norm() < 0.03);
        assert_eq!(localization.lifecycle.state, LocalizationState::Tracking);
    }
}

#[test]
fn rejected_recovery_does_not_replace_the_active_trajectory() {
    let (mut localization, estimate) = lost_localization(false);
    let now = estimate.time;
    let before = estimate.robot_to_local.pose.inner;
    let mut recovery = frame(3.375, true);
    recovery.inner.associations[0].detection.inner.x += 200.0;
    assert!(
        localization
            .ingest_visual_localization_frame(recovery)
            .unwrap()
    );
    let solved = localization.solve(now);
    assert_eq!(localization.lifecycle.state, LocalizationState::LostTrack);
    let after = solved.estimate.unwrap();
    assert_eq!(after.time, now);
    assert!(
        (after.robot_to_local.pose.inner.translation.vector - before.translation.vector).norm()
            < 0.001
    );
    assert!(
        after
            .robot_to_local
            .pose
            .inner
            .rotation
            .angle_to(&before.rotation)
            < 0.001
    );
    assert!(
        localization
            .ingest_visual_localization_frame(frame(3.375, true))
            .unwrap()
    );
    localization.solve(now);
    assert_eq!(localization.lifecycle.state, LocalizationState::Tracking);
}

#[test]
fn global_fallback_while_tracking_preserves_history_and_rejects_a_translated_match() {
    let (mut localization, before) = lost_localization(true);
    localization
        .ingest_visual_localization_frame(frame(3.375, false))
        .unwrap();
    localization.solve(before.time);
    assert_eq!(localization.lifecycle.state, LocalizationState::Tracking);
    for index in 361..=370 {
        ingest_motion(&mut localization, index, true);
    }
    let mut wrong = frame(3.7, false);
    for association in &mut wrong.inner.associations {
        association.field_point.inner.x += 22.0;
    }
    assert!(
        localization
            .ingest_visual_localization_frame(wrong)
            .unwrap()
    );
    let rejected = localization.solve(Time::from_nanos(4_700_000_000));
    let pose = rejected
        .estimate
        .unwrap()
        .robot_to_field
        .unwrap()
        .pose
        .inner;
    assert!((pose.translation.vector - truth(3.7).translation.vector).norm() < 0.03);
    assert_eq!(
        localization.lifecycle.latest_visual,
        Some(Time::from_nanos(4_375_000_000))
    );
    assert!(
        localization
            .ingest_visual_localization_frame(frame(3.7, false))
            .unwrap()
    );
    let accepted = localization.solve(Time::from_nanos(4_700_000_000));
    let estimate = accepted.estimate.unwrap();
    assert_eq!(estimate.generation, before.generation);
    assert_eq!(
        localization.lifecycle.latest_visual,
        Some(Time::from_nanos(4_700_000_000))
    );
    assert!(
        (estimate
            .robot_to_field
            .unwrap()
            .pose
            .inner
            .translation
            .vector
            - truth(3.7).translation.vector)
            .norm()
            < 0.03
    );
    for index in 371..=380 {
        ingest_motion(&mut localization, index, true);
    }
    // A newer local observation supersedes queued global work before the solve.
    assert!(
        localization
            .ingest_visual_localization_frame(frame(3.75, false))
            .unwrap()
    );
    let mut local = frame(3.8, false);
    local.inner.source = VisualAssociationSource::Tracking;
    assert!(
        localization
            .ingest_visual_localization_frame(local)
            .unwrap()
    );
    localization.solve(Time::from_nanos(4_800_000_000));
    assert_eq!(
        localization.lifecycle.latest_visual,
        Some(Time::from_nanos(4_800_000_000))
    );
    assert!(localization.lifecycle.pending_bootstrap.is_none());
}

#[test]
fn translated_global_match_cannot_discard_the_motion_history() {
    let (mut localization, before) = lost_localization(true);
    let mut wrong = frame(3.375, false);
    // Identical image geometry and heading, but a repeated landmark pattern 22 m away.
    for association in &mut wrong.inner.associations {
        association.field_point.inner.x += 22.0;
    }
    assert!(
        localization
            .ingest_visual_localization_frame(wrong)
            .unwrap()
    );
    let output = localization.solve(before.time);
    let after = output.estimate.expect("retain the motion solution");
    assert_eq!(after.generation, before.generation);
    assert_eq!(localization.lifecycle.state, LocalizationState::LostTrack);
    assert!(
        (after.robot_to_field.unwrap().pose.inner.translation.vector
            - before.robot_to_field.unwrap().pose.inner.translation.vector)
            .norm()
            < 0.03
    );
    assert!(
        localization
            .ingest_visual_localization_frame(frame(3.375, false))
            .unwrap()
    );
    localization.solve(before.time);
    assert_eq!(localization.lifecycle.state, LocalizationState::Tracking);
}

#[test]
fn wrong_half_tracking_update_is_removed_before_publication_or_marginalization() {
    let (mut localization, _) = lost_localization(false);
    let now = Time::from_nanos(4_600_000_000);
    localization
        .ingest_visual_localization_frame(frame(3.375, true))
        .unwrap();
    localization.solve(now);
    assert_eq!(localization.lifecycle.state, LocalizationState::Tracking);
    let reference = localization.lifecycle.heading_reference.unwrap();
    for index in 361..=370 {
        ingest_motion(&mut localization, index, false);
    }
    let mut wrong = frame(3.7, true);
    wrong.inner.generation = localization.generation;
    wrong.inner.source = VisualAssociationSource::Tracking;
    assert!(
        localization
            .ingest_visual_localization_frame(wrong)
            .unwrap()
    );
    let rejected = localization.solve(Time::from_nanos(4_700_000_000));
    assert!(rejected.estimate.is_some(), "{:?}", rejected.diagnostics);
    assert!(localization.lifecycle.pending_visual.is_none());
    assert_eq!(
        localization.lifecycle.latest_visual,
        Some(Time::from_nanos(4_375_000_000))
    );
    // The failed frame is no longer in the graph: a motion-only solve recovers
    // immediately, and subsequent marginalization cannot freeze its bad factors.
    for index in 371..=600 {
        ingest_motion(&mut localization, index, false);
        if index % 5 == 0 {
            let output = localization
                .solve(Time::from_nanos(1_000_000_000) + Duration::from_millis(index * 10));
            let pose = output
                .estimate
                .expect("motion graph remains usable")
                .robot_to_field
                .unwrap()
                .pose
                .inner;
            assert!(pose.rotation.angle_to(&truth(index as f64 * 0.01).rotation) < 0.03);
        }
    }
    let imu = linear_algebra::Orientation3::from_euler_angles(0.0, 0.0, 3.3);
    assert!(
        reference
            .expected(imu)
            .rotation_to(
                localization
                    .lifecycle
                    .heading_reference
                    .unwrap()
                    .expected(imu)
            )
            .inner
            .angle()
            .abs()
            < 1e-12
    );
}

#[test]
fn double_flight_fuses_zero_specific_force_without_ground_contact() {
    let mut localization = new_localization();
    let mut parameters = localization.parameters.clone();
    parameters.accelerometer = Some(crate::AccelerometerParameters {
        noise_density: 0.03,
        ..Default::default()
    });
    parameters.kinematic_odometry_noise = None;
    parameters.visual_feature_noise_variance = 4.0;
    localization = new_localization_with_parameters(&parameters);
    let flying_pose = |t: f64| {
        let mut pose = truth(t);
        pose.translation.z = 0.55 + 2.0 * t - 0.5 * 9.81 * t * t;
        pose
    };
    let camera = camera();
    let mut feet = RobotKinematics::default();
    feet.left_leg.sole_to_robot = Isometry3::from_translation(0.0, 0.05, -0.3);
    feet.right_leg.sole_to_robot = Isometry3::from_translation(0.0, -0.05, -0.3);
    let mut observed_peak = 0.0_f64;
    for index in 0..=200 {
        let t = index as f64 * 0.002;
        let time = Time::from_nanos(1_000_000_000 + index * 2_000_000);
        localization
            .ingest_imu(
                time,
                ImuState {
                    roll_pitch_yaw: vector![0.0, 0.0, 2.7],
                    // Specific force is zero in flight, not +g or a missing measurement.
                    linear_acceleration: Vector3::zeros(),
                    ..Default::default()
                },
            )
            .unwrap();
        if index % 10 != 0 {
            continue;
        }
        localization
            .ingest_kinematics(TimeWrapper {
                time,
                inner: feet.clone(),
            })
            .unwrap();
        if index > 0 {
            let delta = flying_pose(t - 0.02).inverse() * flying_pose(t);
            localization
                .ingest_visual_odometry(
                    VisualOdometer {
                        time,
                        epoch: 1,
                        delta: Some(VisualOdometryDelta {
                            previous_time: time - Duration::from_millis(20),
                            current_left_camera_to_previous_left_camera: camera
                                .robot_to_camera
                                .inner
                                * delta.cast::<f32>()
                                * camera.robot_to_camera.inner.inverse(),
                        }),
                        current_left_camera_to_visual_odometer: nalgebra::Isometry3::identity(),
                    },
                    Some(&camera),
                    Some(&camera),
                )
                .unwrap();
        }
        if index % 50 == 0 {
            let mut image = frame(t, false);
            image.inner.generation = localization.status().generation;
            image.inner.source = if localization.lifecycle.state == LocalizationState::Tracking {
                VisualAssociationSource::Tracking
            } else {
                VisualAssociationSource::Global
            };
            for a in &mut image.inner.associations {
                let p = camera.robot_to_camera.inner
                    * flying_pose(t).cast::<f32>().inverse()
                    * a.field_point.inner;
                a.detection = camera.intrinsics.project(Vector3::wrap(p.coords));
            }
            localization
                .ingest_visual_localization_frame(image)
                .unwrap();
        }
        let output = localization.solve(time);
        if let Some(pose) = output.estimate.and_then(|e| e.robot_to_field) {
            let z = pose.pose.translation().z();
            assert!(
                (z - flying_pose(t).translation.z).abs() < 0.05,
                "t={t}, z={z}, {:?}",
                output.diagnostics
            );
            observed_peak = observed_peak.max(z);
        }
    }
    assert_eq!(localization.lifecycle.state, LocalizationState::Tracking);
    assert!(
        observed_peak > 0.72,
        "the estimator must allow both soles above the floor"
    );
}

#[test]
fn field_inconsistency_does_not_stop_local_motion_or_forget_heading() {
    let (mut localization, before) = lost_localization(true);
    // Simulate a field solution inconsistent with the independent reference.
    let reference = HeadingReference::new(
        before.time,
        linear_algebra::Orientation2::new(1.8),
        linear_algebra::Orientation3::from_euler_angles(0.0, 0.0, 3.3),
    );
    localization.lifecycle.heading_reference = Some(reference);
    ingest_motion(&mut localization, 361, true);
    let output = localization.solve(Time::from_nanos(4_610_000_000));
    let estimate = output.estimate.unwrap();
    assert!(estimate.robot_to_field.is_none());
    assert_eq!(estimate.generation, before.generation);
    assert_eq!(localization.lifecycle.state, LocalizationState::LostTrack);
    for index in 362..=380 {
        ingest_motion(&mut localization, index, true);
        let output =
            localization.solve(Time::from_nanos(1_000_000_000) + Duration::from_millis(index * 10));
        assert!(output.estimate.is_some(), "{:?}", output.diagnostics);
    }
    let imu = linear_algebra::Orientation3::from_euler_angles(0.0, 0.0, 3.3);
    assert!(
        reference
            .expected(imu)
            .rotation_to(
                localization
                    .lifecycle
                    .heading_reference
                    .unwrap()
                    .expected(imu)
            )
            .angle()
            .abs()
            < 1e-12
    );
}
