use std::{
    error::Error,
    fs,
    path::PathBuf,
    sync::Arc,
    time::{Duration, SystemTime},
};

use booster::ImuState;
use field_mark_association::{
    FieldMarkAssociationParameters, GlobalLocalizerParameters, find_detected_visual_features,
    localize_global_visual_features,
};
use linear_algebra::IntoTransform;
use localization_3d::{
    Localization3dParameters, backend_configuration, initial_state_from_camera_matrix,
};
use localization_factrs::{
    OptimizationResult, VisualReprojectionAssociation,
    backend::OptimizationResult as BackendResult, initialize,
};
use projection::camera_matrix::CameraMatrix;
use ros_z::time::Time;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use types::{
    field_dimensions::{FieldDimensions, Half, Side},
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
};

const RECORDING_RELATIVE_PATH: &str = "../../localization-factrs/localization-3d-sample.jsonl";
const OUTPUT_PATH: &str = "/tmp/localization_3d_replay_trajectory.json";
const ASSUMED_SOLVE_TIME: Duration = Duration::from_millis(100);
const ASSUMED_ROUND_TIME: Duration = Duration::from_millis(2);
// Allow one extra solve after a batch drains, but do not let recording transport gaps
// repeatedly optimize an unchanged graph at the same backend timestamp.
const MAX_IDLE_SOLVES_PER_BACKEND_TIME: usize = 1;
const EXPECTED_IMU_SAMPLE_COUNT: usize = 5981;
const EXPECTED_CAMERA_MATRIX_COUNT: usize = 4923;
const EXPECTED_FIELD_DIMENSIONS_COUNT: usize = 6;
const EXPECTED_DETECTED_OBJECT_FRAME_COUNT: usize = 710;
// Static-height-gated global localizations are ingested only when the landmark
// associations are unique enough to be safe as fixed backend factors.
const EXPECTED_VISUAL_FEATURE_FRAME_COUNT: usize = 75;
const EXPECTED_GLOBAL_LOCALIZATION_DEBUG_FRAME_COUNT: usize = 75;
const EXPECTED_RELAXED_GLOBAL_LOCALIZATION_DEBUG_FRAME_COUNT: usize = 37;
const EXPECTED_GOALPOST_DETECTION_COUNT: usize = 723;
const EXPECTED_SOLVER_SOLUTION_COUNT: usize = 185;
const EXPECTED_OPTIMIZED_SAMPLE_COUNT: usize = 170;
const EXPECTED_RAW_BACKEND_SOLVE_SAMPLE_COUNT: usize = 170;
const EXPECTED_LANDMARK_COUNT: usize = 4;

#[derive(Debug, Deserialize)]
struct RecordingEnvelope {
    topic: String,
    source_time_ns: i64,
    transport_time_ns: Option<i64>,
    message: Value,
}

#[derive(Debug, Serialize)]
struct TrajectoryOutput {
    optimized: Vec<PoseSample>,
    raw_backend_solves: Vec<PoseSample>,
    landmarks: Vec<LandmarkSample>,
}

#[derive(Debug, Serialize)]
struct PoseSample {
    timestamp_seconds: f64,
    position: [f64; 3],
    velocity: [f64; 3],
    quaternion_wxyz: [f64; 4],
}

#[derive(Debug, Serialize)]
struct LandmarkSample {
    name: &'static str,
    position: [f64; 3],
}

#[derive(Debug, Default)]
struct ReplayTrajectories {
    optimized: Vec<PoseSample>,
    raw_backend_solves: Vec<PoseSample>,
}

#[test]
fn real_recording_replay_produces_expected_trajectory_streams() -> Result<(), Box<dyn Error>> {
    let recording = fs::read_to_string(recording_path())?;
    let first_camera_matrix = first_camera_matrix(&recording)?;
    let start_time = first_source_time_for_topic(&recording, "inputs/imu_state")?.to_wallclock();
    let start_arrival_time =
        first_transport_time_for_topic(&recording, "inputs/imu_state")?.to_wallclock();

    let initial_state = initial_state_from_camera_matrix(&first_camera_matrix);

    let parameters = Localization3dParameters::default();
    let association_parameters = FieldMarkAssociationParameters::default();
    let solve_every_nth_round = solve_every_nth_round();
    let solve_cadence = solve_cadence_duration(solve_every_nth_round);
    let (mut frontend, mut backend) = initialize(
        backend_configuration(parameters.visual_feature_noise_variance),
        initial_state,
    );
    let relaxed_debug_global_localizer = GlobalLocalizerParameters {
        min_inliers: 4,
        ..Default::default()
    };
    let mut camera_matrices = Vec::new();
    let mut field_dimensions = None;
    let mut trajectories = ReplayTrajectories::default();
    let mut imu_count = 0;
    let mut camera_matrix_count = 0;
    let mut field_dimensions_count = 0;
    let mut object_frame_count = 0;
    let mut visual_frame_count = 0;
    let mut global_localization_debug_frame_count = 0;
    let mut relaxed_global_localization_debug_frame_count = 0;
    let mut goalpost_detection_count = 0;
    let mut solver_solution_count = 0;
    let mut has_pending_measurements = false;
    let mut idle_solves_at_backend_time = 0;
    let mut next_solve_time = start_arrival_time + solve_cadence;

    for line in recording.lines() {
        let envelope: RecordingEnvelope = serde_json::from_str(line)?;
        let source_time = Time::from_nanos(envelope.source_time_ns);
        let arrival_time = Time::from_nanos(
            envelope
                .transport_time_ns
                .unwrap_or(envelope.source_time_ns),
        );
        solve_pending_until(
            &mut backend,
            &mut frontend,
            start_time,
            &mut trajectories,
            &mut solver_solution_count,
            &mut has_pending_measurements,
            &mut idle_solves_at_backend_time,
            &mut next_solve_time,
            arrival_time.to_wallclock(),
            solve_cadence,
        )?;

        match envelope.topic.as_str() {
            "field_dimensions" => {
                field_dimensions_count += 1;
                let dimensions: FieldDimensions = serde_json::from_value(envelope.message)?;
                field_dimensions = Some(Arc::new(dimensions));
            }
            "camera_matrix" => {
                camera_matrix_count += 1;
                let camera_matrix: TimeWrapper<CameraMatrix> =
                    serde_json::from_value(envelope.message)?;
                camera_matrices.push((source_time, camera_matrix.inner));
            }
            "detected_objects" => {
                object_frame_count += 1;
                let objects: Vec<Object<RobocupObjectLabel>> =
                    serde_json::from_value(envelope.message)?;
                let visual_features = find_detected_visual_features(&objects);
                if visual_features.goalposts.is_empty()
                    && visual_features.l_spots.is_empty()
                    && visual_features.t_spots.is_empty()
                    && visual_features.penalty_spots.is_empty()
                {
                    continue;
                }

                let Some(camera_matrix) = nearest_camera_matrix(&camera_matrices, source_time)
                else {
                    continue;
                };
                let Some(field_dimensions) = field_dimensions.clone() else {
                    continue;
                };

                goalpost_detection_count += visual_features.goalposts.len();
                let pose_hint = frontend
                    .peek_last_optimization_result()
                    .map(|result| result.transform.cast::<f32>().framed_transform());
                let localization = localize_global_visual_features(
                    &visual_features,
                    camera_matrix,
                    field_dimensions.as_ref(),
                    pose_hint,
                    &association_parameters.global_localizer,
                );
                let relaxed_debug_localization = localize_global_visual_features(
                    &visual_features,
                    camera_matrix,
                    field_dimensions.as_ref(),
                    pose_hint,
                    &relaxed_debug_global_localizer,
                );
                if localization.debug.is_some() {
                    global_localization_debug_frame_count += 1;
                }
                if relaxed_debug_localization.debug.is_some() {
                    relaxed_global_localization_debug_frame_count += 1;
                }
                if let Some(associations) = localization.unique_associations {
                    let associations =
                        associations
                            .into_iter()
                            .map(|association| VisualReprojectionAssociation {
                                detection: association.detection,
                                field_point: association.field_point,
                            });
                    frontend.ingest_visual_reprojection_associations(
                        source_time.to_wallclock(),
                        associations,
                        (camera_matrix.head_to_camera * camera_matrix.robot_to_head).inner,
                    )?;
                    visual_frame_count += 1;
                    has_pending_measurements = true;
                }
            }
            "inputs/imu_state" => {
                imu_count += 1;
                let imu: ImuState = serde_json::from_value(envelope.message)?;
                frontend.ingest_imu(source_time.to_wallclock(), imu)?;
                has_pending_measurements = true;
            }
            _ => {}
        }
    }

    if has_pending_measurements {
        solve_and_record(
            &mut backend,
            &mut frontend,
            start_time,
            &mut trajectories,
            &mut solver_solution_count,
        )?;
    }

    let field_dimensions = field_dimensions.ok_or("recording did not contain field dimensions")?;
    let output = TrajectoryOutput {
        optimized: trajectories.optimized,
        raw_backend_solves: trajectories.raw_backend_solves,
        landmarks: goalpost_landmarks(&field_dimensions),
    };

    fs::write(OUTPUT_PATH, serde_json::to_string_pretty(&output)?)?;

    assert_eq!(imu_count, EXPECTED_IMU_SAMPLE_COUNT);
    assert_eq!(camera_matrix_count, EXPECTED_CAMERA_MATRIX_COUNT);
    assert_eq!(field_dimensions_count, EXPECTED_FIELD_DIMENSIONS_COUNT);
    assert_eq!(object_frame_count, EXPECTED_DETECTED_OBJECT_FRAME_COUNT);
    assert_eq!(visual_frame_count, EXPECTED_VISUAL_FEATURE_FRAME_COUNT);
    assert_eq!(
        global_localization_debug_frame_count,
        EXPECTED_GLOBAL_LOCALIZATION_DEBUG_FRAME_COUNT
    );
    assert_eq!(
        relaxed_global_localization_debug_frame_count,
        EXPECTED_RELAXED_GLOBAL_LOCALIZATION_DEBUG_FRAME_COUNT
    );
    assert_eq!(goalpost_detection_count, EXPECTED_GOALPOST_DETECTION_COUNT);
    assert_eq!(solver_solution_count, EXPECTED_SOLVER_SOLUTION_COUNT);
    assert_eq!(output.optimized.len(), EXPECTED_OPTIMIZED_SAMPLE_COUNT);
    assert_eq!(
        output.raw_backend_solves.len(),
        EXPECTED_RAW_BACKEND_SOLVE_SAMPLE_COUNT
    );
    assert_eq!(output.landmarks.len(), EXPECTED_LANDMARK_COUNT);
    assert_trajectory_is_finite_and_ordered("optimized", &output.optimized);
    assert_trajectory_is_finite_and_ordered("raw backend solves", &output.raw_backend_solves);

    println!("replayed {imu_count} IMU samples");
    println!("replayed {object_frame_count} detected object frames");
    println!("ingested {visual_frame_count} visual feature frames");
    println!(
        "computed {global_localization_debug_frame_count} deployed global localization debug frames"
    );
    println!(
        "computed {relaxed_global_localization_debug_frame_count} relaxed global localization debug frames"
    );
    println!("ingested {goalpost_detection_count} goalpost detections");
    println!("ran {solver_solution_count} backend solver iterations");
    println!(
        "exported {} raw backend solve states and {} optimization states",
        output.raw_backend_solves.len(),
        output.optimized.len()
    );
    println!(
        "assumed solve time {:.3} ms, solving every {solve_every_nth_round} IMU-equivalent rounds",
        ASSUMED_SOLVE_TIME.as_secs_f64() * 1000.0,
    );
    println!("wrote {OUTPUT_PATH}");

    Ok(())
}

fn solve_every_nth_round() -> usize {
    let solve_time_ns = ASSUMED_SOLVE_TIME.as_nanos();
    let round_time_ns = ASSUMED_ROUND_TIME.as_nanos();
    assert!(round_time_ns > 0, "assumed round time must be positive");

    solve_time_ns
        .div_ceil(round_time_ns)
        .try_into()
        .expect("solve cadence must fit into usize")
}

fn solve_cadence_duration(solve_every_nth_round: usize) -> Duration {
    let cadence_ns = ASSUMED_ROUND_TIME.as_nanos() * solve_every_nth_round as u128;
    Duration::from_nanos(
        cadence_ns
            .try_into()
            .expect("solve cadence must fit into u64 nanoseconds"),
    )
}

#[allow(clippy::too_many_arguments)]
fn solve_pending_until(
    backend: &mut localization_factrs::VinsBackend,
    frontend: &mut localization_factrs::VinsFrontend,
    start_time: SystemTime,
    trajectories: &mut ReplayTrajectories,
    solver_solution_count: &mut usize,
    has_pending_measurements: &mut bool,
    idle_solves_at_backend_time: &mut usize,
    next_solve_time: &mut SystemTime,
    current_time: SystemTime,
    solve_cadence: Duration,
) -> Result<(), Box<dyn Error>> {
    while *next_solve_time <= current_time {
        if *has_pending_measurements
            || (*solver_solution_count > 0
                && *idle_solves_at_backend_time < MAX_IDLE_SOLVES_PER_BACKEND_TIME)
        {
            let solved_pending_measurements = *has_pending_measurements;
            let backend_result_time = solve_and_record(
                backend,
                frontend,
                start_time,
                trajectories,
                solver_solution_count,
            )?;
            if solved_pending_measurements {
                *idle_solves_at_backend_time = 0;
                *has_pending_measurements = false;
            } else if backend_result_time.is_some() {
                *idle_solves_at_backend_time += 1;
            } else {
                *idle_solves_at_backend_time = MAX_IDLE_SOLVES_PER_BACKEND_TIME;
            }
        }

        *next_solve_time += solve_cadence;
    }

    Ok(())
}

fn solve_and_record(
    backend: &mut localization_factrs::VinsBackend,
    frontend: &mut localization_factrs::VinsFrontend,
    start_time: SystemTime,
    trajectories: &mut ReplayTrajectories,
    solver_solution_count: &mut usize,
) -> Result<Option<SystemTime>, Box<dyn Error>> {
    let backend_result = backend.solve_once()?;
    *solver_solution_count += 1;

    if let Some(result) = backend_result.as_ref() {
        push_backend_result_sample(&mut trajectories.raw_backend_solves, result, start_time);
    }

    let Some(result) = frontend.last_optimization_result() else {
        return Ok(backend_result.as_ref().map(|result| result.time));
    };
    push_result_sample(&mut trajectories.optimized, &result, start_time);

    Ok(backend_result.as_ref().map(|result| result.time))
}

fn recording_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(RECORDING_RELATIVE_PATH)
}

fn first_camera_matrix(recording: &str) -> Result<CameraMatrix, Box<dyn Error>> {
    for line in recording.lines() {
        let envelope: RecordingEnvelope = serde_json::from_str(line)?;
        if envelope.topic == "camera_matrix" {
            let camera_matrix: TimeWrapper<CameraMatrix> =
                serde_json::from_value(envelope.message)?;
            return Ok(camera_matrix.inner);
        }
    }

    Err("recording did not contain camera matrices".into())
}

fn first_source_time_for_topic(recording: &str, topic: &str) -> Result<Time, Box<dyn Error>> {
    for line in recording.lines() {
        let envelope: RecordingEnvelope = serde_json::from_str(line)?;
        if envelope.topic == topic {
            return Ok(Time::from_nanos(envelope.source_time_ns));
        }
    }

    Err(format!("recording did not contain topic {topic}").into())
}

fn first_transport_time_for_topic(recording: &str, topic: &str) -> Result<Time, Box<dyn Error>> {
    for line in recording.lines() {
        let envelope: RecordingEnvelope = serde_json::from_str(line)?;
        if envelope.topic == topic {
            return Ok(Time::from_nanos(
                envelope
                    .transport_time_ns
                    .unwrap_or(envelope.source_time_ns),
            ));
        }
    }

    Err(format!("recording did not contain topic {topic}").into())
}

fn nearest_camera_matrix(
    camera_matrices: &[(Time, CameraMatrix)],
    time: Time,
) -> Option<&CameraMatrix> {
    camera_matrices
        .iter()
        .min_by_key(|(candidate_time, _)| candidate_time.as_nanos().abs_diff(time.as_nanos()))
        .map(|(_, camera_matrix)| camera_matrix)
}

fn push_result_sample(
    trajectory: &mut Vec<PoseSample>,
    result: &OptimizationResult,
    start_time: SystemTime,
) {
    let Some(sample) = pose_sample_from_optimization_result(result, start_time) else {
        return;
    };

    if trajectory
        .last()
        .is_none_or(|last| sample.timestamp_seconds > last.timestamp_seconds)
    {
        trajectory.push(sample);
    }
}

fn push_backend_result_sample(
    trajectory: &mut Vec<PoseSample>,
    result: &BackendResult,
    start_time: SystemTime,
) {
    let Some(sample) = pose_sample_from_backend_result(result, start_time) else {
        return;
    };

    if trajectory
        .last()
        .is_none_or(|last| sample.timestamp_seconds > last.timestamp_seconds)
    {
        trajectory.push(sample);
    }
}

fn pose_sample_from_optimization_result(
    result: &OptimizationResult,
    start_time: SystemTime,
) -> Option<PoseSample> {
    let timestamp_seconds = result.time.duration_since(start_time).ok()?.as_secs_f64();
    let translation = result.transform.translation.vector;
    let rotation = result.transform.rotation.quaternion();
    let velocity = result.transform.rotation * result.velocity;

    Some(PoseSample {
        timestamp_seconds,
        position: [translation.x, translation.y, translation.z],
        velocity: [velocity.x, velocity.y, velocity.z],
        quaternion_wxyz: [rotation.w, rotation.i, rotation.j, rotation.k],
    })
}

fn pose_sample_from_backend_result(
    result: &BackendResult,
    start_time: SystemTime,
) -> Option<PoseSample> {
    let timestamp_seconds = result.time.duration_since(start_time).ok()?.as_secs_f64();
    let translation = result.latest_pose.xyz();
    let velocity = result.latest_pose.uvw();
    let rotation = result.latest_pose.rot();

    Some(PoseSample {
        timestamp_seconds,
        position: [translation.x, translation.y, translation.z],
        velocity: [velocity.x, velocity.y, velocity.z],
        quaternion_wxyz: [rotation.w(), rotation.x(), rotation.y(), rotation.z()],
    })
}

fn goalpost_landmarks(field_dimensions: &FieldDimensions) -> Vec<LandmarkSample> {
    [
        ("opponent_left_goal_post", Half::Opponent, Side::Left),
        ("opponent_right_goal_post", Half::Opponent, Side::Right),
        ("own_left_goal_post", Half::Own, Side::Left),
        ("own_right_goal_post", Half::Own, Side::Right),
    ]
    .into_iter()
    .map(|(name, half, side)| {
        let position = field_dimensions.goal_post(half, side).extend(0.0);
        LandmarkSample {
            name,
            position: [
                position.x() as f64,
                position.y() as f64,
                position.z() as f64,
            ],
        }
    })
    .collect()
}

fn assert_trajectory_is_finite_and_ordered(name: &str, trajectory: &[PoseSample]) {
    assert_trajectory_is_finite(name, trajectory);

    for samples in trajectory.windows(2) {
        assert!(
            samples[0].timestamp_seconds < samples[1].timestamp_seconds,
            "{name} timestamps must be strictly increasing"
        );
    }
}

fn assert_trajectory_is_finite(name: &str, trajectory: &[PoseSample]) {
    for sample in trajectory {
        assert!(
            sample.timestamp_seconds.is_finite(),
            "{name} timestamp must be finite"
        );
        assert!(
            sample
                .position
                .iter()
                .all(|component| component.is_finite()),
            "{name} at {:.3}s must have finite position, got {:?}",
            sample.timestamp_seconds,
            sample.position,
        );
        assert!(
            sample
                .velocity
                .iter()
                .all(|component| component.is_finite()),
            "{name} at {:.3}s must have finite velocity, got {:?}",
            sample.timestamp_seconds,
            sample.velocity,
        );
        assert!(
            sample
                .quaternion_wxyz
                .iter()
                .all(|component| component.is_finite()),
            "{name} at {:.3}s must have finite quaternion, got {:?}",
            sample.timestamp_seconds,
            sample.quaternion_wxyz,
        );
    }
}
