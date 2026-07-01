use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    io::{BufWriter, Error as IoError, ErrorKind, Write},
    path::PathBuf,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use booster::ImuState;
use field_mark_association::{
    GlobalLocalizationDebugStatus, GlobalLocalizerParameters, find_detected_visual_features,
    localize_global_visual_features,
};
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::IntoTransform;
use localization_3d::{
    Localization3dParameters, backend_configuration, ingest_foot_heights, ingest_visual_odometry,
    initial_robot_to_field_from_camera_matrix, initial_state_from_camera_matrix,
};
use localization_factrs::{
    BackendConfiguration, VinsBackend, VinsFrontend, VisualReprojectionAssociation,
    VisualReprojectionAssociationKind, backend::BackendSolveDiagnostics, initialize,
};
use mcap::{Message, MessageStream};
use nalgebra::{SMatrix, SVector};
use projection::camera_matrix::CameraMatrix;
use ros_z::time::Time;
use serde::{Deserialize, Serialize};
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
    visual_odometry::VisualOdometryDelta,
};

#[path = "support/recording_decode.rs"]
mod recording_decode;

use recording_decode::{
    WireIsometry3, decode_recorded_camera_matrix, decode_recorded_message, decode_time_prefix,
};

const DEFAULT_RECORDING_RELATIVE_PATH: &str = "../../../recording.mcap";
const DEFAULT_OUTPUT_DIR: &str = "/tmp/localization_3d_mcap_replay";
const DEFAULT_SOLVE_CADENCE: Duration = Duration::from_millis(30);
const KNOT_SPACING: Duration = Duration::from_millis(200);
const STALE_CAMERA_MATRIX_THRESHOLD: Duration = Duration::from_millis(100);
const KNOT_BOUNDARY_WINDOW: Duration = Duration::from_millis(10);
const GLOBAL_CORRECTION_WINDOW: Duration = Duration::from_millis(500);

#[test]
#[ignore = "replays the large localization MCAP recording and writes evaluation reports"]
fn mcap_replay_evaluates_dead_reckoning_and_graph_variants() -> Result<(), Box<dyn Error>> {
    let recording_path = recording_path();
    let output_dir = output_dir();
    fs::create_dir_all(&output_dir)?;

    let recording = Recording::load(&recording_path)?;
    let field_dimensions = recording
        .field_dimensions
        .unwrap_or(FieldDimensions::SPL_2025);
    let field_dimensions_source = if recording.field_dimensions.is_some() {
        "recording"
    } else {
        "FieldDimensions::SPL_2025 fallback"
    };

    let solve_cadence = solve_cadence();
    let variants = vec![
        VariantConfig::new(
            "no_features_mcap_publish_i1_current_covariance",
            TimestampMode::McapPublish,
            false,
            1,
            VisualOdometryCovarianceMode::Current,
        ),
        VariantConfig::new(
            "with_features_mcap_publish_i1_current_covariance",
            TimestampMode::McapPublish,
            true,
            1,
            VisualOdometryCovarianceMode::Current,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i5_current_covariance",
            TimestampMode::McapPublish,
            false,
            5,
            VisualOdometryCovarianceMode::Current,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i10_current_covariance",
            TimestampMode::McapPublish,
            false,
            10,
            VisualOdometryCovarianceMode::Current,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i20_current_covariance",
            TimestampMode::McapPublish,
            false,
            20,
            VisualOdometryCovarianceMode::Current,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i1_validated_covariance",
            TimestampMode::McapPublish,
            false,
            1,
            VisualOdometryCovarianceMode::ValidatedFactrsOrder,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i5_validated_covariance",
            TimestampMode::McapPublish,
            false,
            5,
            VisualOdometryCovarianceMode::ValidatedFactrsOrder,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i10_validated_covariance",
            TimestampMode::McapPublish,
            false,
            10,
            VisualOdometryCovarianceMode::ValidatedFactrsOrder,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i20_validated_covariance",
            TimestampMode::McapPublish,
            false,
            20,
            VisualOdometryCovarianceMode::ValidatedFactrsOrder,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i5_factrs_order_yaw_strong_covariance",
            TimestampMode::McapPublish,
            false,
            5,
            VisualOdometryCovarianceMode::FactrsOrderYawStrong,
        ),
        VariantConfig::new(
            "no_features_mcap_publish_i5_isotropic_tight_covariance",
            TimestampMode::McapPublish,
            false,
            5,
            VisualOdometryCovarianceMode::IsotropicTight,
        ),
        VariantConfig::new(
            "with_features_mcap_publish_i5_validated_covariance",
            TimestampMode::McapPublish,
            true,
            5,
            VisualOdometryCovarianceMode::ValidatedFactrsOrder,
        ),
        VariantConfig::new(
            "with_features_mcap_publish_i5_isotropic_tight_covariance",
            TimestampMode::McapPublish,
            true,
            5,
            VisualOdometryCovarianceMode::IsotropicTight,
        ),
        VariantConfig::new(
            "no_features_embedded_i1_current_covariance",
            TimestampMode::Embedded,
            false,
            1,
            VisualOdometryCovarianceMode::Current,
        ),
    ];

    let mut samples = Vec::new();
    let mut summaries = Vec::new();
    for variant in filter_variants(variants) {
        println!("running variant {}", variant.name);
        let dead_reckoning = build_dead_reckoning(&recording, variant.timestamp_mode)?;
        let replay = replay_graph_variant(
            &recording,
            &field_dimensions,
            &variant,
            &dead_reckoning,
            solve_cadence,
        )?;
        samples.extend(replay.samples.iter().cloned());
        summaries.push(replay.summary);
        println!("finished variant {}", variant.name);
    }

    let report = ReplayReport {
        recording_path: recording_path.display().to_string(),
        output_dir: output_dir.display().to_string(),
        field_dimensions_source: field_dimensions_source.to_string(),
        solve_cadence_ms: duration_ms(solve_cadence),
        stale_camera_matrix_threshold_ms: duration_ms(STALE_CAMERA_MATRIX_THRESHOLD),
        timestamp_diagnostics: recording.timestamp_diagnostics(),
        topic_counts: recording.topic_counts.clone(),
        variants: summaries,
    };

    write_summary_json(&output_dir, &report)?;
    write_samples_csv(&output_dir, &samples)?;

    println!("wrote {}", output_dir.join("summary.json").display());
    println!("wrote {}", output_dir.join("samples.csv").display());

    Ok(())
}

#[derive(Clone)]
struct Recording {
    events: Vec<RecordedEvent>,
    stereo_image_times: Vec<ImageTimestamp>,
    first_camera_matrix: CameraMatrix,
    field_dimensions: Option<FieldDimensions>,
    topic_counts: BTreeMap<String, usize>,
}

impl Recording {
    fn load(path: &PathBuf) -> Result<Self, Box<dyn Error>> {
        let bytes = fs::read(path)?;
        let mut topic_counts = BTreeMap::new();
        let mut events = Vec::new();
        let mut stereo_image_times = Vec::new();
        let mut first_camera_matrix = None;
        let mut field_dimensions = None;

        for (order, message) in MessageStream::new(&bytes)?.enumerate() {
            let message = message?;
            *topic_counts
                .entry(message.channel.topic.clone())
                .or_default() += 1;

            let log_time = system_time_from_nanos(message.log_time);
            let publish_time = system_time_from_nanos(message.publish_time);
            let event = match message.channel.topic.as_str() {
                "inputs/imu_state" => Some(EventKind::Imu(decode_recorded_message(&message)?)),
                "visual_odometry/current_left_camera_to_previous_left_camera" => Some(
                    EventKind::VisualOdometry(decode_recorded_visual_odometry(&message)?),
                ),
                "robot_kinematics" => Some(EventKind::RobotKinematics(decode_recorded_message(
                    &message,
                )?)),
                "camera_matrix" => {
                    let camera_matrix = decode_recorded_camera_matrix(&message)?;
                    if first_camera_matrix.is_none() {
                        first_camera_matrix = Some(camera_matrix.inner.clone());
                    }
                    Some(EventKind::CameraMatrix(camera_matrix))
                }
                "detected_objects" => Some(EventKind::DetectedObjects(decode_recorded_message(
                    &message,
                )?)),
                "field_dimensions" => {
                    let dimensions = decode_recorded_message(&message)?;
                    field_dimensions = Some(dimensions);
                    Some(EventKind::FieldDimensions)
                }
                "inputs/stereo_image_pair" => {
                    stereo_image_times.push(ImageTimestamp {
                        embedded_time: decode_time_prefix(&message.data)?,
                        publish_time,
                    });
                    None
                }
                _ => None,
            };

            if let Some(kind) = event {
                events.push(RecordedEvent {
                    order,
                    log_time,
                    publish_time,
                    kind,
                });
            }
        }

        events.sort_by_key(|event| (nanos_since_epoch(event.log_time), event.order));

        Ok(Self {
            events,
            stereo_image_times,
            first_camera_matrix: first_camera_matrix.ok_or("recording has no camera_matrix")?,
            field_dimensions,
            topic_counts,
        })
    }

    fn aligned_image_time(&self, embedded_time: Time) -> Option<SystemTime> {
        let embedded_time = embedded_time.to_wallclock();
        self.stereo_image_times
            .iter()
            .min_by_key(|candidate| {
                nanos_abs_diff(candidate.embedded_time.to_wallclock(), embedded_time)
            })
            .map(|candidate| candidate.publish_time)
    }

    fn start_log_time(&self) -> SystemTime {
        self.events
            .first()
            .expect("recording must contain replay events")
            .log_time
    }

    fn start_source_time(&self) -> SystemTime {
        self.events
            .first()
            .expect("recording must contain replay events")
            .publish_time
    }

    fn timestamp_diagnostics(&self) -> TimestampDiagnostics {
        let mut camera_offsets = ScalarSummary::default();
        let mut kinematics_offsets = ScalarSummary::default();
        let mut vo_current_offsets = ScalarSummary::default();
        let mut vo_delta_durations = ScalarSummary::default();
        let mut negative_or_zero_vo_delta_durations = 0;

        for event in &self.events {
            match &event.kind {
                EventKind::CameraMatrix(camera_matrix) => {
                    camera_offsets.add(signed_duration_seconds(
                        camera_matrix.time.to_wallclock(),
                        event.publish_time,
                    ));
                }
                EventKind::RobotKinematics(robot_kinematics) => {
                    kinematics_offsets.add(signed_duration_seconds(
                        robot_kinematics.time.to_wallclock(),
                        event.publish_time,
                    ));
                }
                EventKind::VisualOdometry(delta) => {
                    vo_current_offsets.add(signed_duration_seconds(
                        delta.current_time.to_wallclock(),
                        event.publish_time,
                    ));
                    match delta
                        .current_time
                        .to_wallclock()
                        .duration_since(delta.previous_time.to_wallclock())
                    {
                        Ok(duration) if !duration.is_zero() => {
                            vo_delta_durations.add(duration.as_secs_f64());
                        }
                        _ => negative_or_zero_vo_delta_durations += 1,
                    }
                }
                _ => {}
            }
        }

        TimestampDiagnostics {
            camera_matrix_time_minus_publish_time_seconds: camera_offsets.finish(),
            robot_kinematics_time_minus_publish_time_seconds: kinematics_offsets.finish(),
            vo_current_time_minus_publish_time_seconds: vo_current_offsets.finish(),
            vo_delta_duration_seconds: vo_delta_durations.finish(),
            negative_or_zero_vo_delta_durations,
        }
    }
}

#[derive(Clone)]
struct ImageTimestamp {
    embedded_time: Time,
    publish_time: SystemTime,
}

#[derive(Clone)]
struct RecordedEvent {
    order: usize,
    log_time: SystemTime,
    publish_time: SystemTime,
    kind: EventKind,
}

#[derive(Clone)]
enum EventKind {
    Imu(ImuState),
    VisualOdometry(VisualOdometryDelta),
    RobotKinematics(TimeWrapper<RobotKinematics>),
    CameraMatrix(TimeWrapper<CameraMatrix>),
    DetectedObjects(Vec<Object<RobocupObjectLabel>>),
    FieldDimensions,
}

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
enum TimestampMode {
    Embedded,
    McapPublish,
}

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
enum VisualOdometryCovarianceMode {
    Current,
    ValidatedFactrsOrder,
    FactrsOrderYawStrong,
    IsotropicTight,
}

#[derive(Debug, Clone)]
struct VariantConfig {
    name: &'static str,
    timestamp_mode: TimestampMode,
    include_global_features: bool,
    optimizer_iterations: usize,
    covariance_mode: VisualOdometryCovarianceMode,
}

impl VariantConfig {
    fn new(
        name: &'static str,
        timestamp_mode: TimestampMode,
        include_global_features: bool,
        optimizer_iterations: usize,
        covariance_mode: VisualOdometryCovarianceMode,
    ) -> Self {
        Self {
            name,
            timestamp_mode,
            include_global_features,
            optimizer_iterations,
            covariance_mode,
        }
    }
}

#[derive(Default)]
struct OnlineCameraMatrices {
    samples: Vec<CameraMatrixSample>,
}

impl OnlineCameraMatrices {
    fn push(&mut self, event: &RecordedEvent, camera_matrix: TimeWrapper<CameraMatrix>) {
        self.samples.push(CameraMatrixSample {
            payload_time: camera_matrix.time.to_wallclock(),
            publish_time: event.publish_time,
            matrix: camera_matrix.inner,
        });
    }

    fn nearest(&self, time: SystemTime, mode: TimestampMode) -> Option<NearestCameraMatrix<'_>> {
        self.samples
            .iter()
            .min_by_key(|sample| nanos_abs_diff(sample.time_for_mode(mode), time))
            .map(|sample| NearestCameraMatrix {
                matrix: &sample.matrix,
                distance: abs_duration(sample.time_for_mode(mode), time),
            })
    }
}

struct NearestCameraMatrix<'a> {
    matrix: &'a CameraMatrix,
    distance: Duration,
}

#[derive(Clone)]
struct CameraMatrixSample {
    payload_time: SystemTime,
    publish_time: SystemTime,
    matrix: CameraMatrix,
}

impl CameraMatrixSample {
    fn time_for_mode(&self, mode: TimestampMode) -> SystemTime {
        match mode {
            TimestampMode::Embedded => self.payload_time,
            TimestampMode::McapPublish => self.publish_time,
        }
    }
}

#[derive(Clone)]
struct DeadReckoningResult {
    trajectory: PoseTrajectory,
    comparison_start_time: SystemTime,
    vo_stats: VisualOdometryReplayStats,
    vo_consistency: VisualOdometryConsistencyDiagnostics,
}

#[derive(Clone)]
struct AcceptedVisualOdometryMeasurement {
    current_time: SystemTime,
    robot_delta: nalgebra::Isometry3<f64>,
}

#[derive(Debug, Clone, Serialize, Default)]
struct VisualOdometryConsistencyDiagnostics {
    accepted_measurements: usize,
    normal: VisualOdometryConventionError,
    inverted: VisualOdometryConventionError,
}

#[derive(Debug, Clone, Serialize, Default)]
struct VisualOdometryConventionError {
    translation_error_mean_m: f64,
    translation_error_max_m: f64,
    rotation_error_mean_deg: f64,
    rotation_error_max_deg: f64,
}

fn build_dead_reckoning(
    recording: &Recording,
    timestamp_mode: TimestampMode,
) -> Result<DeadReckoningResult, Box<dyn Error>> {
    let mut camera_matrices = OnlineCameraMatrices::default();
    let mut vo_timestamps = VisualOdometryTimestampTracker::default();
    let mut vo_stats = VisualOdometryReplayStats::default();
    let initial_pose =
        initial_robot_to_field_from_camera_matrix(&recording.first_camera_matrix).inner;
    let mut accepted_measurements = Vec::new();
    let mut comparison_start_time = None;

    for event in &recording.events {
        match &event.kind {
            EventKind::CameraMatrix(camera_matrix) => {
                camera_matrices.push(event, camera_matrix.clone());
            }
            EventKind::VisualOdometry(delta) => {
                vo_stats.received += 1;
                let Some((previous_time, current_time)) =
                    vo_timestamps.measurement_times(event, delta, timestamp_mode, recording)
                else {
                    vo_stats.dropped_invalid += 1;
                    continue;
                };
                let Some(lookup) = camera_matrix_pair(
                    &camera_matrices,
                    previous_time,
                    current_time,
                    timestamp_mode,
                    &mut vo_stats,
                ) else {
                    continue;
                };

                classify_vo_interval(
                    recording.start_source_time(),
                    previous_time,
                    current_time,
                    &mut vo_stats,
                );
                if current_time <= previous_time {
                    continue;
                }

                let robot_delta = robot_delta_from_visual_odometry_delta(
                    delta,
                    lookup.previous.matrix,
                    lookup.current.matrix,
                );
                accepted_measurements.push(AcceptedVisualOdometryMeasurement {
                    current_time,
                    robot_delta,
                });
                comparison_start_time.get_or_insert(current_time);
            }
            _ => {}
        }
    }

    let trajectory = visual_odometry_trajectory(
        recording.start_source_time(),
        initial_pose,
        &accepted_measurements,
        invert_dead_reckoning_delta(),
    );
    let vo_consistency = visual_odometry_consistency(initial_pose, &accepted_measurements);

    Ok(DeadReckoningResult {
        trajectory,
        comparison_start_time: comparison_start_time.unwrap_or(recording.start_source_time()),
        vo_stats,
        vo_consistency,
    })
}

fn visual_odometry_trajectory(
    start_time: SystemTime,
    initial_pose: nalgebra::Isometry3<f64>,
    measurements: &[AcceptedVisualOdometryMeasurement],
    invert_delta: bool,
) -> PoseTrajectory {
    let mut pose = initial_pose;
    let mut trajectory = PoseTrajectory::new();
    trajectory.push(start_time, pose);
    for measurement in measurements {
        if invert_delta {
            pose *= measurement.robot_delta.inverse();
        } else {
            pose *= measurement.robot_delta;
        }
        trajectory.push(measurement.current_time, pose);
    }
    trajectory
}

fn visual_odometry_consistency(
    initial_pose: nalgebra::Isometry3<f64>,
    measurements: &[AcceptedVisualOdometryMeasurement],
) -> VisualOdometryConsistencyDiagnostics {
    VisualOdometryConsistencyDiagnostics {
        accepted_measurements: measurements.len(),
        normal: visual_odometry_convention_error(initial_pose, measurements, false),
        inverted: visual_odometry_convention_error(initial_pose, measurements, true),
    }
}

fn visual_odometry_convention_error(
    initial_pose: nalgebra::Isometry3<f64>,
    measurements: &[AcceptedVisualOdometryMeasurement],
    invert_delta: bool,
) -> VisualOdometryConventionError {
    let mut pose = initial_pose;
    let mut translation_errors = Vec::with_capacity(measurements.len());
    let mut rotation_errors = Vec::with_capacity(measurements.len());
    for measurement in measurements {
        let previous_pose = pose;
        if invert_delta {
            pose *= measurement.robot_delta.inverse();
        } else {
            pose *= measurement.robot_delta;
        }
        let predicted_delta = previous_pose.inverse() * pose;
        let delta_error = measurement.robot_delta.inverse() * predicted_delta;
        translation_errors.push(delta_error.translation.vector.norm());
        rotation_errors.push(delta_error.rotation.angle().to_degrees());
    }

    VisualOdometryConventionError {
        translation_error_mean_m: mean(&translation_errors),
        translation_error_max_m: max(&translation_errors),
        rotation_error_mean_deg: mean(&rotation_errors),
        rotation_error_max_deg: max(&rotation_errors),
    }
}

struct CameraMatrixPair<'a> {
    previous: NearestCameraMatrix<'a>,
    current: NearestCameraMatrix<'a>,
}

fn camera_matrix_pair<'a>(
    camera_matrices: &'a OnlineCameraMatrices,
    previous_time: SystemTime,
    current_time: SystemTime,
    timestamp_mode: TimestampMode,
    vo_stats: &mut VisualOdometryReplayStats,
) -> Option<CameraMatrixPair<'a>> {
    let Some(previous) = camera_matrices.nearest(previous_time, timestamp_mode) else {
        vo_stats.skipped_missing_camera_matrix += 1;
        return None;
    };
    let Some(current) = camera_matrices.nearest(current_time, timestamp_mode) else {
        vo_stats.skipped_missing_camera_matrix += 1;
        return None;
    };

    vo_stats.max_previous_camera_matrix_age_seconds = vo_stats
        .max_previous_camera_matrix_age_seconds
        .max(previous.distance.as_secs_f64());
    vo_stats.max_current_camera_matrix_age_seconds = vo_stats
        .max_current_camera_matrix_age_seconds
        .max(current.distance.as_secs_f64());
    if previous.distance > STALE_CAMERA_MATRIX_THRESHOLD
        || current.distance > STALE_CAMERA_MATRIX_THRESHOLD
    {
        vo_stats.skipped_stale_camera_matrix += 1;
    }

    Some(CameraMatrixPair { previous, current })
}

fn replay_graph_variant(
    recording: &Recording,
    field_dimensions: &FieldDimensions,
    variant: &VariantConfig,
    dead_reckoning: &DeadReckoningResult,
    solve_cadence: Duration,
) -> Result<VariantReplay, Box<dyn Error>> {
    let initial_state = initial_state_from_camera_matrix(&recording.first_camera_matrix);
    let (mut frontend, mut backend) = initialize(
        variant_backend_configuration(variant.optimizer_iterations, variant.covariance_mode),
        initial_state,
    );
    let mut camera_matrices = OnlineCameraMatrices::default();
    let mut vo_timestamps = VisualOdometryTimestampTracker::default();
    let mut vo_stats = VisualOdometryReplayStats::default();
    let mut global_stats = GlobalFeatureReplayStats::default();
    let mut samples = Vec::new();
    let mut accepted_global_feature_times = Vec::new();
    let mut alignment = None;
    let mut has_pending_measurements = false;
    let mut next_solve_time = recording.start_log_time() + solve_cadence;
    let global_localizer_parameters = global_localizer_parameters()?;

    for event in &recording.events {
        while next_solve_time <= event.log_time {
            if has_pending_measurements {
                solve_and_record(
                    &mut backend,
                    &mut frontend,
                    next_solve_time,
                    recording.start_source_time(),
                    variant,
                    dead_reckoning,
                    &vo_stats,
                    &global_stats,
                    &accepted_global_feature_times,
                    &mut alignment,
                    &mut samples,
                )?;
                has_pending_measurements = false;
            }
            next_solve_time += solve_cadence;
        }

        match &event.kind {
            EventKind::Imu(imu) => {
                if skip_imu_replay() {
                    continue;
                }
                frontend.ingest_imu(event.publish_time, imu.clone())?;
                has_pending_measurements = true;
            }
            EventKind::CameraMatrix(camera_matrix) => {
                camera_matrices.push(event, camera_matrix.clone());
            }
            EventKind::RobotKinematics(robot_kinematics) => {
                if skip_foot_height_replay() {
                    continue;
                }
                let mut robot_kinematics = robot_kinematics.clone();
                if matches!(variant.timestamp_mode, TimestampMode::McapPublish) {
                    robot_kinematics.time = Time::from_wallclock(event.publish_time);
                }
                ingest_foot_heights(&mut frontend, robot_kinematics)?;
                has_pending_measurements = true;
            }
            EventKind::VisualOdometry(delta) => {
                vo_stats.received += 1;
                let Some((previous_time, current_time)) = vo_timestamps.measurement_times(
                    event,
                    delta,
                    variant.timestamp_mode,
                    recording,
                ) else {
                    vo_stats.dropped_invalid += 1;
                    continue;
                };
                let Some(lookup) = camera_matrix_pair(
                    &camera_matrices,
                    previous_time,
                    current_time,
                    variant.timestamp_mode,
                    &mut vo_stats,
                ) else {
                    continue;
                };
                classify_vo_interval(
                    recording.start_source_time(),
                    previous_time,
                    current_time,
                    &mut vo_stats,
                );
                if current_time <= previous_time {
                    continue;
                }

                let mut delta = delta.clone();
                delta.previous_time = Time::from_wallclock(previous_time);
                delta.current_time = Time::from_wallclock(current_time);
                ingest_visual_odometry(
                    &mut frontend,
                    delta,
                    lookup.previous.matrix,
                    lookup.current.matrix,
                )?;
                has_pending_measurements = true;
            }
            EventKind::DetectedObjects(objects) if variant.include_global_features => {
                global_stats.detected_object_frames += 1;
                let visual_features = find_detected_visual_features(objects);
                if visual_features.supported_feature_count()
                    < global_localizer_parameters.min_inliers.max(3)
                {
                    global_stats.skipped_too_few_supported_features += 1;
                    continue;
                }
                global_stats.localizer_candidate_frames += 1;
                let Some(camera_matrix) = camera_matrices
                    .nearest(event.publish_time, variant.timestamp_mode)
                    .map(|nearest| nearest.matrix)
                else {
                    global_stats.skipped_missing_camera_matrix += 1;
                    continue;
                };

                let pose_hint = frontend
                    .peek_last_optimization_result()
                    .map(|result| result.transform.cast::<f32>().framed_transform());
                let localization = localize_global_visual_features(
                    &visual_features,
                    camera_matrix,
                    field_dimensions,
                    pose_hint,
                    &global_localizer_parameters,
                );
                match localization.debug.as_ref().map(|debug| debug.status) {
                    None => global_stats.localizer_none += 1,
                    Some(GlobalLocalizationDebugStatus::Ambiguous) => {
                        global_stats.localizer_ambiguous += 1;
                    }
                    Some(GlobalLocalizationDebugStatus::UniqueModuloSymmetry) => {
                        global_stats.localizer_unique_modulo_symmetry += 1;
                    }
                }
                let associations = localization.associations;
                if !associations.is_empty() {
                    global_stats.frames_ingested += 1;
                    global_stats.accepted_associations += associations.len();
                    accepted_global_feature_times.push(event.publish_time);
                    let associations =
                        associations
                            .into_iter()
                            .map(|association| VisualReprojectionAssociation {
                                detection: association.detection,
                                field_point: association.field_point,
                                kind: VisualReprojectionAssociationKind::GlobalUnique,
                            });
                    frontend.ingest_visual_reprojection_associations(
                        event.publish_time,
                        associations,
                        robot_to_camera(camera_matrix),
                    )?;
                    has_pending_measurements = true;
                }
            }
            EventKind::DetectedObjects(_) => {
                global_stats.detected_object_frames += 1;
            }
            EventKind::FieldDimensions => {}
        }
    }

    if has_pending_measurements {
        solve_and_record(
            &mut backend,
            &mut frontend,
            recording
                .events
                .last()
                .expect("recording has events")
                .log_time,
            recording.start_source_time(),
            variant,
            dead_reckoning,
            &vo_stats,
            &global_stats,
            &accepted_global_feature_times,
            &mut alignment,
            &mut samples,
        )?;
    }

    let summary = summarize_variant(
        variant,
        &samples,
        dead_reckoning,
        &vo_stats,
        &global_stats,
        &accepted_global_feature_times,
    );
    Ok(VariantReplay { summary, samples })
}

#[allow(clippy::too_many_arguments)]
fn solve_and_record(
    backend: &mut VinsBackend,
    frontend: &mut VinsFrontend,
    replay_time: SystemTime,
    start_time: SystemTime,
    variant: &VariantConfig,
    dead_reckoning: &DeadReckoningResult,
    vo_stats: &VisualOdometryReplayStats,
    global_stats: &GlobalFeatureReplayStats,
    accepted_global_feature_times: &[SystemTime],
    alignment: &mut Option<nalgebra::Isometry3<f64>>,
    samples: &mut Vec<ComparisonSample>,
) -> Result<(), Box<dyn Error>> {
    let backend_result = backend.solve_once()?;
    if backend_result.is_none() {
        return Ok(());
    }
    let diagnostics = backend.compute_last_solve_diagnostics();
    let Some(result) = frontend.last_optimization_result() else {
        return Ok(());
    };
    let Some(dead_pose) = dead_reckoning.trajectory.sample(result.time) else {
        return Ok(());
    };
    if result.time < dead_reckoning.comparison_start_time {
        return Ok(());
    }

    let graph_pose = result.transform;
    let alignment = alignment.get_or_insert_with(|| dead_pose * graph_pose.inverse());
    let graph_pose = *alignment * graph_pose;
    let error = pose_error(dead_pose, graph_pose);
    let graph_delay_seconds = signed_duration_seconds(replay_time, result.time).max(0.0);
    let nearest_global_feature_seconds = accepted_global_feature_times
        .iter()
        .map(|time| abs_duration(*time, result.time).as_secs_f64())
        .min_by(f64::total_cmp);

    samples.push(ComparisonSample {
        variant: variant.name.to_string(),
        timestamp_seconds: seconds_since(result.time, start_time),
        replay_time_seconds: seconds_since(replay_time, start_time),
        graph_delay_seconds,
        graph_position: vector3(graph_pose.translation.vector),
        dead_reckoning_position: vector3(dead_pose.translation.vector),
        translation_error_norm: error.translation_norm,
        translation_error_xyz: vector3(error.translation),
        roll_error_degrees: error.roll.to_degrees(),
        pitch_error_degrees: error.pitch.to_degrees(),
        yaw_error_degrees: error.yaw.to_degrees(),
        cumulative_vo_received: vo_stats.received,
        cumulative_vo_inserted_same_interval: vo_stats.inserted_same_interval,
        cumulative_vo_inserted_adjacent_interval: vo_stats.inserted_adjacent_interval,
        cumulative_vo_dropped_invalid: vo_stats.dropped_invalid,
        cumulative_vo_dropped_multi_interval: vo_stats.dropped_multi_interval,
        cumulative_vo_skipped_missing_camera_matrix: vo_stats.skipped_missing_camera_matrix,
        cumulative_vo_stale_camera_matrix: vo_stats.skipped_stale_camera_matrix,
        cumulative_global_frames_ingested: global_stats.frames_ingested,
        cumulative_global_accepted_associations: global_stats.accepted_associations,
        nearest_global_feature_seconds,
        diagnostics: diagnostics.map(SolveDiagnosticsSample::from),
    });

    Ok(())
}

#[derive(Debug, Clone, Default)]
struct VisualOdometryTimestampTracker {
    previous_mcap_publish_time: Option<SystemTime>,
}

impl VisualOdometryTimestampTracker {
    fn measurement_times(
        &mut self,
        event: &RecordedEvent,
        delta: &VisualOdometryDelta,
        mode: TimestampMode,
        recording: &Recording,
    ) -> Option<(SystemTime, SystemTime)> {
        match mode {
            TimestampMode::Embedded => Some((
                delta.previous_time.to_wallclock(),
                delta.current_time.to_wallclock(),
            )),
            TimestampMode::McapPublish => {
                let current = recording
                    .aligned_image_time(delta.current_time)
                    .unwrap_or(event.publish_time);
                let previous = recording
                    .aligned_image_time(delta.previous_time)
                    .or(self.previous_mcap_publish_time)
                    .or_else(|| {
                        let embedded_duration = delta
                            .current_time
                            .to_wallclock()
                            .duration_since(delta.previous_time.to_wallclock())
                            .ok()?;
                        current.checked_sub(embedded_duration)
                    });
                self.previous_mcap_publish_time = Some(current);
                previous.map(|previous| (previous, current))
            }
        }
    }
}

fn classify_vo_interval(
    interval_start: SystemTime,
    previous_time: SystemTime,
    current_time: SystemTime,
    stats: &mut VisualOdometryReplayStats,
) {
    if current_time <= previous_time {
        stats.dropped_invalid += 1;
        return;
    }
    let Some(previous_index) = interval_index(interval_start, previous_time) else {
        stats.dropped_invalid += 1;
        return;
    };
    let Some(current_index) = interval_index(interval_start, current_time) else {
        stats.dropped_invalid += 1;
        return;
    };
    match current_index.checked_sub(previous_index) {
        Some(0) => stats.inserted_same_interval += 1,
        Some(span) => {
            stats.spanning_boundary += 1;
            stats.inserted_same_interval += span as usize + 1;
            if span > 1 {
                stats.split_multi_interval += 1;
            }
        }
        None => stats.dropped_invalid += 1,
    }
}

fn interval_index(start_time: SystemTime, time: SystemTime) -> Option<u64> {
    Some(time.duration_since(start_time).ok()?.as_nanos() as u64 / KNOT_SPACING.as_nanos() as u64)
}

#[derive(Debug, Clone, Serialize, Default)]
struct VisualOdometryReplayStats {
    received: usize,
    inserted_same_interval: usize,
    inserted_adjacent_interval: usize,
    spanning_boundary: usize,
    split_multi_interval: usize,
    dropped_invalid: usize,
    dropped_multi_interval: usize,
    skipped_missing_camera_matrix: usize,
    skipped_stale_camera_matrix: usize,
    max_previous_camera_matrix_age_seconds: f64,
    max_current_camera_matrix_age_seconds: f64,
}

#[derive(Debug, Clone, Serialize, Default)]
struct GlobalFeatureReplayStats {
    detected_object_frames: usize,
    skipped_too_few_supported_features: usize,
    localizer_candidate_frames: usize,
    localizer_none: usize,
    localizer_ambiguous: usize,
    localizer_unique_modulo_symmetry: usize,
    frames_ingested: usize,
    accepted_associations: usize,
    skipped_missing_camera_matrix: usize,
}

#[derive(Clone)]
struct PoseTrajectory {
    samples: Vec<PoseSample>,
}

impl PoseTrajectory {
    fn new() -> Self {
        Self {
            samples: Vec::new(),
        }
    }

    fn push(&mut self, time: SystemTime, pose: nalgebra::Isometry3<f64>) {
        if self
            .samples
            .last()
            .is_some_and(|sample| sample.time >= time)
        {
            return;
        }
        self.samples.push(PoseSample { time, pose });
    }

    fn sample(&self, time: SystemTime) -> Option<nalgebra::Isometry3<f64>> {
        let first = self.samples.first()?;
        if time <= first.time {
            return Some(first.pose);
        }

        for window in self.samples.windows(2) {
            let [previous, current] = window else {
                unreachable!();
            };
            if previous.time <= time && time <= current.time {
                let interval = current.time.duration_since(previous.time).ok()?;
                if interval.is_zero() {
                    return Some(previous.pose);
                }
                let tau =
                    time.duration_since(previous.time).ok()?.as_secs_f64() / interval.as_secs_f64();
                let translation = previous.pose.translation.vector * (1.0 - tau)
                    + current.pose.translation.vector * tau;
                let rotation = previous.pose.rotation.slerp(&current.pose.rotation, tau);
                return Some(nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::from(translation),
                    rotation,
                ));
            }
        }

        self.samples.last().map(|sample| sample.pose)
    }
}

#[derive(Clone, Copy)]
struct PoseSample {
    time: SystemTime,
    pose: nalgebra::Isometry3<f64>,
}

#[derive(Debug, Clone, Serialize)]
struct ComparisonSample {
    variant: String,
    timestamp_seconds: f64,
    replay_time_seconds: f64,
    graph_delay_seconds: f64,
    graph_position: [f64; 3],
    dead_reckoning_position: [f64; 3],
    translation_error_norm: f64,
    translation_error_xyz: [f64; 3],
    roll_error_degrees: f64,
    pitch_error_degrees: f64,
    yaw_error_degrees: f64,
    cumulative_vo_received: usize,
    cumulative_vo_inserted_same_interval: usize,
    cumulative_vo_inserted_adjacent_interval: usize,
    cumulative_vo_dropped_invalid: usize,
    cumulative_vo_dropped_multi_interval: usize,
    cumulative_vo_skipped_missing_camera_matrix: usize,
    cumulative_vo_stale_camera_matrix: usize,
    cumulative_global_frames_ingested: usize,
    cumulative_global_accepted_associations: usize,
    nearest_global_feature_seconds: Option<f64>,
    diagnostics: Option<SolveDiagnosticsSample>,
}

#[derive(Debug, Clone, Serialize)]
struct SolveDiagnosticsSample {
    optimizer_status: String,
    value_count: usize,
    factor_count: usize,
    total_error: f64,
    visual_odometry_mean_rms: f64,
    visual_odometry_max_rms: f64,
    visual_reprojection_mean_rms: f64,
    visual_reprojection_max_rms: f64,
    gaussian_process_prior_mean_rms: f64,
    gaussian_process_prior_max_rms: f64,
}

impl From<BackendSolveDiagnostics> for SolveDiagnosticsSample {
    fn from(diagnostics: BackendSolveDiagnostics) -> Self {
        Self {
            optimizer_status: format!("{:?}", diagnostics.optimizer_status),
            value_count: diagnostics.value_count,
            factor_count: diagnostics.factor_count,
            total_error: diagnostics.total_error,
            visual_odometry_mean_rms: diagnostics.visual_odometry.mean_rms,
            visual_odometry_max_rms: diagnostics.visual_odometry.max_rms,
            visual_reprojection_mean_rms: diagnostics.visual_reprojection.mean_rms,
            visual_reprojection_max_rms: diagnostics.visual_reprojection.max_rms,
            gaussian_process_prior_mean_rms: diagnostics.gaussian_process_prior.mean_rms,
            gaussian_process_prior_max_rms: diagnostics.gaussian_process_prior.max_rms,
        }
    }
}

struct PoseError {
    translation: nalgebra::Vector3<f64>,
    translation_norm: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

fn pose_error(
    reference: nalgebra::Isometry3<f64>,
    estimate: nalgebra::Isometry3<f64>,
) -> PoseError {
    let error = reference.inverse() * estimate;
    let translation = error.translation.vector;
    let (roll, pitch, yaw) = error.rotation.euler_angles();
    PoseError {
        translation,
        translation_norm: translation.norm(),
        roll,
        pitch,
        yaw,
    }
}

struct VariantReplay {
    summary: VariantSummary,
    samples: Vec<ComparisonSample>,
}

#[derive(Debug, Clone, Serialize)]
struct VariantSummary {
    name: String,
    timestamp_mode: TimestampMode,
    include_global_features: bool,
    optimizer_iterations: usize,
    covariance_mode: VisualOdometryCovarianceMode,
    sample_count: usize,
    dead_reckoning_vo_stats: VisualOdometryReplayStats,
    dead_reckoning_vo_consistency: VisualOdometryConsistencyDiagnostics,
    graph_vo_stats: VisualOdometryReplayStats,
    global_feature_stats: GlobalFeatureReplayStats,
    translation_error_mean_m: f64,
    translation_error_max_m: f64,
    yaw_error_mean_abs_deg: f64,
    yaw_error_max_abs_deg: f64,
    roll_pitch_error_max_abs_deg: f64,
    graph_delay_mean_seconds: f64,
    graph_delay_max_seconds: f64,
    near_knot_boundary_error_mean_m: Option<f64>,
    near_knot_boundary_error_max_m: Option<f64>,
    away_from_knot_boundary_error_mean_m: Option<f64>,
    vo_drop_error_correlation: Option<f64>,
    stale_camera_error_correlation: Option<f64>,
    stationary_translation_drift_mean_cm_per_s: Option<f64>,
    stationary_translation_drift_max_cm_per_s: Option<f64>,
    stationary_yaw_drift_mean_deg_per_s: Option<f64>,
    stationary_yaw_drift_max_deg_per_s: Option<f64>,
    graph_endpoint_displacement_m: Option<f64>,
    dead_reckoning_endpoint_displacement_m: Option<f64>,
    graph_endpoint_yaw_deg: Option<f64>,
    dead_reckoning_endpoint_yaw_deg: Option<f64>,
    correction_jump_count: usize,
    correction_jumps_near_global_features: usize,
    optimizer_status_counts: BTreeMap<String, usize>,
    visual_odometry_residual_mean_rms_max: f64,
    visual_reprojection_residual_mean_rms_max: f64,
    gaussian_process_prior_residual_mean_rms_max: f64,
}

fn summarize_variant(
    variant: &VariantConfig,
    samples: &[ComparisonSample],
    dead_reckoning: &DeadReckoningResult,
    graph_vo_stats: &VisualOdometryReplayStats,
    global_feature_stats: &GlobalFeatureReplayStats,
    accepted_global_feature_times: &[SystemTime],
) -> VariantSummary {
    let sample_count = samples.len();
    let translation_errors = samples
        .iter()
        .map(|sample| sample.translation_error_norm)
        .collect::<Vec<_>>();
    let yaw_errors = samples
        .iter()
        .map(|sample| sample.yaw_error_degrees.abs())
        .collect::<Vec<_>>();
    let graph_delays = samples
        .iter()
        .map(|sample| sample.graph_delay_seconds)
        .collect::<Vec<_>>();
    let mut optimizer_status_counts = BTreeMap::new();
    let mut visual_odometry_residual_mean_rms_max: f64 = 0.0;
    let mut visual_reprojection_residual_mean_rms_max: f64 = 0.0;
    let mut gaussian_process_prior_residual_mean_rms_max: f64 = 0.0;
    for diagnostics in samples
        .iter()
        .filter_map(|sample| sample.diagnostics.as_ref())
    {
        *optimizer_status_counts
            .entry(diagnostics.optimizer_status.clone())
            .or_default() += 1;
        visual_odometry_residual_mean_rms_max =
            visual_odometry_residual_mean_rms_max.max(diagnostics.visual_odometry_mean_rms);
        visual_reprojection_residual_mean_rms_max =
            visual_reprojection_residual_mean_rms_max.max(diagnostics.visual_reprojection_mean_rms);
        gaussian_process_prior_residual_mean_rms_max = gaussian_process_prior_residual_mean_rms_max
            .max(diagnostics.gaussian_process_prior_mean_rms);
    }

    let (near_boundary, away_boundary): (Vec<_>, Vec<_>) = samples.iter().partition(|sample| {
        let phase = Duration::from_secs_f64(
            sample
                .timestamp_seconds
                .rem_euclid(KNOT_SPACING.as_secs_f64()),
        );
        phase <= KNOT_BOUNDARY_WINDOW || KNOT_SPACING - phase <= KNOT_BOUNDARY_WINDOW
    });

    let stationary = stationary_drift(samples);
    let endpoints = endpoint_displacements(samples);
    let correction_jumps = correction_jump_summary(samples, accepted_global_feature_times);

    VariantSummary {
        name: variant.name.to_string(),
        timestamp_mode: variant.timestamp_mode,
        include_global_features: variant.include_global_features,
        optimizer_iterations: variant.optimizer_iterations,
        covariance_mode: variant.covariance_mode,
        sample_count,
        dead_reckoning_vo_stats: dead_reckoning.vo_stats.clone(),
        dead_reckoning_vo_consistency: dead_reckoning.vo_consistency.clone(),
        graph_vo_stats: graph_vo_stats.clone(),
        global_feature_stats: global_feature_stats.clone(),
        translation_error_mean_m: mean(&translation_errors),
        translation_error_max_m: max(&translation_errors),
        yaw_error_mean_abs_deg: mean(&yaw_errors),
        yaw_error_max_abs_deg: max(&yaw_errors),
        roll_pitch_error_max_abs_deg: samples
            .iter()
            .map(|sample| {
                sample
                    .roll_error_degrees
                    .abs()
                    .max(sample.pitch_error_degrees.abs())
            })
            .fold(0.0, f64::max),
        graph_delay_mean_seconds: mean(&graph_delays),
        graph_delay_max_seconds: max(&graph_delays),
        near_knot_boundary_error_mean_m: non_empty_mean(
            near_boundary
                .iter()
                .map(|sample| sample.translation_error_norm),
        ),
        near_knot_boundary_error_max_m: non_empty_max(
            near_boundary
                .iter()
                .map(|sample| sample.translation_error_norm),
        ),
        away_from_knot_boundary_error_mean_m: non_empty_mean(
            away_boundary
                .iter()
                .map(|sample| sample.translation_error_norm),
        ),
        vo_drop_error_correlation: correlation(
            samples.iter().map(|sample| sample.translation_error_norm),
            samples.iter().map(|sample| {
                (sample.cumulative_vo_dropped_invalid
                    + sample.cumulative_vo_dropped_multi_interval
                    + sample.cumulative_vo_skipped_missing_camera_matrix) as f64
            }),
        ),
        stale_camera_error_correlation: correlation(
            samples.iter().map(|sample| sample.translation_error_norm),
            samples
                .iter()
                .map(|sample| sample.cumulative_vo_stale_camera_matrix as f64),
        ),
        stationary_translation_drift_mean_cm_per_s: stationary.translation_mean_cm_per_s,
        stationary_translation_drift_max_cm_per_s: stationary.translation_max_cm_per_s,
        stationary_yaw_drift_mean_deg_per_s: stationary.yaw_mean_deg_per_s,
        stationary_yaw_drift_max_deg_per_s: stationary.yaw_max_deg_per_s,
        graph_endpoint_displacement_m: endpoints.graph_translation_m,
        dead_reckoning_endpoint_displacement_m: endpoints.dead_reckoning_translation_m,
        graph_endpoint_yaw_deg: endpoints.graph_yaw_deg,
        dead_reckoning_endpoint_yaw_deg: endpoints.dead_reckoning_yaw_deg,
        correction_jump_count: correction_jumps.total,
        correction_jumps_near_global_features: correction_jumps.near_global_features,
        optimizer_status_counts,
        visual_odometry_residual_mean_rms_max,
        visual_reprojection_residual_mean_rms_max,
        gaussian_process_prior_residual_mean_rms_max,
    }
}

struct StationaryDriftSummary {
    translation_mean_cm_per_s: Option<f64>,
    translation_max_cm_per_s: Option<f64>,
    yaw_mean_deg_per_s: Option<f64>,
    yaw_max_deg_per_s: Option<f64>,
}

fn stationary_drift(samples: &[ComparisonSample]) -> StationaryDriftSummary {
    let mut translation_rates = Vec::new();
    let mut yaw_rates = Vec::new();

    for window in samples.windows(2) {
        let previous = &window[0];
        let current = &window[1];
        let dt = current.timestamp_seconds - previous.timestamp_seconds;
        if dt <= 0.0 {
            continue;
        }

        let dead_motion = distance(
            previous.dead_reckoning_position,
            current.dead_reckoning_position,
        );
        if dead_motion / dt > 0.02 {
            continue;
        }

        let translation_drift =
            (current.translation_error_norm - previous.translation_error_norm).abs();
        let yaw_drift = (current.yaw_error_degrees - previous.yaw_error_degrees).abs();
        translation_rates.push(translation_drift / dt * 100.0);
        yaw_rates.push(yaw_drift / dt);
    }

    StationaryDriftSummary {
        translation_mean_cm_per_s: non_empty_mean(translation_rates.iter().copied()),
        translation_max_cm_per_s: non_empty_max(translation_rates.iter().copied()),
        yaw_mean_deg_per_s: non_empty_mean(yaw_rates.iter().copied()),
        yaw_max_deg_per_s: non_empty_max(yaw_rates.iter().copied()),
    }
}

struct EndpointDisplacements {
    graph_translation_m: Option<f64>,
    dead_reckoning_translation_m: Option<f64>,
    graph_yaw_deg: Option<f64>,
    dead_reckoning_yaw_deg: Option<f64>,
}

fn endpoint_displacements(samples: &[ComparisonSample]) -> EndpointDisplacements {
    let Some(first) = samples.first() else {
        return EndpointDisplacements {
            graph_translation_m: None,
            dead_reckoning_translation_m: None,
            graph_yaw_deg: None,
            dead_reckoning_yaw_deg: None,
        };
    };
    let Some(last) = samples.last() else {
        unreachable!();
    };

    EndpointDisplacements {
        graph_translation_m: Some(distance(first.graph_position, last.graph_position)),
        dead_reckoning_translation_m: Some(distance(
            first.dead_reckoning_position,
            last.dead_reckoning_position,
        )),
        graph_yaw_deg: None,
        dead_reckoning_yaw_deg: None,
    }
}

struct CorrectionJumpSummary {
    total: usize,
    near_global_features: usize,
}

fn correction_jump_summary(
    samples: &[ComparisonSample],
    _accepted_global_feature_times: &[SystemTime],
) -> CorrectionJumpSummary {
    let mut total = 0;
    let mut near_global_features = 0;
    for window in samples.windows(2) {
        let previous = &window[0];
        let current = &window[1];
        let translation_jump =
            (current.translation_error_norm - previous.translation_error_norm).abs() > 0.05;
        let yaw_jump = (current.yaw_error_degrees - previous.yaw_error_degrees).abs() > 3.0;
        if !translation_jump && !yaw_jump {
            continue;
        }
        total += 1;
        if current
            .nearest_global_feature_seconds
            .is_some_and(|seconds| seconds <= GLOBAL_CORRECTION_WINDOW.as_secs_f64())
        {
            near_global_features += 1;
        }
    }
    CorrectionJumpSummary {
        total,
        near_global_features,
    }
}

#[derive(Debug, Clone, Serialize)]
struct ReplayReport {
    recording_path: String,
    output_dir: String,
    field_dimensions_source: String,
    solve_cadence_ms: f64,
    stale_camera_matrix_threshold_ms: f64,
    topic_counts: BTreeMap<String, usize>,
    timestamp_diagnostics: TimestampDiagnostics,
    variants: Vec<VariantSummary>,
}

#[derive(Debug, Clone, Serialize)]
struct TimestampDiagnostics {
    camera_matrix_time_minus_publish_time_seconds: ScalarSummaryOutput,
    robot_kinematics_time_minus_publish_time_seconds: ScalarSummaryOutput,
    vo_current_time_minus_publish_time_seconds: ScalarSummaryOutput,
    vo_delta_duration_seconds: ScalarSummaryOutput,
    negative_or_zero_vo_delta_durations: usize,
}

#[derive(Default)]
struct ScalarSummary {
    count: usize,
    sum: f64,
    min: f64,
    max: f64,
}

impl ScalarSummary {
    fn add(&mut self, value: f64) {
        if self.count == 0 {
            self.min = value;
            self.max = value;
        } else {
            self.min = self.min.min(value);
            self.max = self.max.max(value);
        }
        self.count += 1;
        self.sum += value;
    }

    fn finish(self) -> ScalarSummaryOutput {
        ScalarSummaryOutput {
            count: self.count,
            mean: if self.count == 0 {
                None
            } else {
                Some(self.sum / self.count as f64)
            },
            min: (self.count > 0).then_some(self.min),
            max: (self.count > 0).then_some(self.max),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
struct ScalarSummaryOutput {
    count: usize,
    mean: Option<f64>,
    min: Option<f64>,
    max: Option<f64>,
}

fn variant_backend_configuration(
    optimizer_iterations: usize,
    covariance_mode: VisualOdometryCovarianceMode,
) -> BackendConfiguration {
    let parameters = Localization3dParameters::default();
    let mut config = backend_configuration(parameters.visual_feature_noise_variance);
    config.optimizer_max_iterations = optimizer_iterations;
    config.visual_odometry_noise = match covariance_mode {
        VisualOdometryCovarianceMode::Current => config.visual_odometry_noise,
        VisualOdometryCovarianceMode::ValidatedFactrsOrder => SMatrix::<f64, 6, 6>::from_diagonal(
            &SVector::<f64, 6>::new(5.0e-5, 5.0e-5, 9.0e-2, 2.5e-3, 2.5e-3, 5.0e-6),
        ),
        VisualOdometryCovarianceMode::FactrsOrderYawStrong => SMatrix::<f64, 6, 6>::from_diagonal(
            &SVector::<f64, 6>::new(5.0e-5, 5.0e-5, 5.0e-6, 2.5e-3, 2.5e-3, 9.0e-2),
        ),
        VisualOdometryCovarianceMode::IsotropicTight => SMatrix::<f64, 6, 6>::identity() * 1.0e-4,
    };
    if let Some(scale) = visual_odometry_covariance_scale_override() {
        config.visual_odometry_noise = SMatrix::<f64, 6, 6>::identity() * scale;
    }
    if let Some(scale) = gp_noise_scale_override() {
        config.gyroscope_process_noise *= scale;
        config.accelerometer_process_noise *= scale;
    }
    if let Some(scale) = attitude_noise_scale_override() {
        config.roll_pitch_yaw_noise *= scale;
    }
    if let Some(max_window) = max_optimization_window_override() {
        config.max_optimization_window = max_window;
    }
    config
}

fn visual_odometry_covariance_scale_override() -> Option<f64> {
    std::env::var("LOCALIZATION_3D_REPLAY_VO_COVARIANCE_SCALE")
        .ok()
        .and_then(|value| value.parse::<f64>().ok())
}

fn gp_noise_scale_override() -> Option<f64> {
    std::env::var("LOCALIZATION_3D_REPLAY_GP_NOISE_SCALE")
        .ok()
        .and_then(|value| value.parse::<f64>().ok())
}

fn attitude_noise_scale_override() -> Option<f64> {
    std::env::var("LOCALIZATION_3D_REPLAY_ATTITUDE_NOISE_SCALE")
        .ok()
        .and_then(|value| value.parse::<f64>().ok())
}

fn max_optimization_window_override() -> Option<Duration> {
    std::env::var("LOCALIZATION_3D_REPLAY_MAX_WINDOW_SECONDS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .map(Duration::from_secs)
}

fn global_localizer_parameters() -> Result<GlobalLocalizerParameters, Box<dyn Error>> {
    reject_removed_env("LOCALIZATION_3D_REPLAY_GLOBAL_REPROJECTION_GATE")?;
    reject_removed_env("LOCALIZATION_3D_REPLAY_GLOBAL_AMBIGUITY_RMSE_MARGIN")?;

    let mut parameters = GlobalLocalizerParameters::default();
    if let Some(min_inliers) = usize_env("LOCALIZATION_3D_REPLAY_GLOBAL_MIN_INLIERS")? {
        parameters.min_inliers = min_inliers;
    }
    if let Some(min_confidence) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_MIN_CONFIDENCE")? {
        parameters.min_confidence = min_confidence;
    }
    if let Some(min_detection_baseline) =
        f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_MIN_DETECTION_BASELINE")?
    {
        parameters.min_detection_baseline = min_detection_baseline;
    }
    if let Some(min_map_baseline) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_MIN_MAP_BASELINE")? {
        parameters.min_map_baseline = min_map_baseline;
    }
    if let Some(height_min) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_HEIGHT_MIN")? {
        parameters.height_min = height_min;
    }
    if let Some(height_max) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_HEIGHT_MAX")? {
        parameters.height_max = height_max;
    }
    if let Some(association_gate) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_ASSOCIATION_GATE")? {
        parameters.association_gate = association_gate;
    }
    if let Some(rms_threshold) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_RMS_THRESHOLD")? {
        parameters.rms_threshold = rms_threshold;
    }
    if let Some(min_score) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_MIN_SCORE")? {
        parameters.min_score = min_score;
    }
    if let Some(score_ratio) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_SCORE_RATIO")? {
        parameters.score_ratio = score_ratio;
    }
    if let Some(residual_weight) = f32_env("LOCALIZATION_3D_REPLAY_GLOBAL_RESIDUAL_WEIGHT")? {
        parameters.residual_weight = residual_weight;
    }
    parameters
        .validate()
        .map_err(|error| IoError::new(ErrorKind::InvalidInput, error))?;
    Ok(parameters)
}

fn reject_removed_env(name: &str) -> Result<(), IoError> {
    if std::env::var_os(name).is_some() {
        Err(IoError::new(
            ErrorKind::InvalidInput,
            format!(
                "{name} was removed; use LOCALIZATION_3D_REPLAY_GLOBAL_ASSOCIATION_GATE, \
                 LOCALIZATION_3D_REPLAY_GLOBAL_RMS_THRESHOLD, and \
                 LOCALIZATION_3D_REPLAY_GLOBAL_SCORE_RATIO instead"
            ),
        ))
    } else {
        Ok(())
    }
}

fn usize_env(name: &str) -> Result<Option<usize>, IoError> {
    parse_env(name)
}

fn f32_env(name: &str) -> Result<Option<f32>, IoError> {
    parse_env(name)
}

fn parse_env<T>(name: &str) -> Result<Option<T>, IoError>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    match std::env::var(name) {
        Ok(value) => value.parse::<T>().map(Some).map_err(|error| {
            IoError::new(
                ErrorKind::InvalidInput,
                format!("invalid {name}={value:?}: {error}"),
            )
        }),
        Err(std::env::VarError::NotPresent) => Ok(None),
        Err(error) => Err(IoError::new(
            ErrorKind::InvalidInput,
            format!("invalid {name}: {error}"),
        )),
    }
}

fn robot_delta_from_visual_odometry_delta(
    delta: &VisualOdometryDelta,
    previous_camera_matrix: &CameraMatrix,
    current_camera_matrix: &CameraMatrix,
) -> nalgebra::Isometry3<f64> {
    let previous_robot_to_left_camera = robot_to_camera(previous_camera_matrix);
    let current_robot_to_left_camera = robot_to_camera(current_camera_matrix);
    let current_robot_to_previous_robot = previous_robot_to_left_camera.inverse()
        * delta.current_left_camera_to_previous_left_camera
        * current_robot_to_left_camera;
    current_robot_to_previous_robot.cast()
}

fn robot_to_camera(camera_matrix: &CameraMatrix) -> nalgebra::Isometry3<f32> {
    (camera_matrix.head_to_camera * camera_matrix.robot_to_head).inner
}

#[derive(Deserialize)]
struct WireVisualOdometryDelta {
    previous_time: Time,
    current_time: Time,
    current_left_camera_to_previous_left_camera: WireIsometry3,
}

impl WireVisualOdometryDelta {
    fn into_visual_odometry_delta(self) -> VisualOdometryDelta {
        VisualOdometryDelta {
            previous_time: self.previous_time,
            current_time: self.current_time,
            current_left_camera_to_previous_left_camera: self
                .current_left_camera_to_previous_left_camera
                .into_isometry(),
        }
    }
}

fn decode_recorded_visual_odometry(
    message: &Message<'_>,
) -> Result<VisualOdometryDelta, Box<dyn Error>> {
    let wire: WireVisualOdometryDelta = decode_recorded_message(message)?;
    Ok(wire.into_visual_odometry_delta())
}

fn write_summary_json(output_dir: &PathBuf, report: &ReplayReport) -> Result<(), Box<dyn Error>> {
    fs::write(
        output_dir.join("summary.json"),
        serde_json::to_string_pretty(report)?,
    )?;
    Ok(())
}

fn write_samples_csv(
    output_dir: &PathBuf,
    samples: &[ComparisonSample],
) -> Result<(), Box<dyn Error>> {
    let mut writer = BufWriter::new(fs::File::create(output_dir.join("samples.csv"))?);
    writeln!(
        writer,
        "variant,timestamp_seconds,replay_time_seconds,graph_delay_seconds,translation_error_norm,translation_error_x,translation_error_y,translation_error_z,roll_error_degrees,pitch_error_degrees,yaw_error_degrees,vo_received,vo_same_interval,vo_adjacent_interval,vo_dropped_invalid,vo_dropped_multi_interval,vo_missing_camera,vo_stale_camera,global_frames,global_associations,nearest_global_feature_seconds,optimizer_status,value_count,factor_count,total_error,vo_mean_rms,vo_max_rms,visual_mean_rms,visual_max_rms,gp_mean_rms,gp_max_rms"
    )?;
    for sample in samples {
        let diagnostics = sample.diagnostics.as_ref();
        writeln!(
            writer,
            "{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{},{},{},{},{},{},{},{},{},{},{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9}",
            sample.variant,
            sample.timestamp_seconds,
            sample.replay_time_seconds,
            sample.graph_delay_seconds,
            sample.translation_error_norm,
            sample.translation_error_xyz[0],
            sample.translation_error_xyz[1],
            sample.translation_error_xyz[2],
            sample.roll_error_degrees,
            sample.pitch_error_degrees,
            sample.yaw_error_degrees,
            sample.cumulative_vo_received,
            sample.cumulative_vo_inserted_same_interval,
            sample.cumulative_vo_inserted_adjacent_interval,
            sample.cumulative_vo_dropped_invalid,
            sample.cumulative_vo_dropped_multi_interval,
            sample.cumulative_vo_skipped_missing_camera_matrix,
            sample.cumulative_vo_stale_camera_matrix,
            sample.cumulative_global_frames_ingested,
            sample.cumulative_global_accepted_associations,
            option_f64(sample.nearest_global_feature_seconds),
            diagnostics
                .map(|diagnostics| diagnostics.optimizer_status.as_str())
                .unwrap_or(""),
            diagnostics
                .map(|diagnostics| diagnostics.value_count)
                .unwrap_or(0),
            diagnostics
                .map(|diagnostics| diagnostics.factor_count)
                .unwrap_or(0),
            diagnostics
                .map(|diagnostics| diagnostics.total_error)
                .unwrap_or(0.0),
            diagnostics
                .map(|diagnostics| diagnostics.visual_odometry_mean_rms)
                .unwrap_or(0.0),
            diagnostics
                .map(|diagnostics| diagnostics.visual_odometry_max_rms)
                .unwrap_or(0.0),
            diagnostics
                .map(|diagnostics| diagnostics.visual_reprojection_mean_rms)
                .unwrap_or(0.0),
            diagnostics
                .map(|diagnostics| diagnostics.visual_reprojection_max_rms)
                .unwrap_or(0.0),
            diagnostics
                .map(|diagnostics| diagnostics.gaussian_process_prior_mean_rms)
                .unwrap_or(0.0),
            diagnostics
                .map(|diagnostics| diagnostics.gaussian_process_prior_max_rms)
                .unwrap_or(0.0),
        )?;
    }
    Ok(())
}

fn recording_path() -> PathBuf {
    std::env::var_os("LOCALIZATION_3D_REPLAY_MCAP")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(DEFAULT_RECORDING_RELATIVE_PATH)
        })
}

fn output_dir() -> PathBuf {
    std::env::var_os("LOCALIZATION_3D_REPLAY_OUTPUT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(DEFAULT_OUTPUT_DIR))
}

fn solve_cadence() -> Duration {
    std::env::var("LOCALIZATION_3D_REPLAY_SOLVE_CADENCE_MS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .map(Duration::from_millis)
        .unwrap_or(DEFAULT_SOLVE_CADENCE)
}

fn invert_dead_reckoning_delta() -> bool {
    std::env::var("LOCALIZATION_3D_REPLAY_INVERT_DEAD_RECKONING")
        .map(|value| matches!(value.as_str(), "1" | "true" | "yes"))
        .unwrap_or(false)
}

fn skip_imu_replay() -> bool {
    env_flag("LOCALIZATION_3D_REPLAY_SKIP_IMU")
}

fn skip_foot_height_replay() -> bool {
    env_flag("LOCALIZATION_3D_REPLAY_SKIP_FOOT_HEIGHTS")
}

fn env_flag(name: &str) -> bool {
    std::env::var(name)
        .map(|value| matches!(value.as_str(), "1" | "true" | "yes"))
        .unwrap_or(false)
}

fn filter_variants(variants: Vec<VariantConfig>) -> Vec<VariantConfig> {
    let Some(filter) = std::env::var("LOCALIZATION_3D_REPLAY_ONLY").ok() else {
        return variants;
    };
    let requested = filter
        .split(',')
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .collect::<Vec<_>>();
    variants
        .into_iter()
        .filter(|variant| requested.iter().any(|name| variant.name.contains(name)))
        .collect()
}

fn system_time_from_nanos(nanos: u64) -> SystemTime {
    UNIX_EPOCH + Duration::from_nanos(nanos)
}

fn nanos_since_epoch(time: SystemTime) -> u128 {
    time.duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::ZERO)
        .as_nanos()
}

fn seconds_since(time: SystemTime, start: SystemTime) -> f64 {
    signed_duration_seconds(time, start)
}

fn signed_duration_seconds(time: SystemTime, start: SystemTime) -> f64 {
    match time.duration_since(start) {
        Ok(duration) => duration.as_secs_f64(),
        Err(error) => -error.duration().as_secs_f64(),
    }
}

fn abs_duration(a: SystemTime, b: SystemTime) -> Duration {
    a.duration_since(b)
        .or_else(|_| b.duration_since(a))
        .unwrap_or(Duration::ZERO)
}

fn nanos_abs_diff(a: SystemTime, b: SystemTime) -> u128 {
    nanos_since_epoch(a).abs_diff(nanos_since_epoch(b))
}

fn duration_ms(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}

fn vector3(vector: nalgebra::Vector3<f64>) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn option_f64(value: Option<f64>) -> String {
    value.map(|value| format!("{value:.9}")).unwrap_or_default()
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<f64>() / values.len() as f64
}

fn max(values: &[f64]) -> f64 {
    values.iter().copied().fold(0.0, f64::max)
}

fn non_empty_mean(values: impl IntoIterator<Item = f64>) -> Option<f64> {
    let mut count = 0;
    let mut sum = 0.0;
    for value in values {
        count += 1;
        sum += value;
    }
    (count > 0).then_some(sum / count as f64)
}

fn non_empty_max(values: impl IntoIterator<Item = f64>) -> Option<f64> {
    let mut max_value: Option<f64> = None;
    for value in values {
        max_value = Some(max_value.map_or(value, |current| current.max(value)));
    }
    max_value
}

fn correlation(
    xs: impl IntoIterator<Item = f64>,
    ys: impl IntoIterator<Item = f64>,
) -> Option<f64> {
    let pairs = xs.into_iter().zip(ys).collect::<Vec<_>>();
    if pairs.len() < 2 {
        return None;
    }
    let mean_x = pairs.iter().map(|(x, _)| x).sum::<f64>() / pairs.len() as f64;
    let mean_y = pairs.iter().map(|(_, y)| y).sum::<f64>() / pairs.len() as f64;
    let mut covariance = 0.0;
    let mut variance_x = 0.0;
    let mut variance_y = 0.0;
    for (x, y) in pairs {
        let dx = x - mean_x;
        let dy = y - mean_y;
        covariance += dx * dy;
        variance_x += dx * dx;
        variance_y += dy * dy;
    }
    if variance_x == 0.0 || variance_y == 0.0 {
        return None;
    }
    Some(covariance / (variance_x.sqrt() * variance_y.sqrt()))
}
