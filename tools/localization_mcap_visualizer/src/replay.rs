use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::Sender,
    },
    time::{Duration, Instant, SystemTime},
};

use color_eyre::Result;
use coordinate_systems::{Field, Robot};
use field_mark_association::{
    FieldMarkAssociationParameters, GlobalLocalizationDebugStatus, GlobalLocalizerParameters,
    find_detected_visual_features, localize_global_visual_features,
};
use linear_algebra::IntoTransform;
use localization_3d::{
    Localization3dParameters, backend_configuration, ingest_foot_heights, ingest_visual_odometry,
};
use localization_factrs::{
    BackendConfiguration, VinsBackend, VinsFrontend, VisualReprojectionAssociation,
    backend::BackendSolveDiagnostics, initialize,
};
use nalgebra::SMatrix;
use projection::camera_matrix::CameraMatrix;
use ros_z::time::Time;
use types::{
    field_dimensions::FieldDimensions, time_wrapper::TimeWrapper,
    visual_odometry::VisualOdometryDelta,
};

use crate::mcap_recording::{
    EventKind, RecordedEvent, Recording, TrajectoryPoint, nanos_abs_diff, seconds_since,
};

#[derive(Clone, Debug, PartialEq)]
pub struct ReplayParameters {
    pub timestamp_mode: TimestampMode,
    pub solve_cadence_ms: f64,
    pub optimizer_iterations: usize,
    pub max_window_seconds: f64,
    pub visual_feature_noise_variance: f64,
    pub visual_odometry_covariance: f64,
    pub include_global_features: bool,
    pub include_imu: bool,
    pub include_foot_heights: bool,
    pub global_localizer: GlobalLocalizerParameters,
}

impl Default for ReplayParameters {
    fn default() -> Self {
        let localization_parameters = Localization3dParameters::default();
        let association_parameters = FieldMarkAssociationParameters::default();
        Self {
            timestamp_mode: TimestampMode::McapPublish,
            solve_cadence_ms: 30.0,
            optimizer_iterations: 5,
            max_window_seconds: 3.0,
            visual_feature_noise_variance: localization_parameters.visual_feature_noise_variance,
            visual_odometry_covariance: 1.0e-4,
            include_global_features: true,
            include_imu: true,
            include_foot_heights: true,
            global_localizer: association_parameters.global_localizer,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimestampMode {
    McapPublish,
    Embedded,
}

#[derive(Clone, Debug)]
pub enum ResolveMessage {
    Progress(ResolveProgress),
    Finished(ResolveResult),
    Failed(String),
    Cancelled,
}

#[derive(Clone, Debug)]
pub struct ResolveProgress {
    pub processed_events: usize,
    pub total_events: usize,
    pub solve_count: usize,
}

#[derive(Clone, Debug)]
pub struct ResolveResult {
    pub parameters: ReplayParameters,
    pub samples: Vec<SolveSample>,
    pub stats: ReplayStats,
    pub elapsed: Duration,
}

impl ResolveResult {
    pub fn trajectory(&self) -> Vec<TrajectoryPoint> {
        self.samples
            .iter()
            .map(|sample| TrajectoryPoint {
                seconds: sample.replay_seconds,
                robot_to_field: sample.robot_to_field,
            })
            .collect()
    }
}

#[derive(Clone, Debug)]
pub struct SolveSample {
    pub replay_seconds: f64,
    pub graph_seconds: f64,
    pub solve_duration: Duration,
    pub robot_to_field: linear_algebra::Isometry3<Robot, Field, f64>,
    pub diagnostics: Option<BackendSolveDiagnostics>,
    pub stats: ReplayStats,
}

#[derive(Clone, Debug, Default)]
pub struct ReplayStats {
    pub imu_ingested: usize,
    pub foot_heights_ingested: usize,
    pub vo_received: usize,
    pub vo_ingested: usize,
    pub vo_dropped_invalid: usize,
    pub vo_skipped_missing_camera_matrix: usize,
    pub vo_skipped_stale_camera_matrix: usize,
    pub global_frames: usize,
    pub global_candidates: usize,
    pub global_none: usize,
    pub global_ambiguous: usize,
    pub global_unique_modulo_symmetry: usize,
    pub global_frames_ingested: usize,
    pub global_associations_ingested: usize,
}

pub fn spawn_resolve(
    recording: Arc<Recording>,
    parameters: ReplayParameters,
    cancelled: Arc<AtomicBool>,
    sender: Sender<ResolveMessage>,
) {
    std::thread::spawn(move || {
        let result = run_resolve(&recording, parameters, &cancelled, &sender);
        let message = match result {
            Ok(Some(result)) => ResolveMessage::Finished(result),
            Ok(None) => ResolveMessage::Cancelled,
            Err(error) => ResolveMessage::Failed(format!("{error:#}")),
        };
        let _ = sender.send(message);
    });
}

fn run_resolve(
    recording: &Recording,
    parameters: ReplayParameters,
    cancelled: &AtomicBool,
    sender: &Sender<ResolveMessage>,
) -> Result<Option<ResolveResult>> {
    let started = Instant::now();
    let initial_state =
        localization_3d::initial_state_from_camera_matrix(&recording.first_camera_matrix);
    let (mut frontend, mut backend) = initialize(backend_config(&parameters), initial_state);
    let mut camera_matrices = OnlineCameraMatrices::default();
    let mut vo_timestamps = VisualOdometryTimestampTracker::default();
    let mut stats = ReplayStats::default();
    let mut samples = Vec::new();
    let mut has_pending_measurements = false;
    let cadence = Duration::from_secs_f64((parameters.solve_cadence_ms / 1000.0).max(0.001));
    let mut next_solve_time = recording.start_log_time() + cadence;
    let total_events = recording.event_count();

    for (index, event) in recording.events().iter().enumerate() {
        if cancelled.load(Ordering::Relaxed) {
            return Ok(None);
        }

        while next_solve_time <= event.log_time {
            if has_pending_measurements {
                solve_and_record(
                    &mut backend,
                    &mut frontend,
                    recording,
                    next_solve_time,
                    &stats,
                    &mut samples,
                )?;
                has_pending_measurements = false;
            }
            next_solve_time += cadence;
        }

        match &event.kind {
            EventKind::Imu(imu) if parameters.include_imu => {
                frontend.ingest_imu(event.publish_time, *imu)?;
                stats.imu_ingested += 1;
                has_pending_measurements = true;
            }
            EventKind::CameraMatrix(camera_matrix) => {
                camera_matrices.push(event, camera_matrix.clone());
            }
            EventKind::RobotKinematics(robot_kinematics) if parameters.include_foot_heights => {
                let mut robot_kinematics = robot_kinematics.as_ref().clone();
                if parameters.timestamp_mode == TimestampMode::McapPublish {
                    robot_kinematics.time = Time::from_wallclock(event.publish_time);
                }
                ingest_foot_heights(&mut frontend, robot_kinematics)?;
                stats.foot_heights_ingested += 1;
                has_pending_measurements = true;
            }
            EventKind::VisualOdometry(delta) => {
                ingest_vo_event(
                    recording,
                    event,
                    delta,
                    parameters.timestamp_mode,
                    &mut vo_timestamps,
                    &camera_matrices,
                    &mut frontend,
                    &mut stats,
                    &mut has_pending_measurements,
                )?;
            }
            EventKind::DetectedObjects(objects) if parameters.include_global_features => {
                ingest_global_features(
                    event,
                    objects,
                    &camera_matrices,
                    &mut frontend,
                    &parameters,
                    &mut stats,
                    &mut has_pending_measurements,
                )?;
            }
            EventKind::DetectedObjects(_) => {
                stats.global_frames += 1;
            }
            _ => {}
        }

        if index % 250 == 0 {
            let _ = sender.send(ResolveMessage::Progress(ResolveProgress {
                processed_events: index + 1,
                total_events,
                solve_count: samples.len(),
            }));
        }
    }

    if has_pending_measurements {
        solve_and_record(
            &mut backend,
            &mut frontend,
            recording,
            recording.end_log_time(),
            &stats,
            &mut samples,
        )?;
    }

    Ok(Some(ResolveResult {
        parameters,
        samples,
        stats,
        elapsed: started.elapsed(),
    }))
}

fn backend_config(parameters: &ReplayParameters) -> BackendConfiguration {
    let mut config = backend_configuration(parameters.visual_feature_noise_variance);
    config.optimizer_max_iterations = parameters.optimizer_iterations.max(1);
    config.max_optimization_window =
        Duration::from_secs_f64(parameters.max_window_seconds.max(0.2));
    config.visual_odometry_noise =
        SMatrix::<f64, 6, 6>::identity() * parameters.visual_odometry_covariance.max(1.0e-12);
    config
}

#[allow(clippy::too_many_arguments)]
fn ingest_vo_event(
    recording: &Recording,
    event: &RecordedEvent,
    delta: &VisualOdometryDelta,
    timestamp_mode: TimestampMode,
    vo_timestamps: &mut VisualOdometryTimestampTracker,
    camera_matrices: &OnlineCameraMatrices,
    frontend: &mut VinsFrontend,
    stats: &mut ReplayStats,
    has_pending_measurements: &mut bool,
) -> Result<()> {
    stats.vo_received += 1;
    let Some((previous_time, current_time)) =
        vo_timestamps.measurement_times(event, delta, timestamp_mode, recording)
    else {
        stats.vo_dropped_invalid += 1;
        return Ok(());
    };
    if current_time <= previous_time {
        stats.vo_dropped_invalid += 1;
        return Ok(());
    }

    let Some(previous_camera_matrix) = camera_matrices.nearest(previous_time, timestamp_mode)
    else {
        stats.vo_skipped_missing_camera_matrix += 1;
        return Ok(());
    };
    let Some(current_camera_matrix) = camera_matrices.nearest(current_time, timestamp_mode) else {
        stats.vo_skipped_missing_camera_matrix += 1;
        return Ok(());
    };
    if previous_camera_matrix.distance > STALE_CAMERA_MATRIX_THRESHOLD
        || current_camera_matrix.distance > STALE_CAMERA_MATRIX_THRESHOLD
    {
        stats.vo_skipped_stale_camera_matrix += 1;
        return Ok(());
    }

    let mut delta = delta.clone();
    delta.previous_time = Time::from_wallclock(previous_time);
    delta.current_time = Time::from_wallclock(current_time);
    ingest_visual_odometry(
        frontend,
        delta,
        &previous_camera_matrix.matrix.matrix.inner,
        &current_camera_matrix.matrix.matrix.inner,
    )?;
    stats.vo_ingested += 1;
    *has_pending_measurements = true;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn ingest_global_features(
    event: &RecordedEvent,
    objects: &[types::object_detection::Object<types::object_detection::RobocupObjectLabel>],
    camera_matrices: &OnlineCameraMatrices,
    frontend: &mut VinsFrontend,
    parameters: &ReplayParameters,
    stats: &mut ReplayStats,
    has_pending_measurements: &mut bool,
) -> Result<()> {
    stats.global_frames += 1;
    let visual_features = find_detected_visual_features(objects);
    if visual_features.supported_feature_count() < parameters.global_localizer.min_inliers.max(3) {
        return Ok(());
    }
    stats.global_candidates += 1;

    let Some(camera_matrix) = camera_matrices
        .nearest(event.publish_time, parameters.timestamp_mode)
        .map(|nearest| nearest.matrix)
    else {
        return Ok(());
    };
    if camera_matrix.distance_to(event.publish_time, parameters.timestamp_mode)
        > STALE_CAMERA_MATRIX_THRESHOLD
    {
        return Ok(());
    }

    let pose_hint = frontend.peek_last_optimization_result().map(|result| {
        result
            .transform
            .cast::<f32>()
            .framed_transform::<Robot, Field>()
    });
    let localization = localize_global_visual_features(
        &visual_features,
        &camera_matrix.matrix.inner,
        &FieldDimensions::SPL_2025,
        pose_hint,
        &parameters.global_localizer,
    );
    match localization.debug.as_ref().map(|debug| debug.status) {
        None => stats.global_none += 1,
        Some(GlobalLocalizationDebugStatus::Ambiguous) => stats.global_ambiguous += 1,
        #[allow(deprecated)]
        Some(GlobalLocalizationDebugStatus::Unique) => stats.global_ambiguous += 1,
        Some(GlobalLocalizationDebugStatus::UniqueModuloSymmetry) => {
            stats.global_unique_modulo_symmetry += 1;
        }
    }

    if let Some(associations) = localization.unique_associations {
        stats.global_frames_ingested += 1;
        stats.global_associations_ingested += associations.len();
        let associations =
            associations
                .into_iter()
                .map(|association| VisualReprojectionAssociation {
                    detection: association.detection,
                    field_point: association.field_point,
                });
        frontend.ingest_visual_reprojection_associations(
            event.publish_time,
            associations,
            robot_to_camera(&camera_matrix.matrix.inner),
        )?;
        *has_pending_measurements = true;
    }
    Ok(())
}

fn solve_and_record(
    backend: &mut VinsBackend,
    frontend: &mut VinsFrontend,
    recording: &Recording,
    replay_time: SystemTime,
    stats: &ReplayStats,
    samples: &mut Vec<SolveSample>,
) -> Result<()> {
    let solve_started = Instant::now();
    let backend_result = backend.solve_once()?;
    let solve_duration = solve_started.elapsed();
    if backend_result.is_none() {
        return Ok(());
    }
    let Some(result) = frontend.last_optimization_result() else {
        return Ok(());
    };

    samples.push(SolveSample {
        replay_seconds: recording.seconds_since_start(replay_time),
        graph_seconds: seconds_since(result.time, recording.start_source_time()),
        solve_duration,
        robot_to_field: result.transform.framed_transform(),
        diagnostics: backend.last_solve_diagnostics().cloned(),
        stats: stats.clone(),
    });
    Ok(())
}

#[derive(Default)]
struct OnlineCameraMatrices {
    matrices: Vec<OnlineCameraMatrix>,
}

impl OnlineCameraMatrices {
    fn push(&mut self, event: &RecordedEvent, matrix: TimeWrapper<CameraMatrix>) {
        self.matrices.push(OnlineCameraMatrix {
            publish_time: event.publish_time,
            matrix,
        });
    }

    fn nearest(
        &self,
        time: SystemTime,
        timestamp_mode: TimestampMode,
    ) -> Option<NearestCameraMatrix<'_>> {
        self.matrices
            .iter()
            .min_by_key(|candidate| nanos_abs_diff(candidate.time(timestamp_mode), time))
            .map(|matrix| NearestCameraMatrix {
                matrix,
                distance: Duration::from_nanos(
                    nanos_abs_diff(matrix.time(timestamp_mode), time).min(u64::MAX as u128) as u64,
                ),
            })
    }
}

struct OnlineCameraMatrix {
    publish_time: SystemTime,
    matrix: TimeWrapper<CameraMatrix>,
}

impl OnlineCameraMatrix {
    fn time(&self, timestamp_mode: TimestampMode) -> SystemTime {
        match timestamp_mode {
            TimestampMode::McapPublish => self.publish_time,
            TimestampMode::Embedded => self.matrix.time.to_wallclock(),
        }
    }

    fn distance_to(&self, time: SystemTime, timestamp_mode: TimestampMode) -> Duration {
        Duration::from_nanos(
            nanos_abs_diff(self.time(timestamp_mode), time).min(u64::MAX as u128) as u64,
        )
    }
}

struct NearestCameraMatrix<'a> {
    matrix: &'a OnlineCameraMatrix,
    distance: Duration,
}

#[derive(Default)]
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
                let current = match recording.aligned_image_time(delta.current_time) {
                    Some(time) => time,
                    None => event.publish_time,
                };
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

fn robot_to_camera(camera_matrix: &CameraMatrix) -> nalgebra::Isometry3<f32> {
    (camera_matrix.head_to_camera * camera_matrix.robot_to_head).inner
}

const STALE_CAMERA_MATRIX_THRESHOLD: Duration = Duration::from_millis(100);
