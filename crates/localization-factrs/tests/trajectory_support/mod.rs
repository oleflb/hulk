use std::{
    error::Error,
    fs,
    path::Path,
    time::{Duration, Instant, SystemTime},
};

use booster::ImuState;
use factrs::{core::SO3, variables::SE23};
use indicatif::ProgressIterator;
use linear_algebra::IntoFramed;
use localization_factrs::{
    BackendConfiguration, CameraIntrinsics, InitialState, LandmarkAssociationCosts, initialize,
};
use nalgebra::{Matrix2, Matrix3, Point2, Point3, SMatrix, Vector3, vector};
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, StandardNormal};
use serde::{Deserialize, Serialize};

const KNOT_SPACING: Duration = Duration::from_millis(200);
const MAX_OPTIMIZATION_WINDOW: Duration = Duration::from_secs(5);
const MIN_SOLVER_STD: f64 = 1.0e-6;
const DENSE_UNMATCHED_COST: f64 = 1.0e3;

#[derive(Debug, Clone, Copy)]
pub struct TrajectoryTestConfig {
    pub initial_state: InitialStateConfig,
    pub measurement_selection: MeasurementSelection,
    pub visual_outliers: Option<VisualOutlierConfig>,
    pub solve_every_nth_frame: usize,
    pub optimizer_max_iterations: usize,
    pub output_path: &'static str,
    pub gyro_noise_std: f64,
    pub accel_noise_std: f64,
    pub detection_noise_std: f64,
    pub noise_seed: u64,
    pub max_position_rmse_meters: f64,
    pub max_orientation_rmse_degrees: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct VisualOutlierConfig {
    pub false_detection_probability: f64,
    pub real_detection_dropout_probability: f64,
    pub seed: u64,
}

#[derive(Debug, Clone, Copy)]
pub enum InitialStateConfig {
    FromSimulationStartPose,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum MeasurementSelection {
    All,
    ImuOnly,
    VisualOnly,
}

impl MeasurementSelection {
    fn use_imu(self) -> bool {
        matches!(self, Self::All | Self::ImuOnly)
    }

    fn use_visual(self) -> bool {
        matches!(self, Self::All | Self::VisualOnly)
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct SimulationData {
    start_pose: SimulatorState,
    landmark_global_positions: LandmarkGlobalPositions,
    measurements: Vec<SimulatorMeasurement>,
}

#[derive(Debug, Serialize, Deserialize)]
struct LandmarkGlobalPositions {
    corner_top_left: [f64; 3],
    corner_top_right: [f64; 3],
    corner_bottom_left: [f64; 3],
    corner_bottom_right: [f64; 3],
}

#[derive(Debug, Serialize, Deserialize)]
struct SimulatorMeasurement {
    timestamp_seconds: f64,
    imu: SimulatorImuMeasurement,
    ground_truth_pose: SimulatorState,
    visual_features: Option<SimulatorDetectedFeatures>,
}

#[derive(Debug, Serialize, Deserialize)]
struct SimulatorImuMeasurement {
    angular_velocity: [f64; 3],
    linear_acceleration: [f64; 3],
}

#[derive(Debug, Serialize, Deserialize)]
struct SimulatorState {
    position: [f64; 3],
    quaternion_wxyz: [f64; 4],
    linear_velocity_global: [f64; 3],
    angular_velocity_local: [f64; 3],
}

#[derive(Debug, Serialize, Deserialize)]
struct SimulatorDetectedFeatures {
    detections: Vec<[f64; 2]>,
    intrinsics: SimulatorCameraIntrinsics,
}

#[derive(Debug, Serialize, Deserialize)]
struct SimulatorCameraIntrinsics {
    focal_x: f64,
    focal_y: f64,
    center_x: f64,
    center_y: f64,
}

#[derive(Debug, Serialize)]
struct TrajectoryOutput {
    ground_truth: Vec<PoseSample>,
    optimized: Vec<PoseSample>,
    landmarks: Vec<LandmarkSample>,
}

#[derive(Debug, Clone, Serialize)]
struct PoseSample {
    timestamp_seconds: f64,
    position: [f64; 3],
    quaternion_wxyz: [f64; 4],
}

#[derive(Debug, Serialize)]
struct LandmarkSample {
    name: &'static str,
    position: [f64; 3],
}

#[derive(Debug)]
struct TrajectoryMetrics {
    compared_samples: usize,
    excluded_outside_ground_truth: usize,
    position_rmse: f64,
    position_mean: f64,
    position_median: f64,
    position_p95: f64,
    position_max: f64,
    position_axis_rmse: [f64; 3],
    orientation_rmse_degrees: f64,
    orientation_mean_degrees: f64,
    orientation_max_degrees: f64,
}

struct SensorNoise {
    rng: ChaCha8Rng,
    gyro_std: f64,
    accel_std: f64,
    detection_std: f64,
}

struct VisualOutlierInjector {
    rng: ChaCha8Rng,
    false_detection_probability: f64,
    real_detection_dropout_probability: f64,
    injected_false_detections: usize,
    dropped_real_detections: usize,
}

pub fn run_trajectory_test(config: TrajectoryTestConfig) -> Result<(), Box<dyn Error>> {
    let _ = env_logger::builder().is_test(true).try_init();

    let data = load_simulation_data()?;
    let mut sensor_noise = SensorNoise::new(&config);
    let mut visual_outliers = config.visual_outliers.map(VisualOutlierInjector::new);
    let initial_state = initial_state(&data, config.initial_state);
    let (mut frontend, mut backend) = initialize(
        BackendConfiguration {
            knot_spacing: KNOT_SPACING,
            max_optimization_window: MAX_OPTIMIZATION_WINDOW,
            optimizer_max_iterations: config.optimizer_max_iterations,
            gyroscope_noise: Matrix3::identity() * solver_variance(config.gyro_noise_std),
            accelerometer_noise: Matrix3::identity() * solver_variance(config.accel_noise_std),
            gyroscope_process_noise: Matrix3::identity() * 0.01,
            accelerometer_process_noise: Matrix3::identity() * 0.01,
            visual_feature_noise: Matrix2::identity() * solver_variance(config.detection_noise_std),
            visual_odometry_noise: SMatrix::<f64, 6, 6>::identity() * 0.05,
            gravity: vector![0.0, 0.0, 9.81],
        },
        initial_state,
    );
    let start_time = SystemTime::UNIX_EPOCH;
    let robot_to_camera = robot_to_camera();
    let landmarks = ordered_landmark_candidates(&data);
    let mut realtime_trajectory = Vec::new();
    let mut total_solve_time_including_ingestion = Duration::ZERO;

    assert!(
        config.solve_every_nth_frame > 0,
        "solve cadence must be positive"
    );

    for (frame_index, frame) in data.measurements.iter().progress().enumerate() {
        let time = start_time + Duration::from_secs_f64(frame.timestamp_seconds);
        let solve_start = Instant::now();

        if config.measurement_selection.use_imu() {
            let angular_velocity = sensor_noise
                .noisy_gyroscope(frame.imu.angular_velocity)
                .cast::<f32>()
                .framed();
            let linear_acceleration = sensor_noise
                .noisy_acceleration(frame.imu.linear_acceleration)
                .cast::<f32>()
                .framed();
            frontend.ingest_imu(
                time,
                ImuState {
                    roll_pitch_yaw: vector![0.0, 0.0, 0.0].framed(),
                    angular_velocity,
                    linear_acceleration,
                },
            )?;
        }

        if config.measurement_selection.use_visual()
            && let Some(visual_features) = &frame.visual_features
        {
            let mut detections = Vec::new();
            for detection in &visual_features.detections {
                if visual_outliers
                    .as_mut()
                    .is_some_and(VisualOutlierInjector::drop_real_detection)
                {
                    continue;
                }

                detections.push(sensor_noise.noisy_detection(*detection));
            }

            if let Some(visual_outliers) = visual_outliers.as_mut() {
                visual_outliers.maybe_push_outlier(&mut detections, visual_features);
            }

            if !detections.is_empty() {
                frontend.ingest_visual_with_association_costs(
                    time,
                    detections,
                    landmarks.clone(),
                    robot_to_camera,
                    Some(LandmarkAssociationCosts {
                        unmatched_landmark: DENSE_UNMATCHED_COST,
                        unmatched_detection: DENSE_UNMATCHED_COST,
                    }),
                )?;
            }
        }

        if should_solve_frame(frame_index, config.solve_every_nth_frame) {
            let _ = backend.solve_once()?;
            total_solve_time_including_ingestion += solve_start.elapsed();
        }

        if let Some(result) = frontend.last_optimization_result()
            && let Some(sample) = pose_sample_from_optimization_result(&result, start_time)
        {
            realtime_trajectory.push(sample);
        }
    }

    let output = trajectory_output(&data, realtime_trajectory);
    let metrics = trajectory_metrics(&output.ground_truth, &output.optimized)
        .ok_or("no optimized samples overlap ground truth")?;

    write_report_and_output(
        config,
        &output,
        &metrics,
        total_solve_time_including_ingestion,
        visual_outliers.as_ref().map_or(0, |visual_outliers| {
            visual_outliers.injected_false_detections
        }),
        visual_outliers
            .as_ref()
            .map_or(0, |visual_outliers| visual_outliers.dropped_real_detections),
    )?;

    assert!(
        metrics.position_rmse <= config.max_position_rmse_meters,
        "position RMSE {:.6} m exceeded {:.6} m",
        metrics.position_rmse,
        config.max_position_rmse_meters,
    );
    assert!(
        metrics.orientation_rmse_degrees <= config.max_orientation_rmse_degrees,
        "orientation RMSE {:.6} deg exceeded {:.6} deg",
        metrics.orientation_rmse_degrees,
        config.max_orientation_rmse_degrees,
    );

    Ok(())
}

fn should_solve_frame(frame_index: usize, solve_every_nth_frame: usize) -> bool {
    (frame_index + 1).is_multiple_of(solve_every_nth_frame)
}

fn load_simulation_data() -> Result<SimulationData, serde_json::Error> {
    serde_json::from_str(include_str!("../../example_trajectory.json"))
}

fn write_report_and_output(
    config: TrajectoryTestConfig,
    output: &TrajectoryOutput,
    metrics: &TrajectoryMetrics,
    total_solve_time_including_ingestion: Duration,
    injected_false_detections: usize,
    dropped_real_detections: usize,
) -> Result<(), Box<dyn Error>> {
    fs::write(
        Path::new(config.output_path),
        serde_json::to_string_pretty(output)?,
    )?;

    println!("trajectory error metrics:");
    println!("  compared samples: {}", metrics.compared_samples);
    println!(
        "  excluded outside ground truth: {}",
        metrics.excluded_outside_ground_truth
    );
    println!("  position RMSE: {:.6} m", metrics.position_rmse);
    println!("  position mean: {:.6} m", metrics.position_mean);
    println!("  position median: {:.6} m", metrics.position_median);
    println!("  position P95: {:.6} m", metrics.position_p95);
    println!("  position max: {:.6} m", metrics.position_max);
    println!(
        "  axis RMSE: x={:.6} m, y={:.6} m, z={:.6} m",
        metrics.position_axis_rmse[0], metrics.position_axis_rmse[1], metrics.position_axis_rmse[2]
    );
    println!(
        "  orientation RMSE: {:.6} deg",
        metrics.orientation_rmse_degrees
    );
    println!(
        "  orientation mean: {:.6} deg",
        metrics.orientation_mean_degrees
    );
    println!(
        "  orientation max: {:.6} deg",
        metrics.orientation_max_degrees
    );
    println!(
        "  total backend solve time including queued ingestion: {:.3} s",
        total_solve_time_including_ingestion.as_secs_f64()
    );
    println!("  injected false visual detections: {injected_false_detections}");
    println!("  dropped real visual detections: {dropped_real_detections}");
    println!("wrote {}", config.output_path);

    Ok(())
}

impl VisualOutlierInjector {
    fn new(config: VisualOutlierConfig) -> Self {
        assert!(
            (0.0..=1.0).contains(&config.false_detection_probability),
            "false detection probability must be in [0, 1]"
        );
        assert!(
            (0.0..=1.0).contains(&config.real_detection_dropout_probability),
            "real detection dropout probability must be in [0, 1]"
        );

        Self {
            rng: ChaCha8Rng::seed_from_u64(config.seed),
            false_detection_probability: config.false_detection_probability,
            real_detection_dropout_probability: config.real_detection_dropout_probability,
            injected_false_detections: 0,
            dropped_real_detections: 0,
        }
    }

    fn drop_real_detection(&mut self) -> bool {
        let drop = self
            .rng
            .random_bool(self.real_detection_dropout_probability);
        if drop {
            self.dropped_real_detections += 1;
        }
        drop
    }

    fn maybe_push_outlier(
        &mut self,
        detections: &mut Vec<Point2<f64>>,
        visual_features: &SimulatorDetectedFeatures,
    ) {
        if !self.rng.random_bool(self.false_detection_probability) {
            return;
        }

        detections.push(misdetected_visual_feature(visual_features));
        self.injected_false_detections += 1;
    }
}

fn misdetected_visual_feature(visual_features: &SimulatorDetectedFeatures) -> Point2<f64> {
    let intrinsics = &visual_features.intrinsics;
    let image_width = intrinsics.center_x * 2.0;
    let image_height = intrinsics.center_y * 2.0;

    // Pick the image-region sample farthest from real detections so the added
    // feature behaves like an unrelated environmental corner, not extra noise.
    let candidates = [
        Point2::new(image_width * 0.1, image_height * 0.1),
        Point2::new(image_width * 0.9, image_height * 0.1),
        Point2::new(image_width * 0.9, image_height * 0.9),
        Point2::new(image_width * 0.1, image_height * 0.9),
        Point2::new(intrinsics.center_x, intrinsics.center_y),
    ];

    candidates
        .into_iter()
        .max_by(|left, right| {
            minimum_detection_distance(*left, visual_features)
                .total_cmp(&minimum_detection_distance(*right, visual_features))
        })
        .expect("misdetection candidates must not be empty")
}

fn minimum_detection_distance(
    detection: Point2<f64>,
    visual_features: &SimulatorDetectedFeatures,
) -> f64 {
    visual_features
        .detections
        .iter()
        .map(|reference| (detection - Point2::from(*reference)).norm())
        .fold(f64::INFINITY, f64::min)
}

impl SensorNoise {
    fn new(config: &TrajectoryTestConfig) -> Self {
        Self {
            rng: ChaCha8Rng::seed_from_u64(config.noise_seed),
            gyro_std: config.gyro_noise_std,
            accel_std: config.accel_noise_std,
            detection_std: config.detection_noise_std,
        }
    }

    fn noisy_gyroscope(&mut self, value: [f64; 3]) -> Vector3<f64> {
        let std = self.gyro_std;
        self.noisy_imu_vector(value, std)
    }

    fn noisy_acceleration(&mut self, value: [f64; 3]) -> Vector3<f64> {
        let std = self.accel_std;
        self.noisy_imu_vector(value, std)
    }

    fn noisy_imu_vector(&mut self, value: [f64; 3], std: f64) -> Vector3<f64> {
        Vector3::from(value).map(|component| component + self.sample(std))
    }

    fn noisy_detection(&mut self, detection: [f64; 2]) -> Point2<f64> {
        let std = self.detection_std;
        Point2::new(
            detection[0] + self.sample(std),
            detection[1] + self.sample(std),
        )
    }

    fn sample(&mut self, std: f64) -> f64 {
        if std == 0.0 {
            return 0.0;
        }

        let sample: f64 = StandardNormal.sample(&mut self.rng);
        sample * std
    }
}

fn solver_variance(noise_std: f64) -> f64 {
    noise_std.max(MIN_SOLVER_STD).powi(2)
}

fn initial_state(data: &SimulationData, config: InitialStateConfig) -> InitialState {
    match config {
        InitialStateConfig::FromSimulationStartPose => initial_state_from_simulation(data),
    }
}

fn initial_state_from_simulation(data: &SimulationData) -> InitialState {
    let camera_intrinsics = data
        .measurements
        .iter()
        .find_map(|measurement| measurement.visual_features.as_ref())
        .map(|visual_features| {
            let intrinsics = &visual_features.intrinsics;
            CameraIntrinsics::new(
                vector![intrinsics.focal_x, intrinsics.focal_y],
                vector![intrinsics.center_x, intrinsics.center_y],
            )
        })
        .unwrap_or_else(|| CameraIntrinsics::new(vector![1.0, 1.0], vector![0.0, 0.0]));

    InitialState::new(simulator_state_to_se23(&data.start_pose), camera_intrinsics)
}

fn simulator_state_to_se23(state: &SimulatorState) -> SE23<f64> {
    let [w, x, y, z] = state.quaternion_wxyz;
    let rotation = SO3::from_xyzw(x, y, z, w);
    let velocity = Vector3::from(state.linear_velocity_global);
    let translation = Vector3::from(state.position);

    SE23::from_rot_vel_trans(rotation, velocity, translation)
}

fn robot_to_camera() -> nalgebra::Isometry3<f32> {
    let rotation = nalgebra::UnitQuaternion::from_matrix(&nalgebra::matrix![
        1.0, 0.0, 0.0;
        0.0, 0.0, -1.0;
        0.0, 1.0, 0.0;
    ]);

    nalgebra::Isometry3::from_parts(nalgebra::Translation3::from(Vector3::zeros()), rotation)
}

fn ordered_landmark_candidates(data: &SimulationData) -> Vec<Point3<f64>> {
    let landmarks = &data.landmark_global_positions;
    [
        landmarks.corner_top_left,
        landmarks.corner_top_right,
        landmarks.corner_bottom_right,
        landmarks.corner_bottom_left,
    ]
    .into_iter()
    .map(Point3::from)
    .collect()
}

fn trajectory_output(data: &SimulationData, optimized: Vec<PoseSample>) -> TrajectoryOutput {
    let ground_truth = data
        .measurements
        .iter()
        .map(|measurement| PoseSample {
            timestamp_seconds: measurement.timestamp_seconds,
            position: measurement.ground_truth_pose.position,
            quaternion_wxyz: measurement.ground_truth_pose.quaternion_wxyz,
        })
        .collect();

    TrajectoryOutput {
        ground_truth,
        optimized,
        landmarks: landmark_output(data),
    }
}

fn pose_sample_from_optimization_result(
    result: &localization_factrs::OptimizationResult,
    start_time: SystemTime,
) -> Option<PoseSample> {
    let timestamp_seconds = result.time.duration_since(start_time).ok()?.as_secs_f64();
    let translation = result.transform.translation.vector;
    let rotation = result.transform.rotation.quaternion();

    Some(PoseSample {
        timestamp_seconds,
        position: vector_to_array(translation.as_view()),
        quaternion_wxyz: [rotation.w, rotation.i, rotation.j, rotation.k],
    })
}

fn trajectory_metrics(
    ground_truth: &[PoseSample],
    optimized: &[PoseSample],
) -> Option<TrajectoryMetrics> {
    let mut excluded_outside_ground_truth = 0;
    let mut position_errors = Vec::new();
    let mut orientation_errors_degrees = Vec::new();
    let mut axis_squared_error_sum = [0.0; 3];

    for estimate in optimized {
        let Some(reference) = interpolate_pose(ground_truth, estimate.timestamp_seconds) else {
            excluded_outside_ground_truth += 1;
            continue;
        };

        let position_error = [
            estimate.position[0] - reference.position[0],
            estimate.position[1] - reference.position[1],
            estimate.position[2] - reference.position[2],
        ];

        for index in 0..3 {
            axis_squared_error_sum[index] += position_error[index] * position_error[index];
        }

        position_errors.push(vector_norm(position_error));
        orientation_errors_degrees.push(
            orientation_error_radians(estimate.quaternion_wxyz, reference.quaternion_wxyz)
                .to_degrees(),
        );
    }

    if position_errors.is_empty() {
        return None;
    }

    let compared_samples = position_errors.len();
    let position_axis_rmse =
        axis_squared_error_sum.map(|sum| (sum / compared_samples as f64).sqrt());

    Some(TrajectoryMetrics {
        compared_samples,
        excluded_outside_ground_truth,
        position_rmse: rmse(&position_errors),
        position_mean: mean(&position_errors),
        position_median: percentile(&position_errors, 50.0),
        position_p95: percentile(&position_errors, 95.0),
        position_max: position_errors.iter().copied().fold(0.0, f64::max),
        position_axis_rmse,
        orientation_rmse_degrees: rmse(&orientation_errors_degrees),
        orientation_mean_degrees: mean(&orientation_errors_degrees),
        orientation_max_degrees: orientation_errors_degrees
            .iter()
            .copied()
            .fold(0.0, f64::max),
    })
}

fn interpolate_pose(ground_truth: &[PoseSample], timestamp_seconds: f64) -> Option<PoseSample> {
    let first = ground_truth.first()?;
    let last = ground_truth.last()?;

    if timestamp_seconds < first.timestamp_seconds || timestamp_seconds > last.timestamp_seconds {
        return None;
    }
    if timestamp_seconds == first.timestamp_seconds {
        return Some(first.clone());
    }
    if timestamp_seconds == last.timestamp_seconds {
        return Some(last.clone());
    }

    let upper_index =
        ground_truth.partition_point(|sample| sample.timestamp_seconds < timestamp_seconds);
    let before = &ground_truth[upper_index - 1];
    let after = &ground_truth[upper_index];
    let time_delta = after.timestamp_seconds - before.timestamp_seconds;
    let interpolation = if time_delta == 0.0 {
        0.0
    } else {
        (timestamp_seconds - before.timestamp_seconds) / time_delta
    };

    Some(PoseSample {
        timestamp_seconds,
        position: lerp3(before.position, after.position, interpolation),
        quaternion_wxyz: slerp_quaternion(
            before.quaternion_wxyz,
            after.quaternion_wxyz,
            interpolation,
        ),
    })
}

fn lerp3(start: [f64; 3], end: [f64; 3], interpolation: f64) -> [f64; 3] {
    [
        lerp(start[0], end[0], interpolation),
        lerp(start[1], end[1], interpolation),
        lerp(start[2], end[2], interpolation),
    ]
}

fn lerp(start: f64, end: f64, interpolation: f64) -> f64 {
    start + (end - start) * interpolation
}

fn slerp_quaternion(start: [f64; 4], end: [f64; 4], interpolation: f64) -> [f64; 4] {
    let start = normalize_quaternion(start);
    let mut end = normalize_quaternion(end);
    let mut dot = dot4(start, end);

    if dot < 0.0 {
        end = end.map(|value| -value);
        dot = -dot;
    }

    let dot = dot.clamp(-1.0, 1.0);
    if dot > 0.9995 {
        return normalize_quaternion([
            lerp(start[0], end[0], interpolation),
            lerp(start[1], end[1], interpolation),
            lerp(start[2], end[2], interpolation),
            lerp(start[3], end[3], interpolation),
        ]);
    }

    let theta = dot.acos();
    let sin_theta = theta.sin();
    let start_weight = ((1.0 - interpolation) * theta).sin() / sin_theta;
    let end_weight = (interpolation * theta).sin() / sin_theta;

    [
        start_weight * start[0] + end_weight * end[0],
        start_weight * start[1] + end_weight * end[1],
        start_weight * start[2] + end_weight * end[2],
        start_weight * start[3] + end_weight * end[3],
    ]
}

fn orientation_error_radians(estimate: [f64; 4], reference: [f64; 4]) -> f64 {
    let estimate = normalize_quaternion(estimate);
    let reference = normalize_quaternion(reference);
    let dot = dot4(estimate, reference).abs().clamp(-1.0, 1.0);
    2.0 * dot.acos()
}

fn normalize_quaternion(quaternion: [f64; 4]) -> [f64; 4] {
    let norm = dot4(quaternion, quaternion).sqrt();
    quaternion.map(|value| value / norm)
}

fn dot4(left: [f64; 4], right: [f64; 4]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2] + left[3] * right[3]
}

fn vector_norm(vector: [f64; 3]) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

fn rmse(values: &[f64]) -> f64 {
    (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
}

fn percentile(values: &[f64], percentile: f64) -> f64 {
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);

    if sorted.len() == 1 {
        return sorted[0];
    }

    let index = (sorted.len() - 1) as f64 * percentile / 100.0;
    let lower = index.floor() as usize;
    let upper = index.ceil() as usize;

    if lower == upper {
        return sorted[lower];
    }

    let interpolation = index - lower as f64;
    lerp(sorted[lower], sorted[upper], interpolation)
}

fn landmark_output(data: &SimulationData) -> Vec<LandmarkSample> {
    vec![
        LandmarkSample {
            name: "corner_top_left",
            position: data.landmark_global_positions.corner_top_left,
        },
        LandmarkSample {
            name: "corner_top_right",
            position: data.landmark_global_positions.corner_top_right,
        },
        LandmarkSample {
            name: "corner_bottom_right",
            position: data.landmark_global_positions.corner_bottom_right,
        },
        LandmarkSample {
            name: "corner_bottom_left",
            position: data.landmark_global_positions.corner_bottom_left,
        },
    ]
}

fn vector_to_array(vector: nalgebra::VectorView3<'_, f64>) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}
