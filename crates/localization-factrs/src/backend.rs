use std::{
    cell::OnceCell,
    time::{Duration, SystemTime},
};

use factrs::{
    containers::FactorBuilder,
    core::{GaussNewton, Graph, PriorResidual, Values, Vector3},
    linalg::Matrix3,
    noise::GaussianNoise,
    optimizers::{BaseOptParams, OptError, OptStatus},
    traits::Optimizer,
    variables::SE23,
};
use itertools::Itertools;
use nalgebra::{Matrix2, SMatrix};
use thiserror::Error;

use crate::{
    gaussian_process_prior_factor::GaussianProcessPriorFactor,
    imu_factor::IntervalGaussianProcessImuFactor,
    initial_state::InitialState,
    interval_measurement::IntervalMeasurements,
    landmark_factor::LandmarkFactor,
    measurements::{ImuMeasurement, SensorMeasurement, VisualMeasurement},
    positive_z_factor::PositiveZFactor,
    schur_marginalization::marginalize,
    splines::SE23Spline,
    symbols::{CameraIntrinsics, State},
    tau,
    visual_odometry_factors::{Measurement as VisualOdometryMeasurement, VisualOdometryFactor},
};

use tokio::sync::{
    mpsc::{UnboundedReceiver, error::TryRecvError},
    watch,
};

const INITIAL_CAMERA_INTRINSICS_PRIOR_SIGMA: f64 = 1.0e-6;
const INITIAL_POSE_PRIOR_SIGMA: f64 = 1.0e-1;
// Sparse detections should not force every candidate landmark to be explained.
const UNMATCHED_LANDMARK_COST: f64 = 0.0;
const UNMATCHED_DETECTION_COST: f64 = 25.0;
// Empty intervals are inserted only to keep graph components connected across
// dropped recording data. They use zero-start-velocity GP priors so stale
// pre-gap velocity is not treated as measured ballistic motion.
const EMPTY_INTERVAL_PROCESS_COVARIANCE_SCALE: f64 = 10.0;
const LONG_GAP_MIN_EMPTY_INTERVALS: u32 = 5;
const POSITIVE_Z_MINIMUM: f64 = 1.0e-3;
const POSITIVE_Z_SOFTNESS: f64 = 1.0e-3;
const POSITIVE_Z_SIGMA: f64 = 0.01;

pub struct BackendConfiguration {
    /// The spacing between control knots on the Gaussian Process
    /// Each control knot represents 9 DoFs for the optimizer.
    pub knot_spacing: Duration,
    /// The maximum optimization window size.
    /// Factors before the optimization window are marginalized.
    pub max_optimization_window: Duration,
    /// Maximum optimizer iterations per solve call.
    /// Slow solve cadences can spend more iterations on each larger batch.
    pub optimizer_max_iterations: usize,

    pub gyroscope_noise: Matrix3<f64>,
    pub accelerometer_noise: Matrix3<f64>,
    pub gyroscope_process_noise: Matrix3<f64>,
    pub accelerometer_process_noise: Matrix3<f64>,
    pub visual_feature_noise: Matrix2<f64>,
    pub visual_odometry_noise: SMatrix<f64, 6, 6>,
    pub gravity: Vector3<f64>,
}

#[derive(Debug, Error)]
pub enum VinsBackendError {
    #[error("frontend disconnected")]
    FrontendDisconnected,
    #[error("failed to ingest IMU measurements")]
    FailedToIngestImu,
    #[error("failed to ingest visual measurements")]
    FailedToIngestVisual,
    #[error("failed to ingest visual odometry measurements")]
    FailedToIngestVisualOdometry,
}

#[derive(Debug, Clone)]
pub struct OptimizationResult {
    /// The timestamp of the most recent variable in the optimized graph.
    pub time: SystemTime,
    /// The latest pose estimate from the optimized graph.
    pub latest_pose: SE23<f64>,
    /// The current optimized camera intrinsics estimate.
    pub camera_intrinsics: crate::camera_intrinsics::CameraIntrinsics<f64>,
}

impl OptimizationResult {
    pub fn position(&self) -> Vector3<f64> {
        self.latest_pose.xyz().into_owned()
    }
}

pub struct VinsBackend {
    /// Channel to retrieve new measurements from the frontend.
    measurement_receiver: UnboundedReceiver<SensorMeasurement>,
    /// Channel to send solver results to the frontend,
    result_sender: watch::Sender<Option<OptimizationResult>>,
    /// Configuration parameters for the solver backend
    config: BackendConfiguration,
    /// Initial state of the graph
    initial_state: InitialState,
    /// Stores the optimizer and the optimization graph
    optimizer: GaussNewton,
    /// Stores the optimized graph values.
    values: Values,
    /// Stores the timestamp of the last knot added to the graph.
    last_knot_time: Option<SystemTime>,
    /// Highest interval whose start/end states and GP prior have been initialized.
    highest_initialized_interval: Option<u32>,
    /// Stores the timestamp of the first measurement received by the backend.
    interval_assigner: IntervalAssigner,
}

pub fn initialize_graph(initial_state: &InitialState) -> (Graph, Values) {
    let mut graph = Graph::default();
    let mut values = Values::default();

    // Initialize camera intrinsics
    let factor = FactorBuilder::new(
        PriorResidual::new(initial_state.camera_intrinsics.clone()),
        CameraIntrinsics(0),
    )
    .noise(GaussianNoise::<4>::from_diag_sigmas(
        INITIAL_CAMERA_INTRINSICS_PRIOR_SIGMA,
        INITIAL_CAMERA_INTRINSICS_PRIOR_SIGMA,
        INITIAL_CAMERA_INTRINSICS_PRIOR_SIGMA,
        INITIAL_CAMERA_INTRINSICS_PRIOR_SIGMA,
    ))
    .build();

    graph.add_factor(factor);
    values.insert(CameraIntrinsics(0), initial_state.camera_intrinsics.clone());

    // Initial orientation
    let initial_pose = initial_state.pose.clone();
    let factor = FactorBuilder::new(PriorResidual::new(initial_pose.clone()), State(0))
        .noise(GaussianNoise::<9>::from_scalar_sigma(
            INITIAL_POSE_PRIOR_SIGMA,
        ))
        .build();
    graph.add_factor(factor);
    values.insert(State(0), initial_pose);
    add_positive_z_factor(&mut graph, State(0));

    (graph, values)
}

impl VinsBackend {
    pub(crate) fn new(
        config: BackendConfiguration,
        initial_state: InitialState,
        measurement_receiver: UnboundedReceiver<SensorMeasurement>,
        result_sender: watch::Sender<Option<OptimizationResult>>,
    ) -> Self {
        assert!(
            config.optimizer_max_iterations > 0,
            "optimizer_max_iterations must be positive"
        );

        let (graph, values) = initialize_graph(&initial_state);

        let mut optimizer = GaussNewton::new(
            BaseOptParams {
                max_iterations: config.optimizer_max_iterations,
                ..Default::default()
            },
            graph,
        );
        optimizer.set_dense_normal_equations(true);

        Self {
            interval_assigner: IntervalAssigner::new(config.knot_spacing),
            measurement_receiver,
            result_sender,
            config,
            initial_state,
            optimizer,
            values,
            last_knot_time: None,
            highest_initialized_interval: None,
        }
    }

    pub fn values(&self) -> &Values {
        &self.values
    }

    /// Loops continuously, ingesting measurements and optimizing the graph.
    /// Only returns if an error occurs.
    pub fn run_loop(mut self) -> Result<(), VinsBackendError> {
        loop {
            let _ = self.solve_once()?;
        }
    }

    pub fn solve_once(&mut self) -> Result<Option<OptimizationResult>, VinsBackendError> {
        self.ingest_until_empty()?;

        let result = self.optimize();
        if self.result_sender.send(result.clone()).is_err() {
            return Err(VinsBackendError::FrontendDisconnected);
        }

        Ok(result)
    }

    /// Ingests a batch of IMU measurements into the graph.
    /// Assumes the measurements are already sorted by time.
    fn ingest_imu(&mut self, imus: Vec<ImuMeasurement>) -> Result<(), VinsBackendError> {
        let Some(last) = imus.last() else {
            return Ok(());
        };
        self.last_knot_time = Some(self.last_knot_time.map_or(last.time, |t| t.max(last.time)));

        let mut interval_groups = Vec::new();
        for (key, chunk) in imus
            .into_iter()
            .chunk_by(|imu| self.interval_assigner.current_interval_start_time(imu.time))
            .into_iter()
        {
            let Some(interval_start_time) = key else {
                return Err(VinsBackendError::FailedToIngestImu);
            };

            let measurements = chunk.collect::<Vec<_>>();

            let Some(interval_start_index) =
                self.interval_assigner.assign_interval(interval_start_time)
            else {
                return Err(VinsBackendError::FailedToIngestImu);
            };

            interval_groups.push((interval_start_index, interval_start_time, measurements));
        }

        for (interval_start_index, interval_start_time, measurements) in interval_groups {
            self.init_intervals_through(interval_start_index);
            if !self.interval_states_available(interval_start_index) {
                log::debug!(
                    "skipping IMU measurements for marginalized interval {interval_start_index}"
                );
                continue;
            }

            let keys = (State(interval_start_index), State(interval_start_index + 1));
            let graph = self.optimizer.graph_mut();
            if let Some(factor) = graph
                .factors_for_residual_mut::<IntervalGaussianProcessImuFactor, _>(keys)
                .next()
            {
                factor
                    .residual_as_mut::<IntervalGaussianProcessImuFactor>()
                    .expect("factor query must return matching residual")
                    .extend_measurements(measurements);
            } else {
                let residual = IntervalGaussianProcessImuFactor::new(
                    measurements,
                    self.config.gyroscope_noise,
                    self.config.accelerometer_noise,
                    self.config.gravity,
                    interval_start_time,
                    interval_start_time + self.config.knot_spacing,
                );
                let factor = FactorBuilder::new(residual, keys).build();

                graph.add_factor(factor);
            }
        }

        Ok(())
    }

    fn ingest_visual(
        &mut self,
        mut visuals: Vec<Vec<VisualMeasurement>>,
    ) -> Result<(), VinsBackendError> {
        visuals.retain(|visual| !visual.is_empty());
        if visuals.is_empty() {
            return Ok(());
        }

        let Some(last) = visuals.last() else {
            return Ok(());
        };

        let last_time = visual_frame_time(last);
        self.last_knot_time = Some(self.last_knot_time.map_or(last_time, |t| t.max(last_time)));

        let mut interval_groups = Vec::new();
        for (key, chunk) in visuals
            .into_iter()
            .chunk_by(|visual| {
                self.interval_assigner
                    .current_interval_start_time(visual_frame_time(visual))
            })
            .into_iter()
        {
            let Some(interval_start_time) = key else {
                return Err(VinsBackendError::FailedToIngestVisual);
            };

            let frames = chunk.collect::<Vec<_>>();

            let Some(interval_start_index) =
                self.interval_assigner.assign_interval(interval_start_time)
            else {
                return Err(VinsBackendError::FailedToIngestVisual);
            };

            interval_groups.push((interval_start_index, interval_start_time, frames));
        }

        for (interval_start_index, interval_start_time, frames) in interval_groups {
            self.init_intervals_through(interval_start_index);
            if !self.interval_states_available(interval_start_index) {
                log::debug!(
                    "skipping visual measurements for marginalized interval {interval_start_index}"
                );
                continue;
            }

            let keys = (
                State(interval_start_index),
                State(interval_start_index + 1),
                CameraIntrinsics(0),
            );
            let graph = self.optimizer.graph_mut();
            if let Some(factor) = graph
                .factors_for_residual_mut::<LandmarkFactor, _>(keys)
                .next()
            {
                factor
                    .residual_as_mut::<LandmarkFactor>()
                    .expect("factor query must return matching residual")
                    .extend_frames(frames);
            } else {
                let residual = LandmarkFactor::new(
                    interval_start_time,
                    interval_start_time + self.config.knot_spacing,
                    frames,
                    self.config.visual_feature_noise,
                )
                .with_unmatched_costs(UNMATCHED_LANDMARK_COST, UNMATCHED_DETECTION_COST);
                let factor = FactorBuilder::new(residual, keys).build();

                graph.add_factor(factor);
            }
        }

        Ok(())
    }

    fn ingest_visual_odometry(
        &mut self,
        visual_odometry: Vec<VisualOdometryMeasurement>,
    ) -> Result<(), VinsBackendError> {
        let Some(last) = visual_odometry.last() else {
            return Ok(());
        };
        self.last_knot_time = Some(
            self.last_knot_time
                .map_or(last.timestamp, |t| t.max(last.timestamp)),
        );

        let mut interval_groups = Vec::new();
        for (key, chunk) in visual_odometry
            .into_iter()
            .chunk_by(|measurement| {
                self.interval_assigner
                    .current_interval_start_time(measurement.timestamp)
            })
            .into_iter()
        {
            let Some(interval_start_time) = key else {
                return Err(VinsBackendError::FailedToIngestVisualOdometry);
            };

            let measurements = chunk.collect::<Vec<_>>();

            let Some(interval_start_index) =
                self.interval_assigner.assign_interval(interval_start_time)
            else {
                return Err(VinsBackendError::FailedToIngestVisualOdometry);
            };

            interval_groups.push((interval_start_index, interval_start_time, measurements));
        }

        for (interval_start_index, interval_start_time, measurements) in interval_groups {
            self.init_intervals_through(interval_start_index);
            if !self.interval_states_available(interval_start_index) {
                log::debug!(
                    "skipping visual odometry measurements for marginalized interval {interval_start_index}"
                );
                continue;
            }

            let keys = (State(interval_start_index), State(interval_start_index + 1));
            let graph = self.optimizer.graph_mut();
            if let Some(factor) = graph
                .factors_for_residual_mut::<VisualOdometryFactor, _>(keys)
                .next()
            {
                factor
                    .residual_as_mut::<VisualOdometryFactor>()
                    .expect("factor query must return matching residual")
                    .extend_measurements(measurements);
            } else {
                let residual = VisualOdometryFactor::new(
                    measurements,
                    self.config.visual_odometry_noise,
                    interval_start_time,
                    interval_start_time + self.config.knot_spacing,
                );
                let factor = FactorBuilder::new(residual, keys).build();

                graph.add_factor(factor);
            }
        }

        Ok(())
    }

    fn init_intervals_through(&mut self, interval_start_index: u32) {
        let first_missing_interval = self
            .highest_initialized_interval
            .map_or(0, |index| index + 1);
        if first_missing_interval > interval_start_index {
            return;
        }

        let is_long_gap = interval_start_index.saturating_sub(first_missing_interval)
            >= LONG_GAP_MIN_EMPTY_INTERVALS;
        if is_long_gap {
            reset_state_velocity(&mut self.values, State(first_missing_interval));
        }
        for index in first_missing_interval..=interval_start_index {
            init_interval_states(
                &mut self.values,
                self.optimizer.graph_mut(),
                &self.config,
                &self.initial_state,
                index,
                is_long_gap && index < interval_start_index,
            );
        }
        self.highest_initialized_interval = Some(interval_start_index);
    }

    fn interval_states_available(&self, interval_start_index: u32) -> bool {
        self.values.get(State(interval_start_index)).is_some()
            && self.values.get(State(interval_start_index + 1)).is_some()
    }

    fn ingest_until_empty(&mut self) -> Result<(), VinsBackendError> {
        let mut new_measurements = IntervalMeasurements::new();
        loop {
            match self.measurement_receiver.try_recv() {
                Ok(SensorMeasurement::Imu(imu)) => new_measurements.push_imu(imu),
                Ok(SensorMeasurement::Visual(visual)) => new_measurements.push_visual(visual),
                Ok(SensorMeasurement::VisualOdometry(visual_odometry)) => {
                    new_measurements.push_visual_odometry(visual_odometry)
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    return Err(VinsBackendError::FrontendDisconnected);
                }
            }
        }

        self.ingest_imu(new_measurements.imu)?;
        self.ingest_visual(new_measurements.visual)?;
        self.ingest_visual_odometry(new_measurements.visual_odometry)?;

        Ok(())
    }

    fn optimize(&mut self) -> Option<OptimizationResult> {
        let time = self.last_knot_time?;
        log::info!("solving graph with {} values", self.values.len());

        // do Schur marginalization
        let cutoff_time = time - self.config.max_optimization_window;
        if let Some(smallest_interval_index_in_window) =
            self.interval_assigner.assign_interval(cutoff_time)
        {
            marginalize(
                &mut self.optimizer,
                &mut self.values,
                State(smallest_interval_index_in_window),
            );
        }

        match self.optimizer.optimize(&mut self.values) {
            Ok(OptStatus::Converged) => {}
            Ok(OptStatus::MaxIterations) => {
                log::warn!("optimizer failed to converge: max iterations reached");
            }
            Err(OptError::FailedToStep) => {
                log::warn!("optimizer failed: failed to step");
            }
            Err(OptError::InvalidSystem) => {
                log::warn!("optimizer failed: invalid system");
            }
        };

        let interval_start_time = self.interval_assigner.current_interval_start_time(time)?;
        let interval_start_index = self
            .interval_assigner
            .assign_interval(interval_start_time)?;
        let start = self.values.get(State(interval_start_index))?.clone();
        let end = self.values.get(State(interval_start_index + 1))?.clone();
        let interval_end_time = interval_start_time + self.config.knot_spacing;
        let latest_pose = SE23Spline::new(start, end, self.config.knot_spacing.as_secs_f64())
            .evaluate(tau(interval_start_time, interval_end_time, time));
        let camera_intrinsics = self.values.get(CameraIntrinsics(0))?.clone();

        Some(OptimizationResult {
            time,
            latest_pose,
            camera_intrinsics,
        })
    }
}

fn visual_frame_time(visual: &[VisualMeasurement]) -> SystemTime {
    visual
        .first()
        .expect("visual frames must contain at least one measurement")
        .time
}

fn reset_state_velocity(values: &mut Values, state: State) {
    if let Some(pose) = values.get_mut(state) {
        *pose = zero_velocity_pose(pose);
    }
}

fn zero_velocity_pose(pose: &SE23) -> SE23 {
    SE23::from_rot_vel_trans(
        pose.rot().clone(),
        Vector3::zeros(),
        pose.xyz().into_owned(),
    )
}

fn init_interval_states(
    values: &mut Values,
    graph: &mut Graph,
    config: &BackendConfiguration,
    initial_state: &InitialState,
    interval_start_index: u32,
    is_empty_bridge_interval: bool,
) {
    let start = State(interval_start_index);
    let end = State(interval_start_index + 1);

    if values.init_if_missing(&initial_state.pose, start, false) {
        add_positive_z_factor(graph, start);
    }
    if values.init_if_missing(&initial_state.pose, end, is_empty_bridge_interval) {
        add_positive_z_factor(graph, end);
        let gp_residual = if is_empty_bridge_interval {
            GaussianProcessPriorFactor::new_zero_start_velocity_bridge(
                config.knot_spacing.as_secs_f64(),
                &config.gyroscope_process_noise,
                &config.accelerometer_process_noise,
                EMPTY_INTERVAL_PROCESS_COVARIANCE_SCALE,
            )
        } else {
            GaussianProcessPriorFactor::new(
                config.knot_spacing.as_secs_f64(),
                &config.gyroscope_process_noise,
                &config.accelerometer_process_noise,
            )
        };

        let gp_factor = FactorBuilder::new(gp_residual, (start, end)).build();

        graph.add_factor(gp_factor);
    }
}

fn add_positive_z_factor(graph: &mut Graph, state: State) {
    let factor = FactorBuilder::new(
        PositiveZFactor::new(POSITIVE_Z_MINIMUM, POSITIVE_Z_SOFTNESS, POSITIVE_Z_SIGMA),
        state,
    )
    .build();
    graph.add_factor(factor);
}

#[derive(Debug, Clone)]
struct IntervalAssigner {
    first_observed_time: OnceCell<SystemTime>,
    interval_length: Duration,
}

impl IntervalAssigner {
    pub fn new(interval_length: Duration) -> Self {
        Self {
            first_observed_time: OnceCell::new(),
            interval_length,
        }
    }

    pub fn assign_interval(&self, measurement_time: SystemTime) -> Option<u32> {
        let start_time = self.first_observed_time.get_or_init(|| measurement_time);
        let time_since_start = measurement_time.duration_since(*start_time).ok()?;
        Some(self.index_of_duration(time_since_start))
    }

    pub fn current_interval_start_time(&self, measurement_time: SystemTime) -> Option<SystemTime> {
        let index = self.assign_interval(measurement_time)?;
        self.interval_start_time(index)
    }

    pub fn interval_start_time(&self, index: u32) -> Option<SystemTime> {
        let start_time = *self.first_observed_time.get()?;
        Some(start_time + self.interval_length * index)
    }

    pub fn index_of_duration(&self, duration: Duration) -> u32 {
        let index = duration.as_nanos() / self.interval_length.as_nanos();
        index.try_into().expect("does not fit interval index")
    }
}

trait InitStateExt {
    fn init_if_missing(&mut self, initial: &SE23, index: State, reset_velocity: bool) -> bool;
}

impl InitStateExt for Values {
    fn init_if_missing(&mut self, initial: &SE23, index: State, reset_velocity: bool) -> bool {
        if self.get(index).is_some() {
            return false;
        }

        let previous = index
            .0
            .checked_sub(1)
            .and_then(|i| self.get(State(i)))
            .unwrap_or(initial);

        self.insert(
            index,
            if reset_velocity {
                zero_velocity_pose(previous)
            } else {
                previous.clone()
            },
        );
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use booster::ImuState;
    use factrs::core::{SE3, SO3};
    use factrs::traits::Variable;
    use linear_algebra::IntoFramed;

    fn backend_configuration() -> BackendConfiguration {
        BackendConfiguration {
            knot_spacing: Duration::from_millis(200),
            max_optimization_window: Duration::from_secs(3),
            optimizer_max_iterations: 1,
            gyroscope_noise: Matrix3::identity() * 0.01,
            accelerometer_noise: Matrix3::identity() * 0.05,
            gyroscope_process_noise: Matrix3::identity() * 0.01,
            accelerometer_process_noise: Matrix3::identity() * 0.01,
            visual_feature_noise: Matrix2::identity() * 5.0,
            visual_odometry_noise: SMatrix::<f64, 6, 6>::identity() * 0.05,
            gravity: Vector3::new(0.0, 0.0, 9.81),
        }
    }

    fn assigner(interval: Duration) -> IntervalAssigner {
        let assigner = IntervalAssigner::new(interval);
        assigner
            .first_observed_time
            .set(SystemTime::UNIX_EPOCH)
            .expect("could not set start time");
        assigner
    }

    fn stationary_imu(time: SystemTime) -> SensorMeasurement {
        SensorMeasurement::Imu(ImuMeasurement {
            time,
            state: ImuState {
                roll_pitch_yaw: Vector3::zeros().framed(),
                angular_velocity: Vector3::zeros().framed(),
                linear_acceleration: Vector3::new(0.0, 0.0, 9.81).framed(),
            },
        })
    }

    fn visual_odometry(time: SystemTime, x: f64) -> SensorMeasurement {
        SensorMeasurement::VisualOdometry(VisualOdometryMeasurement {
            robot_to_left_camera: SE3::identity(),
            odometer: SE3::from_rot_trans(SO3::identity(), Vector3::new(x, 0.0, 0.0)),
            timestamp: time,
        })
    }

    fn moving_initial_state(velocity: Vector3<f64>) -> InitialState {
        InitialState {
            pose: SE23::from_rot_vel_trans(SO3::identity(), velocity, Vector3::zeros()),
            ..InitialState::default()
        }
    }

    fn initial_state_with_intrinsics(
        focal_lengths: nalgebra::Vector2<f64>,
        optical_center: nalgebra::Vector2<f64>,
    ) -> InitialState {
        InitialState {
            camera_intrinsics: crate::camera_intrinsics::CameraIntrinsics::new(
                focal_lengths,
                optical_center,
            ),
            ..InitialState::default()
        }
    }

    #[test]
    fn solve_once_before_measurements_preserves_initial_values() {
        let (_measurement_sender, measurement_receiver) = tokio::sync::mpsc::unbounded_channel();
        let (result_sender, _result_receiver) = tokio::sync::watch::channel(None);
        let mut backend = VinsBackend::new(
            backend_configuration(),
            InitialState::default(),
            measurement_receiver,
            result_sender,
        );

        let _ = backend.solve_once().expect("empty solve should succeed");
        let _ = backend
            .solve_once()
            .expect("repeated empty solve should succeed");

        assert!(backend.values().get_raw(CameraIntrinsics(0)).is_some());
        assert!(backend.values().get_raw(State(0)).is_some());
    }

    #[test]
    fn solve_once_result_includes_camera_intrinsics() {
        let (measurement_sender, measurement_receiver) = tokio::sync::mpsc::unbounded_channel();
        let (result_sender, _result_receiver) = tokio::sync::watch::channel(None);
        let initial_state = initial_state_with_intrinsics(
            nalgebra::vector![200.0, 210.0],
            nalgebra::vector![250.0, 240.0],
        );
        let mut backend = VinsBackend::new(
            backend_configuration(),
            initial_state,
            measurement_receiver,
            result_sender,
        );

        measurement_sender
            .send(stationary_imu(SystemTime::UNIX_EPOCH))
            .expect("IMU should send");

        let result = backend
            .solve_once()
            .expect("solve should succeed")
            .expect("result should be available");

        assert_eq!(
            result.camera_intrinsics.focals(),
            nalgebra::vector![200.0, 210.0]
        );
        assert_eq!(
            result.camera_intrinsics.optical_center(),
            nalgebra::vector![250.0, 240.0]
        );
    }

    #[test]
    fn visual_odometry_measurements_create_interval_factor() {
        let (measurement_sender, measurement_receiver) = tokio::sync::mpsc::unbounded_channel();
        let (result_sender, _result_receiver) = tokio::sync::watch::channel(None);
        let mut backend = VinsBackend::new(
            backend_configuration(),
            InitialState::default(),
            measurement_receiver,
            result_sender,
        );
        let start = SystemTime::UNIX_EPOCH;

        measurement_sender
            .send(visual_odometry(start, 0.0))
            .expect("first visual odometry should send");
        measurement_sender
            .send(visual_odometry(start + Duration::from_millis(100), 0.1))
            .expect("second visual odometry should send");

        let _ = backend.solve_once().expect("solve should succeed");

        assert!(backend.values().get_raw(State(0)).is_some());
        assert!(backend.values().get_raw(State(1)).is_some());
        assert_eq!(
            backend
                .optimizer
                .graph_mut()
                .factors_for_residual::<VisualOdometryFactor, _>((State(0), State(1)))
                .count(),
            1
        );
    }

    #[test]
    fn imu_gaps_are_bridged_with_empty_intervals() {
        let (measurement_sender, measurement_receiver) = tokio::sync::mpsc::unbounded_channel();
        let (result_sender, _result_receiver) = tokio::sync::watch::channel(None);
        let mut backend = VinsBackend::new(
            backend_configuration(),
            InitialState::default(),
            measurement_receiver,
            result_sender,
        );

        let start = SystemTime::UNIX_EPOCH;
        measurement_sender
            .send(stationary_imu(start))
            .expect("first IMU should send");
        measurement_sender
            .send(stationary_imu(start + Duration::from_secs(1)))
            .expect("second IMU should send");

        let _ = backend.solve_once().expect("solve should succeed");

        for index in 0..=6 {
            assert!(
                backend.values().get_raw(State(index)).is_some(),
                "state {index} should be initialized"
            );
        }
        assert_eq!(backend.highest_initialized_interval, Some(5));
    }

    #[test]
    fn long_gap_bridge_states_do_not_inherit_stale_velocity() {
        let (_measurement_sender, measurement_receiver) = tokio::sync::mpsc::unbounded_channel();
        let (result_sender, _result_receiver) = tokio::sync::watch::channel(None);
        let mut backend = VinsBackend::new(
            backend_configuration(),
            moving_initial_state(Vector3::new(0.0, 3.0, 0.0)),
            measurement_receiver,
            result_sender,
        );

        backend
            .interval_assigner
            .assign_interval(SystemTime::UNIX_EPOCH)
            .expect("first interval should be assigned");

        backend.init_intervals_through(0);
        backend.init_intervals_through(LONG_GAP_MIN_EMPTY_INTERVALS + 1);

        let boundary_state = backend
            .values()
            .get(State(1))
            .expect("gap boundary state should be initialized");
        let bridge_state = backend
            .values()
            .get(State(2))
            .expect("bridge state should be initialized");
        let target_state = backend
            .values()
            .get(State(LONG_GAP_MIN_EMPTY_INTERVALS + 1))
            .expect("target state should be initialized");

        assert!(boundary_state.uvw().norm() < 1.0e-9);
        assert!(bridge_state.uvw().norm() < 1.0e-9);
        assert!(target_state.uvw().norm() < 1.0e-9);
    }

    #[test]
    fn late_measurements_for_marginalized_intervals_are_skipped() {
        let (measurement_sender, measurement_receiver) = tokio::sync::mpsc::unbounded_channel();
        let (result_sender, _result_receiver) = tokio::sync::watch::channel(None);
        let mut config = backend_configuration();
        config.max_optimization_window = Duration::from_millis(400);
        let mut backend = VinsBackend::new(
            config,
            InitialState::default(),
            measurement_receiver,
            result_sender,
        );

        let start = SystemTime::UNIX_EPOCH;
        measurement_sender
            .send(stationary_imu(start))
            .expect("first IMU should send");
        measurement_sender
            .send(stationary_imu(start + Duration::from_secs(2)))
            .expect("later IMU should send");
        let _ = backend.solve_once().expect("solve should succeed");
        assert!(
            backend.values().get_raw(State(0)).is_none(),
            "state 0 should have been marginalized"
        );

        measurement_sender
            .send(stationary_imu(start + Duration::from_millis(100)))
            .expect("late IMU should send");
        let _ = backend
            .solve_once()
            .expect("late marginalized measurement should be skipped");

        assert!(
            backend.values().get_raw(State(0)).is_none(),
            "late measurement must not reintroduce marginalized state 0"
        );
    }

    #[test]
    fn test_get_previous_interval_start_time() {
        let interval = Duration::from_millis(200);
        let assigner = assigner(interval);

        // 1. Middle of an interval
        // 265ms since epoch should snap back to 200ms
        let t1 = SystemTime::UNIX_EPOCH + Duration::from_millis(265);
        let expected1 = SystemTime::UNIX_EPOCH + interval;
        assert_eq!(assigner.current_interval_start_time(t1), Some(expected1));

        // 2. Exact boundary
        // 200ms since epoch should stay at 200ms
        let t2 = SystemTime::UNIX_EPOCH + Duration::from_millis(200);
        let expected2 = SystemTime::UNIX_EPOCH + interval;
        assert_eq!(assigner.current_interval_start_time(t2), Some(expected2));

        // 3. Just before a boundary
        // 399ms since epoch should snap back to 200ms
        let t3 = SystemTime::UNIX_EPOCH + Duration::from_millis(399);
        let expected3 = SystemTime::UNIX_EPOCH + interval;
        assert_eq!(assigner.current_interval_start_time(t3), Some(expected3));

        // 4. Very early time
        // 50ms since epoch with 100ms interval should snap to 0 (Unix Epoch)
        let t4 = SystemTime::UNIX_EPOCH + Duration::from_millis(50);
        let expected4 = SystemTime::UNIX_EPOCH;
        assert_eq!(assigner.current_interval_start_time(t4), Some(expected4));
    }

    #[test]
    fn test_large_intervals() {
        let interval = Duration::from_secs(1);
        let assigner = assigner(interval);

        // 10.9 seconds -> 10.0 seconds
        let t = SystemTime::UNIX_EPOCH + Duration::from_millis(10900);
        let expected = SystemTime::UNIX_EPOCH + Duration::from_secs(10);
        assert_eq!(assigner.current_interval_start_time(t), Some(expected));
    }
}
