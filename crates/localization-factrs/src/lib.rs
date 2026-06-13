pub use backend::{BackendConfiguration, VinsBackend, VinsBackendError};
pub use camera_intrinsics::CameraIntrinsics;
pub use frontend::{
    BackendCorrectionConfiguration, FrontendConfiguration, OptimizationResult, VinsFrontend,
    VinsFrontendError,
};
pub use initial_state::InitialState;
pub use measurements::{LandmarkAssociationCosts, VisualClassMeasurement};
pub use splines::{SE23Kinematics, SE23Spline};
pub use symbols::State;
pub use utils::{interval_dt, tau};

pub mod backend;
mod camera_intrinsics;
mod frontend;
pub mod gaussian_process_prior_factor;
mod imu_factor;
mod initial_state;
mod interval_measurement;
mod landmark_factor;
mod measurements;
mod positive_z_factor;
mod prior_factor;
mod schur_marginalization;
mod splines;
mod symbols;
mod utils;

pub fn initialize(
    config: BackendConfiguration,
    initial_state: InitialState,
) -> (VinsFrontend, VinsBackend) {
    initialize_with_frontend_config(config, FrontendConfiguration::default(), initial_state)
}

pub fn initialize_with_frontend_config(
    config: BackendConfiguration,
    frontend_config: FrontendConfiguration,
    initial_state: InitialState,
) -> (VinsFrontend, VinsBackend) {
    let (measurement_sender, measurement_receiver) = tokio::sync::mpsc::unbounded_channel();
    let (result_sender, result_receiver) = tokio::sync::watch::channel(None);
    let gravity = config.gravity;

    let frontend = VinsFrontend::with_config(
        measurement_sender,
        result_receiver,
        gravity,
        frontend_config,
    );
    let backend = VinsBackend::new(config, initial_state, measurement_receiver, result_sender);
    (frontend, backend)
}
