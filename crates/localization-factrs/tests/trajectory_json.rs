use std::error::Error;

mod trajectory_support;

use trajectory_support::{
    InitialStateConfig, MeasurementSelection, TrajectoryTestConfig, run_trajectory_test,
};

const TEST_CONFIG: TrajectoryTestConfig = TrajectoryTestConfig {
    initial_state: InitialStateConfig::FromSimulationStartPose,
    measurement_selection: MeasurementSelection::All,
    visual_outliers: None,
    solve_every_nth_frame: 1,
    optimizer_max_iterations: 1,
    output_path: "/tmp/graph_trajectory.json",
    gyro_noise_std: 0.1,
    accel_noise_std: 0.1,
    detection_noise_std: 1.0,
    noise_seed: 0,
    max_position_rmse_meters: 0.079566,
    max_orientation_rmse_degrees: 0.808693,
};

#[test]
fn trajectory_json_position_rmse_is_bounded() -> Result<(), Box<dyn Error>> {
    run_trajectory_test(TEST_CONFIG)
}
