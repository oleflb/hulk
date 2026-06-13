use std::error::Error;

mod trajectory_support;

use trajectory_support::{
    InitialStateConfig, MeasurementSelection, TrajectoryTestConfig, VisualOutlierConfig,
    run_trajectory_test,
};

const TEST_CONFIG: TrajectoryTestConfig = TrajectoryTestConfig {
    initial_state: InitialStateConfig::FromSimulationStartPose,
    measurement_selection: MeasurementSelection::All,
    visual_outliers: Some(VisualOutlierConfig {
        false_detection_probability: 0.10,
        real_detection_dropout_probability: 0.20,
        seed: 1,
    }),
    solve_every_nth_frame: 1,
    optimizer_max_iterations: 1,
    output_path: "/tmp/graph_trajectory_visual_outliers.json",
    gyro_noise_std: 0.1,
    accel_noise_std: 0.1,
    detection_noise_std: 1.0,
    noise_seed: 0,
    max_position_rmse_meters: 1.6,
    max_orientation_rmse_degrees: 10.0,
};

#[test]
fn trajectory_json_visual_outliers_position_rmse_is_bounded() -> Result<(), Box<dyn Error>> {
    run_trajectory_test(TEST_CONFIG)
}
