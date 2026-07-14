use clap::Parser;
use color_eyre::{
    Result,
    eyre::{Context as _, eyre},
};
use eframe::{NativeOptions, run_native};
use tokio::runtime::Runtime;

mod app;
mod args;
mod calibration;
mod image;
mod marker;
mod ros;
mod ui;

use app::CalibrationApp;
use args::Args;
use marker::{MarkerType, create_marker_detector};
use ros::{RosState, create_ros_resources};

fn main() -> Result<()> {
    color_eyre::install()?;

    let args = Args::parse();
    let runtime = Runtime::new().wrap_err("failed to create Tokio runtime")?;
    let resources = runtime
        .block_on(create_ros_resources(args))
        .wrap_err("failed to initialize ros-z subscriptions")?;
    let ros = RosState::new(resources, runtime);
    let marker_type = MarkerType::default();
    let detector = create_marker_detector(marker_type)?;

    run_native(
        "Intrinsic Calibration",
        NativeOptions::default(),
        Box::new(move |creation_context| {
            Ok(Box::new(CalibrationApp::new(
                creation_context,
                ros,
                detector,
                marker_type,
            )))
        }),
    )
    .map_err(|error| eyre!("eframe failed: {error}"))?;

    Ok(())
}
