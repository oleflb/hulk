use clap::Parser;
use color_eyre::{
    Result,
    eyre::{Context as _, eyre},
};
use eframe::{NativeOptions, run_native};
use tokio::runtime::Runtime;

mod app;
mod apriltag;
mod args;
mod calibration;
mod image;
mod ros;
mod ui;

use app::CalibrationApp;
use apriltag::create_apriltag_detector;
use args::Args;
use ros::{RosState, create_ros_resources};

fn main() -> Result<()> {
    color_eyre::install()?;

    let args = Args::parse();
    let runtime = Runtime::new().wrap_err("failed to create Tokio runtime")?;
    let resources = runtime
        .block_on(create_ros_resources(args))
        .wrap_err("failed to initialize ros-z subscriptions")?;
    let ros = RosState::new(resources, runtime);
    let detector = create_apriltag_detector()?;

    run_native(
        "Intrinsic Calibration",
        NativeOptions::default(),
        Box::new(move |creation_context| {
            Ok(Box::new(CalibrationApp::new(
                creation_context,
                ros,
                detector,
            )))
        }),
    )
    .map_err(|error| eyre!("eframe failed: {error}"))?;

    Ok(())
}
