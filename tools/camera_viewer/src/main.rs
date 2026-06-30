mod app;
mod camera;
mod cli;
mod decoder;
mod fps;
mod frame;
mod state;
mod worker;

use clap::Parser;
use color_eyre::{Result, eyre::eyre};
use eframe::{NativeOptions, run_native};
use tracing_subscriber::EnvFilter;

use crate::{app::CameraViewerApp, cli::Args};

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let args = Args::parse();
    run_native(
        "Camera Viewer",
        NativeOptions::default(),
        Box::new(move |_creation_context| Ok(Box::new(CameraViewerApp::new(args.clone())))),
    )
    .map_err(|error| eyre!("eframe failed: {error}"))?;

    Ok(())
}
