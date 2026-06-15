use std::{path::PathBuf, sync::Arc};

use clap::Parser;
use color_eyre::{Result, eyre::eyre};
use eframe::{
    NativeOptions, Renderer,
    egui_wgpu::{WgpuConfiguration, WgpuSetup},
    run_native,
};
use tracing_subscriber::EnvFilter;

use crate::app::LocalizationMcapVisualizerApp;

mod app;
mod mcap_recording;
mod replay;
mod scene;

#[derive(Clone, Debug, Parser)]
struct Arguments {
    /// Localization recorder MCAP file to inspect.
    #[arg(default_value = "recording.mcap")]
    mcap: PathBuf,
}

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let arguments = Arguments::parse();
    let recording = Arc::new(mcap_recording::Recording::load(&arguments.mcap)?);

    run_native(
        "Localization MCAP Visualizer",
        NativeOptions {
            renderer: Renderer::Wgpu,
            wgpu_options: wgpu_options(),
            ..Default::default()
        },
        Box::new(move |creation_context| {
            let app = LocalizationMcapVisualizerApp::new(
                creation_context,
                arguments.mcap.clone(),
                recording.clone(),
            )?;
            Ok(Box::new(app))
        }),
    )
    .map_err(|error| eyre!("failed to run localization MCAP visualizer: {error}"))?;

    Ok(())
}

fn wgpu_options() -> WgpuConfiguration {
    let mut options = WgpuConfiguration::default();
    if let WgpuSetup::CreateNew(setup) = &mut options.wgpu_setup {
        let previous = setup.device_descriptor.clone();
        setup.device_descriptor = Arc::new(move |adapter| {
            let mut descriptor = previous(adapter);
            descriptor
                .required_limits
                .max_storage_buffers_per_shader_stage = 9;
            descriptor
        });
    }
    options
}

fn nearest_by_distance<T, Distance>(
    previous: Option<(T, Distance)>,
    next: Option<(T, Distance)>,
) -> Option<T>
where
    Distance: PartialOrd,
{
    match (previous, next) {
        (Some((previous, previous_distance)), Some((next, next_distance))) => {
            Some(if previous_distance <= next_distance {
                previous
            } else {
                next
            })
        }
        (Some((previous, _)), None) => Some(previous),
        (None, Some((next, _))) => Some(next),
        (None, None) => None,
    }
}
