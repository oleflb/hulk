pub mod ai_assistant;
pub mod annotation;
pub mod annotator_app;
pub mod boundingbox;
pub mod classes;
pub mod inputs;
pub mod label_document;
pub mod label_widget;
pub mod paths;
pub mod theme;
pub mod user_toml;
pub mod utils;
pub mod widgets;
pub mod workflow;

use std::path::PathBuf;

use annotator_app::AnnotatorApp;
use clap::Parser;
use color_eyre::eyre::Result;
use eframe::{NativeOptions, egui::ViewportBuilder, run_native};
use theme::{MOCHA, apply_theme};

use crate::{
    annotator_app::eframe_error_to_report, inputs::collect_image_paths, user_toml::CONFIG,
};

#[derive(Parser, Debug)]
#[command(name = "annotato")]
pub struct Args {
    /// Image files, folders, or glob patterns to annotate
    #[arg(required = true, value_name = "INPUT")]
    inputs: Vec<String>,

    /// Optional predictions JSON keyed by image file name
    #[arg(long)]
    predictions: Option<PathBuf>,

    /// Override config path. Defaults to $XDG_CONFIG_HOME/annotato/config.toml or ~/.config/annotato/config.toml
    #[arg(long)]
    config: Option<PathBuf>,
}

fn start_labelling_ui(
    image_paths: Vec<PathBuf>,
    predictions: Option<PathBuf>,
) -> eframe::Result<()> {
    let native_options = NativeOptions {
        viewport: ViewportBuilder {
            title: Some("annotato".to_string()),
            maximized: Some(true),
            ..Default::default()
        },
        ..Default::default()
    };

    run_native(
        "annotato",
        native_options,
        Box::new(move |cc| {
            egui_extras::install_image_loaders(&cc.egui_ctx);

            let context = &cc.egui_ctx;
            apply_theme(context, MOCHA);

            let app = AnnotatorApp::try_new(cc, image_paths, predictions)?;

            Ok(Box::new(app))
        }),
    )
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let arguments = Args::parse();

    CONFIG
        .set(user_toml::load_config(arguments.config.as_deref())?)
        .expect("once_cell::set failed");

    let image_paths = collect_image_paths(&arguments.inputs)?;
    start_labelling_ui(image_paths, arguments.predictions).map_err(eframe_error_to_report)?;

    Ok(())
}
