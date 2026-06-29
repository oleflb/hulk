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

fn resolve_inputs_and_predictions(
    inputs: Vec<String>,
    predictions: Option<PathBuf>,
) -> (Vec<String>, Option<PathBuf>) {
    if predictions.is_none() && inputs.len() == 1 {
        let input_path = PathBuf::from(&inputs[0]);
        let prelabels_path = input_path.join("prelabeled-data.json");
        if input_path.is_dir() && prelabels_path.is_file() {
            return (inputs, Some(prelabels_path));
        }
    }

    (inputs, predictions)
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let arguments = Args::parse();

    CONFIG
        .set(user_toml::load_config(arguments.config.as_deref())?)
        .expect("once_cell::set failed");

    let (inputs, predictions) =
        resolve_inputs_and_predictions(arguments.inputs, arguments.predictions);
    let image_paths = collect_image_paths(&inputs)?;
    start_labelling_ui(image_paths, predictions).map_err(eframe_error_to_report)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use std::{
        fs::{self, File},
        path::PathBuf,
        time::{SystemTime, UNIX_EPOCH},
    };

    use super::*;

    fn temp_dir() -> PathBuf {
        let suffix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!("annotato-main-{suffix}"));
        fs::create_dir(&path).unwrap();
        path
    }

    #[test]
    fn image_folder_uses_prelabeled_data_json_when_present() {
        let chunk_path = temp_dir();
        File::create(chunk_path.join("frame.webp")).unwrap();
        File::create(chunk_path.join("prelabeled-data.json")).unwrap();

        let (inputs, predictions) = resolve_inputs_and_predictions(
            vec![chunk_path.display().to_string()],
            None,
        );

        assert_eq!(inputs, vec![chunk_path.display().to_string()]);
        assert_eq!(predictions, Some(chunk_path.join("prelabeled-data.json")));

        fs::remove_dir_all(chunk_path).unwrap();
    }
}
