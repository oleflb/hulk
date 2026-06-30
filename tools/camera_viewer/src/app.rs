use std::{sync::Arc, time::Duration};

use eframe::{
    App, Frame,
    egui::{
        CentralPanel, Color32, ColorImage, Context as EguiContext, Grid, Image, RichText,
        TextureHandle, TextureOptions, TopBottomPanel, Ui, Vec2, load::SizedTexture,
    },
};

use crate::{
    cli::Args,
    fps::FpsMeter,
    frame::{ViewerFrame, expected_rgb_len},
    state::{CameraSnapshot, SharedCameraState},
    worker::{WorkerConfig, WorkerHandle},
};

const REPAINT_INTERVAL: Duration = Duration::from_millis(16);

pub(crate) struct CameraViewerApp {
    args: Args,
    views: Vec<CameraView>,
    _worker: WorkerHandle,
}

impl CameraViewerApp {
    pub(crate) fn new(args: Args) -> Self {
        let states = args
            .camera
            .sides()
            .iter()
            .copied()
            .map(|side| Arc::new(SharedCameraState::new(side)))
            .collect::<Vec<_>>();
        let worker = WorkerHandle::spawn(worker_config_from_args(&args), states.clone());
        let views = states.into_iter().map(CameraView::new).collect::<Vec<_>>();

        Self {
            args,
            views,
            _worker: worker,
        }
    }
}

impl App for CameraViewerApp {
    fn update(&mut self, context: &EguiContext, _frame: &mut Frame) {
        TopBottomPanel::top("connection").show(context, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new("camera_viewer").strong());
                ui.separator();
                ui.label(format!("router: {}", self.args.router));
                ui.separator();
                ui.label(format!("namespace: {}", self.args.namespace));
                ui.separator();
                ui.label(format!("camera: {}", self.args.camera));
                ui.separator();
                ui.label(format!("ffmpeg: {}", self.args.ffmpeg_path.display()));
            });
        });

        CentralPanel::default().show(context, |ui| {
            if self.views.len() == 1 {
                self.views[0].show(ui);
            } else {
                ui.columns(self.views.len(), |columns| {
                    for (view, ui) in self.views.iter_mut().zip(columns.iter_mut()) {
                        view.show(ui);
                    }
                });
            }
        });

        context.request_repaint_after(REPAINT_INTERVAL);
    }
}

fn worker_config_from_args(args: &Args) -> WorkerConfig {
    WorkerConfig {
        router: args.router.clone(),
        namespace: args.namespace.clone(),
        ffmpeg_path: args.ffmpeg_path.clone(),
        frame_timeout: Duration::from_millis(args.frame_timeout_ms),
    }
}

struct CameraView {
    state: Arc<SharedCameraState>,
    texture: Option<TextureHandle>,
    texture_dimensions: Option<(u32, u32)>,
    texture_error: Option<String>,
    display_fps: FpsMeter,
    displayed_frames: u64,
}

impl CameraView {
    fn new(state: Arc<SharedCameraState>) -> Self {
        Self {
            state,
            texture: None,
            texture_dimensions: None,
            texture_error: None,
            display_fps: FpsMeter::new(),
            displayed_frames: 0,
        }
    }

    fn show(&mut self, ui: &mut Ui) {
        let (latest_frame, snapshot) = self.state.take_latest_and_snapshot();
        if let Some(frame) = latest_frame {
            self.upload_frame(ui.ctx(), frame);
        }

        ui.vertical(|ui| {
            ui.heading(format!("{} Camera", self.state.side().label()));
            self.show_stats(ui, &snapshot);
            ui.separator();
            self.show_image(ui);
        });
    }

    fn show_stats(&mut self, ui: &mut Ui, snapshot: &CameraSnapshot) {
        let display_fps = self.display_fps.rate(std::time::Instant::now());

        if let Some(error) = &snapshot.error {
            ui.colored_label(Color32::RED, error);
        }
        if let Some(error) = &self.texture_error {
            ui.colored_label(Color32::RED, error);
        }

        Grid::new(format!("{}-stats", self.state.side().topic()))
            .num_columns(2)
            .striped(true)
            .show(ui, |ui| {
                stat_row(ui, "topic", self.state.side().topic());
                stat_row(ui, "status", &snapshot.status);
                stat_row(ui, "publishers", snapshot.publisher_count.to_string());
                stat_row(
                    ui,
                    "received",
                    format!(
                        "{} frames ({:.1} fps)",
                        snapshot.received_frames, snapshot.received_fps
                    ),
                );
                stat_row(
                    ui,
                    "displayed",
                    format!("{} frames ({display_fps:.1} fps)", self.displayed_frames),
                );
                stat_row(
                    ui,
                    "decoded",
                    format!(
                        "{} frames ({:.1} fps)",
                        snapshot.decoded_frames, snapshot.decoded_fps
                    ),
                );
                stat_row(
                    ui,
                    "decoder restarts",
                    snapshot.decoder_restarts.to_string(),
                );
                stat_row(
                    ui,
                    "pre-IRAP drops",
                    snapshot.dropped_before_sync.to_string(),
                );
                stat_row(
                    ui,
                    "frame id",
                    snapshot
                        .last_frame_identifier
                        .map(|identifier| identifier.to_string())
                        .unwrap_or_else(|| "-".to_string()),
                );
                stat_row(
                    ui,
                    "timestamp ns",
                    snapshot
                        .last_timestamp_ns
                        .map(|timestamp| timestamp.to_string())
                        .unwrap_or_else(|| "-".to_string()),
                );
                stat_row(
                    ui,
                    "pts us",
                    snapshot
                        .last_presentation_timestamp_us
                        .map(|timestamp| timestamp.to_string())
                        .unwrap_or_else(|| "-".to_string()),
                );
                stat_row(
                    ui,
                    "dimensions",
                    snapshot
                        .last_dimensions
                        .map(|(width, height)| format!("{width}x{height}"))
                        .unwrap_or_else(|| "-".to_string()),
                );
            });
    }

    fn upload_frame(&mut self, context: &EguiContext, frame: ViewerFrame) {
        let Some(expected_len) = expected_rgb_len(frame.width, frame.height) else {
            self.texture_error = Some(format!(
                "RGB frame dimensions overflow: {}x{}",
                frame.width, frame.height
            ));
            return;
        };
        if frame.rgb.len() != expected_len {
            self.texture_error = Some(format!(
                "RGB frame size mismatch: got {} bytes, expected {expected_len}",
                frame.rgb.len()
            ));
            return;
        }

        let image = ColorImage::from_rgb(
            [frame.width as usize, frame.height as usize],
            frame.rgb.as_slice(),
        );
        if let Some(texture) = self.texture.as_mut() {
            texture.set(image, TextureOptions::NEAREST);
        } else {
            self.texture = Some(context.load_texture(
                self.state.side().texture_name(),
                image,
                TextureOptions::NEAREST,
            ));
        }

        self.texture_dimensions = Some((frame.width, frame.height));
        self.texture_error = None;
        self.displayed_frames += 1;
        self.display_fps.tick(std::time::Instant::now());
    }

    fn show_image(&self, ui: &mut Ui) {
        let (Some(texture), Some((width, height))) = (&self.texture, self.texture_dimensions)
        else {
            ui.centered_and_justified(|ui| {
                ui.label("waiting for decoded frame");
            });
            return;
        };

        let original_size = Vec2::new(width as f32, height as f32);
        let available_size = ui.available_size();
        let scale = (available_size.x / original_size.x)
            .min(available_size.y / original_size.y)
            .max(0.01);
        let display_size = original_size * scale;
        let sized_texture = SizedTexture {
            id: texture.id(),
            size: original_size,
        };
        ui.add(Image::new(sized_texture).fit_to_exact_size(display_size));
    }
}

fn stat_row(ui: &mut Ui, label: &str, value: impl Into<String>) {
    ui.label(RichText::new(label).color(Color32::GRAY));
    ui.label(value.into());
    ui.end_row();
}
