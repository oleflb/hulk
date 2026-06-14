use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver},
    },
    time::Instant,
};

use coordinate_systems::{Field, Robot};
use eframe::{
    App, CreationContext, Frame,
    egui::{
        self, CentralPanel, Color32, ColorImage, Context, DragValue, FontId, Rect, RichText, Sense,
        SidePanel, Slider, Stroke, StrokeKind, TextureHandle, TextureOptions, TopBottomPanel, Ui,
        Vec2, Widget, pos2, vec2,
    },
};
use egui_bevy::BevyWidget;
use egui_plot::{Line, Plot, PlotPoints};
use linear_algebra::IntoTransform;
use localization_3d::{
    GlobalLocalizationDetailedDebug, VisualFeatureClass, find_detected_visual_features,
    localize_global_visual_features_detailed_debug,
};
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
};

use crate::{
    mcap_recording::{CameraImage, Recording, StereoFrame, TrajectoryPoint},
    replay::{ReplayParameters, ResolveMessage, ResolveProgress, ResolveResult, TimestampMode},
    scene::{self, SceneCameraFrame, SceneData},
};

pub struct LocalizationMcapVisualizerApp {
    recording: Arc<Recording>,
    mcap_path: PathBuf,
    widget: BevyWidget,
    position_seconds: f64,
    playing: bool,
    playback_rate: f64,
    last_frame_time: Instant,
    parameters: ReplayParameters,
    recorded_trajectory: Vec<TrajectoryPoint>,
    selected_camera: StereoSide,
    image_cache: Option<CachedStereoFrame>,
    left_texture: Option<TextureHandle>,
    right_texture: Option<TextureHandle>,
    resolve: ResolveState,
}

impl LocalizationMcapVisualizerApp {
    pub fn new(
        creation_context: &CreationContext,
        mcap_path: PathBuf,
        recording: Arc<Recording>,
    ) -> Self {
        creation_context.egui_ctx.set_visuals(egui::Visuals::dark());

        let mut widget = BevyWidget::new(
            creation_context
                .wgpu_render_state
                .clone()
                .expect("no wgpu render state found"),
        );
        scene::configure(&mut widget.bevy_app);
        widget.bevy_app.finish();
        widget.bevy_app.cleanup();

        Self {
            recorded_trajectory: recording.recorded_localization_trajectory(),
            recording,
            mcap_path,
            widget,
            position_seconds: 0.0,
            playing: false,
            playback_rate: 1.0,
            last_frame_time: Instant::now(),
            parameters: ReplayParameters::default(),
            selected_camera: StereoSide::Left,
            image_cache: None,
            left_texture: None,
            right_texture: None,
            resolve: ResolveState::Idle,
        }
    }
}

impl App for LocalizationMcapVisualizerApp {
    fn update(&mut self, context: &Context, _frame: &mut Frame) {
        self.advance_playback();
        self.poll_resolve();

        let log_time = self.recording.log_time_at_seconds(self.position_seconds);
        let snapshot = self.recording.latest_snapshot(log_time);
        self.update_stereo_textures(context, snapshot.image_index);

        let mut camera_matrix = snapshot.camera_matrix.clone().map(|mut matrix| {
            if let Some(intrinsics) = snapshot.calibrated_intrinsics {
                matrix.inner.intrinsics = intrinsics;
            }
            matrix.inner
        });
        let current_pose = self.current_robot_to_field(snapshot.recorded_localization);
        let global_debug = self.global_debug(
            camera_matrix.as_ref(),
            current_pose,
            &snapshot.detected_objects,
        );

        let active_frame = self
            .image_cache
            .as_ref()
            .map(|cache| cache.active(self.selected_camera));
        let resolved_trajectory = self
            .resolved_result()
            .map(ResolveResult::trajectory)
            .unwrap_or_default();
        let scene_camera_frame = active_frame.map(|image| SceneCameraFrame {
            sequence: self
                .image_cache
                .as_ref()
                .map(|cache| {
                    cache.frame.sequence * 2 + u64::from(self.selected_camera == StereoSide::Right)
                })
                .unwrap_or_default(),
            image: image.clone(),
        });

        self.widget.bevy_app.world_mut().insert_resource(SceneData {
            field_dimensions: FieldDimensions::SPL_2025,
            current_robot_to_field: current_pose.map(|pose| pose.cast()),
            camera_matrix: camera_matrix.take(),
            camera_frame: scene_camera_frame,
            recorded_trajectory: self.recorded_trajectory.clone(),
            resolved_trajectory,
            global_debug: global_debug.clone(),
        });

        self.header(context);
        self.parameters_panel(context);
        self.camera_panel(context, &snapshot.detected_objects, global_debug.as_ref());
        self.timeline_panel(context);
        self.viewport(context);
        context.request_repaint();
    }
}

impl LocalizationMcapVisualizerApp {
    fn advance_playback(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_frame_time).as_secs_f64();
        self.last_frame_time = now;
        if self.playing {
            self.position_seconds = (self.position_seconds + elapsed * self.playback_rate)
                .clamp(0.0, self.recording.duration().as_secs_f64());
            if self.position_seconds >= self.recording.duration().as_secs_f64() {
                self.playing = false;
            }
        }
    }

    fn poll_resolve(&mut self) {
        let mut finished = None;
        let mut failed = None;
        let mut cancelled = false;
        if let ResolveState::Running {
            receiver, progress, ..
        } = &mut self.resolve
        {
            while let Ok(message) = receiver.try_recv() {
                match message {
                    ResolveMessage::Progress(next_progress) => *progress = next_progress,
                    ResolveMessage::Finished(result) => finished = Some(result),
                    ResolveMessage::Failed(error) => failed = Some(error),
                    ResolveMessage::Cancelled => cancelled = true,
                }
            }
        }

        if let Some(result) = finished {
            self.resolve = ResolveState::Done(result);
        } else if let Some(error) = failed {
            self.resolve = ResolveState::Failed(error);
        } else if cancelled {
            self.resolve = ResolveState::Idle;
        }
    }

    fn update_stereo_textures(&mut self, context: &Context, image_index: Option<usize>) {
        let Some(image_index) = image_index else {
            return;
        };
        if self
            .image_cache
            .as_ref()
            .is_some_and(|cache| cache.index == image_index)
        {
            return;
        }

        match self.recording.decode_stereo_image(image_index) {
            Ok(frame) => {
                self.set_texture(context, StereoSide::Left, &frame.left);
                self.set_texture(context, StereoSide::Right, &frame.right);
                self.image_cache = Some(CachedStereoFrame {
                    index: image_index,
                    frame,
                });
            }
            Err(error) => {
                eprintln!("failed to decode stereo frame {image_index}: {error:#}");
            }
        }
    }

    fn set_texture(&mut self, context: &Context, side: StereoSide, image: &CameraImage) {
        let color_image = ColorImage::from_rgba_unmultiplied(
            [image.width as usize, image.height as usize],
            &image.rgba,
        );
        let texture = match side {
            StereoSide::Left => &mut self.left_texture,
            StereoSide::Right => &mut self.right_texture,
        };
        if let Some(texture) = texture {
            texture.set(color_image, TextureOptions::LINEAR);
        } else {
            *texture = Some(context.load_texture(
                match side {
                    StereoSide::Left => "localization_mcap_left_camera",
                    StereoSide::Right => "localization_mcap_right_camera",
                },
                color_image,
                TextureOptions::LINEAR,
            ));
        }
    }

    fn current_robot_to_field(
        &self,
        recorded_localization: Option<linear_algebra::Isometry3<Field, Robot>>,
    ) -> Option<nalgebra::Isometry3<f64>> {
        self.resolved_result()
            .and_then(|result| nearest_sample(&result.samples, self.position_seconds))
            .map(|sample| sample.robot_to_field)
            .or_else(|| {
                recorded_localization.map(|field_to_robot| field_to_robot.inverse().inner.cast())
            })
    }

    fn global_debug(
        &self,
        camera_matrix: Option<&projection::camera_matrix::CameraMatrix>,
        current_pose: Option<nalgebra::Isometry3<f64>>,
        objects: &[Object<RobocupObjectLabel>],
    ) -> Option<GlobalLocalizationDetailedDebug> {
        let camera_matrix = camera_matrix?;
        let visual_features = find_detected_visual_features(objects.to_vec());
        if visual_features.supported_feature_count()
            < self.parameters.global_localizer.min_inliers.max(3)
        {
            return None;
        }
        let pose_hint =
            current_pose.map(|pose| pose.cast::<f32>().framed_transform::<Robot, Field>());
        localize_global_visual_features_detailed_debug(
            &visual_features,
            camera_matrix,
            &FieldDimensions::SPL_2025,
            pose_hint,
            &self.parameters.global_localizer,
        )
    }

    fn header(&self, context: &Context) {
        TopBottomPanel::top("header").show(context, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.heading(RichText::new("Localization MCAP Visualizer").strong());
                ui.separator();
                ui.label(RichText::new(self.mcap_path.display().to_string()).monospace());
                ui.separator();
                ui.label(format!(
                    "{:.1}s, {} events, {} stereo frames, {} topics",
                    self.recording.duration().as_secs_f64(),
                    self.recording.events.len(),
                    self.recording.images.len(),
                    self.recording.topic_counts.len(),
                ));
                ui.separator();
                ui.label(if self.recording.field_dimensions.is_some() {
                    "field: SPL_2025 override (recording has field_dimensions)"
                } else {
                    "field: SPL_2025 fallback"
                });
                if let Some(result) = self.resolved_result() {
                    ui.separator();
                    ui.colored_label(
                        Color32::LIGHT_GREEN,
                        format!(
                            "resolved: {} solves in {:.1}s",
                            result.samples.len(),
                            result.elapsed.as_secs_f64()
                        ),
                    );
                }
            });
        });
    }

    fn parameters_panel(&mut self, context: &Context) {
        SidePanel::left("parameters_panel")
            .resizable(true)
            .default_width(360.0)
            .show(context, |ui| {
                ui.heading("Resolve Parameters");
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.label("timestamps");
                    ui.radio_value(
                        &mut self.parameters.timestamp_mode,
                        TimestampMode::McapPublish,
                        "MCAP/source",
                    );
                    ui.radio_value(
                        &mut self.parameters.timestamp_mode,
                        TimestampMode::Embedded,
                        "embedded",
                    );
                });
                numeric_row(
                    ui,
                    "solve cadence ms",
                    &mut self.parameters.solve_cadence_ms,
                    1.0..=500.0,
                );
                ui.horizontal(|ui| {
                    ui.label("optimizer iterations");
                    ui.add(DragValue::new(&mut self.parameters.optimizer_iterations).range(1..=50));
                });
                numeric_row(
                    ui,
                    "max window s",
                    &mut self.parameters.max_window_seconds,
                    0.2..=10.0,
                );
                numeric_row(
                    ui,
                    "visual feature variance",
                    &mut self.parameters.visual_feature_noise_variance,
                    1.0..=100_000.0,
                );
                numeric_row(
                    ui,
                    "VO covariance",
                    &mut self.parameters.visual_odometry_covariance,
                    1.0e-8..=1.0,
                );
                ui.separator();
                ui.checkbox(
                    &mut self.parameters.include_global_features,
                    "include global visual features",
                );
                ui.checkbox(&mut self.parameters.include_imu, "include IMU orientation");
                ui.checkbox(
                    &mut self.parameters.include_foot_heights,
                    "include foot heights",
                );
                ui.separator();
                ui.label(RichText::new("Global Localizer").strong());
                ui.horizontal(|ui| {
                    ui.label("min inliers");
                    ui.add(
                        DragValue::new(&mut self.parameters.global_localizer.min_inliers)
                            .range(3..=16),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("reprojection gate px");
                    ui.add(
                        DragValue::new(&mut self.parameters.global_localizer.reprojection_gate)
                            .speed(1.0)
                            .range(1.0..=500.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("ambiguity margin px");
                    ui.add(
                        DragValue::new(&mut self.parameters.global_localizer.ambiguity_rmse_margin)
                            .speed(0.01)
                            .range(0.0..=50.0),
                    );
                });
                ui.separator();
                self.resolve_controls(ui);
                ui.separator();
                self.diagnostics(ui);
            });
    }

    fn resolve_controls(&mut self, ui: &mut Ui) {
        match &mut self.resolve {
            ResolveState::Running {
                cancel, progress, ..
            } => {
                ui.horizontal(|ui| {
                    if ui.button("Cancel").clicked() {
                        cancel.store(true, Ordering::Relaxed);
                    }
                    ui.label(format!(
                        "{} / {} events, {} solves",
                        progress.processed_events, progress.total_events, progress.solve_count
                    ));
                });
                if progress.total_events > 0 {
                    ui.add(
                        egui::ProgressBar::new(
                            progress.processed_events as f32 / progress.total_events as f32,
                        )
                        .show_percentage(),
                    );
                }
            }
            _ => {
                if ui.button(RichText::new("Resolve").strong()).clicked() {
                    let (sender, receiver) = mpsc::channel();
                    let cancel = Arc::new(AtomicBool::new(false));
                    crate::replay::spawn_resolve(
                        self.recording.clone(),
                        self.parameters.clone(),
                        cancel.clone(),
                        sender,
                    );
                    self.resolve = ResolveState::Running {
                        receiver,
                        cancel,
                        progress: ResolveProgress {
                            processed_events: 0,
                            total_events: self.recording.events.len(),
                            solve_count: 0,
                        },
                    };
                }
                if let ResolveState::Failed(error) = &self.resolve {
                    ui.colored_label(Color32::LIGHT_RED, error);
                }
            }
        }
    }

    fn diagnostics(&self, ui: &mut Ui) {
        ui.heading("Diagnostics");
        let Some(result) = self.resolved_result() else {
            ui.label(
                RichText::new("Run Resolve to populate solve diagnostics.").color(Color32::GRAY),
            );
            return;
        };

        let stats = &result.stats;
        ui.label(format!(
            "VO: {} received, {} ingested, {} stale camera skips",
            stats.vo_received, stats.vo_ingested, stats.vo_skipped_stale_camera_matrix
        ));
        ui.label(format!(
            "Global: {} candidates, {} frames ingested, {} associations",
            stats.global_candidates,
            stats.global_frames_ingested,
            stats.global_associations_ingested
        ));
        if let Some(sample) = nearest_sample(&result.samples, self.position_seconds) {
            ui.separator();
            ui.label(format!(
                "selected solve: {:.2} ms",
                sample.solve_duration.as_secs_f64() * 1000.0
            ));
            ui.label(format!("graph time: {:.2}s", sample.graph_seconds));
            ui.label(format!(
                "cumulative VO/global: {} / {}",
                sample.stats.vo_ingested, sample.stats.global_associations_ingested
            ));
            if let Some(diagnostics) = &sample.diagnostics {
                ui.label(format!("optimizer: {:?}", diagnostics.optimizer_status));
                ui.label(format!(
                    "values/factors: {} / {}",
                    diagnostics.value_count, diagnostics.factor_count
                ));
                ui.label(format!("total error: {:.3}", diagnostics.total_error));
                ui.label(format!(
                    "VO RMS mean/max: {:.3} / {:.3}",
                    diagnostics.visual_odometry.mean_rms, diagnostics.visual_odometry.max_rms
                ));
                ui.label(format!(
                    "visual RMS mean/max: {:.3} / {:.3}",
                    diagnostics.visual_reprojection.mean_rms,
                    diagnostics.visual_reprojection.max_rms
                ));
                ui.label(format!(
                    "GP RMS mean/max: {:.3} / {:.3}",
                    diagnostics.gaussian_process_prior.mean_rms,
                    diagnostics.gaussian_process_prior.max_rms
                ));
            }
        }
        Plot::new("solve_duration_plot")
            .height(140.0)
            .show(ui, |plot_ui| {
                let points = PlotPoints::from_iter(result.samples.iter().map(|sample| {
                    [
                        sample.replay_seconds,
                        sample.solve_duration.as_secs_f64() * 1000.0,
                    ]
                }));
                plot_ui.line(Line::new("solve ms", points));
            });
        ui.label(format!(
            "resolved with {} iterations, {:.1} ms cadence, visual variance {:.1}",
            result.parameters.optimizer_iterations,
            result.parameters.solve_cadence_ms,
            result.parameters.visual_feature_noise_variance,
        ));
    }

    fn camera_panel(
        &mut self,
        context: &Context,
        detected_objects: &[Object<RobocupObjectLabel>],
        global_debug: Option<&GlobalLocalizationDetailedDebug>,
    ) {
        SidePanel::right("camera_panel")
            .resizable(true)
            .default_width(520.0)
            .show(context, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("Stereo Camera");
                    ui.selectable_value(&mut self.selected_camera, StereoSide::Left, "left");
                    ui.selectable_value(&mut self.selected_camera, StereoSide::Right, "right");
                });
                let texture = match self.selected_camera {
                    StereoSide::Left => self.left_texture.as_ref(),
                    StereoSide::Right => self.right_texture.as_ref(),
                };
                let image = self
                    .image_cache
                    .as_ref()
                    .map(|cache| cache.active(self.selected_camera));
                match (texture, image) {
                    (Some(texture), Some(image)) => {
                        self.camera_image(ui, texture, image, detected_objects, global_debug)
                    }
                    _ => {
                        ui.centered_and_justified(|ui| {
                            ui.label(
                                RichText::new("recording has no decoded stereo image at this time")
                                    .color(Color32::GRAY),
                            );
                        });
                    }
                }
                ui.separator();
                self.global_debug_panel(ui, global_debug);
            });
    }

    fn camera_image(
        &self,
        ui: &mut Ui,
        texture: &TextureHandle,
        image: &CameraImage,
        detected_objects: &[Object<RobocupObjectLabel>],
        global_debug: Option<&GlobalLocalizationDetailedDebug>,
    ) {
        let image_size = vec2(image.width as f32, image.height as f32);
        let available = ui.available_size().max(vec2(1.0, 1.0));
        let scale = (available.x / image_size.x)
            .min((available.y - 220.0).max(1.0) / image_size.y)
            .max(0.05);
        let response = ui.add(
            egui::Image::new((texture.id(), texture.size_vec2()))
                .fit_to_exact_size(image_size * scale)
                .sense(Sense::hover()),
        );
        if self.selected_camera == StereoSide::Left {
            draw_detected_objects(ui, response.rect, image_size, detected_objects);
            if let Some(debug) = global_debug {
                draw_global_debug_overlay(ui, response.rect, image_size, debug);
            }
        }
        ui.horizontal_wrapped(|ui| {
            ui.label(format!("{}x{}", image.width, image.height));
            ui.separator();
            ui.label(format!("{} detections", detected_objects.len()));
            if let Some(cache) = &self.image_cache {
                ui.separator();
                ui.label(format!("frame {}", cache.index));
                ui.separator();
                ui.label(format!(
                    "image log {:.2}s source {:?}",
                    self.recording.seconds_since_start(cache.frame.log_time),
                    cache.frame.source_time,
                ));
                ui.separator();
                ui.label(format!(
                    "publish {:.2}s",
                    self.recording.seconds_since_start(cache.frame.publish_time),
                ));
            }
        });
    }

    fn global_debug_panel(
        &self,
        ui: &mut Ui,
        global_debug: Option<&GlobalLocalizationDetailedDebug>,
    ) {
        ui.heading("Global Localization Debug");
        let Some(debug) = global_debug else {
            ui.label(
                RichText::new("No global-localization candidate at this frame.")
                    .color(Color32::GRAY),
            );
            return;
        };
        ui.label(format!("status: {:?}", debug.status));
        ui.label(format!("inliers: {}", debug.score.inliers));
        ui.label(format!("RMSE: {:.2}px", debug.score.reprojection_rmse));
        ui.label(format!("total cost: {:.1}", debug.score.total_cost));
        ui.label(format!(
            "detections: {}, projected candidates: {}, associations: {}",
            debug.detections.len(),
            debug.projected_features.len(),
            debug.associations.len()
        ));
        ui.separator();
        egui::ScrollArea::vertical()
            .max_height(180.0)
            .show(ui, |ui| {
                for association in &debug.associations {
                    let error = association
                        .reprojection_error_px
                        .map(|error| format!("{error:.1}px"))
                        .unwrap_or_else(|| "not visible".to_string());
                    ui.label(format!(
                        "#{}/#{} {:?}: det ({:.1},{:.1}) -> field ({:.2},{:.2}) error {error}",
                        association.detection_index,
                        association.feature_index,
                        association.class,
                        association.detection_pixel.x(),
                        association.detection_pixel.y(),
                        association.field_point.x(),
                        association.field_point.y(),
                    ));
                }
            });
    }

    fn timeline_panel(&mut self, context: &Context) {
        TopBottomPanel::bottom("timeline_panel").show(context, |ui| {
            ui.horizontal(|ui| {
                if ui
                    .button(if self.playing { "Pause" } else { "Play" })
                    .clicked()
                {
                    self.playing = !self.playing;
                }
                if ui.button("<").clicked() {
                    self.position_seconds = (self.position_seconds - 1.0).max(0.0);
                }
                if ui.button(">").clicked() {
                    self.position_seconds =
                        (self.position_seconds + 1.0).min(self.recording.duration().as_secs_f64());
                }
                ui.label("speed");
                ui.add(
                    DragValue::new(&mut self.playback_rate)
                        .speed(0.1)
                        .range(0.1..=10.0),
                );
                ui.label(format!(
                    "{:.2}s / {:.2}s",
                    self.position_seconds,
                    self.recording.duration().as_secs_f64()
                ));
            });
            ui.add(
                Slider::new(
                    &mut self.position_seconds,
                    0.0..=self.recording.duration().as_secs_f64(),
                )
                .show_value(false),
            );
        });
    }

    fn viewport(&mut self, context: &Context) {
        CentralPanel::default()
            .frame(egui::Frame::central_panel(&context.style()).fill(Color32::from_rgb(16, 18, 22)))
            .show(context, |ui| {
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.heading("3D Field");
                        ui.label(RichText::new("pan/zoom with mouse").color(Color32::GRAY));
                    });
                    self.widget.ui(ui);
                });
            });
    }

    fn resolved_result(&self) -> Option<&ResolveResult> {
        match &self.resolve {
            ResolveState::Done(result) => Some(result),
            _ => None,
        }
    }
}

fn numeric_row(ui: &mut Ui, label: &str, value: &mut f64, range: std::ops::RangeInclusive<f64>) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(DragValue::new(value).speed(0.1).range(range));
    });
}

fn nearest_sample<'a>(
    samples: &'a [crate::replay::SolveSample],
    seconds: f64,
) -> Option<&'a crate::replay::SolveSample> {
    samples.iter().min_by(|left, right| {
        (left.replay_seconds - seconds)
            .abs()
            .total_cmp(&(right.replay_seconds - seconds).abs())
    })
}

fn draw_detected_objects(
    ui: &mut Ui,
    image_rect: Rect,
    image_size: Vec2,
    detected_objects: &[Object<RobocupObjectLabel>],
) {
    let scale = vec2(
        image_rect.width() / image_size.x.max(1.0),
        image_rect.height() / image_size.y.max(1.0),
    );
    for object in detected_objects {
        let color = object_label_color(object.label);
        let min = image_rect.min
            + vec2(
                object.bounding_box.area.min.x() * scale.x,
                object.bounding_box.area.min.y() * scale.y,
            );
        let max = image_rect.min
            + vec2(
                object.bounding_box.area.max.x() * scale.x,
                object.bounding_box.area.max.y() * scale.y,
            );
        let rect = Rect::from_min_max(min, max).intersect(image_rect);
        let painter = ui.painter();
        painter.rect_stroke(
            rect,
            egui::CornerRadius::same(4),
            Stroke::new(2.0, color),
            StrokeKind::Outside,
        );
        let label: String = object.label.into();
        let text = format!("{label} {:.0}%", object.bounding_box.confidence * 100.0);
        let text_position = pos2(rect.min.x + 5.0, rect.min.y + 5.0);
        let galley = painter.layout_no_wrap(text, FontId::proportional(13.0), Color32::WHITE);
        let label_rect = Rect::from_min_size(
            text_position - vec2(3.0, 2.0),
            galley.size() + vec2(6.0, 4.0),
        );
        painter.rect_filled(
            label_rect,
            egui::CornerRadius::same(3),
            color.gamma_multiply(0.85),
        );
        painter.galley(text_position, galley, Color32::WHITE);
    }
}

fn draw_global_debug_overlay(
    ui: &mut Ui,
    image_rect: Rect,
    image_size: Vec2,
    debug: &GlobalLocalizationDetailedDebug,
) {
    let scale = vec2(
        image_rect.width() / image_size.x.max(1.0),
        image_rect.height() / image_size.y.max(1.0),
    );
    let painter = ui.painter();

    for projection in &debug.projected_features {
        let Some(pixel) = projection.projected_pixel else {
            continue;
        };
        let position = image_rect.min + vec2(pixel.x() * scale.x, pixel.y() * scale.y);
        if !image_rect.contains(position) {
            continue;
        }
        let color = if projection.accepted {
            Color32::WHITE
        } else {
            feature_class_color(projection.class).gamma_multiply(0.45)
        };
        painter.circle_stroke(position, 4.0, Stroke::new(1.5, color));
    }

    for association in &debug.associations {
        let Some(projected) = association.projected_pixel else {
            continue;
        };
        let detection = image_rect.min
            + vec2(
                association.detection_pixel.x() * scale.x,
                association.detection_pixel.y() * scale.y,
            );
        let projection = image_rect.min + vec2(projected.x() * scale.x, projected.y() * scale.y);
        let color = feature_class_color(association.class);
        painter.line_segment([detection, projection], Stroke::new(2.0, color));
        painter.circle_filled(detection, 4.0, color);
        painter.circle_filled(projection, 3.0, Color32::WHITE);
    }
}

fn object_label_color(label: RobocupObjectLabel) -> Color32 {
    match label {
        RobocupObjectLabel::Ball => Color32::from_rgb(255, 145, 64),
        RobocupObjectLabel::GoalPost => Color32::from_rgb(245, 245, 245),
        RobocupObjectLabel::Robot => Color32::from_rgb(82, 170, 255),
        RobocupObjectLabel::PenaltySpot => Color32::from_rgb(255, 230, 96),
        RobocupObjectLabel::LSpot | RobocupObjectLabel::TSpot | RobocupObjectLabel::XSpot => {
            Color32::from_rgb(120, 255, 170)
        }
    }
}

fn feature_class_color(class: VisualFeatureClass) -> Color32 {
    match class {
        VisualFeatureClass::GoalPost => Color32::WHITE,
        VisualFeatureClass::LSpot => Color32::from_rgb(120, 255, 170),
        VisualFeatureClass::TSpot => Color32::from_rgb(120, 180, 255),
        VisualFeatureClass::PenaltySpot => Color32::from_rgb(255, 230, 96),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum StereoSide {
    Left,
    Right,
}

#[derive(Clone)]
struct CachedStereoFrame {
    index: usize,
    frame: StereoFrame,
}

impl CachedStereoFrame {
    fn active(&self, side: StereoSide) -> &CameraImage {
        match side {
            StereoSide::Left => &self.frame.left,
            StereoSide::Right => &self.frame.right,
        }
    }
}

enum ResolveState {
    Idle,
    Running {
        receiver: Receiver<ResolveMessage>,
        cancel: Arc<AtomicBool>,
        progress: ResolveProgress,
    },
    Done(ResolveResult),
    Failed(String),
}
