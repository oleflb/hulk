use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver},
    },
    time::{Instant, SystemTime},
};

use color_eyre::{Result, eyre::eyre};
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
    GlobalLocalizationDetailedDebug, GlobalLocalizerParameters, VisualFeatureClass,
    find_detected_visual_features, initial_robot_to_field_from_camera_matrix,
    localize_global_visual_features_detailed_debug,
};
use projection::camera_matrix::CameraMatrix;
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
};

use crate::{
    mcap_recording::{
        CameraImage, Recording, StereoFrame, StereoImageId, TrajectoryPoint, nanos_since_epoch,
    },
    nearest_by_distance,
    replay::{ReplayParameters, ResolveMessage, ResolveProgress, ResolveResult, TimestampMode},
    scene::{self, SceneCameraFrame, SceneCameraSide, SceneData, SceneFrameSequence, SceneVersion},
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
    image_zoom: f32,
    image_cache: Option<CachedStereoFrame>,
    left_texture: Option<TextureHandle>,
    right_texture: Option<TextureHandle>,
    resolve: ResolveState,
    resolve_version: SceneVersion,
    camera_matrix_key: Option<CameraMatrixKey>,
    camera_version: SceneVersion,
    global_debug_cache: CachedGlobalDebug,
}

impl LocalizationMcapVisualizerApp {
    pub fn new(
        creation_context: &CreationContext,
        mcap_path: PathBuf,
        recording: Arc<Recording>,
    ) -> Result<Self> {
        creation_context.egui_ctx.set_visuals(egui::Visuals::dark());

        let render_state = creation_context
            .wgpu_render_state
            .clone()
            .ok_or_else(|| eyre!("no WGPU render state found"))?;
        let mut widget = BevyWidget::new(render_state);
        scene::configure(&mut widget.bevy_app);
        widget.bevy_app.finish();
        widget.bevy_app.cleanup();

        Ok(Self {
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
            image_zoom: 1.0,
            image_cache: None,
            left_texture: None,
            right_texture: None,
            resolve: ResolveState::Idle,
            resolve_version: SceneVersion::default(),
            camera_matrix_key: None,
            camera_version: SceneVersion::default(),
            global_debug_cache: CachedGlobalDebug::default(),
        })
    }
}

impl App for LocalizationMcapVisualizerApp {
    fn update(&mut self, context: &Context, _frame: &mut Frame) {
        self.advance_playback();
        self.poll_resolve();

        let log_time = self.recording.log_time_at_seconds(self.position_seconds);
        let snapshot = self.recording.latest_snapshot(log_time);
        self.update_stereo_textures(context, snapshot.image_id);

        let camera_matrix_key = snapshot
            .camera_matrix
            .as_ref()
            .map(|matrix| CameraMatrixKey {
                time_nanos: matrix.time.as_nanos(),
                calibrated_intrinsics_time_nanos: snapshot
                    .calibrated_intrinsics_time
                    .map(nanos_since_epoch),
            });
        let mut camera_matrix = snapshot.camera_matrix.clone().map(|mut matrix| {
            if let Some(intrinsics) = snapshot.calibrated_intrinsics {
                matrix.inner.intrinsics = intrinsics;
            }
            matrix.inner
        });
        if self.camera_matrix_key != camera_matrix_key {
            self.camera_matrix_key = camera_matrix_key;
            self.camera_version = self.camera_version.next();
        }

        let current_pose = self.current_robot_to_field(snapshot.recorded_localization);
        let debug_pose = self.robot_to_field_at_log_time(snapshot.detected_objects_time);
        let global_debug = self.update_global_debug_cache(
            camera_matrix.as_ref(),
            camera_matrix_key,
            snapshot.detected_objects_time,
            debug_pose,
            &snapshot.detected_objects,
        );
        let camera_matrix_for_ui = camera_matrix.clone();
        self.update_scene_data(camera_matrix.take(), current_pose, global_debug.clone());

        let position_before_ui = self.position_seconds;
        let selected_camera_before_ui = self.selected_camera;
        let parameters_before_ui = self.parameters.clone();
        let resolve_version_before_ui = self.resolve_version;
        let playing_before_ui = self.playing;

        self.header(context);
        self.parameters_panel(context);
        self.camera_panel(
            context,
            &snapshot.detected_objects,
            snapshot.detected_objects_time,
            camera_matrix_for_ui.as_ref(),
            global_debug.as_deref(),
        );
        self.timeline_panel(context);
        self.viewport(context);
        let ui_changed_scene_inputs = self.position_seconds != position_before_ui
            || self.selected_camera != selected_camera_before_ui
            || self.parameters != parameters_before_ui
            || self.resolve_version != resolve_version_before_ui
            || self.playing != playing_before_ui;
        if self.playing
            || matches!(self.resolve, ResolveState::Running { .. })
            || ui_changed_scene_inputs
        {
            context.request_repaint();
        }
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
            self.resolve_version = self.resolve_version.next();
            self.resolve = ResolveState::Done(result);
        } else if let Some(error) = failed {
            self.resolve = ResolveState::Failed(error);
        } else if cancelled {
            self.resolve = ResolveState::Idle;
        }
    }

    fn update_scene_data(
        &mut self,
        camera_matrix: Option<CameraMatrix>,
        current_pose: linear_algebra::Isometry3<Robot, Field, f64>,
        global_debug: Option<Arc<GlobalLocalizationDetailedDebug>>,
    ) {
        let selected_frame_sequence = self.selected_scene_frame_sequence();
        let (
            current_frame_sequence,
            field_dimensions_version,
            camera_version,
            recorded_trajectory_version,
            resolved_trajectory_version,
            global_debug_version,
        ) = {
            let scene_data = self.widget.bevy_app.world_mut().resource::<SceneData>();
            (
                scene_data.camera_frame_sequence(),
                scene_data.field_dimensions_version(),
                scene_data.camera_version(),
                scene_data.recorded_trajectory_version(),
                scene_data.resolved_trajectory_version(),
                scene_data.global_debug_version(),
            )
        };
        let next_camera_frame = if current_frame_sequence != selected_frame_sequence {
            self.selected_scene_camera_frame()
        } else {
            None
        };
        let next_recorded_trajectory = if recorded_trajectory_version != SceneVersion::READY {
            Some(self.recorded_trajectory.clone())
        } else {
            None
        };
        let next_resolved_trajectory = if resolved_trajectory_version != self.resolve_version {
            Some(match self.resolved_result() {
                Some(result) => result.trajectory(),
                None => Vec::new(),
            })
        } else {
            None
        };

        let mut scene_data = self.widget.bevy_app.world_mut().resource_mut::<SceneData>();
        if field_dimensions_version != SceneVersion::READY {
            scene_data.set_field_dimensions(FieldDimensions::SPL_2025);
        }
        scene_data.set_current_robot_to_field(Some(current_pose.inner.cast().framed_transform()));

        if camera_version != self.camera_version {
            scene_data.set_camera_matrix(camera_matrix);
        }
        if current_frame_sequence != selected_frame_sequence {
            scene_data.set_camera_frame(next_camera_frame);
        }
        if let Some(recorded_trajectory) = next_recorded_trajectory {
            scene_data.set_recorded_trajectory(recorded_trajectory);
        }
        if let Some(resolved_trajectory) = next_resolved_trajectory {
            scene_data.set_resolved_trajectory(resolved_trajectory);
        }
        if global_debug_version != self.global_debug_cache.version {
            scene_data.set_global_debug(global_debug);
        }
    }

    fn selected_scene_frame_sequence(&self) -> Option<SceneFrameSequence> {
        self.image_cache
            .as_ref()
            .map(|cache| self.scene_frame_sequence_for(&cache.frame, self.scene_camera_side()))
    }

    fn selected_scene_camera_frame(&self) -> Option<SceneCameraFrame> {
        let side = self.scene_camera_side();
        self.image_cache.as_ref().map(|cache| SceneCameraFrame {
            sequence: self.scene_frame_sequence_for(&cache.frame, side),
            image: cache.active(self.scene_stereo_side(side)).clone(),
        })
    }

    fn scene_frame_sequence_for(
        &self,
        frame: &StereoFrame,
        side: SceneCameraSide,
    ) -> SceneFrameSequence {
        SceneFrameSequence::stereo(frame.sequence, side)
    }

    fn scene_camera_side(&self) -> SceneCameraSide {
        SceneCameraSide::Left
    }

    fn scene_stereo_side(&self, side: SceneCameraSide) -> StereoSide {
        match side {
            SceneCameraSide::Left => StereoSide::Left,
        }
    }

    fn update_stereo_textures(&mut self, context: &Context, image_id: Option<StereoImageId>) {
        let Some(image_id) = image_id else {
            return;
        };
        if self
            .image_cache
            .as_ref()
            .is_some_and(|cache| cache.image_id == image_id)
        {
            return;
        }

        match self.recording.decode_stereo_image(image_id) {
            Ok(frame) => {
                self.set_texture(context, StereoSide::Left, &frame.left);
                self.set_texture(context, StereoSide::Right, &frame.right);
                self.image_cache = Some(CachedStereoFrame { image_id, frame });
            }
            Err(error) => {
                eprintln!("failed to decode stereo frame {image_id}: {error:#}");
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
    ) -> linear_algebra::Isometry3<Robot, Field, f64> {
        let pose = self
            .resolved_result()
            .and_then(|result| nearest_sample(&result.samples, self.position_seconds))
            .map(|sample| sample.robot_to_field)
            .or_else(|| {
                recorded_localization
                    .map(|field_to_robot| field_to_robot.inverse().inner.cast().framed_transform())
            })
            .or_else(|| {
                nearest_trajectory_point(&self.recorded_trajectory, self.position_seconds)
                    .map(|point| point.robot_to_field)
            });
        match pose {
            Some(pose) => pose,
            None => self.initial_robot_to_field(),
        }
    }

    fn robot_to_field_at_log_time(
        &self,
        log_time: Option<SystemTime>,
    ) -> linear_algebra::Isometry3<Robot, Field, f64> {
        let seconds = match log_time {
            Some(log_time) => self.recording.seconds_since_start(log_time),
            None => self.position_seconds,
        };
        let pose = self
            .resolved_result()
            .and_then(|result| nearest_sample(&result.samples, seconds))
            .map(|sample| sample.robot_to_field)
            .or_else(|| {
                nearest_trajectory_point(&self.recorded_trajectory, seconds)
                    .map(|point| point.robot_to_field)
            });
        match pose {
            Some(pose) => pose,
            None => self.initial_robot_to_field(),
        }
    }

    fn initial_robot_to_field(&self) -> linear_algebra::Isometry3<Robot, Field, f64> {
        initial_robot_to_field_from_camera_matrix(&self.recording.first_camera_matrix)
    }

    fn update_global_debug_cache(
        &mut self,
        camera_matrix: Option<&projection::camera_matrix::CameraMatrix>,
        camera_matrix_key: Option<CameraMatrixKey>,
        detected_objects_time: Option<SystemTime>,
        current_pose: linear_algebra::Isometry3<Robot, Field, f64>,
        objects: &[Object<RobocupObjectLabel>],
    ) -> Option<Arc<GlobalLocalizationDetailedDebug>> {
        let key = camera_matrix_key.map(|camera_matrix_key| GlobalDebugKey {
            camera_matrix_key,
            detected_objects_time_nanos: detected_objects_time.map(nanos_since_epoch),
            pose_revision: self.resolve_version,
            global_localizer: self.parameters.global_localizer,
        });

        if self.global_debug_cache.key != key {
            let debug = self
                .compute_global_debug(camera_matrix, current_pose, objects)
                .map(Arc::new);
            self.global_debug_cache.key = key;
            self.global_debug_cache.version = self.global_debug_cache.version.next();
            self.global_debug_cache.debug = debug;
        }

        self.global_debug_cache.debug.clone()
    }

    fn compute_global_debug(
        &self,
        camera_matrix: Option<&projection::camera_matrix::CameraMatrix>,
        current_pose: linear_algebra::Isometry3<Robot, Field, f64>,
        objects: &[Object<RobocupObjectLabel>],
    ) -> Option<GlobalLocalizationDetailedDebug> {
        let camera_matrix = camera_matrix?;
        let visual_features = find_detected_visual_features(objects.to_vec());
        if visual_features.supported_feature_count()
            < self.parameters.global_localizer.min_inliers.max(3)
        {
            return None;
        }
        let pose_hint = Some(current_pose.inner.cast().framed_transform());
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
                    self.recording.event_count(),
                    self.recording.image_count(),
                    self.recording.topic_count(),
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
                    self.resolve_version = self.resolve_version.next();
                    self.resolve = ResolveState::Running {
                        receiver,
                        cancel,
                        progress: ResolveProgress {
                            processed_events: 0,
                            total_events: self.recording.event_count(),
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
        detected_objects_time: Option<SystemTime>,
        camera_matrix: Option<&CameraMatrix>,
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
                ui.horizontal(|ui| {
                    ui.label("zoom");
                    ui.add(Slider::new(&mut self.image_zoom, 0.1..=8.0).logarithmic(true));
                    if ui.button("1:1").clicked() {
                        self.image_zoom = 1.0;
                    }
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
                    (Some(texture), Some(image)) => self.camera_image(
                        ui,
                        texture,
                        image,
                        detected_objects,
                        detected_objects_time,
                        global_debug,
                    ),
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
                ui.separator();
                self.top_down_association_view(ui, global_debug, camera_matrix);
            });
    }

    fn camera_image(
        &self,
        ui: &mut Ui,
        texture: &TextureHandle,
        image: &CameraImage,
        detected_objects: &[Object<RobocupObjectLabel>],
        detected_objects_time: Option<SystemTime>,
        global_debug: Option<&GlobalLocalizationDetailedDebug>,
    ) {
        let image_size = vec2(image.width as f32, image.height as f32);
        let available = ui.available_size().max(vec2(1.0, 1.0));
        let fit_scale = (available.x / image_size.x)
            .min((available.y - 300.0).max(1.0) / image_size.y)
            .max(0.05);
        let scale = fit_scale * self.image_zoom;
        egui::ScrollArea::both()
            .max_height((available.y - 280.0).max(220.0))
            .show(ui, |ui| {
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
                    let hover_info =
                        image_hover_info(&response, image_size, detected_objects, global_debug);
                    if !hover_info.is_empty() {
                        response.on_hover_ui(|ui| {
                            for line in hover_info {
                                ui.label(line);
                            }
                        });
                    }
                }
            });
        ui.horizontal_wrapped(|ui| {
            ui.label(format!("{}x{}", image.width, image.height));
            ui.separator();
            ui.label(format!("{} detections", detected_objects.len()));
            if let Some(cache) = &self.image_cache {
                ui.separator();
                ui.label(format!("frame {}", cache.image_id));
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
                if let Some(detected_objects_time) = detected_objects_time {
                    let delta_ms = (self.recording.seconds_since_start(detected_objects_time)
                        - self.recording.seconds_since_start(cache.frame.log_time))
                        * 1000.0;
                    ui.separator();
                    ui.label(format!("detections Δ {delta_ms:.1} ms"));
                }
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
                    let error = match association.reprojection_error_px {
                        Some(error) => format!("{error:.1}px"),
                        None => "not visible".to_string(),
                    };
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

    fn top_down_association_view(
        &self,
        ui: &mut Ui,
        global_debug: Option<&GlobalLocalizationDetailedDebug>,
        camera_matrix: Option<&CameraMatrix>,
    ) {
        ui.heading("Top-Down Associations");
        let (Some(debug), Some(camera_matrix)) = (global_debug, camera_matrix) else {
            ui.label(RichText::new("No associations for this frame.").color(Color32::GRAY));
            return;
        };

        let available_width = ui.available_width().max(240.0);
        let (rect, response) = ui.allocate_exact_size(vec2(available_width, 220.0), Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(
            rect,
            egui::CornerRadius::same(4),
            Color32::from_rgb(24, 44, 31),
        );

        let dimensions = FieldDimensions::SPL_2025;
        let field_rect = top_down_field_rect(rect, &dimensions);
        painter.rect_stroke(
            field_rect,
            egui::CornerRadius::same(2),
            Stroke::new(1.5, Color32::WHITE),
            StrokeKind::Inside,
        );
        painter.line_segment(
            [
                pos2(field_rect.center().x, field_rect.top()),
                pos2(field_rect.center().x, field_rect.bottom()),
            ],
            Stroke::new(1.0, Color32::GRAY),
        );

        let ground_to_field = debug.robot_to_field.inner * camera_matrix.ground_to_robot.inner;
        for association in &debug.associations {
            let ground = association.back_projected_ground;
            let detected_field =
                ground_to_field * nalgebra::Point3::new(ground.x(), ground.y(), 0.0);
            let detected_position =
                field_to_screen(field_rect, &dimensions, detected_field.x, detected_field.y);
            let feature_position = field_to_screen(
                field_rect,
                &dimensions,
                association.field_point.x(),
                association.field_point.y(),
            );
            let color = feature_class_color(association.class);
            painter.line_segment(
                [detected_position, feature_position],
                Stroke::new(1.5, color.gamma_multiply(0.75)),
            );
            painter.circle_filled(detected_position, 3.5, color);
            painter.circle_stroke(feature_position, 5.0, Stroke::new(1.5, Color32::WHITE));
        }

        let robot_translation = debug.robot_to_field.inner.translation.vector;
        painter.circle_filled(
            field_to_screen(
                field_rect,
                &dimensions,
                robot_translation.x,
                robot_translation.y,
            ),
            4.0,
            Color32::from_rgb(82, 170, 255),
        );

        response.on_hover_text(format!(
            "{} associations, robot ({:.2}, {:.2})",
            debug.associations.len(),
            robot_translation.x,
            robot_translation.y
        ));
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
                if ui.button("< frame").clicked() {
                    self.step_frame(-1);
                }
                if ui.button("frame >").clicked() {
                    self.step_frame(1);
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
            if self.recording.image_count() > 0 {
                let mut frame_index = self.current_frame_index();
                let max_frame_index = self.recording.image_count() - 1;
                if ui
                    .add(Slider::new(&mut frame_index, 0..=max_frame_index).text("frame"))
                    .changed()
                {
                    self.set_frame_index(frame_index);
                }
            }
        });
    }

    fn current_frame_index(&self) -> usize {
        match &self.image_cache {
            Some(cache) => cache.image_id.index(),
            None => 0,
        }
    }

    fn step_frame(&mut self, offset: isize) {
        if self.recording.image_count() == 0 {
            return;
        }
        let max_frame_index = self.recording.image_count() - 1;
        let current = self.current_frame_index();
        let next = if offset.is_negative() {
            current.saturating_sub(offset.unsigned_abs())
        } else {
            current.saturating_add(offset as usize).min(max_frame_index)
        };
        self.set_frame_index(next);
    }

    fn set_frame_index(&mut self, frame_index: usize) {
        let Some(image_id) = self.recording.image_id_from_index(frame_index) else {
            return;
        };
        let Some(log_time) = self.recording.image_log_time(image_id) else {
            return;
        };
        self.position_seconds = self
            .recording
            .seconds_since_start(log_time)
            .clamp(0.0, self.recording.duration().as_secs_f64());
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

fn nearest_sample(
    samples: &[crate::replay::SolveSample],
    seconds: f64,
) -> Option<&crate::replay::SolveSample> {
    if samples.is_empty() {
        return None;
    }
    let next = samples.partition_point(|sample| sample.replay_seconds <= seconds);
    nearest_by_seconds(
        next.checked_sub(1).and_then(|index| samples.get(index)),
        samples.get(next),
        seconds,
        |sample| sample.replay_seconds,
    )
}

fn nearest_trajectory_point(points: &[TrajectoryPoint], seconds: f64) -> Option<&TrajectoryPoint> {
    if points.is_empty() {
        return None;
    }
    let next = points.partition_point(|point| point.seconds <= seconds);
    nearest_by_seconds(
        next.checked_sub(1).and_then(|index| points.get(index)),
        points.get(next),
        seconds,
        |point| point.seconds,
    )
}

fn nearest_by_seconds<'a, T>(
    previous: Option<&'a T>,
    next: Option<&'a T>,
    seconds: f64,
    get_seconds: impl Fn(&T) -> f64,
) -> Option<&'a T> {
    nearest_by_distance(
        previous.map(|previous| (previous, (get_seconds(previous) - seconds).abs())),
        next.map(|next| (next, (get_seconds(next) - seconds).abs())),
    )
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

fn image_hover_info(
    response: &egui::Response,
    image_size: Vec2,
    detected_objects: &[Object<RobocupObjectLabel>],
    global_debug: Option<&GlobalLocalizationDetailedDebug>,
) -> Vec<String> {
    let Some(pointer) = response.hover_pos() else {
        return Vec::new();
    };
    let Some(pixel) = screen_to_image_pixel(response.rect, image_size, pointer) else {
        return Vec::new();
    };
    let mut lines = Vec::new();

    for object in detected_objects {
        let min = vec2(
            object.bounding_box.area.min.x(),
            object.bounding_box.area.min.y(),
        );
        let max = vec2(
            object.bounding_box.area.max.x(),
            object.bounding_box.area.max.y(),
        );
        if pixel.x >= min.x && pixel.x <= max.x && pixel.y >= min.y && pixel.y <= max.y {
            let label: String = object.label.into();
            lines.push(format!(
                "detection: {label} {:.0}% bbox ({:.0},{:.0})-({:.0},{:.0})",
                object.bounding_box.confidence * 100.0,
                min.x,
                min.y,
                max.x,
                max.y,
            ));
        }
    }

    if let Some(debug) = global_debug {
        for detection in &debug.detections {
            if pixel_distance(pixel, vec2(detection.pixel.x(), detection.pixel.y())) <= 10.0 {
                lines.push(format!(
                    "feature detection #{} {:?} pixel ({:.1},{:.1}) ground ({:.2},{:.2})",
                    detection.index,
                    detection.class,
                    detection.pixel.x(),
                    detection.pixel.y(),
                    detection.ground.x(),
                    detection.ground.y(),
                ));
            }
        }
        for projection in &debug.projected_features {
            let Some(projected_pixel) = projection.projected_pixel else {
                continue;
            };
            if pixel_distance(pixel, vec2(projected_pixel.x(), projected_pixel.y())) <= 10.0 {
                lines.push(format!(
                    "projection #{} sym #{} {:?} field ({:.2},{:.2}) {}",
                    projection.index,
                    projection.symmetric_index,
                    projection.class,
                    projection.field_point.x(),
                    projection.field_point.y(),
                    if projection.accepted {
                        "accepted"
                    } else {
                        "candidate"
                    },
                ));
            }
        }
        for association in &debug.associations {
            let detection_distance = pixel_distance(
                pixel,
                vec2(
                    association.detection_pixel.x(),
                    association.detection_pixel.y(),
                ),
            );
            let projection_distance = match association.projected_pixel {
                Some(projected_pixel) => {
                    pixel_distance(pixel, vec2(projected_pixel.x(), projected_pixel.y()))
                }
                None => f32::INFINITY,
            };
            if detection_distance <= 10.0 || projection_distance <= 10.0 {
                let error = match association.reprojection_error_px {
                    Some(error) => format!("{error:.1}px"),
                    None => "not visible".to_string(),
                };
                lines.push(format!(
                    "association det #{} -> feature #{} {:?} error {error} field ({:.2},{:.2})",
                    association.detection_index,
                    association.feature_index,
                    association.class,
                    association.field_point.x(),
                    association.field_point.y(),
                ));
            }
        }
    }

    lines
}

fn screen_to_image_pixel(image_rect: Rect, image_size: Vec2, pointer: egui::Pos2) -> Option<Vec2> {
    if !image_rect.contains(pointer) {
        return None;
    }
    Some(vec2(
        (pointer.x - image_rect.left()) / image_rect.width().max(1.0) * image_size.x,
        (pointer.y - image_rect.top()) / image_rect.height().max(1.0) * image_size.y,
    ))
}

fn pixel_distance(left: Vec2, right: Vec2) -> f32 {
    (left - right).length()
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

fn top_down_field_rect(rect: Rect, dimensions: &FieldDimensions) -> Rect {
    let field_length = dimensions.length.max(1.0);
    let field_width = dimensions.width.max(1.0);
    let scale = (rect.width() / field_length).min(rect.height() / field_width) * 0.92;
    Rect::from_center_size(
        rect.center(),
        vec2(field_length * scale, field_width * scale),
    )
}

fn field_to_screen(
    field_rect: Rect,
    dimensions: &FieldDimensions,
    field_x: f32,
    field_y: f32,
) -> egui::Pos2 {
    let x = field_rect.center().x + field_x / dimensions.length.max(1.0) * field_rect.width();
    let y = field_rect.center().y - field_y / dimensions.width.max(1.0) * field_rect.height();
    pos2(x, y)
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
    image_id: StereoImageId,
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CameraMatrixKey {
    time_nanos: i64,
    calibrated_intrinsics_time_nanos: Option<u128>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GlobalDebugKey {
    camera_matrix_key: CameraMatrixKey,
    detected_objects_time_nanos: Option<u128>,
    pose_revision: SceneVersion,
    global_localizer: GlobalLocalizerParameters,
}

#[derive(Default)]
struct CachedGlobalDebug {
    key: Option<GlobalDebugKey>,
    version: SceneVersion,
    debug: Option<Arc<GlobalLocalizationDetailedDebug>>,
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
