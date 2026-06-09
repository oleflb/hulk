use std::sync::{Arc, Mutex};

use eframe::{
    App, CreationContext, Frame,
    egui::{
        self, CentralPanel, Color32, ColorImage, Context, FontId, Rect, RichText, Sense, SidePanel,
        Stroke, StrokeKind, TextureHandle, TextureOptions, TopBottomPanel, Ui, Vec2, Widget, pos2,
        vec2,
    },
};
use egui_bevy::BevyWidget;
use tokio::runtime::Runtime;
use types::object_detection::{Object, RobocupObjectLabel};

use crate::{
    cli::Arguments,
    scene::{self, ViewerData},
    state::{CameraFrame, ConnectionStatus, SharedState, StreamState, StreamStatus, ViewerState},
    subscriptions,
};

pub(crate) struct RobotViewerApp {
    widget: BevyWidget,
    state: SharedState,
    namespace: String,
    router: String,
    camera_texture: Option<TextureHandle>,
    camera_texture_sequence: u64,
    _runtime: Arc<Runtime>,
}

impl RobotViewerApp {
    pub(crate) fn new(
        creation_context: &CreationContext,
        arguments: Arguments,
        runtime: Arc<Runtime>,
    ) -> Self {
        creation_context.egui_ctx.set_visuals(egui::Visuals::dark());

        let namespace = arguments.namespace();
        let router = arguments.router_display();
        let state = Arc::new(Mutex::new(ViewerState::default()));

        let mut widget = BevyWidget::new(
            creation_context
                .wgpu_render_state
                .clone()
                .expect("no wgpu render state found"),
        );
        scene::configure(&mut widget.bevy_app);
        widget.bevy_app.finish();
        widget.bevy_app.cleanup();

        subscriptions::spawn(
            &runtime,
            arguments,
            state.clone(),
            creation_context.egui_ctx.clone(),
        );

        Self {
            widget,
            state,
            namespace,
            router,
            camera_texture: None,
            camera_texture_sequence: 0,
            _runtime: runtime,
        }
    }
}

impl App for RobotViewerApp {
    fn update(&mut self, context: &Context, _frame: &mut Frame) {
        let snapshot = self
            .state
            .lock()
            .expect("viewer state lock should not be poisoned")
            .clone();

        self.widget
            .bevy_app
            .world_mut()
            .insert_resource(ViewerData::from_state(&snapshot));

        self.update_camera_texture(context, snapshot.camera_frame.as_ref());
        self.header(context, &snapshot);
        self.camera_panel(context, &snapshot);
        self.viewport(context);
    }
}

impl RobotViewerApp {
    fn update_camera_texture(&mut self, context: &Context, frame: Option<&CameraFrame>) {
        let Some(frame) = frame else {
            return;
        };
        if self.camera_texture_sequence == frame.sequence {
            return;
        }

        let image = ColorImage::from_rgba_unmultiplied(
            [frame.width as usize, frame.height as usize],
            &frame.rgba,
        );
        if let Some(texture) = &mut self.camera_texture {
            texture.set(image, TextureOptions::LINEAR);
        } else {
            self.camera_texture =
                Some(context.load_texture("robot_viewer_camera", image, TextureOptions::LINEAR));
        }
        self.camera_texture_sequence = frame.sequence;
    }

    fn header(&self, context: &Context, state: &ViewerState) {
        TopBottomPanel::top("header")
            .min_height(112.0)
            .show(context, |ui| {
                ui.vertical_centered(|ui| {
                    ui.add_space(8.0);
                    ui.horizontal_wrapped(|ui| {
                        ui.heading(RichText::new("Robot Viewer").strong());
                        ui.separator();
                        ui.label(
                            RichText::new(format!("namespace {}", self.namespace)).monospace(),
                        );
                        ui.separator();
                        ui.label(RichText::new(format!("router {}", self.router)).monospace());
                        ui.separator();
                        connection_status(ui, &state.connection);
                    });
                    ui.add_space(4.0);
                    ui.horizontal_wrapped(|ui| {
                        stream_status(ui, "visual odometry", &state.visual_odometry_status);
                        stream_status(
                            ui,
                            "triangulated features",
                            &state.triangulated_features_status,
                        );
                        stream_status(ui, "field", &state.field_status);
                        stream_status(ui, "kinematics", &state.robot_kinematics_status);
                        stream_status(ui, "camera matrix", &state.camera_matrix_status);
                        stream_status(
                            ui,
                            "calibrated intrinsics",
                            &state.calibrated_intrinsics_status,
                        );
                        stream_status(ui, "camera", &state.camera_status);
                        stream_status(ui, "objects", &state.objects_status);
                    });
                    ui.add_space(4.0);
                    ui.horizontal_wrapped(|ui| {
                        let mut use_visual_odometry = state.use_visual_odometry;
                        if ui
                            .checkbox(&mut use_visual_odometry, "use visual odometry")
                            .changed()
                        {
                            self.state
                                .lock()
                                .expect("viewer state lock should not be poisoned")
                                .use_visual_odometry = use_visual_odometry;
                        }
                        ui.separator();
                        ui.label(format!("{} features", state.triangulated_features.len()));
                        ui.separator();
                        ui.label(RichText::new(pose_summary(state)).monospace());
                    });
                });
            });
    }

    fn camera_panel(&mut self, context: &Context, state: &ViewerState) {
        SidePanel::right("camera_panel")
            .resizable(true)
            .default_width(440.0)
            .width_range(300.0..=900.0)
            .show(context, |ui| {
                ui.vertical(|ui| {
                    ui.heading("Camera");
                    ui.label(
                        RichText::new(subscriptions::CAMERA_IMAGE_TOPIC)
                            .monospace()
                            .color(Color32::GRAY),
                    );
                    ui.add_space(8.0);
                    self.camera_image(ui, state);
                });
            });
    }

    fn camera_image(&mut self, ui: &mut Ui, state: &ViewerState) {
        let Some(frame) = &state.camera_frame else {
            ui.centered_and_justified(|ui| {
                ui.label(RichText::new("waiting for camera image").color(Color32::GRAY));
            });
            return;
        };
        let Some(texture) = &self.camera_texture else {
            return;
        };

        let image_size = vec2(frame.width as f32, frame.height as f32);
        let available = ui.available_size().max(vec2(1.0, 1.0));
        let scale = (available.x / image_size.x)
            .min((available.y - 72.0).max(1.0) / image_size.y)
            .max(0.05);

        let response = ui.add(
            egui::Image::new((texture.id(), texture.size_vec2()))
                .fit_to_exact_size(image_size * scale)
                .sense(Sense::hover()),
        );
        draw_detected_objects(ui, response.rect, image_size, &state.detected_objects);

        ui.add_space(8.0);
        ui.horizontal_wrapped(|ui| {
            ui.label(format!("{}x{}", frame.width, frame.height));
            ui.separator();
            ui.label(format!("{} detections", state.detected_objects.len()));
            if let Some(intrinsics) = state.calibrated_intrinsics {
                ui.separator();
                ui.label(format!(
                    "calibrated fx/fy {:.1}/{:.1} cx/cy {:.1}/{:.1}",
                    intrinsics.focals.x,
                    intrinsics.focals.y,
                    intrinsics.optical_center.x(),
                    intrinsics.optical_center.y(),
                ));
            }
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
                    ui.add_space(6.0);
                    self.widget.ui(ui);
                });
            });
    }
}

fn connection_status(ui: &mut Ui, status: &ConnectionStatus) {
    let (text, color) = match status {
        ConnectionStatus::Starting => ("starting".to_string(), Color32::GRAY),
        ConnectionStatus::Connecting => ("connecting".to_string(), Color32::YELLOW),
        ConnectionStatus::Subscribed => ("connected".to_string(), Color32::LIGHT_GREEN),
        ConnectionStatus::Error(error) => (format!("error: {error}"), Color32::LIGHT_RED),
    };
    ui.colored_label(color, RichText::new(text).strong());
}

fn stream_status(ui: &mut Ui, name: &str, status: &StreamStatus) {
    let (label, color) = match status.state {
        StreamState::Waiting => ("waiting", Color32::GRAY),
        StreamState::Matched => ("matched", Color32::YELLOW),
        StreamState::Live => ("live", Color32::LIGHT_GREEN),
        StreamState::Empty => ("empty", Color32::YELLOW),
        StreamState::Error => ("error", Color32::LIGHT_RED),
    };
    let response = ui.colored_label(
        color,
        format!("{name}: {label} ({} pubs)", status.publisher_count),
    );
    if let Some(detail) = &status.detail {
        response.on_hover_text(detail);
    }
}

fn pose_summary(state: &ViewerState) -> String {
    let pose = if state.use_visual_odometry {
        state.visual_odometer_pose
    } else {
        nalgebra::Isometry3::identity()
    };
    let translation = pose.translation.vector;
    let (roll, pitch, yaw) = pose.rotation.euler_angles();
    format!(
        "VO pose x/y/z {:.3}/{:.3}/{:.3} r/p/y {:.1}/{:.1}/{:.1} deg",
        translation.x,
        translation.y,
        translation.z,
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees(),
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
