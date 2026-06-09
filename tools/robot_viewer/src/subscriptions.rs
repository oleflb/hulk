use color_eyre::{Result, eyre::eyre};
use eframe::egui::Context as EguiContext;
use image::RgbImage;
use kinematics::robot_kinematics::RobotKinematics;
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{
    prelude::ContextBuilder,
    qos::{QosDurability, QosProfile},
};
use ros2::sensor_msgs::image::Image as RosImage;
use tokio::runtime::Runtime;
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
    visual_odometry::TriangulatedFeature,
};

use crate::{
    cli::Arguments,
    state::{CameraFrame, ConnectionStatus, SharedState, StreamStatus, ViewerState},
};

pub(crate) const CAMERA_IMAGE_TOPIC: &str = "inputs/left_image";
pub(crate) const VISUAL_ODOMETRY_TOPIC: &str =
    "visual_odometry/current_left_camera_to_visual_odometer";
pub(crate) const TRIANGULATED_FEATURES_TOPIC: &str = "visual_odometry/triangulated_features";

const FIELD_DIMENSIONS_TOPIC: &str = "field_dimensions";
const ROBOT_KINEMATICS_TOPIC: &str = "robot_kinematics";
const CAMERA_MATRIX_TOPIC: &str = "camera_matrix";
const CALIBRATED_INTRINSICS_TOPIC: &str = "debug/calibrated_intrinsics";
const DETECTED_OBJECTS_TOPIC: &str = "detected_objects";

pub(crate) fn spawn(
    runtime: &Runtime,
    arguments: Arguments,
    state: SharedState,
    egui_context: EguiContext,
) {
    runtime.spawn(async move {
        if let Err(error) = run(arguments, state.clone(), egui_context.clone()).await {
            update_state(&state, &egui_context, |state| {
                state.connection = ConnectionStatus::Error(format!("{error:#}"));
            });
        }
    });
}

async fn run(arguments: Arguments, state: SharedState, egui_context: EguiContext) -> Result<()> {
    update_state(&state, &egui_context, |state| {
        state.connection = ConnectionStatus::Connecting;
    });

    let mut builder = ContextBuilder::default().with_namespace(arguments.namespace());
    if let Some(router) = arguments.router {
        builder = builder.with_router_endpoint(router)?;
    }

    let context = builder.build().await?;
    let node = context
        .create_node("robot_viewer")
        .without_schema_service()
        .build()
        .await?;

    let field_dimensions = node
        .subscriber::<FieldDimensions>(FIELD_DIMENSIONS_TOPIC)?
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;
    let visual_odometry = node
        .subscriber::<nalgebra::Isometry3<f32>>(VISUAL_ODOMETRY_TOPIC)?
        .build()
        .await?;
    let triangulated_features = node
        .subscriber::<Vec<TriangulatedFeature>>(TRIANGULATED_FEATURES_TOPIC)?
        .build()
        .await?;
    let robot_kinematics = node
        .subscriber::<TimeWrapper<RobotKinematics>>(ROBOT_KINEMATICS_TOPIC)?
        .build()
        .await?;
    let camera_matrix = node
        .subscriber::<TimeWrapper<CameraMatrix>>(CAMERA_MATRIX_TOPIC)?
        .build()
        .await?;
    let calibrated_intrinsics = node
        .subscriber::<Intrinsic>(CALIBRATED_INTRINSICS_TOPIC)?
        .build()
        .await?;
    let camera = node
        .subscriber::<RosImage>(CAMERA_IMAGE_TOPIC)?
        .build()
        .await?;
    let objects = node
        .subscriber::<Vec<Object<RobocupObjectLabel>>>(DETECTED_OBJECTS_TOPIC)?
        .build()
        .await?;

    update_state(&state, &egui_context, |state| {
        state.connection = ConnectionStatus::Subscribed;
        update_publisher_count(&mut state.field_status, field_dimensions.publisher_count());
        update_publisher_count(
            &mut state.visual_odometry_status,
            visual_odometry.publisher_count(),
        );
        update_publisher_count(
            &mut state.triangulated_features_status,
            triangulated_features.publisher_count(),
        );
        update_publisher_count(
            &mut state.robot_kinematics_status,
            robot_kinematics.publisher_count(),
        );
        update_publisher_count(
            &mut state.camera_matrix_status,
            camera_matrix.publisher_count(),
        );
        update_publisher_count(
            &mut state.calibrated_intrinsics_status,
            calibrated_intrinsics.publisher_count(),
        );
        update_publisher_count(&mut state.camera_status, camera.publisher_count());
        update_publisher_count(&mut state.objects_status, objects.publisher_count());
    });

    let mut publisher_count_interval = tokio::time::interval(std::time::Duration::from_millis(500));
    loop {
        tokio::select! {
            _ = publisher_count_interval.tick() => {
                update_state(&state, &egui_context, |state| {
                    update_publisher_count(&mut state.field_status, field_dimensions.publisher_count());
                    update_publisher_count(&mut state.visual_odometry_status, visual_odometry.publisher_count());
                    update_publisher_count(&mut state.triangulated_features_status, triangulated_features.publisher_count());
                    update_publisher_count(&mut state.robot_kinematics_status, robot_kinematics.publisher_count());
                    update_publisher_count(&mut state.camera_matrix_status, camera_matrix.publisher_count());
                    update_publisher_count(&mut state.calibrated_intrinsics_status, calibrated_intrinsics.publisher_count());
                    update_publisher_count(&mut state.camera_status, camera.publisher_count());
                    update_publisher_count(&mut state.objects_status, objects.publisher_count());
                });
            }
            message = field_dimensions.recv() => match message {
                Ok(message) => update_state(&state, &egui_context, |state| {
                    state.field_dimensions = Some(message);
                    state.field_status.mark_live(field_dimensions.publisher_count());
                }),
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.field_status.mark_error(field_dimensions.publisher_count(), format!("{error:#}"));
                }),
            },
            message = visual_odometry.recv() => match message {
                Ok(message) => update_state(&state, &egui_context, |state| {
                    state.visual_odometer_pose = message;
                    state.visual_odometry_status.mark_live(visual_odometry.publisher_count());
                }),
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.visual_odometry_status.mark_error(visual_odometry.publisher_count(), format!("{error:#}"));
                }),
            },
            message = triangulated_features.recv() => match message {
                Ok(message) => update_state(&state, &egui_context, |state| {
                    let has_features = !message.is_empty();
                    state.triangulated_features = message;
                    state.triangulated_features_status.mark_value(triangulated_features.publisher_count(), has_features);
                }),
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.triangulated_features_status.mark_error(triangulated_features.publisher_count(), format!("{error:#}"));
                }),
            },
            message = robot_kinematics.recv() => match message {
                Ok(message) => update_state(&state, &egui_context, |state| {
                    state.robot_kinematics = Some(message.inner);
                    state.robot_kinematics_status.mark_live(robot_kinematics.publisher_count());
                }),
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.robot_kinematics_status.mark_error(robot_kinematics.publisher_count(), format!("{error:#}"));
                }),
            },
            message = camera_matrix.recv() => match message {
                Ok(message) => update_state(&state, &egui_context, |state| {
                    if state.initial_camera_to_robot.is_none() {
                        state.initial_camera_to_robot = Some(camera_to_robot(&message.inner));
                    }
                    state.camera_matrix = Some(message.inner);
                    state.camera_matrix_status.mark_live(camera_matrix.publisher_count());
                }),
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.camera_matrix_status.mark_error(camera_matrix.publisher_count(), format!("{error:#}"));
                }),
            },
            message = calibrated_intrinsics.recv() => match message {
                Ok(message) => update_state(&state, &egui_context, |state| {
                    state.calibrated_intrinsics = Some(message);
                    state.calibrated_intrinsics_status.mark_live(calibrated_intrinsics.publisher_count());
                }),
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.calibrated_intrinsics_status.mark_error(calibrated_intrinsics.publisher_count(), format!("{error:#}"));
                }),
            },
            message = camera.recv() => match message {
                Ok(image) => match decode_camera_frame(image) {
                    Ok(frame) => update_state(&state, &egui_context, |state| {
                        state.camera_sequence += 1;
                        state.camera_frame = Some(CameraFrame {
                            sequence: state.camera_sequence,
                            ..frame
                        });
                        state.camera_status.mark_live(camera.publisher_count());
                    }),
                    Err(error) => update_state(&state, &egui_context, |state| {
                        state.camera_status.mark_error(camera.publisher_count(), format!("{error:#}"));
                    }),
                },
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.camera_status.mark_error(camera.publisher_count(), format!("{error:#}"));
                }),
            },
            message = objects.recv() => match message {
                Ok(message) => update_state(&state, &egui_context, |state| {
                    state.detected_objects = message;
                    state.objects_status.mark_live(objects.publisher_count());
                }),
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.objects_status.mark_error(objects.publisher_count(), format!("{error:#}"));
                }),
            },
        }
    }
}

fn update_publisher_count(status: &mut StreamStatus, publisher_count: usize) {
    status.update_publishers(publisher_count);
}

fn decode_camera_frame(image: RosImage) -> Result<CameraFrame> {
    let rgb_image: RgbImage = image
        .try_into()
        .map_err(|error| eyre!("failed to decode camera image: {error}"))?;
    let width = rgb_image.width();
    let height = rgb_image.height();
    let rgba = rgb_image
        .into_vec()
        .chunks_exact(3)
        .flat_map(|pixel| [pixel[0], pixel[1], pixel[2], 255])
        .collect();

    Ok(CameraFrame {
        sequence: 0,
        width,
        height,
        rgba,
    })
}

fn camera_to_robot(camera_matrix: &CameraMatrix) -> nalgebra::Isometry3<f32> {
    (camera_matrix.head_to_camera * camera_matrix.robot_to_head)
        .inverse()
        .inner
}

fn update_state(
    state: &SharedState,
    egui_context: &EguiContext,
    update: impl FnOnce(&mut ViewerState),
) {
    update(
        &mut state
            .lock()
            .expect("viewer state lock should not be poisoned"),
    );
    egui_context.request_repaint();
}
