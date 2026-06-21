use std::{
    collections::{BTreeSet, VecDeque},
    num::NonZeroUsize,
    sync::Arc,
    time::Duration,
};

use color_eyre::{
    Result,
    eyre::{WrapErr as _, eyre},
};
use coordinate_systems::{Field, Robot};
use eframe::egui::Context as EguiContext;
use field_mark_association::FieldMarkAssociations;
use image::RgbImage;
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::Isometry3;
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{
    prelude::ContextBuilder,
    pubsub::PublicationId,
    qos::{QosDurability, QosProfile},
    time::Time,
};
use ros_z_debug::{RetentionPolicy, SubscriptionHandle, SubscriptionManager, SubscriptionStatus};
use ros_z_streams::{CreateFutureQueue, QueueEvent};
use ros2::sensor_msgs::image::Image as RosImage;
use tokio::runtime::Runtime;
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
    visual_odometry::VisualOdometer,
};

use crate::{
    cli::Arguments,
    state::{CameraFrame, ConnectionStatus, SharedState, StreamStatus, ViewerState},
};

pub(crate) const CAMERA_IMAGE_TOPIC: &str = "inputs/left_image";

const FIELD_DIMENSIONS_TOPIC: &str = "field_dimensions";
const LOCALIZATION_TOPIC: &str = "localization";
const VISUAL_ODOMETER_TOPIC: &str = "visual_odometry/current_left_camera_to_visual_odometer";
const ROBOT_KINEMATICS_TOPIC: &str = "robot_kinematics";
const CAMERA_MATRIX_TOPIC: &str = "camera_matrix";
const CALIBRATED_INTRINSICS_TOPIC: &str = "debug/calibrated_intrinsics";
// Detections are an announced stream: replays/simulators must provide both this base topic and
// `{DETECTED_OBJECTS_TOPIC}/announce` so the viewer can recover the original image timestamp.
const DETECTED_OBJECTS_TOPIC: &str = "detected_objects";
const FIELD_MARK_ASSOCIATIONS_TOPIC: &str = "field_mark_association/associations";
const DETECTED_OBJECTS_SAFETY_LAG: Duration = Duration::from_millis(50);
const DETECTED_OBJECTS_PENDING_CAPACITY: usize = 64;
const DEBUG_REFRESH_INTERVAL: Duration = Duration::from_millis(33);
const DEBUG_HISTORY_WINDOW: Duration = Duration::from_secs(2);
const DEBUG_HIGH_RATE_HISTORY_CAPACITY: usize = 1024;
const DEBUG_STREAM_HISTORY_CAPACITY: usize = 64;
const PROCESSED_PUBLICATION_CAPACITY: usize = 4096;

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

    let router_display = arguments.router_display();
    let mut builder = ContextBuilder::default().with_namespace(arguments.namespace());
    if let Some(router) = arguments.router.clone() {
        builder = builder.with_mode("client").with_connect_endpoints([router]);
    }

    let context = builder.build().await.wrap_err_with(|| {
        format!(
            "failed to connect to Zenoh router {router_display}; make sure zenohd is running and listening on that address, or use tcp/127.0.0.1:7447 when running on the robot or through an SSH port forward"
        )
    })?;
    let node = Arc::new(
        context
            .create_node("robot_viewer")
            .without_schema_service()
            .build()
            .await?,
    );
    let debug_manager = SubscriptionManager::new(
        Arc::clone(&node),
        ros_z_debug::ManagerOptions::with_target_namespace(arguments.namespace())?,
    );
    let mut debug_subscriptions = DebugSubscriptions::build(&debug_manager).await?;

    let camera = node
        .subscriber::<TimeWrapper<RosImage>>(CAMERA_IMAGE_TOPIC)?
        .build()
        .await?;
    let mut objects = node
        .create_bounded_future_subscriber::<Vec<Object<RobocupObjectLabel>>>(
            DETECTED_OBJECTS_TOPIC,
            DETECTED_OBJECTS_SAFETY_LAG,
            NonZeroUsize::new(DETECTED_OBJECTS_PENDING_CAPACITY).expect("capacity is non-zero"),
        )
        .await?;

    update_state(&state, &egui_context, |state| {
        state.connection = ConnectionStatus::Subscribed;
        update_publisher_count(&mut state.camera_status, camera.publisher_count());
        update_publisher_count(&mut state.objects_status, objects.publisher_count());
    });

    let mut refresh_interval = tokio::time::interval(DEBUG_REFRESH_INTERVAL);
    loop {
        tokio::select! {
            _ = refresh_interval.tick() => {
                update_state(&state, &egui_context, |state| {
                    refresh_debug_streams(state, &mut debug_subscriptions);
                    update_publisher_count(&mut state.camera_status, camera.publisher_count());
                    update_publisher_count(&mut state.objects_status, objects.publisher_count());
                });
            }
            message = camera.recv() => match message {
                Ok(image) => {
                    let time = image.time;
                    match decode_camera_frame(image.inner) {
                        Ok(frame) => update_state(&state, &egui_context, |state| {
                            state.camera_sequence += 1;
                            state.push_camera_frame(time, CameraFrame {
                                sequence: state.camera_sequence,
                                ..frame
                            });
                            state.camera_status.mark_live(camera.publisher_count());
                        }),
                        Err(error) => update_state(&state, &egui_context, |state| {
                            state.camera_status.mark_error(camera.publisher_count(), format!("{error:#}"));
                        }),
                    }
                }
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.camera_status.mark_error(camera.publisher_count(), format!("{error:#}"));
                }),
            },
            message = objects.recv() => match message {
                Ok(QueueEvent::Data(time, message)) => update_state(&state, &egui_context, |state| {
                    state.push_detected_objects(time, message);
                    state.objects_status.mark_live(objects.publisher_count());
                }),
                Ok(QueueEvent::Announcement) => {}
                Err(error) => update_state(&state, &egui_context, |state| {
                    state.objects_status.mark_error(objects.publisher_count(), format!("{error:#}"));
                }),
            },
        }
    }
}

struct DebugSubscriptions {
    field_dimensions: SubscriptionHandle<FieldDimensions>,
    localization: SubscriptionHandle<Option<Isometry3<Field, Robot>>>,
    visual_odometer: SubscriptionHandle<VisualOdometer>,
    robot_kinematics: WindowedDebugStream<RobotKinematics>,
    camera_matrix: WindowedDebugStream<CameraMatrix>,
    calibrated_intrinsics: SubscriptionHandle<Intrinsic>,
    field_mark_associations: WindowedDebugStream<FieldMarkAssociations>,
}

impl DebugSubscriptions {
    async fn build(manager: &SubscriptionManager) -> Result<Self> {
        let high_rate_history = RetentionPolicy::time_window_with_max_samples(
            DEBUG_HISTORY_WINDOW,
            NonZeroUsize::new(DEBUG_HIGH_RATE_HISTORY_CAPACITY).expect("capacity is non-zero"),
        )?;
        let stream_history = RetentionPolicy::time_window_with_max_samples(
            DEBUG_HISTORY_WINDOW,
            NonZeroUsize::new(DEBUG_STREAM_HISTORY_CAPACITY).expect("capacity is non-zero"),
        )?;

        Ok(Self {
            field_dimensions: manager
                .subscribe_typed::<FieldDimensions>(FIELD_DIMENSIONS_TOPIC)
                .qos(QosProfile {
                    durability: QosDurability::TransientLocal,
                    ..Default::default()
                })
                .build()
                .await?,
            localization: manager
                .subscribe_typed::<Option<Isometry3<Field, Robot>>>(LOCALIZATION_TOPIC)
                .build()
                .await?,
            visual_odometer: manager
                .subscribe_typed::<VisualOdometer>(VISUAL_ODOMETER_TOPIC)
                .with_stamp(|message| message.time)
                .build()
                .await?,
            robot_kinematics: WindowedDebugStream::new(
                manager
                    .subscribe_typed::<TimeWrapper<RobotKinematics>>(ROBOT_KINEMATICS_TOPIC)
                    .retention(high_rate_history)
                    .with_stamp(time_wrapper_stamp::<RobotKinematics>)
                    .build()
                    .await?,
            ),
            camera_matrix: WindowedDebugStream::new(
                manager
                    .subscribe_typed::<TimeWrapper<CameraMatrix>>(CAMERA_MATRIX_TOPIC)
                    .retention(high_rate_history)
                    .with_stamp(time_wrapper_stamp::<CameraMatrix>)
                    .build()
                    .await?,
            ),
            calibrated_intrinsics: manager
                .subscribe_typed::<Intrinsic>(CALIBRATED_INTRINSICS_TOPIC)
                .build()
                .await?,
            field_mark_associations: WindowedDebugStream::new(
                manager
                    .subscribe_typed::<TimeWrapper<FieldMarkAssociations>>(
                        FIELD_MARK_ASSOCIATIONS_TOPIC,
                    )
                    .retention(stream_history)
                    .with_stamp(time_wrapper_stamp::<FieldMarkAssociations>)
                    .build()
                    .await?,
            ),
        })
    }
}

fn time_wrapper_stamp<T>(message: &TimeWrapper<T>) -> Time {
    message.time
}

struct WindowedDebugStream<T> {
    handle: SubscriptionHandle<TimeWrapper<T>>,
    processed: ProcessedPublications,
}

impl<T> WindowedDebugStream<T> {
    fn new(handle: SubscriptionHandle<TimeWrapper<T>>) -> Self {
        Self {
            handle,
            processed: ProcessedPublications::default(),
        }
    }

    fn handle(&self) -> &SubscriptionHandle<TimeWrapper<T>> {
        &self.handle
    }

    fn latest(&self) -> Option<Arc<ros_z_debug::SampleRecord<TimeWrapper<T>>>> {
        self.handle.latest()
    }

    fn drain_new(&mut self) -> Vec<Arc<ros_z_debug::SampleRecord<TimeWrapper<T>>>> {
        self.handle
            .window(Time::zero(), Time::from_nanos(i64::MAX))
            .into_iter()
            .filter(|record| self.processed.accept(record.publication_id))
            .collect()
    }
}

struct ProcessedPublications {
    seen: BTreeSet<PublicationId>,
    order: VecDeque<PublicationId>,
}

impl Default for ProcessedPublications {
    fn default() -> Self {
        Self {
            seen: BTreeSet::new(),
            order: VecDeque::with_capacity(PROCESSED_PUBLICATION_CAPACITY),
        }
    }
}

impl ProcessedPublications {
    fn accept(&mut self, publication_id: PublicationId) -> bool {
        if !self.seen.insert(publication_id) {
            return false;
        }

        self.order.push_back(publication_id);
        while self.order.len() > PROCESSED_PUBLICATION_CAPACITY {
            if let Some(oldest) = self.order.pop_front() {
                self.seen.remove(&oldest);
            }
        }

        true
    }
}

fn refresh_debug_streams(state: &mut ViewerState, subscriptions: &mut DebugSubscriptions) {
    if let Some(record) = subscriptions.field_dimensions.latest() {
        state.field_dimensions = Some(record.value.clone());
    }
    update_debug_status(
        &mut state.field_status,
        &subscriptions.field_dimensions,
        state.field_dimensions.is_some(),
    );

    if let Some(record) = subscriptions.localization.latest() {
        state.localization = record.value.clone();
    }
    update_debug_status(
        &mut state.localization_status,
        &subscriptions.localization,
        state.localization.is_some(),
    );

    if let Some(record) = subscriptions.visual_odometer.latest() {
        state.visual_odometer = Some(record.value.current_left_camera_to_visual_odometer);
    }
    update_debug_status(
        &mut state.visual_odometer_status,
        &subscriptions.visual_odometer,
        state.visual_odometer.is_some(),
    );

    for record in subscriptions.robot_kinematics.drain_new() {
        state.push_robot_kinematics(record.source_time, record.value.inner.clone());
    }
    update_debug_status(
        &mut state.robot_kinematics_status,
        subscriptions.robot_kinematics.handle(),
        subscriptions.robot_kinematics.latest().is_some(),
    );

    for record in subscriptions.camera_matrix.drain_new() {
        state.push_camera_matrix(record.source_time, record.value.inner.clone());
    }
    update_debug_status(
        &mut state.camera_matrix_status,
        subscriptions.camera_matrix.handle(),
        subscriptions.camera_matrix.latest().is_some(),
    );

    if let Some(record) = subscriptions.calibrated_intrinsics.latest() {
        state.calibrated_intrinsics = Some(record.value.clone());
    }
    update_debug_status(
        &mut state.calibrated_intrinsics_status,
        &subscriptions.calibrated_intrinsics,
        state.calibrated_intrinsics.is_some(),
    );

    for record in subscriptions.field_mark_associations.drain_new() {
        state.push_field_mark_associations(record.source_time, record.value.inner.clone());
    }
    update_debug_status(
        &mut state.field_mark_associations_status,
        subscriptions.field_mark_associations.handle(),
        subscriptions.field_mark_associations.latest().is_some(),
    );
}

fn update_debug_status<T>(
    status: &mut StreamStatus,
    subscription: &SubscriptionHandle<T>,
    has_value: bool,
) {
    let publisher_count = subscription.publisher_count();
    let snapshot = subscription.status();
    match snapshot.status() {
        SubscriptionStatus::WaitingForFirstSample => status.update_publishers(publisher_count),
        SubscriptionStatus::Ready => status.mark_value(publisher_count, has_value),
        SubscriptionStatus::ProtocolError { .. } | SubscriptionStatus::DecodeError { .. } => {
            status.mark_error(
                publisher_count,
                snapshot
                    .message()
                    .unwrap_or("subscription error")
                    .to_string(),
            );
        }
        SubscriptionStatus::Closed => {
            status.mark_error(publisher_count, "subscription closed".to_string());
        }
        _ => status.update_publishers(publisher_count),
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
