use std::{
    collections::BTreeMap,
    fmt, fs,
    path::Path,
    sync::Arc,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use booster::ImuState;
use color_eyre::{Result, eyre::Context as _};
use coordinate_systems::{Field, Pixel, Robot};
use image::RgbImage;
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::{IntoTransform, Isometry3, vector};
use mcap::{Message, MessageStream};
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::time::Time;
use ros_z_cdr::{LittleEndian, from_bytes};
use ros2::sensor_msgs::image::Image as RosImage;
use serde::{Deserialize, de::DeserializeOwned};
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
    stereo_image_pair::StereoImagePair,
    time_wrapper::TimeWrapper,
    visual_odometry::VisualOdometryDelta,
};

use crate::nearest_by_distance;

pub const TOPIC_IMU_STATE: &str = "inputs/imu_state";
pub const TOPIC_STEREO_IMAGE_PAIR: &str = "inputs/stereo_image_pair";
pub const TOPIC_ROBOT_KINEMATICS: &str = "robot_kinematics";
pub const TOPIC_CAMERA_MATRIX: &str = "camera_matrix";
pub const TOPIC_DETECTED_OBJECTS: &str = "detected_objects";
pub const TOPIC_FIELD_DIMENSIONS: &str = "field_dimensions";
pub const TOPIC_LOCALIZATION: &str = "localization";
pub const TOPIC_VISUAL_ODOMETRY: &str =
    "visual_odometry/current_left_camera_to_previous_left_camera";
pub const TOPIC_CALIBRATED_INTRINSICS: &str = "debug/calibrated_intrinsics";

pub struct Recording {
    events: Vec<RecordedEvent>,
    images: Vec<StereoImageIndex>,
    images_by_embedded_time: Vec<usize>,
    snapshot_index: SnapshotIndex,
    pub first_camera_matrix: CameraMatrix,
    pub field_dimensions: Option<FieldDimensions>,
    topic_counts: BTreeMap<String, usize>,
}

impl Recording {
    pub fn load(path: &Path) -> Result<Self> {
        let bytes: Arc<[u8]> = fs::read(path)
            .wrap_err_with(|| format!("failed to read {}", path.display()))?
            .into();
        let mut events = Vec::new();
        let mut images = Vec::new();
        let mut first_camera_matrix = None;
        let mut field_dimensions = None;
        let mut topic_counts = BTreeMap::new();

        for (order, message) in MessageStream::new(&bytes)
            .wrap_err("failed to open MCAP message stream")?
            .enumerate()
        {
            let message = message.wrap_err("failed to read MCAP message")?;
            *topic_counts
                .entry(message.channel.topic.clone())
                .or_default() += 1;

            let log_time = system_time_from_nanos(message.log_time);
            let publish_time = system_time_from_nanos(message.publish_time);
            let kind = match message.channel.topic.as_str() {
                TOPIC_IMU_STATE => Some(EventKind::Imu(decode_recorded_message(&message)?)),
                TOPIC_VISUAL_ODOMETRY => Some(EventKind::VisualOdometry(
                    decode_recorded_visual_odometry(&message)?,
                )),
                TOPIC_ROBOT_KINEMATICS => Some(EventKind::RobotKinematics(Box::new(
                    decode_recorded_message(&message)?,
                ))),
                TOPIC_CAMERA_MATRIX => {
                    let camera_matrix = decode_recorded_camera_matrix(&message)?;
                    if first_camera_matrix.is_none() {
                        first_camera_matrix = Some(camera_matrix.inner.clone());
                    }
                    Some(EventKind::CameraMatrix(camera_matrix))
                }
                TOPIC_DETECTED_OBJECTS => Some(EventKind::DetectedObjects(
                    decode_recorded_message(&message)?,
                )),
                TOPIC_FIELD_DIMENSIONS => {
                    let dimensions = decode_recorded_message(&message)?;
                    field_dimensions = Some(dimensions);
                    None
                }
                TOPIC_LOCALIZATION => Some(EventKind::RecordedLocalization(
                    decode_recorded_message(&message)?,
                )),
                TOPIC_CALIBRATED_INTRINSICS => Some(EventKind::CalibratedIntrinsics(
                    decode_recorded_message(&message)?,
                )),
                TOPIC_STEREO_IMAGE_PAIR => {
                    let data: Arc<[u8]> = message.data.into_owned().into();
                    let embedded_time = decode_time_prefix(&data).wrap_err_with(|| {
                        format!(
                            "failed to decode {TOPIC_STEREO_IMAGE_PAIR} time prefix order {order} sequence {}",
                            message.sequence
                        )
                    })?;
                    images.push(StereoImageIndex {
                        order,
                        log_time,
                        publish_time,
                        embedded_time,
                        data,
                    });
                    None
                }
                _ => None,
            };

            if let Some(kind) = kind {
                events.push(RecordedEvent {
                    order,
                    log_time,
                    publish_time,
                    kind,
                });
            }
        }

        events.sort_by_key(|event| (nanos_since_epoch(event.log_time), event.order));
        let snapshot_index = SnapshotIndex::new(&events);
        images.sort_by_key(|image| (nanos_since_epoch(image.log_time), image.order));
        let mut images_by_embedded_time = (0..images.len()).collect::<Vec<_>>();
        images_by_embedded_time
            .sort_by_key(|&index| (images[index].embedded_time.as_nanos(), images[index].order));

        Ok(Self {
            events,
            images,
            images_by_embedded_time,
            snapshot_index,
            first_camera_matrix: first_camera_matrix
                .ok_or_else(|| color_eyre::eyre::eyre!("recording has no camera_matrix topic"))?,
            field_dimensions,
            topic_counts,
        })
    }

    pub fn start_log_time(&self) -> SystemTime {
        match self
            .events
            .first()
            .map(|event| event.log_time)
            .or_else(|| self.images.first().map(|image| image.log_time))
        {
            Some(time) => time,
            None => UNIX_EPOCH,
        }
    }

    pub fn start_source_time(&self) -> SystemTime {
        match self
            .events
            .first()
            .map(|event| event.publish_time)
            .or_else(|| self.images.first().map(|image| image.publish_time))
        {
            Some(time) => time,
            None => UNIX_EPOCH,
        }
    }

    pub fn end_log_time(&self) -> SystemTime {
        match self
            .events
            .last()
            .map(|event| event.log_time)
            .into_iter()
            .chain(self.images.last().map(|image| image.log_time))
            .max()
        {
            Some(time) => time,
            None => self.start_log_time(),
        }
    }

    pub fn duration(&self) -> Duration {
        match self.end_log_time().duration_since(self.start_log_time()) {
            Ok(duration) => duration,
            Err(_) => Duration::ZERO,
        }
    }

    pub fn events(&self) -> &[RecordedEvent] {
        &self.events
    }

    pub fn event_count(&self) -> usize {
        self.events.len()
    }

    pub fn image_count(&self) -> usize {
        self.images.len()
    }

    pub fn image_id_from_index(&self, index: usize) -> Option<StereoImageId> {
        if index < self.images.len() {
            Some(StereoImageId(index))
        } else {
            None
        }
    }

    pub fn image_log_time(&self, image_id: StereoImageId) -> Option<SystemTime> {
        self.images
            .get(image_id.index())
            .map(|image| image.log_time)
    }

    pub fn topic_count(&self) -> usize {
        self.topic_counts.len()
    }

    pub fn log_time_at_seconds(&self, seconds: f64) -> SystemTime {
        self.start_log_time() + Duration::from_secs_f64(seconds.max(0.0))
    }

    pub fn seconds_since_start(&self, time: SystemTime) -> f64 {
        seconds_since(time, self.start_log_time())
    }

    pub fn aligned_image_time(&self, embedded_time: Time) -> Option<SystemTime> {
        let target_nanos = embedded_time.as_nanos();
        let next = self
            .images_by_embedded_time
            .partition_point(|&index| self.images[index].embedded_time.as_nanos() <= target_nanos);
        nearest_by_distance(
            next.checked_sub(1)
                .and_then(|index| self.images_by_embedded_time.get(index))
                .and_then(|&index| self.images.get(index))
                .map(|image| {
                    (
                        image,
                        u128::from(image.embedded_time.as_nanos().abs_diff(target_nanos)),
                    )
                }),
            self.images_by_embedded_time
                .get(next)
                .and_then(|&index| self.images.get(index))
                .map(|image| {
                    (
                        image,
                        u128::from(image.embedded_time.as_nanos().abs_diff(target_nanos)),
                    )
                }),
        )
        .map(|candidate| candidate.publish_time)
    }

    pub fn latest_snapshot(&self, log_time: SystemTime) -> RecordingSnapshot {
        let image_id = self.nearest_image_id(log_time);
        let snapshot_time = match image_id.and_then(|image_id| self.images.get(image_id.index())) {
            Some(image) => image.log_time,
            None => log_time,
        };
        let mut snapshot = RecordingSnapshot {
            image_id,
            ..Default::default()
        };

        if let Some(EventKind::CameraMatrix(camera_matrix)) = self
            .snapshot_index
            .latest_camera_matrix(&self.events, snapshot_time)
            .map(|event| &event.kind)
        {
            snapshot.camera_matrix = Some(camera_matrix.clone());
        }
        if let Some(event) = self
            .snapshot_index
            .nearest_detected_objects(&self.events, snapshot_time)
            && let EventKind::DetectedObjects(objects) = &event.kind
        {
            snapshot.detected_objects = objects.clone();
            snapshot.detected_objects_time = Some(event.log_time);
        }
        if let Some(EventKind::RecordedLocalization(localization)) = self
            .snapshot_index
            .latest_recorded_localization(&self.events, snapshot_time)
            .map(|event| &event.kind)
        {
            snapshot.recorded_localization = *localization;
        }
        if let Some(event) = self
            .snapshot_index
            .latest_calibrated_intrinsics(&self.events, snapshot_time)
            && let EventKind::CalibratedIntrinsics(intrinsics) = &event.kind
        {
            snapshot.calibrated_intrinsics = Some(*intrinsics);
            snapshot.calibrated_intrinsics_time = Some(event.publish_time);
        }

        snapshot
    }

    pub fn recorded_localization_trajectory(&self) -> Vec<TrajectoryPoint> {
        self.events
            .iter()
            .filter_map(|event| match &event.kind {
                EventKind::RecordedLocalization(Some(field_to_robot)) => Some(TrajectoryPoint {
                    seconds: self.seconds_since_start(event.log_time),
                    robot_to_field: field_to_robot.inverse().inner.cast().framed_transform(),
                }),
                _ => None,
            })
            .collect()
    }

    pub fn decode_stereo_image(&self, image_id: StereoImageId) -> Result<StereoFrame> {
        let Some(index) = self.images.get(image_id.index()) else {
            return Err(color_eyre::eyre::eyre!(
                "image id {image_id} is out of bounds"
            ));
        };

        let stereo: TimeWrapper<StereoImagePair> =
            decode_message(&index.data).wrap_err_with(|| {
                format!(
                    "failed to decode {TOPIC_STEREO_IMAGE_PAIR} image id {image_id} order {}",
                    index.order
                )
            })?;
        Ok(StereoFrame {
            sequence: index.order as u64,
            source_time: stereo.time,
            log_time: index.log_time,
            publish_time: index.publish_time,
            left: camera_image_from_ros(stereo.inner.left)?,
            right: camera_image_from_ros(stereo.inner.right)?,
        })
    }

    fn nearest_image_id(&self, log_time: SystemTime) -> Option<StereoImageId> {
        if self.images.is_empty() {
            return None;
        }
        let next = self
            .images
            .partition_point(|image| image.log_time <= log_time);
        nearest_by_distance(
            next.checked_sub(1).map(|previous| {
                (
                    previous,
                    nanos_abs_diff(self.images[previous].log_time, log_time),
                )
            }),
            self.images
                .get(next)
                .map(|next_image| (next, nanos_abs_diff(next_image.log_time, log_time))),
        )
        .map(StereoImageId)
    }
}

#[derive(Clone)]
pub struct RecordedEvent {
    pub order: usize,
    pub log_time: SystemTime,
    pub publish_time: SystemTime,
    pub kind: EventKind,
}

#[derive(Default)]
struct SnapshotIndex {
    camera_matrices: Vec<usize>,
    detected_objects: Vec<usize>,
    recorded_localizations: Vec<usize>,
    calibrated_intrinsics: Vec<usize>,
}

impl SnapshotIndex {
    fn new(events: &[RecordedEvent]) -> Self {
        let mut index = Self::default();
        for (event_index, event) in events.iter().enumerate() {
            match event.kind {
                EventKind::CameraMatrix(_) => index.camera_matrices.push(event_index),
                EventKind::DetectedObjects(_) => index.detected_objects.push(event_index),
                EventKind::RecordedLocalization(_) => {
                    index.recorded_localizations.push(event_index)
                }
                EventKind::CalibratedIntrinsics(_) => index.calibrated_intrinsics.push(event_index),
                EventKind::Imu(_)
                | EventKind::VisualOdometry(_)
                | EventKind::RobotKinematics(_) => {}
            }
        }
        index
    }

    fn latest_camera_matrix<'a>(
        &self,
        events: &'a [RecordedEvent],
        log_time: SystemTime,
    ) -> Option<&'a RecordedEvent> {
        Self::latest(events, &self.camera_matrices, log_time)
    }

    fn nearest_detected_objects<'a>(
        &self,
        events: &'a [RecordedEvent],
        log_time: SystemTime,
    ) -> Option<&'a RecordedEvent> {
        Self::nearest(events, &self.detected_objects, log_time)
    }

    fn latest_recorded_localization<'a>(
        &self,
        events: &'a [RecordedEvent],
        log_time: SystemTime,
    ) -> Option<&'a RecordedEvent> {
        Self::latest(events, &self.recorded_localizations, log_time)
    }

    fn latest_calibrated_intrinsics<'a>(
        &self,
        events: &'a [RecordedEvent],
        log_time: SystemTime,
    ) -> Option<&'a RecordedEvent> {
        Self::latest(events, &self.calibrated_intrinsics, log_time)
    }

    fn latest<'a>(
        events: &'a [RecordedEvent],
        indexes: &[usize],
        log_time: SystemTime,
    ) -> Option<&'a RecordedEvent> {
        let next = indexes.partition_point(|&index| events[index].log_time <= log_time);
        next.checked_sub(1)
            .and_then(|index| indexes.get(index))
            .and_then(|&index| events.get(index))
    }

    fn nearest<'a>(
        events: &'a [RecordedEvent],
        indexes: &[usize],
        log_time: SystemTime,
    ) -> Option<&'a RecordedEvent> {
        let next = indexes.partition_point(|&index| events[index].log_time <= log_time);
        nearest_by_distance(
            next.checked_sub(1)
                .and_then(|index| indexes.get(index))
                .map(|&index| {
                    (
                        &events[index],
                        nanos_abs_diff(events[index].log_time, log_time),
                    )
                }),
            indexes.get(next).map(|&index| {
                (
                    &events[index],
                    nanos_abs_diff(events[index].log_time, log_time),
                )
            }),
        )
    }
}

#[derive(Clone)]
pub enum EventKind {
    Imu(ImuState),
    VisualOdometry(VisualOdometryDelta),
    RobotKinematics(Box<TimeWrapper<RobotKinematics>>),
    CameraMatrix(TimeWrapper<CameraMatrix>),
    DetectedObjects(Vec<Object<RobocupObjectLabel>>),
    RecordedLocalization(Option<Isometry3<Field, Robot>>),
    CalibratedIntrinsics(Intrinsic),
}

pub struct StereoImageIndex {
    pub order: usize,
    pub log_time: SystemTime,
    pub publish_time: SystemTime,
    pub embedded_time: Time,
    data: Arc<[u8]>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StereoImageId(usize);

impl StereoImageId {
    pub fn index(self) -> usize {
        self.0
    }
}

impl fmt::Display for StereoImageId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}", self.0)
    }
}

#[derive(Clone, Default)]
pub struct RecordingSnapshot {
    pub image_id: Option<StereoImageId>,
    pub camera_matrix: Option<TimeWrapper<CameraMatrix>>,
    pub detected_objects: Vec<Object<RobocupObjectLabel>>,
    pub detected_objects_time: Option<SystemTime>,
    pub recorded_localization: Option<Isometry3<Field, Robot>>,
    pub calibrated_intrinsics: Option<Intrinsic>,
    pub calibrated_intrinsics_time: Option<SystemTime>,
}

#[derive(Clone)]
pub struct StereoFrame {
    pub sequence: u64,
    pub source_time: Time,
    pub log_time: SystemTime,
    pub publish_time: SystemTime,
    pub left: CameraImage,
    pub right: CameraImage,
}

#[derive(Clone)]
pub struct CameraImage {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

#[derive(Clone)]
pub struct TrajectoryPoint {
    pub seconds: f64,
    pub robot_to_field: Isometry3<Robot, Field, f64>,
}

fn camera_image_from_ros(image: RosImage) -> Result<CameraImage> {
    let rgb: RgbImage = image
        .try_into()
        .map_err(|error| color_eyre::eyre::eyre!("failed to decode ROS image: {error}"))?;
    let (width, height) = rgb.dimensions();
    let mut rgba = Vec::with_capacity(width as usize * height as usize * 4);
    for pixel in rgb.pixels() {
        rgba.extend_from_slice(&[pixel[0], pixel[1], pixel[2], 255]);
    }
    Ok(CameraImage {
        width,
        height,
        rgba,
    })
}

fn decode_message<T>(data: &[u8]) -> Result<T>
where
    T: DeserializeOwned,
{
    let (value, _consumed) = from_bytes::<T, LittleEndian>(cdr_payload(data))?;
    Ok(value)
}

fn cdr_payload(data: &[u8]) -> &[u8] {
    if data.len() >= 4 && matches!(&data[..4], [0, 1, 0, 0] | [0, 0, 0, 0]) {
        &data[4..]
    } else {
        data
    }
}

fn decode_time_prefix(data: &[u8]) -> Result<Time> {
    let data = cdr_payload(data);
    if data.len() < 12 {
        return Err(color_eyre::eyre::eyre!("payload too short for Time prefix"));
    }
    let secs = u64::from_le_bytes(data[0..8].try_into()?);
    let nanos = u32::from_le_bytes(data[8..12].try_into()?);
    let total_nanos = secs
        .saturating_mul(1_000_000_000)
        .saturating_add(u64::from(nanos));
    Ok(Time::from_nanos(total_nanos.min(i64::MAX as u64) as i64))
}

#[derive(Deserialize)]
struct WireTimeWrapper<T> {
    time: Time,
    inner: T,
}

#[derive(Deserialize)]
struct WireCameraMatrix {
    ground_to_robot: WireIsometry3,
    robot_to_head: WireIsometry3,
    head_to_camera: WireIsometry3,
    intrinsics: WireIntrinsic,
    field_of_view: [f32; 2],
    horizon: Option<WireHorizon>,
    image_size: [f32; 2],
}

#[derive(Deserialize)]
struct WireIntrinsic {
    focals: [f32; 2],
    optical_center: [f32; 2],
}

#[derive(Deserialize)]
struct WireHorizon {
    vanishing_point: [f32; 2],
    normal: [f32; 2],
}

#[derive(Deserialize)]
struct WireVisualOdometryDelta {
    previous_time: Time,
    current_time: Time,
    current_left_camera_to_previous_left_camera: WireIsometry3,
}

#[derive(Deserialize)]
struct WireIsometry3 {
    rotation: [f32; 4],
    translation: [f32; 3],
}

impl WireCameraMatrix {
    fn into_camera_matrix(self) -> CameraMatrix {
        let image_size: linear_algebra::Vector2<Pixel> =
            vector![self.image_size[0], self.image_size[1]];
        let normalized_focal = nalgebra::vector![
            self.intrinsics.focals[0] / image_size.inner.x,
            self.intrinsics.focals[1] / image_size.inner.y,
        ];
        let normalized_center = nalgebra::point![
            self.intrinsics.optical_center[0] / image_size.inner.x,
            self.intrinsics.optical_center[1] / image_size.inner.y,
        ];
        let _ = self.field_of_view;
        if let Some(horizon) = self.horizon {
            let _ = (horizon.vanishing_point, horizon.normal);
        }

        CameraMatrix::from_normalized_focal_and_center(
            normalized_focal,
            normalized_center,
            image_size,
            self.ground_to_robot.framed(),
            self.robot_to_head.framed(),
            self.head_to_camera.framed(),
        )
    }
}

impl WireVisualOdometryDelta {
    fn into_visual_odometry_delta(self) -> VisualOdometryDelta {
        VisualOdometryDelta {
            previous_time: self.previous_time,
            current_time: self.current_time,
            current_left_camera_to_previous_left_camera: self
                .current_left_camera_to_previous_left_camera
                .into_isometry(),
        }
    }
}

impl WireIsometry3 {
    fn into_isometry(self) -> nalgebra::Isometry3<f32> {
        let rotation = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            self.rotation[3],
            self.rotation[0],
            self.rotation[1],
            self.rotation[2],
        ));
        nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(
                self.translation[0],
                self.translation[1],
                self.translation[2],
            ),
            rotation,
        )
    }

    fn framed<From, To>(self) -> linear_algebra::Isometry3<From, To> {
        self.into_isometry().framed_transform()
    }
}

fn decode_recorded_camera_matrix(message: &Message<'_>) -> Result<TimeWrapper<CameraMatrix>> {
    let wire: WireTimeWrapper<WireCameraMatrix> = decode_recorded_message(message)?;
    Ok(TimeWrapper {
        time: wire.time,
        inner: wire.inner.into_camera_matrix(),
    })
}

fn decode_recorded_visual_odometry(message: &Message<'_>) -> Result<VisualOdometryDelta> {
    let wire: WireVisualOdometryDelta = decode_recorded_message(message)?;
    Ok(wire.into_visual_odometry_delta())
}

fn decode_recorded_message<T>(message: &Message<'_>) -> Result<T>
where
    T: DeserializeOwned,
{
    decode_message(&message.data).wrap_err_with(|| {
        format!(
            "failed to decode topic {} sequence {}",
            message.channel.topic, message.sequence
        )
    })
}

fn system_time_from_nanos(nanos: u64) -> SystemTime {
    UNIX_EPOCH + Duration::from_nanos(nanos)
}

pub fn nanos_since_epoch(time: SystemTime) -> u128 {
    match time.duration_since(UNIX_EPOCH) {
        Ok(duration) => duration.as_nanos(),
        Err(_) => 0,
    }
}

pub fn nanos_abs_diff(a: SystemTime, b: SystemTime) -> u128 {
    nanos_since_epoch(a).abs_diff(nanos_since_epoch(b))
}

pub fn seconds_since(time: SystemTime, start: SystemTime) -> f64 {
    match time.duration_since(start) {
        Ok(duration) => duration.as_secs_f64(),
        Err(error) => -error.duration().as_secs_f64(),
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::*;

    #[test]
    #[ignore = "loads the large repository localization recording"]
    fn loads_repository_recording() -> Result<()> {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../recording.mcap");
        let recording = Recording::load(&path)?;

        assert!(recording.event_count() > 0);
        assert!(recording.image_count() > 0);
        assert!(recording.topic_counts.contains_key(TOPIC_CAMERA_MATRIX));
        assert!(recording.topic_counts.contains_key(TOPIC_STEREO_IMAGE_PAIR));

        let frame = recording.decode_stereo_image(StereoImageId(0))?;
        assert!(frame.left.width > 0);
        assert!(frame.left.height > 0);
        assert_eq!(
            frame.left.rgba.len(),
            frame.left.width as usize * frame.left.height as usize * 4,
        );

        Ok(())
    }
}
