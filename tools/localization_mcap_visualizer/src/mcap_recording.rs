use std::{
    collections::BTreeMap,
    fs,
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
use localization_3d::GlobalLocalizationDebug;
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

#[derive(Clone)]
pub struct Recording {
    bytes: Arc<[u8]>,
    pub events: Vec<RecordedEvent>,
    pub images: Vec<StereoImageIndex>,
    pub first_camera_matrix: CameraMatrix,
    pub field_dimensions: Option<FieldDimensions>,
    pub topic_counts: BTreeMap<String, usize>,
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
                "inputs/imu_state" => Some(EventKind::Imu(decode_recorded_message(&message)?)),
                "visual_odometry/current_left_camera_to_previous_left_camera" => Some(
                    EventKind::VisualOdometry(decode_recorded_visual_odometry(&message)?),
                ),
                "robot_kinematics" => Some(EventKind::RobotKinematics(decode_recorded_message(
                    &message,
                )?)),
                "camera_matrix" => {
                    let camera_matrix = decode_recorded_camera_matrix(&message)?;
                    if first_camera_matrix.is_none() {
                        first_camera_matrix = Some(camera_matrix.inner.clone());
                    }
                    Some(EventKind::CameraMatrix(camera_matrix))
                }
                "detected_objects" => Some(EventKind::DetectedObjects(decode_recorded_message(
                    &message,
                )?)),
                "field_dimensions" => {
                    let dimensions = decode_recorded_message(&message)?;
                    field_dimensions = Some(dimensions);
                    Some(EventKind::FieldDimensions(dimensions))
                }
                "localization" => Some(EventKind::RecordedLocalization(decode_recorded_message(
                    &message,
                )?)),
                "visual_odometry/current_left_camera_to_visual_odometer" => Some(
                    EventKind::VisualOdometer(decode_recorded_message(&message)?),
                ),
                "debug/global_localization" => Some(EventKind::RecordedGlobalLocalization(
                    decode_recorded_message(&message)?,
                )),
                "debug/calibrated_intrinsics" => Some(EventKind::CalibratedIntrinsics(
                    decode_recorded_message(&message)?,
                )),
                "inputs/stereo_image_pair" => {
                    images.push(StereoImageIndex {
                        order,
                        log_time,
                        publish_time,
                        embedded_time: decode_time_prefix(&message.data)?,
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
        images.sort_by_key(|image| (nanos_since_epoch(image.log_time), image.order));

        Ok(Self {
            bytes,
            events,
            images,
            first_camera_matrix: first_camera_matrix
                .ok_or_else(|| color_eyre::eyre::eyre!("recording has no camera_matrix topic"))?,
            field_dimensions,
            topic_counts,
        })
    }

    pub fn start_log_time(&self) -> SystemTime {
        self.events
            .first()
            .map(|event| event.log_time)
            .or_else(|| self.images.first().map(|image| image.log_time))
            .unwrap_or(UNIX_EPOCH)
    }

    pub fn start_source_time(&self) -> SystemTime {
        self.events
            .first()
            .map(|event| event.publish_time)
            .or_else(|| self.images.first().map(|image| image.publish_time))
            .unwrap_or(UNIX_EPOCH)
    }

    pub fn end_log_time(&self) -> SystemTime {
        self.events
            .last()
            .map(|event| event.log_time)
            .into_iter()
            .chain(self.images.last().map(|image| image.log_time))
            .max()
            .unwrap_or_else(|| self.start_log_time())
    }

    pub fn duration(&self) -> Duration {
        self.end_log_time()
            .duration_since(self.start_log_time())
            .unwrap_or_default()
    }

    pub fn log_time_at_seconds(&self, seconds: f64) -> SystemTime {
        self.start_log_time() + Duration::from_secs_f64(seconds.max(0.0))
    }

    pub fn seconds_since_start(&self, time: SystemTime) -> f64 {
        seconds_since(time, self.start_log_time())
    }

    pub fn aligned_image_time(&self, embedded_time: Time) -> Option<SystemTime> {
        let embedded_time = embedded_time.to_wallclock();
        self.images
            .iter()
            .min_by_key(|candidate| {
                nanos_abs_diff(candidate.embedded_time.to_wallclock(), embedded_time)
            })
            .map(|candidate| candidate.publish_time)
    }

    pub fn latest_snapshot(&self, log_time: SystemTime) -> RecordingSnapshot {
        let mut snapshot = RecordingSnapshot::default();
        snapshot.image_index = self.nearest_image_index(log_time);

        for event in self
            .events
            .iter()
            .take_while(|event| event.log_time <= log_time)
        {
            match &event.kind {
                EventKind::Imu(_) => {}
                EventKind::VisualOdometry(_) => {}
                EventKind::RobotKinematics(robot_kinematics) => {
                    snapshot.robot_kinematics = Some(robot_kinematics.clone());
                }
                EventKind::CameraMatrix(camera_matrix) => {
                    snapshot.camera_matrix = Some(camera_matrix.clone());
                }
                EventKind::DetectedObjects(objects) => {
                    snapshot.detected_objects = objects.clone();
                    snapshot.detected_objects_time = Some(event.publish_time);
                }
                EventKind::FieldDimensions(field_dimensions) => {
                    snapshot.field_dimensions = Some(*field_dimensions);
                }
                EventKind::RecordedLocalization(localization) => {
                    snapshot.recorded_localization = *localization;
                }
                EventKind::VisualOdometer(visual_odometer) => {
                    snapshot.visual_odometer = Some(*visual_odometer);
                }
                EventKind::RecordedGlobalLocalization(global_localization) => {
                    snapshot.recorded_global_localization = global_localization.clone();
                }
                EventKind::CalibratedIntrinsics(intrinsics) => {
                    snapshot.calibrated_intrinsics = Some(*intrinsics);
                }
            }
        }

        snapshot
    }

    pub fn recorded_localization_trajectory(&self) -> Vec<TrajectoryPoint> {
        self.events
            .iter()
            .filter_map(|event| match &event.kind {
                EventKind::RecordedLocalization(Some(field_to_robot)) => Some(TrajectoryPoint {
                    seconds: self.seconds_since_start(event.log_time),
                    robot_to_field: field_to_robot.inverse().inner.cast(),
                }),
                _ => None,
            })
            .collect()
    }

    pub fn decode_stereo_image(&self, image_index: usize) -> Result<StereoFrame> {
        let Some(index) = self.images.get(image_index) else {
            return Err(color_eyre::eyre::eyre!(
                "image index {image_index} is out of bounds"
            ));
        };

        for (order, message) in MessageStream::new(&self.bytes)
            .wrap_err("failed to open MCAP message stream")?
            .enumerate()
        {
            if order != index.order {
                continue;
            }
            let message = message.wrap_err("failed to read MCAP image message")?;
            let stereo: TimeWrapper<StereoImagePair> = decode_recorded_message(&message)?;
            return Ok(StereoFrame {
                sequence: index.order as u64,
                source_time: stereo.time,
                log_time: index.log_time,
                publish_time: index.publish_time,
                left: camera_image_from_ros(stereo.inner.left)?,
                right: camera_image_from_ros(stereo.inner.right)?,
            });
        }

        Err(color_eyre::eyre::eyre!(
            "failed to find image message with order {}",
            index.order
        ))
    }

    fn nearest_image_index(&self, log_time: SystemTime) -> Option<usize> {
        self.images
            .iter()
            .enumerate()
            .min_by_key(|(_, image)| nanos_abs_diff(image.log_time, log_time))
            .map(|(index, _)| index)
    }
}

#[derive(Clone)]
pub struct RecordedEvent {
    pub order: usize,
    pub log_time: SystemTime,
    pub publish_time: SystemTime,
    pub kind: EventKind,
}

#[derive(Clone)]
pub enum EventKind {
    Imu(ImuState),
    VisualOdometry(VisualOdometryDelta),
    RobotKinematics(TimeWrapper<RobotKinematics>),
    CameraMatrix(TimeWrapper<CameraMatrix>),
    DetectedObjects(Vec<Object<RobocupObjectLabel>>),
    FieldDimensions(FieldDimensions),
    RecordedLocalization(Option<Isometry3<Field, Robot>>),
    VisualOdometer(nalgebra::Isometry3<f32>),
    RecordedGlobalLocalization(Option<GlobalLocalizationDebug>),
    CalibratedIntrinsics(Intrinsic),
}

#[derive(Clone)]
pub struct StereoImageIndex {
    pub order: usize,
    pub log_time: SystemTime,
    pub publish_time: SystemTime,
    pub embedded_time: Time,
}

#[derive(Clone, Default)]
pub struct RecordingSnapshot {
    pub image_index: Option<usize>,
    pub camera_matrix: Option<TimeWrapper<CameraMatrix>>,
    pub robot_kinematics: Option<TimeWrapper<RobotKinematics>>,
    pub detected_objects: Vec<Object<RobocupObjectLabel>>,
    pub detected_objects_time: Option<SystemTime>,
    pub recorded_localization: Option<Isometry3<Field, Robot>>,
    pub visual_odometer: Option<nalgebra::Isometry3<f32>>,
    pub recorded_global_localization: Option<GlobalLocalizationDebug>,
    pub calibrated_intrinsics: Option<Intrinsic>,
    pub field_dimensions: Option<FieldDimensions>,
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
    pub robot_to_field: nalgebra::Isometry3<f64>,
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
    time.duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
}

pub fn nanos_abs_diff(a: SystemTime, b: SystemTime) -> u128 {
    nanos_since_epoch(a).abs_diff(nanos_since_epoch(b))
}

pub fn seconds_since(time: SystemTime, start: SystemTime) -> f64 {
    time.duration_since(start)
        .map(|duration| duration.as_secs_f64())
        .unwrap_or_else(|error| -error.duration().as_secs_f64())
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

        assert!(!recording.events.is_empty());
        assert!(!recording.images.is_empty());
        assert!(recording.topic_counts.contains_key("camera_matrix"));
        assert!(
            recording
                .topic_counts
                .contains_key("inputs/stereo_image_pair")
        );

        let frame = recording.decode_stereo_image(0)?;
        assert!(frame.left.width > 0);
        assert!(frame.left.height > 0);
        assert_eq!(
            frame.left.rgba.len(),
            frame.left.width as usize * frame.left.height as usize * 4,
        );

        Ok(())
    }
}
