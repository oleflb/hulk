use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use coordinate_systems::{Field, Robot};
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::Isometry3;
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{cache::CacheInner, time::Time};
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
    visual_localization::VisualLocalizationFrame as FieldMarkAssociations,
};

mod alignment;

use alignment::{exact_sample, nearest_sample};

pub(crate) type SharedState = Arc<Mutex<ViewerState>>;

const CAMERA_FRAME_BUFFER_CAPACITY: usize = 12;
const STREAM_BUFFER_CAPACITY: usize = 64;
const HIGH_RATE_STREAM_BUFFER_CAPACITY: usize = 1024;
const MAX_NEAREST_SAMPLE_DISTANCE: Duration = Duration::from_millis(100);

pub(crate) struct ViewerState {
    pub(crate) connection: ConnectionStatus,
    pub(crate) field_dimensions: Option<FieldDimensions>,
    pub(crate) localizations: CacheInner<Option<Isometry3<Field, Robot>>>,
    pub(crate) visual_odometer: Option<nalgebra::Isometry3<f32>>,
    pub(crate) robot_kinematics: CacheInner<RobotKinematics>,
    pub(crate) camera_matrices: CacheInner<CameraMatrix>,
    pub(crate) calibrated_intrinsics: Option<Intrinsic>,
    pub(crate) camera_frames: CacheInner<CameraFrame>,
    pub(crate) camera_sequence: u64,
    display_anchor_time: Option<Time>,
    displayed_camera_frame: Option<TimeWrapper<Arc<CameraFrame>>>,
    pub(crate) detected_objects: CacheInner<Vec<Object<RobocupObjectLabel>>>,
    pub(crate) field_mark_associations: CacheInner<FieldMarkAssociations>,
    pub(crate) field_status: StreamStatus,
    pub(crate) localization_status: StreamStatus,
    pub(crate) visual_odometer_status: StreamStatus,
    pub(crate) robot_kinematics_status: StreamStatus,
    pub(crate) camera_matrix_status: StreamStatus,
    pub(crate) calibrated_intrinsics_status: StreamStatus,
    pub(crate) camera_status: StreamStatus,
    pub(crate) objects_status: StreamStatus,
    pub(crate) field_mark_associations_status: StreamStatus,
}

impl Default for ViewerState {
    fn default() -> Self {
        Self {
            connection: ConnectionStatus::default(),
            field_dimensions: None,
            localizations: CacheInner::new(HIGH_RATE_STREAM_BUFFER_CAPACITY),
            visual_odometer: None,
            robot_kinematics: CacheInner::new(HIGH_RATE_STREAM_BUFFER_CAPACITY),
            camera_matrices: CacheInner::new(HIGH_RATE_STREAM_BUFFER_CAPACITY),
            calibrated_intrinsics: None,
            camera_frames: CacheInner::new(CAMERA_FRAME_BUFFER_CAPACITY),
            camera_sequence: 0,
            display_anchor_time: None,
            displayed_camera_frame: None,
            detected_objects: CacheInner::new(STREAM_BUFFER_CAPACITY),
            field_mark_associations: CacheInner::new(STREAM_BUFFER_CAPACITY),
            field_status: StreamStatus::default(),
            localization_status: StreamStatus::default(),
            visual_odometer_status: StreamStatus::default(),
            robot_kinematics_status: StreamStatus::default(),
            camera_matrix_status: StreamStatus::default(),
            calibrated_intrinsics_status: StreamStatus::default(),
            camera_status: StreamStatus::default(),
            objects_status: StreamStatus::default(),
            field_mark_associations_status: StreamStatus::default(),
        }
    }
}

impl ViewerState {
    pub(crate) fn push_camera_frame(&mut self, time: Time, frame: CameraFrame) {
        self.camera_frames.insert(time, frame);
    }

    pub(crate) fn push_localization(&mut self, time: Time, value: Option<Isometry3<Field, Robot>>) {
        self.localizations.insert(time, value);
    }

    pub(crate) fn push_robot_kinematics(&mut self, time: Time, value: RobotKinematics) {
        self.robot_kinematics.insert(time, value);
    }

    pub(crate) fn push_camera_matrix(&mut self, time: Time, value: CameraMatrix) {
        self.camera_matrices.insert(time, value);
    }

    pub(crate) fn push_detected_objects(
        &mut self,
        value: TimeWrapper<Vec<Object<RobocupObjectLabel>>>,
    ) {
        self.detected_objects.insert(value.time, value.inner);
    }

    pub(crate) fn push_field_mark_associations(
        &mut self,
        time: Time,
        value: FieldMarkAssociations,
    ) {
        self.field_mark_associations.insert(time, value);
    }

    pub(crate) fn status_snapshot(&self) -> ViewerStatusSnapshot {
        ViewerStatusSnapshot {
            connection: self.connection.clone(),
            field_status: self.field_status.clone(),
            localization_status: self.localization_status.clone(),
            visual_odometer_status: self.visual_odometer_status.clone(),
            robot_kinematics_status: self.robot_kinematics_status.clone(),
            camera_matrix_status: self.camera_matrix_status.clone(),
            calibrated_intrinsics_status: self.calibrated_intrinsics_status.clone(),
            camera_status: self.camera_status.clone(),
            objects_status: self.objects_status.clone(),
            field_mark_associations_status: self.field_mark_associations_status.clone(),
        }
    }

    /// Builds the data snapshot rendered by the camera panel and 3D scene for a selected frame.
    ///
    /// The anchor only advances to a newer camera frame once all active, already-producing aligned
    /// streams have a matching sample for it. Exact image outputs must match the image timestamp;
    /// high-rate pose, matrix, and kinematics streams may use the nearest sample within
    /// `MAX_NEAREST_SAMPLE_DISTANCE`. If no newer complete frame exists, the current frame remains
    /// selected so the UI never clears the image while waiting for delayed inputs.
    pub(crate) fn aligned_snapshot(&mut self) -> AlignedViewerState {
        let latest_camera_time = self.camera_frames.latest_stamp();
        let complete_anchor_time =
            latest_camera_time.and_then(|time| self.latest_complete_anchor_time(time));
        let display_anchor_available = self.display_anchor_time.is_some_and(|time| {
            self.camera_frames.get_exact(time).is_some()
                || self
                    .displayed_camera_frame
                    .as_ref()
                    .is_some_and(|frame| frame.time == time)
        });
        let anchor_time = match (complete_anchor_time, self.display_anchor_time) {
            (Some(complete), Some(displayed))
                if display_anchor_available && complete <= displayed =>
            {
                Some(displayed)
            }
            (Some(complete), _) => Some(complete),
            (None, Some(displayed)) if display_anchor_available => Some(displayed),
            (None, _) => latest_camera_time,
        };

        let camera_frame = anchor_time
            .and_then(|time| exact_sample(&self.camera_frames, time))
            .or_else(|| {
                self.displayed_camera_frame
                    .as_ref()
                    .filter(|frame| Some(frame.time) == anchor_time)
                    .cloned()
            });
        let localization = anchor_time
            .and_then(|time| nearest_sample(&self.localizations, time, MAX_NEAREST_SAMPLE_DISTANCE))
            .map(|sample| TimeWrapper {
                time: sample.time,
                inner: *sample.inner,
            });
        let camera_matrix = anchor_time.and_then(|time| {
            nearest_sample(&self.camera_matrices, time, MAX_NEAREST_SAMPLE_DISTANCE)
        });
        let robot_kinematics = anchor_time.and_then(|time| {
            nearest_sample(&self.robot_kinematics, time, MAX_NEAREST_SAMPLE_DISTANCE)
        });
        let detected_objects =
            anchor_time.and_then(|time| exact_sample(&self.detected_objects, time));
        let field_mark_associations =
            anchor_time.and_then(|time| exact_sample(&self.field_mark_associations, time));

        self.display_anchor_time = anchor_time;
        if let Some(camera_frame) = &camera_frame {
            self.displayed_camera_frame = Some(camera_frame.clone());
        }

        AlignedViewerState {
            anchor_time,
            field_dimensions: self.field_dimensions,
            localization,
            latest_visual_odometer: self.visual_odometer,
            robot_kinematics,
            camera_matrix,
            latest_calibrated_intrinsics: self.calibrated_intrinsics,
            camera_frame,
            detected_objects,
            field_mark_associations,
        }
    }

    fn latest_complete_anchor_time(&self, latest_camera_time: Time) -> Option<Time> {
        let mut candidate_time = Some(latest_camera_time);
        while let Some(time) = candidate_time {
            if self.has_required_samples(time) {
                return Some(time);
            }
            candidate_time = self.camera_frames.latest_stamp_before(time);
        }
        None
    }

    fn has_required_samples(&self, time: Time) -> bool {
        (!requires_stream(&self.objects_status, &self.detected_objects)
            || self.detected_objects.get_exact(time).is_some())
            && (!requires_stream(
                &self.field_mark_associations_status,
                &self.field_mark_associations,
            ) || self.field_mark_associations.get_exact(time).is_some())
            && (!requires_stream(&self.localization_status, &self.localizations)
                || nearest_sample(&self.localizations, time, MAX_NEAREST_SAMPLE_DISTANCE).is_some())
            && (!requires_stream(&self.camera_matrix_status, &self.camera_matrices)
                || nearest_sample(&self.camera_matrices, time, MAX_NEAREST_SAMPLE_DISTANCE)
                    .is_some())
            && (!requires_stream(&self.robot_kinematics_status, &self.robot_kinematics)
                || nearest_sample(&self.robot_kinematics, time, MAX_NEAREST_SAMPLE_DISTANCE)
                    .is_some())
    }
}

fn requires_stream<T>(status: &StreamStatus, cache: &CacheInner<T>) -> bool {
    status.publisher_count > 0 && !cache.is_empty()
}

#[derive(Clone, Default)]
pub(crate) struct ViewerStatusSnapshot {
    pub(crate) connection: ConnectionStatus,
    pub(crate) field_status: StreamStatus,
    pub(crate) localization_status: StreamStatus,
    pub(crate) visual_odometer_status: StreamStatus,
    pub(crate) robot_kinematics_status: StreamStatus,
    pub(crate) camera_matrix_status: StreamStatus,
    pub(crate) calibrated_intrinsics_status: StreamStatus,
    pub(crate) camera_status: StreamStatus,
    pub(crate) objects_status: StreamStatus,
    pub(crate) field_mark_associations_status: StreamStatus,
}

/// Timestamp-aligned render input for one displayed camera frame.
///
/// Optional exact-match streams stay `None` when the corresponding timestamp has not arrived, so
/// the UI can show “unavailable” instead of pretending the stream produced an empty result.
#[derive(Clone, Default)]
pub(crate) struct AlignedViewerState {
    pub(crate) anchor_time: Option<Time>,
    pub(crate) field_dimensions: Option<FieldDimensions>,
    pub(crate) localization: Option<TimeWrapper<Option<Isometry3<Field, Robot>>>>,
    pub(crate) latest_visual_odometer: Option<nalgebra::Isometry3<f32>>,
    pub(crate) robot_kinematics: Option<TimeWrapper<Arc<RobotKinematics>>>,
    pub(crate) camera_matrix: Option<TimeWrapper<Arc<CameraMatrix>>>,
    pub(crate) latest_calibrated_intrinsics: Option<Intrinsic>,
    pub(crate) camera_frame: Option<TimeWrapper<Arc<CameraFrame>>>,
    pub(crate) detected_objects: Option<TimeWrapper<Arc<Vec<Object<RobocupObjectLabel>>>>>,
    pub(crate) field_mark_associations: Option<TimeWrapper<Arc<FieldMarkAssociations>>>,
}

#[derive(Clone, Copy, Default, Eq, PartialEq)]
pub(crate) enum PoseSource {
    #[default]
    Localization,
    VisualOdometer,
}

#[derive(Clone, Default)]
pub(crate) struct CameraFrame {
    pub(crate) sequence: u64,
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) rgba: Vec<u8>,
}

#[derive(Clone, Default)]
pub(crate) enum ConnectionStatus {
    #[default]
    Starting,
    Connecting,
    Subscribed,
    Error(String),
}

#[derive(Clone, Default)]
pub(crate) struct StreamStatus {
    pub(crate) state: StreamState,
    pub(crate) publisher_count: usize,
    pub(crate) detail: Option<String>,
}

impl StreamStatus {
    pub(crate) fn update_publishers(&mut self, publisher_count: usize) {
        self.publisher_count = publisher_count;
        if matches!(self.state, StreamState::Waiting | StreamState::Matched) {
            self.state = if publisher_count > 0 {
                StreamState::Matched
            } else {
                StreamState::Waiting
            };
        }
    }

    pub(crate) fn mark_live(&mut self, publisher_count: usize) {
        self.state = StreamState::Live;
        self.publisher_count = publisher_count;
        self.detail = None;
    }

    pub(crate) fn mark_value(&mut self, publisher_count: usize, has_value: bool) {
        self.state = if has_value {
            StreamState::Live
        } else {
            StreamState::Empty
        };
        self.publisher_count = publisher_count;
        self.detail = None;
    }

    pub(crate) fn mark_error(&mut self, publisher_count: usize, detail: String) {
        self.state = StreamState::Error;
        self.publisher_count = publisher_count;
        self.detail = Some(detail);
    }
}

#[derive(Clone, Default)]
pub(crate) enum StreamState {
    #[default]
    Waiting,
    Matched,
    Live,
    Empty,
    Error,
}
