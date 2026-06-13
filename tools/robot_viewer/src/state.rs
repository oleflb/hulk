use std::sync::{Arc, Mutex};

use coordinate_systems::{Field, Robot};
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::Isometry3;
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
};

pub(crate) type SharedState = Arc<Mutex<ViewerState>>;

#[derive(Clone, Default)]
pub(crate) struct ViewerState {
    pub(crate) connection: ConnectionStatus,
    pub(crate) field_dimensions: Option<FieldDimensions>,
    pub(crate) localization: Option<Isometry3<Field, Robot>>,
    pub(crate) robot_kinematics: Option<RobotKinematics>,
    pub(crate) camera_matrix: Option<CameraMatrix>,
    pub(crate) calibrated_intrinsics: Option<Intrinsic>,
    pub(crate) camera_frame: Option<CameraFrame>,
    pub(crate) camera_sequence: u64,
    pub(crate) detected_objects: Vec<Object<RobocupObjectLabel>>,
    pub(crate) field_status: StreamStatus,
    pub(crate) localization_status: StreamStatus,
    pub(crate) robot_kinematics_status: StreamStatus,
    pub(crate) camera_matrix_status: StreamStatus,
    pub(crate) calibrated_intrinsics_status: StreamStatus,
    pub(crate) camera_status: StreamStatus,
    pub(crate) objects_status: StreamStatus,
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
