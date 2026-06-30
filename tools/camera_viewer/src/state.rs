use std::sync::Mutex;

use crate::{camera::CameraSide, frame::ViewerFrame};

pub(crate) struct SharedCameraState {
    side: CameraSide,
    inner: Mutex<CameraState>,
}

impl SharedCameraState {
    pub(crate) fn new(side: CameraSide) -> Self {
        Self {
            side,
            inner: Mutex::new(CameraState::default()),
        }
    }

    pub(crate) fn side(&self) -> CameraSide {
        self.side
    }

    pub(crate) fn take_latest_and_snapshot(&self) -> (Option<ViewerFrame>, CameraSnapshot) {
        self.with_mut(|state| (state.latest_frame.take(), state.snapshot()))
    }

    pub(crate) fn set_status(&self, status: impl Into<String>) {
        self.with_mut(|state| state.status = status.into());
    }

    pub(crate) fn set_error(&self, error: String) {
        self.with_mut(|state| {
            state.status = "error".to_string();
            state.error = Some(error);
        });
    }

    pub(crate) fn update_idle(&self, received_fps: f32, publisher_count: usize) {
        self.with_mut(|state| {
            state.received_fps = received_fps;
            state.publisher_count = publisher_count;
            if state.error.is_none() {
                state.status = if publisher_count == 0 {
                    "waiting for publisher".to_string()
                } else {
                    "waiting for frame".to_string()
                };
            }
        });
    }

    pub(crate) fn update_received(&self, received_fps: f32, publisher_count: usize) {
        self.with_mut(|state| {
            state.received_frames += 1;
            state.received_fps = received_fps;
            state.publisher_count = publisher_count;
            if state.error.is_none() {
                state.status = "receiving".to_string();
            }
        });
    }

    pub(crate) fn update_waiting_for_irap(&self) {
        self.with_mut(|state| {
            state.dropped_before_sync += 1;
            if state.error.is_none() {
                state.status = "waiting for IRAP".to_string();
            }
        });
    }

    pub(crate) fn update_decoder_timeout(&self) {
        self.with_mut(|state| {
            state.decoder_restarts += 1;
            if state.error.is_none() {
                state.status = "decoder timeout; waiting for IRAP".to_string();
            }
        });
    }

    pub(crate) fn update_decoded(&self, frame: ViewerFrame, decoded_fps: f32) {
        self.with_mut(|state| {
            state.decoded_frames += 1;
            state.decoded_fps = decoded_fps;
            state.last_frame_identifier = Some(frame.frame_identifier);
            state.last_timestamp_ns = Some(frame.timestamp_ns);
            state.last_presentation_timestamp_us = Some(frame.presentation_timestamp_us);
            state.last_dimensions = Some((frame.width, frame.height));
            state.latest_frame = Some(frame);
            if state.error.is_none() {
                state.status = "displaying".to_string();
            }
        });
    }

    fn with_mut<T>(&self, callback: impl FnOnce(&mut CameraState) -> T) -> T {
        let mut state = self
            .inner
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        callback(&mut state)
    }
}

#[derive(Clone)]
pub(crate) struct CameraSnapshot {
    pub(crate) status: String,
    pub(crate) error: Option<String>,
    pub(crate) publisher_count: usize,
    pub(crate) received_frames: u64,
    pub(crate) decoded_frames: u64,
    pub(crate) received_fps: f32,
    pub(crate) decoded_fps: f32,
    pub(crate) decoder_restarts: u64,
    pub(crate) dropped_before_sync: u64,
    pub(crate) last_frame_identifier: Option<u32>,
    pub(crate) last_timestamp_ns: Option<u64>,
    pub(crate) last_presentation_timestamp_us: Option<u64>,
    pub(crate) last_dimensions: Option<(u32, u32)>,
}

struct CameraState {
    latest_frame: Option<ViewerFrame>,
    status: String,
    error: Option<String>,
    publisher_count: usize,
    received_frames: u64,
    decoded_frames: u64,
    received_fps: f32,
    decoded_fps: f32,
    decoder_restarts: u64,
    dropped_before_sync: u64,
    last_frame_identifier: Option<u32>,
    last_timestamp_ns: Option<u64>,
    last_presentation_timestamp_us: Option<u64>,
    last_dimensions: Option<(u32, u32)>,
}

impl Default for CameraState {
    fn default() -> Self {
        Self {
            latest_frame: None,
            status: "starting".to_string(),
            error: None,
            publisher_count: 0,
            received_frames: 0,
            decoded_frames: 0,
            received_fps: 0.0,
            decoded_fps: 0.0,
            decoder_restarts: 0,
            dropped_before_sync: 0,
            last_frame_identifier: None,
            last_timestamp_ns: None,
            last_presentation_timestamp_us: None,
            last_dimensions: None,
        }
    }
}

impl CameraState {
    fn snapshot(&self) -> CameraSnapshot {
        CameraSnapshot {
            status: self.status.clone(),
            error: self.error.clone(),
            publisher_count: self.publisher_count,
            received_frames: self.received_frames,
            decoded_frames: self.decoded_frames,
            received_fps: self.received_fps,
            decoded_fps: self.decoded_fps,
            decoder_restarts: self.decoder_restarts,
            dropped_before_sync: self.dropped_before_sync,
            last_frame_identifier: self.last_frame_identifier,
            last_timestamp_ns: self.last_timestamp_ns,
            last_presentation_timestamp_us: self.last_presentation_timestamp_us,
            last_dimensions: self.last_dimensions,
        }
    }
}
