use std::sync::Arc;

use ros_z::Message;
use serde::{Deserialize, Serialize};

/// Codec used for an encoded camera-frame payload.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize, Message)]
pub enum EncodedFrameCodec {
    /// H.265 / HEVC access unit.
    Hevc,
}

/// One compressed camera frame or access unit.
///
/// `EncodedFrame` is intended for high-rate camera transport where raw image
/// topics would exceed the robot network budget. The payload is owned so it can
/// be serialized by ROS-Z, while producers should keep data borrowed or leased
/// internally until this message boundary.
#[derive(Clone, Debug, Serialize, Deserialize, Message)]
pub struct EncodedFrame {
    /// Monotonic frame identifier from the camera pipeline.
    pub frame_identifier: u32,
    /// Camera-side capture timestamp in nanoseconds.
    ///
    /// `camera_driver` uses the VIN trigger timestamp when the SDK reports it;
    /// otherwise it falls back to the VSE frame timestamp.
    pub timestamp_ns: u64,
    /// Encoder presentation timestamp in microseconds.
    pub presentation_timestamp_us: u64,
    /// Encoded image width in pixels.
    pub width: u32,
    /// Encoded image height in pixels.
    pub height: u32,
    /// Codec used for the payload.
    pub codec: EncodedFrameCodec,
    /// Encoded access-unit bytes.
    pub data: Arc<[u8]>,
}
