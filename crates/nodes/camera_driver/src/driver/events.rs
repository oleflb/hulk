use std::fmt;

#[cfg(x5cam_x5_target)]
pub use crate::codec::EncodedFrameData;

#[cfg(not(x5cam_x5_target))]
/// Placeholder payload type for non-X5 builds that cannot open the camera.
#[derive(Debug)]
pub struct EncodedFrameData;

#[cfg(not(x5cam_x5_target))]
impl EncodedFrameData {
    /// Returns the placeholder payload length in bytes.
    pub fn len(&self) -> usize {
        0
    }
}

/// Camera side for stereo-specific events and counters.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Channel {
    /// Left stereo camera.
    Left,
    /// Right stereo camera.
    Right,
}

impl Channel {
    /// Returns the stable array index for this channel.
    pub fn index(self) -> usize {
        match self {
            Self::Left => 0,
            Self::Right => 1,
        }
    }
}

impl fmt::Display for Channel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Left => f.write_str("left"),
            Self::Right => f.write_str("right"),
        }
    }
}

/// Static camera configuration emitted after successful setup.
#[derive(Clone, Debug)]
pub struct CameraInfo {
    /// Stereo channel described by this camera.
    pub channel: Channel,
    /// X5 MIPI host index used by the camera.
    pub host: i32,
    /// Sensor mode name used by the SDK.
    pub sensor_name: String,
    /// Raw sensor frame width before rotation/rectification.
    pub raw_width: u32,
    /// Raw sensor frame height before rotation/rectification.
    pub raw_height: u32,
    /// Rectified encoder input width.
    pub out_width: u32,
    /// Rectified encoder input height.
    pub out_height: u32,
    /// Required per-camera frame rate.
    pub fps: u32,
    /// Whether hardware GDC rectification is enabled.
    pub gdc_enabled: bool,
    /// Whether H.265 encoding is enabled.
    pub h265_enabled: bool,
    /// Whether MediaCodec consumes external VSE frame buffers.
    pub external_input: bool,
}

/// Stereo calibration summary emitted from the EEPROM data.
#[derive(Clone, Debug)]
pub struct CalibrationInfo {
    /// EEPROM calibration image width.
    pub raw_width: u32,
    /// EEPROM calibration image height.
    pub raw_height: u32,
    /// Rectified output width requested from GDC.
    pub rect_width: u32,
    /// Rectified output height requested from GDC.
    pub rect_height: u32,
    /// Distortion model used by the EEPROM coefficients.
    pub distortion_model: String,
    /// Absolute stereo baseline in meters.
    pub baseline_m: f64,
}

/// Zero-copy H.265 access-unit payload and its source metadata.
#[derive(Debug)]
pub struct EncodedFrame {
    /// Stereo channel that produced the frame.
    pub channel: Channel,
    /// VSE frame identifier associated with the encoder input.
    pub frame_id: u32,
    /// VSE source timestamp in nanoseconds.
    pub timestamp_ns: u64,
    /// Leased H.265 bytes from the MediaCodec output buffer.
    pub data: EncodedFrameData,
    /// Encoder presentation timestamp in microseconds.
    pub pts_us: u64,
}

/// Recoverable camera-side worker error.
#[derive(Clone, Debug)]
pub struct CameraError {
    /// Stereo channel that reported the error.
    pub channel: Channel,
    /// Human-readable error message.
    pub message: String,
}

/// Events emitted by the X5 camera backend.
#[derive(Debug)]
#[cfg_attr(not(x5cam_x5_target), allow(dead_code))]
pub enum Event {
    /// Camera setup information.
    CameraInfo(CameraInfo),
    /// EEPROM calibration information.
    Calibration(CalibrationInfo),
    /// Encoded H.265 payload from one camera.
    EncodedFrame(EncodedFrame),
    /// Worker or SDK error from one camera.
    Error(CameraError),
}
