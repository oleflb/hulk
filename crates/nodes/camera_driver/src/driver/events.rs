use std::fmt;

use camera_driver_sys::{SensorMode, SensorModule};

#[cfg(not(x5cam_x5_target))]
use std::sync::Arc;

#[cfg(x5cam_x5_target)]
type EncodedFrameData = camera_driver_sys::EncodedFrameData;

#[cfg(not(x5cam_x5_target))]
type EncodedFrameData = Arc<[u8]>;

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

    /// Returns the optical frame id used in ROS image headers.
    pub fn frame_id(self) -> &'static str {
        match self {
            Self::Left => "x5_left_optical_frame",
            Self::Right => "x5_right_optical_frame",
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

/// Static camera setup emitted after successful SDK initialization.
#[derive(Clone, Debug)]
pub struct CameraSetup {
    /// Stereo channel described by this camera.
    pub channel: Channel,
    /// X5 MIPI host index used by the camera.
    pub host: i32,
    /// Sensor module used by the SDK.
    pub sensor: SensorModule,
    /// Sensor mode used for capture.
    pub sensor_mode: SensorMode,
    /// Whether VIN LPWM trigger generation is enabled.
    pub lpwm_enabled: bool,
    /// Raw sensor frame width before rotation/rectification.
    pub raw_width: u32,
    /// Raw sensor frame height before rotation/rectification.
    pub raw_height: u32,
    /// Rectified output width.
    pub out_width: u32,
    /// Rectified output height.
    pub out_height: u32,
    /// Required per-camera frame rate.
    pub fps: u32,
    /// Whether hardware GDC rectification is enabled.
    pub gdc_enabled: bool,
    /// Whether HEVC encoding is enabled for this camera.
    pub hevc_enabled: bool,
    /// Whether MediaCodec consumes external VSE frame buffers directly.
    pub external_encoder_input: bool,
}

/// Stereo calibration emitted from EEPROM as ROS-compatible camera info messages.
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
    /// Raw left-camera focal lengths and principal point `[fx, fy, cx, cy]`.
    pub raw_left_intrinsics: [f64; 4],
    /// Raw left-camera distortion coefficients.
    pub raw_left_distortion: [f64; 8],
    /// Absolute stereo baseline in meters.
    pub baseline_m: f64,
    /// Rectified horizontal focal length in pixels.
    pub rect_fx: f64,
    /// Rectified vertical focal length in pixels.
    pub rect_fy: f64,
    /// Rectified horizontal principal point in pixels.
    pub rect_cx: f64,
    /// Rectified vertical principal point in pixels.
    pub rect_cy: f64,
}

/// Encoded HEVC access unit produced by one camera.
#[derive(Debug)]
pub struct EncodedFrame {
    /// Stereo channel that produced the frame.
    pub channel: Channel,
    /// VSE frame identifier.
    pub frame_id: u32,
    /// VSE/VIN frame timestamp in nanoseconds.
    pub timestamp_ns: u64,
    /// VIN trigger timestamp in nanoseconds, when reported by the SDK.
    pub trigger_timestamp_ns: Option<u64>,
    /// Encoded image width in pixels.
    pub width: u32,
    /// Encoded image height in pixels.
    pub height: u32,
    /// Encoder presentation timestamp in microseconds.
    pub pts_us: u64,
    /// Leased encoded bytes. On X5 this borrows the SDK output buffer until dropped.
    pub data: EncodedFrameData,
}

impl EncodedFrame {
    /// Borrows the encoded HEVC bytes without copying.
    pub fn data(&self) -> &[u8] {
        self.data.as_ref()
    }

    /// Returns the encoded payload length in bytes.
    pub fn data_len(&self) -> usize {
        self.data().len()
    }
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
    CameraSetup(CameraSetup),
    /// EEPROM calibration information.
    Calibration(CalibrationInfo),
    /// Encoded HEVC frame from one camera.
    EncodedFrame(EncodedFrame),
    /// Worker or SDK error from one camera.
    Error(CameraError),
}
