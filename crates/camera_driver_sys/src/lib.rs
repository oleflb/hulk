//! Typed, safe ownership wrappers around the X5 camera SDK pieces used by
//! `camera_driver`.
//!
//! The bindgen module generated from the vendor headers is intentionally kept
//! private. Public types in this crate encode SDK concepts with Rust enums,
//! newtypes, and RAII resources so application code does not pass raw strings,
//! magic numbers, or bare handles across the boundary.

use std::num::NonZeroU16;

#[cfg(x5cam_x5_target)]
use std::{
    ffi::CStr,
    fmt,
    marker::PhantomData,
    os::raw::c_char,
    ptr::NonNull,
    sync::{Arc, mpsc},
    time::{Duration, Instant},
};

#[cfg(x5cam_x5_target)]
#[allow(
    dead_code,
    improper_ctypes,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    unused_imports
)]
mod raw {
    include!(concat!(env!("OUT_DIR"), "/x5_sdk_bindings.rs"));
}

/// Result type returned by safe X5 SDK wrappers.
pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{operation:?} failed ret={ret}{info}")]
    Sdk {
        operation: Operation,
        ret: i32,
        info: String,
    },
    #[error("{operation:?} failed ret={ret}")]
    Media { operation: Operation, ret: i32 },
    #[error("{primary}; cleanup failed: {cleanup}")]
    Cleanup {
        primary: Box<Error>,
        cleanup: Box<Error>,
    },
    #[error("invalid input: {0}")]
    InvalidInput(String),
    #[error("{0}")]
    Other(String),
}

impl Error {
    pub fn invalid(message: impl Into<String>) -> Self {
        Self::InvalidInput(message.into())
    }

    pub fn other(message: impl Into<String>) -> Self {
        Self::Other(message.into())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Operation {
    HbMemModuleOpen,
    HbMemModuleClose,
    HbMemAllocCommonBuffer,
    HbMemFlushBuffer,
    HbMemFreeBuffer,
    GenerateGdcBin,
    FreeGdcBin,
    CameraCreate,
    CameraAttachToVin,
    CameraDetachFromVin,
    CameraDestroy,
    VnodeOpen,
    VnodeSetAttr,
    VnodeSetAttrEx,
    VnodeSetInputChannelAttr,
    VnodeSetOutputChannelAttr,
    VnodeSetOutputChannelBufferAttr,
    VnodeGetFrame,
    VnodeReleaseFrame,
    VnodeClose,
    VflowCreate,
    VflowAddVnode,
    VflowBindVnode,
    VflowStart,
    VflowStop,
    VflowDestroy,
    MediaInitialize,
    MediaSetInputBufferListener,
    MediaConfigure,
    MediaStart,
    MediaStop,
    MediaRelease,
    MediaDequeueInputBuffer,
    MediaQueueInputBuffer,
    MediaDequeueOutputBuffer,
    MediaQueueOutputBuffer,
    MediaGetRateControlConfig,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensorModule {
    Sc132gs,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensorMode {
    Normal,
    Dol2,
    Dol3,
    Dol4,
    Pwl,
    Slave,
    Mono,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensorCalibration {
    Disabled,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MipiDataType {
    Raw10,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrameFormat {
    Raw,
    Nv12,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PixelFormat {
    Nv12,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InputMode {
    Ddr,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IspSensorMode {
    Normal,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VnodeType {
    Vin,
    Isp,
    Gdc,
    Vse,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AllocationId {
    Auto,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VinOutputKind {
    Basic,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MclkAttrKind {
    Static,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HdrMode {
    None,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimestampMode {
    Vsync,
    Trigger,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LpwmTriggerSource {
    Internal,
    Sif,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LpwmTriggerMode {
    Internal,
    External,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GdcFrameFormat {
    Semiplanar420,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GdcTransformation {
    Custom,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CodecId {
    Hevc,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GopPreset {
    SingleReferenceIppp,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Rotation {
    None,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Mirror {
    None,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RateControlMode {
    HevcCbr,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BufferUsage {
    CpuReadOften,
    CpuWriteOften,
    Cached,
    GraphicContiguous,
    HwCim,
    HwIsp,
    HwGdcOutput,
    HwVideoCodec,
    MapInitialized,
    PrivateHeap2Reserved,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BufferUsageSet {
    bits: i64,
}

impl BufferUsageSet {
    pub const fn empty() -> Self {
        Self { bits: 0 }
    }

    pub fn with(mut self, usage: BufferUsage) -> Self {
        self.bits |= usage.raw_bits();
        self
    }
}

impl Default for BufferUsageSet {
    fn default() -> Self {
        Self::empty()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TimestampModeSet {
    bits: u32,
}

impl TimestampModeSet {
    pub fn new(mode: TimestampMode) -> Self {
        Self {
            bits: mode.raw_bits(),
        }
    }

    pub fn with(mut self, mode: TimestampMode) -> Self {
        self.bits |= mode.raw_bits();
        self
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ImageSize {
    pub width: u32,
    pub height: u32,
}

impl ImageSize {
    pub const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FrameRate(NonZeroU16);

impl FrameRate {
    pub fn new(value: u32) -> Result<Self> {
        let value = u16::try_from(value)
            .ok()
            .and_then(NonZeroU16::new)
            .ok_or_else(|| Error::invalid(format!("invalid frame rate: {value}")))?;
        Ok(Self(value))
    }

    pub fn as_u16(self) -> u16 {
        self.0.get()
    }

    pub fn as_u32(self) -> u32 {
        u32::from(self.as_u16())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct I2cAddress(pub u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HardwareId(pub u32);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ChannelId(pub u32);

impl ChannelId {
    pub const PRIMARY: Self = Self(0);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MclkFrequencyHz(pub u32);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MipiLaneCount(pub u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MipiClockIndex(pub u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MipiClockMHz(pub u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LineLength(pub u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FrameLength(pub u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SettleTime(pub u16);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PresentationTimestampUs(pub u64);

#[derive(Clone, Copy, Debug)]
pub struct MipiConfig {
    pub rx_enabled: bool,
    pub lane_count: MipiLaneCount,
    pub data_type: MipiDataType,
    pub frame_rate: FrameRate,
    pub mclk: MipiClockIndex,
    pub mipi_clock: MipiClockMHz,
    pub size: ImageSize,
    pub line_length: LineLength,
    pub frame_length: FrameLength,
    pub settle: SettleTime,
    pub channel: ChannelId,
}

#[derive(Clone, Copy, Debug)]
pub struct SensorGpioConfig {
    pub enable_bit: u32,
    pub level_bit: u32,
}

#[derive(Clone, Debug)]
pub struct CameraConfig {
    pub module: SensorModule,
    pub i2c_address: I2cAddress,
    pub sensor_mode: SensorMode,
    pub frame_rate: FrameRate,
    pub format: MipiDataType,
    pub size: ImageSize,
    pub gpio: SensorGpioConfig,
    pub calibration: SensorCalibration,
    pub mipi: MipiConfig,
}

#[derive(Clone, Copy, Debug)]
pub struct VinAttr {
    pub mipi_rx: HardwareId,
    pub vc_index: ChannelId,
    pub ipi_channel: ChannelId,
    pub isp_flyby: bool,
    pub frame_id: FrameIdConfig,
    pub hdr_mode: HdrMode,
    pub timestamp: TimestampConfig,
    pub lpwm: LpwmConfig,
}

#[derive(Clone, Copy, Debug)]
pub struct FrameIdConfig {
    pub enable: bool,
    pub set_initial: bool,
}

#[derive(Clone, Copy, Debug)]
pub enum TimestampConfig {
    Disabled,
    Enabled {
        modes: TimestampModeSet,
        source: u32,
        pps_source: u32,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct LpwmConfig {
    pub enable: bool,
    pub channels: [LpwmChannelConfig; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct LpwmChannelConfig {
    pub trigger_source: LpwmTriggerSource,
    pub trigger_mode: LpwmTriggerMode,
    pub period_us: u32,
    pub offset_us: u32,
    pub duty_time_us: u32,
    pub threshold: u32,
    pub adjust_step: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct VinInputChannelAttr {
    pub format: MipiDataType,
    pub size: ImageSize,
}

#[derive(Clone, Copy, Debug)]
pub struct VinOutputChannelAttr {
    pub ddr_enabled: bool,
    pub kind: VinOutputKind,
    pub format: MipiDataType,
    pub stride: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct VinMclkAttr {
    pub kind: MclkAttrKind,
    pub frequency: MclkFrequencyHz,
}

#[derive(Clone, Copy, Debug)]
pub struct IspAttr {
    pub input_mode: InputMode,
    pub sensor_mode: IspSensorMode,
    pub crop: Rect,
}

#[derive(Clone, Copy, Debug)]
pub struct IspInputChannelAttr {
    pub size: ImageSize,
    pub format: FrameFormat,
    pub bit_width: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct IspOutputChannelAttr {
    pub ddr_enabled: bool,
    pub format: FrameFormat,
    pub bit_width: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct GdcAttr<'a> {
    pub bin: &'a GdcBin,
    pub total_planes: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct GdcInputChannelAttr {
    pub size: ImageSize,
    pub stride: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct GdcOutputChannelAttr {
    pub size: ImageSize,
    pub stride: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct VseAttr {
    pub frame_rate: FrameRateControl,
}

#[derive(Clone, Copy, Debug)]
pub struct VseInputChannelAttr {
    pub size: ImageSize,
    pub format: FrameFormat,
    pub bit_width: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct VseOutputChannelAttr {
    pub enabled: bool,
    pub roi: Rect,
    pub target_size: ImageSize,
    pub format: FrameFormat,
    pub bit_width: u32,
    pub frame_rate: FrameRateControl,
}

#[derive(Clone, Copy, Debug)]
pub struct FrameRateControl {
    pub source: FrameRate,
    pub target: FrameRate,
}

#[derive(Clone, Copy, Debug)]
pub struct Rect {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct BufferAllocation {
    pub buffer_count: u32,
    pub usages: BufferUsageSet,
    pub contiguous: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

/// Parameters passed to the vendor GDC binary generator for a custom point map.
///
/// Sizes are pixels, angles/FOV values are degrees, strengths/zoom are vendor
/// scalar factors, and tile increments are SDK tile sizes. `camera_driver`
/// currently uses full-size custom maps with semiplanar 4:2:0 output.
#[derive(Clone, Copy, Debug)]
pub struct GdcGenerationConfig {
    pub frame_format: GdcFrameFormat,
    pub input_size: ImageSize,
    pub output_size: ImageSize,
    pub diameter: i32,
    pub field_of_view: f64,
    pub transformation: GdcTransformation,
    pub strength: f64,
    pub strength_y: f64,
    pub zoom: f64,
    pub keep_ratio: bool,
    pub horizontal_field_of_view: f64,
    pub vertical_field_of_view: f64,
    pub trapezoid_left_angle: f64,
    pub trapezoid_right_angle: f64,
    pub tile_increment_x: u16,
    pub tile_increment_y: u16,
}

/// MediaCodec encoder parameters for HEVC NV12 input.
#[derive(Clone, Copy, Debug)]
pub struct EncoderConfig {
    pub codec: CodecId,
    pub size: ImageSize,
    pub pixel_format: PixelFormat,
    pub external_frame_input: bool,
    pub frame_buffer_count: u32,
    pub bitstream_buffer_count: u32,
    pub bitstream_buffer_size: u32,
    pub gop_preset: GopPreset,
    pub rotation: Rotation,
    pub mirror: Mirror,
    pub enable_user_pts: bool,
    pub rate_control: HevcCbrConfig,
}

/// HEVC constant-bitrate rate-control parameters.
///
/// Numeric fields map to the vendor H.265 CBR structure. Bit rate is kilobits
/// per second, frame rate is frames per second, and QP values use the SDK's
/// H.265 quantizer range.
#[derive(Clone, Copy, Debug)]
pub struct HevcCbrConfig {
    pub intra_period: u32,
    pub intra_qp: u32,
    pub bit_rate_kbps: u32,
    pub frame_rate: FrameRate,
    pub initial_rc_qp: u32,
    pub vbv_buffer_size: u32,
    pub ctu_level_rc_enable: bool,
    pub min_qp_i: u32,
    pub max_qp_i: u32,
    pub min_qp_p: u32,
    pub max_qp_p: u32,
    pub min_qp_b: u32,
    pub max_qp_b: u32,
    pub hvs_qp_enable: bool,
    pub hvs_qp_scale: u32,
    pub max_delta_qp: u32,
    pub qp_map_enable: bool,
}

#[cfg(x5cam_x5_target)]
/// Process-wide hbmem module guard required before hbmem-backed SDK allocations.
pub struct MemoryModule;

#[cfg(x5cam_x5_target)]
impl MemoryModule {
    pub fn open() -> Result<Self> {
        call_hbn(Operation::HbMemModuleOpen, unsafe {
            raw::hb_mem_module_open()
        })?;
        Ok(Self)
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for MemoryModule {
    fn drop(&mut self) {
        unsafe {
            raw::hb_mem_module_close();
        }
    }
}

#[cfg(x5cam_x5_target)]
/// hbmem common buffer owned by this crate and released on drop.
pub struct CommonBuffer {
    raw: raw::hb_mem_common_buf_t,
}

#[cfg(x5cam_x5_target)]
impl CommonBuffer {
    fn from_raw(raw: raw::hb_mem_common_buf_t) -> Self {
        Self { raw }
    }
}

#[cfg(x5cam_x5_target)]
impl fmt::Debug for CommonBuffer {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("CommonBuffer")
            .field("fd", &self.raw.fd)
            .field("size", &self.raw.size)
            .finish_non_exhaustive()
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for CommonBuffer {
    fn drop(&mut self) {
        if self.raw.fd >= 0 {
            unsafe {
                raw::hb_mem_free_buf(self.raw.fd);
            }
            self.raw.fd = -1;
        }
    }
}

#[cfg(x5cam_x5_target)]
/// hbmem-backed GDC binary generated from a custom point map.
pub struct GdcBin {
    buffer: CommonBuffer,
}

#[cfg(not(x5cam_x5_target))]
#[derive(Debug)]
/// Host-only opaque placeholder so public config types compile without X5 headers.
pub struct GdcBin;

#[cfg(x5cam_x5_target)]
impl fmt::Debug for GdcBin {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("GdcBin")
            .field("buffer", &self.buffer)
            .finish()
    }
}

#[cfg(x5cam_x5_target)]
impl GdcBin {
    pub fn generate_custom(points: &mut [Point], config: GdcGenerationConfig) -> Result<Self> {
        if points.len() != config.output_size.width as usize * config.output_size.height as usize {
            return Err(Error::invalid(format!(
                "GDC point count mismatch: got {}, expected {}",
                points.len(),
                config.output_size.width as usize * config.output_size.height as usize
            )));
        }

        assert_eq!(
            std::mem::size_of::<Point>(),
            std::mem::size_of::<raw::point_t>()
        );
        assert_eq!(
            std::mem::align_of::<Point>(),
            std::mem::align_of::<raw::point_t>()
        );

        unsafe {
            let mut param: raw::param_t = std::mem::zeroed();
            param.format = config.frame_format.raw();
            param.in_.w = config.input_size.width;
            param.in_.h = config.input_size.height;
            param.out.w = config.output_size.width;
            param.out.h = config.output_size.height;
            param.diameter = config.diameter;
            param.fov = config.field_of_view;

            let mut window: raw::window_t = std::mem::zeroed();
            window.out_r.x = 0;
            window.out_r.y = 0;
            window.out_r.w = config.output_size.width as i32;
            window.out_r.h = config.output_size.height as i32;
            window.transform = config.transformation.raw();
            window.input_roi_r.x = 0;
            window.input_roi_r.y = 0;
            window.input_roi_r.w = config.input_size.width as i32;
            window.input_roi_r.h = config.input_size.height as i32;
            window.strength = config.strength;
            window.strengthY = config.strength_y;
            window.zoom = config.zoom;
            window.keep_ratio = i32::from(config.keep_ratio);
            window.FOV_h = config.horizontal_field_of_view;
            window.FOV_w = config.vertical_field_of_view;
            window.trapezoid_left_angle = config.trapezoid_left_angle;
            window.trapezoid_right_angle = config.trapezoid_right_angle;
            window.custom.full_tile_calc = 1;
            window.custom.tile_incr_x = config.tile_increment_x;
            window.custom.tile_incr_y = config.tile_increment_y;
            window.custom.w = config.output_size.width as i32 - 1;
            window.custom.h = config.output_size.height as i32 - 1;
            window.custom.centerx = config.output_size.width as f64 / 2.0 - 1.0;
            window.custom.centery = config.output_size.height as f64 / 2.0 - 1.0;
            window.custom.points = points.as_mut_ptr().cast::<raw::point_t>();

            let mut raw_buf: *mut u32 = std::ptr::null_mut();
            let mut raw_size = 0u64;
            let ret = raw::hbn_gen_gdc_bin(&param, &window, 1, &mut raw_buf, &mut raw_size);
            if ret != 0 || raw_buf.is_null() || raw_size == 0 {
                return Err(Error::Sdk {
                    operation: Operation::GenerateGdcBin,
                    ret,
                    info: format!(" size={raw_size}"),
                });
            }

            let mut bin_buf: raw::hb_mem_common_buf_t = std::mem::zeroed();
            let flags = BufferUsageSet::empty()
                .with(BufferUsage::MapInitialized)
                .with(BufferUsage::PrivateHeap2Reserved)
                .with(BufferUsage::CpuReadOften)
                .with(BufferUsage::CpuWriteOften)
                .with(BufferUsage::Cached);
            let ret = raw::hb_mem_alloc_com_buf(raw_size, flags.bits, &mut bin_buf);
            if ret != 0 || bin_buf.virt_addr.is_null() {
                raw::hbn_free_gdc_bin(raw_buf);
                return Err(sdk_error(Operation::HbMemAllocCommonBuffer, ret));
            }

            std::ptr::copy_nonoverlapping(
                raw_buf.cast::<u8>(),
                bin_buf.virt_addr,
                raw_size as usize,
            );
            raw::hbn_free_gdc_bin(raw_buf);
            let ret = raw::hb_mem_flush_buf(bin_buf.fd, 0, raw_size);
            if ret != 0 {
                raw::hb_mem_free_buf(bin_buf.fd);
                return Err(sdk_error(Operation::HbMemFlushBuffer, ret));
            }

            Ok(Self {
                buffer: CommonBuffer::from_raw(bin_buf),
            })
        }
    }
}

#[cfg(x5cam_x5_target)]
/// Camera sensor handle created from a stable SDK camera configuration.
pub struct Camera {
    handle: Option<raw::camera_handle_t>,
    _config: CameraConfigStorage,
    attached: bool,
}

#[cfg(x5cam_x5_target)]
unsafe impl Send for Camera {}

#[cfg(x5cam_x5_target)]
struct CameraConfigStorage {
    _mipi: Box<raw::mipi_config_t>,
    camera: Box<raw::camera_config_t>,
}

#[cfg(x5cam_x5_target)]
impl Camera {
    pub fn create(config: CameraConfig) -> Result<Self> {
        let mut storage = CameraConfigStorage::new(config)?;
        let mut handle = 0;
        call_hbn(Operation::CameraCreate, unsafe {
            raw::hbn_camera_create(&mut *storage.camera, &mut handle)
        })?;
        Ok(Self {
            handle: Some(handle),
            _config: storage,
            attached: false,
        })
    }

    pub fn attach_to_vin(&mut self, vin: &VinNode) -> Result<()> {
        let handle = self
            .handle
            .ok_or_else(|| Error::other("camera handle missing"))?;
        call_hbn(Operation::CameraAttachToVin, unsafe {
            raw::hbn_camera_attach_to_vin(handle, vin.inner.handle)
        })?;
        self.attached = true;
        Ok(())
    }

    pub fn detach_from_vin(&mut self) -> Result<()> {
        if self.attached {
            if let Some(handle) = self.handle {
                call_hbn(Operation::CameraDetachFromVin, unsafe {
                    raw::hbn_camera_detach_from_vin(handle)
                })?;
            }
            self.attached = false;
        }
        Ok(())
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for Camera {
    fn drop(&mut self) {
        let _ = self.detach_from_vin();
        if let Some(handle) = self.handle.take() {
            unsafe {
                raw::hbn_camera_destroy(handle);
            }
        }
    }
}

#[cfg(x5cam_x5_target)]
impl CameraConfigStorage {
    fn new(config: CameraConfig) -> Result<Self> {
        let mut mipi = Box::new(raw::mipi_config_t::default());
        mipi.rx_enable = i32::from(config.mipi.rx_enabled);
        mipi.rx_attr.lane = config.mipi.lane_count.0;
        mipi.rx_attr.datatype = config.mipi.data_type.raw() as u16;
        mipi.rx_attr.fps = config.mipi.frame_rate.as_u16();
        mipi.rx_attr.mclk = config.mipi.mclk.0;
        mipi.rx_attr.mipiclk = config.mipi.mipi_clock.0;
        mipi.rx_attr.width = u16_from_u32(config.mipi.size.width, "MIPI width")?;
        mipi.rx_attr.height = u16_from_u32(config.mipi.size.height, "MIPI height")?;
        mipi.rx_attr.linelenth = config.mipi.line_length.0;
        mipi.rx_attr.framelenth = config.mipi.frame_length.0;
        mipi.rx_attr.settle = config.mipi.settle.0;
        mipi.rx_attr.channel_num = 1;
        mipi.rx_attr.channel_sel[0] = u16_from_u32(config.mipi.channel.0, "MIPI channel")?;

        let mut camera = Box::new(raw::camera_config_t::default());
        set_c_string(&mut camera.name, config.module.sdk_name())?;
        set_c_string(&mut camera.calib_lname, config.calibration.sdk_name())?;
        camera.addr = config.i2c_address.0 as u32;
        camera.sensor_mode = config.sensor_mode.raw();
        camera.fps = config.frame_rate.as_u32();
        camera.format = config.format.raw();
        camera.width = config.size.width;
        camera.height = config.size.height;
        camera.gpio_enable_bit = config.gpio.enable_bit;
        camera.gpio_level_bit = config.gpio.level_bit;
        camera.mipi_cfg = &mut *mipi;

        Ok(Self {
            _mipi: mipi,
            camera,
        })
    }
}

#[cfg(x5cam_x5_target)]
/// Marker type for a VIN vnode.
pub enum Vin {}

#[cfg(x5cam_x5_target)]
/// Marker type for an ISP vnode.
pub enum Isp {}

#[cfg(x5cam_x5_target)]
/// Marker type for a GDC vnode.
pub enum Gdc {}

#[cfg(x5cam_x5_target)]
/// Marker type for a VSE vnode.
pub enum Vse {}

#[cfg(x5cam_x5_target)]
/// Type-safe VIN vnode handle.
pub type VinNode = Vnode<Vin>;

#[cfg(x5cam_x5_target)]
/// Type-safe ISP vnode handle.
pub type IspNode = Vnode<Isp>;

#[cfg(x5cam_x5_target)]
/// Type-safe GDC vnode handle.
pub type GdcNode = Vnode<Gdc>;

#[cfg(x5cam_x5_target)]
/// Type-safe VSE vnode handle.
pub type VseNode = Vnode<Vse>;

#[cfg(x5cam_x5_target)]
/// Owned X5 vnode handle. The type parameter restricts valid operations by node kind.
pub struct Vnode<K> {
    inner: Arc<VnodeInner>,
    _kind: PhantomData<K>,
}

#[cfg(x5cam_x5_target)]
struct VnodeInner {
    handle: raw::hbn_vnode_handle_t,
}

#[cfg(x5cam_x5_target)]
unsafe impl Send for VnodeInner {}

#[cfg(x5cam_x5_target)]
unsafe impl Sync for VnodeInner {}

#[cfg(x5cam_x5_target)]
unsafe impl<K> Send for Vnode<K> {}

#[cfg(x5cam_x5_target)]
impl VinNode {
    pub fn open_vin(hardware_id: HardwareId, allocation_id: AllocationId) -> Result<Self> {
        Self::open(VnodeType::Vin, hardware_id, allocation_id)
    }

    pub fn set_vin_attr(&mut self, attr: VinAttr) -> Result<()> {
        let mut raw_attr = raw::vin_node_attr_t::default();
        raw_attr.cim_attr.mipi_rx = attr.mipi_rx.0;
        raw_attr.cim_attr.vc_index = attr.vc_index.0;
        raw_attr.cim_attr.ipi_channel = attr.ipi_channel.0;
        raw_attr.cim_attr.cim_isp_flyby = u32::from(attr.isp_flyby);
        raw_attr.cim_attr.func.enable_frame_id = u32::from(attr.frame_id.enable);
        raw_attr.cim_attr.func.set_init_frame_id = u32::from(attr.frame_id.set_initial);
        raw_attr.cim_attr.func.hdr_mode = attr.hdr_mode.raw();
        match attr.timestamp {
            TimestampConfig::Disabled => {
                raw_attr.cim_attr.func.time_stamp_en = 0;
                raw_attr.cim_attr.func.time_stamp_mode = 0;
                raw_attr.cim_attr.func.ts_src = 0;
                raw_attr.cim_attr.func.pps_src = 0;
            }
            TimestampConfig::Enabled {
                modes,
                source,
                pps_source,
            } => {
                raw_attr.cim_attr.func.time_stamp_en = 1;
                raw_attr.cim_attr.func.time_stamp_mode = modes.raw();
                raw_attr.cim_attr.func.ts_src = source;
                raw_attr.cim_attr.func.pps_src = pps_source;
            }
        }
        raw_attr.lpwm_attr.enable = u32::from(attr.lpwm.enable);
        for (raw_channel, channel) in raw_attr
            .lpwm_attr
            .lpwm_chn_attr
            .iter_mut()
            .zip(attr.lpwm.channels)
        {
            raw_channel.trigger_source = channel.trigger_source.raw();
            raw_channel.trigger_mode = channel.trigger_mode.raw();
            raw_channel.period = channel.period_us;
            raw_channel.offset = channel.offset_us;
            raw_channel.duty_time = channel.duty_time_us;
            raw_channel.threshold = channel.threshold;
            raw_channel.adjust_step = channel.adjust_step;
        }
        self.set_attr(&mut raw_attr)
    }

    pub fn set_vin_input_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: VinInputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::vin_ichn_attr_t {
            format: attr.format.raw(),
            width: attr.size.width,
            height: attr.size.height,
        };
        self.set_ichn_attr(channel, &mut raw_attr)
    }

    pub fn set_vin_output_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: VinOutputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::vin_ochn_attr_t {
            ddr_en: u32::from(attr.ddr_enabled),
            ochn_attr_type: attr.kind.raw(),
            vin_basic_attr: raw::vin_basic_attr_t {
                format: attr.format.raw(),
                wstride: attr.stride,
                ..Default::default()
            },
            ..Default::default()
        };
        self.set_ochn_attr(channel, &mut raw_attr)
    }

    pub fn set_vin_mclk_attr(&mut self, attr: VinMclkAttr) -> Result<()> {
        let mclk_freq = i32::try_from(attr.frequency.0).map_err(|_| {
            Error::invalid(format!("MCLK frequency too large: {} Hz", attr.frequency.0))
        })?;
        let mut raw_attr = raw::vin_attr_ex_t {
            ex_attr_type: attr.kind.raw(),
            mclk_ex_attr: raw::mclk_attr_ex_t { mclk_freq },
            vin_attr_ex_mask: 0x80,
            ..Default::default()
        };
        call_hbn(Operation::VnodeSetAttrEx, unsafe {
            raw::hbn_vnode_set_attr_ex(self.inner.handle, as_mut_void(&mut raw_attr))
        })
    }
}

#[cfg(x5cam_x5_target)]
impl IspNode {
    pub fn open_isp(hardware_id: HardwareId, allocation_id: AllocationId) -> Result<Self> {
        Self::open(VnodeType::Isp, hardware_id, allocation_id)
    }

    pub fn set_isp_attr(&mut self, attr: IspAttr) -> Result<()> {
        let mut raw_attr = raw::isp_attr_t {
            input_mode: attr.input_mode.raw(),
            sensor_mode: attr.sensor_mode.raw(),
            crop: attr.crop.raw(),
            ..Default::default()
        };
        self.set_attr(&mut raw_attr)
    }

    pub fn set_isp_input_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: IspInputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::isp_ichn_attr_t {
            width: attr.size.width,
            height: attr.size.height,
            fmt: attr.format.raw(),
            bit_width: attr.bit_width,
            ..Default::default()
        };
        self.set_ichn_attr(channel, &mut raw_attr)
    }

    pub fn set_isp_output_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: IspOutputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::isp_ochn_attr_t {
            ddr_en: attr.ddr_enabled.raw_bool(),
            fmt: attr.format.raw(),
            bit_width: attr.bit_width,
            ..Default::default()
        };
        self.set_ochn_attr(channel, &mut raw_attr)
    }
}

#[cfg(x5cam_x5_target)]
impl GdcNode {
    pub fn open_gdc(hardware_id: HardwareId, allocation_id: AllocationId) -> Result<Self> {
        Self::open(VnodeType::Gdc, hardware_id, allocation_id)
    }

    pub fn set_gdc_attr(&mut self, attr: GdcAttr<'_>) -> Result<()> {
        let config_size = u32::try_from(attr.bin.buffer.raw.size).map_err(|_| {
            Error::invalid(format!(
                "GDC bin too large: {} bytes",
                attr.bin.buffer.raw.size
            ))
        })?;
        let mut raw_attr = raw::gdc_attr_t {
            config_addr: attr.bin.buffer.raw.phys_addr,
            config_size,
            total_planes: attr.total_planes,
            binary_ion_id: attr.bin.buffer.raw.share_id,
            binary_offset: attr.bin.buffer.raw.offset,
            ..Default::default()
        };
        self.set_attr(&mut raw_attr)
    }

    pub fn set_gdc_input_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: GdcInputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::gdc_ichn_attr_t {
            input_width: attr.size.width,
            input_height: attr.size.height,
            input_stride: attr.stride,
            ..Default::default()
        };
        self.set_ichn_attr(channel, &mut raw_attr)
    }

    pub fn set_gdc_output_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: GdcOutputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::gdc_ochn_attr_t {
            output_width: attr.size.width,
            output_height: attr.size.height,
            output_stride: attr.stride,
        };
        self.set_ochn_attr(channel, &mut raw_attr)
    }
}

#[cfg(x5cam_x5_target)]
impl VseNode {
    pub fn open_vse(hardware_id: HardwareId, allocation_id: AllocationId) -> Result<Self> {
        Self::open(VnodeType::Vse, hardware_id, allocation_id)
    }

    pub fn set_vse_attr(&mut self, attr: VseAttr) -> Result<()> {
        let mut raw_attr = raw::vse_attr_t::default();
        raw_attr.fps = attr.frame_rate.raw();
        self.set_attr(&mut raw_attr)
    }

    pub fn set_vse_input_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: VseInputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::vse_ichn_attr_t {
            width: attr.size.width,
            height: attr.size.height,
            fmt: attr.format.raw(),
            bit_width: attr.bit_width,
            ..Default::default()
        };
        self.set_ichn_attr(channel, &mut raw_attr)
    }

    pub fn set_vse_output_channel_attr(
        &mut self,
        channel: ChannelId,
        attr: VseOutputChannelAttr,
    ) -> Result<()> {
        let mut raw_attr = raw::vse_ochn_attr_t {
            chn_en: attr.enabled.raw_bool(),
            roi: attr.roi.raw(),
            target_w: attr.target_size.width,
            target_h: attr.target_size.height,
            fmt: attr.format.raw(),
            bit_width: attr.bit_width,
            fps: attr.frame_rate.raw(),
            ..Default::default()
        };
        self.set_ochn_attr(channel, &mut raw_attr)
    }

    pub fn get_frame(&mut self, channel: ChannelId, timeout_ms: u32) -> Result<FrameLease> {
        let mut image = raw::hbn_vnode_image_t::default();
        call_hbn(Operation::VnodeGetFrame, unsafe {
            raw::hbn_vnode_getframe(self.inner.handle, channel.0, timeout_ms, &mut image)
        })?;
        Ok(FrameLease {
            vnode: self.inner.clone(),
            channel: channel.0,
            image,
            released: false,
        })
    }
}

#[cfg(x5cam_x5_target)]
impl<K> Vnode<K> {
    fn open(
        node_type: VnodeType,
        hardware_id: HardwareId,
        allocation_id: AllocationId,
    ) -> Result<Self> {
        let mut handle = 0;
        call_hbn(Operation::VnodeOpen, unsafe {
            raw::hbn_vnode_open(
                node_type.raw(),
                hardware_id.0,
                allocation_id.raw(),
                &mut handle,
            )
        })?;
        Ok(Self {
            inner: Arc::new(VnodeInner { handle }),
            _kind: PhantomData,
        })
    }

    pub fn set_output_channel_buffer_attr(
        &mut self,
        channel: ChannelId,
        attr: BufferAllocation,
    ) -> Result<()> {
        let mut raw_attr = raw::hbn_buf_alloc_attr_t {
            flags: attr
                .usages
                .with(BufferUsage::CpuReadOften)
                .with(BufferUsage::CpuWriteOften)
                .with(BufferUsage::Cached)
                .bits,
            buffers_num: attr.buffer_count,
            is_contig: u32::from(attr.contiguous),
        };
        call_hbn(Operation::VnodeSetOutputChannelBufferAttr, unsafe {
            raw::hbn_vnode_set_ochn_buf_attr(self.inner.handle, channel.0, &mut raw_attr)
        })
    }

    fn set_attr<T>(&mut self, attr: &mut T) -> Result<()> {
        call_hbn(Operation::VnodeSetAttr, unsafe {
            raw::hbn_vnode_set_attr(self.inner.handle, as_mut_void(attr))
        })
    }

    fn set_ichn_attr<T>(&mut self, channel: ChannelId, attr: &mut T) -> Result<()> {
        call_hbn(Operation::VnodeSetInputChannelAttr, unsafe {
            raw::hbn_vnode_set_ichn_attr(self.inner.handle, channel.0, as_mut_void(attr))
        })
    }

    fn set_ochn_attr<T>(&mut self, channel: ChannelId, attr: &mut T) -> Result<()> {
        call_hbn(Operation::VnodeSetOutputChannelAttr, unsafe {
            raw::hbn_vnode_set_ochn_attr(self.inner.handle, channel.0, as_mut_void(attr))
        })
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for VnodeInner {
    fn drop(&mut self) {
        unsafe {
            raw::hbn_vnode_close(self.handle);
        }
    }
}

#[cfg(x5cam_x5_target)]
/// Vflow graph that owns VIN/ISP/GDC/VSE bindings and stops itself on drop.
pub struct Vflow {
    handle: raw::hbn_vflow_handle_t,
    started: bool,
}

#[cfg(x5cam_x5_target)]
unsafe impl Send for Vflow {}

#[cfg(x5cam_x5_target)]
impl Vflow {
    pub fn create() -> Result<Self> {
        let mut handle = 0;
        call_hbn(Operation::VflowCreate, unsafe {
            raw::hbn_vflow_create(&mut handle)
        })?;
        Ok(Self {
            handle,
            started: false,
        })
    }

    pub fn add_vnode<K>(&mut self, vnode: &Vnode<K>) -> Result<()> {
        call_hbn(Operation::VflowAddVnode, unsafe {
            raw::hbn_vflow_add_vnode(self.handle, vnode.inner.handle)
        })
    }

    pub fn bind(
        &mut self,
        source: &Vnode<impl Sized>,
        source_channel: ChannelId,
        target: &Vnode<impl Sized>,
        target_channel: ChannelId,
    ) -> Result<()> {
        call_hbn(Operation::VflowBindVnode, unsafe {
            raw::hbn_vflow_bind_vnode(
                self.handle,
                source.inner.handle,
                source_channel.0,
                target.inner.handle,
                target_channel.0,
            )
        })
    }

    pub fn start(&mut self) -> Result<()> {
        call_hbn(Operation::VflowStart, unsafe {
            raw::hbn_vflow_start(self.handle)
        })?;
        self.started = true;
        Ok(())
    }

    pub fn stop(&mut self) -> Result<()> {
        if self.started {
            call_hbn(Operation::VflowStop, unsafe {
                raw::hbn_vflow_stop(self.handle)
            })?;
            self.started = false;
        }
        Ok(())
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for Vflow {
    fn drop(&mut self) {
        let _ = self.stop();
        unsafe {
            raw::hbn_vflow_destroy(self.handle);
        }
    }
}

#[cfg(x5cam_x5_target)]
/// Lease for one VSE output frame. Dropping it releases the frame to the SDK.
pub struct FrameLease {
    vnode: Arc<VnodeInner>,
    channel: u32,
    image: raw::hbn_vnode_image_t,
    released: bool,
}

#[cfg(x5cam_x5_target)]
unsafe impl Send for FrameLease {}

#[cfg(x5cam_x5_target)]
impl FrameLease {
    pub fn frame_id(&self) -> u32 {
        self.image.info.frame_id
    }

    pub fn timestamp_ns(&self) -> u64 {
        self.image.info.timestamps
    }

    pub fn trigger_timestamp_ns(&self) -> Option<u64> {
        timeval_to_nanos(self.image.info.trig_tv)
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for FrameLease {
    fn drop(&mut self) {
        if self.released {
            return;
        }
        unsafe {
            raw::hbn_vnode_releaseframe(self.vnode.handle, self.channel, &mut self.image);
        }
        self.released = true;
    }
}

#[cfg(x5cam_x5_target)]
/// X5 MediaCodec encoder configured by typed `EncoderConfig`.
pub struct MediaEncoder {
    ctx: Box<raw::media_codec_context_t>,
    _callback: raw::media_codec_callback_t,
    input_release_tx: mpsc::Sender<QueuedInputFrame>,
    input_release_rx: mpsc::Receiver<QueuedInputFrame>,
    pending_input_count: usize,
    output_release_tx: mpsc::Sender<EncodedOutputBuffer>,
    output_release_rx: mpsc::Receiver<EncodedOutputBuffer>,
    pending_output_count: usize,
    initialized: bool,
    started: bool,
}

#[cfg(x5cam_x5_target)]
unsafe impl Send for MediaEncoder {}

#[cfg(x5cam_x5_target)]
struct EncodedOutputBuffer(raw::media_codec_buffer_t);

#[cfg(x5cam_x5_target)]
unsafe impl Send for EncodedOutputBuffer {}

#[cfg(x5cam_x5_target)]
struct QueuedInputFrame {
    _lease: FrameLease,
    release: Option<mpsc::Sender<QueuedInputFrame>>,
}

#[cfg(x5cam_x5_target)]
/// Borrowed encoded HEVC bytes backed by an SDK output buffer.
///
/// Dropping this value returns the underlying output buffer through the encoder's
/// release channel; callers must not retain slices beyond this value's lifetime.
pub struct EncodedFrameData {
    ptr: NonNull<u8>,
    len: usize,
    release: Option<EncodedOutputRelease>,
}

#[cfg(x5cam_x5_target)]
unsafe impl Send for EncodedFrameData {}

#[cfg(x5cam_x5_target)]
struct EncodedOutputRelease {
    buffer: EncodedOutputBuffer,
    releaser: mpsc::Sender<EncodedOutputBuffer>,
}

#[cfg(x5cam_x5_target)]
#[derive(Debug)]
pub struct EncodedOutput {
    pub data: EncodedFrameData,
    pub pts_us: u64,
}

#[cfg(x5cam_x5_target)]
impl EncodedFrameData {
    fn new(output: raw::media_codec_buffer_t, releaser: mpsc::Sender<EncodedOutputBuffer>) -> Self {
        let stream = unsafe { output.__bindgen_anon_1.vstream_buf };
        debug_assert!(!stream.vir_ptr.is_null());
        // `dequeue_output` checks the stream pointer before constructing this lease.
        let ptr = unsafe { NonNull::new_unchecked(stream.vir_ptr.cast::<u8>()) };
        let len = stream.size as usize;
        Self {
            ptr,
            len,
            release: Some(EncodedOutputRelease {
                buffer: EncodedOutputBuffer(output),
                releaser,
            }),
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr.as_ptr(), self.len) }
    }
}

#[cfg(x5cam_x5_target)]
impl AsRef<[u8]> for EncodedFrameData {
    fn as_ref(&self) -> &[u8] {
        self.as_slice()
    }
}

#[cfg(x5cam_x5_target)]
impl std::ops::Deref for EncodedFrameData {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

#[cfg(x5cam_x5_target)]
impl fmt::Debug for EncodedFrameData {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("EncodedFrameData")
            .field("len", &self.len)
            .finish()
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for EncodedFrameData {
    fn drop(&mut self) {
        if let Some(release) = self.release.take() {
            let _ = release.releaser.send(release.buffer);
        }
    }
}

#[cfg(x5cam_x5_target)]
impl MediaEncoder {
    /// Creates, configures, and starts an X5 MediaCodec encoder.
    ///
    /// For external-frame input, queue VSE `FrameLease`s with
    /// `queue_external_frame`, periodically call `release_pending_inputs` on the
    /// capture thread, dequeue encoded output with `dequeue_output`, and call
    /// `release_pending_outputs` after dropping returned `EncodedFrameData`.
    pub fn create(config: EncoderConfig) -> Result<Self> {
        let (input_release_tx, input_release_rx) = mpsc::channel();
        let (output_release_tx, output_release_rx) = mpsc::channel();
        let mut encoder = Self {
            ctx: Box::new(raw::media_codec_context_t::default()),
            _callback: raw::media_codec_callback_t::default(),
            input_release_tx,
            input_release_rx,
            pending_input_count: 0,
            output_release_tx,
            output_release_rx,
            pending_output_count: 0,
            initialized: false,
            started: false,
        };
        unsafe {
            encoder.configure_params(config)?;
            call_media(
                Operation::MediaInitialize,
                raw::hb_mm_mc_initialize(&mut *encoder.ctx),
            )?;
            encoder.initialized = true;

            encoder._callback.on_input_buffer_consumed = Some(on_input_buffer_consumed);
            call_media(
                Operation::MediaSetInputBufferListener,
                raw::hb_mm_mc_set_input_buffer_listener(
                    &mut *encoder.ctx,
                    &encoder._callback,
                    std::ptr::null_mut(),
                ),
            )?;

            call_media(
                Operation::MediaConfigure,
                raw::hb_mm_mc_configure(&mut *encoder.ctx),
            )?;
            let startup = raw::mc_av_codec_startup_params_t::default();
            call_media(
                Operation::MediaStart,
                raw::hb_mm_mc_start(&mut *encoder.ctx, &startup),
            )?;
            encoder.started = true;
        }
        Ok(encoder)
    }

    pub fn external_frame_input(&self) -> bool {
        unsafe {
            self.ctx
                .__bindgen_anon_1
                .video_enc_params
                .external_frame_buf
                != 0
        }
    }

    /// Queues one leased NV12 VSE frame as encoder input.
    ///
    /// The lease stays owned by the encoder until MediaCodec reports the input
    /// buffer as consumed. Call `release_pending_inputs` or
    /// `wait_input_consumed` from the same thread that owns the VSE pipeline to
    /// return those frame leases to VIO.
    pub fn queue_external_frame(
        &mut self,
        lease: FrameLease,
        pts: PresentationTimestampUs,
    ) -> Result<()> {
        let buffer = &lease.image.buffer;
        if buffer.virt_addr[0].is_null()
            || buffer.virt_addr[1].is_null()
            || buffer.phys_addr[0] == 0
            || buffer.phys_addr[1] == 0
        {
            return Err(Error::invalid(format!(
                "VSE frame has invalid NV12 planes: vir={:?} phy={:?}",
                &buffer.virt_addr[..2],
                &buffer.phys_addr[..2]
            )));
        }

        unsafe {
            let mut input = raw::media_codec_buffer_t {
                type_: raw::_media_codec_buffer_type_MC_VIDEO_FRAME_BUFFER,
                ..Default::default()
            };
            call_media(
                Operation::MediaDequeueInputBuffer,
                raw::hb_mm_mc_dequeue_input_buffer(&mut *self.ctx, &mut input, 2_000),
            )?;

            let frame = &mut input.__bindgen_anon_1.vframe_buf;
            frame.vir_ptr[0] = buffer.virt_addr[0];
            frame.vir_ptr[1] = buffer.virt_addr[1];
            frame.phy_ptr[0] = buffer.phys_addr[0];
            frame.phy_ptr[1] = buffer.phys_addr[1];
            frame.fd[0] = buffer.fd[0];
            frame.fd[1] = buffer.fd[1];
            frame.compSize[0] = buffer.size[0] as u32;
            frame.compSize[1] = buffer.size[1] as u32;
            frame.size = (buffer.size[0] + buffer.size[1]) as u32;
            frame.width = self.ctx.__bindgen_anon_1.video_enc_params.width;
            frame.height = self.ctx.__bindgen_anon_1.video_enc_params.height;
            frame.pix_fmt = PixelFormat::Nv12.raw();
            frame.stride = buffer.stride;
            frame.vstride = buffer.vstride;
            frame.pts = pts.0;
            frame.frame_end = 0;

            let queued = Box::new(QueuedInputFrame {
                _lease: lease,
                release: Some(self.input_release_tx.clone()),
            });
            let raw_queued = Box::into_raw(queued);
            input.user_ptr = raw_queued.cast();
            let ret = raw::hb_mm_mc_queue_input_buffer(&mut *self.ctx, &mut input, 2_000);
            if ret != 0 {
                let mut queued = Box::from_raw(raw_queued);
                queued.release = None;
                drop(queued);
                let error = Error::Media {
                    operation: Operation::MediaQueueInputBuffer,
                    ret,
                };
                if let Err(shutdown_error) = self.shutdown(Duration::from_secs(2)) {
                    return Err(Error::Cleanup {
                        primary: Box::new(error),
                        cleanup: Box::new(shutdown_error),
                    });
                }
                return Err(error);
            }
            self.pending_input_count += 1;
        }
        Ok(())
    }

    /// Dequeues one encoded output buffer, if available before `timeout_ms`.
    ///
    /// The returned `EncodedFrameData` borrows the SDK output buffer. Drop it,
    /// then call `release_pending_outputs` or `wait_release_output` to return
    /// the buffer to MediaCodec.
    pub fn dequeue_output(&mut self, timeout_ms: i32) -> Result<Option<EncodedOutput>> {
        unsafe {
            let mut output = raw::media_codec_buffer_t::default();
            let mut info = raw::media_codec_output_buffer_info_t::default();
            let ret = raw::hb_mm_mc_dequeue_output_buffer(
                &mut *self.ctx,
                &mut output,
                &mut info,
                timeout_ms,
            );
            if ret == raw::HB_MEDIA_ERR_WAIT_TIMEOUT {
                return Ok(None);
            }
            if ret != 0 {
                return Err(Error::Media {
                    operation: Operation::MediaDequeueOutputBuffer,
                    ret,
                });
            }
            if output.type_ != raw::_media_codec_buffer_type_MC_VIDEO_STREAM_BUFFER {
                call_media(
                    Operation::MediaQueueOutputBuffer,
                    raw::hb_mm_mc_queue_output_buffer(&mut *self.ctx, &mut output, 0),
                )?;
                return Ok(None);
            }
            let stream = output.__bindgen_anon_1.vstream_buf;
            let encoded = if stream.size == 0 || stream.vir_ptr.is_null() {
                call_media(
                    Operation::MediaQueueOutputBuffer,
                    raw::hb_mm_mc_queue_output_buffer(&mut *self.ctx, &mut output, 0),
                )?;
                None
            } else {
                self.pending_output_count += 1;
                Some(EncodedOutput {
                    data: EncodedFrameData::new(output, self.output_release_tx.clone()),
                    pts_us: stream.pts,
                })
            };
            Ok(encoded)
        }
    }

    /// Releases all input frame leases already consumed by MediaCodec.
    pub fn release_pending_inputs(&mut self) -> usize {
        let mut released = 0;
        while let Ok(_queued) = self.input_release_rx.try_recv() {
            self.pending_input_count = self.pending_input_count.saturating_sub(1);
            released += 1;
        }
        released
    }

    pub fn pending_input_count(&self) -> usize {
        self.pending_input_count
    }

    /// Waits for one queued input frame to be consumed and releases its VIO lease.
    pub fn wait_input_consumed(&mut self, timeout: Duration) -> Result<bool> {
        if self.pending_input_count == 0 {
            return Ok(true);
        }
        match self.input_release_rx.recv_timeout(timeout) {
            Ok(_queued) => {
                self.pending_input_count = self.pending_input_count.saturating_sub(1);
                Ok(true)
            }
            Err(mpsc::RecvTimeoutError::Timeout) => Ok(false),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                Err(Error::other("input frame release channel disconnected"))
            }
        }
    }

    /// Returns all dropped encoded output buffers to MediaCodec.
    pub fn release_pending_outputs(&mut self) -> Result<usize> {
        let mut released = 0;
        loop {
            match self.output_release_rx.try_recv() {
                Ok(buffer) => {
                    self.release_output_buffer(buffer)?;
                    self.pending_output_count = self.pending_output_count.saturating_sub(1);
                    released += 1;
                }
                Err(mpsc::TryRecvError::Empty | mpsc::TryRecvError::Disconnected) => {
                    return Ok(released);
                }
            }
        }
    }

    pub fn pending_output_count(&self) -> usize {
        self.pending_output_count
    }

    /// Waits for one encoded output lease to be dropped and releases its buffer.
    pub fn wait_release_output(&mut self, timeout: Duration) -> Result<bool> {
        if self.pending_output_count == 0 {
            return Ok(true);
        }
        match self.output_release_rx.recv_timeout(timeout) {
            Ok(buffer) => {
                self.release_output_buffer(buffer)?;
                self.pending_output_count = self.pending_output_count.saturating_sub(1);
                Ok(true)
            }
            Err(mpsc::RecvTimeoutError::Timeout) => Ok(false),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                Err(Error::other("encoded output release channel disconnected"))
            }
        }
    }

    /// Drains outstanding SDK buffer leases, stops the encoder, and releases it.
    ///
    /// `Drop` calls this as a best-effort fallback. Callers that own the camera
    /// lifecycle should call it explicitly so SDK cleanup failures are reported.
    pub fn shutdown(&mut self, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        let mut first_error = None;

        while self.pending_output_count > 0 && Instant::now() < deadline {
            if let Err(error) = self.wait_release_output(Duration::from_millis(100)) {
                first_error = Some(error);
                break;
            }
        }
        while self.pending_input_count > 0 && Instant::now() < deadline {
            if let Err(error) = self.wait_input_consumed(Duration::from_millis(100)) {
                if first_error.is_none() {
                    first_error = Some(error);
                }
                break;
            }
        }
        if self.pending_output_count > 0 && first_error.is_none() {
            first_error = Some(Error::other(format!(
                "timed out waiting for {} encoded output release(s)",
                self.pending_output_count
            )));
        }
        if self.pending_input_count > 0 && first_error.is_none() {
            first_error = Some(Error::other(format!(
                "timed out waiting for {} encoder input release(s)",
                self.pending_input_count
            )));
        }

        unsafe {
            if self.started {
                match call_media(Operation::MediaStop, raw::hb_mm_mc_stop(&mut *self.ctx)) {
                    Ok(()) => self.started = false,
                    Err(error) if first_error.is_none() => first_error = Some(error),
                    Err(_) => {}
                }
            }
            if self.initialized {
                match call_media(
                    Operation::MediaRelease,
                    raw::hb_mm_mc_release(&mut *self.ctx),
                ) {
                    Ok(()) => {
                        self.started = false;
                        self.initialized = false;
                    }
                    Err(error) if first_error.is_none() => first_error = Some(error),
                    Err(_) => {}
                }
            }
        }

        if let Some(error) = first_error {
            Err(error)
        } else {
            Ok(())
        }
    }

    fn release_output_buffer(&mut self, mut buffer: EncodedOutputBuffer) -> Result<()> {
        unsafe {
            call_media(
                Operation::MediaQueueOutputBuffer,
                raw::hb_mm_mc_queue_output_buffer(&mut *self.ctx, &mut buffer.0, 0),
            )
        }
    }

    unsafe fn configure_params(&mut self, config: EncoderConfig) -> Result<()> {
        self.ctx.codec_id = config.codec.raw();
        self.ctx.encoder = 1;

        let ctx = &mut *self.ctx as *mut raw::media_codec_context_t;
        {
            let params = unsafe { &mut (*ctx).__bindgen_anon_1.video_enc_params };
            params.width = config.size.width as i32;
            params.height = config.size.height as i32;
            params.pix_fmt = config.pixel_format.raw();
            params.frame_buf_count = config.frame_buffer_count;
            params.external_frame_buf = i32::from(config.external_frame_input);
            params.bitstream_buf_count = config.bitstream_buffer_count;
            params.bitstream_buf_size = config.bitstream_buffer_size;
            params.gop_params.decoding_refresh_type = 2;
            params.gop_params.gop_preset_idx = config.gop_preset.raw();
            params.rot_degree = config.rotation.raw();
            params.mir_direction = config.mirror.raw();
            params.frame_cropping_flag = 0;
            params.enable_user_pts = i32::from(config.enable_user_pts);
            params.rc_params.mode = RateControlMode::HevcCbr.raw();
        }

        unsafe {
            call_media(
                Operation::MediaGetRateControlConfig,
                raw::hb_mm_mc_get_rate_control_config(
                    ctx,
                    &mut (*ctx).__bindgen_anon_1.video_enc_params.rc_params,
                ),
            )?;
        }

        let params = unsafe { &mut (*ctx).__bindgen_anon_1.video_enc_params };
        params.rc_params.mode = RateControlMode::HevcCbr.raw();
        let rc = unsafe { &mut params.rc_params.__bindgen_anon_1.h265_cbr_params };
        let cbr = config.rate_control;
        let vbv_buffer_size = i32::try_from(cbr.vbv_buffer_size).map_err(|_| {
            Error::invalid(format!(
                "VBV buffer size too large: {}",
                cbr.vbv_buffer_size
            ))
        })?;
        let hvs_qp_scale = i32::try_from(cbr.hvs_qp_scale)
            .map_err(|_| Error::invalid(format!("HVS QP scale too large: {}", cbr.hvs_qp_scale)))?;
        rc.intra_period = cbr.intra_period;
        rc.intra_qp = cbr.intra_qp;
        rc.bit_rate = cbr.bit_rate_kbps;
        rc.frame_rate = cbr.frame_rate.as_u32();
        rc.initial_rc_qp = cbr.initial_rc_qp;
        rc.vbv_buffer_size = vbv_buffer_size;
        rc.ctu_level_rc_enalbe = u32::from(cbr.ctu_level_rc_enable);
        rc.min_qp_I = cbr.min_qp_i;
        rc.max_qp_I = cbr.max_qp_i;
        rc.min_qp_P = cbr.min_qp_p;
        rc.max_qp_P = cbr.max_qp_p;
        rc.min_qp_B = cbr.min_qp_b;
        rc.max_qp_B = cbr.max_qp_b;
        rc.hvs_qp_enable = u32::from(cbr.hvs_qp_enable);
        rc.hvs_qp_scale = hvs_qp_scale;
        rc.max_delta_qp = cbr.max_delta_qp;
        rc.qp_map_enable = i32::from(cbr.qp_map_enable);
        Ok(())
    }
}

#[cfg(x5cam_x5_target)]
impl Drop for MediaEncoder {
    fn drop(&mut self) {
        let _ = self.shutdown(Duration::from_secs(2));
    }
}

#[cfg(x5cam_x5_target)]
unsafe extern "C" fn on_input_buffer_consumed(
    _userdata: raw::hb_ptr,
    buffer: *mut raw::media_codec_buffer_t,
) {
    if buffer.is_null() {
        return;
    }
    let user_ptr = unsafe { (*buffer).user_ptr };
    if user_ptr.is_null() {
        return;
    }
    unsafe {
        (*buffer).user_ptr = std::ptr::null_mut();
        let mut queued = *Box::from_raw(user_ptr.cast::<QueuedInputFrame>());
        if let Some(release) = queued.release.take() {
            if let Err(err) = release.send(queued) {
                // The receiver only disconnects during teardown. Do not release
                // VIO frames on the MediaCodec callback thread in that race.
                std::mem::forget(err.0);
            }
        }
    }
}

#[cfg(x5cam_x5_target)]
fn call_hbn(operation: Operation, ret: i32) -> Result<()> {
    if ret == 0 {
        Ok(())
    } else {
        Err(sdk_error(operation, ret))
    }
}

#[cfg(x5cam_x5_target)]
fn call_media(operation: Operation, ret: i32) -> Result<()> {
    if ret == 0 {
        Ok(())
    } else {
        Err(Error::Media { operation, ret })
    }
}

#[cfg(x5cam_x5_target)]
fn sdk_error(operation: Operation, ret: i32) -> Error {
    Error::Sdk {
        operation,
        ret,
        info: hbn_error_suffix(ret),
    }
}

#[cfg(x5cam_x5_target)]
fn hbn_error_suffix(ret: i32) -> String {
    unsafe {
        let ptr = raw::hbn_err_info(ret);
        if ptr.is_null() {
            return String::new();
        }
        match CStr::from_ptr(ptr).to_str() {
            Ok(info) if !info.is_empty() => format!(" ({info})"),
            _ => String::new(),
        }
    }
}

#[cfg(x5cam_x5_target)]
fn as_mut_void<T>(value: &mut T) -> *mut std::ffi::c_void {
    value as *mut T as *mut std::ffi::c_void
}

#[cfg(x5cam_x5_target)]
fn u16_from_u32(value: u32, name: &str) -> Result<u16> {
    u16::try_from(value).map_err(|_| Error::invalid(format!("{name} too large: {value}")))
}

#[cfg(x5cam_x5_target)]
fn set_c_string<const N: usize>(dst: &mut [c_char; N], value: &str) -> Result<()> {
    let bytes = value.as_bytes();
    if bytes.len() >= N {
        return Err(Error::invalid(format!(
            "C string does not fit in {N} bytes"
        )));
    }
    dst.fill(0);
    for (out, byte) in dst.iter_mut().zip(bytes.iter().copied()) {
        *out = byte as c_char;
    }
    Ok(())
}

#[cfg(x5cam_x5_target)]
fn timeval_to_nanos(value: raw::timeval) -> Option<u64> {
    if value.tv_sec == 0 && value.tv_usec == 0 {
        return None;
    }
    let seconds = u64::try_from(value.tv_sec).ok()?;
    let micros = u64::try_from(value.tv_usec).ok()?;
    seconds
        .checked_mul(1_000_000_000)?
        .checked_add(micros.checked_mul(1_000)?)
}

#[cfg(x5cam_x5_target)]
trait BoolExt {
    fn raw_bool(self) -> raw::enum_cam_bool_e;
}

#[cfg(x5cam_x5_target)]
impl BoolExt for bool {
    fn raw_bool(self) -> raw::enum_cam_bool_e {
        if self {
            raw::enum_cam_bool_e_CAM_TRUE
        } else {
            raw::enum_cam_bool_e_CAM_FALSE
        }
    }
}

#[cfg(x5cam_x5_target)]
impl SensorModule {
    fn sdk_name(self) -> &'static str {
        match self {
            Self::Sc132gs => "sc132gs",
        }
    }
}

#[cfg(x5cam_x5_target)]
impl SensorCalibration {
    fn sdk_name(self) -> &'static str {
        match self {
            Self::Disabled => "disable",
        }
    }
}

#[cfg(x5cam_x5_target)]
impl SensorMode {
    fn raw(self) -> u32 {
        match self {
            Self::Normal => raw::sensor_mode_e_NORMAL_M,
            Self::Dol2 => raw::sensor_mode_e_DOL2_M,
            Self::Dol3 => raw::sensor_mode_e_DOL3_M,
            Self::Dol4 => raw::sensor_mode_e_DOL4_M,
            Self::Pwl => raw::sensor_mode_e_PWL_M,
            Self::Slave => raw::sensor_mode_e_SLAVE_M,
            Self::Mono => raw::sensor_mode_e_MONO_M,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl MipiDataType {
    fn raw(self) -> u32 {
        match self {
            Self::Raw10 => raw::sensor_format_e_DATA_TYPE_RAW10,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl FrameFormat {
    fn raw(self) -> raw::enum_frame_format_e {
        match self {
            Self::Raw => raw::enum_frame_format_e_FRM_FMT_RAW,
            Self::Nv12 => raw::enum_frame_format_e_FRM_FMT_NV12,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl PixelFormat {
    fn raw(self) -> raw::_mc_pixel_format {
        match self {
            Self::Nv12 => raw::_mc_pixel_format_MC_PIXEL_FORMAT_NV12,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl InputMode {
    fn raw(self) -> raw::enum_input_mode_e {
        match self {
            Self::Ddr => raw::enum_input_mode_e_DDR_MODE,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl IspSensorMode {
    fn raw(self) -> raw::isp_sensor_mode_e {
        match self {
            Self::Normal => raw::isp_sensor_mode_e_ISP_NORMAL_M,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl VnodeType {
    fn raw(self) -> raw::hb_vnode_type_e {
        match self {
            Self::Vin => raw::hb_vnode_type_e_HB_VIN,
            Self::Isp => raw::hb_vnode_type_e_HB_ISP,
            Self::Gdc => raw::hb_vnode_type_e_HB_GDC,
            Self::Vse => raw::hb_vnode_type_e_HB_VSE,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl AllocationId {
    fn raw(self) -> i32 {
        match self {
            Self::Auto => raw::AUTO_ALLOC_ID,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl VinOutputKind {
    fn raw(self) -> raw::vin_ochn_attr_type_s {
        match self {
            Self::Basic => raw::vin_ochn_attr_type_s_VIN_BASIC_ATTR,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl MclkAttrKind {
    fn raw(self) -> raw::vin_attr_ex_type_s {
        match self {
            Self::Static => raw::vin_attr_ex_type_s_VIN_STATIC_MCLK_ATTR,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl HdrMode {
    fn raw(self) -> raw::hdr_mode {
        match self {
            Self::None => raw::hdr_mode_NOT_HDR,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl TimestampMode {
    fn raw_bits(self) -> u32 {
        match self {
            Self::Vsync => raw::time_stamp_mode_TS_IPI_VSYNC,
            Self::Trigger => raw::time_stamp_mode_TS_IPI_TRIGGER,
        }
    }
}

#[cfg(not(x5cam_x5_target))]
impl TimestampMode {
    fn raw_bits(self) -> u32 {
        match self {
            Self::Vsync => 0x1,
            Self::Trigger => 0x2,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl TimestampModeSet {
    fn raw(self) -> raw::time_stamp_mode {
        self.bits
    }
}

#[cfg(x5cam_x5_target)]
impl LpwmTriggerSource {
    fn raw(self) -> u32 {
        match self {
            Self::Internal => 0,
            Self::Sif => 10,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl LpwmTriggerMode {
    fn raw(self) -> u32 {
        match self {
            Self::Internal => 0,
            Self::External => 1,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl BufferUsage {
    fn raw_bits(self) -> i64 {
        match self {
            Self::CpuReadOften => raw::mem_usage_t_HB_MEM_USAGE_CPU_READ_OFTEN as i64,
            Self::CpuWriteOften => raw::mem_usage_t_HB_MEM_USAGE_CPU_WRITE_OFTEN as i64,
            Self::Cached => raw::mem_usage_t_HB_MEM_USAGE_CACHED as i64,
            Self::GraphicContiguous => raw::mem_usage_t_HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF as i64,
            Self::HwCim => raw::mem_usage_t_HB_MEM_USAGE_HW_CIM as i64,
            Self::HwIsp => raw::mem_usage_t_HB_MEM_USAGE_HW_ISP as i64,
            Self::HwGdcOutput => raw::mem_usage_t_HB_MEM_USAGE_HW_GDC_OUT as i64,
            Self::HwVideoCodec => raw::mem_usage_t_HB_MEM_USAGE_HW_VIDEO_CODEC as i64,
            Self::MapInitialized => raw::mem_usage_t_HB_MEM_USAGE_MAP_INITIALIZED as i64,
            Self::PrivateHeap2Reserved => raw::mem_usage_t_HB_MEM_USAGE_PRIV_HEAP_2_RESERVED as i64,
        }
    }
}

#[cfg(not(x5cam_x5_target))]
impl BufferUsage {
    fn raw_bits(self) -> i64 {
        0
    }
}

#[cfg(x5cam_x5_target)]
impl GdcFrameFormat {
    fn raw(self) -> raw::frame_format {
        match self {
            Self::Semiplanar420 => raw::frame_format_FMT_SEMIPLANAR_420,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl GdcTransformation {
    fn raw(self) -> raw::gdc_transformation {
        match self {
            Self::Custom => raw::gdc_transformation_CUSTOM,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl CodecId {
    fn raw(self) -> raw::_media_codec_id {
        match self {
            Self::Hevc => raw::_media_codec_id_MEDIA_CODEC_ID_H265,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl GopPreset {
    fn raw(self) -> u32 {
        match self {
            Self::SingleReferenceIppp => 9,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl Rotation {
    fn raw(self) -> raw::_mc_rotate_degree {
        match self {
            Self::None => raw::_mc_rotate_degree_MC_CCW_0,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl Mirror {
    fn raw(self) -> raw::_mc_mirror_direction {
        match self {
            Self::None => raw::_mc_mirror_direction_MC_DIRECTION_NONE,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl RateControlMode {
    fn raw(self) -> raw::_mc_video_rate_control_mode {
        match self {
            Self::HevcCbr => raw::_mc_video_rate_control_mode_MC_AV_RC_MODE_H265CBR,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl Rect {
    fn raw(self) -> raw::common_rect_t {
        raw::common_rect_t {
            x: self.x,
            y: self.y,
            w: self.width,
            h: self.height,
        }
    }
}

#[cfg(x5cam_x5_target)]
impl FrameRateControl {
    fn raw(self) -> raw::frame_fps_ctrl_t {
        raw::frame_fps_ctrl_t {
            src: self.source.as_u16(),
            dst: self.target.as_u16(),
        }
    }
}
