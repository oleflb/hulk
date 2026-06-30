use camera_driver_sys as sys;
use camera_driver_sys::{
    AllocationId, BufferAllocation, BufferUsage, BufferUsageSet, CameraConfig, ChannelId,
    FrameFormat, FrameIdConfig, FrameLength, FrameRate, FrameRateControl, GdcAttr,
    GdcInputChannelAttr, GdcOutputChannelAttr, HardwareId, HdrMode, I2cAddress, ImageSize,
    InputMode, IspAttr, IspInputChannelAttr, IspOutputChannelAttr, IspSensorMode, LineLength,
    LpwmChannelConfig, LpwmConfig, LpwmTriggerMode, LpwmTriggerSource, MclkAttrKind,
    MclkFrequencyHz, MipiClockIndex, MipiClockMHz, MipiConfig, MipiDataType, MipiLaneCount,
    MipiRxExtensionConfig, Rect, SensorCalibration, SensorGpioConfig, SensorMode, SensorModule,
    SettleTime, TimestampConfig, TimestampMode, TimestampModeSet, VinAttr, VinInputChannelAttr,
    VinMclkAttr, VinOutputChannelAttr, VinOutputKind, VseAttr, VseInputChannelAttr,
    VseOutputChannelAttr,
};
use color_eyre::eyre::{Result, WrapErr, eyre};
use tracing::{info, trace};

use crate::config::Config;
use crate::gdc::GdcBin;
use crate::sensor::SensorHost;

/// Primary channel id used by the offline VIN -> ISP path.
const CHN_ID: ChannelId = ChannelId::PRIMARY;
/// VIN IPI channel used by the X5 sample pipeline setup.
const VIN_IPI_CHANNEL: ChannelId = ChannelId(1);

/// RAII wrapper for one X5 camera-to-VSE hardware pipeline.
pub struct VioPipeline {
    /// Vflow handle binding all vnodes.
    vflow: sys::Vflow,
    /// VSE vnode handle used for frame capture.
    vse: sys::VseNode,
    /// GDC vnode handle kept alive while the vflow runs.
    _gdc: sys::GdcNode,
    /// ISP vnode handle kept alive while the vflow runs.
    _isp: sys::IspNode,
    /// VIN vnode handle kept alive while the camera is attached.
    _vin: sys::VinNode,
    /// Camera handle attached to VIN.
    camera: sys::Camera,
    /// GDC config buffer installed into the GDC vnode.
    _gdc_bin: GdcBin,
}

unsafe impl Send for VioPipeline {}

impl VioPipeline {
    /// Creates and attaches the camera, VIN, ISP, GDC, VSE, and vflow nodes.
    pub fn create(config: &Config, sensor: &SensorHost, gdc_bin: GdcBin) -> Result<Self> {
        let camera_config = camera_config(config, sensor)?;
        info!(
            host = sensor.host,
            i2c = %format_args!("0x{:02x}", sensor.i2c_addr),
            sensor_mode = ?camera_config.sensor_mode,
            sensor_mode_sdk = camera_config.sensor_mode.sdk_value(),
            calibration = "disable",
            "camera config",
        );
        let mut camera = trace_sdk_call("hbn_camera_create", || sys::Camera::create(camera_config))
            .wrap_err("create camera")?;
        let vin = create_vin(config, sensor)?;
        let isp = create_isp(config)?;
        let gdc = create_gdc(config, &gdc_bin)?;
        let vse = create_vse(config)?;
        let vflow = create_vflow(&vin, &isp, &gdc, &vse)?;
        trace_sdk_call("hbn_camera_attach_to_vin", || camera.attach_to_vin(&vin))
            .wrap_err("attach camera to VIN")?;

        Ok(Self {
            vflow,
            vse,
            _gdc: gdc,
            _isp: isp,
            _vin: vin,
            camera,
            _gdc_bin: gdc_bin,
        })
    }

    /// Starts the hardware vflow.
    pub fn start(&mut self) -> Result<()> {
        Ok(trace_sdk_call("hbn_vflow_start", || self.vflow.start())?)
    }

    /// Gets one VSE output frame and returns a release-on-drop lease.
    pub fn get_frame(&mut self, timeout_ms: u32) -> Result<sys::FrameLease> {
        Ok(trace_sdk_call("hbn_vnode_getframe(VSE)", || {
            self.vse.get_frame(CHN_ID, timeout_ms)
        })?)
    }

    /// Stops the vflow and detaches the camera from VIN.
    pub fn shutdown(&mut self) -> Result<()> {
        let mut first_error = None;
        if let Err(error) =
            trace_sdk_call("hbn_vflow_stop", || self.vflow.stop()).wrap_err("stop VIO vflow")
        {
            first_error = Some(error);
        }
        if let Err(error) = trace_sdk_call("hbn_camera_detach_from_vin", || {
            self.camera.detach_from_vin()
        })
        .wrap_err("detach camera from VIN")
        {
            if first_error.is_none() {
                first_error = Some(error);
            }
        }

        if let Some(error) = first_error {
            Err(error)
        } else {
            Ok(())
        }
    }
}

impl Drop for VioPipeline {
    /// Stops and detaches dependency edges before the owned sys resources drop.
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}

/// Builds SDK sensor and camera config structs for one SC132GS host.
fn camera_config(config: &Config, sensor: &SensorHost) -> Result<CameraConfig> {
    Ok(CameraConfig {
        module: SensorModule::Sc132gs,
        i2c_address: I2cAddress(sensor.i2c_addr),
        sensor_mode: SensorMode::Normal,
        frame_rate: FrameRate::new(config.fps)?,
        format: MipiDataType::Raw10,
        size: ImageSize::new(config.raw_width, config.raw_height),
        gpio: SensorGpioConfig {
            enable_bit: 0x01,
            level_bit: 0x00,
        },
        calibration: SensorCalibration::Disabled,
        mipi: MipiConfig {
            rx_enabled: true,
            rx_extension: MipiRxExtensionConfig { nocheck: true },
            lane_count: MipiLaneCount(1),
            data_type: MipiDataType::Raw10,
            frame_rate: FrameRate::new(config.fps)?,
            mclk: MipiClockIndex(1),
            mipi_clock: MipiClockMHz(1200),
            size: ImageSize::new(config.raw_width, config.raw_height),
            line_length: LineLength(1400),
            frame_length: FrameLength(1500),
            settle: SettleTime(20),
            channel: CHN_ID,
        },
    })
}

/// Opens and configures the VIN node for RAW10 sensor input.
fn create_vin(config: &Config, sensor: &SensorHost) -> Result<sys::VinNode> {
    let mipi_rx = u32::try_from(sensor.mipi_rx_phy)
        .map_err(|_| eyre!("invalid negative MIPI RX PHY {}", sensor.mipi_rx_phy))?;
    let mut vin = trace_sdk_call("hbn_vnode_open(VIN)", || {
        sys::VinNode::open_vin(HardwareId(mipi_rx), AllocationId::Auto)
    })
    .wrap_err("open VIN vnode")?;
    let attr = vin_attr(config, HardwareId(mipi_rx));
    let vin_ddr_enabled = true;
    info!(
        mipi_rx,
        isp_flyby = attr.isp_flyby,
        vin_ddr = vin_ddr_enabled,
        vin_out = CHN_ID.0,
        isp_in = CHN_ID.0,
        "vin attr",
    );
    trace_sdk_call("hbn_vnode_set_attr(VIN)", || vin.set_vin_attr(attr))
        .wrap_err("set VIN attr")?;
    trace_sdk_call("hbn_vnode_set_ichn_attr(VIN)", || {
        vin.set_vin_input_channel_attr(
            CHN_ID,
            VinInputChannelAttr {
                format: MipiDataType::Raw10,
                size: ImageSize::new(config.raw_width, config.raw_height),
            },
        )
    })
    .wrap_err("set VIN input channel attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_attr(VIN)", || {
        vin.set_vin_output_channel_attr(
            CHN_ID,
            VinOutputChannelAttr {
                ddr_enabled: vin_ddr_enabled,
                kind: VinOutputKind::Basic,
                format: MipiDataType::Raw10,
                stride: config.raw_width * 2,
            },
        )
    })
    .wrap_err("set VIN output channel attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_buf_attr(VIN)", || {
        vin.set_output_channel_buffer_attr(
            CHN_ID,
            buffer_allocation(
                BufferUsageSet::empty()
                    .with(BufferUsage::HwCim)
                    .with(BufferUsage::GraphicContiguous),
            ),
        )
    })
    .wrap_err("set VIN output buffer attr")?;

    if sensor.mclk_configured {
        trace_sdk_call("hbn_vnode_set_attr_ex(VIN mclk)", || {
            vin.set_vin_mclk_attr(VinMclkAttr {
                kind: MclkAttrKind::Static,
                frequency: MclkFrequencyHz(24_000_000),
            })
        })
        .wrap_err("set VIN MCLK attr")?;
    } else {
        trace!(mipi_rx, "skip VIN MCLK attr because MCLK is not configured");
    }

    Ok(vin)
}

/// Opens and configures the ISP node to produce NV12 frames.
fn create_isp(config: &Config) -> Result<sys::IspNode> {
    let mut isp = trace_sdk_call("hbn_vnode_open(ISP)", || {
        sys::IspNode::open_isp(HardwareId(0), AllocationId::Auto)
    })
    .wrap_err("open ISP vnode")?;
    let input_mode = InputMode::Ddr;
    let sensor_mode = IspSensorMode::Normal;
    info!(
        input_mode = ?input_mode,
        input_mode_sdk = input_mode.sdk_value(),
        sensor_mode = ?sensor_mode,
        sensor_mode_sdk = sensor_mode.sdk_value(),
        crop_x = 0,
        crop_y = 0,
        crop_w = config.raw_width,
        crop_h = config.raw_height,
        "isp attr",
    );
    trace_sdk_call("hbn_vnode_set_attr(ISP)", || {
        isp.set_isp_attr(IspAttr {
            input_mode,
            sensor_mode,
            crop: Rect {
                x: 0,
                y: 0,
                width: config.raw_width,
                height: config.raw_height,
            },
        })
    })
    .wrap_err("set ISP attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_attr(ISP)", || {
        isp.set_isp_output_channel_attr(
            CHN_ID,
            IspOutputChannelAttr {
                ddr_enabled: true,
                format: FrameFormat::Nv12,
                bit_width: 8,
            },
        )
    })
    .wrap_err("set ISP output channel attr")?;
    trace_sdk_call("hbn_vnode_set_ichn_attr(ISP)", || {
        isp.set_isp_input_channel_attr(
            CHN_ID,
            IspInputChannelAttr {
                size: ImageSize::new(config.raw_width, config.raw_height),
                format: FrameFormat::Raw,
                bit_width: 10,
            },
        )
    })
    .wrap_err("set ISP input channel attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_buf_attr(ISP)", || {
        isp.set_output_channel_buffer_attr(
            CHN_ID,
            buffer_allocation(
                BufferUsageSet::empty()
                    .with(BufferUsage::HwIsp)
                    .with(BufferUsage::GraphicContiguous),
            ),
        )
    })
    .wrap_err("set ISP output buffer attr")?;
    Ok(isp)
}

/// Opens and configures the GDC node with the generated calibration bin.
fn create_gdc(config: &Config, gdc_bin: &GdcBin) -> Result<sys::GdcNode> {
    let mut gdc = trace_sdk_call("hbn_vnode_open(GDC)", || {
        sys::GdcNode::open_gdc(HardwareId(0), AllocationId::Auto)
    })
    .wrap_err("open GDC vnode")?;
    trace_sdk_call("hbn_vnode_set_attr(GDC)", || {
        gdc.set_gdc_attr(GdcAttr {
            bin: gdc_bin,
            total_planes: 2,
        })
    })
    .wrap_err("set GDC attr")?;
    trace_sdk_call("hbn_vnode_set_ichn_attr(GDC)", || {
        gdc.set_gdc_input_channel_attr(
            CHN_ID,
            GdcInputChannelAttr {
                size: ImageSize::new(config.raw_width, config.raw_height),
                stride: config.raw_width,
            },
        )
    })
    .wrap_err("set GDC input channel attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_attr(GDC)", || {
        gdc.set_gdc_output_channel_attr(
            CHN_ID,
            GdcOutputChannelAttr {
                size: ImageSize::new(config.out_width, config.out_height),
                stride: config.out_width,
            },
        )
    })
    .wrap_err("set GDC output channel attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_buf_attr(GDC)", || {
        gdc.set_output_channel_buffer_attr(
            CHN_ID,
            buffer_allocation(
                BufferUsageSet::empty()
                    .with(BufferUsage::HwGdcOutput)
                    .with(BufferUsage::GraphicContiguous),
            ),
        )
    })
    .wrap_err("set GDC output buffer attr")?;
    Ok(gdc)
}

/// Opens and configures the VSE node for full-size NV12 output.
fn create_vse(config: &Config) -> Result<sys::VseNode> {
    let mut vse = trace_sdk_call("hbn_vnode_open(VSE)", || {
        sys::VseNode::open_vse(HardwareId(0), AllocationId::Auto)
    })
    .wrap_err("open VSE vnode")?;
    let frame_rate = FrameRateControl::Disabled;
    trace_sdk_call("hbn_vnode_set_attr(VSE)", || {
        vse.set_vse_attr(VseAttr { frame_rate })
    })
    .wrap_err("set VSE attr")?;
    trace_sdk_call("hbn_vnode_set_ichn_attr(VSE)", || {
        vse.set_vse_input_channel_attr(
            CHN_ID,
            VseInputChannelAttr {
                size: ImageSize::new(config.out_width, config.out_height),
                format: FrameFormat::Nv12,
                bit_width: 8,
            },
        )
    })
    .wrap_err("set VSE input channel attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_attr(VSE)", || {
        vse.set_vse_output_channel_attr(
            CHN_ID,
            VseOutputChannelAttr {
                enabled: true,
                roi: Rect {
                    x: 0,
                    y: 0,
                    width: config.out_width,
                    height: config.out_height,
                },
                target_size: ImageSize::new(config.out_width, config.out_height),
                format: FrameFormat::Nv12,
                bit_width: 8,
                frame_rate,
            },
        )
    })
    .wrap_err("set VSE output channel attr")?;
    trace_sdk_call("hbn_vnode_set_ochn_buf_attr(VSE)", || {
        vse.set_output_channel_buffer_attr(
            CHN_ID,
            buffer_allocation(BufferUsageSet::empty().with(BufferUsage::GraphicContiguous)),
        )
    })
    .wrap_err("set VSE output buffer attr")?;
    Ok(vse)
}

/// Creates the vflow and binds VIN -> ISP -> GDC -> VSE.
fn create_vflow(
    vin: &sys::VinNode,
    isp: &sys::IspNode,
    gdc: &sys::GdcNode,
    vse: &sys::VseNode,
) -> Result<sys::Vflow> {
    let mut vflow = trace_sdk_call("hbn_vflow_create", sys::Vflow::create)?;
    trace_sdk_call("hbn_vflow_add_vnode(VIN)", || vflow.add_vnode(vin))
        .wrap_err("add VIN to vflow")?;
    trace_sdk_call("hbn_vflow_add_vnode(ISP)", || vflow.add_vnode(isp))
        .wrap_err("add ISP to vflow")?;
    trace_sdk_call("hbn_vflow_add_vnode(GDC)", || vflow.add_vnode(gdc))
        .wrap_err("add GDC to vflow")?;
    trace_sdk_call("hbn_vflow_add_vnode(VSE)", || vflow.add_vnode(vse))
        .wrap_err("add VSE to vflow")?;
    trace_sdk_call("hbn_vflow_bind_vnode(VIN->ISP)", || {
        vflow.bind(vin, CHN_ID, isp, CHN_ID)
    })
    .wrap_err("bind VIN to ISP")?;
    trace_sdk_call("hbn_vflow_bind_vnode(ISP->GDC)", || {
        vflow.bind(isp, CHN_ID, gdc, CHN_ID)
    })
    .wrap_err("bind ISP to GDC")?;
    trace_sdk_call("hbn_vflow_bind_vnode(GDC->VSE)", || {
        vflow.bind(gdc, CHN_ID, vse, CHN_ID)
    })
    .wrap_err("bind GDC to VSE")?;
    Ok(vflow)
}

/// Builds VIN node attributes from the selected sensor host.
fn vin_attr(config: &Config, mipi_rx: HardwareId) -> VinAttr {
    let timestamp_modes = TimestampModeSet::new(TimestampMode::Vsync);
    VinAttr {
        mipi_rx,
        vc_index: CHN_ID,
        ipi_channel: VIN_IPI_CHANNEL,
        isp_flyby: true,
        frame_id: FrameIdConfig {
            enable: true,
            set_initial: false,
        },
        hdr_mode: HdrMode::None,
        timestamp: TimestampConfig::Enabled {
            modes: timestamp_modes,
            source: 1,
            pps_source: 6,
        },
        lpwm: LpwmConfig {
            enable: false,
            channels: [lpwm_channel(config); 4],
        },
    }
}

/// Builds one LPWM channel config for sensor trigger output.
fn lpwm_channel(config: &Config) -> LpwmChannelConfig {
    let period_us = ((1_000_000 + config.fps / 2) / config.fps).saturating_sub(1);
    LpwmChannelConfig {
        trigger_source: LpwmTriggerSource::Internal,
        trigger_mode: LpwmTriggerMode::Internal,
        period_us,
        offset_us: 10,
        duty_time_us: 100,
        threshold: 0,
        adjust_step: 0,
    }
}

/// Builds common SDK buffer allocation attributes.
fn buffer_allocation(usages: BufferUsageSet) -> BufferAllocation {
    BufferAllocation {
        buffer_count: 3,
        usages,
        contiguous: true,
    }
}

/// Emits trace logs around a synchronous SDK call, including calls that hang.
fn trace_sdk_call<T, E>(
    name: &'static str,
    call: impl FnOnce() -> std::result::Result<T, E>,
) -> std::result::Result<T, E>
where
    E: std::fmt::Display,
{
    trace!(sdk_call = name, "sdk call start");
    let result = call();
    match &result {
        Ok(_) => trace!(sdk_call = name, "sdk call ok"),
        Err(error) => trace!(sdk_call = name, error = %error, "sdk call error"),
    }
    result
}
