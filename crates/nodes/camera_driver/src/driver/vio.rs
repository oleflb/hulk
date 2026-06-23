use std::ffi::CStr;
use std::os::raw::{c_char, c_void};

use crate::config::Config;
use crate::gdc::GdcBin;
use crate::sensor::SensorHost;

/// MIPI RAW10 data type value used by SC132GS.
const RAW10: u32 = 0x2b;
/// Single-channel VIO pipelines use channel id 0 throughout.
const CHN_ID: u32 = 0;

/// RAII wrapper for one X5 camera-to-VSE hardware pipeline.
pub struct VioPipeline {
    /// Camera handle created by `hbn_camera_create`.
    camera: Option<crate::ffi::camera_handle_t>,
    /// VIN vnode handle.
    vin: Option<crate::ffi::hbn_vnode_handle_t>,
    /// ISP vnode handle.
    isp: Option<crate::ffi::hbn_vnode_handle_t>,
    /// GDC vnode handle.
    gdc: Option<crate::ffi::hbn_vnode_handle_t>,
    /// VSE vnode handle.
    vse: Option<crate::ffi::hbn_vnode_handle_t>,
    /// Vflow handle binding all vnodes.
    vflow: Option<crate::ffi::hbn_vflow_handle_t>,
    /// Whether the camera is attached to VIN.
    attached: bool,
    /// Whether the vflow has been started.
    started: bool,
    /// Sensor and camera SDK structs kept alive for the pipeline lifetime.
    config: Sc132gsConfig,
}

unsafe impl Send for VioPipeline {}

impl VioPipeline {
    /// Creates and attaches the camera, VIN, ISP, GDC, VSE, and vflow nodes.
    pub fn create(config: &Config, sensor: &SensorHost, gdc_bin: &GdcBin) -> Result<Self, String> {
        let sensor_cfg = Sc132gsConfig::new(config, sensor)?;
        let mut pipeline = Self {
            camera: None,
            vin: None,
            isp: None,
            gdc: None,
            vse: None,
            vflow: None,
            attached: false,
            started: false,
            config: sensor_cfg,
        };

        unsafe {
            pipeline.create_camera()?;
            pipeline.create_vin(config, sensor)?;
            pipeline.create_isp(config)?;
            pipeline.create_gdc(config, gdc_bin)?;
            pipeline.create_vse(config)?;
            pipeline.create_vflow()?;
            pipeline.attach_camera()?;
        }

        Ok(pipeline)
    }

    /// Starts the hardware vflow.
    pub fn start(&mut self) -> Result<(), String> {
        let vflow = self.vflow.ok_or("vflow handle missing")?;
        unsafe {
            call("hbn_vflow_start", crate::ffi::hbn_vflow_start(vflow))?;
        }
        self.started = true;
        Ok(())
    }

    /// Gets one VSE output frame and returns a release-on-drop lease.
    pub fn get_frame(&mut self, timeout_ms: u32) -> Result<FrameLease, String> {
        let vnode = self.vse.ok_or("VSE node missing")?;
        let mut image = crate::ffi::hbn_vnode_image_t::default();
        unsafe {
            call(
                "hbn_vnode_getframe(VSE)",
                crate::ffi::hbn_vnode_getframe(vnode, CHN_ID, timeout_ms, &mut image),
            )?;
        }
        Ok(FrameLease {
            vnode,
            channel: CHN_ID,
            image,
            released: false,
        })
    }

    /// Creates the camera handle from the prepared SC132GS config.
    unsafe fn create_camera(&mut self) -> Result<(), String> {
        let mut camera = 0;
        call(
            "hbn_camera_create",
            crate::ffi::hbn_camera_create(&mut *self.config.camera, &mut camera),
        )?;
        self.camera = Some(camera);
        Ok(())
    }

    /// Opens and configures the VIN node for RAW10 sensor input.
    unsafe fn create_vin(&mut self, config: &Config, sensor: &SensorHost) -> Result<(), String> {
        let mut vin = 0;
        call(
            "hbn_vnode_open(HB_VIN)",
            crate::ffi::hbn_vnode_open(
                crate::ffi::hb_vnode_type_e_HB_VIN,
                sensor.mipi_rx_phy as u32,
                crate::ffi::AUTO_ALLOC_ID,
                &mut vin,
            ),
        )?;
        self.vin = Some(vin);

        let mut vin_attr = vin_node_attr(config, sensor);
        call(
            "hbn_vnode_set_attr(VIN)",
            crate::ffi::hbn_vnode_set_attr(vin, as_mut_void(&mut vin_attr)),
        )?;

        let mut ichn = crate::ffi::vin_ichn_attr_t {
            format: RAW10,
            width: config.raw_width,
            height: config.raw_height,
        };
        call(
            "hbn_vnode_set_ichn_attr(VIN)",
            crate::ffi::hbn_vnode_set_ichn_attr(vin, CHN_ID, as_mut_void(&mut ichn)),
        )?;

        let mut ochn = crate::ffi::vin_ochn_attr_t {
            ddr_en: 1,
            ochn_attr_type: crate::ffi::vin_ochn_attr_type_s_VIN_BASIC_ATTR,
            vin_basic_attr: crate::ffi::vin_basic_attr_t {
                format: RAW10,
                wstride: config.raw_width * 2,
                ..Default::default()
            },
            ..Default::default()
        };
        call(
            "hbn_vnode_set_ochn_attr(VIN)",
            crate::ffi::hbn_vnode_set_ochn_attr(vin, CHN_ID, as_mut_void(&mut ochn)),
        )?;

        let mut alloc = alloc_attr(
            3,
            crate::ffi::mem_usage_t_HB_MEM_USAGE_HW_CIM
                | crate::ffi::mem_usage_t_HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
        );
        call(
            "hbn_vnode_set_ochn_buf_attr(VIN)",
            crate::ffi::hbn_vnode_set_ochn_buf_attr(vin, CHN_ID, &mut alloc),
        )?;

        if sensor.mclk_configured {
            let mut attr_ex = crate::ffi::vin_attr_ex_t {
                ex_attr_type: crate::ffi::vin_attr_ex_type_s_VIN_STATIC_MCLK_ATTR,
                mclk_ex_attr: crate::ffi::mclk_attr_ex_t {
                    mclk_freq: 24_000_000,
                },
                vin_attr_ex_mask: 0x80,
                ..Default::default()
            };
            call(
                "hbn_vnode_set_attr_ex(VIN:MCLK)",
                crate::ffi::hbn_vnode_set_attr_ex(vin, as_mut_void(&mut attr_ex)),
            )?;
        }

        Ok(())
    }

    /// Opens and configures the ISP node to produce NV12 frames.
    unsafe fn create_isp(&mut self, config: &Config) -> Result<(), String> {
        let mut isp = 0;
        call(
            "hbn_vnode_open(HB_ISP)",
            crate::ffi::hbn_vnode_open(
                crate::ffi::hb_vnode_type_e_HB_ISP,
                0,
                crate::ffi::AUTO_ALLOC_ID,
                &mut isp,
            ),
        )?;
        self.isp = Some(isp);

        let mut attr = crate::ffi::isp_attr_t {
            input_mode: crate::ffi::enum_input_mode_e_DDR_MODE,
            sensor_mode: crate::ffi::isp_sensor_mode_e_ISP_NORMAL_M,
            crop: crate::ffi::common_rect_t {
                x: 0,
                y: 0,
                w: config.raw_width,
                h: config.raw_height,
            },
            ..Default::default()
        };
        call(
            "hbn_vnode_set_attr(ISP)",
            crate::ffi::hbn_vnode_set_attr(isp, as_mut_void(&mut attr)),
        )?;

        let mut ochn = crate::ffi::isp_ochn_attr_t {
            ddr_en: crate::ffi::enum_cam_bool_e_CAM_TRUE,
            fmt: crate::ffi::enum_frame_format_e_FRM_FMT_NV12,
            bit_width: 8,
            ..Default::default()
        };
        call(
            "hbn_vnode_set_ochn_attr(ISP)",
            crate::ffi::hbn_vnode_set_ochn_attr(isp, CHN_ID, as_mut_void(&mut ochn)),
        )?;

        let mut ichn = crate::ffi::isp_ichn_attr_t {
            width: config.raw_width,
            height: config.raw_height,
            fmt: crate::ffi::enum_frame_format_e_FRM_FMT_RAW,
            bit_width: 10,
            ..Default::default()
        };
        call(
            "hbn_vnode_set_ichn_attr(ISP)",
            crate::ffi::hbn_vnode_set_ichn_attr(isp, CHN_ID, as_mut_void(&mut ichn)),
        )?;

        let mut alloc = alloc_attr(
            3,
            crate::ffi::mem_usage_t_HB_MEM_USAGE_HW_ISP
                | crate::ffi::mem_usage_t_HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
        );
        call(
            "hbn_vnode_set_ochn_buf_attr(ISP)",
            crate::ffi::hbn_vnode_set_ochn_buf_attr(isp, CHN_ID, &mut alloc),
        )?;

        Ok(())
    }

    /// Opens and configures the GDC node with the generated calibration bin.
    unsafe fn create_gdc(&mut self, config: &Config, gdc_bin: &GdcBin) -> Result<(), String> {
        let mut gdc = 0;
        call(
            "hbn_vnode_open(HB_GDC)",
            crate::ffi::hbn_vnode_open(
                crate::ffi::hb_vnode_type_e_HB_GDC,
                0,
                crate::ffi::AUTO_ALLOC_ID,
                &mut gdc,
            ),
        )?;
        self.gdc = Some(gdc);

        let config_size = u32::try_from(gdc_bin.buf.size)
            .map_err(|_| format!("GDC bin too large: {} bytes", gdc_bin.buf.size))?;
        let mut attr = crate::ffi::gdc_attr_t {
            config_addr: gdc_bin.buf.phys_addr,
            config_size,
            total_planes: 2,
            binary_ion_id: gdc_bin.buf.share_id,
            binary_offset: gdc_bin.buf.offset,
            ..Default::default()
        };
        call(
            "hbn_vnode_set_attr(GDC)",
            crate::ffi::hbn_vnode_set_attr(gdc, as_mut_void(&mut attr)),
        )?;

        let mut ichn = crate::ffi::gdc_ichn_attr_t {
            input_width: config.raw_width,
            input_height: config.raw_height,
            input_stride: config.raw_width,
            ..Default::default()
        };
        call(
            "hbn_vnode_set_ichn_attr(GDC)",
            crate::ffi::hbn_vnode_set_ichn_attr(gdc, CHN_ID, as_mut_void(&mut ichn)),
        )?;

        let mut ochn = crate::ffi::gdc_ochn_attr_t {
            output_width: config.out_width,
            output_height: config.out_height,
            output_stride: config.out_width,
        };
        call(
            "hbn_vnode_set_ochn_attr(GDC)",
            crate::ffi::hbn_vnode_set_ochn_attr(gdc, CHN_ID, as_mut_void(&mut ochn)),
        )?;

        let mut alloc = alloc_attr(3, crate::ffi::mem_usage_t_HB_MEM_USAGE_HW_GDC_OUT);
        call(
            "hbn_vnode_set_ochn_buf_attr(GDC)",
            crate::ffi::hbn_vnode_set_ochn_buf_attr(gdc, CHN_ID, &mut alloc),
        )?;

        Ok(())
    }

    /// Opens and configures the VSE node for full-size NV12 output.
    unsafe fn create_vse(&mut self, config: &Config) -> Result<(), String> {
        let mut vse = 0;
        call(
            "hbn_vnode_open(HB_VSE)",
            crate::ffi::hbn_vnode_open(
                crate::ffi::hb_vnode_type_e_HB_VSE,
                0,
                crate::ffi::AUTO_ALLOC_ID,
                &mut vse,
            ),
        )?;
        self.vse = Some(vse);

        let mut attr = crate::ffi::vse_attr_t::default();
        attr.fps.src = config.fps as u16;
        attr.fps.dst = config.fps as u16;
        call(
            "hbn_vnode_set_attr(VSE)",
            crate::ffi::hbn_vnode_set_attr(vse, as_mut_void(&mut attr)),
        )?;

        let mut ichn = crate::ffi::vse_ichn_attr_t {
            width: config.out_width,
            height: config.out_height,
            fmt: crate::ffi::enum_frame_format_e_FRM_FMT_NV12,
            bit_width: 8,
            ..Default::default()
        };
        call(
            "hbn_vnode_set_ichn_attr(VSE)",
            crate::ffi::hbn_vnode_set_ichn_attr(vse, CHN_ID, as_mut_void(&mut ichn)),
        )?;

        let mut ochn = crate::ffi::vse_ochn_attr_t {
            chn_en: crate::ffi::enum_cam_bool_e_CAM_TRUE,
            roi: crate::ffi::common_rect_t {
                x: 0,
                y: 0,
                w: config.out_width,
                h: config.out_height,
            },
            target_w: config.out_width,
            target_h: config.out_height,
            fmt: crate::ffi::enum_frame_format_e_FRM_FMT_NV12,
            bit_width: 8,
            fps: crate::ffi::frame_fps_ctrl_t {
                src: config.fps as u16,
                dst: config.fps as u16,
            },
            ..Default::default()
        };
        call(
            "hbn_vnode_set_ochn_attr(VSE)",
            crate::ffi::hbn_vnode_set_ochn_attr(vse, CHN_ID, as_mut_void(&mut ochn)),
        )?;

        let mut alloc = alloc_attr(
            3,
            crate::ffi::mem_usage_t_HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF
                | crate::ffi::mem_usage_t_HB_MEM_USAGE_HW_VIDEO_CODEC,
        );
        call(
            "hbn_vnode_set_ochn_buf_attr(VSE)",
            crate::ffi::hbn_vnode_set_ochn_buf_attr(vse, CHN_ID, &mut alloc),
        )?;

        Ok(())
    }

    /// Creates the vflow and binds VIN -> ISP -> GDC -> VSE.
    unsafe fn create_vflow(&mut self) -> Result<(), String> {
        let mut vflow = 0;
        call("hbn_vflow_create", crate::ffi::hbn_vflow_create(&mut vflow))?;
        self.vflow = Some(vflow);

        let vin = self.vin.ok_or("VIN node missing")?;
        let isp = self.isp.ok_or("ISP node missing")?;
        let gdc = self.gdc.ok_or("GDC node missing")?;
        let vse = self.vse.ok_or("VSE node missing")?;

        for (name, vnode) in [("VIN", vin), ("ISP", isp), ("GDC", gdc), ("VSE", vse)] {
            call(
                &format!("hbn_vflow_add_vnode({name})"),
                crate::ffi::hbn_vflow_add_vnode(vflow, vnode),
            )?;
        }

        call(
            "hbn_vflow_bind_vnode(VIN->ISP)",
            crate::ffi::hbn_vflow_bind_vnode(vflow, vin, CHN_ID, isp, CHN_ID),
        )?;
        call(
            "hbn_vflow_bind_vnode(ISP->GDC)",
            crate::ffi::hbn_vflow_bind_vnode(vflow, isp, CHN_ID, gdc, CHN_ID),
        )?;
        call(
            "hbn_vflow_bind_vnode(GDC->VSE)",
            crate::ffi::hbn_vflow_bind_vnode(vflow, gdc, CHN_ID, vse, CHN_ID),
        )?;

        Ok(())
    }

    /// Attaches the camera handle to the configured VIN node.
    unsafe fn attach_camera(&mut self) -> Result<(), String> {
        let camera = self.camera.ok_or("camera handle missing")?;
        let vin = self.vin.ok_or("VIN node missing")?;
        call(
            "hbn_camera_attach_to_vin",
            crate::ffi::hbn_camera_attach_to_vin(camera, vin),
        )?;
        self.attached = true;
        Ok(())
    }
}

/// Lease for one VSE frame that releases back to the SDK on drop.
pub struct FrameLease {
    /// VSE vnode that produced the frame.
    vnode: crate::ffi::hbn_vnode_handle_t,
    /// VSE channel id that produced the frame.
    channel: u32,
    /// SDK image and buffer metadata.
    image: crate::ffi::hbn_vnode_image_t,
    /// Whether the SDK frame has already been released.
    released: bool,
}

unsafe impl Send for FrameLease {}

impl FrameLease {
    /// Returns the SDK frame id.
    pub fn frame_id(&self) -> u32 {
        self.image.info.frame_id
    }

    /// Returns the SDK frame timestamp in nanoseconds.
    pub fn timestamp_ns(&self) -> u64 {
        self.image.info.timestamps
    }

    /// Returns the VSE graphic buffer backing this frame.
    pub fn buffer(&self) -> &crate::ffi::hb_mem_graphic_buf_t {
        &self.image.buffer
    }
}

impl Drop for FrameLease {
    /// Releases the leased VSE frame back to the SDK.
    fn drop(&mut self) {
        if self.released {
            return;
        }
        unsafe {
            crate::ffi::hbn_vnode_releaseframe(self.vnode, self.channel, &mut self.image);
        }
        self.released = true;
    }
}

impl Drop for VioPipeline {
    /// Stops, detaches, destroys, and closes SDK resources in dependency order.
    fn drop(&mut self) {
        unsafe {
            if self.started {
                if let Some(vflow) = self.vflow {
                    crate::ffi::hbn_vflow_stop(vflow);
                }
                self.started = false;
            }
            if self.attached {
                if let Some(camera) = self.camera {
                    crate::ffi::hbn_camera_detach_from_vin(camera);
                }
                self.attached = false;
            }
            if let Some(vflow) = self.vflow.take() {
                crate::ffi::hbn_vflow_destroy(vflow);
            }
            for vnode in [
                self.vse.take(),
                self.gdc.take(),
                self.isp.take(),
                self.vin.take(),
            ]
            .into_iter()
            .flatten()
            {
                crate::ffi::hbn_vnode_close(vnode);
            }
            if let Some(camera) = self.camera.take() {
                crate::ffi::hbn_camera_destroy(camera);
            }
        }
    }
}

/// SC132GS SDK configuration buffers with stable addresses.
struct Sc132gsConfig {
    /// MIPI config referenced by `camera`.
    _mipi: Box<crate::ffi::mipi_config_t>,
    /// Camera config passed to `hbn_camera_create`.
    camera: Box<crate::ffi::camera_config_t>,
}

impl Sc132gsConfig {
    /// Builds SDK sensor and camera config structs for one host.
    fn new(config: &Config, sensor: &SensorHost) -> Result<Self, String> {
        let mut mipi = Box::new(crate::ffi::mipi_config_t::default());
        mipi.rx_enable = 1;
        mipi.rx_attr.lane = 1;
        mipi.rx_attr.datatype = RAW10 as u16;
        mipi.rx_attr.fps = config.fps as u16;
        mipi.rx_attr.mclk = 1;
        mipi.rx_attr.mipiclk = 1200;
        mipi.rx_attr.width = config.raw_width as u16;
        mipi.rx_attr.height = config.raw_height as u16;
        mipi.rx_attr.linelenth = 1400;
        mipi.rx_attr.framelenth = 1500;
        mipi.rx_attr.settle = 20;
        mipi.rx_attr.channel_num = 1;
        mipi.rx_attr.channel_sel[0] = 0;

        let mut camera = Box::new(crate::ffi::camera_config_t::default());
        set_c_string(&mut camera.name, "sc132gs")?;
        set_c_string(&mut camera.calib_lname, "disable")?;
        camera.addr = sensor.i2c_addr as u32;
        camera.sensor_mode = crate::ffi::sensor_mode_e_NORMAL_M;
        camera.fps = config.fps;
        camera.format = RAW10;
        camera.width = config.raw_width;
        camera.height = config.raw_height;
        camera.gpio_enable_bit = 0x01;
        camera.gpio_level_bit = 0x00;
        camera.mipi_cfg = &mut *mipi;

        Ok(Self {
            _mipi: mipi,
            camera,
        })
    }
}

/// Builds VIN node attributes from the selected sensor host.
fn vin_node_attr(config: &Config, sensor: &SensorHost) -> crate::ffi::vin_node_attr_t {
    let mut attr = crate::ffi::vin_node_attr_t::default();
    attr.cim_attr.mipi_rx = sensor.mipi_rx_phy as u32;
    attr.cim_attr.vc_index = 0;
    attr.cim_attr.ipi_channel = 1;
    attr.cim_attr.cim_isp_flyby = 1;
    attr.cim_attr.func.enable_frame_id = 1;
    attr.cim_attr.func.set_init_frame_id = 0;
    attr.cim_attr.func.hdr_mode = crate::ffi::hdr_mode_NOT_HDR;
    attr.cim_attr.func.time_stamp_en = 0;
    attr.lpwm_attr.enable = 0;
    for chn in &mut attr.lpwm_attr.lpwm_chn_attr {
        chn.period = 1_000_000 / config.fps;
        chn.offset = 10;
        chn.duty_time = 100;
    }
    attr
}

/// Builds common SDK buffer allocation attributes.
fn alloc_attr(
    buffers_num: u32,
    extra_flags: crate::ffi::mem_usage_t,
) -> crate::ffi::hbn_buf_alloc_attr_t {
    crate::ffi::hbn_buf_alloc_attr_t {
        flags: (crate::ffi::mem_usage_t_HB_MEM_USAGE_CPU_READ_OFTEN
            | crate::ffi::mem_usage_t_HB_MEM_USAGE_CPU_WRITE_OFTEN
            | crate::ffi::mem_usage_t_HB_MEM_USAGE_CACHED
            | extra_flags) as i64,
        buffers_num,
        is_contig: 1,
    }
}

/// Casts a mutable Rust reference to the SDK's untyped attr pointer.
fn as_mut_void<T>(value: &mut T) -> *mut c_void {
    value as *mut T as *mut c_void
}

/// Converts an HBN integer return code into a contextual Rust result.
fn call(name: &str, ret: i32) -> Result<(), String> {
    if ret == 0 {
        return Ok(());
    }
    Err(format!("{name} failed ret={ret}{}", hbn_error_suffix(ret)))
}

/// Returns a readable SDK error suffix when available.
fn hbn_error_suffix(ret: i32) -> String {
    unsafe {
        let ptr = crate::ffi::hbn_err_info(ret);
        if ptr.is_null() {
            return String::new();
        }
        match CStr::from_ptr(ptr).to_str() {
            Ok(info) if !info.is_empty() => format!(" ({info})"),
            _ => String::new(),
        }
    }
}

/// Writes a Rust string into a fixed-size NUL-terminated C char array.
fn set_c_string<const N: usize>(dst: &mut [c_char; N], value: &str) -> Result<(), String> {
    let bytes = value.as_bytes();
    if bytes.len() >= N {
        return Err(format!("C string {value:?} does not fit in {N} bytes"));
    }
    dst.fill(0);
    for (out, byte) in dst.iter_mut().zip(bytes.iter().copied()) {
        *out = byte as c_char;
    }
    Ok(())
}
