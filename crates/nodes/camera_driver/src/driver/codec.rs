use std::fmt;
use std::ops::Deref;
use std::ptr::NonNull;
use std::sync::mpsc;
use std::time::Duration;

use super::{config::Config, ffi, vio::FrameLease};

/// RAII wrapper around a Hobot MediaCodec H.265 encoder instance.
pub struct H265Encoder {
    /// SDK-owned encoder context copied into stable heap storage.
    ctx: Box<ffi::media_codec_context_t>,
    /// Input-consumed callback kept alive for the encoder lifetime.
    _callback: ffi::media_codec_callback_t,
    /// Whether `hb_mm_mc_initialize` succeeded.
    initialized: bool,
    /// Whether `hb_mm_mc_start` succeeded.
    started: bool,
}

unsafe impl Send for H265Encoder {}

/// Sender used by encoded payload leases to return SDK output buffers.
#[derive(Clone)]
pub struct EncodedOutputReleaser {
    /// Release queue consumed by the encoder worker thread.
    tx: mpsc::Sender<EncodedOutputBuffer>,
}

/// Receiver for output-buffer release requests on the encoder worker thread.
pub struct EncodedOutputReleases {
    /// Release queue populated by dropped encoded payload leases.
    rx: mpsc::Receiver<EncodedOutputBuffer>,
}

/// Sendable wrapper around one SDK output buffer handle.
struct EncodedOutputBuffer(ffi::media_codec_buffer_t);

unsafe impl Send for EncodedOutputBuffer {}

/// Leased H.265 output buffer borrowed directly from MediaCodec.
pub struct EncodedFrameData {
    /// Start of the encoded bytes in the SDK output buffer.
    ptr: NonNull<u8>,
    /// Number of encoded bytes available from `ptr`.
    len: usize,
    /// Buffer release state consumed when this payload is dropped.
    release: Option<EncodedOutputRelease>,
}

unsafe impl Send for EncodedFrameData {}

/// Output buffer and release queue owned by one encoded payload.
struct EncodedOutputRelease {
    /// SDK output buffer to return to MediaCodec.
    buffer: EncodedOutputBuffer,
    /// Worker-thread release queue.
    releaser: EncodedOutputReleaser,
}

/// H.265 output metadata plus a zero-copy payload lease.
#[derive(Debug)]
pub struct EncodedOutput {
    /// Leased H.265 bytes for one output access unit.
    pub data: EncodedFrameData,
    /// Encoder presentation timestamp in microseconds.
    pub pts_us: u64,
}

/// Creates a release channel for one encoder worker.
pub fn encoded_output_release_channel() -> (EncodedOutputReleaser, EncodedOutputReleases) {
    let (tx, rx) = mpsc::channel();
    (EncodedOutputReleaser { tx }, EncodedOutputReleases { rx })
}

impl EncodedFrameData {
    /// Creates a zero-copy payload lease from a dequeued SDK stream buffer.
    fn new(
        output: ffi::media_codec_buffer_t,
        releaser: EncodedOutputReleaser,
    ) -> Result<Self, String> {
        let stream = unsafe { output.__bindgen_anon_1.vstream_buf };
        let ptr = NonNull::new(stream.vir_ptr.cast::<u8>())
            .ok_or_else(|| "encoded stream buffer has null data pointer".to_string())?;
        let len = usize::try_from(stream.size)
            .map_err(|_| format!("encoded stream buffer too large: {} bytes", stream.size))?;
        if len == 0 {
            return Err("encoded stream buffer is empty".to_string());
        }
        Ok(Self {
            ptr,
            len,
            release: Some(EncodedOutputRelease {
                buffer: EncodedOutputBuffer(output),
                releaser,
            }),
        })
    }

    /// Returns the encoded H.265 bytes in the leased SDK output buffer.
    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr.as_ptr(), self.len) }
    }

    /// Returns the encoded payload length in bytes.
    pub fn len(&self) -> usize {
        self.len
    }
}

impl Deref for EncodedFrameData {
    type Target = [u8];

    /// Borrows the encoded bytes without copying them.
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl fmt::Debug for EncodedFrameData {
    /// Prints payload metadata without dumping encoded bytes.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EncodedFrameData")
            .field("len", &self.len)
            .finish()
    }
}

impl Drop for EncodedFrameData {
    /// Returns the SDK output buffer to the encoder worker release queue.
    fn drop(&mut self) {
        if let Some(release) = self.release.take() {
            let _ = release.releaser.tx.send(release.buffer);
        }
    }
}

impl H265Encoder {
    /// Creates, configures, and starts an H.265 encoder for the requested config.
    pub fn create(config: &Config) -> Result<Self, String> {
        let mut encoder = Self {
            ctx: Box::new(ffi::media_codec_context_t::default()),
            _callback: ffi::media_codec_callback_t::default(),
            initialized: false,
            started: false,
        };
        unsafe {
            encoder.configure_params(config)?;
            call_media(
                "hb_mm_mc_initialize",
                ffi::hb_mm_mc_initialize(&mut *encoder.ctx),
            )?;
            encoder.initialized = true;

            encoder._callback.on_input_buffer_consumed = Some(on_input_buffer_consumed);
            call_media(
                "hb_mm_mc_set_input_buffer_listener",
                ffi::hb_mm_mc_set_input_buffer_listener(
                    &mut *encoder.ctx,
                    &encoder._callback,
                    std::ptr::null_mut(),
                ),
            )?;

            call_media(
                "hb_mm_mc_configure",
                ffi::hb_mm_mc_configure(&mut *encoder.ctx),
            )?;
            let startup = ffi::mc_av_codec_startup_params_t::default();
            call_media(
                "hb_mm_mc_start",
                ffi::hb_mm_mc_start(&mut *encoder.ctx, &startup),
            )?;
            encoder.started = true;
        }
        Ok(encoder)
    }

    /// Reports whether the encoder is configured for external input frames.
    pub fn external_input(&self) -> bool {
        unsafe {
            self.ctx
                .__bindgen_anon_1
                .video_enc_params
                .external_frame_buf
                != 0
        }
    }

    /// Queues a leased VSE NV12 frame as an external encoder input buffer.
    pub fn queue_external_frame(
        &mut self,
        lease: Box<FrameLease>,
        pts_us: u64,
    ) -> Result<(), String> {
        unsafe {
            let mut input = ffi::media_codec_buffer_t {
                type_: ffi::_media_codec_buffer_type_MC_VIDEO_FRAME_BUFFER,
                ..Default::default()
            };
            call_media(
                "hb_mm_mc_dequeue_input_buffer",
                ffi::hb_mm_mc_dequeue_input_buffer(&mut *self.ctx, &mut input, 2_000),
            )?;

            let frame = &mut input.__bindgen_anon_1.vframe_buf;
            let buffer = lease.buffer();
            if buffer.virt_addr[0].is_null()
                || buffer.virt_addr[1].is_null()
                || buffer.phys_addr[0] == 0
                || buffer.phys_addr[1] == 0
            {
                return Err(format!(
                    "VSE frame has invalid NV12 planes: vir={:?} phy={:?}",
                    &buffer.virt_addr[..2],
                    &buffer.phys_addr[..2]
                ));
            }

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
            frame.pix_fmt = ffi::_mc_pixel_format_MC_PIXEL_FORMAT_NV12;
            frame.stride = buffer.stride;
            frame.vstride = buffer.vstride;
            frame.pts = pts_us;
            frame.frame_end = 0;

            let raw_lease = Box::into_raw(lease);
            input.user_ptr = raw_lease.cast();
            let ret = ffi::hb_mm_mc_queue_input_buffer(&mut *self.ctx, &mut input, 2_000);
            if ret != 0 {
                drop(Box::from_raw(raw_lease));
                return Err(format!("hb_mm_mc_queue_input_buffer failed ret={ret}"));
            }
        }
        Ok(())
    }

    /// Dequeues an encoded output buffer, returning `None` on timeout.
    pub fn dequeue_output(
        &mut self,
        timeout_ms: i32,
        releaser: EncodedOutputReleaser,
    ) -> Result<Option<EncodedOutput>, String> {
        unsafe {
            let mut output = ffi::media_codec_buffer_t::default();
            let mut info = ffi::media_codec_output_buffer_info_t::default();
            let ret = ffi::hb_mm_mc_dequeue_output_buffer(
                &mut *self.ctx,
                &mut output,
                &mut info,
                timeout_ms,
            );
            if ret == ffi::HB_MEDIA_ERR_WAIT_TIMEOUT {
                return Ok(None);
            }
            if ret != 0 {
                return Err(format!("hb_mm_mc_dequeue_output_buffer failed ret={ret}"));
            }
            if output.type_ != ffi::_media_codec_buffer_type_MC_VIDEO_STREAM_BUFFER {
                ffi::hb_mm_mc_queue_output_buffer(&mut *self.ctx, &mut output, 0);
                return Ok(None);
            }
            let stream = output.__bindgen_anon_1.vstream_buf;
            let encoded = if stream.size == 0 || stream.vir_ptr.is_null() {
                ffi::hb_mm_mc_queue_output_buffer(&mut *self.ctx, &mut output, 0);
                None
            } else {
                Some(EncodedOutput {
                    data: EncodedFrameData::new(output, releaser)?,
                    pts_us: stream.pts,
                })
            };
            Ok(encoded)
        }
    }

    /// Releases every output buffer whose payload lease has been dropped.
    pub fn release_pending_outputs(
        &mut self,
        releases: &EncodedOutputReleases,
    ) -> Result<usize, String> {
        let mut released = 0;
        loop {
            match releases.rx.try_recv() {
                Ok(buffer) => {
                    self.release_output_buffer(buffer)?;
                    released += 1;
                }
                Err(mpsc::TryRecvError::Empty) => return Ok(released),
                Err(mpsc::TryRecvError::Disconnected) => return Ok(released),
            }
        }
    }

    /// Waits for one dropped payload lease and releases its SDK output buffer.
    pub fn wait_release_output(
        &mut self,
        releases: &EncodedOutputReleases,
        timeout: Duration,
    ) -> Result<bool, String> {
        match releases.rx.recv_timeout(timeout) {
            Ok(buffer) => {
                self.release_output_buffer(buffer)?;
                Ok(true)
            }
            Err(mpsc::RecvTimeoutError::Timeout) => Ok(false),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                Err("encoded output release channel disconnected".to_string())
            }
        }
    }

    /// Returns one SDK output buffer to MediaCodec.
    fn release_output_buffer(&mut self, mut buffer: EncodedOutputBuffer) -> Result<(), String> {
        unsafe {
            call_media(
                "hb_mm_mc_queue_output_buffer",
                ffi::hb_mm_mc_queue_output_buffer(&mut *self.ctx, &mut buffer.0, 0),
            )
        }
    }

    /// Writes all MediaCodec encoder parameters into the SDK context.
    unsafe fn configure_params(&mut self, config: &Config) -> Result<(), String> {
        self.ctx.codec_id = ffi::_media_codec_id_MEDIA_CODEC_ID_H265;
        self.ctx.encoder = 1;

        let ctx = &mut *self.ctx as *mut ffi::media_codec_context_t;
        {
            let params = &mut (*ctx).__bindgen_anon_1.video_enc_params;
            params.width = config.out_width as i32;
            params.height = config.out_height as i32;
            params.pix_fmt = ffi::_mc_pixel_format_MC_PIXEL_FORMAT_NV12;
            params.frame_buf_count = 5;
            params.external_frame_buf = 1;
            params.bitstream_buf_count = 8;
            params.bitstream_buf_size = align_up(
                (2 * 1024 * 1024).max(config.out_width * config.out_height * 3),
                1024,
            );
            params.gop_params.decoding_refresh_type = 2;
            // X5 Wave521CL supports all-I (1) and single-reference IPPP (9) only.
            params.gop_params.gop_preset_idx = 9;
            params.rot_degree = ffi::_mc_rotate_degree_MC_CCW_0;
            params.mir_direction = ffi::_mc_mirror_direction_MC_DIRECTION_NONE;
            params.frame_cropping_flag = 0;
            params.enable_user_pts = 1;
            params.rc_params.mode = ffi::_mc_video_rate_control_mode_MC_AV_RC_MODE_H265CBR;
        }

        call_media(
            "hb_mm_mc_get_rate_control_config",
            ffi::hb_mm_mc_get_rate_control_config(
                ctx,
                &mut (*ctx).__bindgen_anon_1.video_enc_params.rc_params,
            ),
        )?;

        let params = &mut (*ctx).__bindgen_anon_1.video_enc_params;
        params.rc_params.mode = ffi::_mc_video_rate_control_mode_MC_AV_RC_MODE_H265CBR;
        let rc = &mut params.rc_params.__bindgen_anon_1.h265_cbr_params;
        rc.intra_period = 60;
        rc.intra_qp = 30;
        rc.bit_rate = config.bitrate_kbps;
        rc.frame_rate = config.fps;
        rc.initial_rc_qp = 30;
        rc.vbv_buffer_size = 3000;
        rc.ctu_level_rc_enalbe = 1;
        rc.min_qp_I = 8;
        rc.max_qp_I = 50;
        rc.min_qp_P = 8;
        rc.max_qp_P = 50;
        rc.min_qp_B = 8;
        rc.max_qp_B = 50;
        rc.hvs_qp_enable = 1;
        rc.hvs_qp_scale = 2;
        rc.max_delta_qp = 10;
        rc.qp_map_enable = 0;
        Ok(())
    }
}

impl Drop for H265Encoder {
    /// Stops and releases the encoder in reverse initialization order.
    fn drop(&mut self) {
        unsafe {
            if self.started {
                ffi::hb_mm_mc_stop(&mut *self.ctx);
                self.started = false;
            }
            if self.initialized {
                ffi::hb_mm_mc_release(&mut *self.ctx);
                self.initialized = false;
            }
        }
    }
}

/// Releases the VSE frame lease once MediaCodec consumes the input buffer.
unsafe extern "C" fn on_input_buffer_consumed(
    _userdata: ffi::hb_ptr,
    buffer: *mut ffi::media_codec_buffer_t,
) {
    if buffer.is_null() {
        return;
    }
    let user_ptr = (*buffer).user_ptr;
    if user_ptr.is_null() {
        return;
    }
    (*buffer).user_ptr = std::ptr::null_mut();
    drop(Box::from_raw(user_ptr.cast::<FrameLease>()));
}

/// Aligns `value` upward to a power-of-two boundary.
fn align_up(value: u32, align: u32) -> u32 {
    debug_assert!(align.is_power_of_two());
    (value + align - 1) & !(align - 1)
}

/// Converts a MediaCodec integer return code into a Rust result.
fn call_media(name: &str, ret: i32) -> Result<(), String> {
    if ret == 0 {
        Ok(())
    } else {
        Err(format!("{name} failed ret={ret}"))
    }
}
