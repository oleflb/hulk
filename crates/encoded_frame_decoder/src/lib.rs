//! FFmpeg-backed decoder for `types::encoded_frame::EncodedFrame` streams.
//!
//! The crate intentionally does not link to FFmpeg development libraries.
//! Instead it keeps a persistent `ffmpeg` process alive, feeds HEVC access units
//! through stdin, and reads fixed-size raw frames from stdout. This keeps the
//! workspace build independent of system FFmpeg headers while still allowing
//! runtime hardware acceleration via FFmpeg's `-hwaccel` support.
//!
//! FFmpeg must be available at `DecoderConfig::ffmpeg_path`, which defaults to
//! `ffmpeg` on `PATH`.
//! Decoded frames are returned as `ros_z::ZBuf`; use
//! `DecoderConfig::with_output_shm_pool_size` when the caller wants decoded
//! output allocated directly from a Zenoh SHM pool.
//! With the `orin-gst-cuda` feature, the crate also exposes an Orin-specific
//! GStreamer/NVMM/CUDA stereo decoder that packs matched left/right NV12 frames
//! into a guarded CUDA device allocation for same-process inference.
//!
//! ```no_run
//! use encoded_frame_decoder::{DecoderConfig, FfmpegHevcDecoder};
//! use types::encoded_frame::EncodedFrame;
//!
//! # fn main() -> color_eyre::Result<()> {
//! # let encoded: EncodedFrame = todo!();
//! let mut decoder = FfmpegHevcDecoder::spawn(DecoderConfig::new(1280, 1088))?;
//! if let Some(decoded) = decoder.decode_frame(&encoded)? {
//!     println!("decoded {} bytes", decoded.data.len());
//! }
//! # Ok(())
//! # }
//! ```

pub mod device_stereo;
pub mod orin;

use std::{
    collections::VecDeque,
    ffi::OsString,
    fmt,
    io::{self, Read, Write},
    path::PathBuf,
    process::{Child, ChildStdin, Command, Stdio},
    sync::{
        Arc, Mutex,
        mpsc::{self, Receiver, SyncSender},
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use color_eyre::{
    Result,
    eyre::{WrapErr, bail, ensure, eyre},
};
use ros_z::{ZBuf, shm::ShmProviderBuilder};
use types::encoded_frame::{EncodedFrame, EncodedFrameCodec};
use zenoh::Wait;
use zenoh::shm::{BlockOn, GarbageCollect, PosixShmProviderBackend, ShmProvider};

const STDERR_TAIL_BYTES: usize = 8 * 1024;

/// Hardware acceleration mode passed to FFmpeg.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum HardwareAcceleration {
    /// Let FFmpeg select a supported hardware decoder automatically.
    Auto,
    /// Disable FFmpeg hardware acceleration flags.
    Disabled,
    /// Use a named FFmpeg hardware accelerator such as `vaapi`, `cuda`, or `v4l2m2m`.
    Named(String),
}

/// Raw pixel format produced by the decoder process.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OutputPixelFormat {
    /// Planar Y followed by interleaved UV at half vertical resolution.
    Nv12,
    /// Packed 8-bit RGB pixels.
    Rgb24,
}

impl OutputPixelFormat {
    /// Returns the FFmpeg `-pix_fmt` argument for this format.
    pub fn ffmpeg_name(self) -> &'static str {
        match self {
            Self::Nv12 => "nv12",
            Self::Rgb24 => "rgb24",
        }
    }

    fn frame_size(self, width: u32, height: u32) -> Result<usize> {
        let pixels = (width as usize)
            .checked_mul(height as usize)
            .ok_or_else(|| eyre!("decoded frame size overflow"))?;
        match self {
            Self::Nv12 => {
                ensure!(
                    width.is_multiple_of(2),
                    "NV12 width must be even, got {width}"
                );
                ensure!(
                    height.is_multiple_of(2),
                    "NV12 height must be even, got {height}"
                );
                pixels
                    .checked_mul(3)
                    .and_then(|bytes| bytes.checked_div(2))
                    .ok_or_else(|| eyre!("NV12 decoded frame size overflow"))
            }
            Self::Rgb24 => pixels
                .checked_mul(3)
                .ok_or_else(|| eyre!("RGB decoded frame size overflow")),
        }
    }
}

/// Configuration for a persistent FFmpeg HEVC decoder process.
#[derive(Clone)]
pub struct DecoderConfig {
    /// Encoded stream width in pixels.
    pub width: u32,
    /// Encoded stream height in pixels.
    pub height: u32,
    /// Raw pixel format requested from FFmpeg.
    pub output_format: OutputPixelFormat,
    /// FFmpeg executable path.
    pub ffmpeg_path: PathBuf,
    /// Hardware acceleration mode.
    pub hardware_acceleration: HardwareAcceleration,
    /// Maximum time to wait for one decoded output frame after pushing input.
    pub frame_timeout: Duration,
    /// Extra arguments inserted before the `-i pipe:0` input.
    pub extra_input_args: Vec<OsString>,
    /// Optional SHM provider used for decoded output buffers.
    pub output_shm_provider: Option<Arc<ShmProvider<PosixShmProviderBackend>>>,
}

impl fmt::Debug for DecoderConfig {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("DecoderConfig")
            .field("width", &self.width)
            .field("height", &self.height)
            .field("output_format", &self.output_format)
            .field("ffmpeg_path", &self.ffmpeg_path)
            .field("hardware_acceleration", &self.hardware_acceleration)
            .field("frame_timeout", &self.frame_timeout)
            .field("extra_input_args", &self.extra_input_args)
            .field("output_shm_enabled", &self.output_shm_provider.is_some())
            .finish()
    }
}

impl DecoderConfig {
    /// Creates a decoder config for an HEVC stream of the given dimensions.
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            output_format: OutputPixelFormat::Nv12,
            ffmpeg_path: PathBuf::from("ffmpeg"),
            hardware_acceleration: HardwareAcceleration::Auto,
            frame_timeout: Duration::from_millis(250),
            extra_input_args: Vec::new(),
            output_shm_provider: None,
        }
    }

    /// Requests RGB output instead of the default NV12 output.
    pub fn with_rgb_output(mut self) -> Self {
        self.output_format = OutputPixelFormat::Rgb24;
        self
    }

    /// Sets the raw output pixel format.
    pub fn with_output_format(mut self, output_format: OutputPixelFormat) -> Self {
        self.output_format = output_format;
        self
    }

    /// Sets the FFmpeg hardware acceleration mode.
    pub fn with_hardware_acceleration(
        mut self,
        hardware_acceleration: HardwareAcceleration,
    ) -> Self {
        self.hardware_acceleration = hardware_acceleration;
        self
    }

    /// Sets the `ffmpeg` executable path.
    pub fn with_ffmpeg_path(mut self, ffmpeg_path: impl Into<PathBuf>) -> Self {
        self.ffmpeg_path = ffmpeg_path.into();
        self
    }

    /// Sets the maximum wait time for decoded output after each pushed access unit.
    pub fn with_frame_timeout(mut self, frame_timeout: Duration) -> Self {
        self.frame_timeout = frame_timeout;
        self
    }

    /// Adds an extra FFmpeg input argument before `-i pipe:0`.
    pub fn with_extra_input_arg(mut self, arg: impl Into<OsString>) -> Self {
        self.extra_input_args.push(arg.into());
        self
    }

    /// Allocates decoded output frames from a Zenoh SHM pool of `size_bytes`.
    pub fn with_output_shm_pool_size(mut self, size_bytes: usize) -> Result<Self> {
        self.output_shm_provider = Some(Arc::new(ShmProviderBuilder::new(size_bytes).build()?));
        Ok(self)
    }

    /// Uses an existing Zenoh SHM provider for decoded output frames.
    pub fn with_output_shm_provider(
        mut self,
        provider: Arc<ShmProvider<PosixShmProviderBackend>>,
    ) -> Self {
        self.output_shm_provider = Some(provider);
        self
    }
}

/// One decoded raw frame emitted by FFmpeg.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecodedFrame {
    /// Decoded frame width in pixels.
    pub width: u32,
    /// Decoded frame height in pixels.
    pub height: u32,
    /// Pixel format of `data`.
    pub pixel_format: OutputPixelFormat,
    /// Raw decoded frame bytes.
    pub data: ZBuf,
}

/// Persistent FFmpeg HEVC decoder.
///
/// The decoder keeps FFmpeg's process, codec state, and hardware context alive
/// across frames. Input access units are copied once into FFmpeg's stdin pipe;
/// decoded output is read directly into a heap or SHM-backed `ZBuf`.
pub struct FfmpegHevcDecoder {
    child: Child,
    stdin: ChildStdin,
    decoded_rx: Option<Receiver<io::Result<DecodedFrame>>>,
    reader: Option<JoinHandle<()>>,
    stderr_tail: StderrTail,
    stderr_reader: Option<JoinHandle<()>>,
    config: DecoderConfig,
    timed_out: bool,
}

impl FfmpegHevcDecoder {
    /// Spawns a persistent FFmpeg decoder process.
    pub fn spawn(config: DecoderConfig) -> Result<Self> {
        ensure!(config.width != 0, "decoder width must be nonzero");
        ensure!(config.height != 0, "decoder height must be nonzero");
        let frame_size = config
            .output_format
            .frame_size(config.width, config.height)?;

        let mut command = Command::new(&config.ffmpeg_path);
        command
            .arg("-hide_banner")
            .arg("-loglevel")
            .arg("error")
            .arg("-fflags")
            .arg("nobuffer")
            .arg("-flags")
            .arg("low_delay");

        match &config.hardware_acceleration {
            HardwareAcceleration::Auto => {
                command.arg("-hwaccel").arg("auto");
            }
            HardwareAcceleration::Disabled => {}
            HardwareAcceleration::Named(name) => {
                command.arg("-hwaccel").arg(name);
            }
        }

        command
            .args(&config.extra_input_args)
            .arg("-f")
            .arg("hevc")
            .arg("-i")
            .arg("pipe:0")
            .arg("-an")
            .arg("-sn")
            .arg("-dn")
            .arg("-f")
            .arg("rawvideo")
            .arg("-pix_fmt")
            .arg(config.output_format.ffmpeg_name())
            .arg("pipe:1")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());

        let mut child = command.spawn().wrap_err_with(|| {
            format!("failed to spawn ffmpeg at {}", config.ffmpeg_path.display())
        })?;
        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| eyre!("failed to capture ffmpeg stdin"))?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| eyre!("failed to capture ffmpeg stdout"))?;
        let stderr = child
            .stderr
            .take()
            .ok_or_else(|| eyre!("failed to capture ffmpeg stderr"))?;
        let (decoded_tx, decoded_rx) = mpsc::sync_channel(1);
        let reader_config = config.clone();
        let reader = thread::spawn(move || {
            read_decoded_frames(stdout, reader_config, frame_size, decoded_tx);
        });
        let stderr_tail = StderrTail::new();
        let stderr_reader_tail = stderr_tail.clone();
        let stderr_reader = thread::spawn(move || {
            read_stderr_tail(stderr, stderr_reader_tail);
        });

        Ok(Self {
            child,
            stdin,
            decoded_rx: Some(decoded_rx),
            reader: Some(reader),
            stderr_tail,
            stderr_reader: Some(stderr_reader),
            config,
            timed_out: false,
        })
    }

    /// Pushes one encoded frame and waits for the next decoded frame.
    ///
    /// Returns `Ok(None)` when FFmpeg accepted the data but did not emit a
    /// complete decoded frame before `DecoderConfig::frame_timeout` elapsed.
    /// After this happens, the decoder state is no longer synchronized with
    /// input metadata; drop this decoder and create a new one before decoding
    /// more frames.
    pub fn decode_frame(&mut self, frame: &EncodedFrame) -> Result<Option<DecodedFrame>> {
        ensure!(
            frame.codec == EncodedFrameCodec::Hevc,
            "unsupported encoded frame codec {:?}",
            frame.codec
        );
        ensure!(
            frame.width == self.config.width && frame.height == self.config.height,
            "encoded frame dimensions {}x{} do not match decoder config {}x{}",
            frame.width,
            frame.height,
            self.config.width,
            self.config.height
        );
        self.decode_hevc_access_unit(&frame.data)
    }

    /// Pushes one raw HEVC access unit and waits for the next decoded frame.
    pub fn decode_hevc_access_unit(&mut self, access_unit: &[u8]) -> Result<Option<DecodedFrame>> {
        ensure!(
            !self.timed_out,
            "ffmpeg decoder timed out; recreate it before decoding more frames"
        );
        ensure!(!access_unit.is_empty(), "HEVC access unit is empty");
        if let Err(err) = self.stdin.write_all(access_unit) {
            bail!(
                "failed to write HEVC data to ffmpeg: {err}{}",
                self.ffmpeg_diagnostics()
            );
        }
        if let Err(err) = self.stdin.flush() {
            bail!(
                "failed to flush ffmpeg stdin: {err}{}",
                self.ffmpeg_diagnostics()
            );
        }

        let decoded_rx = self
            .decoded_rx
            .as_ref()
            .ok_or_else(|| eyre!("ffmpeg decoded-frame reader is stopped"))?;
        match decoded_rx.recv_timeout(self.config.frame_timeout) {
            Ok(Ok(frame)) => Ok(Some(frame)),
            Ok(Err(err)) => Err(err).wrap_err(format!(
                "ffmpeg decoded-frame reader failed{}",
                self.ffmpeg_diagnostics()
            )),
            Err(mpsc::RecvTimeoutError::Timeout) => {
                self.timed_out = true;
                Ok(None)
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                bail!(
                    "ffmpeg decoded-frame reader stopped{}",
                    self.ffmpeg_diagnostics()
                )
            }
        }
    }

    fn ffmpeg_diagnostics(&mut self) -> String {
        let status = match self.child.try_wait() {
            Ok(Some(status)) => format!("ffmpeg exited with {status}"),
            Ok(None) => "ffmpeg is still running".to_string(),
            Err(err) => format!("failed to query ffmpeg status: {err}"),
        };
        let stderr = self.stderr_tail.snapshot_lossy();
        if stderr.is_empty() {
            format!(" ({status})")
        } else {
            format!(" ({status}; stderr tail: {stderr})")
        }
    }
}

impl Drop for FfmpegHevcDecoder {
    fn drop(&mut self) {
        drop(self.decoded_rx.take());
        let _ = self.child.kill();
        let _ = self.child.wait();
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
        if let Some(stderr_reader) = self.stderr_reader.take() {
            let _ = stderr_reader.join();
        }
    }
}

#[derive(Clone)]
struct StderrTail {
    bytes: Arc<Mutex<VecDeque<u8>>>,
}

impl StderrTail {
    fn new() -> Self {
        Self {
            bytes: Arc::new(Mutex::new(VecDeque::with_capacity(STDERR_TAIL_BYTES))),
        }
    }

    fn push(&self, bytes: &[u8]) {
        if let Ok(mut tail) = self.bytes.lock() {
            for &byte in bytes {
                if tail.len() == STDERR_TAIL_BYTES {
                    tail.pop_front();
                }
                tail.push_back(byte);
            }
        }
    }

    fn snapshot_lossy(&self) -> String {
        let Ok(tail) = self.bytes.lock() else {
            return "stderr tail unavailable".to_string();
        };
        let bytes = tail.iter().copied().collect::<Vec<_>>();
        String::from_utf8_lossy(&bytes).trim().to_string()
    }
}

fn read_decoded_frames(
    mut stdout: impl Read,
    config: DecoderConfig,
    frame_size: usize,
    decoded_tx: SyncSender<io::Result<DecodedFrame>>,
) {
    loop {
        match read_decoded_frame_data(&mut stdout, &config, frame_size) {
            Ok(data) => {
                let frame = DecodedFrame {
                    width: config.width,
                    height: config.height,
                    pixel_format: config.output_format,
                    data,
                };
                if decoded_tx.send(Ok(frame)).is_err() {
                    return;
                }
            }
            Err(err) if err.kind() == io::ErrorKind::UnexpectedEof => {
                let _ = decoded_tx.send(Err(io::Error::new(
                    err.kind(),
                    "ffmpeg stdout closed before a complete decoded frame",
                )));
                return;
            }
            Err(err) => {
                let _ = decoded_tx.send(Err(err));
                return;
            }
        }
    }
}

fn read_decoded_frame_data(
    stdout: &mut impl Read,
    config: &DecoderConfig,
    frame_size: usize,
) -> io::Result<ZBuf> {
    if let Some(provider) = config.output_shm_provider.as_ref() {
        let mut shm = provider
            .alloc(frame_size)
            .with_policy::<BlockOn<GarbageCollect>>()
            .wait()
            .map_err(|err| io::Error::other(format!("allocate decoded SHM frame: {err}")))?;
        stdout.read_exact(&mut shm[..frame_size])?;
        Ok(ZBuf::from(shm))
    } else {
        let mut data = vec![0; frame_size];
        stdout.read_exact(&mut data)?;
        Ok(ZBuf::from(data))
    }
}

fn read_stderr_tail(mut stderr: impl Read, tail: StderrTail) {
    let mut buffer = [0; 1024];
    loop {
        match stderr.read(&mut buffer) {
            Ok(0) => return,
            Ok(count) => tail.push(&buffer[..count]),
            Err(err) if err.kind() == io::ErrorKind::Interrupted => {}
            Err(_) => return,
        }
    }
}
