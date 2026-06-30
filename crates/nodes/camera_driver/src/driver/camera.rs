use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use camera_driver_sys as sys;
use camera_driver_sys::{
    CodecId, EncoderConfig, FrameRate, GopPreset, HevcCbrConfig, ImageSize, Mirror, PixelFormat,
    PresentationTimestampUs, Rotation, SensorMode, SensorModule,
};
use color_eyre::eyre::{Result, WrapErr, bail, eyre};
use tracing::warn;

use super::{
    config::Config,
    eeprom::{self, StereoCalibration},
    events::{CalibrationInfo, CameraError, CameraSetup, Channel, EncodedFrame, Event},
    gdc, sensor, vio,
};

const EVENT_QUEUE_CAPACITY: usize = 4;
const ENCODER_DRAIN_TIMEOUT: Duration = Duration::from_secs(2);
const VSE_FRAME_TIMEOUT_MS: u32 = 1_000;
const MAX_CONSECUTIVE_VSE_GETFRAME_ERRORS: u32 = 5;

/// X5 camera backend with two VIO pipelines and HEVC encoders.
pub struct X5Camera {
    /// Receiver for setup, payload, and error events.
    rx: Option<mpsc::Receiver<Event>>,
    /// Shared stop flag observed by both worker threads.
    running: Arc<AtomicBool>,
    /// Worker thread join handles.
    workers: Vec<Worker>,
    /// Global hbmem module handle kept open for SDK allocations.
    _hbmem: sys::MemoryModule,
}

impl X5Camera {
    /// Returns the active backend name.
    pub fn backend_name() -> &'static str {
        "x5-real"
    }

    /// Opens sensors, calibration, GDC, VIO, and HEVC encoder resources.
    pub fn open(config: &Config) -> Result<Self> {
        config.validate()?;

        let left_sensor =
            sensor::prepare_sc132gs_host(config.left_host).wrap_err("prepare left SC132GS host")?;
        let right_sensor = sensor::prepare_sc132gs_host(config.right_host)
            .wrap_err("prepare right SC132GS host")?;
        let hbmem = sys::MemoryModule::open()?;
        let calibration =
            eeprom::read_sc132gs_calibration([left_sensor.i2c_bus, right_sensor.i2c_bus])
                .wrap_err("read SC132GS stereo calibration")?;
        let (gdc_left, gdc_right) = gdc::generate_rectified_gdc_bins(
            &calibration,
            config.raw_width,
            config.raw_height,
            config.out_width,
            config.out_height,
        )
        .wrap_err("generate rectified GDC maps")?;
        let mut pipe_left = vio::VioPipeline::create(config, &left_sensor, gdc_left)
            .wrap_err("create left VIO pipeline")?;
        let mut pipe_right = vio::VioPipeline::create(config, &right_sensor, gdc_right)
            .wrap_err("create right VIO pipeline")?;
        let encoder_left = create_hevc_encoder(config).wrap_err("create left HEVC encoder")?;
        let encoder_right = create_hevc_encoder(config).wrap_err("create right HEVC encoder")?;
        pipe_left.start().wrap_err("start left VIO pipeline")?;
        pipe_right.start().wrap_err("start right VIO pipeline")?;

        let (tx, rx) = mpsc::sync_channel(EVENT_QUEUE_CAPACITY);
        tx.send(Event::Calibration(calibration_info(config, &calibration)?))
            .map_err(|err| eyre!("queue calibration event: {err}"))?;
        tx.send(Event::CameraSetup(camera_setup(
            Channel::Left,
            &left_sensor,
            config,
            encoder_left.external_frame_input(),
        )))
        .map_err(|err| eyre!("queue left camera setup event: {err}"))?;
        tx.send(Event::CameraSetup(camera_setup(
            Channel::Right,
            &right_sensor,
            config,
            encoder_right.external_frame_input(),
        )))
        .map_err(|err| eyre!("queue right camera setup event: {err}"))?;

        let running = Arc::new(AtomicBool::new(true));
        let workers = vec![
            spawn_worker(
                Channel::Left,
                config.clone(),
                pipe_left,
                encoder_left,
                tx.clone(),
                running.clone(),
            ),
            spawn_worker(
                Channel::Right,
                config.clone(),
                pipe_right,
                encoder_right,
                tx,
                running.clone(),
            ),
        ];

        Ok(Self {
            rx: Some(rx),
            running,
            workers,
            _hbmem: hbmem,
        })
    }

    /// Receives the next backend event within `timeout`.
    pub fn next_event(&mut self, timeout: Duration) -> Result<Option<Event>> {
        let rx = self
            .rx
            .as_ref()
            .ok_or_else(|| eyre!("camera event receiver is stopped"))?;
        match rx.recv_timeout(timeout) {
            Ok(event) => Ok(Some(event)),
            Err(mpsc::RecvTimeoutError::Timeout) => Ok(None),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                bail!("camera workers stopped and event channel disconnected")
            }
        }
    }

    /// Requests worker shutdown and joins both worker threads.
    pub fn stop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        drop(self.rx.take());
        for worker in &mut self.workers {
            if let Some(handle) = worker.handle.take() {
                let _ = handle.join();
            }
        }
    }
}

impl Drop for X5Camera {
    /// Ensures workers and SDK resources stop before dropping the backend.
    fn drop(&mut self) {
        self.stop();
    }
}

/// Join handle for one camera worker thread.
struct Worker {
    /// Optional handle so `stop` can take and join it once.
    handle: Option<thread::JoinHandle<()>>,
}

struct PendingFrameMetadata {
    frame_id: u32,
    timestamp_ns: u64,
    trigger_timestamp_ns: Option<u64>,
    pts_us: u64,
}

/// Starts one capture/encode worker for a stereo channel.
fn spawn_worker(
    channel: Channel,
    config: Config,
    pipe: vio::VioPipeline,
    encoder: sys::MediaEncoder,
    tx: mpsc::SyncSender<Event>,
    running: Arc<AtomicBool>,
) -> Worker {
    let handle = thread::spawn(move || {
        let mut pipe = pipe;
        let mut encoder = encoder;

        let mut pts_index = 0u64;
        let mut consecutive_vse_getframe_errors = 0u32;
        let mut pending_metadata = VecDeque::new();
        while running.load(Ordering::SeqCst) {
            if let Err(err) = encoder.release_pending_outputs() {
                send_error(&tx, channel, format!("release HEVC output: {err:#}"));
                break;
            }
            encoder.release_pending_inputs();

            let lease = match pipe.get_frame(VSE_FRAME_TIMEOUT_MS) {
                Ok(lease) => {
                    if consecutive_vse_getframe_errors > 0 {
                        warn!(
                            ?channel,
                            recovered_after = consecutive_vse_getframe_errors,
                            "VSE frame dequeue recovered"
                        );
                        consecutive_vse_getframe_errors = 0;
                    }
                    lease
                }
                Err(err) => {
                    if !running.load(Ordering::SeqCst) {
                        break;
                    }
                    consecutive_vse_getframe_errors += 1;
                    if consecutive_vse_getframe_errors >= MAX_CONSECUTIVE_VSE_GETFRAME_ERRORS {
                        send_error(&tx, channel, format!("get VSE frame: {err:#}"));
                        break;
                    }
                    warn!(
                        ?channel,
                        consecutive_errors = consecutive_vse_getframe_errors,
                        max_errors = MAX_CONSECUTIVE_VSE_GETFRAME_ERRORS,
                        error = %err,
                        "VSE frame dequeue failed; retrying"
                    );
                    continue;
                }
            };

            let frame_id = lease.frame_id();
            let timestamp_ns = lease.timestamp_ns();
            let trigger_timestamp_ns = lease.trigger_timestamp_ns();
            let pts_us = (pts_index * 1_000_000) / config.fps.max(1) as u64;
            pts_index = pts_index.wrapping_add(1);
            let metadata = PendingFrameMetadata {
                frame_id,
                timestamp_ns,
                trigger_timestamp_ns,
                pts_us,
            };

            if let Err(err) = encoder.queue_external_frame(lease, PresentationTimestampUs(pts_us)) {
                send_error(&tx, channel, format!("queue HEVC input: {err:#}"));
                break;
            }
            pending_metadata.push_back(metadata);

            match encoder.dequeue_output(2_000) {
                Ok(Some(output)) => {
                    let metadata = match take_pending_metadata(&mut pending_metadata, output.pts_us)
                    {
                        Ok(metadata) => metadata,
                        Err(err) => {
                            send_error(
                                &tx,
                                channel,
                                format!("match HEVC output metadata: {err:#}"),
                            );
                            break;
                        }
                    };
                    let event = Event::EncodedFrame(EncodedFrame {
                        channel,
                        frame_id: metadata.frame_id,
                        timestamp_ns: metadata.timestamp_ns,
                        trigger_timestamp_ns: metadata.trigger_timestamp_ns,
                        width: config.out_width,
                        height: config.out_height,
                        pts_us: output.pts_us,
                        data: output.data,
                    });
                    match tx.try_send(event) {
                        Ok(()) => {}
                        Err(mpsc::TrySendError::Full(_)) => {}
                        Err(mpsc::TrySendError::Disconnected(_)) => break,
                    }
                    if let Err(err) = encoder.release_pending_outputs() {
                        send_error(&tx, channel, format!("release HEVC output: {err:#}"));
                        break;
                    }
                }
                Ok(None) => {}
                Err(err) => {
                    send_error(&tx, channel, format!("dequeue HEVC output: {err:#}"));
                    break;
                }
            }
        }

        wait_for_input_releases(&tx, channel, &mut encoder);
        wait_for_output_releases(&tx, channel, &mut encoder);
        if let Err(err) = encoder.shutdown(ENCODER_DRAIN_TIMEOUT) {
            send_error(&tx, channel, format!("shutdown HEVC encoder: {err:#}"));
        }
        if let Err(err) = pipe.shutdown() {
            send_error(&tx, channel, format!("shutdown VIO pipeline: {err:#}"));
        }
    });
    Worker {
        handle: Some(handle),
    }
}

fn take_pending_metadata(
    pending: &mut VecDeque<PendingFrameMetadata>,
    pts_us: u64,
) -> Result<PendingFrameMetadata> {
    let index = pending
        .iter()
        .position(|metadata| metadata.pts_us == pts_us)
        .ok_or_else(|| eyre!("encoder output PTS {pts_us} has no queued frame metadata"))?;
    if index > 0 {
        pending.drain(..index);
    }
    pending
        .pop_front()
        .ok_or_else(|| eyre!("encoder output PTS {pts_us} metadata queue was empty"))
}

/// Waits for outstanding zero-copy payload leases before encoder teardown.
fn wait_for_output_releases(
    tx: &mpsc::SyncSender<Event>,
    channel: Channel,
    encoder: &mut sys::MediaEncoder,
) {
    let deadline = Instant::now() + ENCODER_DRAIN_TIMEOUT;
    while encoder.pending_output_count() > 0 {
        if Instant::now() >= deadline {
            send_error(
                tx,
                channel,
                format!(
                    "timed out waiting for {} HEVC output release(s)",
                    encoder.pending_output_count()
                ),
            );
            break;
        }
        match encoder.wait_release_output(Duration::from_millis(100)) {
            Ok(true) => {}
            Ok(false) => {}
            Err(err) => {
                send_error(
                    tx,
                    channel,
                    format!("wait for HEVC output release: {err:#}"),
                );
                break;
            }
        }
    }
}

/// Waits for queued VSE input leases to be consumed before VIO teardown.
fn wait_for_input_releases(
    tx: &mpsc::SyncSender<Event>,
    channel: Channel,
    encoder: &mut sys::MediaEncoder,
) {
    encoder.release_pending_inputs();
    let deadline = Instant::now() + ENCODER_DRAIN_TIMEOUT;
    while encoder.pending_input_count() > 0 {
        if Instant::now() >= deadline {
            send_error(
                tx,
                channel,
                format!(
                    "timed out waiting for {} HEVC input release(s)",
                    encoder.pending_input_count()
                ),
            );
            break;
        }
        match encoder.wait_input_consumed(Duration::from_millis(100)) {
            Ok(true) => {}
            Ok(false) => {}
            Err(err) => {
                send_error(tx, channel, format!("wait for HEVC input release: {err:#}"));
                break;
            }
        }
    }
}

/// Builds a camera setup event for one prepared sensor.
fn camera_setup(
    channel: Channel,
    sensor: &sensor::SensorHost,
    config: &Config,
    external_encoder_input: bool,
) -> CameraSetup {
    CameraSetup {
        channel,
        host: sensor.host,
        sensor: SensorModule::Sc132gs,
        sensor_mode: SensorMode::Normal,
        lpwm_enabled: false,
        raw_width: config.raw_width,
        raw_height: config.raw_height,
        out_width: config.out_width,
        out_height: config.out_height,
        fps: config.fps,
        gdc_enabled: true,
        hevc_enabled: true,
        external_encoder_input,
    }
}

/// Builds ROS-compatible camera-info messages from EEPROM calibration.
fn calibration_info(config: &Config, calibration: &StereoCalibration) -> Result<CalibrationInfo> {
    let rectified = gdc::rectified_intrinsics(
        calibration,
        config.raw_width,
        config.raw_height,
        config.out_width,
        config.out_height,
    )?;
    Ok(CalibrationInfo {
        raw_width: calibration.width,
        raw_height: calibration.height,
        rect_width: config.out_width,
        rect_height: config.out_height,
        distortion_model: calibration.distortion_model.to_string(),
        raw_left_intrinsics: [
            calibration.left.fx,
            calibration.left.fy,
            calibration.left.cx,
            calibration.left.cy,
        ],
        raw_left_distortion: calibration.left.distortion,
        baseline_m: calibration.baseline_m(),
        rect_fx: rectified.fx,
        rect_fy: rectified.fy,
        rect_cx: rectified.cx,
        rect_cy: rectified.cy,
    })
}

fn create_hevc_encoder(config: &Config) -> Result<sys::MediaEncoder> {
    let size = ImageSize::new(config.out_width, config.out_height);
    let frame_rate = FrameRate::new(config.fps)?;
    let encoder_config = EncoderConfig {
        codec: CodecId::Hevc,
        size,
        pixel_format: PixelFormat::Nv12,
        external_frame_input: true,
        frame_buffer_count: 5,
        bitstream_buffer_count: 8,
        bitstream_buffer_size: align_up((2 * 1024 * 1024).max(size.width * size.height * 3), 1024),
        gop_preset: GopPreset::SingleReferenceIppp,
        rotation: Rotation::None,
        mirror: Mirror::None,
        enable_user_pts: true,
        rate_control: HevcCbrConfig {
            intra_period: 60,
            intra_qp: 30,
            bit_rate_kbps: config.bitrate_kbps,
            frame_rate,
            initial_rc_qp: 30,
            vbv_buffer_size: 3000,
            ctu_level_rc_enable: true,
            min_qp_i: 8,
            max_qp_i: 50,
            min_qp_p: 8,
            max_qp_p: 50,
            min_qp_b: 8,
            max_qp_b: 50,
            hvs_qp_enable: true,
            hvs_qp_scale: 2,
            max_delta_qp: 10,
            qp_map_enable: false,
        },
    };
    Ok(sys::MediaEncoder::create(encoder_config)?)
}

fn align_up(value: u32, align: u32) -> u32 {
    debug_assert!(align.is_power_of_two());
    (value + align - 1) & !(align - 1)
}

/// Error event sender used from worker threads.
fn send_error(tx: &mpsc::SyncSender<Event>, channel: Channel, message: String) {
    let _ = tx.send(Event::Error(CameraError { channel, message }));
}
