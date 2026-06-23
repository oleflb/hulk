use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::Duration;

use super::{
    codec,
    config::Config,
    eeprom,
    events::{CalibrationInfo, CameraError, CameraInfo, Channel, EncodedFrame, Event},
    gdc, sensor, vio,
};

/// X5 camera backend with two VIO pipelines and encoders.
pub struct X5Camera {
    /// Receiver for setup, payload, and error events.
    rx: mpsc::Receiver<Event>,
    /// Shared stop flag observed by both worker threads.
    running: Arc<AtomicBool>,
    /// Worker thread join handles.
    workers: Vec<Worker>,
    /// GDC config buffers kept alive while the pipelines run.
    _gdc_bins: [gdc::GdcBin; 2],
    /// Global hbmem module handle kept open for SDK allocations.
    _hbmem: HbMemModule,
}

impl X5Camera {
    /// Returns the active backend name.
    pub fn backend_name() -> &'static str {
        "x5-real"
    }

    /// Opens sensors, calibration, GDC, VIO, and H.265 encoder resources.
    pub fn open(config: &Config) -> Result<Self, String> {
        let left_sensor = sensor::prepare_sc132gs_host(config.left_host)?;
        let right_sensor = sensor::prepare_sc132gs_host(config.right_host)?;
        let hbmem = HbMemModule::open()?;
        let calibration =
            eeprom::read_sc132gs_calibration([left_sensor.i2c_bus, right_sensor.i2c_bus])?;
        let (gdc_left, gdc_right) = gdc::generate_rectified_gdc_bins(
            &calibration,
            config.raw_width,
            config.raw_height,
            config.out_width,
            config.out_height,
        )?;
        let pipe_left = vio::VioPipeline::create(config, &left_sensor, &gdc_left)?;
        let pipe_right = vio::VioPipeline::create(config, &right_sensor, &gdc_right)?;
        let encoder_left = codec::H265Encoder::create(config)?;
        let encoder_right = codec::H265Encoder::create(config)?;

        let (tx, rx) = mpsc::channel();
        tx.send(Event::Calibration(CalibrationInfo {
            raw_width: config.raw_width,
            raw_height: config.raw_height,
            rect_width: config.out_width,
            rect_height: config.out_height,
            distortion_model: calibration.distortion_model.to_string(),
            baseline_m: calibration.baseline_m(),
        }))
        .map_err(|err| format!("queue calibration event: {err}"))?;
        tx.send(Event::CameraInfo(camera_info(
            Channel::Left,
            &left_sensor,
            config,
            encoder_left.external_input(),
        )))
        .map_err(|err| format!("queue left camera event: {err}"))?;
        tx.send(Event::CameraInfo(camera_info(
            Channel::Right,
            &right_sensor,
            config,
            encoder_right.external_input(),
        )))
        .map_err(|err| format!("queue right camera event: {err}"))?;

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
            rx,
            running,
            workers,
            _gdc_bins: [gdc_left, gdc_right],
            _hbmem: hbmem,
        })
    }

    /// Receives the next backend event within `timeout`.
    pub fn next_event(&mut self, timeout: Duration) -> Result<Option<Event>, String> {
        match self.rx.recv_timeout(timeout) {
            Ok(event) => Ok(Some(event)),
            Err(mpsc::RecvTimeoutError::Timeout) => Ok(None),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                Err("camera workers stopped and event channel disconnected".to_string())
            }
        }
    }

    /// Requests worker shutdown and joins both worker threads.
    pub fn stop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
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

/// Starts one capture/encode worker for a stereo channel.
fn spawn_worker(
    channel: Channel,
    config: Config,
    pipe: vio::VioPipeline,
    encoder: codec::H265Encoder,
    tx: mpsc::Sender<Event>,
    running: Arc<AtomicBool>,
) -> Worker {
    let handle = thread::spawn(move || {
        let mut pipe = pipe;
        let mut encoder = encoder;
        let (output_releaser, output_releases) = codec::encoded_output_release_channel();
        let mut outstanding_outputs = 0usize;

        if let Err(err) = pipe.start() {
            send_error(&tx, channel, format!("start VIO pipeline: {err}"));
            return;
        }

        let mut pts_index = 0u64;
        while running.load(Ordering::SeqCst) {
            if let Err(err) =
                release_pending_outputs(&mut encoder, &output_releases, &mut outstanding_outputs)
            {
                send_error(&tx, channel, format!("release H.265 output: {err}"));
                break;
            }

            let lease = match pipe.get_frame(1_000) {
                Ok(lease) => lease,
                Err(err) => {
                    if running.load(Ordering::SeqCst) {
                        send_error(&tx, channel, format!("get VSE frame: {err}"));
                    }
                    break;
                }
            };

            let frame_id = lease.frame_id();
            let timestamp_ns = lease.timestamp_ns();
            let pts_us = (pts_index * 1_000_000) / config.fps.max(1) as u64;
            pts_index = pts_index.wrapping_add(1);

            if let Err(err) = encoder.queue_external_frame(Box::new(lease), pts_us) {
                send_error(&tx, channel, format!("queue H.265 input: {err}"));
                break;
            }

            match encoder.dequeue_output(2_000, output_releaser.clone()) {
                Ok(Some(output)) => {
                    outstanding_outputs += 1;
                    let _ = tx.send(Event::EncodedFrame(EncodedFrame {
                        channel,
                        frame_id,
                        timestamp_ns,
                        data: output.data,
                        pts_us: output.pts_us,
                    }));
                    if let Err(err) = release_pending_outputs(
                        &mut encoder,
                        &output_releases,
                        &mut outstanding_outputs,
                    ) {
                        send_error(&tx, channel, format!("release H.265 output: {err}"));
                        break;
                    }
                }
                Ok(None) => {}
                Err(err) => {
                    send_error(&tx, channel, format!("dequeue H.265 output: {err}"));
                    break;
                }
            }
        }

        wait_for_output_releases(
            &tx,
            channel,
            &mut encoder,
            &output_releases,
            &mut outstanding_outputs,
        );
    });
    Worker {
        handle: Some(handle),
    }
}

/// Releases dropped encoded-output leases without blocking the worker.
fn release_pending_outputs(
    encoder: &mut codec::H265Encoder,
    releases: &codec::EncodedOutputReleases,
    outstanding: &mut usize,
) -> Result<(), String> {
    let released = encoder.release_pending_outputs(releases)?;
    *outstanding = outstanding.saturating_sub(released);
    Ok(())
}

/// Waits for outstanding zero-copy payload leases before encoder teardown.
fn wait_for_output_releases(
    tx: &mpsc::Sender<Event>,
    channel: Channel,
    encoder: &mut codec::H265Encoder,
    releases: &codec::EncodedOutputReleases,
    outstanding: &mut usize,
) {
    while *outstanding > 0 {
        match encoder.wait_release_output(releases, Duration::from_millis(100)) {
            Ok(true) => *outstanding -= 1,
            Ok(false) => {}
            Err(err) => {
                send_error(tx, channel, format!("wait for H.265 output release: {err}"));
                break;
            }
        }
    }
}

/// Builds a camera setup event for one prepared sensor.
fn camera_info(
    channel: Channel,
    sensor: &sensor::SensorHost,
    config: &Config,
    external_input: bool,
) -> CameraInfo {
    CameraInfo {
        channel,
        host: sensor.host,
        sensor_name: "sc132gs-1280p".to_string(),
        raw_width: config.raw_width,
        raw_height: config.raw_height,
        out_width: config.out_width,
        out_height: config.out_height,
        fps: config.fps,
        gdc_enabled: true,
        h265_enabled: true,
        external_input,
    }
}

/// Best-effort error event sender used from worker threads.
fn send_error(tx: &mpsc::Sender<Event>, channel: Channel, message: String) {
    let _ = tx.send(Event::Error(CameraError { channel, message }));
}

/// Process-wide hbmem module guard.
struct HbMemModule;

impl HbMemModule {
    /// Opens the hbmem module before any hbmem-backed SDK allocations.
    fn open() -> Result<Self, String> {
        let ret = unsafe { super::ffi::hb_mem_module_open() };
        if ret != 0 {
            return Err(format!("hb_mem_module_open failed ret={ret}"));
        }
        Ok(Self)
    }
}

impl Drop for HbMemModule {
    /// Closes the hbmem module when all dependent resources are gone.
    fn drop(&mut self) {
        unsafe {
            super::ffi::hb_mem_module_close();
        }
    }
}
