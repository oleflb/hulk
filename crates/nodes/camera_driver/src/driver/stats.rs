use std::time::Instant;

use camera_driver_sys::SensorMode;
use color_eyre::eyre::{Result, bail, ensure};

use super::config::Config;
use super::events::{CameraSetup, Channel, EncodedFrame};

/// Per-channel counters used for status and startup validation.
#[derive(Clone, Debug)]
struct ChannelStats {
    /// Total encoded frames seen since startup.
    frames_total: u64,
    /// Encoded frames seen in the current print window.
    frames_window: u64,
    /// Total encoded bytes seen since startup.
    bytes_total: u64,
    /// Encoded bytes seen in the current print window.
    bytes_window: u64,
    /// Frames with a VIN trigger timestamp seen since startup.
    trigger_timestamps_total: u64,
    /// Frames with a VIN trigger timestamp seen in the current print window.
    trigger_timestamps_window: u64,
    /// Total recoverable errors reported for this channel.
    errors: u64,
    /// Last VSE frame id associated with an encoded frame.
    last_frame_id: Option<u32>,
    /// Count of nonconsecutive VSE frame ids.
    frame_gaps: u64,
    /// Last VSE timestamp in nanoseconds.
    last_timestamp_ns: Option<u64>,
    /// Wall-clock time when the first image frame arrived.
    first_frame_at: Option<Instant>,
    /// Wall-clock time when the last image frame arrived.
    last_frame_at: Option<Instant>,
}

impl ChannelStats {
    /// Creates empty counters for one channel.
    fn new() -> Self {
        Self {
            frames_total: 0,
            frames_window: 0,
            bytes_total: 0,
            bytes_window: 0,
            trigger_timestamps_total: 0,
            trigger_timestamps_window: 0,
            errors: 0,
            last_frame_id: None,
            frame_gaps: 0,
            last_timestamp_ns: None,
            first_frame_at: None,
            last_frame_at: None,
        }
    }
}

/// Stereo timestamp delta statistics for the current print window.
#[derive(Clone, Debug)]
struct StereoDeltaStats {
    samples_window: u64,
    last_delta_ns: Option<i128>,
    max_abs_delta_ns: u128,
}

impl StereoDeltaStats {
    /// Creates empty delta counters.
    fn new() -> Self {
        Self {
            samples_window: 0,
            last_delta_ns: None,
            max_abs_delta_ns: 0,
        }
    }

    /// Records one left-minus-right timestamp delta.
    fn note_delta(&mut self, delta_ns: i128) {
        self.samples_window += 1;
        self.last_delta_ns = Some(delta_ns);
        self.max_abs_delta_ns = self.max_abs_delta_ns.max(delta_ns.unsigned_abs());
    }

    /// Resets the current print window.
    fn reset_window(&mut self) {
        self.samples_window = 0;
        self.last_delta_ns = None;
        self.max_abs_delta_ns = 0;
    }
}

/// Runtime statistics and startup validation state.
pub struct Stats {
    /// Wall-clock time when the current print window started.
    window_started_at: Instant,
    /// Per-channel frame, byte, and error counters.
    channel: [ChannelStats; 2],
    /// Camera setup events observed for each channel.
    camera_setup: [Option<CameraSetup>; 2],
    /// Whether an EEPROM calibration event was observed.
    calibration_seen: bool,
    /// Left-minus-right timestamp delta diagnostics.
    stereo_delta: StereoDeltaStats,
}

impl Stats {
    /// Creates empty runtime statistics.
    pub fn new() -> Self {
        let now = Instant::now();
        Self {
            window_started_at: now,
            channel: [ChannelStats::new(), ChannelStats::new()],
            camera_setup: [None, None],
            calibration_seen: false,
            stereo_delta: StereoDeltaStats::new(),
        }
    }

    /// Records a camera setup event for later validation and printing.
    pub fn note_camera_setup(&mut self, setup: CameraSetup) {
        let index = setup.channel.index();
        self.camera_setup[index] = Some(setup);
    }

    /// Records that the EEPROM calibration event was emitted.
    pub fn note_calibration(&mut self) {
        self.calibration_seen = true;
    }

    /// Records a recoverable camera error.
    pub fn note_error(&mut self, channel: Channel) {
        self.channel[channel.index()].errors += 1;
    }

    /// Records an encoded frame and accounts for its payload bytes.
    pub fn note_encoded_frame(&mut self, frame: &EncodedFrame) {
        let now = Instant::now();
        let st = &mut self.channel[frame.channel.index()];
        if st.first_frame_at.is_none() {
            st.first_frame_at = Some(now);
        }
        if let Some(last) = st.last_frame_id {
            let expected = last.wrapping_add(1);
            if frame.frame_id != expected {
                st.frame_gaps += 1;
            }
        }
        st.frames_total += 1;
        st.frames_window += 1;
        st.bytes_total += frame.data_len() as u64;
        st.bytes_window += frame.data_len() as u64;
        if frame.trigger_timestamp_ns.is_some() {
            st.trigger_timestamps_total += 1;
            st.trigger_timestamps_window += 1;
        }
        st.last_frame_id = Some(frame.frame_id);
        st.last_timestamp_ns = Some(frame.timestamp_ns);
        st.last_frame_at = Some(now);

        let left = &self.channel[Channel::Left.index()];
        let right = &self.channel[Channel::Right.index()];
        if let (Some(left_id), Some(right_id), Some(left_timestamp), Some(right_timestamp)) = (
            left.last_frame_id,
            right.last_frame_id,
            left.last_timestamp_ns,
            right.last_timestamp_ns,
        ) {
            if left_id == right_id {
                self.stereo_delta
                    .note_delta(left_timestamp as i128 - right_timestamp as i128);
            }
        }
    }

    /// Verifies that startup produced calibration, setup, and encoded frames.
    pub fn validate_startup(&self, cfg: &Config) -> Result<()> {
        ensure!(self.calibration_seen, "calibration event was not emitted");
        for (idx, expected_channel) in [Channel::Left, Channel::Right].iter().enumerate() {
            let setup = self.camera_setup[idx].as_ref().ok_or_else(|| {
                color_eyre::eyre::eyre!("missing camera setup for {expected_channel}")
            })?;
            if setup.raw_width != cfg.raw_width
                || setup.raw_height != cfg.raw_height
                || setup.fps != cfg.fps
            {
                bail!(
                    "{} raw mode mismatch: got {}x{}@{}, expected {}x{}@{}",
                    expected_channel,
                    setup.raw_width,
                    setup.raw_height,
                    setup.fps,
                    cfg.raw_width,
                    cfg.raw_height,
                    cfg.fps,
                );
            }
            if setup.out_width != cfg.out_width || setup.out_height != cfg.out_height {
                bail!(
                    "{} output mismatch: got {}x{}, expected {}x{}",
                    expected_channel,
                    setup.out_width,
                    setup.out_height,
                    cfg.out_width,
                    cfg.out_height
                );
            }
            ensure!(
                setup.sensor_mode == SensorMode::Slave && setup.lpwm_enabled,
                "{} external trigger setup is not active: mode={:?} lpwm={}",
                expected_channel,
                setup.sensor_mode,
                setup.lpwm_enabled,
            );
            ensure!(setup.gdc_enabled, "{} GDC flag is false", expected_channel);
        }

        let min_frames = (cfg.fps as u64 * cfg.startup_timeout_s as u64 * 9) / 10;
        for channel in [Channel::Left, Channel::Right] {
            let st = &self.channel[channel.index()];
            if st.frames_total < min_frames {
                bail!(
                    "{} emitted {} encoded frames in startup window, expected at least {}",
                    channel,
                    st.frames_total,
                    min_frames
                );
            }
            if st.trigger_timestamps_total < min_frames {
                bail!(
                    "{} reported {} VIN trigger timestamps in startup window, expected at least {}",
                    channel,
                    st.trigger_timestamps_total,
                    min_frames
                );
            }
        }
        Ok(())
    }

    /// Prints one-second window statistics and resets window counters.
    pub fn print_tick(&mut self) {
        let elapsed = self.window_started_at.elapsed().as_secs_f64().max(0.001);
        println!("stats {:.1}s window:", elapsed);
        for channel in [Channel::Left, Channel::Right] {
            let st = &mut self.channel[channel.index()];
            let fps = st.frames_window as f64 / elapsed;
            let mbps = st.bytes_window as f64 * 8.0 / elapsed / 1_000_000.0;
            let dims = self.camera_setup[channel.index()]
                .as_ref()
                .map(|setup| format!("{}x{}", setup.out_width, setup.out_height))
                .unwrap_or_else(|| "unknown".to_string());
            println!(
                "  {:5} frames={:<5} fps={:5.1} hevc={:7.3}MB bandwidth={:6.2}Mbps dim={} trigger_ts={}/{} last_id={:?} gaps={} errors={}",
                channel,
                st.frames_window,
                fps,
                st.bytes_window as f64 / (1024.0 * 1024.0),
                mbps,
                dims,
                st.trigger_timestamps_window,
                st.frames_window,
                st.last_frame_id,
                st.frame_gaps,
                st.errors,
            );
            st.frames_window = 0;
            st.bytes_window = 0;
            st.trigger_timestamps_window = 0;
        }
        if self.stereo_delta.samples_window != 0 {
            println!(
                "  sync  samples={} last_delta={:.3}ms max_abs_delta={:.3}ms",
                self.stereo_delta.samples_window,
                self.stereo_delta.last_delta_ns.unwrap_or(0) as f64 / 1_000_000.0,
                self.stereo_delta.max_abs_delta_ns as f64 / 1_000_000.0,
            );
        }
        self.stereo_delta.reset_window();
        self.window_started_at = Instant::now();
    }
}
