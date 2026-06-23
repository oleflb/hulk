use std::time::{Duration, Instant};

use super::config::Config;
use super::events::{CameraInfo, Channel, EncodedFrame};

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
    /// Total recoverable errors reported for this channel.
    errors: u64,
    /// Last VSE frame id associated with an encoded output.
    last_frame_id: Option<u32>,
    /// Count of nonconsecutive VSE frame ids.
    frame_gaps: u64,
    /// Last VSE timestamp in nanoseconds.
    last_timestamp_ns: Option<u64>,
    /// Last encoder PTS in microseconds.
    last_pts_us: Option<u64>,
    /// Wall-clock time when the first encoded frame arrived.
    first_frame_at: Option<Instant>,
    /// Wall-clock time when the last encoded frame arrived.
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
            errors: 0,
            last_frame_id: None,
            frame_gaps: 0,
            last_timestamp_ns: None,
            last_pts_us: None,
            first_frame_at: None,
            last_frame_at: None,
        }
    }
}

/// Runtime statistics and startup validation state.
pub struct Stats {
    /// Wall-clock time when statistics started.
    started_at: Instant,
    /// Wall-clock time when the current print window started.
    window_started_at: Instant,
    /// Per-channel frame, byte, and error counters.
    channel: [ChannelStats; 2],
    /// Camera setup events observed for each channel.
    camera_info: [Option<CameraInfo>; 2],
    /// Whether an EEPROM calibration event was observed.
    calibration_seen: bool,
}

impl Stats {
    /// Creates empty runtime statistics.
    pub fn new() -> Self {
        let now = Instant::now();
        Self {
            started_at: now,
            window_started_at: now,
            channel: [ChannelStats::new(), ChannelStats::new()],
            camera_info: [None, None],
            calibration_seen: false,
        }
    }

    /// Records a camera setup event for later validation and printing.
    pub fn note_camera_info(&mut self, info: CameraInfo) {
        let index = info.channel.index();
        self.camera_info[index] = Some(info);
    }

    /// Records that the EEPROM calibration event was emitted.
    pub fn note_calibration(&mut self) {
        self.calibration_seen = true;
    }

    /// Records a recoverable camera error.
    pub fn note_error(&mut self, channel: Channel) {
        self.channel[channel.index()].errors += 1;
    }

    /// Records an encoded H.265 frame and accounts for its payload bytes.
    pub fn note_encoded(&mut self, frame: EncodedFrame) {
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
        st.bytes_total += frame.data.len() as u64;
        st.bytes_window += frame.data.len() as u64;
        st.last_frame_id = Some(frame.frame_id);
        st.last_timestamp_ns = Some(frame.timestamp_ns);
        st.last_pts_us = Some(frame.pts_us);
        st.last_frame_at = Some(now);
    }

    /// Verifies that startup produced calibration, info, and encoded frames.
    pub fn validate_startup(&self, cfg: &Config) -> Result<(), String> {
        if !self.calibration_seen {
            return Err("calibration event was not emitted".to_string());
        }
        for (idx, expected_channel) in [Channel::Left, Channel::Right].iter().enumerate() {
            let info = self.camera_info[idx]
                .as_ref()
                .ok_or_else(|| format!("missing camera info for {expected_channel}"))?;
            if info.raw_width != cfg.raw_width
                || info.raw_height != cfg.raw_height
                || info.fps != cfg.fps
            {
                return Err(format!(
                    "{} raw mode mismatch: got {}x{}@{}, expected {}x{}@{}",
                    expected_channel,
                    info.raw_width,
                    info.raw_height,
                    info.fps,
                    cfg.raw_width,
                    cfg.raw_height,
                    cfg.fps,
                ));
            }
            if info.out_width != cfg.out_width || info.out_height != cfg.out_height {
                return Err(format!(
                    "{} output mismatch: got {}x{}, expected {}x{}",
                    expected_channel,
                    info.out_width,
                    info.out_height,
                    cfg.out_width,
                    cfg.out_height
                ));
            }
            if !info.gdc_enabled || !info.h265_enabled || !info.external_input {
                return Err(format!(
                    "{} flags invalid: gdc={} h265={} external_input={}",
                    expected_channel, info.gdc_enabled, info.h265_enabled, info.external_input
                ));
            }
        }

        let min_frames = (cfg.fps as u64 * cfg.startup_timeout_s as u64 * 9) / 10;
        for channel in [Channel::Left, Channel::Right] {
            let st = &self.channel[channel.index()];
            if st.frames_total < min_frames {
                return Err(format!(
                    "{} emitted {} encoded frames in startup window, expected at least {}",
                    channel, st.frames_total, min_frames
                ));
            }
        }
        Ok(())
    }

    /// Returns true once the configured maximum runtime has elapsed.
    pub fn should_stop(&self, max_seconds: Option<u64>) -> bool {
        max_seconds
            .map(|s| self.started_at.elapsed() >= Duration::from_secs(s))
            .unwrap_or(false)
    }

    /// Prints one-second window statistics and resets window counters.
    pub fn print_tick(&mut self) {
        let elapsed = self.window_started_at.elapsed().as_secs_f64().max(0.001);
        println!("stats {:.1}s window:", elapsed);
        for channel in [Channel::Left, Channel::Right] {
            let st = &mut self.channel[channel.index()];
            let fps = st.frames_window as f64 / elapsed;
            let mbps = st.bytes_window as f64 * 8.0 / elapsed / 1_000_000.0;
            let dims = self.camera_info[channel.index()]
                .as_ref()
                .map(|info| format!("{}x{}", info.out_width, info.out_height))
                .unwrap_or_else(|| "unknown".to_string());
            println!(
                "  {:5} frames={:<5} fps={:5.1} h265={:7.3}MB bitrate={:6.2}Mbps dim={} last_id={:?} pts_us={:?} gaps={} errors={}",
                channel,
                st.frames_window,
                fps,
                st.bytes_window as f64 / (1024.0 * 1024.0),
                mbps,
                dims,
                st.last_frame_id,
                st.last_pts_us,
                st.frame_gaps,
                st.errors,
            );
            st.frames_window = 0;
            st.bytes_window = 0;
        }
        self.window_started_at = Instant::now();
    }

    /// Prints final aggregate statistics.
    pub fn print_final(&self) {
        let elapsed = self.started_at.elapsed().as_secs_f64().max(0.001);
        println!("final {:.1}s:", elapsed);
        for channel in [Channel::Left, Channel::Right] {
            let st = &self.channel[channel.index()];
            println!(
                "  {:5} total_frames={} avg_fps={:.2} total_h265={:.2}MB gaps={} errors={}",
                channel,
                st.frames_total,
                st.frames_total as f64 / elapsed,
                st.bytes_total as f64 / (1024.0 * 1024.0),
                st.frame_gaps,
                st.errors,
            );
        }
    }
}
