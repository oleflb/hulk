use clap::Parser;
use color_eyre::Result;
use std::time::{Duration, Instant};

mod cli;
mod driver;

use driver::{Config, X5Camera};

fn main() -> Result<()> {
    let args = cli::Args::parse();

    let config = Config::default();

    println!("camera-driver");
    println!("  backend      : {}", X5Camera::backend_name());
    println!("  calibration  : SC132GS EEPROM (read-only)");
    println!(
        "  raw sensor   : {}x{} @ {} fps",
        config.raw_width, config.raw_height, config.fps
    );
    println!(
        "  output       : {}x{} @ {} fps",
        config.out_width, config.out_height, config.fps
    );
    println!("  h265 bitrate : {} kbps per camera", config.bitrate_kbps);
    println!(
        "  hosts        : left={} right={}",
        config.left_host, config.right_host
    );

    let mut camera = X5Camera::open(&config).wrap_err("failed to open camera")?;

    let mut stats = Stats::new();
    let mut last_print = Instant::now();
    let startup_deadline = Instant::now() + Duration::from_secs(config.startup_timeout_s as u64);
    let mut validated = false;

    loop {
        match camera.next_event(Duration::from_millis(500)) {
            Ok(Some(event)) => match event {
                Event::CameraInfo(info) => {
                    println!(
                        "camera {}: host={} sensor={} raw={}x{}@{} output={}x{} gdc={} h265={} external_input={}",
                        info.channel,
                        info.host,
                        info.sensor_name,
                        info.raw_width,
                        info.raw_height,
                        info.fps,
                        info.out_width,
                        info.out_height,
                        info.gdc_enabled,
                        info.h265_enabled,
                        info.external_input,
                    );
                    stats.note_camera_info(info);
                }
                Event::Calibration(calib) => {
                    println!(
                        "calibration: raw={}x{} rectified={}x{} model={} baseline={:.6}m",
                        calib.raw_width,
                        calib.raw_height,
                        calib.rect_width,
                        calib.rect_height,
                        calib.distortion_model,
                        calib.baseline_m,
                    );
                    stats.note_calibration();
                }
                Event::EncodedFrame(frame) => {
                    stats.note_encoded(frame);
                }
                Event::Error(err) => {
                    eprintln!("camera error: {}", err.message);
                    stats.note_error(err.channel);
                }
            },
            Ok(None) => {}
            Err(err) => {
                eprintln!("fatal: backend error: {err}");
                return ExitCode::from(1);
            }
        }

        if !validated && Instant::now() >= startup_deadline {
            if let Err(err) = stats.validate_startup(&config) {
                eprintln!("fatal: startup validation failed: {err}");
                return ExitCode::from(1);
            }
            validated = true;
            println!("startup validation passed");
        }

        if last_print.elapsed() >= Duration::from_secs(1) {
            stats.print_tick();
            last_print = Instant::now();
        }
    }
}

pub fn configure_rosz() {}
