use clap::Parser;
use color_eyre::{
    Result,
    eyre::{WrapErr, bail, eyre},
};
use std::time::{Duration, Instant};
use tracing_subscriber::EnvFilter;

use ros_z::{
    node::Node,
    prelude::{Context, ContextBuilder},
    pubsub::Publisher,
};

mod cli;
mod driver;

use driver::{CalibrationInfo as Calibration, CameraInfo, Config, EncodedFrame, X5Camera};

use crate::{
    cli::Args,
    driver::{Event, Stats},
};

type PendingPublisher<T> = Publisher<T, ros_z::dynamic::DynamicCdrCodec>;

#[tokio::main]
async fn main() -> Result<()> {
    let args = cli::Args::parse();
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let config = Config::default();
    let context = build_context(args).await?;
    let node = context.create_node("camera_driver").build().await?;
    let publishers = Publishers::create(&node).wrap_err("failed to create publishers")?;

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

    let mut camera = X5Camera::open(&config)
        .map_err(|err| eyre!(err))
        .wrap_err("failed to open camera")?;

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
                    tracing::warn!("camera error: {}", err.message);
                    stats.note_error(err.channel);
                }
            },
            Ok(None) => {}
            Err(err) => {
                bail!("fatal: backend error: {err}");
            }
        }
    }
}

pub async fn build_context(args: Args) -> Result<Context> {
    Ok(ContextBuilder::default()
        .with_namespace(args.namespace)
        .with_mode("client")
        .with_router_endpoint(args.router)?
        .build()
        .await?)
}

struct Publishers {
    frames: PendingPublisher<EncodedFrame>,
    calibration: PendingPublisher<Calibration>,
    camera_info: PendingPublisher<CameraInfo>,
}

impl Publishers {
    pub fn create(node: &Node) -> Result<Self> {
        todo!()
    }
}
