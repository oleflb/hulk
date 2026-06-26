use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use clap::Parser;
use color_eyre::{Result, eyre::WrapErr, eyre::bail};
use ros_z::{
    node::Node,
    prelude::{Context, ContextBuilder, QosProfile},
    pubsub::Publisher,
    qos::QosDurability,
};
use ros2::{
    builtin_interfaces::time::Time,
    sensor_msgs::{camera_info::CameraInfo as RosCameraInfo, region_of_interest::RegionOfInterest},
    std_msgs::header::Header,
};
use tracing_subscriber::EnvFilter;
use types::{
    encoded_frame::{EncodedFrame as RosEncodedFrame, EncodedFrameCodec},
    time_wrapper::TimeWrapper,
};

mod cli;
mod driver;

#[cfg(x5cam_x5_target)]
pub(crate) use driver::{config, gdc, sensor};

use crate::{
    cli::Args,
    driver::{CalibrationInfo, CameraSetup, Channel, Config, EncodedFrame, Event, Stats, X5Camera},
};

const ENCODED_SHM_POOL_SIZE: usize = 64 * 1024 * 1024;
// Encoded frames are always large, but the generic CDR size hint does not
// include Arc<[u8]> payload bytes. Force SHM so the final Zenoh payload is
// shared-memory backed.
const ENCODED_SHM_THRESHOLD: usize = 1;

#[tokio::main]
async fn main() -> Result<()> {
    let args = cli::Args::parse();
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let config = Config::default();
    config.validate().wrap_err("invalid camera configuration")?;
    let context = build_context(args).await?;
    let node = context.create_node("camera_driver").build().await?;
    let publishers = Publishers::create(&node)
        .await
        .wrap_err("failed to create publishers")?;

    print_startup_banner(&config);
    let mut camera = X5Camera::open(&config).wrap_err("failed to open camera")?;

    let mut stats = Stats::new();
    let startup_deadline = Instant::now() + Duration::from_secs(config.startup_timeout_s as u64);
    let mut startup_validated = false;
    let mut last_print = Instant::now();

    loop {
        match camera.next_event(Duration::from_millis(500))? {
            Some(event) => match event {
                Event::CameraSetup(setup) => {
                    print_camera_setup(&setup);
                    stats.note_camera_setup(setup);
                }
                Event::Calibration(calibration) => {
                    print_calibration(&calibration);
                    publishers.publish_calibration(&calibration).await?;
                    stats.note_calibration();
                }
                Event::EncodedFrame(frame) => {
                    stats.note_encoded_frame(&frame);
                    publishers.publish_encoded_frame(&node, frame).await?;
                }
                Event::Error(err) => {
                    stats.note_error(err.channel);
                    bail!("camera {} worker failed: {}", err.channel, err.message);
                }
            },
            None => {}
        }

        if !startup_validated && Instant::now() >= startup_deadline {
            stats
                .validate_startup(&config)
                .wrap_err("startup validation failed")?;
            println!("startup validation: ok");
            startup_validated = true;
        }
        if last_print.elapsed() >= Duration::from_secs(1) {
            stats.print_tick();
            last_print = Instant::now();
        }
    }
}

pub async fn build_context(args: Args) -> Result<Context> {
    Ok(ContextBuilder::default()
        .with_shm_pool_size(ENCODED_SHM_POOL_SIZE)?
        .with_shm_threshold(ENCODED_SHM_THRESHOLD)
        .with_namespace(args.namespace)
        .with_mode("client")
        .with_router_endpoint(args.router)?
        .build()
        .await?)
}

/// Typed ROS-Z publishers exposed by the camera driver node.
struct Publishers {
    left_encoded_frame: Publisher<TimeWrapper<RosEncodedFrame>>,
    right_encoded_frame: Publisher<TimeWrapper<RosEncodedFrame>>,
    camera_info: Publisher<RosCameraInfo>,
    left_camera_info: Publisher<RosCameraInfo>,
    right_camera_info: Publisher<RosCameraInfo>,
}

impl Publishers {
    /// Creates all ROS-Z publishers used by the camera driver.
    pub async fn create(node: &Node) -> Result<Self> {
        let camera_info_qos = QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        };

        Ok(Self {
            left_encoded_frame: node
                .publisher::<TimeWrapper<RosEncodedFrame>>("inputs/left_encoded_frame")?
                .build()
                .await?,
            right_encoded_frame: node
                .publisher::<TimeWrapper<RosEncodedFrame>>("inputs/right_encoded_frame")?
                .build()
                .await?,
            camera_info: node
                .publisher::<RosCameraInfo>("inputs/camera_info")?
                .qos(camera_info_qos)
                .build()
                .await?,
            left_camera_info: node
                .publisher::<RosCameraInfo>("inputs/left_camera_info")?
                .qos(camera_info_qos)
                .build()
                .await?,
            right_camera_info: node
                .publisher::<RosCameraInfo>("inputs/right_camera_info")?
                .qos(camera_info_qos)
                .build()
                .await?,
        })
    }

    /// Publishes static calibration on transient-local camera-info topics.
    async fn publish_calibration(&self, calibration: &CalibrationInfo) -> Result<()> {
        let legacy_camera_info = legacy_left_camera_info(calibration);
        let left_camera_info = ros_camera_info(Channel::Left, calibration);
        let right_camera_info = ros_camera_info(Channel::Right, calibration);
        self.camera_info.publish(&legacy_camera_info).await?;
        self.left_camera_info.publish(&left_camera_info).await?;
        self.right_camera_info.publish(&right_camera_info).await?;
        Ok(())
    }

    /// Publishes one HEVC access unit to its side-specific encoded stream topic.
    async fn publish_encoded_frame(&self, node: &Node, frame: EncodedFrame) -> Result<()> {
        let publisher = match frame.channel {
            Channel::Left => &self.left_encoded_frame,
            Channel::Right => &self.right_encoded_frame,
        };
        if !publisher.has_subscribers() {
            return Ok(());
        }

        let time = time_from_timestamp_ns(frame.timestamp_ns).unwrap_or_else(|| node.clock().now());
        let message = encoded_frame_message(&frame);
        drop(frame);
        publisher
            .publish(&TimeWrapper {
                time,
                inner: message,
            })
            .await?;
        Ok(())
    }
}

fn time_from_timestamp_ns(timestamp_ns: u64) -> Option<ros_z::time::Time> {
    Some(ros_z::time::Time::from_nanos(
        i64::try_from(timestamp_ns).ok()?,
    ))
}

fn encoded_frame_message(frame: &EncodedFrame) -> RosEncodedFrame {
    RosEncodedFrame {
        frame_identifier: frame.frame_id,
        timestamp_ns: frame.timestamp_ns,
        presentation_timestamp_us: frame.pts_us,
        width: frame.width,
        height: frame.height,
        codec: EncodedFrameCodec::Hevc,
        data: Arc::from(frame.data()),
    }
}

fn legacy_left_camera_info(calibration: &CalibrationInfo) -> RosCameraInfo {
    let [fx, fy, cx, cy] = calibration.raw_left_intrinsics;

    RosCameraInfo {
        header: Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: "x5".to_string(),
        },
        height: calibration.raw_height,
        width: calibration.raw_width,
        distortion_model: calibration.distortion_model.clone(),
        d: calibration.raw_left_distortion.to_vec(),
        k: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: [
            calibration.rect_fx,
            0.0,
            calibration.rect_cx,
            0.0,
            0.0,
            calibration.rect_fy,
            calibration.rect_cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
        binning_x: 0,
        binning_y: 0,
        roi: RegionOfInterest::default(),
    }
}

fn ros_camera_info(channel: Channel, calibration: &CalibrationInfo) -> RosCameraInfo {
    let tx = match channel {
        Channel::Left => 0.0,
        Channel::Right => -calibration.rect_fx * calibration.baseline_m,
    };

    RosCameraInfo {
        header: Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: channel.frame_id().to_string(),
        },
        height: calibration.rect_height,
        width: calibration.rect_width,
        distortion_model: "plumb_bob".to_string(),
        d: vec![0.0; 5],
        k: [
            calibration.rect_fx,
            0.0,
            calibration.rect_cx,
            0.0,
            calibration.rect_fy,
            calibration.rect_cy,
            0.0,
            0.0,
            1.0,
        ],
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: [
            calibration.rect_fx,
            0.0,
            calibration.rect_cx,
            tx,
            0.0,
            calibration.rect_fy,
            calibration.rect_cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
        binning_x: 0,
        binning_y: 0,
        roi: RegionOfInterest::default(),
    }
}

fn print_startup_banner(config: &Config) {
    println!("camera-driver");
    println!("  backend     : {}", X5Camera::backend_name());
    println!("  calibration : SC132GS EEPROM (read-only)");
    println!(
        "  raw sensor  : {}x{} @ {} fps",
        config.raw_width, config.raw_height, config.fps
    );
    println!(
        "  output      : {}x{} HEVC @ {} fps, {} kbps per camera",
        config.out_width, config.out_height, config.fps, config.bitrate_kbps
    );
    println!("  sync        : SC132GS SLAVE_M with VIN LPWM trigger timestamps");
    println!(
        "  hosts       : left={} right={}",
        config.left_host, config.right_host
    );
}

fn print_camera_setup(setup: &CameraSetup) {
    println!(
        "camera {}: host={} sensor={:?} mode={:?} lpwm={} raw={}x{}@{} output={}x{} gdc={} hevc={} external_input={}",
        setup.channel,
        setup.host,
        setup.sensor,
        setup.sensor_mode,
        setup.lpwm_enabled,
        setup.raw_width,
        setup.raw_height,
        setup.fps,
        setup.out_width,
        setup.out_height,
        setup.gdc_enabled,
        setup.hevc_enabled,
        setup.external_encoder_input,
    );
}

fn print_calibration(calibration: &CalibrationInfo) {
    println!(
        "calibration: raw={}x{} rectified={}x{} model={} baseline={:.6}m",
        calibration.raw_width,
        calibration.raw_height,
        calibration.rect_width,
        calibration.rect_height,
        calibration.distortion_model,
        calibration.baseline_m,
    );
}
