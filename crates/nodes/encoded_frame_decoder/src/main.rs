use std::{path::PathBuf, sync::Arc};

use clap::Parser;
use color_eyre::Result;
use encoded_frame_decoder_node::{DecoderNodeConfig, run};
use ros_z::prelude::*;
use tracing_subscriber::EnvFilter;

const ROS_Z_SHM_POOL_SIZE: usize = 128 * 1024 * 1024;

#[derive(Debug, Parser)]
struct Args {
    /// Router to connect to.
    #[arg(long)]
    router: String,
    /// ros-z namespace.
    #[arg(long)]
    namespace: String,
    /// FFmpeg executable path.
    #[arg(long)]
    ffmpeg_path: Option<PathBuf>,
    /// SHM pool size for decoded NV12 frame payloads.
    #[arg(long)]
    decoded_shm_pool_size: Option<usize>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();
    let decoder_config = decoder_config_from_args(&args);

    let context = Arc::new(
        ContextBuilder::default()
            .with_shm_pool_size(ROS_Z_SHM_POOL_SIZE)?
            .with_shm_threshold(1)
            .with_namespace(args.namespace)
            .with_mode("client")
            .with_router_endpoint(args.router)?
            .build()
            .await?,
    );

    run(context, decoder_config).await
}

fn decoder_config_from_args(args: &Args) -> DecoderNodeConfig {
    let mut config = DecoderNodeConfig::default();
    if let Some(ffmpeg_path) = &args.ffmpeg_path {
        config.ffmpeg_path = ffmpeg_path.clone();
    }
    if let Some(decoded_shm_pool_size) = args.decoded_shm_pool_size {
        config.decoded_shm_pool_size = decoded_shm_pool_size;
    }
    config
}
