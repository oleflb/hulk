use std::path::PathBuf;

use clap::Parser;

use crate::camera::CameraSelection;

#[derive(Clone, Debug, Parser)]
#[command(about = "View camera_driver HEVC streams via ROS-Z and FFmpeg")]
pub(crate) struct Args {
    /// Router to connect to, for example tcp/42-head:7447.
    #[arg(long)]
    pub(crate) router: String,
    /// ROS-Z namespace containing the camera_driver topics.
    #[arg(long)]
    pub(crate) namespace: String,
    /// Camera stream to view.
    #[arg(long, value_enum, default_value_t = CameraSelection::Left)]
    pub(crate) camera: CameraSelection,
    /// FFmpeg executable path.
    #[arg(
        long = "ffmpeg",
        alias = "ffmpeg-path",
        value_name = "PATH",
        default_value = "ffmpeg"
    )]
    pub(crate) ffmpeg_path: PathBuf,
    /// Maximum time to wait for one decoded frame before restarting FFmpeg.
    #[arg(long, default_value_t = 250)]
    pub(crate) frame_timeout_ms: u64,
}
