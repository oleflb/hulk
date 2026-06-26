use color_eyre::eyre::{Result, bail, ensure};

/// Runtime configuration for the strict SC132GS stereo pipeline.
#[derive(Clone, Debug)]
pub struct Config {
    /// Raw sensor frame width before rotation/rectification.
    pub raw_width: u32,
    /// Raw sensor frame height before rotation/rectification.
    pub raw_height: u32,
    /// Rectified NV12/VSE output width.
    pub out_width: u32,
    /// Rectified NV12/VSE output height.
    pub out_height: u32,
    /// Required per-camera frame rate.
    pub fps: u32,
    /// Target HEVC bitrate per camera in kilobits per second.
    pub bitrate_kbps: u32,
    /// MIPI host index for the left camera.
    pub left_host: i32,
    /// MIPI host index for the right camera.
    pub right_host: i32,
    /// Startup validation window in seconds.
    pub startup_timeout_s: u32,
}

impl Default for Config {
    /// Returns the required production defaults for the X5 stereo module.
    fn default() -> Self {
        Self {
            raw_width: 1088,
            raw_height: 1280,
            out_width: 1280,
            out_height: 1088,
            fps: 60,
            bitrate_kbps: 12_000,
            left_host: 2,
            right_host: 0,
            startup_timeout_s: 3,
        }
    }
}

impl Config {
    /// Rejects unsupported sensor modes and invalid runtime values.
    pub fn validate(&self) -> Result<()> {
        if self.raw_width != 1088 || self.raw_height != 1280 || self.fps != 60 {
            bail!(
                "only strict SC132GS 1088x1280@60 is accepted, got {}x{}@{}",
                self.raw_width,
                self.raw_height,
                self.fps
            );
        }
        ensure!(
            self.out_width != 0 && self.out_height != 0,
            "output dimensions must be nonzero"
        );
        if self.out_width != 1280 || self.out_height != 1088 {
            bail!(
                "only strict rectified output 1280x1088 is accepted, got {}x{}",
                self.out_width,
                self.out_height
            );
        }
        ensure!(
            self.left_host != self.right_host,
            "left-host and right-host must differ"
        );
        ensure!(self.bitrate_kbps != 0, "bitrate must be nonzero");
        ensure!(
            self.startup_timeout_s != 0,
            "startup-timeout-s must be nonzero"
        );
        Ok(())
    }
}
