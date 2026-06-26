use std::time::Duration;

use color_eyre::eyre::{Result, bail};

use super::{config::Config, events::Event};

/// Host-side placeholder used for editor analysis without the X5 SDK/sysroot.
pub struct X5Camera;

impl X5Camera {
    /// Returns the active backend name.
    pub fn backend_name() -> &'static str {
        "x5-stub"
    }

    /// Fails clearly when the host-only build is run directly.
    pub fn open(_config: &Config) -> Result<Self> {
        bail!("X5 camera backend requires an aarch64 Linux target with the X5 SDK")
    }

    /// The stub never produces camera events.
    pub fn next_event(&mut self, _timeout: Duration) -> Result<Option<Event>> {
        bail!("X5 camera backend is not available for this build target")
    }
}
