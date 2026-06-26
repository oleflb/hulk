pub(crate) mod config;
mod events;
mod stats;

#[cfg(x5cam_x5_target)]
mod camera;
#[cfg(x5cam_x5_target)]
mod eeprom;
#[cfg(x5cam_x5_target)]
pub(crate) mod gdc;
#[cfg(x5cam_x5_target)]
pub(crate) mod sensor;
#[cfg(not(x5cam_x5_target))]
mod stub;
#[cfg(x5cam_x5_target)]
mod vio;

#[cfg(x5cam_x5_target)]
pub use camera::X5Camera;
pub use config::Config;
pub use events::{CalibrationInfo, CameraSetup, Channel, EncodedFrame, Event};
pub use stats::Stats;
#[cfg(not(x5cam_x5_target))]
pub use stub::X5Camera;
