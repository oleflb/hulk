//! Deterministic, headless inputs for exercising 3D localization.

pub mod config;
pub mod trajectory;

pub use config::{AssociationMode, SimulationConfig, VisualOdometryMode, VisualOdometryOutlier};
pub use trajectory::{PoseKeyframe, Scenario};
