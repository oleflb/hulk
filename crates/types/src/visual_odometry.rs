use ros_z::{Message, time::Time};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize, Message)]
pub struct VisualOdometryDelta {
    pub previous_time: Time,
    pub current_time: Time,
    /// Transformation from the current left-camera frame to the previous one.
    pub current_left_camera_to_previous_left_camera: nalgebra::Isometry3<f32>,
}
