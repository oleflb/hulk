use std::error::Error;

use coordinate_systems::Pixel;
use linear_algebra::{IntoTransform, vector};
use mcap::Message;
use projection::camera_matrix::CameraMatrix;
use ros_z::time::Time;
use ros_z_cdr::{LittleEndian, from_bytes};
use serde::{Deserialize, de::DeserializeOwned};
use types::time_wrapper::TimeWrapper;

pub fn decode_message<T>(data: &[u8]) -> Result<T, Box<dyn Error>>
where
    T: DeserializeOwned,
{
    let (value, _consumed) = from_bytes::<T, LittleEndian>(cdr_payload(data))?;
    Ok(value)
}

pub fn cdr_payload(data: &[u8]) -> &[u8] {
    if data.len() >= 4 && matches!(&data[..4], [0, 1, 0, 0] | [0, 0, 0, 0]) {
        &data[4..]
    } else {
        data
    }
}

#[allow(dead_code)]
pub fn decode_time_prefix(data: &[u8]) -> Result<Time, Box<dyn Error>> {
    let data = cdr_payload(data);
    if data.len() < 12 {
        return Err("payload too short for Time prefix".into());
    }
    let secs = u64::from_le_bytes(data[0..8].try_into()?);
    let nanos = u32::from_le_bytes(data[8..12].try_into()?);
    let total_nanos = secs
        .saturating_mul(1_000_000_000)
        .saturating_add(u64::from(nanos));
    Ok(Time::from_nanos(total_nanos.min(i64::MAX as u64) as i64))
}

pub fn decode_recorded_camera_matrix(
    message: &Message<'_>,
) -> Result<TimeWrapper<CameraMatrix>, Box<dyn Error>> {
    let wire: WireTimeWrapper<WireCameraMatrix> = decode_recorded_message(message)?;
    Ok(TimeWrapper {
        time: wire.time,
        inner: wire.inner.into_camera_matrix(),
    })
}

pub fn decode_recorded_message<T>(message: &Message<'_>) -> Result<T, Box<dyn Error>>
where
    T: DeserializeOwned,
{
    decode_message(&message.data).map_err(|error| {
        format!(
            "failed to decode topic {} sequence {}: {error}",
            message.channel.topic, message.sequence
        )
        .into()
    })
}

#[derive(Deserialize)]
pub struct WireTimeWrapper<T> {
    time: Time,
    inner: T,
}

#[derive(Deserialize)]
struct WireCameraMatrix {
    ground_to_robot: WireIsometry3,
    robot_to_head: WireIsometry3,
    head_to_camera: WireIsometry3,
    intrinsics: WireIntrinsic,
    field_of_view: [f32; 2],
    horizon: Option<WireHorizon>,
    image_size: [f32; 2],
}

#[derive(Deserialize)]
struct WireIntrinsic {
    focals: [f32; 2],
    optical_center: [f32; 2],
}

#[derive(Deserialize)]
struct WireHorizon {
    vanishing_point: [f32; 2],
    normal: [f32; 2],
}

#[derive(Deserialize)]
pub struct WireIsometry3 {
    rotation: [f32; 4],
    translation: [f32; 3],
}

impl WireCameraMatrix {
    fn into_camera_matrix(self) -> CameraMatrix {
        let image_size: linear_algebra::Vector2<Pixel> =
            vector![self.image_size[0], self.image_size[1]];
        let normalized_focal = nalgebra::vector![
            self.intrinsics.focals[0] / image_size.inner.x,
            self.intrinsics.focals[1] / image_size.inner.y,
        ];
        let normalized_center = nalgebra::point![
            self.intrinsics.optical_center[0] / image_size.inner.x,
            self.intrinsics.optical_center[1] / image_size.inner.y,
        ];
        let _ = self.field_of_view;
        if let Some(horizon) = self.horizon {
            let _ = (horizon.vanishing_point, horizon.normal);
        }

        CameraMatrix::from_normalized_focal_and_center(
            normalized_focal,
            normalized_center,
            image_size,
            self.ground_to_robot.framed(),
            self.robot_to_head.framed(),
            self.head_to_camera.framed(),
        )
    }
}

impl WireIsometry3 {
    pub fn into_isometry(self) -> nalgebra::Isometry3<f32> {
        let rotation = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            self.rotation[3],
            self.rotation[0],
            self.rotation[1],
            self.rotation[2],
        ));
        nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(
                self.translation[0],
                self.translation[1],
                self.translation[2],
            ),
            rotation,
        )
    }

    fn framed<From, To>(self) -> linear_algebra::Isometry3<From, To> {
        self.into_isometry().framed_transform()
    }
}
