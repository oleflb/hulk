use std::time::Duration;

use coordinate_systems::ImuReference;
use linear_algebra::Orientation3;
use ros_z::time::Time;
use types::{time_wrapper::TimeWrapper, visual_localization::AssociationGeometry};

/// Reject invalid raw IMU angles before constructing an exposure attitude.
pub fn exposure_attitude(roll: f32, pitch: f32, yaw: f32) -> Option<Orientation3<ImuReference>> {
    [roll, pitch, yaw]
        .iter()
        .all(|angle| angle.is_finite())
        .then(|| Orientation3::from_euler_angles(roll, pitch, yaw))
}

/// Interpolate only within a finite, sufficiently close pair of source-time samples.
pub fn interpolate_attitude(
    before: &TimeWrapper<Orientation3<ImuReference>>,
    after: &TimeWrapper<Orientation3<ImuReference>>,
    time: Time,
    max_gap: Duration,
) -> Option<Orientation3<ImuReference>> {
    if time < before.time
        || time > after.time
        || !before
            .inner
            .inner
            .coords
            .iter()
            .all(|value| value.is_finite())
        || !after
            .inner
            .inner
            .coords
            .iter()
            .all(|value| value.is_finite())
    {
        return None;
    }
    let gap = after.time.duration_since(before.time);
    if gap > max_gap {
        return None;
    }
    let mut orientation = if gap.is_zero() {
        before.inner
    } else {
        before.inner.slerp(
            after.inner,
            time.duration_since(before.time).as_secs_f32() / gap.as_secs_f32(),
        )
    };
    orientation.inner.renormalize();
    orientation
        .inner
        .coords
        .iter()
        .all(|value| value.is_finite())
        .then_some(orientation)
}

/// Select a complete preceding snapshot from the latest eligible estimate's lifecycle.
pub fn geometry_for_image(
    image_time: Time,
    history: impl IntoIterator<Item = AssociationGeometry>,
    latest: &AssociationGeometry,
    max_age: Duration,
) -> Option<AssociationGeometry> {
    // Include latest because the independently populated history may not contain it yet.
    std::iter::once(latest.clone())
        .chain(history)
        .filter(|geometry| {
            geometry.last_successful_solve <= image_time
                && geometry.last_successful_solve.abs_diff(image_time) <= max_age
                && geometry.epoch == latest.epoch
                && geometry.generation == latest.generation
        })
        .min_by_key(|geometry| {
            (
                geometry.last_successful_solve.abs_diff(image_time),
                geometry.last_successful_solve,
            )
        })
}
