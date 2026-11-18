use std::{boxed::Box, future::Future, pin::Pin, sync::Arc};

use booster::ImuState;
use color_eyre::Result;
use coordinate_systems::Odometry;
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::Pose2;
use ros_z::cache::CacheInner;
use ros_z::prelude::*;
use ros_z::time::Time;
use ros_z_streams::CreateAnnouncingPublisher;
use types::fall_detection::FallDetection;
use types::odometry::{KINEMATIC_ODOMETRY_TOPIC, KinematicOdometryDelta};
use types::time_wrapper::TimeWrapper;

mod estimator;

pub use estimator::{EstimatorInput, OdometryEstimator, Parameters};

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("odometry").build().await?;

    let parameters = node.bind_parameter_as::<Parameters>("odometry")?;
    let imu_state_sub = node
        .subscriber::<ImuState>("inputs/imu_state")
        .build()
        .await?;
    let robot_kinematics_cache = node
        .subscriber::<TimeWrapper<RobotKinematics>>("robot_kinematics")
        .cache(10)
        .with_stamp(|wrapper| wrapper.time)
        .build()
        .await?;
    let fall_detection_cache = node
        .subscriber::<FallDetection>("fall_detection/status")
        .cache(10)
        .build()
        .await?;
    let odometry_pub = node
        .announcing_publisher::<Pose2<Odometry>>("inputs/odometry")
        .await?;

    let mut estimator = OdometryEstimator::default();
    let deltas = node
        .publisher::<KinematicOdometryDelta>(KINEMATIC_ODOMETRY_TOPIC)
        .build()
        .await?;
    let mut imu_history = CacheInner::new(128);

    loop {
        let parameters = parameters.snapshot().typed().clone();
        let imu_state = imu_state_sub.recv_with_metadata().await?;
        let imu_time = imu_state.source_time;
        let imu_state = imu_state.into_message();
        imu_history.insert(
            imu_time,
            TimeWrapper {
                time: imu_time,
                inner: imu_state,
            },
        );
        let robot_kinematics = robot_kinematics_cache.get_before(imu_time);
        let (imu_state, robot_kinematics) =
            align_input(imu_time, imu_state, &imu_history, robot_kinematics);
        let fall_detection = fall_detection_cache.get_nearest(imu_time);

        if let Some(pose) = estimator.update(
            EstimatorInput {
                time: imu_time,
                imu_state: &imu_state,
                robot_kinematics: robot_kinematics.as_deref(),
                fall_detection: fall_detection
                    .as_deref()
                    .filter(|fall| fall.is_valid(node.clock().now())),
            },
            &parameters,
        ) {
            odometry_pub
                .announce(imu_time)
                .await?
                .publish(&pose)
                .await?;
        }
        if let Some(delta) = estimator.take_delta() {
            deltas.publish(&delta).await?;
        }
    }
}

/// Align attitude to the last encoder at or before `time`, admitting only bracketed fresh samples.
pub fn align_input(
    time: Time,
    imu_state: ImuState,
    imu_history: &CacheInner<TimeWrapper<ImuState>>,
    robot_kinematics: Option<Arc<TimeWrapper<RobotKinematics>>>,
) -> (ImuState, Option<Arc<TimeWrapper<RobotKinematics>>>) {
    let robot_kinematics =
        robot_kinematics.filter(|sample| sample.time.abs_diff(time) <= estimator::MAX_SAMPLE_GAP);
    let aligned_imu = robot_kinematics
        .as_ref()
        .and_then(|sample| imu_at(imu_history, sample.time));
    (
        aligned_imu.unwrap_or(imu_state),
        robot_kinematics.filter(|_| aligned_imu.is_some()),
    )
}

fn imu_at(history: &CacheInner<TimeWrapper<ImuState>>, time: Time) -> Option<ImuState> {
    let before = history.get_before(time)?;
    let after = history.get_after(time)?;
    let gap = after.time.duration_since(before.time);
    if gap > estimator::MAX_SAMPLE_GAP {
        return None;
    }
    let before_rpy = before.inner.roll_pitch_yaw.inner;
    let after_rpy = after.inner.roll_pitch_yaw.inner;
    if !before_rpy
        .iter()
        .chain(after_rpy.iter())
        .all(|v| v.is_finite())
    {
        return None;
    }
    let mut imu = before.inner;
    if !gap.is_zero() {
        let orientation =
            nalgebra::UnitQuaternion::from_euler_angles(before_rpy.x, before_rpy.y, before_rpy.z)
                .slerp(
                    &nalgebra::UnitQuaternion::from_euler_angles(
                        after_rpy.x,
                        after_rpy.y,
                        after_rpy.z,
                    ),
                    time.duration_since(before.time).as_secs_f32() / gap.as_secs_f32(),
                );
        let (roll, pitch, yaw) = orientation.euler_angles();
        imu.roll_pitch_yaw.inner = nalgebra::Vector3::new(roll, pitch, yaw);
    }
    Some(imu)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn attitude_is_interpolated_at_encoder_source_time_without_extrapolation() {
        let mut history = CacheInner::new(2);
        for (milliseconds, yaw) in [(1000, 3.0), (1100, -3.0)] {
            let time = Time::from_nanos(milliseconds * 1_000_000);
            let mut imu = ImuState::default();
            imu.roll_pitch_yaw.inner.z = yaw;
            history.insert(time, TimeWrapper { time, inner: imu });
        }

        let encoder_time = Time::from_nanos(1_025_000_000);
        let aligned = imu_at(&history, encoder_time).unwrap();
        let expected_yaw = 3.0 + (std::f32::consts::TAU - 6.0) * 0.25;
        assert!((aligned.roll_pitch_yaw.z() - expected_yaw).abs() < 1e-6);
        assert!(imu_at(&history, Time::from_nanos(999_000_000)).is_none());
        assert!(imu_at(&history, Time::from_nanos(1_101_000_000)).is_none());
        assert_eq!(
            imu_at(&history, Time::from_nanos(1_100_000_000))
                .unwrap()
                .roll_pitch_yaw
                .z(),
            -3.0
        );
    }
}
