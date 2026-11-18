use super::{Localization, recovery::MotionRecord};
use booster::ImuState;
use color_eyre::Result;
use coordinate_systems::{ImuReference, Robot};
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::{Orientation3, Point3, Vector3};
use localization_fagra::factors::{FootGround, FootObservation};
use ros_z::time::Time;
use types::time_wrapper::TimeWrapper;

impl Localization {
    pub fn ingest_imu(&mut self, time: Time, imu: ImuState) -> Result<bool> {
        if !imu
            .roll_pitch_yaw
            .inner
            .iter()
            .chain(imu.angular_velocity.inner.iter())
            .all(|v| v.is_finite())
        {
            tracing::warn!(?time, "discarding nonfinite IMU measurement");
            return Ok(false);
        }
        if self.check_time(time, "IMU")?.is_none() {
            return Ok(false);
        }
        let rpy = imu.roll_pitch_yaw.inner.cast::<f64>();
        let attitude = Orientation3::from_euler_angles(rpy.x, rpy.y, rpy.z);
        let angular_velocity = Vector3::wrap(imu.angular_velocity.inner.cast());
        self.accept_motion(MotionRecord::Imu {
            time,
            angular_velocity,
            attitude,
            force: Vector3::wrap(imu.linear_acceleration.inner.cast()),
        })?;
        Ok(true)
    }

    pub(super) fn insert_imu(
        &mut self,
        time: Time,
        angular_velocity: Vector3<Robot, f64>,
        orientation: Orientation3<ImuReference, f64>,
        force: Vector3<Robot, f64>,
    ) -> Result<()> {
        let (segment, _) = self.segment_and_tau(time)?;
        self.ensure_segment(segment)?;
        self.ensure_biases(time)?;
        self.preintegration
            .insert(self.origin, time, angular_velocity, force);
        self.history.attitudes.insert(time, orientation);
        self.segment_factors
            .entry(segment)
            .or_default()
            .measurements += 1;
        Ok(())
    }

    pub fn ingest_kinematics(&mut self, sample: TimeWrapper<RobotKinematics>) -> Result<bool> {
        // Sample the penetration guard at odometry cadence, before allocating/history storage.
        // It remains an inequality: airborne soles are not pulled onto the ground.
        if self
            .history
            .records
            .iter()
            .rev()
            .find_map(|record| match record {
                MotionRecord::Feet { time, .. } => Some(*time),
                _ => None,
            })
            .is_some_and(|last| {
                sample.time < last.saturating_add(std::time::Duration::from_millis(50))
            })
        {
            return Ok(false);
        }
        let vertices: [Point3<Robot, f64>; 2] = [
            Point3::wrap(
                sample
                    .inner
                    .left_leg
                    .sole_to_robot
                    .inner
                    .translation
                    .vector
                    .cast()
                    .into(),
            ),
            Point3::wrap(
                sample
                    .inner
                    .right_leg
                    .sole_to_robot
                    .inner
                    .translation
                    .vector
                    .cast()
                    .into(),
            ),
        ];
        if !vertices
            .iter()
            .all(|point| point.inner.iter().all(|v| v.is_finite()))
        {
            tracing::warn!(time = ?sample.time, "discarding nonfinite foot measurement");
            return Ok(false);
        }
        if self.check_time(sample.time, "kinematics")?.is_none() {
            return Ok(false);
        }
        self.accept_motion(MotionRecord::Feet {
            time: sample.time,
            vertices,
        })
    }

    pub(super) fn insert_feet(
        &mut self,
        time: Time,
        vertices: [Point3<Robot, f64>; 2],
    ) -> Result<()> {
        let (segment, tau) = self.segment_and_tau(time)?;
        let controls = self.ensure_segment(segment)?;
        let batch = *self
            .segment_factors
            .entry(segment)
            .or_default()
            .feet
            .get_or_insert_with(|| {
                self.graph.add_batch(FootGround {
                    controls,
                    duration: self.parameters.timing.trajectory_spacing.as_secs_f64(),
                    sigma: self.parameters.model.foot_sigma,
                })
            });
        self.graph
            .add_factor_to(batch, FootObservation { tau, vertices })?;
        self.segment_factors
            .entry(segment)
            .or_default()
            .measurements += 1;
        Ok(())
    }
}
