use booster::ImuState;
use color_eyre::Result;
use kinematics::robot_kinematics::RobotKinematics;
use odometry::{EstimatorInput, OdometryEstimator, Parameters, align_input};
use ros_z::{cache::CacheInner, time::Time};
use types::{
    fall_detection::FallDetection, odometry::KinematicOdometryDelta, time_wrapper::TimeWrapper,
};

/// Reconstruct the live odometry inputs in recorded delivery order. LowState carries
/// the IMU sample, but recorder delivery cannot reproduce live subscriber scheduling.
pub struct OdometryReplay {
    estimator: OdometryEstimator,
    parameters: Parameters,
    imu: CacheInner<TimeWrapper<ImuState>>,
    pub feet: CacheInner<TimeWrapper<RobotKinematics>>,
    pub falls: CacheInner<FallDetection>,
    pub missing_aligned_inputs: usize,
}

impl OdometryReplay {
    pub fn new() -> Result<Self> {
        Ok(Self {
            estimator: OdometryEstimator::default(),
            parameters: json5::from_str(include_str!(
                "../../../../../etc/parameters/base/odometry.json5"
            ))?,
            imu: CacheInner::new(128),
            feet: CacheInner::new(10),
            falls: CacheInner::new(10),
            missing_aligned_inputs: 0,
        })
    }

    pub fn update(
        &mut self,
        time: Time,
        now: Time,
        imu: ImuState,
    ) -> Option<KinematicOdometryDelta> {
        self.imu.insert(time, TimeWrapper { time, inner: imu });
        let (imu, feet) = align_input(time, imu, &self.imu, self.feet.get_before(time));
        let fall = self
            .falls
            .get_nearest(time)
            .filter(|fall| fall.is_valid(now));
        if feet.is_none() || fall.is_none() {
            self.missing_aligned_inputs += 1;
        }
        self.estimator.update(
            EstimatorInput {
                time,
                imu_state: &imu,
                robot_kinematics: feet.as_deref(),
                fall_detection: fall.as_deref(),
            },
            &self.parameters,
        );
        self.estimator.take_delta()
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use types::fall_detection::Posture;

    #[test]
    fn reconstruction_emits_measured_stops_but_not_missing_or_fallen_intervals() -> Result<()> {
        let mut replay = OdometryReplay::new()?;
        let mut deltas = Vec::new();
        for milliseconds in [1000, 1025, 1050, 1075, 1100, 1125, 1150] {
            let time = Time::from_nanos(milliseconds * 1_000_000);
            replay.feet.insert(
                time,
                TimeWrapper {
                    time,
                    inner: RobotKinematics::default(),
                },
            );
            if milliseconds >= 1025 {
                replay.falls.insert(
                    time,
                    FallDetection {
                        time,
                        valid_until: time.saturating_add(Duration::from_millis(25)),
                        posture: if milliseconds < 1100 {
                            Posture::Upright
                        } else {
                            Posture::Falling
                        },
                    },
                );
            }
            if let Some(delta) = replay.update(time, time, ImuState::default()) {
                deltas.push(delta);
            }
        }
        assert_eq!(replay.missing_aligned_inputs, 1);
        assert_eq!(deltas.len(), 1);
        assert_eq!(deltas[0].previous_time, Time::from_nanos(1_025_000_000));
        assert_eq!(deltas[0].time, Time::from_nanos(1_075_000_000));
        assert_eq!(
            deltas[0].current_to_previous,
            linear_algebra::Isometry2::identity()
        );
        Ok(())
    }

    #[test]
    fn reconstruction_requires_bracketed_finite_attitude() -> Result<()> {
        let mut history = CacheInner::new(2);
        for (nanos, yaw) in [(1_000_000_000, 3.0), (1_100_000_000, -3.0)] {
            let time = Time::from_nanos(nanos);
            let mut imu = ImuState::default();
            imu.roll_pitch_yaw.inner.z = yaw;
            history.insert(time, TimeWrapper { time, inner: imu });
        }
        let aligned = |time| {
            let time = Time::from_nanos(time);
            let (imu, feet) = align_input(
                time,
                ImuState::default(),
                &history,
                Some(std::sync::Arc::new(TimeWrapper {
                    time,
                    inner: RobotKinematics::default(),
                })),
            );
            feet.map(|_| imu)
        };
        let imu =
            aligned(1_050_000_000).ok_or_else(|| color_eyre::eyre::eyre!("missing midpoint"))?;
        assert!((imu.roll_pitch_yaw.z().abs() - std::f32::consts::PI).abs() < 1e-5);
        assert!(aligned(999_999_999).is_none());
        assert!(aligned(1_100_000_001).is_none());
        Ok(())
    }
}
