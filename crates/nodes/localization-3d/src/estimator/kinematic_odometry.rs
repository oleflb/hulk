use color_eyre::Result;
use localization_fagra::factors::{AdjacentKinematicOdometry, KinematicOdometry};
use nalgebra::Matrix2;
use ros_z::time::Time;
use types::odometry::KinematicOdometryDelta;

use super::{Localization, recovery::MotionRecord};

#[derive(Clone, Copy)]
pub(super) enum KinematicFactor {
    Same(fagra::FactorKey<KinematicOdometry>),
    Adjacent(fagra::FactorKey<AdjacentKinematicOdometry>),
}

impl KinematicFactor {
    pub(super) fn remove(&self, graph: &mut super::Graph) -> Result<()> {
        match self {
            Self::Same(key) => graph.remove_factor(*key)?,
            Self::Adjacent(key) => graph.remove_factor(*key)?,
        }
        Ok(())
    }
}

impl Localization {
    pub fn ingest_kinematic_odometry(&mut self, sample: KinematicOdometryDelta) -> Result<bool> {
        let KinematicOdometryDelta {
            previous_time,
            time,
            current_to_previous,
        } = sample;
        let Some(noise) = self.parameters.kinematic_odometry_noise.as_ref() else {
            return Ok(false);
        };
        if time <= previous_time
            || self
                .history
                .latest_kinematic_time
                .is_some_and(|last| previous_time < last)
            || !current_to_previous
                .inner
                .to_homogeneous()
                .iter()
                .all(|v| v.is_finite())
            || previous_time.as_nanos()
                < self
                    .latest_time
                    .max(time)
                    .as_nanos()
                    .saturating_sub(self.parameters.timing.window_ns())
        {
            return Ok(false);
        }
        let Some((a, _)) = self.check_time(previous_time, "kinematic odometry")? else {
            return Ok(false);
        };
        let Some((b, _)) = self.check_time(time, "kinematic odometry")? else {
            return Ok(false);
        };
        if b - a > 1 {
            return Ok(false);
        }
        let dt = time.duration_since(previous_time).as_secs_f64();
        // Keep the displacement floor: duration normalization regressed recorded motion solves.
        let variance =
            noise.position_sigma.map(|s| s * s) + noise.translation_variance_per_second * dt;
        let information_root = Matrix2::from_diagonal(&variance.map(|v| v.sqrt().recip()));
        let translation =
            linear_algebra::Vector2::wrap(current_to_previous.translation().coords().inner.cast());
        self.accept_motion(MotionRecord::Kinematic {
            previous_time,
            time,
            translation,
            information_root,
        })?;
        self.history.latest_kinematic_time = Some(time);
        Ok(true)
    }

    pub(super) fn insert_kinematic_odometry(
        &mut self,
        previous_time: Time,
        time: Time,
        translation: linear_algebra::Vector2<coordinate_systems::Ground, f64>,
        information_root: Matrix2<f64>,
    ) -> Result<()> {
        let (a, previous_tau) = self.segment_and_tau(previous_time)?;
        let (b, current_tau) = self.segment_and_tau(time)?;
        let first = self.ensure_segment(a)?;
        let second = self.ensure_segment(b)?;
        let duration = self.parameters.timing.trajectory_spacing.as_secs_f64();
        let huber_threshold = self.parameters.model.huber_threshold;
        let factor = if a == b {
            KinematicFactor::Same(self.graph.add_factor(KinematicOdometry {
                controls: first,
                duration,
                previous_tau,
                current_tau,
                translation,
                information_root,
                huber_threshold,
            })?)
        } else {
            KinematicFactor::Adjacent(self.graph.add_factor(AdjacentKinematicOdometry {
                controls: [first[0], first[1], first[2], first[3], second[3]],
                duration,
                previous_tau,
                current_tau,
                translation,
                information_root,
                huber_threshold,
            })?)
        };
        self.kinematic_factors.insert(previous_time, (time, factor));
        self.segment_factors.entry(a).or_default().measurements += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use linear_algebra::{Isometry2, Isometry3};
    use nalgebra::SMatrix;
    use projection::intrinsic::Intrinsic;
    use types::field_dimensions::FieldDimensions;

    use crate::parameters::Localization3dParameters;

    use super::*;

    #[test]
    fn shipped_short_tails_preserve_motion_without_overconfidence() -> Result<()> {
        let parameters: Localization3dParameters = json5::from_str(include_str!(
            "../../../../../etc/parameters/base/localization3d.json5"
        ))?;
        for (speed, expected) in [
            (0.1, 1.0 / 3.0),
            (1.0, 4.0 * (0.05 / 0.000075_f64.sqrt() - 1.0)),
        ] {
            let mut reference: Option<(nalgebra::Vector3<f64>, nalgebra::Matrix3<f64>)> = None;
            for (index, batches) in [vec![50; 2], vec![48, 2, 50], vec![100], vec![2; 50]]
                .into_iter()
                .enumerate()
            {
                let mut estimator = Localization::new_motion(
                    Time::from_nanos(1_000_000_000),
                    0,
                    Isometry3::identity(),
                    &Intrinsic::default(),
                    parameters.clone(),
                    &FieldDimensions::SPL_2025,
                )?;
                // Exactly 500 ms leaves this synthetic graph rank-deficient; use off-knot IMU support.
                for milliseconds in (0..=510).step_by(2) {
                    estimator.ingest_imu(
                        estimator.origin + Duration::from_millis(milliseconds),
                        booster::ImuState {
                            linear_acceleration: linear_algebra::vector![0.0, 0.0, 9.81],
                            ..Default::default()
                        },
                    )?;
                }
                estimator.prepare_preintegration()?;
                estimator.prepare_attitude()?;
                let mut time = estimator.origin;
                // Stops keep the nominal large-displacement case in Huber's tail.
                for milliseconds in [50; 4].into_iter().chain(batches).chain([50; 4]) {
                    let speed = if (estimator.origin + Duration::from_millis(200)
                        ..estimator.origin + Duration::from_millis(300))
                        .contains(&time)
                    {
                        speed
                    } else {
                        0.0
                    };
                    let sample = KinematicOdometryDelta {
                        previous_time: time,
                        time: time + Duration::from_millis(milliseconds),
                        current_to_previous: Isometry2::wrap(nalgebra::Isometry2::translation(
                            milliseconds as f32 * speed * 0.001,
                            0.0,
                        )),
                    };
                    assert!(estimator.ingest_kinematic_odometry(sample)?);
                    assert!(!estimator.ingest_kinematic_odometry(sample)?);
                    time = sample.time;
                }
                let options = estimator.solve_options();
                let controls = estimator.ensure_segment(2)?;
                let blocks = estimator.estimate_covariance_blocks(controls)?;
                let (report, covariance) = estimator.optimizer.solve_batch_with_covariance(
                    &mut estimator.graph,
                    &options,
                    &blocks,
                    &Default::default(),
                )?;
                let covariance = SMatrix::<f64, 24, 24>::from_fn(|r, c| covariance[(r, c)]);
                let spline = estimator.spline(controls)?;
                let linearized = spline.linearize()?;
                let (start, end) = (linearized.pose(0.0)?, linearized.pose(1.0)?);
                let displacement =
                    end.pose.inner.translation.vector - start.pose.inner.translation.vector;
                let mut jacobian = SMatrix::<f64, 3, 24>::zeros();
                for index in 0..4 {
                    let a = start.pose.inner.rotation.to_rotation_matrix()
                        * start.jacobians[index].fixed_rows::<3>(3);
                    let b = end.pose.inner.rotation.to_rotation_matrix()
                        * end.jacobians[index].fixed_rows::<3>(3);
                    jacobian
                        .fixed_view_mut::<3, 6>(0, index * 6)
                        .copy_from(&(b - a));
                }
                let covariance = jacobian * covariance * jacobian.transpose();
                assert!(covariance.cholesky().is_some());
                assert!(displacement.x > 0.0 && displacement.x < speed as f64 * 0.1);
                if let Some((motion, confidence)) = &reference {
                    if index == 1 {
                        assert!(
                            (displacement.xy() - motion.xy()).norm() < 0.03 * motion.xy().norm()
                        );
                        let covariance = covariance.fixed_view::<2, 2>(0, 0);
                        let confidence = confidence.fixed_view::<2, 2>(0, 0);
                        assert!((covariance - confidence).norm() < 0.03 * confidence.norm());
                    } else if index == 3 {
                        // The retained per-delta floor does not claim sampling invariance.
                        assert!(covariance[(0, 0)] > confidence[(0, 0)]);
                        assert!(displacement.x < motion.x);
                    }
                } else {
                    assert!((report.initial_cost - expected).abs() < 1e-5, "{report:?}");
                    let residual =
                        (speed as f64 * 0.1 - displacement.x) / (2.0 * 0.000075_f64.sqrt());
                    assert_eq!(residual > parameters.model.huber_threshold, speed == 1.0);
                    reference = Some((displacement, covariance));
                }
            }
        }
        Ok(())
    }
}
