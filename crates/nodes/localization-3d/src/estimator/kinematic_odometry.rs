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
