use super::{Estimator, control_keys};
use color_eyre::Result;
use coordinate_systems::ImuReference;
use linear_algebra::Orientation3;
use localization_fagra::factors::RelativeYaw;
use ros_z::time::Time;

impl Estimator {
    /// Boundary attitudes summarize the source-time IMU stream. Relative headings
    /// carry no absolute-yaw anchor. Rebuilding the small active set handles late samples.
    pub(super) fn prepare_attitude(&mut self) -> Result<Option<fagra::FactorKey<RelativeYaw>>> {
        for segment in self.segments() {
            let start = Time::from_nanos(
                self.origin.as_nanos() + segment * self.parameters.timing.knot_ns(),
            );
            let end = Time::from_nanos(start.as_nanos() + self.parameters.timing.knot_ns());
            let Some(a) = self.attitude_at(start) else {
                continue;
            };
            let Some(b) = self.attitude_at(end) else {
                continue;
            };
            if let Some(key) = self.segment_factors.entry(segment).or_default().yaw.take() {
                self.graph.remove_factor(key)?;
            }
            let key = self.graph.add_factor(RelativeYaw {
                controls: control_keys(&self.controls, segment)?,
                duration: self.parameters.timing.trajectory_spacing.as_secs_f64(),
                end_tau: 1.0,
                measured_yaw_change: yaw_change(a, b),
                information_root: self.parameters.model.relative_yaw_variance.sqrt().recip(),
            })?;
            self.segment_factors.entry(segment).or_default().yaw = Some(key);
        }
        if let Some((&time, &orientation)) = self.history.attitudes.last_key_value() {
            let (segment, tau) = self.segment_and_tau(time)?;
            let start = Time::from_nanos(
                self.origin.as_nanos() + segment * self.parameters.timing.knot_ns(),
            );
            if tau > 0.0
                && let Some(anchor) = self.attitude_at(start)
            {
                return Ok(Some(self.graph.add_factor(RelativeYaw {
                    controls: control_keys(&self.controls, segment)?,
                    duration: self.parameters.timing.trajectory_spacing.as_secs_f64(),
                    end_tau: tau,
                    measured_yaw_change: yaw_change(anchor, orientation),
                    information_root: self.parameters.model.relative_yaw_variance.sqrt().recip(),
                })?));
            }
        }
        Ok(None)
    }

    pub(crate) fn attitude_at(&self, time: Time) -> Option<Orientation3<ImuReference, f64>> {
        self.history
            .attitude_at(time, self.parameters.timing.max_imu_gap)
    }
}

fn yaw_change(a: Orientation3<ImuReference, f64>, b: Orientation3<ImuReference, f64>) -> f64 {
    let a = a.inner * nalgebra::Vector3::x();
    let b = b.inner * nalgebra::Vector3::x();
    let difference = b.y.atan2(b.x) - a.y.atan2(a.x);
    difference.sin().atan2(difference.cos())
}
