use std::time::Duration;

use coordinate_systems::Ground;
use filtering::kalman_filter::KalmanFilter;
use linear_algebra::Isometry2;
use nalgebra::Matrix2;
use types::multivariate_normal_distribution::MultivariateNormalDistribution;

pub(super) trait RestingPredict {
    fn predict(
        &mut self,
        delta_time: Duration,
        last_to_current_odometry: Isometry2<Ground, Ground>,
        process_noise_per_second: Matrix2<f32>,
    );
}

pub(super) trait RestingUpdate {
    fn update(&mut self, measurement: MultivariateNormalDistribution<2>);
}

impl RestingPredict for MultivariateNormalDistribution<2> {
    fn predict(
        &mut self,
        delta_time: Duration,
        last_to_current_odometry: Isometry2<Ground, Ground>,
        process_noise_per_second: Matrix2<f32>,
    ) {
        let process_noise = process_noise_per_second * delta_time.as_secs_f32();
        let rotation = last_to_current_odometry.inner.rotation.to_rotation_matrix();
        let translation = last_to_current_odometry.inner.translation.vector;

        KalmanFilter::predict(
            self,
            *rotation.matrix(),
            Matrix2::identity(),
            translation,
            process_noise,
        );
    }
}

impl RestingUpdate for MultivariateNormalDistribution<2> {
    fn update(&mut self, measurement: MultivariateNormalDistribution<2>) {
        KalmanFilter::update(
            self,
            Matrix2::identity(),
            measurement.mean,
            measurement.covariance,
        )
    }
}
