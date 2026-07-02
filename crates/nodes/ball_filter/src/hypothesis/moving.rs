use std::time::Duration;

use coordinate_systems::Ground;
use filtering::kalman_filter::KalmanFilter;
use linear_algebra::Isometry2;
use nalgebra::{Matrix2x4, Matrix4, Matrix4x2, matrix};
use types::multivariate_normal_distribution::MultivariateNormalDistribution;

pub(super) trait MovingPredict {
    fn predict(
        &mut self,
        delta_time: Duration,
        last_to_current_odometry: Isometry2<Ground, Ground>,
        velocity_decay_per_second: f32,
        process_noise_per_second: Matrix4<f32>,
    );
}

pub(super) trait MovingUpdate {
    fn update(&mut self, measurement: MultivariateNormalDistribution<2>);
}

impl MovingPredict for MultivariateNormalDistribution<4> {
    fn predict(
        &mut self,
        delta_time: Duration,
        last_to_current_odometry: Isometry2<Ground, Ground>,
        velocity_decay_per_second: f32,
        process_noise_per_second: Matrix4<f32>,
    ) {
        let dt = delta_time.as_secs_f32();
        let velocity_decay = velocity_decay_per_second.powf(dt);
        let position_velocity_factor = if velocity_decay_per_second > 0.0
            && (velocity_decay_per_second - 1.0).abs() > f32::EPSILON
        {
            (velocity_decay - 1.0) / velocity_decay_per_second.ln()
        } else {
            dt
        };
        let process_noise = process_noise_per_second * dt;
        let constant_velocity_prediction = matrix![
            1.0, 0.0, position_velocity_factor, 0.0;
            0.0, 1.0, 0.0, position_velocity_factor;
            0.0, 0.0, velocity_decay, 0.0;
            0.0, 0.0, 0.0, velocity_decay;
        ];

        let rotation = last_to_current_odometry.inner.rotation.to_rotation_matrix();
        let rotation = rotation.matrix();
        let translation = last_to_current_odometry.inner.translation.vector;

        let state_rotation = matrix![
            rotation.m11, rotation.m12, 0.0, 0.0;
            rotation.m21, rotation.m22, 0.0, 0.0;
            0.0, 0.0, rotation.m11, rotation.m12;
            0.0, 0.0, rotation.m21, rotation.m22;
        ];

        let state_prediction = constant_velocity_prediction * state_rotation;
        KalmanFilter::predict(
            self,
            state_prediction,
            Matrix4x2::identity(),
            translation,
            process_noise,
        );
    }
}

impl MovingUpdate for MultivariateNormalDistribution<4> {
    fn update(&mut self, measurement: MultivariateNormalDistribution<2>) {
        KalmanFilter::update(
            self,
            Matrix2x4::identity(),
            measurement.mean,
            measurement.covariance,
        )
    }
}
