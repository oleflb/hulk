use std::time::Duration;

use coordinate_systems::Ground;
use filtering::kalman_filter::KalmanFilter;
use linear_algebra::{point, vector, Isometry2, Point2, Vector2};
use nalgebra::{matrix, Matrix2, Matrix2x4, Matrix4, Matrix4x2};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};
use types::multivariate_normal_distribution::MultivariateNormalDistribution;

#[derive(
    Clone, Copy, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct MovingHypothesis(pub MultivariateNormalDistribution<4>);

impl MovingHypothesis {
    pub fn position(&self) -> Point2<Ground> {
        point![self.0.mean.x, self.0.mean.y]
    }

    pub fn velocity(&self) -> Vector2<Ground> {
        vector![self.0.mean.z, self.0.mean.w]
    }

    pub fn predict(
        &mut self,
        delta_time: Duration,
        last_to_current_odometry: Isometry2<Ground, Ground>,
        velocity_decay: f32,
        process_noise: Matrix4<f32>,
    ) {
        let dt = delta_time.as_secs_f32();
        let constant_velocity_prediction = matrix![
            1.0, 0.0, dt, 0.0;
            0.0, 1.0, 0.0, dt;
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
        self.0.predict(
            state_prediction,
            Matrix4x2::identity(),
            translation,
            process_noise,
        );
    }

    pub fn update(&mut self, measurement: Point2<Ground>, noise: Matrix2<f32>) {
        self.0
            .update(Matrix2x4::identity(), measurement.inner.coords, noise)
    }

    pub fn merge(&mut self, other: Self) {
        self.0
            .update(Matrix4::identity(), other.0.mean, other.0.covariance)
    }
}
