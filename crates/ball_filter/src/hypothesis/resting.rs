use coordinate_systems::Ground;
use filtering::kalman_filter::KalmanFilter;
use linear_algebra::{point, Isometry2, Point2};
use nalgebra::Matrix2;
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};
use types::multivariate_normal_distribution::MultivariateNormalDistribution;

#[derive(
    Clone, Copy, Debug, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct RestingHypothesis(pub MultivariateNormalDistribution<2>);

impl RestingHypothesis {
    pub fn position(&self) -> Point2<Ground> {
        point![self.0.mean.x, self.0.mean.y]
    }

    pub fn reset(&mut self, position: Point2<Ground>) {
        self.0.mean.x = position.x();
        self.0.mean.y = position.y();
    }

    pub fn predict(
        &mut self,
        last_to_current_odometry: Isometry2<Ground, Ground>,
        process_noise: Matrix2<f32>,
    ) {
        let rotation = last_to_current_odometry.inner.rotation.to_rotation_matrix();
        let translation = last_to_current_odometry.inner.translation.vector;

        self.0.predict(
            *rotation.matrix(),
            Matrix2::identity(),
            translation,
            process_noise,
        );
    }

    pub fn update(&mut self, measurement: Point2<Ground>, noise: Matrix2<f32>) {
        self.0
            .update(Matrix2::identity(), measurement.inner.coords, noise)
    }

    pub fn merge(&mut self, other: Self) {
        self.0
            .update(Matrix2::identity(), other.0.mean, other.0.covariance)
    }
}
