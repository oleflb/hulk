use factrs::{
    linalg::{ForwardProp, Numeric, VectorX},
    traits::Residual,
    variables::SE23,
};

#[derive(Debug, Clone, Copy)]
pub struct PositiveZFactor {
    minimum_z: f64,
    softness: f64,
    sigma: f64,
}

impl PositiveZFactor {
    pub fn new(minimum_z: f64, softness: f64, sigma: f64) -> Self {
        assert!(softness > 0.0, "positive z softness must be positive");
        assert!(sigma > 0.0, "positive z sigma must be positive");

        Self {
            minimum_z,
            softness,
            sigma,
        }
    }

    fn residual_impl<T: Numeric>(&self, pose: &SE23<T>) -> VectorX<T> {
        let z_error = T::from(self.minimum_z) - pose.xyz().z;
        let residual = smooth_hinge(z_error, T::from(self.softness)) / T::from(self.sigma);

        VectorX::from_element(1, residual)
    }
}

#[factrs::mark]
impl Residual for PositiveZFactor {
    type Input = SE23;
    type Differ = ForwardProp;

    fn dim_out(&self) -> usize {
        1
    }

    fn residual<T: Numeric>(&self, pose: SE23<T>) -> VectorX<T> {
        self.residual_impl(&pose)
    }
}

fn smooth_hinge<T: Numeric>(x: T, softness: T) -> T {
    T::from(0.5) * (x + (x * x + softness * softness).sqrt())
}

#[cfg(test)]
mod tests {
    use factrs::{
        core::SO3,
        traits::{Residual, Variable},
        variables::SE23,
    };
    use nalgebra::{Vector3, vector};

    use super::*;

    fn pose_at_z(z: f64) -> SE23 {
        SE23::from_rot_vel_trans(SO3::identity(), Vector3::zeros(), vector![0.0, 0.0, z])
    }

    #[test]
    fn residual_is_near_zero_for_positive_z() {
        let factor = PositiveZFactor::new(1.0e-3, 1.0e-6, 0.01);

        let residual = factor.residual(pose_at_z(0.5));

        assert!(residual[0] < 1.0e-9, "residual was {residual:?}");
    }

    #[test]
    fn residual_penalizes_negative_z() {
        let factor = PositiveZFactor::new(1.0e-3, 1.0e-6, 0.01);

        let residual = factor.residual(pose_at_z(-0.1));

        assert!(residual[0] > 10.0, "residual was {residual:?}");
    }
}
