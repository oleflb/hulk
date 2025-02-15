use nalgebra::{Dim, Storage, Vector};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct LinearRegressor<D: Dim, S: Storage<f32, D>> {
    coefficients: Vector<f32, D, S>,
    intercept: f32,
}

impl<D: Dim, S: Storage<f32, D>> LinearRegressor<D, S> {
    pub fn new(coefficients: Vector<f32, D, S>, intercept: f32) -> Self {
        Self {
            coefficients,
            intercept,
        }
    }

    pub fn predict(&self, features: &Vector<f32, D, S>) -> f32 {
        self.coefficients.dot(features) + self.intercept
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;

    #[test]
    fn it_works() {
        let coefficients = Vector2::new(2.0, 3.0);
        let intercept = 4.0;
        let regressor = LinearRegressor::new(coefficients, intercept);
        let features = Vector2::new(5.0, 6.0);
        let prediction = regressor.predict(&features);
        assert_eq!(prediction, 2.0 * 5.0 + 3.0 * 6.0 + 4.0);
    }
}
