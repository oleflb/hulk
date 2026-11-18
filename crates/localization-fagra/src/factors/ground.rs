use coordinate_systems::Robot;
use fagra::{
    BlockId, EvaluationError, FactorBatch, FactorSelection, LinearizationSink, StateKey, StateStore,
};
use linear_algebra::Point3;
use nalgebra::{RealField, SMatrix};

use super::common;
use crate::variables::PoseControl;

/// Two sole origins transformed into Robot at one observation time.
#[derive(Clone, Debug)]
pub struct FootObservation<R: RealField + Copy = f64> {
    pub tau: R,
    pub vertices: [Point3<Robot, R>; 2],
}

/// Penalize sole origins below local z = 0 without constraining airborne feet.
#[derive(Clone, Debug)]
pub struct FootGround<R: RealField + Copy = f64> {
    pub controls: [StateKey<PoseControl<R>>; 4],
    pub duration: R,
    /// Positive height standard deviation in metres.
    pub sigma: R,
}

impl<R: RealField + Copy, S: StateStore<PoseControl<R>>> FactorBatch<S> for FootGround<R> {
    type Scalar = R;
    type Factor = FootObservation<R>;

    fn visit_variables(&self, _factor: &Self::Factor, mut visitor: impl FnMut(BlockId)) {
        for key in self.controls {
            visitor(key.block_id());
        }
    }

    fn cost(
        &self,
        states: &S,
        factors: FactorSelection<'_, Self::Factor>,
    ) -> Result<R, EvaluationError> {
        if factors.is_empty() {
            return Ok(R::zero());
        }
        let inverse_sigma = common::positive(self.sigma)?;
        let spline = common::spline(states, &self.controls, self.duration)?;
        let mut cost = R::zero();
        for (_, observation) in factors {
            let pose = spline.pose(observation.tau)?;
            let (residual, _) = foot_residual(&pose.inner, observation, inverse_sigma)?;
            cost += common::cost(&residual)?;
        }
        common::checked_cost(cost)
    }

    fn linearize<L: LinearizationSink<Scalar = R>>(
        &self,
        states: &S,
        factors: FactorSelection<'_, Self::Factor>,
        sink: &mut L,
    ) -> Result<(), EvaluationError> {
        if factors.is_empty() {
            return Ok(());
        }
        let inverse_sigma = common::positive(self.sigma)?;
        let spline = common::spline(states, &self.controls, self.duration)?;
        let mut linearized = None;
        for (id, observation) in factors {
            let value = spline.pose(observation.tau)?;
            let (residual, vertices) = foot_residual(&value.inner, observation, inverse_sigma)?;
            if vertices.iter().all(Option::is_none) {
                sink.factor(id, |_| Ok(()))?;
                continue;
            }
            let prepared = match linearized {
                Some(ref prepared) => prepared,
                None => linearized.insert(spline.linearize()?),
            };
            let pose = prepared.pose(observation.tau)?;
            let rotation = pose.pose.inner.rotation.to_rotation_matrix().into_inner();
            let mut h = SMatrix::<R, 2, 6>::zeros();
            for (row, vertex) in vertices.iter().enumerate() {
                if let Some(vertex) = vertex {
                    h.row_mut(row).copy_from(
                        &(common::point_jacobian(rotation, vertex.inner.coords).row(2)
                            * -inverse_sigma),
                    );
                }
            }
            let jacobians = pose.jacobians.map(|j| h * j);
            sink.factor(id, |sink| {
                common::emit(sink, &self.controls, &residual, &jacobians)
            })?;
        }
        Ok(())
    }
}

type FootResidual<R> = (nalgebra::Vector2<R>, [Option<Point3<Robot, R>>; 2]);

fn foot_residual<R: RealField + Copy>(
    pose: &nalgebra::Isometry3<R>,
    observation: &FootObservation<R>,
    inverse_sigma: R,
) -> Result<FootResidual<R>, EvaluationError> {
    let mut residual = nalgebra::Vector2::zeros();
    let mut active = [None; 2];
    for (index, sole) in observation.vertices.iter().enumerate() {
        common::finite(sole.inner.coords.iter())?;
        let height = (pose * sole.inner).z;
        if !height.is_finite() {
            return Err(EvaluationError::InvalidEvaluation);
        }
        if height < R::zero() {
            residual[index] = -height * inverse_sigma;
            active[index] = Some(*sole);
        }
    }
    Ok((residual, active))
}
