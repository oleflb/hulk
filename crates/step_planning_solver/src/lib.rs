use color_eyre::Result;
use geometry::{angle::Angle, arc::Arc, circle::Circle, line_segment::LineSegment};
use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};
use linear_algebra::point;
use nalgebra::{dvector, vector, DVector, Dyn, Matrix, Owned, U1};
use num_dual::{Derivative, DualNum, DualNumFloat, DualVec};

use step_planning::{
    geometry::Pose,
    loss_fields::step_size::{WalkVolumeCoefficients, WalkVolumeExtents},
    step_plan::{StepPlan, StepPlanning},
    traits::{LossField, ScaledGradient, UnwrapDual, WrapDual},
};
use types::{
    planned_path::{Path, PathSegment},
    support_foot::Side,
};

fn duals<F: DualNumFloat + DualNum<F>>(reals: &DVector<F>) -> DVector<DualVec<F, F, Dyn>> {
    let num_variables = reals.nrows();

    reals.map_with_location(|row, _, real| {
        DualVec::new(
            real,
            Derivative::some(DVector::from_fn(num_variables, |i, _| {
                if i == row {
                    F::one()
                } else {
                    F::zero()
                }
            })),
        )
    })
}

#[derive(Clone, Debug)]
struct StepPlanningProblem {
    step_planning: StepPlanning,
    variables: DVector<f32>,
}

impl LeastSquaresProblem<f32, U1, Dyn> for StepPlanningProblem {
    type ResidualStorage = Owned<f32, U1, U1>;
    type JacobianStorage = Owned<f32, U1, Dyn>;
    type ParameterStorage = Owned<f32, Dyn, U1>;

    fn set_params(&mut self, x: &nalgebra::Vector<f32, Dyn, Self::ParameterStorage>) {
        self.variables = x.clone();
    }

    fn params(&self) -> nalgebra::Vector<f32, Dyn, Self::ParameterStorage> {
        self.variables.clone()
    }

    fn residuals(&self) -> Option<nalgebra::Vector<f32, U1, Self::ResidualStorage>> {
        let step_planning_loss = self.step_planning.loss_field();

        let step_plan = StepPlan::from(self.variables.as_slice());

        let loss = self
            .step_planning
            .planned_steps(
                self.step_planning
                    .initial_pose
                    .clone()
                    .with_support_foot(self.step_planning.initial_support_foot),
                &step_plan,
            )
            .map(|planned_step| step_planning_loss.loss(planned_step))
            .sum();

        // eprintln!("loss: {loss}\n\t({:.4?})", self.variables.as_slice());

        Some(vector![loss])
    }

    fn jacobian(&self) -> Option<nalgebra::Matrix<f32, U1, Dyn, Self::JacobianStorage>> {
        let num_variables = self.variables.nrows();
        let dual_param = duals(&self.variables);

        let step_planning_loss = self.step_planning.loss_field();

        let step_plan = StepPlan::from(dual_param.as_slice());

        let gradient: DVector<f32> = self
            .step_planning
            .planned_steps(
                self.step_planning
                    .initial_pose
                    .clone()
                    .with_support_foot(self.step_planning.initial_support_foot)
                    .wrap_dual(),
                &step_plan,
            )
            .map(|dual_planned_step| {
                let (planned_step, planned_step_gradients) = dual_planned_step.unwrap_dual();

                let derivatives = step_planning_loss.grad(planned_step);

                planned_step_gradients
                    .scaled_gradient(derivatives)
                    .unwrap_generic(Dyn(num_variables), U1)
            })
            .sum();

        let step_planning_loss = self.step_planning.loss_field();

        let step_plan = StepPlan::from(self.variables.as_slice());

        let loss: f32 = self
            .step_planning
            .planned_steps(
                self.step_planning
                    .initial_pose
                    .clone()
                    .with_support_foot(self.step_planning.initial_support_foot),
                &step_plan,
            )
            .map(|planned_step| step_planning_loss.loss(planned_step))
            .sum();

        // eprintln!(
        //     "grad: {loss}\n\t({:.4?})\n\t[{:.4?}]",
        //     self.variables.as_slice(),
        //     &gradient.as_slice()
        // );

        Some(gradient.transpose())
    }
}

pub fn plan_steps(
    path: Path,
    initial_pose: Pose<f32>,
    initial_support_foot: Side,
    initial_parameter_guess: DVector<f32>,
) -> Result<DVector<f32>> {
    let mut problem = StepPlanningProblem {
        step_planning: StepPlanning {
            path: path.clone(),
            initial_pose: initial_pose.clone(),
            initial_support_foot,
            path_progress_reward: 5.0,
            path_distance_penalty: 50.0,
            path_progress_smoothness: 1.0,
            step_size_penalty: 0.5,
            walk_volume_coefficients: WalkVolumeCoefficients::from_extents_and_exponents(
                &WalkVolumeExtents {
                    forward: 0.045,
                    backward: 0.04,
                    outward: 0.1,
                    inward: 0.01,
                    outward_rotation: 1.0,
                    inward_rotation: 1.0,
                },
                1.5,
                2.0,
            ),
        },
        variables: initial_parameter_guess,
    };

    // let (result, report) = LevenbergMarquardt::new()
    //     .with_patience(5000)
    //     .minimize(problem.clone());

    gradient_decent(&mut problem);

    // if !matches!(report.termination, TerminationReason::Converged { .. }) {
    //     dbg!(&report);
    // }

    // if result.variables.iter().all(|&x| x == 0.0) {
    //     dbg!(&report, problem.step_planning);
    // }

    // if !report.termination.was_successful() {
    //     eprintln!("kagge! {report:?}");
    // }

    Ok(problem.variables)
}

fn gradient_decent(problem: &mut StepPlanningProblem) {
    for i in 0..100 {
        let gradient = problem.jacobian().unwrap().transpose();

        if gradient[0].is_nan() {
            dbg!(problem, gradient);
            panic!();
        }
        problem.variables -= gradient * 0.0001;
    }
}

// #[test]
fn foo() {
    let path = Path {
        segments: vec![PathSegment::LineSegment(LineSegment(
            point![0.0, 0.0],
            point![0.49826843, -0.005],
        ))],
    };

    let initial_pose = Pose {
        position: point![0.0, 0.0],
        orientation: 0.0,
    };
    let initial_support_foot = Side::Right;

    let problem = StepPlanningProblem {
        step_planning: StepPlanning {
            path: path.clone(),
            initial_pose: initial_pose.clone(),
            initial_support_foot,
            path_progress_reward: 5.0,
            path_distance_penalty: 50.0,
            path_progress_smoothness: 1.0,
            step_size_penalty: 0.1,
            walk_volume_coefficients: WalkVolumeCoefficients::from_extents_and_exponents(
                &WalkVolumeExtents {
                    forward: 0.045,
                    backward: 0.04,
                    outward: 0.1,
                    inward: 0.01,
                    outward_rotation: 1.0,
                    inward_rotation: 1.0,
                },
                1.5,
                2.0,
            ),
        },
        variables: DVector::zeros(15),
    };

    let (result, report) = LevenbergMarquardt::new()
        .with_stepbound(0.01)
        .minimize(problem);

    if result.variables.iter().all(|&x| x == 0.0) {
        dbg!(result, report);
    }
    panic!();
}

#[test]
fn foo2() {
    let problem = StepPlanningProblem {
        step_planning: StepPlanning {
            path: Path {
                segments: vec![
                    PathSegment::LineSegment(LineSegment(
                        point![0.0, 0.0],
                        point![0.63413036, -4.4703484e-7,],
                    )),
                    PathSegment::Arc(Arc {
                        circle: Circle {
                            center: point![0.6341301, -0.35000044,],
                            radius: 0.35,
                        },
                        start: Angle(1.5707957),
                        end: Angle(1.5694388),
                        direction: geometry::direction::Direction::Clockwise,
                    }),
                    PathSegment::LineSegment(LineSegment(
                        point![0.6346052, -7.4505806e-7,],
                        point![0.79499525, -0.00021849573,],
                    )),
                ],
            },
            initial_pose: Pose {
                position: point![-0.0, 0.0,],
                orientation: -0.0,
            },
            initial_support_foot: Side::Right,
            path_progress_smoothness: 1.0,
            path_progress_reward: 5.0,
            path_distance_penalty: 50.0,
            step_size_penalty: 0.5,
            walk_volume_coefficients: WalkVolumeCoefficients {
                forward_cost: 22.222221,
                backward_cost: 25.0,
                outward_cost: 10.0,
                inward_cost: 100.0,
                outward_rotation_cost: 1.0,
                inward_rotation_cost: 1.0,
                translation_exponent: 1.5,
                rotation_exponent: 2.0,
            },
        },
        variables: dvector![
            0.007884634,
            -5.5583076e-9,
            -6.4623485e-27,
            0.0063005835,
            -4.4416417e-9,
            -1.9984014e-19,
            0.004721283,
            -3.3283027e-9,
            8.881784e-20,
            0.003145544,
            -2.2174744e-9,
            -3.330669e-20,
            0.0015721788,
            -1.108319e-9,
            0.0,
        ],
    };

    let gradient = problem.jacobian().unwrap().transpose();

    dbg!(&gradient);
    assert!(gradient.iter().all(|x| !x.is_nan()));
}

// #[cfg(test)]
// mod tests {
//     use geometry::line_segment::LineSegment;
//     use linear_algebra::point;
//     use types::planned_path::PathSegment;

//     use super::*;

//     #[test]
//     fn foo() {
//         let path = Path {
//             segments: vec![PathSegment::LineSegment(LineSegment(
//                 point![0.0, 0.0,],
//                 point![1.5393465, 1.0662808,],
//             ))],
//         };

//         let initial_parameter_guess = DVector::zeros(15);
//         let initial_pose = Pose {
//             position: point![-0.0014890115, 0.0011079945,],
//             orientation: 0.013183115,
//         };
//         let initial_support_foot = Side::Left;

//         let line_search = MoreThuenteLineSearch::new()
//             .with_bounds(f32::EPSILON.sqrt(), 1.0)
//             .unwrap();
//         let solver = LBFGS::new(line_search, 10);

//         let problem = StepPlanningProblem {
//             step_planning: StepPlanning {
//                 path: path.clone(),
//                 initial_pose: initial_pose.clone(),
//                 initial_support_foot,
//                 path_progress_reward: 5.0,
//                 path_distance_penalty: 50.0,
//                 path_progress_smoothness: 1.0,
//                 step_size_penalty: 1.0,
//                 walk_volume_coefficients: WalkVolumeCoefficients::from_extents_and_exponents(
//                     &WalkVolumeExtents {
//                         forward: 0.045,
//                         backward: 0.04,
//                         outward: 0.1,
//                         inward: 0.01,
//                         outward_rotation: 1.0,
//                         inward_rotation: 1.0,
//                     },
//                     1.5,
//                     2.0,
//                 ),
//             },
//         };

//         let result = Executor::new(problem.clone(), solver)
//             .configure(|state| state.param(initial_parameter_guess).max_iters(1000))
//             .run()
//             .map_err(|error| eyre!("Executor failed: {error:?}"))
//             .unwrap();

//         if let TerminationStatus::Terminated(SolverExit(reason)) = result.state.termination_status {
//             println!("executor failed: {reason:?}");
//             // dbg!(path.segments, initial_pose, initial_support_foot);

//             panic!();
//         };
//     }
// }
