use std::{collections::BTreeMap, future::Future, pin::Pin, sync::Arc, time::Duration};

use color_eyre::Result;
use hungarian_algorithm::AssignmentProblem;
use linear_algebra::{IntoFramed, Isometry2};
use nalgebra::{Matrix2, Matrix4};
use ndarray::Array2;
use ordered_float::NotNan;
use ros_z::{cache::Cache, qos::QosDurability};

use booster::Odometer;
use coordinate_systems::{Ground, Pixel};
use geometry::circle::Circle;
use projection::{Projection, camera_matrix::CameraMatrix};
use ros_z::{context::Context, prelude::*, time::Time};
use ros_z_streams::CreateFutureMapBuilder;
use types::{
    ball_detection::BallPercept,
    ball_position::{BallPosition, HypotheticalBallPosition},
    field_dimensions::FieldDimensions,
    multivariate_normal_distribution::MultivariateNormalDistribution,
    object_detection::{Object, RobocupObjectLabel},
    parameters::BallFilterParameters,
    time_wrapper::TimeWrapper,
};

pub use crate::{
    filter::BallFilter,
    hypothesis::{BallHypothesis, BallMode},
};

mod filter;
mod hypothesis;

type Events = BTreeMap<Time, (Option<Odometer>, Option<Vec<Object<RobocupObjectLabel>>>)>;

const ODOMETRY_HISTORY_DURATION: Duration = Duration::from_secs(2);

#[derive(Clone, Default)]
struct FilterState {
    ball_filter: BallFilter,
    last_odometer: Option<Odometer>,
    last_prediction_time: Option<Time>,
    last_visibility_decay_time: Option<Time>,
}

#[derive(Clone, Default)]
struct OdometryHistory {
    samples: BTreeMap<Time, Odometer>,
}

#[derive(Default)]
struct ProcessingResult {
    latest_ball_percepts: Option<Vec<BallPercept>>,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

pub async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("ball_filter").build().await?;

    let parameters = node.bind_parameter_as::<BallFilterParameters>("ball_filter")?;
    let field_dimensions_sub = node
        .subscriber::<FieldDimensions>("field_dimensions")
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .cache(1)
        .build()
        .await?;
    let camera_matrix_cache = node
        .subscriber::<TimeWrapper<CameraMatrix>>("camera_matrix")
        .cache(10)
        .with_stamp(|wrapper: &TimeWrapper<CameraMatrix>| wrapper.time)
        .build()
        .await?;
    let mut future_map = node
        .create_future_map_builder()
        .create_future_subscriber::<Odometer>("inputs/odometer", Duration::from_millis(1))
        .await?
        .create_future_subscriber::<Vec<Object<RobocupObjectLabel>>>(
            "detected_objects",
            Duration::from_millis(1),
        )
        .await?
        .build();
    let filter_state_pub = node
        .publisher::<BallFilter>("ball_filter/ball_filter_state")
        .build()
        .await?;
    let best_ball_hypothesis_pub = node
        .publisher::<Option<BallHypothesis>>("ball_filter/best_ball_hypothesis")
        .build()
        .await?;
    let filtered_balls_in_image_pub = node
        .publisher::<Vec<Circle<Pixel>>>("ball_filter/filtered_balls_in_image")
        .build()
        .await?;
    let ball_percepts_pub = node
        .publisher::<Vec<BallPercept>>("ball_filter/ball_percepts")
        .build()
        .await?;
    let ball_position_pub = node
        .publisher::<Option<BallPosition<Ground>>>("ball_filter/ball_position")
        .build()
        .await?;
    let hypothetical_ball_positions_pub = node
        .publisher::<Vec<HypotheticalBallPosition<Ground>>>(
            "ball_filter/hypothetical_ball_positions",
        )
        .build()
        .await?;

    let mut committed_state = FilterState::default();
    let mut odometry_history = OdometryHistory::default();

    loop {
        let parameters_snapshot = parameters.snapshot();
        let parameters = parameters_snapshot.typed();

        let Some(field_dimensions) = field_dimensions_sub.get_latest() else {
            node.clock().sleep(Duration::from_millis(10)).await;
            continue;
        };

        let future_map_item = future_map.recv().await?;

        let persistent_output_time = future_map_item
            .persistent
            .last_key_value()
            .map(|(time, _)| *time);
        let temporary_output_time = future_map_item
            .temporary
            .last_key_value()
            .map(|(time, _)| *time);

        odometry_history.insert_events(&future_map_item.persistent);
        let persistent_result = process_events(
            &mut committed_state,
            &future_map_item.persistent,
            &odometry_history,
            &camera_matrix_cache,
            parameters,
            &field_dimensions,
        );

        if let Some(output_time) = persistent_output_time {
            remove_invalid_and_merge_hypotheses(
                &mut committed_state.ball_filter,
                output_time,
                parameters,
                &field_dimensions,
            );
            odometry_history.prune(output_time, ODOMETRY_HISTORY_DURATION);
        }

        let mut output_state = committed_state.clone();
        let mut output_odometry_history = odometry_history.clone();
        output_odometry_history.insert_events(future_map_item.temporary);
        let temporary_result = process_events(
            &mut output_state,
            future_map_item.temporary,
            &output_odometry_history,
            &camera_matrix_cache,
            parameters,
            &field_dimensions,
        );

        let Some(output_time) = temporary_output_time.or(persistent_output_time) else {
            continue;
        };
        remove_invalid_and_merge_hypotheses(
            &mut output_state.ball_filter,
            output_time,
            parameters,
            &field_dimensions,
        );

        if let Some(ball_percepts) = temporary_result
            .latest_ball_percepts
            .or(persistent_result.latest_ball_percepts)
        {
            ball_percepts_pub.publish(&ball_percepts).await?;
        }
        filter_state_pub
            .publish(&output_state.ball_filter.clone())
            .await?;

        let best_hypothesis = output_state
            .ball_filter
            .best_hypothesis(parameters.validity_output_threshold);

        best_ball_hypothesis_pub
            .publish(&best_hypothesis.cloned())
            .await?;

        let filtered_ball = best_hypothesis.map(|hypothesis| hypothesis.position());

        let output_balls: Vec<_> = output_state
            .ball_filter
            .hypotheses
            .iter()
            .filter_map(|hypothesis| {
                if hypothesis.validity >= parameters.validity_output_threshold {
                    Some(hypothesis.position())
                } else {
                    None
                }
            })
            .collect();

        let ball_radius = field_dimensions.ball_radius;

        let filtered_balls_in_image = if let Some(timed_camera_matrix) =
            get_recent_camera_matrix(&camera_matrix_cache, output_time, parameters)
        {
            project_to_image(&output_balls, &timed_camera_matrix.inner, ball_radius)
        } else {
            vec![]
        };
        filtered_balls_in_image_pub
            .publish(&filtered_balls_in_image)
            .await?;

        ball_position_pub.publish(&filtered_ball).await?;
        let hypothetical_ball_positions = hypothetical_ball_positions(
            &output_state.ball_filter,
            parameters.validity_output_threshold,
        );
        hypothetical_ball_positions_pub
            .publish(&hypothetical_ball_positions)
            .await?;
    }
}

impl OdometryHistory {
    fn insert_events(&mut self, events: &Events) {
        self.samples.extend(
            events
                .iter()
                .filter_map(|(time, (odometer, _))| Some((*time, (*odometer)?))),
        );
    }

    fn odometer_at(&self, time: Time, maximum_age: Duration) -> Option<Odometer> {
        let before = self.samples.range(..=time).next_back();
        let after = self.samples.range(time..).next();

        match (before, after) {
            (Some((before_time, before_odometer)), Some((after_time, after_odometer))) => {
                if before_time == after_time {
                    return Some(*before_odometer);
                }
                if time.duration_since(*before_time) > maximum_age
                    || after_time.duration_since(time) > maximum_age
                {
                    return None;
                }
                Some(interpolate_odometer(
                    *before_time,
                    *before_odometer,
                    *after_time,
                    *after_odometer,
                    time,
                ))
            }
            (Some((before_time, before_odometer)), None) => {
                (time.duration_since(*before_time) <= maximum_age).then_some(*before_odometer)
            }
            _ => None,
        }
    }

    fn prune(&mut self, latest_time: Time, retention: Duration) {
        let cutoff = latest_time - retention;
        self.samples.retain(|time, _| *time >= cutoff);
    }
}

impl FilterState {
    fn predict_to_time(
        &mut self,
        time: Time,
        odometry_history: &OdometryHistory,
        filter_parameters: &BallFilterParameters,
    ) -> bool {
        if self
            .last_prediction_time
            .is_some_and(|last_time| time < last_time)
        {
            return false;
        }

        let Some(current_odometer) =
            odometry_history.odometer_at(time, filter_parameters.maximum_odometry_age)
        else {
            return false;
        };
        let last_to_current = self
            .last_odometer
            .map_or(Isometry2::identity(), |last_odometer| {
                last_odometer.to(current_odometer)
            });
        let delta_time = self
            .last_prediction_time
            .map_or(Duration::ZERO, |last_time| time.duration_since(last_time));

        self.last_odometer = Some(current_odometer);
        self.last_prediction_time = Some(time);

        self.ball_filter.hypotheses.retain(|hypothesis| {
            hypothesis.validity > filter_parameters.validity_discard_threshold
        });

        self.ball_filter.predict(
            delta_time,
            last_to_current,
            filter_parameters.velocity_decay_factor,
            Matrix4::from_diagonal(&filter_parameters.noise.process_noise_moving),
            Matrix2::from_diagonal(&filter_parameters.noise.process_noise_resting),
            filter_parameters.log_likelihood_of_zero_velocity_threshold,
        );
        true
    }

    fn decay_hypotheses_by_visibility(
        &mut self,
        time: Time,
        camera_matrix: &CameraMatrix,
        filter_parameters: &BallFilterParameters,
        field_dimensions: &FieldDimensions,
    ) {
        if self
            .last_visibility_decay_time
            .is_some_and(|last_time| time < last_time)
        {
            return;
        }
        let delta_time = self
            .last_visibility_decay_time
            .map_or(Duration::ZERO, |last_time| time.duration_since(last_time));
        self.last_visibility_decay_time = Some(time);
        let delta_time = delta_time.as_secs_f32();

        self.ball_filter.decay_hypotheses(|hypothesis| {
            let decay_factor_per_second = decide_validity_decay_for_hypothesis(
                hypothesis,
                camera_matrix,
                field_dimensions.ball_radius,
                filter_parameters,
            );
            decay_factor_per_second.powf(delta_time)
        });
    }
}

fn process_events(
    state: &mut FilterState,
    events: &Events,
    odometry_history: &OdometryHistory,
    camera_matrix_cache: &Cache<TimeWrapper<CameraMatrix>>,
    filter_parameters: &BallFilterParameters,
    field_dimensions: &FieldDimensions,
) -> ProcessingResult {
    let mut result = ProcessingResult::default();

    for (time, (_odometer, detected_objects)) in events {
        if !state.predict_to_time(*time, odometry_history, filter_parameters) {
            continue;
        }

        let Some(detected_objects) = detected_objects else {
            continue;
        };
        let Some(timed_camera_matrix) =
            get_recent_camera_matrix(camera_matrix_cache, *time, filter_parameters)
        else {
            continue;
        };
        let camera_matrix = &timed_camera_matrix.inner;
        let projected_balls = project_detected_balls(
            detected_objects,
            camera_matrix,
            filter_parameters,
            field_dimensions.ball_radius,
        );

        state.decay_hypotheses_by_visibility(
            *time,
            camera_matrix,
            filter_parameters,
            field_dimensions,
        );
        update_hypotheses_with_percepts(
            &mut state.ball_filter,
            *time,
            &projected_balls,
            filter_parameters,
        );
        result.latest_ball_percepts = Some(projected_balls);
    }

    result
}

fn update_hypotheses_with_percepts(
    ball_filter: &mut BallFilter,
    time: Time,
    ball_percepts: &[BallPercept],
    filter_parameters: &BallFilterParameters,
) {
    ball_filter
        .hypotheses
        .retain(|hypothesis| hypothesis.validity > filter_parameters.validity_discard_threshold);

    if ball_percepts.is_empty() {
        return;
    }

    let match_matrix =
        mahalanobis_matrix_of_hypotheses_and_percepts(&ball_filter.hypotheses, ball_percepts);

    let assignment = AssignmentProblem::from_costs(match_matrix).solve();

    let mut used_percepts = vec![];

    for (hypothesis, assigned_percept) in ball_filter.hypotheses.iter_mut().zip(assignment.iter()) {
        if let Some(assigned_percept) = assigned_percept {
            let mahalanobis_distance = -assigned_percept.cost;
            if mahalanobis_distance > filter_parameters.maximum_matching_cost {
                hypothesis.validity *=
                    filter_parameters.maximum_matching_cost_validity_penalty_factor;
                continue;
            }
            let validity_increase = assigned_percept.cost.exp();
            let percept = ball_percepts[assigned_percept.to];
            used_percepts.push(assigned_percept.to);
            hypothesis.update(time, percept.percept_in_ground, validity_increase);
        }
    }

    let unused_percepts = {
        let mut all_percepts = ball_percepts.to_vec();
        used_percepts.sort_unstable();
        for index in used_percepts.into_iter().rev() {
            all_percepts.remove(index);
        }
        all_percepts
    };

    for percept in unused_percepts {
        ball_filter.spawn(
            time,
            percept.percept_in_ground,
            Matrix4::from_diagonal(&filter_parameters.noise.initial_covariance),
        );
    }
}

fn get_recent_camera_matrix(
    camera_matrix_cache: &Cache<TimeWrapper<CameraMatrix>>,
    time: Time,
    filter_parameters: &BallFilterParameters,
) -> Option<Arc<TimeWrapper<CameraMatrix>>> {
    let timed_camera_matrix = camera_matrix_cache.get_before(time)?;
    (time.duration_since(timed_camera_matrix.time) <= filter_parameters.maximum_camera_matrix_age)
        .then_some(timed_camera_matrix)
}

fn interpolate_odometer(
    before_time: Time,
    before: Odometer,
    after_time: Time,
    after: Odometer,
    time: Time,
) -> Odometer {
    let interval = after_time.duration_since(before_time).as_secs_f32();
    if interval <= f32::EPSILON {
        return before;
    }

    let ratio = time.duration_since(before_time).as_secs_f32() / interval;
    Odometer {
        x: before.x + (after.x - before.x) * ratio,
        y: before.y + (after.y - before.y) * ratio,
        theta: before.theta + shortest_angular_difference(before.theta, after.theta) * ratio,
    }
}

fn shortest_angular_difference(from: f32, to: f32) -> f32 {
    (to - from).sin().atan2((to - from).cos())
}

fn remove_invalid_and_merge_hypotheses(
    ball_filter: &mut BallFilter,
    time: Time,
    filter_parameters: &BallFilterParameters,
    field_dimensions: &FieldDimensions,
) {
    let is_hypothesis_valid = |hypothesis: &BallHypothesis| {
        let ball = hypothesis.position();
        let Some(duration_since_last_observation) = ball.age_at(time) else {
            return false;
        };
        let validity_high_enough =
            hypothesis.validity >= filter_parameters.validity_discard_threshold;
        is_ball_inside_field(ball, field_dimensions)
            && validity_high_enough
            && duration_since_last_observation < filter_parameters.hypothesis_timeout
    };

    let should_merge_hypotheses =
        |hypothesis1: &BallHypothesis, hypothesis2: &BallHypothesis| match (
            &hypothesis1.mode,
            &hypothesis2.mode,
        ) {
            (BallMode::Resting(ball1), BallMode::Resting(ball2)) => {
                (ball1.mean - ball2.mean).norm() < filter_parameters.hypothesis_merge_distance
            }
            _ => false,
        };

    ball_filter.remove_hypotheses(is_hypothesis_valid, should_merge_hypotheses);
    ball_filter
        .hypotheses
        .sort_unstable_by(|a, b| b.validity.total_cmp(&a.validity));
    ball_filter
        .hypotheses
        .truncate(filter_parameters.maximum_number_of_hypotheses);
}

fn hypothetical_ball_positions(
    ball_filter: &BallFilter,
    validity_limit: f32,
) -> Vec<HypotheticalBallPosition<Ground>> {
    ball_filter
        .hypotheses
        .iter()
        .filter_map(|hypothesis| {
            if hypothesis.validity < validity_limit {
                Some(HypotheticalBallPosition {
                    position: hypothesis.position().position,
                    validity: hypothesis.validity,
                })
            } else {
                None
            }
        })
        .collect()
}

fn mahalanobis_matrix_of_hypotheses_and_percepts(
    hypotheses: &[BallHypothesis],
    percepts: &[BallPercept],
) -> Array2<NotNan<f32>> {
    Array2::from_shape_fn((hypotheses.len(), percepts.len()), |(i, j)| {
        let hypothesis = &hypotheses[i];
        let percept = &percepts[j];
        let ball = hypothesis.position();

        let residual = percept.percept_in_ground.mean - ball.position.inner.coords;
        let covariance = hypothesis.position_covariance() + percept.percept_in_ground.covariance;

        let Some(cholesky) = covariance.cholesky() else {
            return NotNan::new(-f32::MAX).expect("finite cost is not NaN");
        };
        let mahalanobis_distance = residual.dot(&cholesky.solve(&residual));

        NotNan::new(-mahalanobis_distance)
            .unwrap_or_else(|_| NotNan::new(-f32::MAX).expect("finite cost is not NaN"))
    })
}

fn project_detected_balls(
    detections: &[Object<RobocupObjectLabel>],
    camera_matrix: &CameraMatrix,
    parameters: &BallFilterParameters,
    ball_radius: f32,
) -> Vec<BallPercept> {
    detections
        .iter()
        .filter_map(|detection| {
            if detection.label != RobocupObjectLabel::Ball {
                return None;
            }
            let area = detection.bounding_box.area;
            let position = camera_matrix
                .pixel_to_ground_with_z(area.center(), ball_radius)
                .ok()?;

            let detected_ball_radius =
                (area.max.x() - area.min.x()).min(area.max.y() - area.min.y()) / 2.0;

            let circle = Circle {
                center: area.center(),
                radius: detected_ball_radius,
            };

            let projected_covariance = {
                let scaled_noise = parameters
                    .noise
                    .detection_noise
                    .inner
                    .map(|x| (detected_ball_radius * x).powi(2))
                    .framed();
                camera_matrix
                    .project_noise_to_ground(position, scaled_noise)
                    .ok()?
            };

            Some(BallPercept {
                percept_in_ground: MultivariateNormalDistribution {
                    mean: position.inner.coords,
                    covariance: projected_covariance,
                },
                image_location: circle,
            })
        })
        .collect()
}

fn decide_validity_decay_for_hypothesis(
    hypothesis: &BallHypothesis,
    camera_matrix: &CameraMatrix,
    ball_radius: f32,
    configuration: &BallFilterParameters,
) -> f32 {
    let ball = hypothesis.position();
    let is_ball_in_view = is_visible_to_camera(&ball, camera_matrix, ball_radius);

    match is_ball_in_view {
        true => configuration.visible_validity_exponential_decay_factor,
        false => configuration.hidden_validity_exponential_decay_factor,
    }
}

fn is_ball_inside_field(ball: BallPosition<Ground>, field_dimensions: &FieldDimensions) -> bool {
    ball.position.x().abs() < field_dimensions.length / 2.0
        && ball.position.y().abs() < field_dimensions.width / 2.0
}

fn project_to_image(
    filtered_balls: &[BallPosition<Ground>],
    camera_matrix: &CameraMatrix,
    ball_radius: f32,
) -> Vec<Circle<Pixel>> {
    filtered_balls
        .iter()
        .filter_map(|filtered_ball| {
            let position_in_image = camera_matrix
                .ground_with_z_to_pixel(filtered_ball.position, ball_radius)
                .ok()?;
            let radius = camera_matrix
                .get_pixel_radius(ball_radius, position_in_image)
                .ok()?;
            Some(Circle {
                center: position_in_image,
                radius,
            })
        })
        .collect()
}

fn is_visible_to_camera(
    ball: &BallPosition<Ground>,
    camera_matrix: &CameraMatrix,
    ball_radius: f32,
) -> bool {
    let position_in_image = match camera_matrix.ground_with_z_to_pixel(ball.position, ball_radius) {
        Ok(position_in_image) => position_in_image,
        Err(_) => return false,
    };
    (0.0..camera_matrix.image_size.x()).contains(&position_in_image.x())
        && (0.0..camera_matrix.image_size.y()).contains(&position_in_image.y())
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use linear_algebra::point;
    use nalgebra::vector;
    use types::{
        multivariate_normal_distribution::MultivariateNormalDistribution,
        parameters::BallFilterNoise,
    };

    use super::*;

    fn assert_approx_eq(left: f32, right: f32) {
        assert!(
            (left - right).abs() < 1e-5,
            "expected {left} to approximately equal {right}"
        );
    }

    fn moving_hypothesis(mean: nalgebra::Vector4<f32>) -> BallHypothesis {
        BallHypothesis {
            mode: BallMode::Moving(MultivariateNormalDistribution {
                mean,
                covariance: Matrix4::identity(),
            }),
            last_seen: Time::zero(),
            validity: 1.0,
        }
    }

    fn filter_parameters() -> BallFilterParameters {
        BallFilterParameters {
            hypothesis_timeout: Duration::from_secs(20),
            maximum_odometry_age: Duration::from_millis(50),
            maximum_camera_matrix_age: Duration::from_millis(100),
            maximum_number_of_hypotheses: 15,
            log_likelihood_of_zero_velocity_threshold: f32::INFINITY,
            hypothesis_merge_distance: 0.1,
            visible_validity_exponential_decay_factor: 0.5,
            hidden_validity_exponential_decay_factor: 0.9,
            validity_output_threshold: 0.5,
            validity_discard_threshold: 0.2,
            velocity_decay_factor: 1.0,
            noise: BallFilterNoise {
                detection_noise: vector![5.0, 5.0].framed(),
                process_noise_moving: vector![0.0, 0.0, 0.0, 0.0],
                process_noise_resting: vector![0.0, 0.0],
                initial_covariance: vector![1.0, 1.0, 1.0, 1.0],
            },
            maximum_matching_cost: 1.0,
            maximum_matching_cost_validity_penalty_factor: 0.1,
        }
    }

    #[test]
    fn hypothesis_update_matching() {
        let hypothesis1 = BallHypothesis {
            mode: BallMode::Moving(MultivariateNormalDistribution {
                mean: nalgebra::vector![0.0, 1.0, 0.0, 0.0],
                covariance: Matrix4::identity(),
            }),
            last_seen: Time::zero(),
            validity: 0.0,
        };
        let hypothesis2 = BallHypothesis {
            mode: BallMode::Moving(MultivariateNormalDistribution {
                mean: nalgebra::vector![0.0, -1.0, 0.0, 0.0],
                covariance: Matrix4::identity(),
            }),
            last_seen: Time::zero(),
            validity: 0.0,
        };

        let percept1 = BallPercept {
            percept_in_ground: MultivariateNormalDistribution {
                mean: vector![0.0, 0.4],
                covariance: Matrix2::identity(),
            },
            image_location: Circle::new(point![0.0, 0.0], 1.0),
        };
        let percept2 = BallPercept {
            percept_in_ground: MultivariateNormalDistribution {
                mean: vector![0.0, -0.6],
                covariance: Matrix2::identity(),
            },
            image_location: Circle::new(point![0.0, 0.0], 1.0),
        };

        let hypotheses = vec![hypothesis1, hypothesis2];
        let percepts = vec![percept1, percept2];

        let costs = mahalanobis_matrix_of_hypotheses_and_percepts(&hypotheses, &percepts);
        let assignment = AssignmentProblem::from_costs(costs).solve();

        let percept_of_hypothesis1 = assignment[0].unwrap().to;
        assert_eq!(percept_of_hypothesis1, 0);

        let percept_of_hypothesis2 = assignment[1].unwrap().to;
        assert_eq!(percept_of_hypothesis2, 1);

        assert_eq!(assignment.len(), 2);
        assert_eq!(assignment.into_iter().flatten().count(), 2);
    }

    #[test]
    fn moving_prediction_is_independent_of_step_size() {
        let mut single_step = moving_hypothesis(vector![0.0, 0.0, 4.0, -2.0]);
        let mut two_steps = single_step.clone();

        single_step.predict(
            Duration::from_secs(2),
            Isometry2::identity(),
            0.5,
            Matrix4::zeros(),
            Matrix2::zeros(),
            f32::INFINITY,
        );
        for _ in 0..2 {
            two_steps.predict(
                Duration::from_secs(1),
                Isometry2::identity(),
                0.5,
                Matrix4::zeros(),
                Matrix2::zeros(),
                f32::INFINITY,
            );
        }

        let (BallMode::Moving(single_step), BallMode::Moving(two_steps)) =
            (&single_step.mode, &two_steps.mode)
        else {
            panic!("hypotheses should remain moving");
        };
        for (single_step, two_steps) in single_step.mean.iter().zip(two_steps.mean.iter()) {
            assert_approx_eq(*single_step, *two_steps);
        }
    }

    #[test]
    fn process_noise_scales_with_elapsed_seconds() {
        let mut moving = BallHypothesis {
            mode: BallMode::Moving(MultivariateNormalDistribution {
                mean: vector![0.0, 0.0, 0.0, 0.0],
                covariance: Matrix4::zeros(),
            }),
            last_seen: Time::zero(),
            validity: 1.0,
        };
        let moving_process_noise = Matrix4::from_diagonal(&vector![2.0, 4.0, 6.0, 8.0]);

        moving.predict(
            Duration::from_millis(500),
            Isometry2::identity(),
            1.0,
            moving_process_noise,
            Matrix2::zeros(),
            f32::INFINITY,
        );

        let BallMode::Moving(moving) = &moving.mode else {
            panic!("hypothesis should remain moving");
        };
        assert_approx_eq(moving.covariance[(0, 0)], 1.0);
        assert_approx_eq(moving.covariance[(1, 1)], 2.0);
        assert_approx_eq(moving.covariance[(2, 2)], 3.0);
        assert_approx_eq(moving.covariance[(3, 3)], 4.0);

        let mut resting = BallHypothesis {
            mode: BallMode::Resting(MultivariateNormalDistribution {
                mean: vector![0.0, 0.0],
                covariance: Matrix2::zeros(),
            }),
            last_seen: Time::zero(),
            validity: 1.0,
        };

        resting.predict(
            Duration::from_millis(250),
            Isometry2::identity(),
            1.0,
            Matrix4::zeros(),
            Matrix2::from_diagonal(&vector![4.0, 8.0]),
            f32::INFINITY,
        );

        let BallMode::Resting(resting) = &resting.mode else {
            panic!("hypothesis should remain resting");
        };
        assert_approx_eq(resting.covariance[(0, 0)], 1.0);
        assert_approx_eq(resting.covariance[(1, 1)], 2.0);
    }

    #[test]
    fn zero_velocity_likelihood_uses_velocity_covariance() {
        let mut covariance = Matrix4::identity();
        covariance[(0, 0)] = 1000.0;
        covariance[(1, 1)] = 1000.0;
        covariance[(2, 2)] = 0.01;
        covariance[(3, 3)] = 0.01;

        let mut hypothesis = BallHypothesis {
            mode: BallMode::Moving(MultivariateNormalDistribution {
                mean: vector![0.0, 0.0, 1.0, 0.0],
                covariance,
            }),
            last_seen: Time::zero(),
            validity: 1.0,
        };

        hypothesis.predict(
            Duration::ZERO,
            Isometry2::identity(),
            1.0,
            Matrix4::zeros(),
            Matrix2::zeros(),
            -10.0,
        );

        assert!(matches!(hypothesis.mode, BallMode::Moving(_)));
    }

    #[test]
    fn prediction_requires_recent_odometry_at_target_time() {
        let parameters = filter_parameters();
        let mut state = FilterState::default();
        state
            .ball_filter
            .hypotheses
            .push(moving_hypothesis(vector![0.0, 0.0, 1.0, 0.0]));

        let mut history = OdometryHistory::default();
        history.samples.insert(
            Time::from_nanos(0),
            Odometer {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
            },
        );

        assert!(state.predict_to_time(Time::from_nanos(10_000_000), &history, &parameters));
        assert_eq!(
            state.last_prediction_time,
            Some(Time::from_nanos(10_000_000))
        );

        assert!(!state.predict_to_time(Time::from_nanos(100_000_000), &history, &parameters));
        assert_eq!(
            state.last_prediction_time,
            Some(Time::from_nanos(10_000_000))
        );
    }

    #[test]
    fn odometry_is_interpolated_at_target_time() {
        let mut history = OdometryHistory::default();
        history.samples.insert(
            Time::from_nanos(0),
            Odometer {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
            },
        );
        history.samples.insert(
            Time::from_nanos(100_000_000),
            Odometer {
                x: 1.0,
                y: 2.0,
                theta: 1.0,
            },
        );

        let odometer = history
            .odometer_at(Time::from_nanos(50_000_000), Duration::from_millis(100))
            .unwrap();

        assert_approx_eq(odometer.x, 0.5);
        assert_approx_eq(odometer.y, 1.0);
        assert_approx_eq(odometer.theta, 0.5);
    }

    #[test]
    fn spawned_hypothesis_starts_at_unmatched_measurement() {
        let mut ball_filter = BallFilter {
            hypotheses: vec![moving_hypothesis(vector![100.0, 100.0, 0.0, 0.0])],
        };
        let measurement = MultivariateNormalDistribution {
            mean: vector![1.0, 2.0],
            covariance: Matrix2::identity(),
        };

        ball_filter.spawn(Time::zero(), measurement, Matrix4::identity());

        let BallMode::Moving(spawned) = &ball_filter.hypotheses[1].mode else {
            panic!("spawned hypothesis should be moving");
        };
        assert_approx_eq(spawned.mean.x, 1.0);
        assert_approx_eq(spawned.mean.y, 2.0);
    }
}
