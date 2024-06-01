use std::time::{Duration, SystemTime};

use color_eyre::Result;
use nalgebra::{matrix, Matrix2, Matrix2x4, Matrix4, Matrix4x2};
use serde::{Deserialize, Serialize};

use context_attribute::context;
use coordinate_systems::{Ground, Pixel};
use filtering::kalman_filter::KalmanFilter;
use framework::{AdditionalOutput, HistoricInput, MainOutput, PerceptionInput};
use geometry::circle::Circle;
use linear_algebra::Point2;
use projection::{camera_matrices::CameraMatrices, camera_matrix::CameraMatrix, Projection};
use types::{
    ball::Ball,
    ball_filter::Hypothesis,
    ball_position::{BallPosition, HypotheticalBallPosition},
    cycle_time::CycleTime,
    field_dimensions::FieldDimensions,
    limb::{is_above_limbs, Limb, ProjectedLimbs},
    multivariate_normal_distribution::MultivariateNormalDistribution,
    parameters::{BallFilterNoise, BallFilterParameters},
};

#[derive(Deserialize, Serialize)]
pub struct BallFilter {
    hypotheses: Vec<Hypothesis>,
}

struct RemovedHypotheses(Vec<Hypothesis>);

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    ball_filter_hypotheses: AdditionalOutput<Vec<Hypothesis>, "ball_filter_hypotheses">,
    best_ball_hypothesis: AdditionalOutput<Option<Hypothesis>, "best_ball_hypothesis">,
    chooses_resting_model: AdditionalOutput<Option<bool>, "chooses_resting_model">,

    filtered_balls_in_image_bottom:
        AdditionalOutput<Vec<Circle<Pixel>>, "filtered_balls_in_image_bottom">,
    filtered_balls_in_image_top:
        AdditionalOutput<Vec<Circle<Pixel>>, "filtered_balls_in_image_top">,

    current_odometry_to_last_odometry:
        HistoricInput<Option<nalgebra::Isometry2<f32>>, "current_odometry_to_last_odometry?">,
    historic_camera_matrices: HistoricInput<Option<CameraMatrices>, "camera_matrices?">,

    camera_matrices: RequiredInput<Option<CameraMatrices>, "camera_matrices?">,
    cycle_time: Input<CycleTime, "cycle_time">,

    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    ball_filter_configuration: Parameter<BallFilterParameters, "ball_filter">,

    balls_bottom: PerceptionInput<Option<Vec<Ball>>, "VisionBottom", "balls?">,
    balls_top: PerceptionInput<Option<Vec<Ball>>, "VisionTop", "balls?">,
    projected_limbs: PerceptionInput<Option<ProjectedLimbs>, "VisionBottom", "projected_limbs?">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub ball_position: MainOutput<Option<BallPosition<Ground>>>,
    pub removed_ball_positions: MainOutput<Vec<Point2<Ground>>>,
    pub hypothetical_ball_positions: MainOutput<Vec<HypotheticalBallPosition<Ground>>>,
}

trait KalmanStep {
    fn predict_resting(
        &mut self,
        process_noise: Matrix2<f32>,
        cycle_time: Duration,
        ground_rotation: &Matrix2<f32>,
        ground_translation: nalgebra::Vector2<f32>,
    );

    fn predict_moving(
        &mut self,
        process_noise: Matrix4<f32>,
        cycle_time: Duration,
        velocity_decay_factor: f32,
        ground_rotation: &Matrix2<f32>,
        ground_translation: nalgebra::Vector2<f32>,
    );

    fn update_resting(
        &mut self,
        measurement: nalgebra::Vector2<f32>,
        measurement_noise: Matrix2<f32>,
    );

    fn update_moving(
        &mut self,
        measurement: nalgebra::Vector2<f32>,
        measurement_noise: Matrix2<f32>,
    );
}

impl KalmanStep for Hypothesis {
    fn predict_resting(
        &mut self,
        process_noise: Matrix2<f32>,
        _cycle_time: Duration,
        ground_rotation: &Matrix2<f32>,
        ground_translation: nalgebra::Vector2<f32>,
    ) {
        let constant_position_prediction = matrix![
            1.0, 0.0;
            0.0, 1.0;
        ];
        let state_prediction = constant_position_prediction * ground_rotation;
        self.resting_state.predict(
            state_prediction,
            Matrix2::identity(),
            ground_translation,
            process_noise,
        );
    }

    fn predict_moving(
        &mut self,
        process_noise: Matrix4<f32>,
        cycle_time: Duration,
        velocity_decay_factor: f32,
        rotation: &Matrix2<f32>,
        translation: nalgebra::Vector2<f32>,
    ) {
        let dt = cycle_time.as_secs_f32();
        let constant_velocity_prediction = matrix![
            1.0, 0.0, dt, 0.0;
            0.0, 1.0, 0.0, dt;
            0.0, 0.0, velocity_decay_factor, 0.0;
            0.0, 0.0, 0.0, velocity_decay_factor;
        ];

        let state_rotation = matrix![
            rotation.m11, rotation.m12, 0.0, 0.0;
            rotation.m21, rotation.m22, 0.0, 0.0;
            0.0, 0.0, rotation.m11, rotation.m12;
            0.0, 0.0, rotation.m21, rotation.m22;
        ];

        let state_prediction = constant_velocity_prediction * state_rotation;
        self.moving_state.predict(
            state_prediction,
            Matrix4x2::identity(),
            translation,
            process_noise,
        );
    }

    fn update_resting(
        &mut self,
        measurement: nalgebra::Vector2<f32>,
        measurement_noise: Matrix2<f32>,
    ) {
        self.resting_state.update(
            Matrix2::identity(),
            measurement,
            // Matrix2::from_diagonal(&configuration.measurement_noise_resting)
            //     * detected_position.coords().norm_squared(),
            measurement_noise,
        );
    }

    fn update_moving(
        &mut self,
        measurement: nalgebra::Vector2<f32>,
        measurement_noise: Matrix2<f32>,
    ) {
        self.moving_state.update(
            Matrix2x4::identity(),
            measurement,
            // Matrix2::from_diagonal(&configuration.measurement_noise_moving)
            //     * detected_position.coords().norm_squared(),
            measurement_noise,
        );
    }
}

impl BallFilter {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            hypotheses: Vec::new(),
        })
    }

    fn persistent_balls_in_control_cycle<'a>(
        context: &'a CycleContext,
    ) -> Vec<(&'a SystemTime, Vec<&'a Ball>)> {
        context
            .balls_top
            .persistent
            .iter()
            .zip(context.balls_bottom.persistent.values())
            .map(|((detection_time, balls_top), balls_bottom)| {
                let balls = balls_top
                    .iter()
                    .chain(balls_bottom.iter())
                    .filter_map(|data| data.as_ref())
                    .flat_map(|data| data.iter())
                    .collect();
                (detection_time, balls)
            })
            .collect()
    }

    fn advance_all_hypotheses(
        hypotheses: &mut Vec<Hypothesis>,
        measurements: Vec<(&SystemTime, Vec<&Ball>)>,
        context: &CycleContext,
    ) -> RemovedHypotheses {
        let cycle_time = Duration::from_secs_f32(0.012);
        let filter_parameters = &context.ball_filter_configuration;

        for (detection_time, balls) in measurements {
            let current_odometry_to_last_odometry = context
                .current_odometry_to_last_odometry
                .get(detection_time)
                .expect("current_odometry_to_last_odometry should not be None");

            Self::predict_hypotheses_with_odometry(
                hypotheses,
                cycle_time,
                current_odometry_to_last_odometry.inverse(),
                filter_parameters.velocity_decay_factor,
                filter_parameters.resting_ball_velocity_threshold,
                &filter_parameters.noise,
            );

            let camera_matrices = context.historic_camera_matrices.get(detection_time);
            let projected_limbs_bottom = context
                .projected_limbs
                .persistent
                .get(detection_time)
                .and_then(|limbs| limbs.last())
                .and_then(|limbs| *limbs);

            Self::decay_hypotheses(
                hypotheses,
                camera_matrices,
                projected_limbs_bottom,
                context.field_dimensions.ball_radius,
                filter_parameters,
            );

            for ball in balls {
                let new_hypothesis = Self::update_hypotheses_with_measurement(
                    hypotheses,
                    ball.position,
                    *detection_time,
                    &filter_parameters.noise,
                    context
                        .ball_filter_configuration
                        .measurement_matching_distance,
                );
                hypotheses.extend_from_slice(new_hypothesis.as_slice());
            }
        }

        Self::remove_hypotheses(
            hypotheses,
            context.cycle_time.start_time,
            context.ball_filter_configuration,
            context.field_dimensions,
        )
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let persistent_updates = Self::persistent_balls_in_control_cycle(&context);
        let removed_hypotheses =
            Self::advance_all_hypotheses(&mut self.hypotheses, persistent_updates, &context);

        let filter_parameters = context.ball_filter_configuration;

        context
            .ball_filter_hypotheses
            .fill_if_subscribed(|| self.hypotheses.clone());
        let ball_radius = context.field_dimensions.ball_radius;

        let ball_positions = self
            .hypotheses
            .iter()
            .map(|hypothesis| {
                hypothesis.selected_ball_position(filter_parameters.resting_ball_velocity_threshold)
            })
            .collect::<Vec<_>>();

        let best_ball_hypothesis =
            self.find_best_hypothesis(filter_parameters.validity_output_threshold);
        let best_ball_position = best_ball_hypothesis.map(|hypothesis| {
            hypothesis.selected_ball_position(filter_parameters.resting_ball_velocity_threshold)
        });

        context.chooses_resting_model.fill_if_subscribed(|| {
            best_ball_hypothesis.map(|hypothesis| {
                hypothesis.is_resting(filter_parameters.resting_ball_velocity_threshold)
            })
        });
        context.filtered_balls_in_image_top.fill_if_subscribed(|| {
            project_to_image(&ball_positions, &context.camera_matrices.top, ball_radius)
        });
        context
            .filtered_balls_in_image_bottom
            .fill_if_subscribed(|| {
                project_to_image(
                    &ball_positions,
                    &context.camera_matrices.bottom,
                    ball_radius,
                )
            });

        context
            .best_ball_hypothesis
            .fill_if_subscribed(|| best_ball_hypothesis.cloned());

        let removed_ball_positions = removed_hypotheses
            .0
            .into_iter()
            .filter(|hypothesis| {
                hypothesis.validity >= context.ball_filter_configuration.validity_output_threshold
            })
            .map(|hypothesis| {
                hypothesis
                    .selected_ball_position(
                        context
                            .ball_filter_configuration
                            .resting_ball_velocity_threshold,
                    )
                    .position
            })
            .collect::<Vec<_>>();

        let hypothetical_ball_positions = self
            .hypotheses
            .iter()
            .filter_map(|hypothesis| {
                if hypothesis.validity < context.ball_filter_configuration.validity_output_threshold
                {
                    Some(HypotheticalBallPosition {
                        position: hypothesis
                            .selected_ball_position(
                                context
                                    .ball_filter_configuration
                                    .resting_ball_velocity_threshold,
                            )
                            .position,
                        validity: hypothesis.validity,
                    })
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();

        Ok(MainOutputs {
            ball_position: best_ball_position.into(),
            removed_ball_positions: removed_ball_positions.into(),
            hypothetical_ball_positions: hypothetical_ball_positions.into(),
        })
    }

    fn decay_hypotheses(
        hypotheses: &mut [Hypothesis],
        camera_matrices: Option<&CameraMatrices>,
        projected_limbs: Option<&ProjectedLimbs>,
        ball_radius: f32,
        configuration: &BallFilterParameters,
    ) {
        for hypothesis in hypotheses.iter_mut() {
            let ball_in_view = match (camera_matrices.as_ref(), projected_limbs.as_ref()) {
                (Some(camera_matrices), Some(projected_limbs)) => {
                    is_visible_to_camera(
                        hypothesis,
                        &camera_matrices.bottom,
                        ball_radius,
                        &projected_limbs.limbs,
                        configuration.resting_ball_velocity_threshold,
                    ) || is_visible_to_camera(
                        hypothesis,
                        &camera_matrices.top,
                        ball_radius,
                        &[],
                        configuration.resting_ball_velocity_threshold,
                    )
                }
                _ => false,
            };

            let decay_factor = if ball_in_view {
                configuration.visible_validity_exponential_decay_factor
            } else {
                configuration.hidden_validity_exponential_decay_factor
            };
            hypothesis.validity *= decay_factor;
        }
    }

    fn predict_hypotheses_with_odometry(
        hypotheses: &mut [Hypothesis],
        cycle_time: Duration,
        last_odometry_to_current_odometry: nalgebra::Isometry2<f32>,
        velocity_decay_factor: f32,
        velocity_threshold: f32,
        noise: &BallFilterNoise,
    ) {
        let ground_rotation = last_odometry_to_current_odometry
            .rotation
            .to_rotation_matrix();
        let ground_translation = last_odometry_to_current_odometry.translation.vector;

        for hypothesis in hypotheses {
            hypothesis.predict_moving(
                Matrix4::from_diagonal(&noise.process_noise_moving),
                cycle_time,
                velocity_decay_factor,
                ground_rotation.matrix(),
                ground_translation,
            );
            hypothesis.predict_resting(
                Matrix2::from_diagonal(&noise.process_noise_resting),
                cycle_time,
                ground_rotation.matrix(),
                ground_translation,
            );

            if !hypothesis.is_resting(velocity_threshold) {
                hypothesis.resting_state.mean = hypothesis.moving_state.mean.xy();
            }
        }
    }

    fn update_hypothesis_with_measurement(
        hypothesis: &mut Hypothesis,
        detected_position: Point2<Ground>,
        detection_time: SystemTime,
        noise: &BallFilterNoise,
    ) {
        let measurement_noise = Matrix2::from_diagonal(&noise.measurement_noise)
            * detected_position.coords().norm_squared();

        hypothesis.update_resting(detected_position.inner.coords, measurement_noise);
        hypothesis.update_moving(detected_position.inner.coords, measurement_noise);

        hypothesis.last_update = detection_time;
        hypothesis.validity += 1.0;
    }

    fn update_hypotheses_with_measurement(
        hypotheses: &mut [Hypothesis],
        detected_position: Point2<Ground>,
        detection_time: SystemTime,
        noise: &BallFilterNoise,
        measurement_matching_distance: f32,
    ) -> Option<Hypothesis> {
        let mut matching_hypotheses = hypotheses
            .iter_mut()
            .filter(|hypothesis| {
                (hypothesis.moving_state.mean.xy() - detected_position.inner.coords).norm()
                    < measurement_matching_distance
                    || (hypothesis.resting_state.mean.xy() - detected_position.inner.coords).norm()
                        < measurement_matching_distance
            })
            .peekable();

        if matching_hypotheses.peek().is_none() {
            return Some(Self::spawn_hypothesis(
                detected_position,
                detection_time,
                Matrix4::from_diagonal(&noise.initial_covariance),
                Matrix2::from_diagonal(&noise.initial_covariance.xy()),
            ));
        }

        matching_hypotheses.for_each(|hypothesis| {
            Self::update_hypothesis_with_measurement(
                hypothesis,
                detected_position,
                detection_time,
                noise,
            )
        });

        None
    }

    fn find_best_hypothesis(&self, minimum_validity: f32) -> Option<&Hypothesis> {
        self.hypotheses
            .iter()
            .filter(|hypothesis| hypothesis.validity > minimum_validity)
            .max_by(|left, right| left.validity.total_cmp(&right.validity))
    }

    fn spawn_hypothesis(
        detected_position: Point2<Ground>,
        detection_time: SystemTime,
        moving_covariance: Matrix4<f32>,
        resting_covariance: Matrix2<f32>,
    ) -> Hypothesis {
        let initial_state =
            nalgebra::vector![detected_position.x(), detected_position.y(), 0.0, 0.0];

        Hypothesis {
            moving_state: MultivariateNormalDistribution {
                mean: initial_state,
                covariance: moving_covariance,
            },
            resting_state: MultivariateNormalDistribution {
                mean: initial_state.xy(),
                covariance: resting_covariance,
            },
            validity: 1.0,
            last_update: detection_time,
        }
    }

    fn remove_hypotheses(
        hypotheses: &mut Vec<Hypothesis>,
        now: SystemTime,
        configuration: &BallFilterParameters,
        field_dimensions: &FieldDimensions,
    ) -> RemovedHypotheses {
        let velocity_threshold = configuration.resting_ball_velocity_threshold;

        let (retained_hypotheses, removed_hypotheses) =
            hypotheses.drain(..).partition::<Vec<_>, _>(|hypothesis| {
                let selected_position = hypothesis
                    .selected_ball_position(velocity_threshold)
                    .position;
                let is_inside_field = {
                    selected_position.coords().x().abs()
                        < field_dimensions.length / 2.0 + field_dimensions.border_strip_width
                        && selected_position.y().abs()
                            < field_dimensions.width / 2.0 + field_dimensions.border_strip_width
                };
                now.duration_since(hypothesis.last_update)
                    .expect("Time has run backwards")
                    < configuration.hypothesis_timeout
                    && hypothesis.validity > configuration.validity_discard_threshold
                    && is_inside_field
            });

        let mut deduplicated_hypotheses = Vec::<Hypothesis>::new();
        for hypothesis in retained_hypotheses {
            let hypothesis_in_merge_distance =
                deduplicated_hypotheses
                    .iter_mut()
                    .find(|existing_hypothesis| {
                        (existing_hypothesis
                            .selected_ball_position(velocity_threshold)
                            .position
                            - hypothesis
                                .selected_ball_position(velocity_threshold)
                                .position)
                            .norm()
                            < configuration.hypothesis_merge_distance
                    });

            match hypothesis_in_merge_distance {
                Some(existing_hypothesis) => {
                    existing_hypothesis.moving_state.update(
                        Matrix4::identity(),
                        hypothesis.moving_state.mean,
                        hypothesis.moving_state.covariance,
                    );
                    existing_hypothesis.resting_state.update(
                        Matrix2::identity(),
                        hypothesis.resting_state.mean,
                        hypothesis.resting_state.covariance,
                    );
                }
                None => deduplicated_hypotheses.push(hypothesis),
            }
        }

        *hypotheses = deduplicated_hypotheses;

        RemovedHypotheses(removed_hypotheses)
    }
}

fn project_to_image(
    ball_position: &[BallPosition<Ground>],
    camera_matrix: &CameraMatrix,
    ball_radius: f32,
) -> Vec<Circle<Pixel>> {
    ball_position
        .iter()
        .filter_map(|ball_position| {
            let position_in_image = camera_matrix
                .ground_with_z_to_pixel(ball_position.position, ball_radius)
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
    hypothesis: &Hypothesis,
    camera_matrix: &CameraMatrix,
    ball_radius: f32,
    projected_limbs: &[Limb],
    velocity_threshold: f32,
) -> bool {
    let position_on_ground = hypothesis
        .selected_ball_position(velocity_threshold)
        .position;
    let position_in_image =
        match camera_matrix.ground_with_z_to_pixel(position_on_ground, ball_radius) {
            Ok(position_in_image) => position_in_image,
            Err(_) => return false,
        };
    (0.0..640.0).contains(&position_in_image.x())
        && (0.0..480.0).contains(&position_in_image.y())
        && is_above_limbs(position_in_image, projected_limbs)
}
