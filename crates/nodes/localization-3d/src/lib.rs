use std::{num::NonZeroUsize, pin::Pin, sync::Arc, time::Duration};

use booster::ImuState;
use color_eyre::{
    Result,
    eyre::{Context as _, bail},
};
use coordinate_systems::{Camera, Field, Pixel, Robot};
use global_localizer::{GlobalLocalizationInput, GlobalLocalizationResult, GlobalLocalizer};
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::{IntoTransform, Isometry3, Point2, point};
use localization_factrs::{
    BackendConfiguration, CameraIntrinsics, InitialState, VinsFrontend, VinsFrontendError,
    VisualReprojectionAssociation, initialize,
};
use nalgebra::{Matrix2, Matrix3, Point3, SMatrix, Vector3};
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{
    Message,
    cache::Cache,
    context::Context,
    parameter::NodeParametersExt,
    qos::{QosDurability, QosHistory, QosProfile},
    time::Time,
};
use serde::{Deserialize, Serialize};
use tokio::{select, task::JoinHandle};
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
    visual_odometry::VisualOdometryDelta as VisualOdometryDeltaMessage,
};

mod global_localizer;

/// Global field-feature localizer thresholds.
pub use global_localizer::GlobalLocalizerConfig as GlobalLocalizerParameters;

/// Runtime parameters for the 3D localization node.
#[derive(Clone, Debug, Deserialize, Serialize, Message)]
#[serde(deny_unknown_fields)]
pub struct Localization3dParameters {
    /// Parameters for fixed-field-feature global localization from object detections.
    pub global_localizer: GlobalLocalizerParameters,
    /// Pixel residual variance for accepted visual feature associations.
    pub visual_feature_noise_variance: f64,
}

impl Default for Localization3dParameters {
    fn default() -> Self {
        Self {
            global_localizer: GlobalLocalizerParameters::default(),
            visual_feature_noise_variance: 3600.0,
        }
    }
}

impl Localization3dParameters {
    fn validate(&self) -> std::result::Result<(), String> {
        self.global_localizer.validate()?;
        if !self.visual_feature_noise_variance.is_finite()
            || self.visual_feature_noise_variance <= 0.0
        {
            return Err("visual_feature_noise_variance must be finite and > 0".to_string());
        }
        Ok(())
    }
}

pub struct GlobalVisualLocalization {
    /// Debug payload for the best visual global localization result, if any.
    pub debug: Option<GlobalLocalizationDebug>,
    /// Fixed associations that are unique up to the field symmetry and safe to ingest.
    pub unique_associations: Option<Vec<VisualReprojectionAssociation>>,
}

const MAX_CAMERA_MATRIX_TIME_DISTANCE: Duration = Duration::from_millis(100);

struct GlobalLocalizationTaskOutput {
    source_time: Time,
    robot_to_camera: nalgebra::Isometry3<f32>,
    localization: GlobalVisualLocalization,
}

pub fn backend_configuration(visual_feature_noise_variance: f64) -> BackendConfiguration {
    BackendConfiguration {
        knot_spacing: Duration::from_millis(200),
        max_optimization_window: Duration::from_secs(3),
        optimizer_max_iterations: 5,
        gyroscope_process_noise: Matrix3::identity() * 0.01,
        roll_pitch_yaw_noise: Matrix3::identity() * 0.01,
        accelerometer_process_noise: Matrix3::identity() * 0.01,
        visual_feature_noise: Matrix2::identity() * visual_feature_noise_variance,
        // factrs::SE3 tangent order is [rot_x, rot_y, rot_z, trans_x, trans_y, trans_z].
        visual_odometry_noise: SMatrix::<f64, 6, 6>::identity() * 1.0e-4,
        foot_ground_softness: 1.0e-3,
        foot_ground_sigma: 0.01,
        gravity: Vector3::new(0.0, 0.0, 9.81),
    }
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

pub async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("localization3d").build().await?;
    let parameters = node.bind_parameter_as::<Localization3dParameters>("localization3d")?;
    parameters.add_validation_hook(Localization3dParameters::validate)?;

    let imu_subscriber = node
        .subscriber::<ImuState>("inputs/imu_state")?
        .build()
        .await?;

    let camera_matrix_cache = node
        .create_cache::<TimeWrapper<CameraMatrix>>("camera_matrix", 128)?
        .with_stamp(|message| message.time)
        .build()
        .await?;

    let object_subscriber = node
        .subscriber::<Vec<Object<RobocupObjectLabel>>>("detected_objects")?
        .qos(QosProfile {
            history: QosHistory::KeepLast(NonZeroUsize::MIN),
            ..Default::default()
        })
        .build()
        .await?;

    let visual_odometry_subscriber = node
        .subscriber::<VisualOdometryDeltaMessage>(
            "visual_odometry/current_left_camera_to_previous_left_camera",
        )?
        .build()
        .await?;

    let robot_kinematics_subscriber = node
        .subscriber::<TimeWrapper<RobotKinematics>>("robot_kinematics")?
        .build()
        .await?;

    let field_dimensions_cache = node
        .create_cache::<FieldDimensions>("field_dimensions", 1)?
        .with_qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let localization_publisher = node
        .publisher::<Option<Isometry3<Field, Robot>>>("localization")?
        .build()
        .await?;
    let calibrated_intrinsics_publisher = node
        .publisher::<Intrinsic>("debug/calibrated_intrinsics")?
        .build()
        .await?;
    let global_localization_publisher = node
        .publisher::<Option<GlobalLocalizationDebug>>("debug/global_localization")?
        .build()
        .await?;

    let initial_state = wait_for_initial_state(&camera_matrix_cache).await;
    let initial_parameters = parameters.snapshot().typed().clone();
    let (mut frontend, backend) = initialize(
        backend_configuration(initial_parameters.visual_feature_noise_variance),
        initial_state,
    );
    let mut backend_handle = std::pin::pin!(tokio::task::spawn_blocking(|| backend.run_loop()));
    let mut global_localization_task: Option<JoinHandle<GlobalLocalizationTaskOutput>> = None;

    loop {
        select! {
            // The detection publisher announces the originating image time as source time.
            objects = object_subscriber.recv_with_metadata(), if global_localization_task.is_none() => {
                let objects = objects?;
                let visual_features = find_detected_visual_features(objects.message);
                let parameters = parameters.snapshot().typed().clone();
                if visual_features.supported_feature_count()
                    < parameters.global_localizer.min_inliers.max(3)
                {
                    let debug: Option<GlobalLocalizationDebug> = None;
                    global_localization_publisher.publish(&debug).await?;
                    continue;
                }

                let Some(camera_matrix) = camera_matrix_cache.get_nearest(objects.source_time) else {
                    continue;
                };
                if !camera_matrix_is_fresh(&camera_matrix, objects.source_time) {
                    continue;
                }
                let Some(field_dimensions) = field_dimensions_cache.get_nearest(objects.source_time) else {
                    continue;
                };

                let source_time = objects.source_time;
                let camera_matrix = camera_matrix.inner.clone();
                let field_dimensions = *field_dimensions.as_ref();
                let pose_hint = pose_hint_from_frontend(&frontend);
                global_localization_task = Some(tokio::task::spawn_blocking(move || {
                    let robot_to_camera = robot_to_left_camera(&camera_matrix);
                    let localization = localize_global_visual_features(
                        &visual_features,
                        &camera_matrix,
                        &field_dimensions,
                        pose_hint,
                        &parameters.global_localizer,
                    );

                    GlobalLocalizationTaskOutput {
                        source_time,
                        robot_to_camera,
                        localization,
                    }
                }));
            }
            global_localization = async {
                global_localization_task
                    .as_mut()
                    .expect("task exists because select branch is gated")
                    .await
            }, if global_localization_task.is_some() => {
                let output = global_localization.wrap_err("global localization task failed")?;
                global_localization_task = None;

                global_localization_publisher
                    .publish(&output.localization.debug)
                    .await?;

                if let Some(associations) = output.localization.unique_associations {
                    frontend
                        .ingest_visual_reprojection_associations(
                            output.source_time.to_wallclock(),
                            associations,
                            output.robot_to_camera,
                        )
                        .wrap_err("ingest of globally associated visual features failed")?;
                }
            }
            // IMU payloads have no sensor timestamp; ros-z source time is the aligned clock.
            imu = imu_subscriber.recv_with_metadata() => {
                let imu = imu?;
                frontend.ingest_imu(imu.source_time.to_wallclock(), imu.message)
                    .wrap_err("failed to ingest imu measurement into frontend")?;

                // let transform = frontend
                //     .last_optimization_result()
                //     .map(|result| result.transform.cast::<f32>().framed_transform());
                // localization_publisher.publish(&transform).await?;
            }
            visual_odometry = visual_odometry_subscriber.recv() => {
                let visual_odometry = visual_odometry?;
                let Some(previous_camera_matrix) = camera_matrix_cache.get_nearest(visual_odometry.previous_time) else {
                    continue;
                };
                if !camera_matrix_is_fresh(&previous_camera_matrix, visual_odometry.previous_time) {
                    continue;
                }
                let Some(current_camera_matrix) = camera_matrix_cache.get_nearest(visual_odometry.current_time) else {
                    continue;
                };
                if !camera_matrix_is_fresh(&current_camera_matrix, visual_odometry.current_time) {
                    continue;
                }

                ingest_visual_odometry(&mut frontend, visual_odometry, &previous_camera_matrix.inner, &current_camera_matrix.inner)
                    .wrap_err("failed to ingest visual odometry measurement into frontend")?;
            }
            robot_kinematics = robot_kinematics_subscriber.recv() => {
                let robot_kinematics = robot_kinematics?;
                ingest_foot_heights(&mut frontend, robot_kinematics)
                    .wrap_err("failed to ingest foot height measurement into frontend")?;
            }
            result = &mut backend_handle => {
                result.wrap_err("failed to join")?.wrap_err("solver failed")?;
                bail!("solver stopped unexpectedly")
            }
            result = frontend.wait_for_optimization_result() => {
                result?;
                let result = frontend.last_optimization_result();
                let transform = result
                    .as_ref()
                    .map(|result| localization_transform_from_backend_pose(&result.transform));

                localization_publisher.publish(&transform).await?;

                if let Some(result) = result {
                    calibrated_intrinsics_publisher
                        .publish(&intrinsic_from_camera_intrinsics(&result.camera_intrinsics))
                        .await?;
                }
            }
        }
    }
}

fn localization_transform_from_backend_pose(
    robot_to_field: &nalgebra::Isometry3<f64>,
) -> Isometry3<Field, Robot> {
    robot_to_field.inverse().cast::<f32>().framed_transform()
}

fn pose_hint_from_frontend(frontend: &VinsFrontend) -> Option<Isometry3<Robot, Field>> {
    frontend
        .peek_last_optimization_result()
        .map(|result| result.transform.cast::<f32>().framed_transform())
}

fn camera_matrix_is_fresh(camera_matrix: &TimeWrapper<CameraMatrix>, time: Time) -> bool {
    time_distance(camera_matrix.time, time) <= MAX_CAMERA_MATRIX_TIME_DISTANCE
}

fn time_distance(a: Time, b: Time) -> Duration {
    Duration::from_nanos(a.as_nanos().abs_diff(b.as_nanos()))
}

/// Runs global localization and returns debug data plus backend-safe associations.
pub fn localize_global_visual_features(
    visual_features: &DetectedVisualFeatures,
    camera_matrix: &CameraMatrix,
    field_dimensions: &FieldDimensions,
    pose_hint: Option<Isometry3<Robot, Field>>,
    parameters: &GlobalLocalizerParameters,
) -> GlobalVisualLocalization {
    let localizer = GlobalLocalizer::new(*parameters);
    let result = localizer.localize(GlobalLocalizationInput {
        visual_features,
        field_dimensions,
        ground_to_robot: camera_matrix.ground_to_robot,
        robot_to_camera: robot_to_camera(camera_matrix),
        camera_intrinsic: camera_matrix.intrinsics,
        pose_hint,
    });

    GlobalVisualLocalization {
        debug: result.as_ref().map(global_localization_debug_from_result),
        unique_associations: result.as_ref().and_then(|result| {
            result
                .unique_reprojection_associations()
                .map(Iterator::collect)
        }),
    }
}

fn global_localization_debug_from_result(
    result: &GlobalLocalizationResult,
) -> GlobalLocalizationDebug {
    let status = match result {
        GlobalLocalizationResult::Ambiguous(_) => GlobalLocalizationDebugStatus::Ambiguous,
        GlobalLocalizationResult::Unique(_) => GlobalLocalizationDebugStatus::Unique,
        GlobalLocalizationResult::UniqueModuloSymmetry(_) => {
            GlobalLocalizationDebugStatus::UniqueModuloSymmetry
        }
    };
    let associations = result.associations();
    GlobalLocalizationDebug {
        robot_to_field: associations.robot_to_field,
        status,
        inliers: associations.score.inliers,
        reprojection_rmse: associations.score.reprojection_rmse,
        total_cost: associations.score.total_cost,
    }
}

async fn wait_for_initial_state(
    camera_matrix_cache: &Cache<TimeWrapper<CameraMatrix>>,
) -> InitialState {
    let mut interval = tokio::time::interval(Duration::from_millis(10));
    loop {
        if let Some(camera_matrix) = camera_matrix_cache.get_latest() {
            return initial_state_from_camera_matrix(&camera_matrix.inner);
        }
        interval.tick().await;
    }
}

pub fn initial_state_from_camera_matrix(camera_matrix: &CameraMatrix) -> InitialState {
    let robot_to_ground = camera_matrix.ground_to_robot.inverse().inner;
    let initial_pose = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(0.0, 0.0, robot_to_ground.translation.vector.z as f64),
        robot_to_ground.rotation.cast::<f64>(),
    );

    InitialState::from_isometry_and_intrinsics(
        initial_pose,
        Vector3::zeros(),
        camera_intrinsics_from_matrix(camera_matrix),
    )
}

pub fn camera_intrinsics_from_matrix(camera_matrix: &CameraMatrix) -> CameraIntrinsics {
    CameraIntrinsics::new(
        nalgebra::vector![
            camera_matrix.intrinsics.focals.x as f64,
            camera_matrix.intrinsics.focals.y as f64,
        ],
        nalgebra::vector![
            camera_matrix.intrinsics.optical_center.x() as f64,
            camera_matrix.intrinsics.optical_center.y() as f64,
        ],
    )
}

pub fn intrinsic_from_camera_intrinsics(camera_intrinsics: &CameraIntrinsics) -> Intrinsic {
    let focals = camera_intrinsics.focals();
    let optical_center = camera_intrinsics.optical_center();
    Intrinsic::new(
        nalgebra::vector![focals.x as f32, focals.y as f32],
        point![optical_center.x as f32, optical_center.y as f32],
    )
}

pub fn find_detected_goalposts(detections: Vec<Object<RobocupObjectLabel>>) -> Vec<Point2<Pixel>> {
    find_detected_visual_features(detections).goalposts
}

#[derive(Debug, Default, PartialEq)]
pub struct DetectedVisualFeatures {
    pub goalposts: Vec<Point2<Pixel>>,
    pub l_spots: Vec<Point2<Pixel>>,
    pub t_spots: Vec<Point2<Pixel>>,
    pub penalty_spots: Vec<Point2<Pixel>>,
}

impl DetectedVisualFeatures {
    pub fn supported_feature_count(&self) -> usize {
        self.goalposts.len() + self.l_spots.len() + self.t_spots.len() + self.penalty_spots.len()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
/// Published debug data for a successful global localization hypothesis.
pub struct GlobalLocalizationDebug {
    /// Best robot pose in the field frame for this visual result.
    pub robot_to_field: Isometry3<Robot, Field>,
    /// Whether the best result is ambiguous, unique, or unique modulo field symmetry.
    pub status: GlobalLocalizationDebugStatus,
    /// Number of fixed feature associations accepted by the reprojection gate.
    pub inliers: usize,
    /// Root-mean-square reprojection error in pixels.
    pub reprojection_rmse: f32,
    /// Sum of squared reprojection errors in pixels squared.
    pub total_cost: f32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Message)]
/// Classification of a successful global localization result.
pub enum GlobalLocalizationDebugStatus {
    /// A plausible non-symmetric alternative assignment remains.
    Ambiguous,
    /// No plausible alternative survived the deterministic search.
    Unique,
    /// The assignment is unique after quotienting the unavoidable 180 degree
    /// field symmetry. The chosen branch follows the pose hint when available.
    UniqueModuloSymmetry,
}

pub fn find_detected_visual_features(
    detections: Vec<Object<RobocupObjectLabel>>,
) -> DetectedVisualFeatures {
    detections
        .into_iter()
        .fold(DetectedVisualFeatures::default(), |mut features, object| {
            match object.label {
                RobocupObjectLabel::GoalPost => {
                    features.goalposts.push(pixel_bottom_center(object))
                }
                RobocupObjectLabel::LSpot => features.l_spots.push(pixel_center(object)),
                RobocupObjectLabel::TSpot => features.t_spots.push(pixel_center(object)),
                RobocupObjectLabel::PenaltySpot => {
                    features.penalty_spots.push(pixel_center(object))
                }
                _ => {}
            }
            features
        })
}

fn pixel_bottom_center(object: Object<RobocupObjectLabel>) -> Point2<Pixel> {
    let area = object.bounding_box.area;
    point![(area.min.x() + area.max.x()) * 0.5, area.max.y()]
}

fn pixel_center(object: Object<RobocupObjectLabel>) -> Point2<Pixel> {
    let area = object.bounding_box.area;
    point![
        (area.min.x() + area.max.x()) * 0.5,
        (area.min.y() + area.max.y()) * 0.5
    ]
}

pub fn ingest_visual_odometry(
    frontend: &mut VinsFrontend,
    delta: VisualOdometryDeltaMessage,
    previous_camera_matrix: &CameraMatrix,
    current_camera_matrix: &CameraMatrix,
) -> Result<(), VinsFrontendError> {
    frontend.ingest_visual_odometry_delta(
        delta.previous_time.to_wallclock(),
        delta.current_time.to_wallclock(),
        robot_to_left_camera(previous_camera_matrix),
        robot_to_left_camera(current_camera_matrix),
        delta.current_left_camera_to_previous_left_camera,
    )
}

pub fn ingest_foot_heights(
    frontend: &mut VinsFrontend,
    robot_kinematics: TimeWrapper<RobotKinematics>,
) -> Result<(), VinsFrontendError> {
    let (left_sole_in_robot, right_sole_in_robot) = foot_height_points(&robot_kinematics.inner);

    frontend.ingest_foot_heights(
        robot_kinematics.time.to_wallclock(),
        left_sole_in_robot,
        right_sole_in_robot,
    )
}

fn foot_height_points(robot_kinematics: &RobotKinematics) -> (Point3<f64>, Point3<f64>) {
    (
        robot_kinematics
            .left_leg
            .sole_to_robot
            .translation()
            .inner
            .cast(),
        robot_kinematics
            .right_leg
            .sole_to_robot
            .translation()
            .inner
            .cast(),
    )
}

fn robot_to_left_camera(camera_matrix: &CameraMatrix) -> nalgebra::Isometry3<f32> {
    robot_to_camera(camera_matrix).inner
}

fn robot_to_camera(camera_matrix: &CameraMatrix) -> Isometry3<Robot, Camera> {
    camera_matrix.head_to_camera * camera_matrix.robot_to_head
}

#[cfg(test)]
mod tests {
    use coordinate_systems::{Camera, Head, LeftSole, RightSole};
    use geometry::rectangle::Rectangle;
    use types::bounding_box::BoundingBox;

    use super::*;

    #[test]
    fn goalpost_detection_uses_pixel_bottom_center() {
        let detections = vec![Object {
            label: RobocupObjectLabel::GoalPost,
            bounding_box: BoundingBox {
                area: Rectangle {
                    min: point![10.0, 20.0],
                    max: point![30.0, 50.0],
                },
                confidence: 1.0,
            },
        }];

        let goalposts = find_detected_goalposts(detections);

        assert_eq!(goalposts.len(), 1);
        assert_eq!(goalposts[0], point![20.0, 50.0]);
    }

    #[test]
    fn spot_detections_use_pixel_center() {
        let detections = vec![
            Object {
                label: RobocupObjectLabel::LSpot,
                bounding_box: BoundingBox {
                    area: Rectangle {
                        min: point![10.0, 20.0],
                        max: point![30.0, 50.0],
                    },
                    confidence: 1.0,
                },
            },
            Object {
                label: RobocupObjectLabel::TSpot,
                bounding_box: BoundingBox {
                    area: Rectangle {
                        min: point![40.0, 60.0],
                        max: point![60.0, 80.0],
                    },
                    confidence: 1.0,
                },
            },
            Object {
                label: RobocupObjectLabel::PenaltySpot,
                bounding_box: BoundingBox {
                    area: Rectangle {
                        min: point![70.0, 90.0],
                        max: point![90.0, 110.0],
                    },
                    confidence: 1.0,
                },
            },
        ];

        let features = find_detected_visual_features(detections);

        assert_eq!(features.l_spots, vec![point![20.0, 35.0]]);
        assert_eq!(features.t_spots, vec![point![50.0, 70.0]]);
        assert_eq!(features.penalty_spots, vec![point![80.0, 100.0]]);
    }

    #[test]
    fn initial_state_from_camera_matrix_uses_live_camera_geometry() {
        let robot_to_ground_rotation = nalgebra::UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3);
        let robot_to_ground = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(1.0, 2.0, 0.42),
            robot_to_ground_rotation,
        );
        let camera_matrix = CameraMatrix {
            ground_to_robot: robot_to_ground.inverse().framed_transform(),
            intrinsics: Intrinsic::new(nalgebra::vector![216.0, 217.0], point![251.0, 235.0]),
            ..Default::default()
        };

        let initial_state = initial_state_from_camera_matrix(&camera_matrix);

        assert!(initial_state.pose.xyz().x.abs() < 1.0e-9);
        assert!(initial_state.pose.xyz().y.abs() < 1.0e-9);
        assert!((initial_state.pose.xyz().z - 0.42).abs() < 1.0e-6);
        assert!(initial_state.pose.uvw().norm() < 1.0e-9);
        assert_eq!(
            initial_state.camera_intrinsics.focals(),
            nalgebra::vector![216.0, 217.0]
        );
        assert_eq!(
            initial_state.camera_intrinsics.optical_center(),
            nalgebra::vector![251.0, 235.0]
        );
        assert!((initial_state.pose.rot().w() - robot_to_ground_rotation.w as f64).abs() < 1.0e-9);
        assert!((initial_state.pose.rot().x() - robot_to_ground_rotation.i as f64).abs() < 1.0e-9);
        assert!((initial_state.pose.rot().y() - robot_to_ground_rotation.j as f64).abs() < 1.0e-9);
        assert!((initial_state.pose.rot().z() - robot_to_ground_rotation.k as f64).abs() < 1.0e-9);
    }

    #[test]
    fn intrinsic_from_camera_intrinsics_casts_solver_intrinsics() {
        let camera_intrinsics = CameraIntrinsics::new(
            nalgebra::vector![216.5, 217.5],
            nalgebra::vector![251.25, 235.75],
        );

        let intrinsic = intrinsic_from_camera_intrinsics(&camera_intrinsics);

        assert_eq!(intrinsic.focals, nalgebra::vector![216.5, 217.5]);
        assert_eq!(intrinsic.optical_center, point![251.25, 235.75]);
    }

    #[test]
    fn localization_publisher_outputs_field_to_robot() {
        let robot_to_field = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(-3.0, 0.25, 0.4),
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3),
        );

        let field_to_robot = localization_transform_from_backend_pose(&robot_to_field);
        let roundtrip_robot_to_field = field_to_robot.inverse().inner.cast::<f64>();

        assert!(
            (roundtrip_robot_to_field.translation.vector - robot_to_field.translation.vector)
                .norm()
                < 1.0e-6
        );
        assert!(
            roundtrip_robot_to_field
                .rotation
                .angle_to(&robot_to_field.rotation)
                < 1.0e-6
        );
    }

    #[test]
    fn visual_odometry_extrinsic_uses_head_and_camera_transforms() {
        let robot_to_head: Isometry3<Robot, Head> =
            nalgebra::Isometry3::translation(1.0, 2.0, 3.0).framed_transform();
        let head_to_camera: Isometry3<Head, Camera> =
            nalgebra::Isometry3::translation(0.5, 0.0, -0.25).framed_transform();
        let camera_matrix = CameraMatrix {
            robot_to_head,
            head_to_camera,
            ..Default::default()
        };

        let robot_to_camera = robot_to_left_camera(&camera_matrix);
        let expected = (head_to_camera * robot_to_head).inner;

        assert!((robot_to_camera.translation.vector - expected.translation.vector).norm() < 1.0e-6);
        assert!(robot_to_camera.rotation.angle_to(&expected.rotation) < 1.0e-6);
    }

    #[test]
    fn foot_height_points_use_sole_positions_in_robot_frame() {
        let left_sole_to_robot: Isometry3<LeftSole, Robot> =
            nalgebra::Isometry3::translation(0.1, 0.2, -0.3).framed_transform();
        let right_sole_to_robot: Isometry3<RightSole, Robot> =
            nalgebra::Isometry3::translation(0.4, -0.5, -0.6).framed_transform();
        let robot_kinematics = RobotKinematics {
            left_leg: kinematics::robot_kinematics::RobotLeftLegKinematics {
                sole_to_robot: left_sole_to_robot,
                ..Default::default()
            },
            right_leg: kinematics::robot_kinematics::RobotRightLegKinematics {
                sole_to_robot: right_sole_to_robot,
                ..Default::default()
            },
            ..Default::default()
        };

        let (left, right) = foot_height_points(&robot_kinematics);

        assert!((left - nalgebra::Point3::new(0.1, 0.2, -0.3)).norm() < 1.0e-6);
        assert!((right - nalgebra::Point3::new(0.4, -0.5, -0.6)).norm() < 1.0e-6);
    }
}
