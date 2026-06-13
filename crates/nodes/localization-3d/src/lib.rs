use std::{pin::Pin, sync::Arc, time::Duration};

use booster::ImuState;
use color_eyre::{
    Result,
    eyre::{Context as _, bail},
};
use coordinate_systems::{Field, Pixel, Robot};
use linear_algebra::{IntoTransform, Isometry3, Point2, point};
use localization_factrs::{
    BackendConfiguration, CameraIntrinsics, InitialState, LandmarkAssociationCosts, VinsFrontend,
    VinsFrontendError, VisualClassMeasurement, initialize,
};
use nalgebra::{Matrix2, Matrix3, Point3, Vector3};
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{
    cache::Cache,
    context::Context,
    qos::{QosDurability, QosProfile},
    time::Time,
};
use tokio::select;
use types::{
    field_dimensions::{FieldDimensions, Half, Side},
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
};
const GOALPOST_UNMATCHED_LANDMARK_COST: f64 = 0.0;
const GOALPOST_UNMATCHED_DETECTION_COST: f64 = 25.0;

pub fn backend_configuration() -> BackendConfiguration {
    BackendConfiguration {
        knot_spacing: Duration::from_millis(200),
        max_optimization_window: Duration::from_secs(3),
        optimizer_max_iterations: 1,
        gyroscope_noise: Matrix3::identity() * 0.01,
        accelerometer_noise: Matrix3::identity() * 0.05,
        gyroscope_process_noise: Matrix3::identity() * 0.01,
        accelerometer_process_noise: Matrix3::identity() * 0.01,
        visual_feature_noise: Matrix2::identity() * 5.0,
        gravity: Vector3::new(0.0, 0.0, 9.81),
    }
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

pub async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("localization3d").build().await?;

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

    let initial_state = wait_for_initial_state(&camera_matrix_cache).await;
    let (mut frontend, backend) = initialize(backend_configuration(), initial_state);
    let mut backend_handle = std::pin::pin!(tokio::task::spawn_blocking(|| backend.run_loop()));

    loop {
        select! {
            // TODO(oleflb): Use the correct image timestamp, not the source time
            objects = object_subscriber.recv_with_metadata() => {
                let objects = objects?;
                let visual_features = find_detected_visual_features(objects.message);
                let Some(camera_matrix) = camera_matrix_cache.get_nearest(objects.source_time) else {
                    continue;
                };
                let Some(field_dimensions) = field_dimensions_cache.get_nearest(objects.source_time) else {
                    continue;
                };

                ingest_visual_features(&mut frontend, objects.source_time, visual_features, &camera_matrix.inner, field_dimensions)
                    .wrap_err("ingest of visual features failed")?;
            }
            // TODO(oleflb): Use the correct sensor timestamp, not the source time
            imu = imu_subscriber.recv_with_metadata() => {
                let imu = imu?;
                frontend.ingest_imu(imu.source_time.to_wallclock(), imu.message)
                    .wrap_err("failed to ingest imu measurement into frontend")?;

                // let transform = frontend
                //     .last_optimization_result()
                //     .map(|result| result.transform.cast::<f32>().framed_transform());
                // localization_publisher.publish(&transform).await?;
            }
            result = &mut backend_handle => {
                result.wrap_err("failed to join")?.wrap_err("solver failed")?;
                bail!("solver stopped unexpectedly")
            }
            result = frontend.wait_for_backend_optimization_result() => {
                result?;
                let result = frontend.last_backend_optimization_result();
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

pub fn ingest_visual_features(
    frontend: &mut VinsFrontend,
    time: Time,
    visual_features: DetectedVisualFeatures,
    camera_matrix: &CameraMatrix,
    field_dimensions: Arc<FieldDimensions>,
) -> Result<(), VinsFrontendError> {
    let robot_to_camera = camera_matrix.head_to_camera * camera_matrix.robot_to_head;
    let mut classes = Vec::new();

    if let Some(class) = visual_class_measurement(
        visual_features.goalposts,
        goalpost_candidate_positions(&field_dimensions),
    ) {
        classes.push(class);
    }
    if let Some(class) = visual_class_measurement(
        visual_features.l_spots,
        l_spot_candidate_positions(&field_dimensions),
    ) {
        classes.push(class);
    }
    if let Some(class) = visual_class_measurement(
        visual_features.t_spots,
        t_spot_candidate_positions(&field_dimensions),
    ) {
        classes.push(class);
    }

    frontend.ingest_visual_classes(time.to_wallclock(), classes, robot_to_camera.inner)
}

fn visual_class_measurement(
    detections: Vec<Point2<Pixel>>,
    candidates: Vec<Point3<f64>>,
) -> Option<VisualClassMeasurement> {
    if detections.is_empty() {
        return None;
    }

    Some(VisualClassMeasurement {
        detections: detections
            .into_iter()
            .map(|detection| detection.inner.cast())
            .collect(),
        candidates,
        association_costs: Some(LandmarkAssociationCosts {
            unmatched_landmark: GOALPOST_UNMATCHED_LANDMARK_COST,
            unmatched_detection: GOALPOST_UNMATCHED_DETECTION_COST,
        }),
    })
}

fn goalpost_candidate_positions(field_dimensions: &FieldDimensions) -> Vec<Point3<f64>> {
    [Half::Opponent, Half::Own]
        .into_iter()
        .flat_map(|half| {
            [Side::Left, Side::Right]
                .into_iter()
                .map(move |side| field_candidate(field_dimensions.goal_post(half, side)))
        })
        .collect()
}

fn l_spot_candidate_positions(field_dimensions: &FieldDimensions) -> Vec<Point3<f64>> {
    [Half::Opponent, Half::Own]
        .into_iter()
        .flat_map(|half| {
            [Side::Left, Side::Right].into_iter().flat_map(move |side| {
                [
                    field_dimensions.corner(half, side),
                    field_dimensions.goal_box_corner(half, side),
                    field_dimensions.penalty_box_corner(half, side),
                ]
                .into_iter()
                .map(field_candidate)
            })
        })
        .collect()
}

fn t_spot_candidate_positions(field_dimensions: &FieldDimensions) -> Vec<Point3<f64>> {
    let sideline_t_crossings = [Side::Left, Side::Right]
        .into_iter()
        .map(|side| field_candidate(field_dimensions.t_crossing(side)));
    let box_goal_line_intersections = [Half::Opponent, Half::Own].into_iter().flat_map(|half| {
        [Side::Left, Side::Right].into_iter().flat_map(move |side| {
            [
                field_dimensions.goal_box_goal_line_intersection(half, side),
                field_dimensions.penalty_box_goal_line_intersection(half, side),
            ]
            .into_iter()
            .map(field_candidate)
        })
    });

    sideline_t_crossings
        .chain(box_goal_line_intersections)
        .collect()
}

fn field_candidate(point: Point2<Field>) -> Point3<f64> {
    point.extend(0.0).inner.cast()
}

#[cfg(test)]
mod tests {
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
        ];

        let features = find_detected_visual_features(detections);

        assert_eq!(features.l_spots, vec![point![20.0, 35.0]]);
        assert_eq!(features.t_spots, vec![point![50.0, 70.0]]);
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
}
