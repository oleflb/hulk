use crate::trajectory::fixed_robot_to_camera;
use coordinate_systems::{Camera, Field, Ground, Head, Robot};
use linear_algebra::{Framed, IntoTransform, Isometry3 as FramedIsometry3};
use nalgebra::{Isometry3, Point2, Translation3, UnitQuaternion};
use projection::camera_matrix::CameraMatrix;

pub fn camera_matrix(robot_to_field: &FramedIsometry3<Robot, Field>) -> CameraMatrix {
    let (_, _, yaw) = robot_to_field.inner.rotation.euler_angles();
    let ground_to_field: FramedIsometry3<Ground, Field> = Isometry3::from_parts(
        Translation3::new(
            robot_to_field.translation().x(),
            robot_to_field.translation().y(),
            0.0,
        ),
        UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
    )
    .framed_transform();
    let ground_to_robot: FramedIsometry3<Ground, Robot> =
        robot_to_field.inverse() * ground_to_field;
    let robot_to_head: FramedIsometry3<Robot, Head> = Isometry3::identity().framed_transform();
    let head_to_camera: FramedIsometry3<Head, Camera> =
        fixed_robot_to_camera() * robot_to_head.inverse();
    CameraMatrix::from_normalized_focal_and_center(
        nalgebra::vector![0.625, 0.625],
        Point2::new(0.5, 0.5),
        Framed::wrap(nalgebra::vector![640.0, 480.0]),
        ground_to_robot,
        robot_to_head,
        head_to_camera,
    )
}
