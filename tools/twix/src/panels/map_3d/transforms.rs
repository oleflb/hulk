use super::{CameraMatrix, ViewerData};
use bevy::{asset::RenderAssetUsages, mesh::PrimitiveTopology, prelude::*};

pub(super) fn robot_to_display(data: &ViewerData) -> Option<nalgebra::Isometry3<f32>> {
    data.localization
        .as_ref()
        .map(|field_to_robot| field_to_robot.inverse().inner)
}

pub(super) fn robot_to_camera(camera_matrix: &CameraMatrix) -> nalgebra::Isometry3<f32> {
    (camera_matrix.head_to_camera * camera_matrix.robot_to_head).inner
}
pub(super) fn empty_mesh(topology: PrimitiveTopology) -> Mesh {
    let mut mesh = Mesh::new(topology, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new());
    mesh
}
pub(super) fn transform_from_isometry(isometry: nalgebra::Isometry3<f32>) -> Transform {
    Transform::from_translation(convert_point(isometry.translation.vector.into()))
        .with_rotation(convert_rotation(isometry.rotation))
}

fn convert_rotation(rotation: nalgebra::UnitQuaternion<f32>) -> Quat {
    let source = rotation.to_rotation_matrix();
    let source = source.matrix();
    let source = Mat3::from_cols(
        Vec3::new(source[(0, 0)], source[(1, 0)], source[(2, 0)]),
        Vec3::new(source[(0, 1)], source[(1, 1)], source[(2, 1)]),
        Vec3::new(source[(0, 2)], source[(1, 2)], source[(2, 2)]),
    );
    let conversion = Mat3::from_cols(Vec3::X, Vec3::NEG_Z, Vec3::Y);

    Quat::from_mat3(&(conversion * source * conversion.transpose()))
}

pub(super) fn convert_point([x, y, z]: [f32; 3]) -> Vec3 {
    Vec3::new(x, z, -y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basis_conversion_preserves_transformed_points() {
        let pose = nalgebra::Isometry3::new(
            nalgebra::Vector3::new(1.0, 2.0, 3.0),
            nalgebra::Vector3::new(0.2, -0.4, 0.8),
        );
        let point = nalgebra::Point3::new(0.5, -2.0, 4.0);
        let expected = convert_point((pose * point).coords.into());
        let actual =
            transform_from_isometry(pose).transform_point(convert_point(point.coords.into()));
        assert!((actual - expected).length() < 1e-5);
        assert_eq!(convert_point([1.0, 2.0, 3.0]), Vec3::new(1.0, 3.0, -2.0));
    }

    #[test]
    fn localization_is_inverted() {
        let data = ViewerData {
            localization: Some(linear_algebra::Isometry3::wrap(
                nalgebra::Isometry3::translation(1.0, 2.0, 3.0),
            )),
            ..Default::default()
        };
        assert_eq!(
            robot_to_display(&data).unwrap().translation.vector,
            nalgebra::Vector3::new(-1.0, -2.0, -3.0)
        );
    }

    #[test]
    fn missing_pose_is_not_replaced_by_the_field_origin() {
        let data = ViewerData::default();
        assert!(robot_to_display(&data).is_none());
    }
}
