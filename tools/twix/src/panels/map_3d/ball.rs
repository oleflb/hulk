use bevy::{
    asset::RenderAssetUsages,
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};

use super::{Settings, ViewerData};

#[derive(Component)]
pub(super) struct Ball;

pub(super) fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    let texture = match image::load_from_memory(include_bytes!(
        "../../../../mujoco-simulator/mujoco-simulator/K1/textures/football_base_color.png"
    )) {
        Ok(texture) => texture.to_rgba8(),
        Err(error) => {
            log::error!("failed to load football texture: {error}");
            return;
        }
    };
    let image = Image::new(
        Extent3d {
            width: texture.width(),
            height: texture.height(),
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        texture.into_raw(),
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );
    commands.spawn((
        Ball,
        Mesh3d(meshes.add(Sphere::new(1.0).mesh().uv(64, 32))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(images.add(image)),
            perceptual_roughness: 0.65,
            ..default()
        })),
        Transform::default(),
        Visibility::Hidden,
    ));
}

pub(super) fn update(
    data: Res<ViewerData>,
    settings: Res<Settings>,
    mut balls: Query<(&mut Transform, &mut Visibility), With<Ball>>,
) {
    for (mut transform, mut visibility) in &mut balls {
        let ball = data.ball.as_ref().filter(|_| settings.ball);
        let dimensions = data.field_dimensions.as_ref().map(|record| &record.value);
        match ball.zip(dimensions).filter(|(ball, dimensions)| {
            dimensions.ball_radius.is_finite()
                && dimensions.ball_radius > 0.0
                && ball.ball_in_field.x().is_finite()
                && ball.ball_in_field.y().is_finite()
        }) {
            Some((ball, dimensions)) => {
                *transform = Transform::from_xyz(
                    ball.ball_in_field.x(),
                    dimensions.ball_radius,
                    -ball.ball_in_field.y(),
                )
                .with_scale(Vec3::splat(dimensions.ball_radius));
                *visibility = Visibility::Visible;
            }
            None => *visibility = Visibility::Hidden,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::panels::map_observation::tests::sample;
    use ros_z_debug::query::decode_record;

    #[test]
    fn football_uses_recorded_field_position_and_radius_and_clears_when_absent() {
        use coordinate_systems::Ground;
        use linear_algebra::point;
        use types::{
            field_dimensions::{FieldDimensions, Side},
            world_state::{BallSource, BallState},
        };

        let mut dimensions = FieldDimensions::SPL_2025;
        dimensions.ball_radius = 0.11;
        let sample = sample("field_dimensions", 1, 10, &dimensions);
        let dimensions = decode_record(&sample.record, &sample.topic).unwrap();
        let mut app = App::new();
        app.init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .init_resource::<Assets<Image>>()
            .insert_resource(Settings::default())
            .insert_resource(ViewerData {
                field_dimensions: Some(dimensions),
                ball: Some(BallState {
                    ball_in_field: point![2.0, -3.0],
                    ball_in_ground: linear_algebra::Point2::<Ground>::origin(),
                    ball_in_ground_velocity: linear_algebra::Vector2::zeros(),
                    last_seen_ball: std::time::UNIX_EPOCH,
                    field_side: Side::Left,
                    source: BallSource::Own,
                }),
                ..default()
            })
            .add_systems(Startup, setup)
            .add_systems(Update, update);
        app.update();
        let (transform, visibility) = app
            .world_mut()
            .query_filtered::<(&Transform, &Visibility), With<Ball>>()
            .single(app.world())
            .unwrap();
        assert_eq!(transform.translation, Vec3::new(2.0, 0.11, 3.0));
        assert_eq!(transform.scale, Vec3::splat(0.11));
        assert_eq!(*visibility, Visibility::Visible);
        app.world_mut().resource_mut::<ViewerData>().ball = None;
        app.update();
        assert_eq!(
            *app.world_mut()
                .query_filtered::<&Visibility, With<Ball>>()
                .single(app.world())
                .unwrap(),
            Visibility::Hidden
        );
    }
}
