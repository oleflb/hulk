use bevy::{
    camera_controller::free_camera::{FreeCamera, FreeCameraPlugin},
    dev_tools::diagnostics_overlay::{DiagnosticsOverlay, DiagnosticsOverlayPlugin},
    diagnostic::FrameTimeDiagnosticsPlugin,
    prelude::*,
};

use crate::{bevy_mujoco::MujocoWorldPlugin, scene::field::FieldPlugin};

mod bevy_mujoco;
mod scene;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins((
            MujocoWorldPlugin,
            FieldPlugin,
            FreeCameraPlugin,
            FrameTimeDiagnosticsPlugin::default(),
            DiagnosticsOverlayPlugin,
        ))
        .add_systems(Startup, setup_scene)
        .run();
}

fn setup_scene(mut commands: Commands) {
    commands.spawn(DiagnosticsOverlay::fps());

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        FreeCamera {
            walk_speed: 3.0,
            run_speed: 10.0,
            ..Default::default()
        },
    ));

    commands.spawn((
        DirectionalLight::default(),
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
