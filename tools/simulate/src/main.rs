use std::{path::PathBuf, time::Duration};

use bevy::{
    asset::AssetPlugin,
    camera_controller::free_camera::{FreeCamera, FreeCameraPlugin},
    dev_tools::diagnostics_overlay::{DiagnosticsOverlay, DiagnosticsOverlayPlugin},
    diagnostic::FrameTimeDiagnosticsPlugin,
    picking::mesh_picking::{MeshPickingCamera, MeshPickingPlugin, MeshPickingSettings},
    prelude::*,
};
use clap::Parser;
use color_eyre::{Result, eyre::Context as _};
use ros_z::prelude::*;

use crate::{
    bevy_mujoco::MujocoWorldPlugin,
    parameters::{SimulatorParameters, SimulatorParametersPlugin},
    scene::{
        ObjectsPlugin,
        field::FieldPlugin,
        palette::{ObjectPalettePlugin, WorldCamera},
    },
};

mod bevy_mujoco;
mod parameters;
mod scene;

const RUNTIME_SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, value_name = "DIRECTORY")]
    parameter_root: Option<PathBuf>,
    #[arg(long)]
    router: Option<String>,
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();
    let parameter_root = args
        .parameter_root
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("parameters"));

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .wrap_err("failed to build Tokio runtime")?;
    let (context, node, simulator_parameters) = runtime.block_on(async {
        let mut builder = ContextBuilder::default()
            .with_namespace("/simulator")
            .with_parameter_layers([parameter_root]);
        builder = match args.router {
            Some(router) => builder.with_mode("client").with_router_endpoint(router)?,
            None => builder
                .with_mode("router")
                .disable_multicast_scouting()
                .with_connect_endpoints(std::iter::empty::<&str>())
                .with_listen_endpoints(["tcp/127.0.0.1:7447"]),
        };

        let context = builder.build().await?;
        let node = context.create_node("parameters").build().await?;
        let simulator_parameters = node.bind_parameter_as::<SimulatorParameters>("simulator")?;
        simulator_parameters.add_validation_hook(SimulatorParameters::validate)?;

        Ok::<_, color_eyre::Report>((context, node, simulator_parameters))
    })?;

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        file_path: format!("{}/assets", env!("CARGO_MANIFEST_DIR")),
        ..default()
    }))
    .add_plugins(MeshPickingPlugin)
    .add_plugins(SimulatorParametersPlugin::new(simulator_parameters.clone()))
    .insert_resource(MeshPickingSettings {
        require_markers: true,
        ..default()
    })
    .add_plugins((
        MujocoWorldPlugin,
        FieldPlugin,
        ObjectsPlugin,
        ObjectPalettePlugin,
        FreeCameraPlugin,
        FrameTimeDiagnosticsPlugin::default(),
        DiagnosticsOverlayPlugin,
    ))
    .add_systems(Startup, setup_scene);
    app.run();

    drop(app);
    let shutdown_result = context
        .shutdown()
        .wrap_err("failed to shut down ROS-Z context");
    drop(simulator_parameters);
    drop(node);
    drop(context);
    runtime.shutdown_timeout(RUNTIME_SHUTDOWN_TIMEOUT);
    shutdown_result
}

fn setup_scene(mut commands: Commands) {
    commands.spawn(DiagnosticsOverlay::fps());

    commands.spawn((
        Camera3d::default(),
        WorldCamera,
        MeshPickingCamera,
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
