use std::{collections::HashSet, sync::Arc};

use eframe::{
    egui::{self, epaint::Primitive},
    egui_wgpu::{RenderState, WgpuConfiguration, WgpuSetup},
    wgpu,
};

use super::Map3DPanel;
use crate::{
    backend::RobotBackend,
    panel::{Panel, PanelCreationContext, PanelUiContext},
};

async fn headless_render_state() -> RenderState {
    let mut config = WgpuConfiguration::default();
    let WgpuSetup::CreateNew(setup) = &mut config.wgpu_setup else {
        panic!("expected default WGPU device creation");
    };
    // Match Twix's Bevy device limit.
    let previous = setup.device_descriptor.clone();
    setup.device_descriptor = Arc::new(move |adapter| {
        let mut descriptor = previous(adapter);
        descriptor
            .required_limits
            .max_storage_buffers_per_shader_stage = 9;
        descriptor
    });
    let instance = config.wgpu_setup.new_instance().await;
    let render_state = RenderState::create(&config, &instance, None, Default::default())
        .await
        .expect("headless WGPU adapter/device creation failed");
    eprintln!("GPU smoke adapter: {:?}", render_state.adapter.get_info());
    render_state
}

// Run: cargo test -p twix gpu_multiple_tabs -- --ignored --nocapture
#[test]
#[ignore = "requires a working WGPU adapter"]
fn gpu_multiple_tabs() {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(1)
        .enable_all()
        .build()
        .unwrap();
    runtime.block_on(async {
        let render_state = headless_render_state().await;
        let backend = Arc::new(
            RobotBackend::new(runtime.handle().clone(), None, "/".to_string())
                .await
                .unwrap(),
        );
        let context = egui::Context::default();
        let new_panel = || {
            let panel = Map3DPanel::new(PanelCreationContext {
                backend: backend.clone(),
                value: None,
                egui_context: context.clone(),
                render_state: Some(render_state.clone()),
            });
            assert!(panel.widget.is_some());
            assert!(panel.observations.is_ok());
            panel
        };
        let mut panels = [Some(new_panel()), Some(new_panel())];
        for (frame, size) in [
            [800.0, 600.0],
            [1280.0, 900.0],
            [640.0, 480.0],
            [1280.0, 900.0],
            [800.0, 600.0],
        ]
        .into_iter()
        .enumerate()
        {
            if frame == 2 {
                drop(panels[0].take());
            } else if frame == 3 {
                panels[0] = Some(new_panel());
            }
            let output = context.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(size[0], size[1]),
                    )),
                    ..Default::default()
                },
                |ui| {
                    ui.columns(2, |columns| {
                        for (panel, ui) in panels.iter_mut().zip(columns) {
                            if let Some(panel) = panel {
                                panel.ui(
                                    ui,
                                    PanelUiContext {
                                        backend: &backend,
                                        egui_context: &context,
                                    },
                                );
                            }
                        }
                    });
                },
            );
            let textures: HashSet<_> = context
                .tessellate(output.shapes, output.pixels_per_point)
                .into_iter()
                .filter_map(|primitive| match primitive.primitive {
                    Primitive::Mesh(mesh)
                        if matches!(mesh.texture_id, egui::TextureId::User(_)) =>
                    {
                        Some(mesh.texture_id)
                    }
                    _ => None,
                })
                .collect();
            assert_eq!(textures.len(), panels.iter().flatten().count());
            for texture in textures {
                assert!(render_state.renderer.read().texture(&texture).is_some());
            }
            render_state
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
        }
        drop(panels);
        render_state
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
    });
}

#[test]
#[ignore = "requires a working WGPU adapter"]
fn gpu_camera_image_is_transparent_and_updates_after_resize() {
    use super::{CameraMatrix, Settings, ViewerData, camera};
    use crate::panels::map_observation::tests::sample;
    use bevy::{
        camera::RenderTarget,
        prelude::*,
        render::{
            gpu_readback::{Readback, ReadbackComplete},
            render_resource::{TextureFormat, TextureUsages},
        },
        transform::TransformSystems,
    };
    use egui_bevy::BevyWidget;
    use ros_z_debug::query::decode_record;
    use ros2::sensor_msgs::image::Image as RosImage;

    #[derive(Default, Resource)]
    struct Pixels(Vec<u8>);

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(1)
        .enable_all()
        .build()
        .unwrap();
    runtime.block_on(async {
        let render_state = headless_render_state().await;
        let mut widget = BevyWidget::new(render_state.clone());
        let app = &mut widget.bevy_app;
        let mut target = Image::new_target_texture(512, 512, TextureFormat::Rgba8UnormSrgb, None);
        target.texture_descriptor.usage |= TextureUsages::COPY_SRC;
        let target = app.world_mut().resource_mut::<Assets<Image>>().add(target);
        let matrix = CameraMatrix {
            image_size: linear_algebra::vector![8.0, 8.0],
            intrinsics: projection::intrinsic::Intrinsic::new(
                nalgebra::vector![8.0, 8.0],
                linear_algebra::point![4.0, 4.0],
            ),
            ..default()
        };
        app.insert_resource(Settings::default())
            .insert_resource(ViewerData {
                localization: Some(linear_algebra::Isometry3::identity()),
                camera_matrix: Some(matrix),
                ..default()
            })
            .init_resource::<Pixels>()
            .add_systems(Startup, camera::setup)
            .add_systems(
                Update,
                (camera::update_camera_image, camera::update_camera_viewport).chain(),
            )
            .add_systems(
                PostUpdate,
                (move |mut cameras: Query<
                    (&mut Transform, &mut RenderTarget, &mut Camera),
                    With<Camera3d>,
                >, mut frustums: Query<&mut Visibility, With<camera::CameraFrustum>>| {
                    for (mut transform, mut render_target, mut camera) in &mut cameras {
                        *transform =
                            Transform::from_xyz(0.0, 3.0, 0.0).looking_at(Vec3::Y, Vec3::Z);
                        *render_target = RenderTarget::Image(target.clone().into());
                        camera.clear_color = Color::srgb(0.0, 0.0, 1.0).into();
                    }
                    for mut visibility in &mut frustums {
                        *visibility = Visibility::Hidden;
                    }
                })
                .before(TransformSystems::Propagate),
            );
        app.finish();
        app.cleanup();

        // Read actual rendered pixels, not just whether the output texture exists.
        app.update();
        let camera = app
            .world_mut()
            .query_filtered::<Entity, With<Camera3d>>()
            .single(app.world())
            .unwrap();
        app.world_mut()
            .entity_mut(camera)
            .insert(bevy::core_pipeline::tonemapping::Tonemapping::None);
        let render_target = app
            .world_mut()
            .query_filtered::<&RenderTarget, With<Camera3d>>()
            .single(app.world())
            .unwrap()
            .clone();
        let RenderTarget::Image(target) = render_target else {
            panic!("expected image target")
        };
        app.world_mut()
            .spawn(Readback::texture(target.handle))
            .observe(|event: On<ReadbackComplete>, mut pixels: ResMut<Pixels>| {
                pixels.0 = event.data.clone()
            });
        let render = |app: &mut App| {
            for _ in 0..8 {
                app.update();
                render_state
                    .device
                    .poll(wgpu::PollType::wait_indefinitely())
                    .unwrap();
            }
            let pixels = &app.world().resource::<Pixels>().0;
            assert_eq!(pixels.len(), 512 * 512 * 4);
            let offset = (256 * 512 + 256) * 4;
            <[u8; 4]>::try_from(&pixels[offset..offset + 4]).unwrap()
        };
        let background = render(app);
        eprintln!("camera image background: {background:?}");
        assert!(background[2] > 200 && background[0] < 20);

        for (id, width, color) in [
            (1, 8, [255, 0, 0]),
            (2, 8, [0, 255, 0]),
            (3, 16, [255, 255, 0]),
        ] {
            let image = RosImage {
                width,
                height: 8,
                step: width * 3,
                encoding: "rgb8".into(),
                data: color.repeat((width * 8) as usize).into(),
                ..default()
            };
            let image = sample("/image", id, id as i64, &image);
            app.world_mut().resource_mut::<ViewerData>().camera_frame =
                Some(decode_record(&image.record, &image.topic).unwrap());
            let pixel = render(app);
            eprintln!("camera image {id}, width {width}: {pixel:?}");
            // The blue background must remain visible through the 50%-opaque image.
            assert!(pixel[2] > 40, "image must remain transparent: {pixel:?}");
            for channel in 0..2 {
                if color[channel] > 0 {
                    assert!(
                        pixel[channel] > 40,
                        "camera image channel {channel} is missing: {pixel:?}"
                    );
                } else {
                    assert!(
                        pixel[channel] < 30,
                        "camera image did not update: {pixel:?}"
                    );
                }
            }
        }
    });
}
