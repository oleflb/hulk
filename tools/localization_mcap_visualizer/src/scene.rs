use std::sync::Arc;

use bevy::{
    asset::RenderAssetUsages,
    mesh::{Indices, PrimitiveTopology},
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};
use bevy_panorbit_camera::PanOrbitCamera;
use coordinate_systems::{Field, Robot};
use localization_3d::GlobalLocalizationDetailedDebug;
use projection::camera_matrix::CameraMatrix;
use types::field_dimensions::FieldDimensions;

use crate::mcap_recording::{CameraImage, TrajectoryPoint};

const CAMERA_VIEWPORT_DEPTH: f32 = 1.0;

pub fn configure(app: &mut App) {
    app.insert_resource(SceneData::default())
        .insert_resource(GlobalAmbientLight {
            color: Color::WHITE,
            brightness: 600.0,
            ..default()
        })
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                update_field_plane,
                update_field_markings,
                configure_view_camera_once,
                update_robot_marker,
                update_camera_viewport,
                update_camera_image,
                update_trajectories,
                update_global_debug_lines,
            ),
        );
}

#[derive(Clone, Default, Resource)]
pub struct SceneData {
    field_dimensions: FieldDimensions,
    field_dimensions_version: SceneVersion,
    current_robot_to_field: Option<linear_algebra::Isometry3<Robot, Field>>,
    camera_matrix: Option<CameraMatrix>,
    camera_version: SceneVersion,
    camera_frame: Option<SceneCameraFrame>,
    recorded_trajectory: Vec<TrajectoryPoint>,
    recorded_trajectory_version: SceneVersion,
    resolved_trajectory: Vec<TrajectoryPoint>,
    resolved_trajectory_version: SceneVersion,
    global_debug: Option<Arc<GlobalLocalizationDetailedDebug>>,
    global_debug_version: SceneVersion,
}

impl SceneData {
    pub fn camera_frame_sequence(&self) -> Option<SceneFrameSequence> {
        self.camera_frame.as_ref().map(|frame| frame.sequence)
    }

    pub fn field_dimensions_version(&self) -> SceneVersion {
        self.field_dimensions_version
    }

    pub fn camera_version(&self) -> SceneVersion {
        self.camera_version
    }

    pub fn recorded_trajectory_version(&self) -> SceneVersion {
        self.recorded_trajectory_version
    }

    pub fn resolved_trajectory_version(&self) -> SceneVersion {
        self.resolved_trajectory_version
    }

    pub fn global_debug_version(&self) -> SceneVersion {
        self.global_debug_version
    }

    pub fn set_field_dimensions(&mut self, field_dimensions: FieldDimensions) {
        self.field_dimensions = field_dimensions;
        self.field_dimensions_version = self.field_dimensions_version.next();
    }

    pub fn set_current_robot_to_field(
        &mut self,
        current_robot_to_field: Option<linear_algebra::Isometry3<Robot, Field>>,
    ) {
        self.current_robot_to_field = current_robot_to_field;
    }

    pub fn set_camera_matrix(&mut self, camera_matrix: Option<CameraMatrix>) {
        self.camera_matrix = camera_matrix;
        self.camera_version = self.camera_version.next();
    }

    pub fn set_camera_frame(&mut self, camera_frame: Option<SceneCameraFrame>) {
        self.camera_frame = camera_frame;
    }

    pub fn set_recorded_trajectory(&mut self, trajectory: Vec<TrajectoryPoint>) {
        self.recorded_trajectory = trajectory;
        self.recorded_trajectory_version = self.recorded_trajectory_version.next();
    }

    pub fn set_resolved_trajectory(&mut self, trajectory: Vec<TrajectoryPoint>) {
        self.resolved_trajectory = trajectory;
        self.resolved_trajectory_version = self.resolved_trajectory_version.next();
    }

    pub fn set_global_debug(&mut self, global_debug: Option<Arc<GlobalLocalizationDetailedDebug>>) {
        self.global_debug = global_debug;
        self.global_debug_version = self.global_debug_version.next();
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SceneVersion(u64);

impl SceneVersion {
    pub const READY: Self = Self(1);

    pub fn next(self) -> Self {
        Self(self.0.wrapping_add(1))
    }
}

#[derive(Clone)]
pub struct SceneCameraFrame {
    pub sequence: SceneFrameSequence,
    pub image: CameraImage,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SceneFrameSequence(u64);

impl SceneFrameSequence {
    pub fn stereo(stereo_sequence: u64, side: SceneCameraSide) -> Self {
        let side_offset = match side {
            SceneCameraSide::Left => 0,
        };
        Self(stereo_sequence * 2 + side_offset)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SceneCameraSide {
    Left,
}

#[derive(Component)]
struct FieldPlane;

#[derive(Component)]
struct FieldMarkings;

#[derive(Component)]
struct RobotMarker;

#[derive(Component)]
struct CameraFrustum;

#[derive(Component)]
struct CameraImagePlane {
    texture: Handle<Image>,
    sequence: Option<SceneFrameSequence>,
}

#[derive(Component)]
struct RecordedTrajectory;

#[derive(Component)]
struct ResolvedTrajectory;

#[derive(Component)]
struct GlobalDebugLines;

type CameraMeshQuery<'world, 'state, Filter> = Query<
    'world,
    'state,
    (
        &'static Mesh3d,
        &'static mut Transform,
        &'static mut Visibility,
    ),
    Filter,
>;
type CameraFrustumQuery<'world, 'state> = CameraMeshQuery<'world, 'state, With<CameraFrustum>>;
type CameraImagePlaneQuery<'world, 'state> =
    CameraMeshQuery<'world, 'state, (With<CameraImagePlane>, Without<CameraFrustum>)>;

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    commands.spawn((
        PointLight {
            intensity: 2_500.0,
            range: 14.0,
            ..default()
        },
        Transform::from_xyz(0.0, 5.0, 0.0),
    ));

    let field_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.04, 0.34, 0.13),
        perceptual_roughness: 0.95,
        ..default()
    });
    let markings_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        cull_mode: None,
        ..default()
    });
    let robot_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.3, 0.65, 1.0),
        unlit: true,
        cull_mode: None,
        ..default()
    });
    let frustum_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.85, 0.15),
        unlit: true,
        ..default()
    });
    let recorded_trajectory_material = materials.add(StandardMaterial {
        base_color: Color::srgba(0.7, 0.7, 0.7, 0.8),
        unlit: true,
        ..default()
    });
    let resolved_trajectory_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.1, 1.0, 0.45),
        unlit: true,
        ..default()
    });
    let debug_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.25, 0.75),
        unlit: true,
        ..default()
    });
    let camera_image_texture = images.add(Image::transparent());
    let camera_image_material = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 1.0, 1.0, 0.45),
        base_color_texture: Some(camera_image_texture.clone()),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        cull_mode: None,
        ..default()
    });

    commands.spawn((
        FieldPlane,
        Mesh3d(meshes.add(field_mesh())),
        MeshMaterial3d(field_material),
        Transform::default(),
    ));
    commands.spawn((
        FieldMarkings,
        Mesh3d(meshes.add(field_markings_mesh(&FieldDimensions::SPL_2025))),
        MeshMaterial3d(markings_material),
        Transform::default(),
    ));
    commands.spawn((
        RobotMarker,
        Mesh3d(meshes.add(robot_marker_mesh())),
        MeshMaterial3d(robot_material),
        Transform::default(),
        Visibility::Hidden,
    ));
    commands.spawn((
        CameraFrustum,
        Mesh3d(meshes.add(empty_mesh(PrimitiveTopology::LineList))),
        MeshMaterial3d(frustum_material),
        Transform::default(),
        Visibility::Hidden,
    ));
    commands.spawn((
        CameraImagePlane {
            texture: camera_image_texture,
            sequence: None,
        },
        Mesh3d(meshes.add(empty_mesh(PrimitiveTopology::TriangleList))),
        MeshMaterial3d(camera_image_material),
        Transform::default(),
        Visibility::Hidden,
    ));
    commands.spawn((
        RecordedTrajectory,
        Mesh3d(meshes.add(empty_mesh(PrimitiveTopology::LineList))),
        MeshMaterial3d(recorded_trajectory_material),
        Transform::default(),
    ));
    commands.spawn((
        ResolvedTrajectory,
        Mesh3d(meshes.add(empty_mesh(PrimitiveTopology::LineList))),
        MeshMaterial3d(resolved_trajectory_material),
        Transform::default(),
    ));
    commands.spawn((
        GlobalDebugLines,
        Mesh3d(meshes.add(empty_mesh(PrimitiveTopology::LineList))),
        MeshMaterial3d(debug_material),
        Transform::default(),
    ));
}

fn configure_view_camera_once(
    mut positioned: Local<bool>,
    mut cameras: Query<(&mut Transform, &mut PanOrbitCamera), With<Camera3d>>,
) {
    if *positioned {
        return;
    }
    for (mut transform, mut pan_orbit) in &mut cameras {
        *transform = Transform::from_xyz(4.0, 6.0, 7.0).looking_at(Vec3::ZERO, Vec3::Y);
        pan_orbit.focus = Vec3::ZERO;
        pan_orbit.target_focus = Vec3::ZERO;
        pan_orbit.radius = None;
        pan_orbit.target_radius = 10.0;
        pan_orbit.zoom_lower_limit = 1.0;
        pan_orbit.zoom_upper_limit = Some(35.0);
        pan_orbit.orbit_smoothness = 0.0;
        pan_orbit.pan_smoothness = 0.0;
        pan_orbit.zoom_smoothness = 0.0;
        *positioned = true;
    }
}

fn update_field_plane(
    data: Res<SceneData>,
    mut previous_version: Local<SceneVersion>,
    mut field: Query<&mut Transform, With<FieldPlane>>,
) {
    if *previous_version == data.field_dimensions_version {
        return;
    }
    let dimensions = data.field_dimensions;
    let length = dimensions.length + 2.0 * dimensions.border_strip_width;
    let width = dimensions.width + 2.0 * dimensions.border_strip_width;

    for mut transform in &mut field {
        transform.scale = Vec3::new(length, 1.0, width);
    }
    *previous_version = data.field_dimensions_version;
}

fn update_field_markings(
    data: Res<SceneData>,
    mut previous_version: Local<SceneVersion>,
    markings: Query<&Mesh3d, With<FieldMarkings>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if *previous_version == data.field_dimensions_version {
        return;
    }
    for markings in &markings {
        let _ = meshes.insert(markings.id(), field_markings_mesh(&data.field_dimensions));
    }
    *previous_version = data.field_dimensions_version;
}

fn update_robot_marker(
    data: Res<SceneData>,
    mut markers: Query<(&mut Transform, &mut Visibility), With<RobotMarker>>,
) {
    for (mut transform, mut visibility) in &mut markers {
        let Some(robot_to_field) = data.current_robot_to_field else {
            *visibility = Visibility::Hidden;
            continue;
        };
        *transform = transform_from_isometry(robot_to_field.inner);
        *visibility = Visibility::Visible;
    }
}

fn update_camera_viewport(
    data: Res<SceneData>,
    mut previous_version: Local<SceneVersion>,
    mut frustums: CameraFrustumQuery,
    mut image_planes: CameraImagePlaneQuery,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let Some(camera_matrix) = &data.camera_matrix else {
        for (_, _, mut visibility) in &mut frustums {
            *visibility = Visibility::Hidden;
        }
        for (_, _, mut visibility) in &mut image_planes {
            *visibility = Visibility::Hidden;
        }
        return;
    };
    let Some(robot_to_field) = data.current_robot_to_field else {
        return;
    };

    let transform = camera_to_field_transform(robot_to_field, camera_matrix);
    let geometry_changed = *previous_version != data.camera_version;
    for (mesh, mut entity_transform, mut visibility) in &mut frustums {
        *entity_transform = transform;
        *visibility = Visibility::Visible;
        if geometry_changed {
            let _ = meshes.insert(mesh.id(), camera_frustum_mesh(camera_matrix));
        }
    }
    for (mesh, mut entity_transform, mut visibility) in &mut image_planes {
        *entity_transform = transform;
        *visibility = Visibility::Visible;
        if geometry_changed {
            let _ = meshes.insert(mesh.id(), camera_image_plane_mesh(camera_matrix));
        }
    }
    *previous_version = data.camera_version;
}

fn update_camera_image(
    data: Res<SceneData>,
    mut image_planes: Query<&mut CameraImagePlane>,
    mut images: ResMut<Assets<Image>>,
) {
    let Some(frame) = &data.camera_frame else {
        return;
    };
    for mut image_plane in &mut image_planes {
        if image_plane.sequence == Some(frame.sequence) {
            continue;
        }
        let _ = images.insert(image_plane.texture.id(), camera_frame_image(&frame.image));
        image_plane.sequence = Some(frame.sequence);
    }
}

fn update_trajectories(
    data: Res<SceneData>,
    mut previous_versions: Local<(SceneVersion, SceneVersion)>,
    recorded: Query<&Mesh3d, With<RecordedTrajectory>>,
    resolved: Query<&Mesh3d, With<ResolvedTrajectory>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if previous_versions.0 != data.recorded_trajectory_version {
        for mesh in &recorded {
            let _ = meshes.insert(mesh.id(), trajectory_mesh(&data.recorded_trajectory));
        }
        previous_versions.0 = data.recorded_trajectory_version;
    }
    if previous_versions.1 != data.resolved_trajectory_version {
        for mesh in &resolved {
            let _ = meshes.insert(mesh.id(), trajectory_mesh(&data.resolved_trajectory));
        }
        previous_versions.1 = data.resolved_trajectory_version;
    }
}

fn update_global_debug_lines(
    data: Res<SceneData>,
    mut previous_version: Local<SceneVersion>,
    lines: Query<&Mesh3d, With<GlobalDebugLines>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    if *previous_version == data.global_debug_version {
        return;
    }
    for mesh in &lines {
        let _ = meshes.insert(mesh.id(), global_debug_lines_mesh(&data));
    }
    *previous_version = data.global_debug_version;
}

fn camera_to_field_transform(
    robot_to_field: linear_algebra::Isometry3<Robot, Field>,
    camera_matrix: &CameraMatrix,
) -> Transform {
    let camera_to_robot = robot_to_camera(camera_matrix).inverse();
    transform_from_isometry(robot_to_field.inner * camera_to_robot)
}

fn robot_to_camera(camera_matrix: &CameraMatrix) -> nalgebra::Isometry3<f32> {
    (camera_matrix.head_to_camera * camera_matrix.robot_to_head).inner
}

fn camera_frustum_mesh(camera_matrix: &CameraMatrix) -> Mesh {
    let corners = camera_viewport_corners(camera_matrix, CAMERA_VIEWPORT_DEPTH);
    let mut positions = Vec::with_capacity(16);

    for corner in corners {
        positions.push(camera_point(corner));
        positions.push(camera_point([0.0, 0.0, 0.0]));
    }
    for [start, end] in [[0, 1], [1, 2], [2, 3], [3, 0]] {
        positions.push(camera_point(corners[start]));
        positions.push(camera_point(corners[end]));
    }

    let mut mesh = Mesh::new(PrimitiveTopology::LineList, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh
}

fn camera_image_plane_mesh(camera_matrix: &CameraMatrix) -> Mesh {
    let corners = camera_viewport_corners(camera_matrix, CAMERA_VIEWPORT_DEPTH);
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, corners.map(camera_point).to_vec());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; 4]);
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_UV_0,
        vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    );
    mesh.insert_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3]));
    mesh
}

fn camera_viewport_corners(camera_matrix: &CameraMatrix, depth: f32) -> [[f32; 3]; 4] {
    let width = camera_matrix.image_size.x().max(1.0);
    let height = camera_matrix.image_size.y().max(1.0);
    let fx = camera_matrix.intrinsics.focals.x.max(f32::EPSILON);
    let fy = camera_matrix.intrinsics.focals.y.max(f32::EPSILON);
    let cx = camera_matrix.intrinsics.optical_center.x();
    let cy = camera_matrix.intrinsics.optical_center.y();

    [
        camera_viewport_corner(0.0, 0.0, depth, fx, fy, cx, cy),
        camera_viewport_corner(width, 0.0, depth, fx, fy, cx, cy),
        camera_viewport_corner(width, height, depth, fx, fy, cx, cy),
        camera_viewport_corner(0.0, height, depth, fx, fy, cx, cy),
    ]
}

fn camera_viewport_corner(
    pixel_x: f32,
    pixel_y: f32,
    depth: f32,
    focal_x: f32,
    focal_y: f32,
    center_x: f32,
    center_y: f32,
) -> [f32; 3] {
    [
        (pixel_x - center_x) / focal_x * depth,
        (pixel_y - center_y) / focal_y * depth,
        depth,
    ]
}

fn camera_point(point: [f32; 3]) -> [f32; 3] {
    convert_point(point).to_array()
}

fn camera_frame_image(frame: &CameraImage) -> Image {
    Image::new(
        Extent3d {
            width: frame.width,
            height: frame.height,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        frame.rgba.clone(),
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

fn trajectory_mesh(points: &[TrajectoryPoint]) -> Mesh {
    let mut positions = Vec::new();
    for window in points.windows(2) {
        if !window[0].seconds.is_finite() || !window[1].seconds.is_finite() {
            continue;
        }
        let a = window[0]
            .robot_to_field
            .inner
            .translation
            .vector
            .cast::<f32>();
        let b = window[1]
            .robot_to_field
            .inner
            .translation
            .vector
            .cast::<f32>();
        positions.push(convert_point([a.x, a.y, a.z]).to_array());
        positions.push(convert_point([b.x, b.y, b.z]).to_array());
    }

    let mut mesh = Mesh::new(PrimitiveTopology::LineList, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh
}

fn global_debug_lines_mesh(data: &SceneData) -> Mesh {
    let Some(debug) = data.global_debug.as_deref() else {
        return empty_mesh(PrimitiveTopology::LineList);
    };
    let Some(camera_matrix) = &data.camera_matrix else {
        return empty_mesh(PrimitiveTopology::LineList);
    };

    let ground_to_field = debug.robot_to_field.inner * camera_matrix.ground_to_robot.inner;
    let mut positions = Vec::with_capacity(debug.associations.len() * 2);
    for association in &debug.associations {
        let ground = association.back_projected_ground;
        let ground_in_field = ground_to_field * nalgebra::Point3::new(ground.x(), ground.y(), 0.03);
        let field = association.field_point;
        positions.push(
            convert_point([ground_in_field.x, ground_in_field.y, ground_in_field.z]).to_array(),
        );
        positions.push(convert_point([field.x(), field.y(), 0.03]).to_array());
    }

    let mut mesh = Mesh::new(PrimitiveTopology::LineList, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh
}

fn robot_marker_mesh() -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![
            [0.25, 0.04, 0.0],
            [-0.18, 0.04, 0.14],
            [-0.18, 0.04, -0.14],
            [0.25, 0.22, 0.0],
            [-0.18, 0.22, 0.14],
            [-0.18, 0.22, -0.14],
        ],
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; 6]);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0]; 6]);
    mesh.insert_indices(Indices::U32(vec![
        0, 1, 2, 3, 5, 4, 0, 3, 4, 0, 4, 1, 0, 2, 5, 0, 5, 3, 1, 4, 5, 1, 5, 2,
    ]));
    mesh
}

fn field_markings_mesh(dimensions: &FieldDimensions) -> Mesh {
    let mut mesh = FieldMarkingMesh::default();
    let line_width = dimensions.line_width.max(0.001);
    let half_length = dimensions.length / 2.0;
    let half_width = dimensions.width / 2.0;

    mesh.add_rect_stroke(
        -half_length,
        -half_width,
        half_length,
        half_width,
        line_width,
    );
    mesh.add_segment([0.0, -half_width], [0.0, half_width], line_width);
    mesh.add_arc(
        [0.0, 0.0],
        dimensions.center_circle_diameter / 2.0,
        0.0,
        std::f32::consts::TAU,
        line_width,
    );

    for sign in [-1.0, 1.0] {
        mesh.add_goal_area(
            dimensions,
            sign,
            dimensions.goal_box_area_length,
            dimensions.goal_box_area_width,
            line_width,
        );
        mesh.add_goal_area(
            dimensions,
            sign,
            dimensions.penalty_area_length,
            dimensions.penalty_area_width,
            line_width,
        );
        let penalty_x = sign * (half_length - dimensions.penalty_marker_distance);
        mesh.add_marker_cross([penalty_x, 0.0], dimensions.penalty_marker_size, line_width);
        let post_y = (dimensions.goal_inner_width + dimensions.goal_post_diameter) / 2.0;
        mesh.add_disk(
            [sign * half_length, post_y],
            dimensions.goal_post_diameter / 2.0,
        );
        mesh.add_disk(
            [sign * half_length, -post_y],
            dimensions.goal_post_diameter / 2.0,
        );
    }

    mesh.finish()
}

#[derive(Default)]
struct FieldMarkingMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    uvs: Vec<[f32; 2]>,
    indices: Vec<u32>,
}

impl FieldMarkingMesh {
    const HEIGHT: f32 = 0.025;

    fn add_goal_area(
        &mut self,
        dimensions: &FieldDimensions,
        sign: f32,
        length: f32,
        width: f32,
        line_width: f32,
    ) {
        let goal_line_x = sign * dimensions.length / 2.0;
        let inner_x = goal_line_x - sign * length;
        let half_width = width / 2.0;
        self.add_segment(
            [goal_line_x, -half_width],
            [inner_x, -half_width],
            line_width,
        );
        self.add_segment([inner_x, -half_width], [inner_x, half_width], line_width);
        self.add_segment([inner_x, half_width], [goal_line_x, half_width], line_width);
    }

    fn add_marker_cross(&mut self, center: [f32; 2], size: f32, line_width: f32) {
        let half_size = size / 2.0;
        self.add_segment(
            [center[0] - half_size, center[1]],
            [center[0] + half_size, center[1]],
            line_width,
        );
        self.add_segment(
            [center[0], center[1] - half_size],
            [center[0], center[1] + half_size],
            line_width,
        );
    }

    fn add_rect_stroke(&mut self, min_x: f32, min_y: f32, max_x: f32, max_y: f32, width: f32) {
        self.add_segment([min_x, min_y], [max_x, min_y], width);
        self.add_segment([max_x, min_y], [max_x, max_y], width);
        self.add_segment([max_x, max_y], [min_x, max_y], width);
        self.add_segment([min_x, max_y], [min_x, min_y], width);
    }

    fn add_segment(&mut self, start: [f32; 2], end: [f32; 2], width: f32) {
        let delta = [end[0] - start[0], end[1] - start[1]];
        let length = delta[0].hypot(delta[1]);
        if length <= f32::EPSILON {
            return;
        }
        let half_width = width / 2.0;
        let perpendicular = [
            -delta[1] / length * half_width,
            delta[0] / length * half_width,
        ];
        self.add_quad([
            [start[0] - perpendicular[0], start[1] - perpendicular[1]],
            [end[0] - perpendicular[0], end[1] - perpendicular[1]],
            [end[0] + perpendicular[0], end[1] + perpendicular[1]],
            [start[0] + perpendicular[0], start[1] + perpendicular[1]],
        ]);
    }

    fn add_arc(&mut self, center: [f32; 2], radius: f32, start: f32, end: f32, width: f32) {
        if radius <= 0.0 {
            return;
        }
        let half_width = width / 2.0;
        let inner_radius = (radius - half_width).max(0.0);
        let outer_radius = radius + half_width;
        let segments = ((radius * (end - start).abs()) / 0.05).ceil() as usize;
        let segments = segments.clamp(8, 96);
        for index in 0..segments {
            let angle0 = start + (end - start) * index as f32 / segments as f32;
            let angle1 = start + (end - start) * (index + 1) as f32 / segments as f32;
            self.add_quad([
                arc_point(center, inner_radius, angle0),
                arc_point(center, inner_radius, angle1),
                arc_point(center, outer_radius, angle1),
                arc_point(center, outer_radius, angle0),
            ]);
        }
    }

    fn add_disk(&mut self, center: [f32; 2], radius: f32) {
        if radius <= 0.0 {
            return;
        }
        for index in 0..32 {
            let angle0 = std::f32::consts::TAU * index as f32 / 32.0;
            let angle1 = std::f32::consts::TAU * (index + 1) as f32 / 32.0;
            self.add_triangle([
                center,
                arc_point(center, radius, angle1),
                arc_point(center, radius, angle0),
            ]);
        }
    }

    fn add_quad(&mut self, points: [[f32; 2]; 4]) {
        let base = self.positions.len() as u32;
        for point in points {
            self.add_vertex(point);
        }
        self.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    fn add_triangle(&mut self, points: [[f32; 2]; 3]) {
        let base = self.positions.len() as u32;
        for point in points {
            self.add_vertex(point);
        }
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }

    fn add_vertex(&mut self, point: [f32; 2]) {
        self.positions.push([point[0], Self::HEIGHT, -point[1]]);
        self.normals.push([0.0, 1.0, 0.0]);
        self.uvs.push([0.0, 0.0]);
    }

    fn finish(self) -> Mesh {
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::RENDER_WORLD,
        );
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals);
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, self.uvs);
        mesh.insert_indices(Indices::U32(self.indices));
        mesh
    }
}

fn arc_point(center: [f32; 2], radius: f32, angle: f32) -> [f32; 2] {
    [
        center[0] + radius * angle.cos(),
        center[1] + radius * angle.sin(),
    ]
}

fn empty_mesh(topology: PrimitiveTopology) -> Mesh {
    let mut mesh = Mesh::new(topology, RenderAssetUsages::RENDER_WORLD);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new());
    mesh
}

fn field_mesh() -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![
            [-0.5, 0.0, -0.5],
            [0.5, 0.0, -0.5],
            [0.5, 0.0, 0.5],
            [-0.5, 0.0, 0.5],
        ],
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; 4]);
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_UV_0,
        vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    );
    mesh.insert_indices(Indices::U32(vec![0, 2, 1, 0, 3, 2]));
    mesh
}

fn transform_from_isometry(isometry: nalgebra::Isometry3<f32>) -> Transform {
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

fn convert_point([x, y, z]: [f32; 3]) -> Vec3 {
    Vec3::new(x, z, -y)
}
