use eframe::{
    egui::{pos2, Color32, Mesh, Painter, Pos2, Rect, Stroke, TextureId},
    emath::RectTransform,
    epaint::{PathShape, Vertex, WHITE_UV},
};
use lyon_tessellation::{
    geometry_builder::simple_builder, math::Point, path::Path, FillOptions, FillTessellator,
    VertexBuffers,
};

pub struct Polygon {
    pub points: Vec<Pos2>,
}

impl Polygon {
    pub fn new() -> Self {
        Polygon { points: Vec::new() }
    }

    pub fn add_point(&mut self, point: Pos2) {
        self.points.push(point);
    }

    pub fn triangles(&self) -> Vec<[Pos2; 3]> {
        let identity = RectTransform::identity(Rect::from_min_max(pos2(0.0, 0.0), pos2(1.0, 1.0)));
        let mesh = meshify(&self.points, identity, Color32::TRANSPARENT);

        mesh.indices
            .chunks_exact(3)
            .map(|indices| {
                let a = mesh.vertices[indices[0] as usize].pos;
                let b = mesh.vertices[indices[1] as usize].pos;
                let c = mesh.vertices[indices[2] as usize].pos;
                [a, b, c]
            })
            .collect()
    }
}

pub fn paint_polygon(
    painter: &Painter,
    points: impl Iterator<Item = Pos2>,
    transform: RectTransform,
    color: Color32,
) {
    let points = points.collect::<Vec<_>>();
    if points.len() >= 3 {
        let mesh = meshify(&points, transform, color.gamma_multiply(0.5));
        painter.add(mesh);
    }
    if points.len() >= 2 {
        // mal ne nlinies
        painter.add(PathShape {
            points: points
                .iter()
                .map(|point| transform.transform_pos(*point))
                .collect(),
            closed: points.len() > 2,
            stroke: Stroke::new(3.0, color),
            fill: Color32::TRANSPARENT,
        });
    }
}

fn meshify(points: &[Pos2], transform: RectTransform, color: Color32) -> Mesh {
    let path = {
        let mut path_builder = Path::builder();
        if let Some(first_point) = points.first() {
            path_builder.begin(convert(first_point));
        }
        for point in points.iter().skip(1) {
            path_builder.line_to(convert(point));
        }
        path_builder.end(true);
        path_builder.build()
    };
    let mut buffers: VertexBuffers<Point, u16> = VertexBuffers::new();
    {
        let mut vertex_builder = simple_builder(&mut buffers);

        // Create the tessellator.
        let mut tessellator = FillTessellator::new();

        // Compute the tessellation.
        tessellator
            .tessellate_path(
                &path,
                &FillOptions::default().with_tolerance(0.001),
                &mut vertex_builder,
            )
            .expect("failed to tessellate");
    }

    Mesh {
        vertices: buffers
            .vertices
            .iter()
            .map(|v| Vertex {
                pos: transform.transform_pos(Pos2::new(v.x, v.y)),
                uv: WHITE_UV,
                color,
            })
            .collect(),
        indices: buffers.indices.into_iter().map(|i| i as u32).collect(),
        texture_id: TextureId::Managed(0),
    }
}

fn convert(point: &Pos2) -> lyon_tessellation::math::Point {
    lyon_tessellation::math::Point::new(point.x, point.y)
}
