use std::sync::{Arc, Mutex};

use eframe::{egui::{Widget, Sense}, glow, epaint::{Vec2, PaintCallback}, egui_glow};
use serde_json::{Value, json};

use crate::{panel::Panel, nao::Nao};

pub struct Nao3dPanel {
    nao: Arc<Nao>,
    angle: f32,
    rotating_triangle: Option<Arc<Mutex<RotatingTriangle>>>,
}

impl Panel for Nao3dPanel {
    const NAME: &'static str = "Nao3d";

    fn new(nao: Arc<Nao>, _value: Option<&Value>) -> Self {
        Nao3dPanel { nao, angle: 0.0, rotating_triangle: None }
    }

    fn save(&self) -> Value {
        json!({})
    }
}

impl Nao3dPanel {
    pub fn set_gl(&mut self, context: &Arc<glow::Context>) {
        self.rotating_triangle = Some(Arc::new(Mutex::new(RotatingTriangle::new(context.as_ref()))));
    }
}

impl Widget for &mut Nao3dPanel {
    fn ui(self, ui: &mut eframe::egui::Ui) -> eframe::egui::Response {
        ui.vertical(|ui| {
            ui.label("Nao 3D View");
            let (rect, response) =
            ui.allocate_exact_size(Vec2::splat(300.0), Sense::drag());

            self.angle += response.drag_delta().x * 0.01;

            // Clone locals so we can move them into the paint callback:
            let angle = self.angle;
            if let Some(triangle) = &self.rotating_triangle {
                let triangle = triangle.clone();

                let callback = PaintCallback {
                    rect,
                    callback: Arc::new(egui_glow::CallbackFn::new(move |_info, painter| {
                        triangle.lock().expect("").paint(painter.gl(), angle)
                    }))
                };
                ui.painter().add(callback);
            }
        }).response
    }
}

struct RotatingTriangle {
    program: glow::Program,
    vertex_array: glow::VertexArray,
}

impl RotatingTriangle {
    fn new(gl: &glow::Context) -> Self {
        use glow::HasContext as _;

        let shader_version = if cfg!(target_arch = "wasm32") {
            "#version 300 es"
        } else {
            "#version 330"
        };

        unsafe {
            let program = gl.create_program().expect("Cannot create program");

            let (vertex_shader_source, fragment_shader_source) = (
                r#"
                    const vec2 verts[3] = vec2[3](
                        vec2(0.0, 1.0),
                        vec2(-1.0, -1.0),
                        vec2(1.0, -1.0)
                    );
                    const vec4 colors[3] = vec4[3](
                        vec4(1.0, 0.0, 0.0, 1.0),
                        vec4(0.0, 1.0, 0.0, 1.0),
                        vec4(0.0, 0.0, 1.0, 1.0)
                    );
                    out vec4 v_color;
                    uniform float u_angle;
                    void main() {
                        v_color = colors[gl_VertexID];
                        gl_Position = vec4(verts[gl_VertexID], 0.0, 1.0);
                        gl_Position.x *= cos(u_angle);
                    }
                "#,
                r#"
                    precision mediump float;
                    in vec4 v_color;
                    out vec4 out_color;
                    void main() {
                        out_color = v_color;
                    }
                "#,
            );

            let shader_sources = [
                (glow::VERTEX_SHADER, vertex_shader_source),
                (glow::FRAGMENT_SHADER, fragment_shader_source),
            ];

            let shaders: Vec<_> = shader_sources
                .iter()
                .map(|(shader_type, shader_source)| {
                    let shader = gl
                        .create_shader(*shader_type)
                        .expect("Cannot create shader");
                    gl.shader_source(shader, &format!("{}\n{}", shader_version, shader_source));
                    gl.compile_shader(shader);
                    assert!(
                        gl.get_shader_compile_status(shader),
                        "Failed to compile {shader_type}: {}",
                        gl.get_shader_info_log(shader)
                    );
                    gl.attach_shader(program, shader);
                    shader
                })
                .collect();

            gl.link_program(program);
            if !gl.get_program_link_status(program) {
                panic!("{}", gl.get_program_info_log(program));
            }

            for shader in shaders {
                gl.detach_shader(program, shader);
                gl.delete_shader(shader);
            }

            let vertex_array = gl
                .create_vertex_array()
                .expect("Cannot create vertex array");

            Self {
                program,
                vertex_array,
            }
        }
    }

    fn destroy(&self, gl: &glow::Context) {
        use glow::HasContext as _;
        unsafe {
            gl.delete_program(self.program);
            gl.delete_vertex_array(self.vertex_array);
        }
    }

    fn paint(&self, gl: &glow::Context, angle: f32) {
        use glow::HasContext as _;
        unsafe {
            gl.use_program(Some(self.program));
            gl.uniform_1_f32(
                gl.get_uniform_location(self.program, "u_angle").as_ref(),
                angle,
            );
            gl.bind_vertex_array(Some(self.vertex_array));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
        }
    }
}
