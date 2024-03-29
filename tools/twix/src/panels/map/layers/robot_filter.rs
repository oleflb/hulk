use std::sync::Arc;

use color_eyre::Result;
use communication::client::{Cycler, CyclerOutput, Output};
use coordinate_systems::{Field, Ground};
use eframe::egui::{Color32, Stroke};
use linear_algebra::{Isometry2, Point2};
use types::field_dimensions::FieldDimensions;

use crate::{
    nao::Nao, panels::map::layer::Layer, twix_painter::TwixPainter, value_buffer::ValueBuffer,
};

pub struct RobotFilter {
    ground_to_field: ValueBuffer,
    filtered_robots: ValueBuffer,
}

impl Layer for RobotFilter {
    const NAME: &'static str = "Robot Filter";

    fn new(nao: Arc<Nao>) -> Self {
        let robot_to_field = nao.subscribe_output(CyclerOutput {
            cycler: Cycler::Control,
            output: Output::Main {
                path: "robot_to_field".to_string(),
            },
        });
        let filtered_robots = nao.subscribe_output(CyclerOutput {
            cycler: Cycler::Control,
            output: Output::Main {
                path: "robot_positions".to_string(),
            },
        });
        Self {
            ground_to_field: robot_to_field,
            filtered_robots,
        }
    }

    fn paint(&self, painter: &TwixPainter<Field>, _field_dimensions: &FieldDimensions) -> Result<()> {
        let robot_to_field = self.ground_to_field.parse_latest::<Option<Isometry2<Ground, Field>>>()?.unwrap_or_default();
        let filtered_robots: Vec<Point2<Ground>> =
            self.filtered_robots.parse_latest()?;

        for robot in filtered_robots {
            let position = robot_to_field * robot;
            let stroke = Stroke::new(0.01, Color32::BLACK);
            painter.circle(position, 0.1, Color32::RED, stroke);
        }

        Ok(())
    }
}
