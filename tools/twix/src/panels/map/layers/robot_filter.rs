use std::sync::Arc;

use color_eyre::Result;
use communication::client::{Cycler, CyclerOutput, Output};
use coordinate_systems::Ground;
use eframe::egui::{Color32, Stroke};
use linear_algebra::Point2;
use types::field_dimensions::FieldDimensions;

use crate::{
    nao::Nao, panels::map::layer::Layer, twix_painter::TwixPainter, value_buffer::ValueBuffer,
};

pub struct RobotFilter {
    filtered_robots: ValueBuffer,
}

impl Layer<Ground> for RobotFilter {
    const NAME: &'static str = "Robot Filter";

    fn new(nao: Arc<Nao>) -> Self {
        let filtered_robots = nao.subscribe_output(CyclerOutput {
            cycler: Cycler::Control,
            output: Output::Main {
                path: "robot_positions".to_string(),
            },
        });

        Self { filtered_robots }
    }

    fn paint(
        &self,
        painter: &TwixPainter<Ground>,
        _field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        let filtered_robots: Vec<Point2<Ground>> = self.filtered_robots.parse_latest()?;

        for robot in filtered_robots {
            let stroke = Stroke::new(0.01, Color32::BLACK);
            painter.circle(robot, 0.1, Color32::RED, stroke);
        }

        Ok(())
    }
}
