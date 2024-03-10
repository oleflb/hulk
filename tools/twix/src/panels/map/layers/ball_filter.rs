use std::sync::Arc;

use color_eyre::{eyre::bail, Result};
use communication::client::{Cycler, CyclerOutput, Output};
use eframe::epaint::{Color32, Stroke};

use coordinate_systems::{Field, Ground};
use linear_algebra::{Isometry2, Point2};
use types::{
    ball_filter::Hypothesis, field_dimensions::FieldDimensions,
    parameters::BallFilterParameters,
};

use crate::{
    nao::Nao, panels::map::layer::Layer, twix_painter::TwixPainter, value_buffer::ValueBuffer,
};

pub struct BallFilter {
    ground_to_field: ValueBuffer,
    ball_hypotheses: ValueBuffer,
    ball_filter_parameters: ValueBuffer,
}

impl Layer for BallFilter {
    const NAME: &'static str = "Ball Filter";

    fn new(nao: Arc<Nao>) -> Self {
        let ground_to_field = nao.subscribe_output(CyclerOutput {
            cycler: Cycler::Control,
            output: Output::Main {
                path: "ground_to_field".to_string(),
            },
        });
        let ball_hypotheses = nao.subscribe_output(CyclerOutput {
            cycler: Cycler::Control,
            output: Output::Additional {
                path: "ball_filter_hypotheses".to_string(),
            },
        });
        let ball_filter_parameters = nao.subscribe_parameter("ball_filter");

        Self {
            ground_to_field,
            ball_hypotheses,
            ball_filter_parameters,
        }
    }

    fn paint(&self, painter: &TwixPainter<Field>, _field_dimensions: &FieldDimensions) -> Result<()> {
        let ground_to_field: Option<Isometry2<Ground, Field>> = self.ground_to_field.parse_latest()?;
        let ball_filter_parameters: BallFilterParameters =
            match self.ball_filter_parameters.parse_latest() {
                Ok(parameters) => parameters,
                Err(error) => {
                    bail!("Failed to parse ball filter parameters: {}", error);
                }
            };

        let ball_hypotheses: Vec<Hypothesis> =
            self.ball_hypotheses.parse_latest().unwrap_or_default();

        for hypothesis in ball_hypotheses {
            let state = hypothesis.selected_state(&ball_filter_parameters);
            let position = ground_to_field.unwrap_or_default() * Point2::from(state.mean.xy());
            let covariance = state.covariance.fixed_view::<2, 2>(0, 0).into_owned();
            let stroke = Stroke::new(0.01, Color32::BLACK);
            let fill_color = Color32::from_rgba_unmultiplied(255, 255, 0, 100);
            painter.covariance(position, covariance, stroke, fill_color);
        }

        Ok(())
    }
}
