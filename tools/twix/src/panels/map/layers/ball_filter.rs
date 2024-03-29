use std::sync::Arc;

use color_eyre::Result;
use communication::client::{Cycler, CyclerOutput, Output};
use eframe::epaint::{Color32, Stroke};

use coordinate_systems::{Field, Ground};
use linear_algebra::{IntoFramed, Isometry2};
use types::{ball_filter::Hypothesis, field_dimensions::FieldDimensions};

use crate::{
    nao::Nao, panels::map::layer::Layer, twix_painter::TwixPainter, value_buffer::ValueBuffer,
};

pub struct BallFilter {
    ground_to_field: ValueBuffer,
    ball_hypotheses: ValueBuffer,
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

        Self {
            ground_to_field,
            ball_hypotheses,
        }
    }

    fn paint(
        &self,
        painter: &TwixPainter<Field>,
        _field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        let ground_to_field: Option<Isometry2<Ground, Field>> =
            self.ground_to_field.parse_latest()?;

        let ball_hypotheses: Vec<Hypothesis> =
            self.ball_hypotheses.parse_latest().unwrap_or_default();

        for hypothesis in ball_hypotheses {
            let rolling_state = &hypothesis.rolling_state;
            let rolling_position = ground_to_field.unwrap_or_default() * rolling_state.mean.xy().framed().as_point();
            let rolling_covariance = rolling_state
                .covariance
                .fixed_view::<2, 2>(0, 0)
                .into_owned();
            let stroke = Stroke::new(0.01, Color32::BLACK);
            painter.covariance(rolling_position, rolling_covariance, stroke, Color32::YELLOW.gamma_multiply(0.5));

            let resting_state = &hypothesis.resting_state;
            let resting_position = ground_to_field.unwrap_or_default() * resting_state.mean.xy().framed().as_point();
            let resting_covariance = resting_state
                .covariance
                .into_owned();
            let stroke = Stroke::new(0.01, Color32::BLACK);
            painter.covariance(resting_position, resting_covariance, stroke, Color32::BLUE.gamma_multiply(0.5));
        }

        Ok(())
    }
}
