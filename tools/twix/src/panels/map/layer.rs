use std::{marker::PhantomData, sync::Arc};

use color_eyre::Result;
use convert_case::{Case, Casing};
use eframe::egui::Ui;
use log::error;
use serde_json::{Value, json};

use types::field_dimensions::FieldDimensions;

use crate::backend::RobotBackend;
use twix_visualization::twix_painter::TwixPainter;

pub trait Layer<Frame> {
    const NAME: &'static str;
    fn is_querying(&self) -> bool {
        false
    }
    fn new(backend: Arc<RobotBackend>) -> Self;
    fn paint(&self, painter: &TwixPainter<Frame>, field_dimensions: &FieldDimensions)
    -> Result<()>;
}

pub struct EnabledLayer<T, Frame>
where
    T: Layer<Frame>,
{
    backend: Arc<RobotBackend>,
    layer: Option<T>,
    frame: PhantomData<Frame>,
    error: Option<String>,
}

impl<T, Frame> EnabledLayer<T, Frame>
where
    T: Layer<Frame>,
{
    pub fn is_querying(&self) -> bool {
        self.layer.as_ref().is_some_and(Layer::is_querying)
    }

    pub fn new(backend: Arc<RobotBackend>, value: Option<&Value>, active: bool) -> Self {
        let active = value
            .and_then(|value| value.get(T::NAME.to_case(Case::Snake)))
            .and_then(|value| value.get("active"))
            .and_then(|value| value.as_bool())
            .unwrap_or(active);
        let layer = active.then(|| T::new(backend.clone()));
        Self {
            backend,
            layer,
            frame: PhantomData,
            error: None,
        }
    }

    pub fn checkbox(&mut self, ui: &mut Ui) {
        let mut active = self.layer.is_some();
        if ui.checkbox(&mut active, T::NAME).changed() {
            self.error = None;
            match self.layer.is_some() {
                false => self.layer = Some(T::new(self.backend.clone())),
                true => self.layer = None,
            }
        }
    }

    pub fn paint(&mut self, painter: &TwixPainter<Frame>, field_dimensions: &FieldDimensions) {
        if let Some(layer) = &self.layer {
            let error = layer
                .paint(painter, field_dimensions)
                .err()
                .map(|error| format!("{error:#}"));
            if self.error != error {
                if let Some(error) = &error {
                    error!(
                        "map panel: failed to paint map overlay {}: {}",
                        T::NAME,
                        error
                    );
                }
                self.error = error;
            }
        }
    }

    pub fn save(&self) -> Value {
        json!({
            "active": self.layer.is_some(),
        })
    }
}
