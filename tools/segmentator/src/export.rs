use std::{collections::BTreeMap, path::Path};

use color_eyre::{eyre::Context, Result};
use eframe::egui::Pos2;
use serde::Serialize;

use crate::{polygon::Polygon, segmentator_widget::Class};

#[derive(Debug, Serialize)]
pub struct Export {
    segmentation: BTreeMap<Class, Vec<[Pos2; 3]>>,
}

impl From<BTreeMap<Class, Vec<Polygon>>> for Export {
    fn from(polygons: BTreeMap<Class, Vec<Polygon>>) -> Self {
        let segmentation = polygons
            .into_iter()
            .map(|(class, polygons)| {
                let triangles = polygons.iter().flat_map(Polygon::triangles).collect();
                (class, triangles)
            })
            .collect();

        Export { segmentation }
    }
}

impl Export {
    pub fn save_to(self, path: impl AsRef<Path>) -> Result<()> {
        let content = serde_json::to_string(&self)?;
        std::fs::write(path, content).wrap_err("failed to write to file")
    }
}
