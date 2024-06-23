use std::collections::BTreeMap;

use eframe::egui::Pos2;

use crate::{polygon::Polygon, segmentator_widget::Class};

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
