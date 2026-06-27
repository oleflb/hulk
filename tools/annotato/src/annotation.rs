use eframe::egui::Pos2;
use serde::{Deserialize, Deserializer, Serialize, de::Error as DeError};

use crate::{
    boundingbox::{BoundingBox, clamp_normalized},
    classes::Class,
};

#[derive(Serialize, Debug, Clone, PartialEq)]
pub struct AnnotationFormat {
    class: Class,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    points: Option<[[f32; 2]; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    point: Option<[f32; 2]>,
}

impl<'de> Deserialize<'de> for AnnotationFormat {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct RawAnnotationFormat {
            class: Class,
            #[serde(default)]
            points: Option<[[f32; 2]; 2]>,
            #[serde(default)]
            point: Option<[f32; 2]>,
        }

        let raw = RawAnnotationFormat::deserialize(deserializer)?;
        if raw.points.is_none() && raw.point.is_none() {
            return Err(D::Error::custom(
                "annotation must contain `points` or `point` geometry",
            ));
        }
        if let Some(points) = raw.points {
            for point in points {
                validate_normalized_point(point).map_err(D::Error::custom)?;
            }
        }
        if let Some(point) = raw.point {
            validate_normalized_point(point).map_err(D::Error::custom)?;
        }
        if raw.point.is_some() && !raw.class.supports_points() {
            return Err(D::Error::custom(format!(
                "{} does not support point geometry",
                raw.class.as_str()
            )));
        }
        if raw.points.is_some() && raw.point.is_some() && !raw.class.requires_point() {
            return Err(D::Error::custom(format!(
                "{} cannot combine `points` and `point` geometry",
                raw.class.as_str()
            )));
        }

        Ok(Self {
            class: raw.class,
            points: raw.points,
            point: raw.point,
        })
    }
}

fn validate_normalized_point([x, y]: [f32; 2]) -> Result<(), String> {
    if x.is_finite() && y.is_finite() && (0.0..=1.0).contains(&x) && (0.0..=1.0).contains(&y) {
        Ok(())
    } else {
        Err(format!(
            "normalized coordinates must be finite and between 0 and 1, got [{x}, {y}]"
        ))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Annotation {
    pub class: Class,
    geometry: AnnotationGeometry,
}

#[derive(Debug, Clone, PartialEq)]
pub enum AnnotationGeometry {
    BoundingBox(BoundingBox),
    Point(Pos2),
    BoundingBoxAndPoint {
        bounding_box: BoundingBox,
        point: Pos2,
    },
}

impl Annotation {
    pub fn point(class: Class, point: Pos2) -> Self {
        Self {
            class,
            geometry: AnnotationGeometry::Point(point),
        }
    }

    pub fn bounding_box(class: Class, bounding_box: BoundingBox) -> Self {
        Self {
            class,
            geometry: AnnotationGeometry::BoundingBox(bounding_box),
        }
    }

    pub fn from_format(format: AnnotationFormat, image_size: [f32; 2]) -> Self {
        let class = format.class;
        let bounding_box = format
            .points
            .map(|points| BoundingBox::from_points(points, image_size));
        let point = format
            .point
            .map(|[x, y]| Pos2::new(x * image_size[0], y * image_size[1]));
        let geometry = match (bounding_box, point) {
            (Some(bounding_box), Some(point)) => AnnotationGeometry::BoundingBoxAndPoint {
                bounding_box,
                point,
            },
            (Some(bounding_box), None) => AnnotationGeometry::BoundingBox(bounding_box),
            (None, Some(point)) => AnnotationGeometry::Point(point),
            (None, None) => unreachable!("AnnotationFormat rejects missing geometry"),
        };

        Self { class, geometry }
    }

    pub fn to_format(&self, image_size: [f32; 2]) -> AnnotationFormat {
        AnnotationFormat {
            class: self.class,
            points: self
                .bounding_box_ref()
                .map(|bounding_box| bounding_box.to_points(image_size)),
            point: self.point_position().map(|point| {
                [
                    clamp_normalized(point.x / image_size[0]),
                    clamp_normalized(point.y / image_size[1]),
                ]
            }),
        }
    }

    pub fn needs_point_migration(&self) -> bool {
        self.class.requires_point()
            && self.bounding_box_ref().is_some()
            && self.point_position().is_none()
    }

    pub fn bounding_box_ref(&self) -> Option<&BoundingBox> {
        match &self.geometry {
            AnnotationGeometry::BoundingBox(bounding_box)
            | AnnotationGeometry::BoundingBoxAndPoint { bounding_box, .. } => Some(bounding_box),
            AnnotationGeometry::Point(_) => None,
        }
    }

    pub fn bounding_box_mut(&mut self) -> Option<&mut BoundingBox> {
        match &mut self.geometry {
            AnnotationGeometry::BoundingBox(bounding_box)
            | AnnotationGeometry::BoundingBoxAndPoint { bounding_box, .. } => Some(bounding_box),
            AnnotationGeometry::Point(_) => None,
        }
    }

    pub fn point_position(&self) -> Option<Pos2> {
        match self.geometry {
            AnnotationGeometry::Point(point)
            | AnnotationGeometry::BoundingBoxAndPoint { point, .. } => Some(point),
            AnnotationGeometry::BoundingBox(_) => None,
        }
    }

    pub fn point_mut(&mut self) -> Option<&mut Pos2> {
        match &mut self.geometry {
            AnnotationGeometry::Point(point)
            | AnnotationGeometry::BoundingBoxAndPoint { point, .. } => Some(point),
            AnnotationGeometry::BoundingBox(_) => None,
        }
    }

    pub fn set_point(&mut self, point: Pos2) {
        self.geometry =
            match std::mem::replace(&mut self.geometry, AnnotationGeometry::Point(point)) {
                AnnotationGeometry::BoundingBox(bounding_box)
                | AnnotationGeometry::BoundingBoxAndPoint { bounding_box, .. } => {
                    AnnotationGeometry::BoundingBoxAndPoint {
                        bounding_box,
                        point,
                    }
                }
                AnnotationGeometry::Point(_) => AnnotationGeometry::Point(point),
            };
    }

    pub fn clear_bounding_box(&mut self) -> bool {
        match std::mem::replace(&mut self.geometry, AnnotationGeometry::Point(Pos2::ZERO)) {
            AnnotationGeometry::BoundingBox(bounding_box) => {
                self.geometry = AnnotationGeometry::BoundingBox(bounding_box);
                true
            }
            AnnotationGeometry::BoundingBoxAndPoint { point, .. } => {
                self.geometry = AnnotationGeometry::Point(point);
                false
            }
            AnnotationGeometry::Point(point) => {
                self.geometry = AnnotationGeometry::Point(point);
                false
            }
        }
    }

    pub fn clear_point(&mut self) -> bool {
        match std::mem::replace(&mut self.geometry, AnnotationGeometry::Point(Pos2::ZERO)) {
            AnnotationGeometry::Point(point) => {
                self.geometry = AnnotationGeometry::Point(point);
                true
            }
            AnnotationGeometry::BoundingBoxAndPoint { bounding_box, .. } => {
                self.geometry = AnnotationGeometry::BoundingBox(bounding_box);
                false
            }
            AnnotationGeometry::BoundingBox(bounding_box) => {
                self.geometry = AnnotationGeometry::BoundingBox(bounding_box);
                false
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use eframe::egui::Rect;

    use super::*;

    #[test]
    fn legacy_bbox_only_labels_deserialize() {
        let json = r#"{"class":"LSpot","points":[[0.1,0.2],[0.3,0.4]]}"#;

        let format: AnnotationFormat = serde_json::from_str(json).unwrap();
        let annotation = Annotation::from_format(format, [1000.0, 500.0]);

        assert_eq!(annotation.class, Class::LSpot);
        assert!(annotation.bounding_box_ref().is_some());
        assert!(annotation.point_position().is_none());
        assert!(annotation.needs_point_migration());
    }

    #[test]
    fn point_only_labels_serialize_without_bbox() {
        let annotation = Annotation::point(Class::TSpot, Pos2::new(20.0, 50.0));

        let format = annotation.to_format([100.0, 200.0]);

        assert_eq!(format.class, Class::TSpot);
        assert_eq!(format.points, None);
        assert_eq!(format.point, Some([0.2, 0.25]));
    }

    #[test]
    fn migrated_labels_keep_bbox_and_add_point() {
        let mut annotation = Annotation::bounding_box(
            Class::XSpot,
            BoundingBox::from_rect(Rect::from_min_max(
                Pos2::new(10.0, 20.0),
                Pos2::new(40.0, 80.0),
            )),
        );
        annotation.set_point(Pos2::new(25.0, 60.0));

        let format = annotation.to_format([100.0, 100.0]);

        assert_eq!(format.points, Some([[0.1, 0.2], [0.4, 0.8]]));
        assert_eq!(format.point, Some([0.25, 0.6]));
    }

    #[test]
    fn labels_without_geometry_are_rejected() {
        let error = serde_json::from_str::<AnnotationFormat>(r#"{"class":"Ball"}"#).unwrap_err();

        assert!(error.to_string().contains("points"));
    }

    #[test]
    fn unsupported_point_labels_are_rejected() {
        let error =
            serde_json::from_str::<AnnotationFormat>(r#"{"class":"Robot","point":[0.5,0.5]}"#)
                .unwrap_err();

        assert!(error.to_string().contains("point geometry"));
    }

    #[test]
    fn non_migration_labels_cannot_mix_box_and_point() {
        let error = serde_json::from_str::<AnnotationFormat>(
            r#"{"class":"GoalPost","points":[[0.1,0.2],[0.3,0.4]],"point":[0.2,0.3]}"#,
        )
        .unwrap_err();

        assert!(error.to_string().contains("cannot combine"));
    }

    #[test]
    fn out_of_range_normalized_coordinates_are_rejected() {
        let error = serde_json::from_str::<AnnotationFormat>(
            r#"{"class":"Ball","points":[[-0.1,0.2],[0.3,0.4]]}"#,
        )
        .unwrap_err();

        assert!(error.to_string().contains("between 0 and 1"));
    }
}
