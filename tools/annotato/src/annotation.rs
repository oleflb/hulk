use eframe::egui::Pos2;
use serde::{Deserialize, Deserializer, Serialize, de::Error as DeError};

use crate::{
    boundingbox::{BoundingBox, clamp_normalized},
    classes::Class,
};

#[derive(Serialize, Debug, Clone, PartialEq)]
pub struct AnnotationFormat {
    pub class: Class,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub points: Option<[[f32; 2]; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<[f32; 2]>,
}

impl<'de> Deserialize<'de> for AnnotationFormat {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
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

        Ok(Self {
            class: raw.class,
            points: raw.points,
            point: raw.point,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Annotation {
    pub class: Class,
    pub bounding_box: Option<BoundingBox>,
    pub point: Option<Pos2>,
}

impl Annotation {
    pub fn point(class: Class, point: Pos2) -> Self {
        Self {
            class,
            bounding_box: None,
            point: Some(point),
        }
    }

    pub fn bounding_box(class: Class, bounding_box: BoundingBox) -> Self {
        Self {
            class,
            bounding_box: Some(bounding_box),
            point: None,
        }
    }

    pub fn from_format(format: AnnotationFormat, image_size: [f32; 2]) -> Self {
        let class = format.class;
        Self {
            class,
            bounding_box: format
                .points
                .map(|points| BoundingBox::from_points(points, image_size)),
            point: format
                .point
                .map(|[x, y]| Pos2::new(x * image_size[0], y * image_size[1])),
        }
    }

    pub fn to_format(&self, image_size: [f32; 2]) -> AnnotationFormat {
        AnnotationFormat {
            class: self.class,
            points: self
                .bounding_box
                .as_ref()
                .map(|bounding_box| bounding_box.to_points(image_size)),
            point: self.point.map(|point| {
                [
                    clamp_normalized(point.x / image_size[0]),
                    clamp_normalized(point.y / image_size[1]),
                ]
            }),
        }
    }

    pub fn needs_point_migration(&self) -> bool {
        self.class.requires_point() && self.bounding_box.is_some() && self.point.is_none()
    }

    pub fn is_empty(&self) -> bool {
        self.bounding_box.is_none() && self.point.is_none()
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
        assert!(annotation.bounding_box.is_some());
        assert!(annotation.point.is_none());
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
        annotation.point = Some(Pos2::new(25.0, 60.0));

        let format = annotation.to_format([100.0, 100.0]);

        assert_eq!(format.points, Some([[0.1, 0.2], [0.4, 0.8]]));
        assert_eq!(format.point, Some([0.25, 0.6]));
    }

    #[test]
    fn labels_without_geometry_are_rejected() {
        let error = serde_json::from_str::<AnnotationFormat>(r#"{"class":"Ball"}"#).unwrap_err();

        assert!(error.to_string().contains("points"));
    }
}
