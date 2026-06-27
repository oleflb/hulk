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

impl AnnotationFormat {
    pub fn class(&self) -> Class {
        self.class
    }

    pub fn needs_point_migration(&self) -> bool {
        self.class.requires_point() && self.points.is_some() && self.point.is_none()
    }
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

#[derive(Serialize, Debug, Clone, PartialEq)]
pub struct LabelFileFormat {
    pub labeled_classes: Vec<Class>,
    pub annotations: Vec<AnnotationFormat>,
}

impl LabelFileFormat {
    pub fn from_unlabeled_annotations(annotations: Vec<AnnotationFormat>) -> Self {
        Self {
            labeled_classes: Vec::new(),
            annotations,
        }
    }

    pub fn class_is_labeled(&self, class: Class) -> bool {
        self.labeled_classes.contains(&class)
    }

    pub fn has_pending_migration_for_class(&self, class: Class) -> bool {
        self.annotations
            .iter()
            .any(|annotation| annotation.class() == class && annotation.needs_point_migration())
    }
}

impl<'de> Deserialize<'de> for LabelFileFormat {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct CurrentLabelFileFormat {
            #[serde(default)]
            labeled_classes: Vec<Class>,
            #[serde(default)]
            annotations: Vec<AnnotationFormat>,
        }

        #[derive(Deserialize)]
        #[serde(untagged)]
        enum AnyLabelFileFormat {
            Current(CurrentLabelFileFormat),
            Legacy(Vec<AnnotationFormat>),
        }

        match AnyLabelFileFormat::deserialize(deserializer)? {
            AnyLabelFileFormat::Current(current) => Ok(Self {
                labeled_classes: normalize_labeled_classes(current.labeled_classes),
                annotations: current.annotations,
            }),
            AnyLabelFileFormat::Legacy(annotations) => Ok(Self {
                labeled_classes: infer_labeled_classes(&annotations),
                annotations,
            }),
        }
    }
}

pub fn normalize_labeled_classes(classes: Vec<Class>) -> Vec<Class> {
    Class::ALL
        .into_iter()
        .filter(|class| classes.contains(class))
        .collect()
}

fn infer_labeled_classes(annotations: &[AnnotationFormat]) -> Vec<Class> {
    Class::ALL
        .into_iter()
        .filter(|class| {
            annotations
                .iter()
                .any(|annotation| annotation.class() == *class)
        })
        .collect()
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
    bounding_box: Option<BoundingBox>,
    point: Option<Pos2>,
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
        let bounding_box = format
            .points
            .map(|points| BoundingBox::from_points(points, image_size));
        let point = format
            .point
            .map(|[x, y]| Pos2::new(x * image_size[0], y * image_size[1]));

        Self {
            class,
            bounding_box,
            point,
        }
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
        self.bounding_box.as_ref()
    }

    pub fn bounding_box_mut(&mut self) -> Option<&mut BoundingBox> {
        self.bounding_box.as_mut()
    }

    pub fn point_position(&self) -> Option<Pos2> {
        self.point
    }

    pub fn point_mut(&mut self) -> Option<&mut Pos2> {
        self.point.as_mut()
    }

    pub fn set_point(&mut self, point: Pos2) {
        self.point = Some(point);
    }

    pub fn clear_bounding_box(&mut self) -> bool {
        self.bounding_box = None;
        self.point.is_none()
    }

    pub fn clear_point(&mut self) -> bool {
        self.point = None;
        self.bounding_box.is_none()
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
        let annotation = Annotation::point(Class::PenaltySpot, Pos2::new(20.0, 50.0));

        let format = annotation.to_format([100.0, 200.0]);

        assert_eq!(format.class, Class::PenaltySpot);
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
    fn goal_post_legacy_boxes_can_be_migrated_to_points() {
        let format: AnnotationFormat = serde_json::from_str(
            r#"{"class":"GoalPost","points":[[0.1,0.2],[0.3,0.4]],"point":[0.2,0.3]}"#,
        )
        .unwrap();

        assert_eq!(format.class, Class::GoalPost);
        assert_eq!(format.points, Some([[0.1, 0.2], [0.3, 0.4]]));
        assert_eq!(format.point, Some([0.2, 0.3]));
    }

    #[test]
    fn out_of_range_normalized_coordinates_are_rejected() {
        let error = serde_json::from_str::<AnnotationFormat>(
            r#"{"class":"Ball","points":[[-0.1,0.2],[0.3,0.4]]}"#,
        )
        .unwrap_err();

        assert!(error.to_string().contains("between 0 and 1"));
    }

    #[test]
    fn legacy_label_arrays_infer_labeled_classes() {
        let json = r#"[{"class":"Robot","points":[[0.1,0.2],[0.3,0.4]]}]"#;

        let label_file: LabelFileFormat = serde_json::from_str(json).unwrap();

        assert_eq!(label_file.labeled_classes, vec![Class::Robot]);
        assert_eq!(label_file.annotations.len(), 1);
    }

    #[test]
    fn label_file_can_mark_empty_class_as_labeled() {
        let json = r#"{"labeled_classes":["Person"],"annotations":[]}"#;

        let label_file: LabelFileFormat = serde_json::from_str(json).unwrap();

        assert!(label_file.class_is_labeled(Class::Person));
        assert!(label_file.annotations.is_empty());
    }

    #[test]
    fn labeled_class_still_requires_pending_point_migration() {
        let json = r#"{"labeled_classes":["TSpot"],"annotations":[{"class":"TSpot","points":[[0.1,0.2],[0.3,0.4]]}]}"#;

        let label_file: LabelFileFormat = serde_json::from_str(json).unwrap();

        assert!(label_file.class_is_labeled(Class::TSpot));
        assert!(label_file.has_pending_migration_for_class(Class::TSpot));
    }
}
