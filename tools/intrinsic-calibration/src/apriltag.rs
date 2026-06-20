use color_eyre::Result;
use eframe::egui::{self, Pos2};
use image::RgbImage;
use opencv::{core, objdetect, prelude::*};

#[derive(Clone, Copy)]
pub(crate) struct DetectedTag {
    pub(crate) id: i32,
    pub(crate) corners: [Pos2; 4],
}

pub(crate) fn create_apriltag_detector() -> Result<objdetect::ArucoDetector> {
    let dictionary = objdetect::get_predefined_dictionary(
        objdetect::PredefinedDictionaryType::DICT_APRILTAG_36h11,
    )?;
    let mut parameters = objdetect::DetectorParameters::default()?;
    parameters.set_corner_refinement_method(objdetect::CORNER_REFINE_APRILTAG);
    Ok(objdetect::ArucoDetector::new(
        &dictionary,
        &parameters,
        objdetect::RefineParameters::new_def()?,
    )?)
}

pub(crate) fn detect_tags(
    detector: &objdetect::ArucoDetector,
    image: &RgbImage,
) -> Result<Vec<DetectedTag>> {
    let gray: Vec<_> = image
        .pixels()
        .map(|p| ((77 * u16::from(p[0]) + 150 * u16::from(p[1]) + 29 * u16::from(p[2])) >> 8) as u8)
        .collect();
    let mat =
        core::Mat::new_rows_cols_with_data(image.height() as i32, image.width() as i32, &gray)?;
    let mut corners = core::Vector::<core::Vector<core::Point2f>>::new();
    let mut ids = core::Vector::<i32>::new();
    detector.detect_markers(&mat, &mut corners, &mut ids, &mut core::no_array())?;

    (0..corners.len())
        .map(|i| {
            let points = corners.get(i)?;
            Ok(DetectedTag {
                id: ids.get(i)?,
                corners: [0, 1, 2, 3].map(|j| {
                    let point = points.get(j).unwrap_or_default();
                    egui::pos2(point.x, point.y)
                }),
            })
        })
        .collect()
}
