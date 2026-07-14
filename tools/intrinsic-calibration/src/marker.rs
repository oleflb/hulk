use color_eyre::Result;
use eframe::egui::{self, Pos2};
use image::RgbImage;
use opencv::{core, objdetect, prelude::*};

#[derive(Clone, Copy)]
pub(crate) struct DetectedTag {
    pub(crate) id: i32,
    pub(crate) corners: [Pos2; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MarkerType {
    Aruco4x4_50,
    Aruco4x4_100,
    Aruco4x4_250,
    Aruco4x4_1000,
    Aruco5x5_50,
    Aruco5x5_100,
    Aruco5x5_250,
    Aruco5x5_1000,
    Aruco6x6_50,
    Aruco6x6_100,
    Aruco6x6_250,
    Aruco6x6_1000,
    Aruco7x7_50,
    Aruco7x7_100,
    Aruco7x7_250,
    Aruco7x7_1000,
    ArucoOriginal,
    AprilTag16h5,
    AprilTag25h9,
    AprilTag36h10,
    AprilTag36h11,
    ArucoMip36h12,
}

impl Default for MarkerType {
    fn default() -> Self {
        Self::AprilTag36h11
    }
}

impl MarkerType {
    pub(crate) const ALL: [Self; 22] = [
        Self::Aruco4x4_50,
        Self::Aruco4x4_100,
        Self::Aruco4x4_250,
        Self::Aruco4x4_1000,
        Self::Aruco5x5_50,
        Self::Aruco5x5_100,
        Self::Aruco5x5_250,
        Self::Aruco5x5_1000,
        Self::Aruco6x6_50,
        Self::Aruco6x6_100,
        Self::Aruco6x6_250,
        Self::Aruco6x6_1000,
        Self::Aruco7x7_50,
        Self::Aruco7x7_100,
        Self::Aruco7x7_250,
        Self::Aruco7x7_1000,
        Self::ArucoOriginal,
        Self::AprilTag16h5,
        Self::AprilTag25h9,
        Self::AprilTag36h10,
        Self::AprilTag36h11,
        Self::ArucoMip36h12,
    ];

    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Aruco4x4_50 => "ArUco 4x4 50",
            Self::Aruco4x4_100 => "ArUco 4x4 100",
            Self::Aruco4x4_250 => "ArUco 4x4 250",
            Self::Aruco4x4_1000 => "ArUco 4x4 1000",
            Self::Aruco5x5_50 => "ArUco 5x5 50",
            Self::Aruco5x5_100 => "ArUco 5x5 100",
            Self::Aruco5x5_250 => "ArUco 5x5 250",
            Self::Aruco5x5_1000 => "ArUco 5x5 1000",
            Self::Aruco6x6_50 => "ArUco 6x6 50",
            Self::Aruco6x6_100 => "ArUco 6x6 100",
            Self::Aruco6x6_250 => "ArUco 6x6 250",
            Self::Aruco6x6_1000 => "ArUco 6x6 1000",
            Self::Aruco7x7_50 => "ArUco 7x7 50",
            Self::Aruco7x7_100 => "ArUco 7x7 100",
            Self::Aruco7x7_250 => "ArUco 7x7 250",
            Self::Aruco7x7_1000 => "ArUco 7x7 1000",
            Self::ArucoOriginal => "ArUco original",
            Self::AprilTag16h5 => "AprilTag 16h5",
            Self::AprilTag25h9 => "AprilTag 25h9",
            Self::AprilTag36h10 => "AprilTag 36h10",
            Self::AprilTag36h11 => "AprilTag 36h11",
            Self::ArucoMip36h12 => "ArUco MIP 36h12",
        }
    }

    fn dictionary(self) -> objdetect::PredefinedDictionaryType {
        match self {
            Self::Aruco4x4_50 => objdetect::PredefinedDictionaryType::DICT_4X4_50,
            Self::Aruco4x4_100 => objdetect::PredefinedDictionaryType::DICT_4X4_100,
            Self::Aruco4x4_250 => objdetect::PredefinedDictionaryType::DICT_4X4_250,
            Self::Aruco4x4_1000 => objdetect::PredefinedDictionaryType::DICT_4X4_1000,
            Self::Aruco5x5_50 => objdetect::PredefinedDictionaryType::DICT_5X5_50,
            Self::Aruco5x5_100 => objdetect::PredefinedDictionaryType::DICT_5X5_100,
            Self::Aruco5x5_250 => objdetect::PredefinedDictionaryType::DICT_5X5_250,
            Self::Aruco5x5_1000 => objdetect::PredefinedDictionaryType::DICT_5X5_1000,
            Self::Aruco6x6_50 => objdetect::PredefinedDictionaryType::DICT_6X6_50,
            Self::Aruco6x6_100 => objdetect::PredefinedDictionaryType::DICT_6X6_100,
            Self::Aruco6x6_250 => objdetect::PredefinedDictionaryType::DICT_6X6_250,
            Self::Aruco6x6_1000 => objdetect::PredefinedDictionaryType::DICT_6X6_1000,
            Self::Aruco7x7_50 => objdetect::PredefinedDictionaryType::DICT_7X7_50,
            Self::Aruco7x7_100 => objdetect::PredefinedDictionaryType::DICT_7X7_100,
            Self::Aruco7x7_250 => objdetect::PredefinedDictionaryType::DICT_7X7_250,
            Self::Aruco7x7_1000 => objdetect::PredefinedDictionaryType::DICT_7X7_1000,
            Self::ArucoOriginal => objdetect::PredefinedDictionaryType::DICT_ARUCO_ORIGINAL,
            Self::AprilTag16h5 => objdetect::PredefinedDictionaryType::DICT_APRILTAG_16h5,
            Self::AprilTag25h9 => objdetect::PredefinedDictionaryType::DICT_APRILTAG_25h9,
            Self::AprilTag36h10 => objdetect::PredefinedDictionaryType::DICT_APRILTAG_36h10,
            Self::AprilTag36h11 => objdetect::PredefinedDictionaryType::DICT_APRILTAG_36h11,
            Self::ArucoMip36h12 => objdetect::PredefinedDictionaryType::DICT_ARUCO_MIP_36h12,
        }
    }

    fn is_apriltag(self) -> bool {
        matches!(
            self,
            Self::AprilTag16h5 | Self::AprilTag25h9 | Self::AprilTag36h10 | Self::AprilTag36h11
        )
    }
}

pub(crate) fn create_marker_detector(marker_type: MarkerType) -> Result<objdetect::ArucoDetector> {
    let dictionary = objdetect::get_predefined_dictionary(marker_type.dictionary())?;
    let mut parameters = objdetect::DetectorParameters::default()?;
    parameters.set_corner_refinement_method(if marker_type.is_apriltag() {
        objdetect::CORNER_REFINE_APRILTAG
    } else {
        objdetect::CORNER_REFINE_SUBPIX
    });
    Ok(objdetect::ArucoDetector::new(
        &dictionary,
        &parameters,
        objdetect::RefineParameters::new_def()?,
    )?)
}

pub(crate) fn detect_markers(
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
