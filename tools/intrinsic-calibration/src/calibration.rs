use color_eyre::{Result, eyre::eyre};
use eframe::egui::{Pos2, Vec2};
use opencv::{calib3d, core, prelude::*};

use crate::marker::DetectedTag;

const DEFAULT_MARKER_SIZE_METERS: f32 = 0.055;
const MIN_SAMPLES: usize = 10;

pub(crate) struct CalibrationState {
    samples: Vec<CalibrationSample>,
    result: Option<CalibrationResult>,
    status: String,
    marker_size_meters: f32,
}

#[derive(Clone)]
pub(crate) struct CalibrationSample {
    pub(crate) frame_identifier: u32,
    pub(crate) tag_id: i32,
    pub(crate) left: [Pos2; 4],
    pub(crate) right: [Pos2; 4],
    pub(crate) image_size: Vec2,
}

pub(crate) struct CalibrationResult {
    pub(crate) rms: f64,
    pub(crate) left_camera_matrix: Vec<f64>,
    pub(crate) left_dist_coeffs: Vec<f64>,
    pub(crate) right_camera_matrix: Vec<f64>,
    pub(crate) right_dist_coeffs: Vec<f64>,
    pub(crate) rotation: Vec<f64>,
    pub(crate) translation: Vec<f64>,
    pub(crate) baseline: f64,
}

impl Default for CalibrationState {
    fn default() -> Self {
        Self {
            samples: Vec::new(),
            result: None,
            status: format!("need {MIN_SAMPLES} samples"),
            marker_size_meters: DEFAULT_MARKER_SIZE_METERS,
        }
    }
}

impl CalibrationState {
    pub(crate) fn samples(&self) -> &[CalibrationSample] {
        &self.samples
    }

    pub(crate) fn result(&self) -> Option<&CalibrationResult> {
        self.result.as_ref()
    }

    pub(crate) fn status(&self) -> &str {
        &self.status
    }

    pub(crate) fn marker_size_meters(&self) -> f32 {
        self.marker_size_meters
    }

    pub(crate) fn set_marker_size_meters(&mut self, marker_size_meters: f32) {
        let marker_size_meters = marker_size_meters.max(0.001);
        if (self.marker_size_meters - marker_size_meters).abs() <= f32::EPSILON {
            return;
        }

        self.marker_size_meters = marker_size_meters;
        self.recalibrate();
    }

    pub(crate) fn clear_samples(&mut self) {
        self.samples.clear();
        self.result = None;
        self.status = format!("need {MIN_SAMPLES} samples");
    }

    pub(crate) fn capture(
        &mut self,
        frame_identifier: u32,
        image_size: Vec2,
        left_tags: &[DetectedTag],
        right_tags: &[DetectedTag],
    ) -> usize {
        let mut added = 0;
        for (left, right) in common_tags(left_tags, right_tags) {
            if self.samples.iter().any(|sample| {
                sample.frame_identifier == frame_identifier && sample.tag_id == left.id
            }) {
                continue;
            }

            self.samples.push(CalibrationSample {
                frame_identifier,
                tag_id: left.id,
                left: left.corners,
                right: right.corners,
                image_size,
            });
            added += 1;
        }

        if added > 0 {
            self.recalibrate();
        }
        added
    }

    fn recalibrate(&mut self) {
        if self.samples.len() < MIN_SAMPLES {
            self.result = None;
            self.status = format!("need {} more samples", MIN_SAMPLES - self.samples.len());
            return;
        }

        match calibrate(&self.samples, self.marker_size_meters) {
            Ok(result) => {
                self.status = format!("RMS {:.4}", result.rms);
                self.result = Some(result);
            }
            Err(error) => {
                self.result = None;
                self.status = format!("calibration failed: {error}");
            }
        }
    }
}

pub(crate) fn common_tags<'a>(
    left_tags: &'a [DetectedTag],
    right_tags: &'a [DetectedTag],
) -> Vec<(&'a DetectedTag, &'a DetectedTag)> {
    left_tags
        .iter()
        .filter_map(|left| {
            right_tags
                .iter()
                .find(|right| right.id == left.id)
                .map(|right| (left, right))
        })
        .collect()
}

fn calibrate(samples: &[CalibrationSample], marker_size_meters: f32) -> Result<CalibrationResult> {
    let image_size = image_size(samples)?;
    let object_points = object_points(samples.len(), marker_size_meters);
    let left_points = image_points(samples.iter().map(|sample| sample.left));
    let right_points = image_points(samples.iter().map(|sample| sample.right));

    let (mut left_camera_matrix, mut left_dist_coeffs) =
        calibrate_single(&object_points, &left_points, image_size)?;
    let (mut right_camera_matrix, mut right_dist_coeffs) =
        calibrate_single(&object_points, &right_points, image_size)?;
    let mut rotation = zero_mat(3, 3)?;
    let mut translation = zero_mat(3, 1)?;
    let mut essential = zero_mat(3, 3)?;
    let mut fundamental = zero_mat(3, 3)?;

    let rms = calib3d::stereo_calibrate(
        &object_points,
        &left_points,
        &right_points,
        &mut left_camera_matrix,
        &mut left_dist_coeffs,
        &mut right_camera_matrix,
        &mut right_dist_coeffs,
        image_size,
        &mut rotation,
        &mut translation,
        &mut essential,
        &mut fundamental,
        calib3d::CALIB_USE_INTRINSIC_GUESS | calib3d::CALIB_RATIONAL_MODEL,
        core::TermCriteria::new(core::TermCriteria_COUNT | core::TermCriteria_EPS, 100, 1e-6)?,
    )?;

    let translation_values = mat_values(&translation)?;
    Ok(CalibrationResult {
        rms,
        left_camera_matrix: mat_values(&left_camera_matrix)?,
        left_dist_coeffs: mat_values(&left_dist_coeffs)?,
        right_camera_matrix: mat_values(&right_camera_matrix)?,
        right_dist_coeffs: mat_values(&right_dist_coeffs)?,
        rotation: mat_values(&rotation)?,
        baseline: translation_values
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt(),
        translation: translation_values,
    })
}

fn calibrate_single(
    object_points: &core::Vector<core::Vector<core::Point3f>>,
    image_points: &core::Vector<core::Vector<core::Point2f>>,
    image_size: core::Size,
) -> Result<(core::Mat, core::Mat)> {
    let mut camera_matrix = core::Mat::eye(3, 3, core::CV_64F)?.to_mat()?;
    let mut dist_coeffs = zero_mat(1, 8)?;
    let mut rvecs = core::Vector::<core::Mat>::new();
    let mut tvecs = core::Vector::<core::Mat>::new();
    calib3d::calibrate_camera(
        object_points,
        image_points,
        image_size,
        &mut camera_matrix,
        &mut dist_coeffs,
        &mut rvecs,
        &mut tvecs,
        calib3d::CALIB_RATIONAL_MODEL,
        core::TermCriteria::new(core::TermCriteria_COUNT | core::TermCriteria_EPS, 100, 1e-6)?,
    )?;
    Ok((camera_matrix, dist_coeffs))
}

fn image_size(samples: &[CalibrationSample]) -> Result<core::Size> {
    let size = samples[0].image_size;
    if samples.iter().any(|sample| sample.image_size != size) {
        return Err(eyre!("captured samples have different image sizes"));
    }
    Ok(core::Size::new(size.x as i32, size.y as i32))
}

fn object_points(
    count: usize,
    marker_size_meters: f32,
) -> core::Vector<core::Vector<core::Point3f>> {
    let corners = core::Vector::from_slice(&[
        core::Point3f::new(0.0, 0.0, 0.0),
        core::Point3f::new(marker_size_meters, 0.0, 0.0),
        core::Point3f::new(marker_size_meters, marker_size_meters, 0.0),
        core::Point3f::new(0.0, marker_size_meters, 0.0),
    ]);
    let mut points = core::Vector::new();
    for _ in 0..count {
        points.push(corners.clone());
    }
    points
}

fn image_points(
    corners: impl Iterator<Item = [Pos2; 4]>,
) -> core::Vector<core::Vector<core::Point2f>> {
    let mut points = core::Vector::new();
    for corners in corners {
        points.push(core::Vector::from_slice(
            &corners.map(|point| core::Point2f::new(point.x, point.y)),
        ));
    }
    points
}

fn zero_mat(rows: i32, cols: i32) -> Result<core::Mat> {
    Ok(core::Mat::new_rows_cols_with_default(
        rows,
        cols,
        core::CV_64F,
        core::Scalar::all(0.0),
    )?)
}

fn mat_values(mat: &core::Mat) -> Result<Vec<f64>> {
    Ok(mat.data_typed::<f64>()?.to_vec())
}
