use std::{
    cmp::Reverse,
    collections::{BinaryHeap, HashMap, HashSet},
    sync::{Arc, Mutex, OnceLock},
};

use ::types::field_dimensions::FieldDimensions;
use coordinate_systems::{Camera, Field, Ground, Pixel, Robot};
use hungarian_algorithm::AssignmentProblem;
use linear_algebra::{Isometry3, Point2, point};
use nalgebra::{Similarity2, Translation3, Vector2};
use ndarray::Array2;
use ordered_float::NotNan;
use projection::intrinsic::Intrinsic;

use crate::{DetectedVisualFeature, DetectedVisualFeatures};

use super::{
    CLASS_COUNT, FeatureAssociation, FeatureAssociations, GLOBAL_LOCALIZER_MAX_DETECTIONS,
    GlobalAssociationConfig, GlobalLocalizationDebugAssociation, GlobalLocalizationDebugDetection,
    GlobalLocalizationDebugProjection, GlobalLocalizationDetailedDebug,
    GlobalLocalizationDetailedStatus, GlobalLocalizationScore, PoseHintAssociationConfig,
    VisualFeatureClass,
    map::{LandmarkMap, MapTriplet, MapTripletBin, triplet_bin},
};

mod bounds;
mod certification;
mod cheap;
mod fitting;
mod output;
mod problem;
mod search;
mod seeding;
#[cfg(test)]
mod tests;
mod types;

use bounds::*;
use certification::*;
use cheap::*;
use fitting::*;
use output::*;
use problem::*;
use search::*;
use seeding::*;
use types::*;
pub(crate) use types::{GlobalLocalizationInput, GlobalLocalizationResult};

pub(crate) fn solve(
    input: GlobalLocalizationInput<'_>,
    config: GlobalAssociationConfig,
) -> Option<GlobalLocalizationResult> {
    let problem = Problem::new(input, config)?;
    solve_problem(&problem)
}

pub(crate) fn solve_detailed(
    input: GlobalLocalizationInput<'_>,
    config: GlobalAssociationConfig,
) -> Option<GlobalLocalizationDetailedDebug> {
    let problem = Problem::new(input, config)?;
    let result = solve_problem(&problem)?;
    Some(detailed_debug_from_result(&result, &problem))
}

pub(crate) fn associate_with_pose_hint(
    input: GlobalLocalizationInput<'_>,
    global_config: GlobalAssociationConfig,
    pose_config: PoseHintAssociationConfig,
) -> Vec<FeatureAssociation> {
    if !pose_config.enabled || !valid_intrinsic(input.camera_intrinsic) {
        return Vec::new();
    }
    let Some(robot_to_field) = input.pose_hint else {
        return Vec::new();
    };

    let map = cached_landmark_map(input.field_dimensions, global_config.min_map_baseline);
    let ground_to_camera = input.robot_to_camera * input.ground_to_robot;
    let camera_to_ground = ground_to_camera.inverse();
    let camera_origin = camera_to_ground.inner.translation.vector;
    if !camera_origin.iter().all(|value| value.is_finite())
        || camera_origin.z.abs() <= HORIZON_EPSILON
    {
        return Vec::new();
    }

    let detection_set = detection_points(
        input.visual_features,
        &map,
        &camera_to_ground,
        input.camera_intrinsic,
        global_config,
    );
    let ground_to_field = robot_to_field * input.ground_to_robot;
    let field_to_camera =
        field_to_camera_from_robot_to_field(input.robot_to_camera, robot_to_field);
    let mut options = detection_set
        .detections
        .iter()
        .enumerate()
        .filter_map(|(detection_index, detection)| {
            pose_hint_option(
                &map,
                detection_index,
                detection,
                ground_to_field,
                field_to_camera,
                input.camera_intrinsic,
                pose_config,
            )
        })
        .collect::<Vec<_>>();

    options.sort_by(|left, right| {
        left.residual
            .total_cmp(&right.residual)
            .then_with(|| right.confidence.total_cmp(&left.confidence))
            .then_with(|| left.detection_index.cmp(&right.detection_index))
            .then_with(|| left.landmark_id.cmp(&right.landmark_id))
    });

    let mut used_detections = HashSet::new();
    let mut used_landmarks = HashSet::new();
    let mut associations = Vec::new();
    for option in options {
        if used_detections.contains(&option.detection_index)
            || used_landmarks.contains(&option.landmark_id)
        {
            continue;
        }
        used_detections.insert(option.detection_index);
        used_landmarks.insert(option.landmark_id);
        let Some(detection) = detection_set.detections.get(option.detection_index) else {
            continue;
        };
        let Some(landmark) = map.landmarks.get(option.landmark_id) else {
            continue;
        };
        associations.push(FeatureAssociation {
            detection_id: detection.id,
            landmark_id: landmark.id,
            detection: detection.pixel,
            field_point: landmark.xy,
        });
    }

    associations
}

#[derive(Clone, Copy)]
struct PoseHintOption {
    detection_index: usize,
    landmark_id: usize,
    confidence: f32,
    residual: f32,
}

fn pose_hint_option(
    map: &LandmarkMap,
    detection_index: usize,
    detection: &DetectionPoint,
    ground_to_field: Isometry3<Ground, Field>,
    field_to_camera: Isometry3<Field, Camera>,
    intrinsic: Intrinsic,
    config: PoseHintAssociationConfig,
) -> Option<PoseHintOption> {
    let predicted = ground_to_field * detection.ground.extend(0.0);
    let predicted = point![<Field>, predicted.x(), predicted.y()];

    let mut best: Option<(usize, f32)> = None;
    let mut second_best = f32::INFINITY;
    for &landmark_id in &map.landmarks_by_class[detection.class.index()] {
        let landmark = map.landmarks.get(landmark_id)?;
        let residual = (landmark.xy - predicted).inner.norm();
        if !residual.is_finite() {
            continue;
        }
        if best.is_none_or(|(_, best_residual)| residual < best_residual) {
            if let Some((_, best_residual)) = best {
                second_best = best_residual;
            }
            best = Some((landmark_id, residual));
        } else if residual < second_best {
            second_best = residual;
        }
    }

    let (landmark_id, residual) = best?;
    if residual > config.association_gate || second_best - residual < config.second_best_margin {
        return None;
    }

    let landmark = map.landmarks.get(landmark_id)?;
    let projected = project_field_point(field_to_camera, intrinsic, landmark.xy)?;
    let reprojection_error = (projected - detection.pixel).inner.norm();
    if !reprojection_error.is_finite() || reprojection_error > config.max_reprojection_error_px {
        return None;
    }

    Some(PoseHintOption {
        detection_index,
        landmark_id,
        confidence: detection.confidence,
        residual,
    })
}
