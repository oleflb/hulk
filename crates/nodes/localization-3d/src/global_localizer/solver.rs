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
    GlobalLocalizationDebugAssociation, GlobalLocalizationDebugDetection,
    GlobalLocalizationDebugProjection, GlobalLocalizationDetailedDebug,
    GlobalLocalizationDetailedStatus, GlobalLocalizationScore, GlobalLocalizerConfig,
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
    config: GlobalLocalizerConfig,
) -> Option<GlobalLocalizationResult> {
    let problem = Problem::new(input, config)?;
    solve_problem(&problem)
}

pub(crate) fn solve_detailed(
    input: GlobalLocalizationInput<'_>,
    config: GlobalLocalizerConfig,
) -> Option<GlobalLocalizationDetailedDebug> {
    let problem = Problem::new(input, config)?;
    let result = solve_problem(&problem)?;
    Some(detailed_debug_from_result(&result, &problem))
}
