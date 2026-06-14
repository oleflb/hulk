use std::{collections::HashMap, f32::consts::PI};

use coordinate_systems::{Camera, Field, Ground, Pixel, Robot};
use itertools::Itertools;
use linear_algebra::{Isometry3, Point2};
use localization_factrs::VisualReprojectionAssociation;
use nalgebra::{Translation3, UnitQuaternion, Vector2, Vector3};
use projection::{camera_projection::InverseCameraProjection, intrinsic::Intrinsic};
use ros_z::Message;
use serde::{Deserialize, Serialize};
use types::field_dimensions::{FieldDimensions, Half, Side};

use crate::DetectedVisualFeatures;

const MIN_PAIR_DISTANCE: f32 = 0.25;
const PAIR_DISTANCE_GATE: f32 = 0.5;
const PAIR_DISTANCE_RELATIVE_GATE: f32 = 0.15;
const MAX_REFINED_HYPOTHESES: usize = 512;
const MAX_SCORED_HYPOTHESES: usize = 8_192;
const LOCAL_OPTIMIZATION_PASSES: usize = 2;
const SYMMETRY_EPSILON: f32 = 1.0e-4;
const POSE_HINT_TRANSLATION_TIE_EPSILON_SQUARED: f32 = 1.0e-6;
const CLASS_COUNT: usize = 4;

#[derive(Clone, Debug)]
pub(crate) struct GlobalLocalizer {
    config: GlobalLocalizerConfig,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, Message)]
#[serde(deny_unknown_fields)]
pub struct GlobalLocalizerConfig {
    /// Minimum accepted fixed associations for any published result.
    pub min_inliers: usize,
    /// Coarse maximum per-feature reprojection error in pixels for assignment.
    /// The factor graph performs the final metric optimization after a fixed
    /// assignment is selected.
    pub reprojection_gate: f32,
    /// Minimum top-1 to top-2 RMSE separation in pixels for distinct assignments.
    /// Field-symmetric alternatives are part of the same equivalence class.
    pub ambiguity_rmse_margin: f32,
}

impl Default for GlobalLocalizerConfig {
    fn default() -> Self {
        Self {
            min_inliers: 3,
            reprojection_gate: 100.0,
            ambiguity_rmse_margin: 0.1,
        }
    }
}

impl GlobalLocalizerConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.min_inliers < 3 {
            return Err("global_localizer.min_inliers must be at least 3".to_string());
        }
        if !self.reprojection_gate.is_finite() || self.reprojection_gate <= 0.0 {
            return Err("global_localizer.reprojection_gate must be finite and > 0".to_string());
        }
        if !self.ambiguity_rmse_margin.is_finite() || self.ambiguity_rmse_margin < 0.0 {
            return Err(
                "global_localizer.ambiguity_rmse_margin must be finite and >= 0".to_string(),
            );
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub(crate) struct GlobalLocalizationInput<'a> {
    pub visual_features: &'a DetectedVisualFeatures,
    pub field_dimensions: &'a FieldDimensions,
    pub ground_to_robot: Isometry3<Ground, Robot>,
    pub robot_to_camera: Isometry3<Robot, Camera>,
    pub camera_intrinsic: Intrinsic,
    pub pose_hint: Option<Isometry3<Robot, Field>>,
}

#[derive(Clone, Debug)]
pub(crate) enum GlobalLocalizationResult {
    /// A distinct non-symmetric assignment remains plausible.
    Ambiguous(FeatureAssociations),
    /// The assignment is unique in field coordinates.
    Unique(FeatureAssociations),
    /// The assignment is unique after quotienting the unavoidable 180 degree
    /// field symmetry. The selected branch is closest to the pose hint when
    /// available; otherwise it is the deterministic best-scoring branch.
    UniqueModuloSymmetry(FeatureAssociations),
}

impl GlobalLocalizationResult {
    pub fn associations(&self) -> &FeatureAssociations {
        match self {
            Self::Ambiguous(associations)
            | Self::Unique(associations)
            | Self::UniqueModuloSymmetry(associations) => associations,
        }
    }

    #[cfg(test)]
    pub fn is_unique(&self) -> bool {
        matches!(self, Self::Unique(_) | Self::UniqueModuloSymmetry(_))
    }

    pub fn unique_reprojection_associations(
        &self,
    ) -> Option<impl Iterator<Item = VisualReprojectionAssociation> + '_> {
        let associations = match self {
            Self::Unique(associations) | Self::UniqueModuloSymmetry(associations) => associations,
            Self::Ambiguous(_) => return None,
        };

        Some(
            associations
                .features
                .iter()
                .map(|association| VisualReprojectionAssociation {
                    detection: association.detection,
                    field_point: association.field_point.extend(0.0),
                }),
        )
    }
}

#[derive(Clone, Debug)]
pub(crate) struct FeatureAssociations {
    pub robot_to_field: Isometry3<Robot, Field>,
    pub features: Vec<FeatureAssociation>,
    pub score: GlobalLocalizationScore,
}

#[derive(Clone, Debug)]
pub(crate) struct FeatureAssociation {
    pub detection: Point2<Pixel>,
    pub field_point: Point2<Field>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum VisualFeatureClass {
    GoalPost,
    LSpot,
    TSpot,
    PenaltySpot,
}

impl VisualFeatureClass {
    const ALL: [Self; CLASS_COUNT] = [Self::GoalPost, Self::LSpot, Self::TSpot, Self::PenaltySpot];

    fn index(self) -> usize {
        match self {
            Self::GoalPost => 0,
            Self::LSpot => 1,
            Self::TSpot => 2,
            Self::PenaltySpot => 3,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct GlobalLocalizationScore {
    pub inliers: usize,
    pub reprojection_rmse: f32,
    pub total_cost: f32,
}

#[derive(Clone, Copy)]
struct Detection {
    id: usize,
    class: VisualFeatureClass,
    pixel: Point2<Pixel>,
    ground: Point2<Ground>,
}

#[derive(Clone, Copy)]
struct FieldFeature {
    id: usize,
    symmetric_id: usize,
    class: VisualFeatureClass,
    point: Point2<Field>,
}

#[derive(Clone, Copy)]
struct Association {
    detection: usize,
    feature: usize,
    error: f32,
}

struct Hypothesis {
    pose: Isometry3<Robot, Field>,
    associations: Vec<Association>,
    score: Score,
    priority: f32,
}

#[derive(Clone, Copy)]
struct Score {
    inliers: usize,
    cost: f32,
}

struct Problem {
    cfg: GlobalLocalizerConfig,
    k: Intrinsic,
    ground_to_robot: Isometry3<Ground, Robot>,
    robot_to_camera: Isometry3<Robot, Camera>,
    pose_hint: Option<Isometry3<Robot, Field>>,
    detections: Vec<Detection>,
    detections_by_class: [Vec<usize>; CLASS_COUNT],
    features: Vec<FieldFeature>,
    features_by_class: [Vec<usize>; CLASS_COUNT],
}

struct SearchResult {
    hypotheses: Vec<Hypothesis>,
    exhaustive: bool,
}

impl GlobalLocalizer {
    pub fn new(config: GlobalLocalizerConfig) -> Self {
        Self { config }
    }

    pub fn localize(&self, input: GlobalLocalizationInput<'_>) -> Option<GlobalLocalizationResult> {
        let problem = Problem::new(input, self.config)?;
        classify(search(&problem), &problem)
    }
}

impl Default for GlobalLocalizer {
    fn default() -> Self {
        Self::new(GlobalLocalizerConfig::default())
    }
}

impl Problem {
    fn new(input: GlobalLocalizationInput<'_>, cfg: GlobalLocalizerConfig) -> Option<Self> {
        if !valid_intrinsic(input.camera_intrinsic) {
            return None;
        }
        let k = input.camera_intrinsic;
        let ground_to_camera = input.robot_to_camera * input.ground_to_robot;
        let pixel_to_ground = projection::camera_projection::CameraProjection::new(
            ground_to_camera,
            input.camera_intrinsic,
        )
        .inverse(0.0);
        let detections = raw_detections(input.visual_features)
            .into_iter()
            .filter_map(|(class, pixel)| {
                let ground = back_project_to_ground(pixel, ground_to_camera, &pixel_to_ground)?;
                Some((class, pixel, ground))
            })
            .enumerate()
            .map(|(id, (class, pixel, ground))| Detection {
                id,
                class,
                pixel,
                ground,
            })
            .collect_vec();
        if detections.len() < cfg.min_inliers.max(3) {
            return None;
        }

        let mut detections_by_class = std::array::from_fn(|_| Vec::new());
        for detection in &detections {
            detections_by_class[detection.class.index()].push(detection.id);
        }

        let mut features = candidate_features(input.visual_features, input.field_dimensions);
        fill_symmetric_ids(&mut features);
        let mut features_by_class = std::array::from_fn(|_| Vec::new());
        for feature in &features {
            features_by_class[feature.class.index()].push(feature.id);
        }

        Some(Self {
            cfg,
            k,
            ground_to_robot: input.ground_to_robot,
            robot_to_camera: input.robot_to_camera,
            pose_hint: input.pose_hint,
            detections,
            detections_by_class,
            features,
            features_by_class,
        })
    }
}

fn valid_intrinsic(intrinsic: Intrinsic) -> bool {
    intrinsic.focals.x.is_finite()
        && intrinsic.focals.y.is_finite()
        && intrinsic.optical_center.x().is_finite()
        && intrinsic.optical_center.y().is_finite()
        && intrinsic.focals.x.abs() > 1.0e-6
        && intrinsic.focals.y.abs() > 1.0e-6
}

fn raw_detections(features: &DetectedVisualFeatures) -> Vec<(VisualFeatureClass, Point2<Pixel>)> {
    [
        (VisualFeatureClass::GoalPost, features.goalposts.as_slice()),
        (VisualFeatureClass::LSpot, features.l_spots.as_slice()),
        (VisualFeatureClass::TSpot, features.t_spots.as_slice()),
        (
            VisualFeatureClass::PenaltySpot,
            features.penalty_spots.as_slice(),
        ),
    ]
    .into_iter()
    .flat_map(|(class, detections)| detections.iter().copied().map(move |pixel| (class, pixel)))
    .collect()
}

fn back_project_to_ground(
    pixel: Point2<Pixel>,
    ground_to_camera: Isometry3<Ground, Camera>,
    pixel_to_ground: &InverseCameraProjection<Ground>,
) -> Option<Point2<Ground>> {
    let ground = pixel_to_ground.back_project_unchecked(pixel);
    let camera_point = ground_to_camera * ground;
    (camera_point
        .coords()
        .inner
        .iter()
        .all(|value| value.is_finite())
        && camera_point.z() > 1.0e-4)
        .then(|| ground.xy())
}

fn candidate_features(
    visual_features: &DetectedVisualFeatures,
    field: &FieldDimensions,
) -> Vec<FieldFeature> {
    let mut candidates = Vec::new();
    if !visual_features.goalposts.is_empty() {
        candidates.extend(
            goalpost_candidates(field)
                .into_iter()
                .map(|point| (VisualFeatureClass::GoalPost, point)),
        );
    }
    if !visual_features.l_spots.is_empty() {
        candidates.extend(
            l_spot_candidates(field)
                .into_iter()
                .map(|point| (VisualFeatureClass::LSpot, point)),
        );
    }
    if !visual_features.t_spots.is_empty() {
        candidates.extend(
            t_spot_candidates(field)
                .into_iter()
                .map(|point| (VisualFeatureClass::TSpot, point)),
        );
    }
    if !visual_features.penalty_spots.is_empty() {
        candidates.extend(
            penalty_spot_candidates(field)
                .into_iter()
                .map(|point| (VisualFeatureClass::PenaltySpot, point)),
        );
    }

    candidates
        .into_iter()
        .enumerate()
        .map(|(id, (class, point))| FieldFeature {
            id,
            symmetric_id: id,
            class,
            point,
        })
        .collect()
}

fn search(problem: &Problem) -> SearchResult {
    let mut pool = HypothesisPool::new(MAX_REFINED_HYPOTHESES);
    let mut scored = 0;
    let mut exhaustive = true;

    'pairs: for (left, right) in problem.detections.iter().tuple_combinations() {
        let ground_delta = (right.ground - left.ground).inner;
        let ground_distance = ground_delta.norm();
        if ground_distance < MIN_PAIR_DISTANCE {
            continue;
        }

        for (left_feature, right_feature) in problem.features_by_class[left.class.index()]
            .iter()
            .copied()
            .cartesian_product(
                problem.features_by_class[right.class.index()]
                    .iter()
                    .copied(),
            )
        {
            if left_feature == right_feature {
                continue;
            }

            let left_feature = &problem.features[left_feature];
            let right_feature = &problem.features[right_feature];
            let field_distance = (right_feature.point - left_feature.point).inner.norm();
            if !pair_distances_compatible(ground_distance, field_distance) {
                continue;
            }

            if scored >= MAX_SCORED_HYPOTHESES {
                exhaustive = false;
                break 'pairs;
            }
            scored += 1;

            if let Some(hypothesis) = hypothesis_from_pair(
                problem,
                left,
                right,
                left_feature,
                right_feature,
                ground_distance,
            ) && valid(&hypothesis, problem.cfg)
            {
                pool.push(hypothesis);
            }
        }
    }

    exhaustive &= !pool.truncated;
    let mut hypotheses = pool.into_sorted_vec();

    for _ in 0..LOCAL_OPTIMIZATION_PASSES {
        let mut refined = HypothesisPool::new(MAX_REFINED_HYPOTHESES);
        for hypothesis in hypotheses {
            let hypothesis = optimize(problem, hypothesis);
            if valid(&hypothesis, problem.cfg) {
                refined.push(hypothesis);
            }
        }
        exhaustive &= !refined.truncated;
        hypotheses = refined.into_sorted_vec();
    }

    SearchResult {
        hypotheses,
        exhaustive,
    }
}

struct HypothesisPool {
    hypotheses: Vec<Hypothesis>,
    capacity: usize,
    worst: Option<usize>,
    truncated: bool,
}

impl HypothesisPool {
    fn new(capacity: usize) -> Self {
        Self {
            hypotheses: Vec::with_capacity(capacity),
            capacity,
            worst: None,
            truncated: false,
        }
    }

    fn push(&mut self, hypothesis: Hypothesis) {
        if self.hypotheses.len() < self.capacity {
            self.hypotheses.push(hypothesis);
            if self.hypotheses.len() == self.capacity {
                self.refresh_worst();
            }
            return;
        }

        self.truncated = true;
        let Some(worst) = self.worst else {
            return;
        };
        if hypothesis.cmp_quality(&self.hypotheses[worst]).is_gt() {
            self.hypotheses[worst] = hypothesis;
            self.refresh_worst();
        }
    }

    fn refresh_worst(&mut self) {
        self.worst = self
            .hypotheses
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.cmp_quality(b))
            .map(|(index, _)| index);
    }

    fn into_sorted_vec(mut self) -> Vec<Hypothesis> {
        self.hypotheses.sort_by(|a, b| b.cmp_quality(a));
        self.hypotheses
    }
}

fn hypothesis_from_pair(
    problem: &Problem,
    left: &Detection,
    right: &Detection,
    left_feature: &FieldFeature,
    right_feature: &FieldFeature,
    ground_distance: f32,
) -> Option<Hypothesis> {
    let field_delta = (right_feature.point - left_feature.point).inner;
    let field_distance = field_delta.norm();
    if !pair_distances_compatible(ground_distance, field_distance) {
        return None;
    }

    let ground_delta = (right.ground - left.ground).inner;
    let yaw = field_delta.y.atan2(field_delta.x) - ground_delta.y.atan2(ground_delta.x);
    let pose = pose_from_ground_to_field(problem, left.ground, left_feature.point, yaw);
    score_pose(problem, pose)
}

fn pair_distances_compatible(ground_distance: f32, field_distance: f32) -> bool {
    if field_distance < MIN_PAIR_DISTANCE {
        return false;
    }
    let gate =
        PAIR_DISTANCE_GATE + PAIR_DISTANCE_RELATIVE_GATE * ground_distance.max(field_distance);
    (ground_distance - field_distance).abs() <= gate
}

fn pose_from_ground_to_field(
    problem: &Problem,
    ground_anchor: Point2<Ground>,
    field_anchor: Point2<Field>,
    yaw: f32,
) -> Isometry3<Robot, Field> {
    let rotated_ground = rotate(yaw, ground_anchor.coords().inner);
    let translation = field_anchor.coords().inner - rotated_ground;
    let ground_to_field = Isometry3::<Ground, Field>::wrap(nalgebra::Isometry3::from_parts(
        Translation3::new(translation.x, translation.y, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
    ));
    ground_to_field * problem.ground_to_robot.inverse()
}

fn rotate(angle: f32, point: Vector2<f32>) -> Vector2<f32> {
    let (sin, cos) = angle.sin_cos();
    Vector2::new(cos * point.x - sin * point.y, sin * point.x + cos * point.y)
}

fn score_pose(problem: &Problem, pose: Isometry3<Robot, Field>) -> Option<Hypothesis> {
    let field_to_camera = problem.robot_to_camera * pose.inverse();
    let projections = problem
        .features
        .iter()
        .map(|feature| project_feature(field_to_camera, *feature, problem.k))
        .collect_vec();
    let mut associations = VisualFeatureClass::ALL
        .into_iter()
        .flat_map(|class| assign_class(problem, class, &projections))
        .collect_vec();
    associations.sort_by_key(|a| a.detection);
    (!associations.is_empty()).then(|| Hypothesis::new(pose, associations, priority(problem, pose)))
}

fn assign_class(
    problem: &Problem,
    class: VisualFeatureClass,
    projections: &[Option<Point2<Pixel>>],
) -> Vec<Association> {
    let class = class.index();
    let detections = &problem.detections_by_class[class];
    let features = &problem.features_by_class[class];
    if detections.is_empty() || features.is_empty() {
        return Vec::new();
    }

    let edges = assignment_edges(problem, detections, features, projections)
        .into_iter()
        .filter(|edges| !edges.is_empty())
        .collect_vec();
    if edges.is_empty() {
        return Vec::new();
    }

    if features.len() > usize::BITS as usize {
        return Vec::new();
    }
    let mut nodes = vec![AssignmentNode {
        parent: None,
        association: None,
        score: Score {
            inliers: 0,
            cost: 0.0,
        },
    }];
    let mut states = HashMap::from([(0usize, 0usize)]);

    for edges in edges {
        let current_states = states
            .iter()
            .map(|(&mask, &state)| (mask, state))
            .collect_vec();
        for (mask, state) in current_states {
            for edge in &edges {
                let feature_mask = 1usize << edge.feature_index;
                if mask & feature_mask != 0 {
                    continue;
                }

                let association = Association {
                    detection: edge.detection,
                    feature: edge.feature,
                    error: edge.error,
                };
                let score = nodes[state].score.with(edge.error);
                let next_mask = mask | feature_mask;
                if states
                    .get(&next_mask)
                    .is_none_or(|old| score.better_than(nodes[*old].score))
                {
                    let node = nodes.len();
                    nodes.push(AssignmentNode {
                        parent: Some(state),
                        association: Some(association),
                        score,
                    });
                    states.insert(next_mask, node);
                }
            }
        }
    }

    states
        .into_values()
        .max_by(|left, right| nodes[*left].score.cmp_quality(nodes[*right].score))
        .map_or_else(Vec::new, |best| assignment_associations(&nodes, best))
}

struct AssignmentEdge {
    detection: usize,
    feature: usize,
    feature_index: usize,
    error: f32,
}

fn assignment_edges(
    problem: &Problem,
    detections: &[usize],
    features: &[usize],
    projections: &[Option<Point2<Pixel>>],
) -> Vec<Vec<AssignmentEdge>> {
    detections
        .iter()
        .copied()
        .map(|detection| {
            features
                .iter()
                .copied()
                .enumerate()
                .filter_map(|(feature_index, feature)| {
                    let projection = projections[feature]?;
                    let error = (projection - problem.detections[detection].pixel)
                        .inner
                        .norm();
                    (error <= problem.cfg.reprojection_gate).then_some(AssignmentEdge {
                        detection,
                        feature,
                        feature_index,
                        error,
                    })
                })
                .collect()
        })
        .collect()
}

struct AssignmentNode {
    parent: Option<usize>,
    association: Option<Association>,
    score: Score,
}

fn assignment_associations(nodes: &[AssignmentNode], mut node: usize) -> Vec<Association> {
    let mut associations = Vec::with_capacity(nodes[node].score.inliers);
    while let Some(parent) = nodes[node].parent {
        if let Some(association) = nodes[node].association {
            associations.push(association);
        }
        node = parent;
    }
    associations.reverse();
    associations
}

impl Hypothesis {
    fn new(pose: Isometry3<Robot, Field>, associations: Vec<Association>, priority: f32) -> Self {
        let cost = associations.iter().map(|a| a.error.powi(2)).sum();
        Self {
            pose,
            score: Score {
                inliers: associations.len(),
                cost,
            },
            associations,
            priority,
        }
    }

    fn cmp_quality(&self, other: &Self) -> std::cmp::Ordering {
        self.score
            .cmp_quality(other.score)
            .then_with(|| other.priority.total_cmp(&self.priority))
    }
}

fn optimize(problem: &Problem, hypothesis: Hypothesis) -> Hypothesis {
    if hypothesis.associations.len() < 2 {
        return hypothesis;
    }

    let Some(pose) = fit_pose(problem, &hypothesis.associations) else {
        return hypothesis;
    };
    let Some(refined) = score_pose(problem, pose) else {
        return hypothesis;
    };
    if refined.score.better_than(hypothesis.score) {
        refined
    } else {
        hypothesis
    }
}

fn fit_pose(problem: &Problem, associations: &[Association]) -> Option<Isometry3<Robot, Field>> {
    let count = associations.len() as f32;
    let (ground_center, field_center) = associations.iter().fold(
        (Vector2::zeros(), Vector2::zeros()),
        |(ground_sum, field_sum), association| {
            (
                ground_sum
                    + problem.detections[association.detection]
                        .ground
                        .coords()
                        .inner,
                field_sum + problem.features[association.feature].point.coords().inner,
            )
        },
    );
    let ground_center = ground_center / count;
    let field_center = field_center / count;

    let (sin_sum, cos_sum) = associations
        .iter()
        .map(|association| {
            let ground = problem.detections[association.detection]
                .ground
                .coords()
                .inner
                - ground_center;
            let field = problem.features[association.feature].point.coords().inner - field_center;
            (
                ground.x * field.y - ground.y * field.x,
                ground.x * field.x + ground.y * field.y,
            )
        })
        .fold((0.0, 0.0), |(sin_acc, cos_acc), (sin, cos)| {
            (sin_acc + sin, cos_acc + cos)
        });
    if sin_sum.abs() + cos_sum.abs() <= 1.0e-6 {
        return None;
    }

    let yaw = sin_sum.atan2(cos_sum);
    let translation = field_center - rotate(yaw, ground_center);
    let ground_to_field = Isometry3::<Ground, Field>::wrap(nalgebra::Isometry3::from_parts(
        Translation3::new(translation.x, translation.y, 0.0),
        UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
    ));
    Some(ground_to_field * problem.ground_to_robot.inverse())
}

impl Score {
    fn with(self, error: f32) -> Self {
        Self {
            inliers: self.inliers + 1,
            cost: self.cost + error.powi(2),
        }
    }

    fn rmse(self) -> f32 {
        if self.inliers == 0 {
            f32::INFINITY
        } else {
            (self.cost / self.inliers as f32).sqrt()
        }
    }

    fn better_than(self, other: Self) -> bool {
        self.inliers > other.inliers || (self.inliers == other.inliers && self.cost < other.cost)
    }

    fn cmp_quality(self, other: Self) -> std::cmp::Ordering {
        self.inliers
            .cmp(&other.inliers)
            .then_with(|| other.cost.total_cmp(&self.cost))
    }
}

fn valid(h: &Hypothesis, cfg: GlobalLocalizerConfig) -> bool {
    h.score.inliers >= cfg.min_inliers && h.score.rmse() <= cfg.reprojection_gate
}

fn classify(mut result: SearchResult, problem: &Problem) -> Option<GlobalLocalizationResult> {
    result.hypotheses.sort_by(|a, b| b.cmp_quality(a));
    let exhaustive = result.exhaustive;
    let mut hypotheses = result.hypotheses.into_iter();
    let mut selected = hypotheses.next()?;
    let best_score = selected.score;
    let best_ids = association_ids(&selected).collect_vec();
    let best_symmetric_ids = symmetric_association_ids(&selected, &problem.features).collect_vec();
    let mut has_symmetric_alternative = false;
    let mut best_distinct_score = None;

    for alternative in hypotheses.filter(|h| h.score.inliers == best_score.inliers) {
        let alternative_ids = association_ids(&alternative).collect_vec();
        if alternative_ids == best_symmetric_ids {
            has_symmetric_alternative = true;
        } else if alternative_ids != best_ids {
            best_distinct_score = Some(alternative.score);
            break;
        }

        if let Some(hint) = problem.pose_hint
            && closer_to_hint(alternative.pose, selected.pose, hint)
        {
            selected = alternative;
        }
    }

    let selected = select_branch(selected, problem);
    let has_symmetry_equivalent = has_nontrivial_symmetric_branch(&selected, &problem.features);
    let associations = to_public(selected, problem);
    if !exhaustive
        || best_distinct_score.is_some_and(|score| !separated(best_score, score, problem.cfg))
    {
        Some(GlobalLocalizationResult::Ambiguous(associations))
    } else if has_symmetric_alternative || has_symmetry_equivalent {
        Some(GlobalLocalizationResult::UniqueModuloSymmetry(associations))
    } else {
        Some(GlobalLocalizationResult::Unique(associations))
    }
}

fn has_nontrivial_symmetric_branch(h: &Hypothesis, features: &[FieldFeature]) -> bool {
    h.associations
        .iter()
        .any(|a| features[a.feature].symmetric_id != a.feature)
}

fn separated(best: Score, second: Score, cfg: GlobalLocalizerConfig) -> bool {
    second.rmse() > best.rmse() + cfg.ambiguity_rmse_margin
}

fn association_ids(h: &Hypothesis) -> impl Iterator<Item = (usize, usize)> + '_ {
    h.associations.iter().map(|a| (a.detection, a.feature))
}

fn symmetric_association_ids<'a>(
    h: &'a Hypothesis,
    features: &'a [FieldFeature],
) -> impl Iterator<Item = (usize, usize)> + 'a {
    h.associations
        .iter()
        .map(|a| (a.detection, features[a.feature].symmetric_id))
}

fn select_branch(h: Hypothesis, problem: &Problem) -> Hypothesis {
    let Some(hint) = problem.pose_hint else {
        return h;
    };
    let symmetric = symmetric_hypothesis(&h, &problem.features);
    if closer_to_hint(symmetric.pose, h.pose, hint) {
        symmetric
    } else {
        h
    }
}

fn symmetric_hypothesis(h: &Hypothesis, features: &[FieldFeature]) -> Hypothesis {
    let field_symmetry =
        Isometry3::<Field, Field>::wrap(nalgebra::Isometry3::rotation(Vector3::new(0.0, 0.0, PI)));
    let associations = h
        .associations
        .iter()
        .map(|a| Association {
            feature: features[a.feature].symmetric_id,
            ..*a
        })
        .collect_vec();
    Hypothesis {
        pose: field_symmetry * h.pose,
        associations,
        score: h.score,
        priority: h.priority,
    }
}

fn to_public(h: Hypothesis, problem: &Problem) -> FeatureAssociations {
    FeatureAssociations {
        robot_to_field: h.pose,
        features: h
            .associations
            .into_iter()
            .map(|a| FeatureAssociation {
                detection: problem.detections[a.detection].pixel,
                field_point: problem.features[a.feature].point,
            })
            .collect(),
        score: GlobalLocalizationScore {
            inliers: h.score.inliers,
            reprojection_rmse: h.score.rmse(),
            total_cost: h.score.cost,
        },
    }
}

fn project_feature(
    field_to_camera: Isometry3<Field, Camera>,
    feature: FieldFeature,
    k: Intrinsic,
) -> Option<Point2<Pixel>> {
    let camera_point = field_to_camera * feature.point.extend(0.0);
    (camera_point.z().is_finite() && camera_point.z() > 1.0e-4)
        .then(|| k.project(camera_point.coords()))
}

fn priority(problem: &Problem, pose: Isometry3<Robot, Field>) -> f32 {
    problem.pose_hint.map_or(0.0, |hint| {
        translation_distance_squared(pose, hint) + rotation_distance(pose, hint).powi(2)
    })
}

fn closer_to_hint(
    candidate: Isometry3<Robot, Field>,
    current: Isometry3<Robot, Field>,
    hint: Isometry3<Robot, Field>,
) -> bool {
    let candidate_translation = translation_distance_squared(candidate, hint);
    let current_translation = translation_distance_squared(current, hint);
    if (candidate_translation - current_translation).abs()
        > POSE_HINT_TRANSLATION_TIE_EPSILON_SQUARED
    {
        return candidate_translation < current_translation;
    }

    rotation_distance(candidate, hint) < rotation_distance(current, hint)
}

fn translation_distance_squared(a: Isometry3<Robot, Field>, b: Isometry3<Robot, Field>) -> f32 {
    (a.inner.translation.vector - b.inner.translation.vector).norm_squared()
}

fn rotation_distance(a: Isometry3<Robot, Field>, b: Isometry3<Robot, Field>) -> f32 {
    a.inner.rotation.angle_to(&b.inner.rotation)
}

fn fill_symmetric_ids(features: &mut [FieldFeature]) {
    for i in 0..features.len() {
        if let Some(other) = features.iter().find(|f| {
            f.class == features[i].class
                && (f.point.x() + features[i].point.x()).abs() <= SYMMETRY_EPSILON
                && (f.point.y() + features[i].point.y()).abs() <= SYMMETRY_EPSILON
        }) {
            features[i].symmetric_id = other.id;
        }
    }
}

fn goalpost_candidates(field: &FieldDimensions) -> Vec<Point2<Field>> {
    [Half::Opponent, Half::Own]
        .into_iter()
        .cartesian_product([Side::Left, Side::Right])
        .map(|(half, side)| field.goal_post(half, side))
        .collect()
}

fn l_spot_candidates(field: &FieldDimensions) -> Vec<Point2<Field>> {
    [Half::Opponent, Half::Own]
        .into_iter()
        .cartesian_product([Side::Left, Side::Right])
        .flat_map(|(half, side)| {
            [
                field.corner(half, side),
                field.goal_box_corner(half, side),
                field.penalty_box_corner(half, side),
            ]
        })
        .collect()
}

fn t_spot_candidates(field: &FieldDimensions) -> Vec<Point2<Field>> {
    [Side::Left, Side::Right]
        .into_iter()
        .map(|side| field.t_crossing(side))
        .chain(
            [Half::Opponent, Half::Own]
                .into_iter()
                .cartesian_product([Side::Left, Side::Right])
                .flat_map(|(half, side)| {
                    [
                        field.goal_box_goal_line_intersection(half, side),
                        field.penalty_box_goal_line_intersection(half, side),
                    ]
                }),
        )
        .collect()
}

fn penalty_spot_candidates(field: &FieldDimensions) -> Vec<Point2<Field>> {
    [Half::Opponent, Half::Own]
        .into_iter()
        .map(|half| field.penalty_spot(half))
        .collect()
}

#[cfg(test)]
mod tests {
    use coordinate_systems::{Camera, Field, Ground, Robot};
    use linear_algebra::{IntoTransform, Isometry2, point, vector};
    use nalgebra::UnitQuaternion;

    use super::*;

    fn input<'a>(
        visual_features: &'a DetectedVisualFeatures,
        field_dimensions: &'a FieldDimensions,
        intrinsic: Intrinsic,
    ) -> GlobalLocalizationInput<'a> {
        GlobalLocalizationInput {
            visual_features,
            field_dimensions,
            ground_to_robot: Isometry3::identity(),
            robot_to_camera: robot_to_camera(),
            camera_intrinsic: intrinsic,
            pose_hint: Some(nalgebra::Isometry3::identity().framed_transform::<Robot, Field>()),
        }
    }

    fn camera_intrinsic() -> Intrinsic {
        Intrinsic::new(nalgebra::vector![100.0, 100.0], point![320.0, 240.0])
    }

    fn robot_to_camera() -> Isometry3<Robot, Camera> {
        nalgebra::Isometry3::translation(0.0, 0.0, 8.0).framed_transform()
    }

    fn ground_to_camera() -> Isometry3<Ground, Camera> {
        robot_to_camera() * Isometry3::identity()
    }

    fn ground_to_field() -> Isometry2<Ground, Field> {
        Isometry2::from_parts(vector![1.0, -0.6], 0.42)
    }

    fn robot_to_field_from_ground_to_field(
        ground_to_field: Isometry2<Ground, Field>,
        ground_to_robot: Isometry3<Ground, Robot>,
    ) -> Isometry3<Robot, Field> {
        let translation = ground_to_field.inner.translation.vector;
        let yaw = ground_to_field.inner.rotation.angle();
        let ground_to_field = Isometry3::<Ground, Field>::wrap(nalgebra::Isometry3::from_parts(
            Translation3::new(translation.x, translation.y, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
        ));
        ground_to_field * ground_to_robot.inverse()
    }

    fn project_points(
        ground_to_field: Isometry2<Ground, Field>,
        points: impl IntoIterator<Item = Point2<Field>>,
    ) -> Vec<Point2<Pixel>> {
        let field_to_ground = ground_to_field.inverse();
        let intrinsic = camera_intrinsic();
        let ground_to_camera = ground_to_camera();
        points
            .into_iter()
            .map(|point| {
                let camera_point = ground_to_camera * (field_to_ground * point).extend(0.0);
                assert!(camera_point.z() > 1.0e-4);
                intrinsic.project(camera_point.coords())
            })
            .collect()
    }

    #[test]
    fn no_result_before_minimum_detection_count() {
        let localizer = GlobalLocalizer::default();
        let config = GlobalLocalizerConfig::default();
        let field_dimensions = FieldDimensions::SPL_2025;
        let intrinsic = Intrinsic::new(nalgebra::vector![1.0, 1.0], point![0.0, 0.0]);
        let visual_features = DetectedVisualFeatures {
            goalposts: vec![point![0.0, 0.0]; config.min_inliers - 1],
            ..Default::default()
        };

        assert!(
            localizer
                .localize(input(&visual_features, &field_dimensions, intrinsic))
                .is_none()
        );
    }

    #[test]
    fn invalid_intrinsics_return_no_result() {
        let localizer = GlobalLocalizer::default();
        let config = GlobalLocalizerConfig::default();
        let field_dimensions = FieldDimensions::SPL_2025;
        let intrinsic = Intrinsic::new(nalgebra::vector![0.0, 1.0], point![0.0, 0.0]);
        let visual_features = DetectedVisualFeatures {
            goalposts: vec![point![0.0, 0.0]; config.min_inliers],
            ..Default::default()
        };

        assert!(
            localizer
                .localize(input(&visual_features, &field_dimensions, intrinsic))
                .is_none()
        );
    }

    #[test]
    fn candidate_sets_match_field_feature_classes() {
        let field = FieldDimensions::SPL_2025;
        assert_eq!(goalpost_candidates(&field).len(), 4);
        assert_eq!(l_spot_candidates(&field).len(), 12);
        assert_eq!(t_spot_candidates(&field).len(), 10);
        assert_eq!(penalty_spot_candidates(&field).len(), 2);
    }

    #[test]
    fn result_helpers_expose_associations_and_uniqueness() {
        fn empty_associations() -> FeatureAssociations {
            FeatureAssociations {
                robot_to_field: Isometry3::identity(),
                features: Vec::new(),
                score: GlobalLocalizationScore {
                    inliers: 0,
                    reprojection_rmse: 0.0,
                    total_cost: 0.0,
                },
            }
        }

        let ambiguous = GlobalLocalizationResult::Ambiguous(empty_associations());
        let unique = GlobalLocalizationResult::Unique(empty_associations());
        let unique_modulo_symmetry =
            GlobalLocalizationResult::UniqueModuloSymmetry(empty_associations());

        assert!(!ambiguous.is_unique());
        assert!(unique.is_unique());
        assert!(unique_modulo_symmetry.is_unique());
        assert_eq!(ambiguous.associations().features.len(), 0);
        assert_eq!(unique.associations().features.len(), 0);
        assert_eq!(unique_modulo_symmetry.associations().features.len(), 0);
    }

    #[test]
    fn back_projection_recovers_ground_points() {
        let intrinsic = camera_intrinsic();
        let ground_to_camera = ground_to_camera();
        let pixel_to_ground =
            projection::camera_projection::CameraProjection::new(ground_to_camera, intrinsic)
                .inverse(0.0);
        let ground = point![<Ground>, 2.0, 0.4];
        let pixel = intrinsic.project((ground_to_camera * ground.extend(0.0)).coords());

        let recovered = back_project_to_ground(pixel, ground_to_camera, &pixel_to_ground)
            .expect("pixel should back-project");

        assert!((recovered - ground).inner.norm() < 1.0e-4);
    }

    #[test]
    fn recovers_mixed_feature_associations_with_pose_hint() {
        let field = FieldDimensions::SPL_2025;
        let ground_to_field = ground_to_field();
        let expected = robot_to_field_from_ground_to_field(ground_to_field, Isometry3::identity());
        let camera_intrinsic = camera_intrinsic();
        let mut goalposts = project_points(
            ground_to_field,
            [
                goalpost_candidates(&field)[0],
                goalpost_candidates(&field)[2],
            ],
        );
        let mut l_spots = project_points(
            ground_to_field,
            [l_spot_candidates(&field)[0], l_spot_candidates(&field)[5]],
        );
        let mut t_spots = project_points(ground_to_field, [t_spot_candidates(&field)[0]]);
        let penalty_spots = project_points(ground_to_field, [penalty_spot_candidates(&field)[0]]);
        goalposts.reverse();
        l_spots.swap(0, 1);
        t_spots.reverse();
        let visual_features = DetectedVisualFeatures {
            goalposts,
            l_spots,
            t_spots,
            penalty_spots,
        };
        let localizer = GlobalLocalizer::new(GlobalLocalizerConfig {
            ambiguity_rmse_margin: 1.0e-3,
            ..Default::default()
        });

        let result = localizer
            .localize(GlobalLocalizationInput {
                visual_features: &visual_features,
                field_dimensions: &field,
                ground_to_robot: Isometry3::identity(),
                robot_to_camera: robot_to_camera(),
                camera_intrinsic,
                pose_hint: Some(expected),
            })
            .expect("synthetic detections should localize");

        let associations = result.associations();
        assert!(matches!(
            &result,
            GlobalLocalizationResult::Unique(_) | GlobalLocalizationResult::UniqueModuloSymmetry(_)
        ));
        assert_eq!(associations.score.inliers, 6);
        assert!(associations.score.reprojection_rmse < 1.0e-3);
        assert_eq!(associations.features.len(), 6);
        assert!(
            (associations.robot_to_field.inner.translation.vector
                - expected.inner.translation.vector)
                .norm()
                < 1.0e-3
        );
        assert!(
            associations
                .robot_to_field
                .inner
                .rotation
                .angle_to(&expected.inner.rotation)
                < 1.0e-3
        );
    }

    #[test]
    fn three_features_with_penalty_spot_are_unique_modulo_symmetry() {
        let field = FieldDimensions::SPL_2025;
        let ground_to_field = ground_to_field();
        let expected = robot_to_field_from_ground_to_field(ground_to_field, Isometry3::identity());
        let camera_intrinsic = camera_intrinsic();
        let visual_features = DetectedVisualFeatures {
            l_spots: project_points(ground_to_field, [l_spot_candidates(&field)[0]]),
            t_spots: project_points(ground_to_field, [t_spot_candidates(&field)[1]]),
            penalty_spots: project_points(ground_to_field, [penalty_spot_candidates(&field)[0]]),
            ..Default::default()
        };
        let localizer = GlobalLocalizer::new(GlobalLocalizerConfig {
            ambiguity_rmse_margin: 1.0e-3,
            ..Default::default()
        });

        let result = localizer
            .localize(GlobalLocalizationInput {
                visual_features: &visual_features,
                field_dimensions: &field,
                ground_to_robot: Isometry3::identity(),
                robot_to_camera: robot_to_camera(),
                camera_intrinsic,
                pose_hint: Some(expected),
            })
            .expect("synthetic detections should localize");

        assert!(matches!(
            &result,
            GlobalLocalizationResult::UniqueModuloSymmetry(_)
        ));
        assert_eq!(result.associations().score.inliers, 3);
    }

    #[test]
    fn four_goalposts_are_unique_modulo_symmetry() {
        let field = FieldDimensions::SPL_2025;
        let ground_to_field = ground_to_field();
        let camera_intrinsic = camera_intrinsic();
        let mut detections = project_points(ground_to_field, goalpost_candidates(&field));
        detections.swap(0, 2);
        detections.swap(1, 3);
        let visual_features = DetectedVisualFeatures {
            goalposts: detections,
            ..Default::default()
        };
        let localizer = GlobalLocalizer::new(GlobalLocalizerConfig {
            min_inliers: 4,
            ambiguity_rmse_margin: 1.0e-3,
            ..Default::default()
        });

        let result = localizer
            .localize(GlobalLocalizationInput {
                visual_features: &visual_features,
                field_dimensions: &field,
                ground_to_robot: Isometry3::identity(),
                robot_to_camera: robot_to_camera(),
                camera_intrinsic,
                pose_hint: None,
            })
            .expect("synthetic detections should localize");

        assert!(matches!(
            &result,
            GlobalLocalizationResult::UniqueModuloSymmetry(_)
        ));
        assert_eq!(result.associations().score.inliers, 4);
    }

    #[test]
    fn pose_hint_selects_branch_for_unique_modulo_symmetry() {
        let field = FieldDimensions::SPL_2025;
        let ground_to_field = ground_to_field();
        let camera_intrinsic = camera_intrinsic();
        let visual_features = DetectedVisualFeatures {
            l_spots: project_points(ground_to_field, [l_spot_candidates(&field)[0]]),
            t_spots: project_points(ground_to_field, [t_spot_candidates(&field)[1]]),
            penalty_spots: project_points(ground_to_field, [penalty_spot_candidates(&field)[0]]),
            ..Default::default()
        };
        let expected = robot_to_field_from_ground_to_field(ground_to_field, Isometry3::identity());
        let localizer = GlobalLocalizer::new(GlobalLocalizerConfig {
            ambiguity_rmse_margin: 1.0e-3,
            ..Default::default()
        });

        let result = localizer
            .localize(GlobalLocalizationInput {
                visual_features: &visual_features,
                field_dimensions: &field,
                ground_to_robot: Isometry3::identity(),
                robot_to_camera: robot_to_camera(),
                camera_intrinsic,
                pose_hint: Some(expected),
            })
            .expect("synthetic detections should localize");

        assert!(matches!(
            &result,
            GlobalLocalizationResult::UniqueModuloSymmetry(_)
        ));
        assert!(
            result
                .associations()
                .robot_to_field
                .inner
                .rotation
                .angle_to(&expected.inner.rotation)
                < 1.0e-3
        );
    }
}
