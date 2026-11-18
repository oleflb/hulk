use coordinate_systems::{Camera, Ground, ImuReference, Robot};
use linear_algebra::{Isometry3, Orientation2, Orientation3, Rotation3};
use projection::intrinsic::Intrinsic;
use ros_z::time::Time;
pub use types::localization::HeadingConstraint;
use types::{
    field_dimensions::FieldDimensions,
    localization::{LocalizationState, LocalizationStatus},
    visual_localization::{
        AssociationGeometry, FieldMarkAssociation, GlobalLocalizationDebug, VisualAssociationSource,
    },
};

use crate::{
    DetectedVisualFeature, DetectedVisualFeatures, FieldMarkAssociationParameters,
    GlobalLocalizerParameters, VisualFeatureClass, features::filter_detections,
    global_association::solver, map::LandmarkMap, tracking,
};

pub(crate) struct PreparedAssociation {
    pub map: LandmarkMap,
    pub detections: Vec<(usize, VisualFeatureClass, DetectedVisualFeature)>,
}

impl PreparedAssociation {
    pub fn new(
        features: &DetectedVisualFeatures,
        field: &FieldDimensions,
        config: GlobalLocalizerParameters,
    ) -> Option<Self> {
        if features.supported_feature_count() > config.max_input_detections {
            return None;
        }
        let map = LandmarkMap::new(field);
        if map
            .landmarks
            .iter()
            .any(|landmark| !landmark.xy.coords().inner.iter().all(|x| x.is_finite()))
        {
            return None;
        }
        let detections = filter_detections(features, &map, config)?;
        Some(Self { map, detections })
    }
}

/// Exposure-time sensor inputs and an optional coherent tracking snapshot.
#[derive(Clone, Copy)]
pub struct AssociationInput<'a> {
    pub status: &'a LocalizationStatus,
    pub attitude: Option<Orientation3<ImuReference>>,
    pub tracking: Option<&'a AssociationGeometry>,
    pub visual_features: &'a DetectedVisualFeatures,
    pub robot_to_camera: Isometry3<Robot, Camera>,
    pub camera_intrinsic: Intrinsic,
    pub field_dimensions: &'a FieldDimensions,
    pub time: Time,
}

/// Pose prediction for an established Tracking state.
#[derive(Clone, Copy)]
pub struct TrackingAssociationInput<'a> {
    pub visual_features: &'a DetectedVisualFeatures,
    pub robot_to_camera: Isometry3<Robot, Camera>,
    pub geometry: &'a AssociationGeometry,
    pub camera_intrinsic: Intrinsic,
    pub field_dimensions: &'a FieldDimensions,
    pub time: Time,
}

#[derive(Clone, Copy)]
pub struct GlobalAssociationInput<'a> {
    pub visual_features: &'a DetectedVisualFeatures,
    /// Exposure-time leveling rotation, with robot heading removed.
    pub robot_to_ground: Rotation3<Robot, Ground>,
    pub robot_to_camera: Isometry3<Robot, Camera>,
    pub camera_intrinsic: Intrinsic,
    pub field_dimensions: &'a FieldDimensions,
    pub parameters: &'a GlobalLocalizerParameters,
    pub heading: Option<HeadingConstraint>,
}

/// Fixed correspondences only; pose estimation belongs to localization.
pub struct AssociationResult {
    pub associations: Vec<FieldMarkAssociation>,
    pub source: VisualAssociationSource,
    pub debug: Option<GlobalLocalizationDebug>,
}

/// Shared lifecycle dispatch for the robot and simulator. Global matching uses
/// sensor attitude at exposure; only tracking consumes optimizer geometry.
pub fn associate_visual_features(
    input: AssociationInput<'_>,
    parameters: &FieldMarkAssociationParameters,
) -> Option<AssociationResult> {
    if parameters.validate().is_err() {
        return None;
    }
    if input.status.state == LocalizationState::LostTrack && input.time <= input.status.time {
        return None;
    }
    let prepared = PreparedAssociation::new(
        input.visual_features,
        input.field_dimensions,
        parameters.global_localizer,
    )?;
    if input.status.state != LocalizationState::Startup {
        let geometry = input.tracking.filter(|geometry| {
            geometry.epoch == input.status.epoch && geometry.generation == input.status.generation
        });
        // Retain the motion posterior through a visual dropout. Its covariance
        // supplies the expanding search region; global recovery is the fallback.
        let local = geometry.and_then(|geometry| {
            let tracking_input = TrackingAssociationInput {
                geometry,
                visual_features: input.visual_features,
                robot_to_camera: input.robot_to_camera,
                camera_intrinsic: input.camera_intrinsic,
                field_dimensions: input.field_dimensions,
                time: input.time,
            };
            let age = tracking::validated_age(tracking_input, parameters)?;
            tracking::associate(tracking_input, parameters, &prepared, age)
        });
        if let Some(local) = local {
            return Some(local);
        }
    }
    let attitude = input.attitude?;
    let (roll, pitch, yaw) = attitude.euler_angles();
    let heading = if input.status.state != LocalizationState::Startup {
        match input.status.heading {
            Some(reference) => Some(reference.at(input.time, Orientation2::new(f64::from(yaw)))?),
            None => None,
        }
    } else {
        None
    };
    solver::associate(
        GlobalAssociationInput {
            visual_features: input.visual_features,
            robot_to_ground: Rotation3::from_euler_angles(roll, pitch, 0.0),
            robot_to_camera: input.robot_to_camera,
            camera_intrinsic: input.camera_intrinsic,
            field_dimensions: input.field_dimensions,
            parameters: &parameters.global_localizer,
            heading,
        },
        &prepared,
    )
}

/// Guided two-point hypotheses and pixel-space refinement. Crossing substitutions
/// require a trusted heading; uncorroborated startup uses exact classes.
/// Reject near-tied sampled poses, ambiguous fixed-pose assignments and exhausted work.
/// Without heading, return a representative modulo field half-turn symmetry;
/// bounded sampling is not an exhaustive uniqueness proof.
pub fn associate_global_visual_features(
    input: GlobalAssociationInput<'_>,
) -> Option<AssociationResult> {
    input.parameters.validate().ok()?;
    let prepared = PreparedAssociation::new(
        input.visual_features,
        input.field_dimensions,
        *input.parameters,
    )?;
    solver::associate(input, &prepared)
}

/// Predict landmarks with a pose prior, independently of lifecycle dispatch.
pub fn associate_tracking_visual_features(
    input: TrackingAssociationInput<'_>,
    parameters: &FieldMarkAssociationParameters,
) -> Option<AssociationResult> {
    parameters.validate().ok()?;
    let prepared = PreparedAssociation::new(
        input.visual_features,
        input.field_dimensions,
        parameters.global_localizer,
    )?;
    let age = tracking::validated_age(input, parameters)?;
    tracking::associate(input, parameters, &prepared, age)
}
