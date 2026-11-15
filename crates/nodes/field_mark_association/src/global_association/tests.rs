use coordinate_systems::{Camera, Field, Robot};
use linear_algebra::{Isometry3, Point2, point};
use nalgebra::{SMatrix, UnitQuaternion, vector};
use projection::intrinsic::Intrinsic;
use ros_z::time::Time;
use types::{
    field_dimensions::{FieldDimensions, Half, Side},
    localization::{LocalizationState, LocalizationStatus, PoseEstimate},
    visual_localization::AssociationGeometry,
};

use super::{GlobalAssociationConfig, solver};
use crate::{
    AssociationInput, AssociationResult, DetectedVisualFeature, DetectedVisualFeatures,
    FieldMarkAssociationParameters, GlobalAssociationInput, VisualFeatureClass,
    associate_global_visual_features, associate_visual_features, map::LandmarkMap,
};

fn geometry() -> AssociationGeometry {
    AssociationGeometry {
        epoch: 0,
        generation: 0,
        estimate: tracking_estimate(0.0),
        last_successful_solve: Time::from_nanos(1_000_000_000),
    }
}

fn intrinsic() -> Intrinsic {
    Intrinsic::new(vector![500.0, 500.0], point![320.0, 240.0])
}

fn robot_to_camera() -> Isometry3<Robot, Camera> {
    Isometry3::wrap(
        nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(0.0, 0.0, 0.1),
            UnitQuaternion::from_euler_angles(std::f32::consts::PI, 0.0, 0.0),
        )
        .inverse(),
    )
}

fn input<'a>(
    features: &'a DetectedVisualFeatures,
    geometry: &'a AssociationGeometry,
) -> AssociationInput<'a> {
    static STATUS: std::sync::LazyLock<LocalizationStatus> =
        std::sync::LazyLock::new(|| LocalizationStatus {
            time: Time::zero(),
            epoch: 0,
            generation: 0,
            state: LocalizationState::Tracking,
            heading: None,
        });
    AssociationInput {
        status: &STATUS,
        attitude: None,
        visual_features: features,
        robot_to_camera: robot_to_camera(),
        tracking: Some(geometry),
        camera_intrinsic: intrinsic(),
        field_dimensions: &FieldDimensions::SPL_2025,
        time: Time::from_nanos(1_000_000_000),
    }
}

fn global_input<'a>(
    input: AssociationInput<'a>,
    parameters: &'a GlobalAssociationConfig,
) -> GlobalAssociationInput<'a> {
    let (roll, pitch, _) = input
        .tracking
        .map_or((f64::NAN, f64::NAN, f64::NAN), |geometry| {
            geometry.estimate.pose.inner.rotation.euler_angles()
        });
    GlobalAssociationInput {
        visual_features: input.visual_features,
        robot_to_ground: linear_algebra::Rotation3::from_euler_angles(
            roll as f32,
            pitch as f32,
            0.0,
        ),
        robot_to_camera: input.robot_to_camera,
        camera_intrinsic: input.camera_intrinsic,
        field_dimensions: input.field_dimensions,
        parameters,
        heading: None,
    }
}

fn calibrated_parameters() -> FieldMarkAssociationParameters {
    let mut parameters = FieldMarkAssociationParameters::default();
    parameters.global_localizer.imu_tilt_sigma = 0.001;
    parameters.global_localizer.height_sigma = 0.001;
    parameters
}

fn project(
    landmarks: impl IntoIterator<Item = (VisualFeatureClass, Point2<Field>)>,
    geometry: &AssociationGeometry,
) -> DetectedVisualFeatures {
    let field_to_camera = robot_to_camera()
        * Isometry3::<Robot, Field>::wrap(geometry.estimate.pose.inner.cast()).inverse();
    let mut features = DetectedVisualFeatures::default();
    for (class, landmark) in landmarks {
        let pixel = intrinsic().project((field_to_camera * landmark.extend(0.0)).coords());
        let feature = DetectedVisualFeature {
            pixel,
            confidence: 0.95,
        };
        match class {
            VisualFeatureClass::GoalPost => &mut features.goalposts,
            VisualFeatureClass::LSpot => &mut features.l_spots,
            VisualFeatureClass::TSpot => &mut features.t_spots,
            VisualFeatureClass::XSpot => &mut features.x_spots,
            VisualFeatureClass::PenaltySpot => &mut features.penalty_spots,
        }
        .push(feature);
    }
    features
}

fn stationary_three() -> [(VisualFeatureClass, Point2<Field>); 3] {
    let field = FieldDimensions::SPL_2025;
    [
        (
            VisualFeatureClass::GoalPost,
            field.goal_post(Half::Opponent, Side::Left),
        ),
        (
            VisualFeatureClass::GoalPost,
            field.goal_post(Half::Opponent, Side::Right),
        ),
        (
            VisualFeatureClass::PenaltySpot,
            field.penalty_spot(Half::Opponent),
        ),
    ]
}

#[test]
fn lost_tracking_reuses_the_motion_posterior_without_global_reinitialization() {
    use types::localization::{LocalizationState, LocalizationStatus};
    let geometry = geometry();
    let features = project(stationary_three(), &geometry);
    let parameters = calibrated_parameters();
    let mut status = LocalizationStatus {
        time: Time::from_nanos(900_000_000),
        epoch: 0,
        generation: 0,
        state: LocalizationState::LostTrack,
        heading: None,
    };
    let associate = |status: &LocalizationStatus| {
        crate::associate_visual_features(
            crate::AssociationInput {
                status,
                attitude: None,
                tracking: Some(&geometry),
                visual_features: &features,
                robot_to_camera: robot_to_camera(),
                camera_intrinsic: intrinsic(),
                field_dimensions: &FieldDimensions::SPL_2025,
                time: Time::from_nanos(1_000_000_000),
            },
            &parameters,
        )
    };
    let result = associate(&status).unwrap();
    assert_eq!(
        result.source,
        types::visual_localization::VisualAssociationSource::Tracking
    );
    assert_eq!(result.associations.len(), 3);
    for count in 1..=2 {
        let sparse = project(stationary_three().into_iter().take(count), &geometry);
        let result = associate_visual_features(input(&sparse, &geometry), &parameters).unwrap();
        assert_eq!(result.associations.len(), count);
    }
    status.generation += 1;
    assert!(associate(&status).is_none());
    status.generation = 0;
    status.time = Time::from_nanos(1_000_000_000);
    assert!(associate(&status).is_none());
    status.state = LocalizationState::Startup;
    assert!(associate(&status).is_none());
}

fn key(result: &AssociationResult) -> Vec<([f32; 2], [f32; 2])> {
    result
        .associations
        .iter()
        .map(|a| {
            (
                [a.detection.x(), a.detection.y()],
                [a.field_point.x(), a.field_point.y()],
            )
        })
        .collect()
}

fn tracking_estimate(covariance: f64) -> PoseEstimate<Robot, Field> {
    PoseEstimate {
        pose: Isometry3::wrap(nalgebra::Isometry3::translation(2.0, 0.0, 0.45)),
        covariance: SMatrix::identity() * covariance,
    }
}

#[derive(serde::Deserialize)]
pub(super) struct RecordedStartup {
    time_ns: u64,
    imu_before: (u64, [f32; 3]),
    imu_after: (u64, [f32; 3]),
    robot_to_camera: Isometry3<Robot, Camera>,
    camera_intrinsic: Intrinsic,
    field_dimensions: FieldDimensions,
    goalposts: Vec<[f32; 3]>,
    l_spots: Vec<[f32; 3]>,
    t_spots: Vec<[f32; 3]>,
    #[serde(default)]
    penalty_spots: Vec<[f32; 3]>,
}

impl RecordedStartup {
    pub(super) fn epoch4() -> Self {
        serde_json::from_str(include_str!("robot45_epoch4_startup.json")).unwrap()
    }
    pub(super) fn load() -> Self {
        serde_json::from_str(include_str!("robot45_startup.json")).unwrap()
    }

    pub(super) fn features(&self) -> DetectedVisualFeatures {
        let features = |values: &[[f32; 3]]| {
            values
                .iter()
                .map(|&[x, y, confidence]| DetectedVisualFeature {
                    pixel: point![x, y],
                    confidence,
                })
                .collect()
        };
        DetectedVisualFeatures {
            goalposts: features(&self.goalposts),
            l_spots: features(&self.l_spots),
            t_spots: features(&self.t_spots),
            penalty_spots: features(&self.penalty_spots),
            ..Default::default()
        }
    }

    pub(super) fn input<'a>(
        &'a self,
        features: &'a DetectedVisualFeatures,
        parameters: &'a GlobalAssociationConfig,
    ) -> GlobalAssociationInput<'a> {
        let rotation = |(_, [r, p, y]): (u64, [f32; 3])| UnitQuaternion::from_euler_angles(r, p, y);
        let fraction = (self.time_ns - self.imu_before.0) as f32
            / (self.imu_after.0 - self.imu_before.0) as f32;
        let (roll, pitch, _) = rotation(self.imu_before)
            .slerp(&rotation(self.imu_after), fraction)
            .euler_angles();
        GlobalAssociationInput {
            visual_features: features,
            robot_to_ground: linear_algebra::Rotation3::from_euler_angles(roll, pitch, 0.0),
            robot_to_camera: self.robot_to_camera,
            camera_intrinsic: self.camera_intrinsic,
            field_dimensions: &self.field_dimensions,
            parameters,
            heading: None,
        }
    }
}

#[test]
fn recorded_epoch4_later_crossing_consensus_remains_available() {
    let frame: RecordedStartup =
        serde_json::from_str(include_str!("robot45_epoch4_good.json")).unwrap();
    let features = frame.features();
    let result = associate_global_visual_features(
        frame.input(&features, &GlobalAssociationConfig::default()),
    )
    .unwrap();
    assert_eq!(
        key(&result),
        vec![
            ([496.75, 140.84375], [-6.045, -3.95]),
            ([305.5, 90.53125], [-9.035, -2.45]),
            ([48.140625, 130.53125], [-10.955, -6.965]),
            ([200.4375, 96.3125], [-10.955, -3.95]),
            ([245.75, 85.90625], [-10.955, -2.45]),
        ]
    );
}

#[test]
fn recorded_robot45_startup_prefers_goalpost_supported_arrangement() {
    // Original robot45 MCAP, epoch 9, elapsed 466.496024 s; IMU interpolated at exposure.
    let frame = RecordedStartup::load();
    let features = frame.features();
    let config = GlobalAssociationConfig::default();
    let input = frame.input(&features, &config);
    let (_, retained, observations) = solver::preprocess(input).unwrap();
    assert_eq!((retained.len(), observations), (5, 5));
    let result = associate_global_visual_features(input).unwrap();
    assert_eq!(
        key(&result),
        vec![
            ([443.0, 222.0], [-10.955, 1.55]),
            ([187.8125, 225.3125], [-6.045, 3.95]),
            ([350.5, 222.0], [-9.035, 2.45]),
            ([326.375, 201.4375], [-9.035, -2.45]),
        ]
    );
    assert_eq!(
        key(&associate_global_visual_features(input).unwrap()),
        key(&result)
    );
}

#[test]
fn recorded_robot45_startup_rejects_near_tied_arrangements() {
    let frame = RecordedStartup::load();
    let mut features = frame.features();
    features.goalposts[0].confidence = 0.35;
    features.t_spots[0].confidence = 0.95;
    assert!(
        associate_global_visual_features(
            frame.input(&features, &GlobalAssociationConfig::default())
        )
        .is_none()
    );
    let relaxed = GlobalAssociationConfig {
        score_ratio: 1.001,
        ..Default::default()
    };
    assert!(associate_global_visual_features(frame.input(&features, &relaxed)).is_some());
}

#[test]
fn rare_false_positive_does_not_require_a_goalpost_match() {
    let geometry = geometry();
    let config = calibrated_parameters().global_localizer;
    let map = LandmarkMap::new(&FieldDimensions::SPL_2025);
    let landmarks = map
        .landmarks
        .iter()
        .filter(|l| l.class != VisualFeatureClass::GoalPost);
    let mut features = project(landmarks.map(|l| (l.class, l.xy)), &geometry);
    let mut false_positive = project(
        [(VisualFeatureClass::GoalPost, point![15.0, 7.0])],
        &geometry,
    )
    .goalposts[0];
    false_positive.confidence = 1.0;
    features.goalposts.push(false_positive);
    let result =
        associate_global_visual_features(global_input(input(&features, &geometry), &config))
            .unwrap();
    assert_eq!(result.associations.len(), map.landmarks.len() - 4);
    assert!(
        result
            .associations
            .iter()
            .all(|a| a.detection != false_positive.pixel)
    );
}

#[test]
fn three_stationary_features_certify_without_history_or_pose() {
    let geometry = geometry();
    let features = project(stationary_three(), &geometry);
    let mut parameters = FieldMarkAssociationParameters::default();
    assert_eq!(parameters.global_localizer.min_inliers, 3);
    let first = associate_global_visual_features(global_input(
        input(&features, &geometry),
        &parameters.global_localizer,
    ))
    .unwrap();
    assert_eq!(first.associations.len(), 3);
    assert!(first.associations.iter().all(|a| a.field_point.x() < 0.0));
    assert_eq!(first.debug.as_ref().unwrap().association_count, 3);
    assert!(first.debug.as_ref().unwrap().pairwise_distance_rms < 1.0e-4);
    // Global fitting estimates height; tracking's height-noise floor is irrelevant.
    parameters.global_localizer.height_sigma = 0.2;
    assert_eq!(
        key(&first),
        key(&associate_global_visual_features(global_input(
            input(&features, &geometry),
            &parameters.global_localizer
        ))
        .unwrap())
    );
}

#[test]
fn lifecycle_dispatch_uses_exposure_attitude_and_matches_generations() {
    use types::localization::{FieldHeadingReference, LocalizationState, LocalizationStatus};
    let geometry = geometry();
    let features = project(stationary_three(), &geometry);
    let parameters = FieldMarkAssociationParameters::default();
    let mut wrong = geometry.clone();
    wrong.estimate = tracking_estimate(0.01);
    wrong.estimate.pose.inner.rotation = UnitQuaternion::from_euler_angles(0.8, 0.7, 1.2);
    let mut status = LocalizationStatus {
        time: Time::from_nanos(0),
        epoch: 0,
        generation: 0,
        state: LocalizationState::Startup,
        heading: None,
    };
    let call = |status: &LocalizationStatus, tracking| {
        crate::associate_visual_features(
            crate::AssociationInput {
                status,
                attitude: Some(linear_algebra::Orientation3::from_euler_angles(
                    0.0, 0.0, 1.3,
                )),
                tracking,
                visual_features: &features,
                robot_to_camera: robot_to_camera(),
                camera_intrinsic: intrinsic(),
                field_dimensions: &FieldDimensions::SPL_2025,
                time: Time::from_nanos(1_000_000_000),
            },
            &parameters,
        )
    };
    // Startup must ignore an optimizer tilt that disagrees with the source IMU.
    assert_eq!(call(&status, Some(&wrong)).unwrap().associations.len(), 3);
    status.state = LocalizationState::LostTrack;
    assert!(call(&status, None).is_some());
    status.heading = Some(FieldHeadingReference {
        time: status.time,
        imu_to_field: linear_algebra::Rotation2::new(-1.3),
        max_error: 0.3,
    });
    let recovery = call(&status, None).unwrap();
    assert_eq!(
        recovery.source,
        types::visual_localization::VisualAssociationSource::Global
    );
    assert_eq!(recovery.associations.len(), 3);
    assert!(
        recovery
            .associations
            .iter()
            .all(|a| a.field_point.x() > 0.0)
    );
    status.state = LocalizationState::Tracking;
    status.generation = 1;
    let fallback = call(&status, Some(&wrong)).unwrap();
    assert_eq!(
        fallback.source,
        types::visual_localization::VisualAssociationSource::Global
    );
    assert_eq!(fallback.associations.len(), 3);
    // Matching generations enter tracking, whose wrong tilt must still fall back identically.
    status.generation = 0;
    let fallback = call(&status, Some(&wrong)).unwrap();
    assert_eq!(fallback.source, recovery.source);
    assert_eq!(key(&fallback), key(&recovery));
    assert_eq!(
        fallback.debug.as_ref().unwrap().pairwise_distance_rms,
        recovery.debug.as_ref().unwrap().pairwise_distance_rms
    );
    status.heading = None;
    assert!(call(&status, Some(&wrong)).is_some());
}

#[test]
fn heading_preserves_orientation_and_budget_exhaustion_still_rejects() {
    let geometry = geometry();
    let features = project(stationary_three(), &geometry);
    let config = GlobalAssociationConfig::default();
    let mut input = global_input(input(&features, &geometry), &config);
    let heading = crate::HeadingConstraint {
        expected: linear_algebra::Orientation2::identity(),
        max_error: 0.3,
    };
    input.heading = Some(heading);
    let result = associate_global_visual_features(input).unwrap();
    assert_eq!(result.associations.len(), 3);
    assert!(result.associations.iter().all(|a| a.field_point.x() > 0.0));
    let exhausted = GlobalAssociationConfig {
        max_work: 1,
        ..config
    };
    assert!(
        associate_global_visual_features(GlobalAssociationInput {
            parameters: &exhausted,
            ..input
        })
        .is_none()
    );
    for limit in [f64::NAN, 0.0, std::f64::consts::FRAC_PI_2] {
        input.heading = Some(crate::HeadingConstraint {
            max_error: limit,
            ..heading
        });
        assert!(associate_global_visual_features(input).is_none());
    }
}

#[test]
fn canonical_landmarks_do_not_select_the_robot_half() {
    for robot_x in [-2.0, 2.0] {
        let mut geometry = geometry();
        geometry.estimate.pose.inner.translation.vector.x = robot_x;
        let features = project(stationary_three(), &geometry);
        let config = calibrated_parameters().global_localizer;
        let result =
            associate_global_visual_features(global_input(input(&features, &geometry), &config))
                .unwrap();
        assert_eq!(result.associations.len(), 3, "robot x {robot_x}");
        assert!(result.associations.iter().all(|a| a.field_point.x() < 0.0));
        // These correspondences imply a half-turn, so the canonical robot is at -robot_x.
        // In particular, a robot really at -2 yields a representative at +2, not own-half.
        for (association, (_, expected)) in result.associations.iter().zip(stationary_three()) {
            assert!(
                (association.field_point.xy().coords().inner + expected.coords().inner).norm()
                    < 1.0e-5
            );
        }
    }
}

#[test]
fn halfturn_equivalence_survives_center_landmarks_and_detection_reordering() {
    let geometry = geometry();
    let field = FieldDimensions::SPL_2025;
    let config = calibrated_parameters().global_localizer;
    // Swapping the two penalties and the circle crossing is exactly a field half-turn.
    for center in [None, Some((VisualFeatureClass::XSpot, field.center()))] {
        let mut features = project(
            center.into_iter().chain([
                (
                    VisualFeatureClass::PenaltySpot,
                    field.penalty_spot(Half::Own),
                ),
                (
                    VisualFeatureClass::PenaltySpot,
                    field.penalty_spot(Half::Opponent),
                ),
                (VisualFeatureClass::XSpot, field.x_crossing(Side::Left)),
            ]),
            &geometry,
        );
        let first =
            associate_global_visual_features(global_input(input(&features, &geometry), &config))
                .unwrap();
        assert_eq!(first.associations.len(), 3 + usize::from(center.is_some()));
        if center.is_none() {
            continue;
        }
        features.x_spots.reverse();
        let second =
            associate_global_visual_features(global_input(input(&features, &geometry), &config))
                .unwrap();
        let expected = key(&first);
        let actual = key(&second);
        assert_eq!(actual.len(), expected.len());
        assert!(actual.iter().all(|pair| expected.contains(pair)));
    }
}

#[test]
fn translated_triangle_orbits_are_rejected() {
    let geometry = geometry();
    let field = FieldDimensions {
        penalty_area_length: FieldDimensions::SPL_2025.goal_box_area_length,
        ..FieldDimensions::SPL_2025
    };
    // Translation by -(goal_box_width + penalty_box_width)/2 yields another L,T,T
    // triangle on the same goal line, with identical edge lengths AND chirality.
    let features = project(
        [
            (
                VisualFeatureClass::LSpot,
                field.goal_box_corner(Half::Opponent, Side::Left),
            ),
            (
                VisualFeatureClass::TSpot,
                field.goal_box_goal_line_intersection(Half::Opponent, Side::Left),
            ),
            (
                VisualFeatureClass::TSpot,
                field.penalty_box_goal_line_intersection(Half::Opponent, Side::Left),
            ),
        ],
        &geometry,
    );
    let config = calibrated_parameters().global_localizer;
    let mut input = input(&features, &geometry);
    input.field_dimensions = &field;
    assert!(associate_global_visual_features(global_input(input, &config)).is_none());
}

#[test]
fn collinear_and_duplicate_seeds_do_not_certify() {
    let geometry = geometry();
    let field = FieldDimensions::SPL_2025;
    let features = project(
        [
            (VisualFeatureClass::XSpot, field.center()),
            (VisualFeatureClass::XSpot, field.x_crossing(Side::Left)),
            (VisualFeatureClass::XSpot, field.x_crossing(Side::Right)),
        ],
        &geometry,
    );
    assert!(
        associate_global_visual_features(global_input(
            input(&features, &geometry),
            &GlobalAssociationConfig::default()
        ))
        .is_none()
    );
    let mut features = project(stationary_three(), &geometry);
    features.goalposts[1] = features.goalposts[0];
    assert!(
        associate_global_visual_features(global_input(
            input(&features, &geometry),
            &GlobalAssociationConfig::default()
        ))
        .is_none()
    );
}

#[test]
fn rich_frame_has_bounded_search_and_budget_exhaustion_rejects() {
    let geometry = geometry();
    let map = LandmarkMap::new(&FieldDimensions::SPL_2025);
    let features = project(map.landmarks.iter().map(|l| (l.class, l.xy)), &geometry);
    let mut config = calibrated_parameters().global_localizer;
    let result =
        associate_global_visual_features(global_input(input(&features, &geometry), &config))
            .unwrap();
    assert_eq!(result.associations.len(), map.landmarks.len());
    config.max_work = 1;
    assert!(
        associate_global_visual_features(global_input(input(&features, &geometry), &config))
            .is_none()
    );
}

#[test]
fn tracking_keeps_opponent_half_prediction() {
    let mut geometry = geometry();
    geometry.estimate = tracking_estimate(0.0001);
    let features = project(stationary_three(), &geometry);
    let result = associate_visual_features(
        input(&features, &geometry),
        &FieldMarkAssociationParameters::default(),
    )
    .unwrap();
    assert_eq!(result.associations.len(), 3);
    assert!(result.associations.iter().all(|a| a.field_point.x() > 0.0));
    geometry.estimate.pose.inner.translation.x = f64::NAN;
    assert!(
        associate_visual_features(
            input(&features, &geometry),
            &FieldMarkAssociationParameters::default()
        )
        .is_none()
    );
}

#[test]
fn tracking_covariance_and_age_widen_the_gate_but_not_the_distance_limit() {
    for robot_x in [1.0, 2.0] {
        let mut geometry = geometry();
        geometry.estimate.pose.inner.translation.x = robot_x;
        let features = project(stationary_three(), &geometry);
        let mut parameters = calibrated_parameters();
        // Generate observations from truth before shifting the single prior by 20 pixels.
        geometry.estimate.pose.inner.translation.x += 20.0 * 0.55 / 500.0;
        assert!(associate_visual_features(input(&features, &geometry), &parameters).is_none());
        // Keep stored covariance zero; age-dependent noise alone must explain the offset.
        let mut older = input(&features, &geometry);
        older.time = Time::from_nanos(2_000_000_000);
        assert_eq!(
            associate_visual_features(older, &parameters)
                .unwrap()
                .associations
                .len(),
            3,
            "robot x {robot_x}"
        );
        geometry.estimate.covariance = SMatrix::identity() * 0.001;
        assert_eq!(
            associate_visual_features(input(&features, &geometry), &parameters)
                .unwrap()
                .associations
                .len(),
            3,
            "robot x {robot_x}"
        );
        parameters.tracking.max_pixel_distance = 19.0;
        assert!(associate_visual_features(input(&features, &geometry), &parameters).is_none());
    }
}

#[test]
fn guided_matching_handles_crossing_class_errors_and_outliers() {
    let geometry = geometry();
    let config = calibrated_parameters().global_localizer;
    let map = LandmarkMap::new(&FieldDimensions::SPL_2025);
    let mut features = project(map.landmarks.iter().map(|l| (l.class, l.xy)), &geometry);
    let mislabeled = features.l_spots.remove(0);
    features.t_spots.push(mislabeled);
    let outlier = project([(VisualFeatureClass::LSpot, point![15.0, 7.0])], &geometry).l_spots[0];
    features.l_spots.push(outlier);
    let mut input = global_input(input(&features, &geometry), &config);
    let Some(startup) = associate_global_visual_features(input) else {
        panic!("exact-class startup should retain the correctly classified landmarks");
    };
    assert_eq!(startup.associations.len(), map.landmarks.len() - 1);
    assert!(
        !startup
            .associations
            .iter()
            .any(|a| a.detection == mislabeled.pixel)
    );
    input.heading = Some(crate::HeadingConstraint {
        expected: linear_algebra::Orientation2::new(std::f64::consts::PI),
        max_error: 0.3,
    });
    let result = associate_global_visual_features(input).unwrap();
    assert_eq!(result.associations.len(), map.landmarks.len());
    assert!(
        !result
            .associations
            .iter()
            .any(|a| a.detection == outlier.pixel)
    );
    let actual = result
        .associations
        .iter()
        .find(|a| a.detection == mislabeled.pixel)
        .unwrap();
    assert_eq!(actual.field_point.xy(), -map.landmarks[4].xy);
    assert_eq!(
        key(&result),
        key(&associate_global_visual_features(input).unwrap())
    );
}

#[test]
fn recorded_near_and_distant_marks_are_matched_in_pixel_space() {
    let frame: serde_json::Value =
        serde_json::from_str(include_str!("recorded_frame.json")).unwrap();
    let mut features = DetectedVisualFeatures::default();
    for detection in frame["detections"].as_array().unwrap() {
        let feature = DetectedVisualFeature {
            pixel: serde_json::from_value(detection["pixel"].clone()).unwrap(),
            confidence: detection["confidence"].as_f64().unwrap() as f32,
        };
        match detection["class"].as_str().unwrap() {
            "LSpot" => features.l_spots.push(feature),
            "TSpot" => features.t_spots.push(feature),
            "XSpot" => features.x_spots.push(feature),
            _ => unreachable!(),
        }
    }
    let field = serde_json::from_value(frame["field"].clone()).unwrap();
    let config = GlobalAssociationConfig::default();
    let input = GlobalAssociationInput {
        visual_features: &features,
        robot_to_ground: linear_algebra::Rotation3::from_euler_angles(
            frame["roll"].as_f64().unwrap() as f32,
            frame["pitch"].as_f64().unwrap() as f32,
            0.0,
        ),
        robot_to_camera: serde_json::from_value(frame["robot_to_camera"].clone()).unwrap(),
        camera_intrinsic: serde_json::from_value(frame["intrinsics"].clone()).unwrap(),
        field_dimensions: &field,
        parameters: &config,
        heading: Some(crate::HeadingConstraint {
            expected: linear_algebra::Orientation2::new(frame["heading"].as_f64().unwrap()),
            max_error: 20.0_f64.to_radians(),
        }),
    };
    let result = associate_global_visual_features(input).unwrap();
    assert_eq!(result.associations.len(), 7);
    for detection in frame["detections"].as_array().unwrap() {
        let pixel: Point2<_> = serde_json::from_value(detection["pixel"].clone()).unwrap();
        let field: Point2<_> = serde_json::from_value(detection["field"].clone()).unwrap();
        let actual = result
            .associations
            .iter()
            .find(|a| a.detection == pixel)
            .unwrap();
        assert!((actual.field_point.xy() - field).inner.norm() < 1.0e-5);
    }
}

#[test]
fn uncertain_ray_pruning_does_not_turn_four_random_matches_into_consensus() {
    let frame: serde_json::Value =
        serde_json::from_str(include_str!("recorded_frame.json")).unwrap();
    let field = serde_json::from_value(frame["field"].clone()).unwrap();
    let mut features = DetectedVisualFeatures::default();
    for (class, pixel) in [
        (VisualFeatureClass::TSpot, point![63.39251, 253.52103]),
        (VisualFeatureClass::TSpot, point![68.95977, 306.8301]),
        (VisualFeatureClass::LSpot, point![254.74667, 217.54425]),
        (VisualFeatureClass::XSpot, point![142.33624, 229.30377]),
        (VisualFeatureClass::LSpot, point![358.85547, 166.777]),
        (VisualFeatureClass::LSpot, point![253.22707, 358.46127]),
    ] {
        let feature = DetectedVisualFeature {
            pixel,
            confidence: 0.7,
        };
        match class {
            VisualFeatureClass::LSpot => features.l_spots.push(feature),
            VisualFeatureClass::TSpot => features.t_spots.push(feature),
            VisualFeatureClass::XSpot => features.x_spots.push(feature),
            _ => unreachable!(),
        }
    }
    let config = GlobalAssociationConfig::default();
    let input = GlobalAssociationInput {
        visual_features: &features,
        robot_to_ground: linear_algebra::Rotation3::from_euler_angles(
            0.018662889,
            -0.007815877,
            0.0,
        ),
        robot_to_camera: Isometry3::wrap(nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(-0.0015389584, 0.33240578, 0.024560608),
            UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                0.4541692, 0.6579697, -0.494341, 0.34122306,
            )),
        )),
        camera_intrinsic: serde_json::from_value(frame["intrinsics"].clone()).unwrap(),
        field_dimensions: &field,
        parameters: &config,
        heading: Some(crate::HeadingConstraint {
            expected: linear_algebra::Orientation2::new(-2.442100588),
            max_error: 20.0_f64.to_radians(),
        }),
    };
    let (_, rays, observation_count) = solver::preprocess(input).unwrap();
    assert_eq!(rays.len(), 5);
    assert_eq!(observation_count, 6);
    // A fitted four-feature subset is insufficient: at least ceil(0.7 * 6) = 5 must agree.
    assert!(associate_global_visual_features(input).is_none());
}

#[test]
fn invalid_geometry_and_input_caps_fail_closed() {
    let mut geometry = geometry();
    let mut features = project(stationary_three(), &geometry);
    let config = GlobalAssociationConfig::default();
    let mut invalid = input(&features, &geometry);
    invalid.camera_intrinsic.focals.x = f32::NAN;
    assert!(associate_global_visual_features(global_input(invalid, &config)).is_none());
    let mut over_cap = project(
        (0..33).map(|i| {
            (
                VisualFeatureClass::LSpot,
                point![2.0 + i as f32 * 0.01, 0.0],
            )
        }),
        &geometry,
    );
    assert!(solver::preprocess(global_input(input(&over_cap, &geometry), &config)).is_none());
    over_cap.l_spots[32] = over_cap.l_spots[0];
    assert_eq!(
        solver::preprocess(global_input(input(&over_cap, &geometry), &config))
            .unwrap()
            .1
            .len(),
        32
    );
    geometry.estimate.pose.inner.rotation = UnitQuaternion::from_euler_angles(f64::NAN, 0.0, 0.0);
    assert!(
        associate_global_visual_features(global_input(input(&features, &geometry), &config))
            .is_none()
    );
    geometry.estimate.pose.inner.rotation = UnitQuaternion::identity();
    features.goalposts = vec![features.goalposts[0]; 129];
    assert!(
        associate_global_visual_features(global_input(input(&features, &geometry), &config))
            .is_none()
    );
}

#[test]
fn global_detection_order_preserves_correspondences_up_to_a_common_halfturn() {
    let config = calibrated_parameters().global_localizer;
    let geometry = geometry();
    let mut features = project(stationary_three(), &geometry);
    for _ in 0..2 {
        let result =
            associate_global_visual_features(global_input(input(&features, &geometry), &config))
                .unwrap();
        assert_eq!(result.associations.len(), 3);
        // Detection reordering may choose the other half-turn representative, but never a reflection.
        let (_, detections, _) =
            solver::preprocess(global_input(input(&features, &geometry), &config)).unwrap();
        let expected = result
            .associations
            .iter()
            .map(|association| {
                let detection = detections
                    .iter()
                    .find(|d| d.pixel == association.detection)
                    .unwrap();
                let robot_to_field =
                    Isometry3::<Robot, Field>::wrap(geometry.estimate.pose.inner.cast());
                let camera_to_field = robot_to_field * robot_to_camera().inverse();
                let origin = camera_to_field.translation().coords().inner;
                let ground_to_field =
                    nalgebra::UnitComplex::new(robot_to_field.inner.rotation.euler_angles().2);
                let field = origin.xy() + origin.z * (ground_to_field * detection.xy);
                (field, association.field_point.xy().coords().inner)
            })
            .collect::<Vec<_>>();
        assert!(
            [1.0, -1.0]
                .into_iter()
                .any(|sign| expected.iter().all(|(a, b)| (sign * a - b).norm() < 1.0e-4))
        );
        features.goalposts.reverse();
    }
}

#[test]
fn pixel_uncertainty_reaching_horizon_is_rejected() {
    let geometry = geometry();
    let features = DetectedVisualFeatures {
        penalty_spots: vec![DetectedVisualFeature {
            pixel: point![320.0, 235.0],
            confidence: 1.0,
        }],
        ..Default::default()
    };
    let mut input = input(&features, &geometry);
    input.robot_to_camera = Isometry3::wrap(nalgebra::Isometry3::rotation(vector![
        std::f32::consts::FRAC_PI_2,
        0.0,
        0.0
    ]))
    .inverse();
    let config = GlobalAssociationConfig {
        detection_pixel_sigma: 0.1,
        imu_tilt_sigma: 0.0001,
        ..Default::default()
    };
    assert_eq!(
        solver::preprocess(global_input(input, &config))
            .unwrap()
            .1
            .len(),
        1
    );
    assert!(
        solver::preprocess(global_input(
            input,
            &GlobalAssociationConfig {
                detection_pixel_sigma: 2.0,
                ..config
            }
        ))
        .unwrap()
        .1
        .is_empty()
    );
}

#[test]
fn tracking_rejects_invalid_covariance_future_solve_and_ambiguous_assignment() {
    let mut geometry = geometry();
    let features = project(stationary_three(), &geometry);
    let parameters = FieldMarkAssociationParameters::default();
    let mut invalid_covariances = vec![SMatrix::identity() * f64::NAN, -SMatrix::identity()];
    let mut nonsymmetric = SMatrix::identity();
    nonsymmetric[(0, 1)] = 0.1;
    invalid_covariances.push(nonsymmetric);
    let mut indefinite = SMatrix::identity();
    indefinite[(0, 1)] = 2.0;
    indefinite[(1, 0)] = 2.0;
    invalid_covariances.push(indefinite);
    for covariance in invalid_covariances {
        geometry.estimate.covariance = covariance;
        assert!(associate_visual_features(input(&features, &geometry), &parameters).is_none());
    }
    geometry.estimate = tracking_estimate(0.0001);
    let mut future = input(&features, &geometry);
    future.time = Time::from_nanos(999_999_999);
    assert!(associate_visual_features(future, &parameters).is_none());
    geometry.estimate = tracking_estimate(1.0);
    let field = FieldDimensions::SPL_2025;
    let features = project(
        [
            stationary_three()[0],
            stationary_three()[1],
            (
                VisualFeatureClass::XSpot,
                point![0.0, field.center_circle_diameter / 4.0],
            ),
        ],
        &geometry,
    );
    assert!(associate_visual_features(input(&features, &geometry), &parameters).is_none());
}

#[test]
fn global_never_certifies_a_seed_subset_with_a_competing_penalty() {
    let geometry = geometry();
    let config = calibrated_parameters().global_localizer;
    let mut features = project(
        stationary_three()
            .into_iter()
            .chain([(VisualFeatureClass::PenaltySpot, point![5.8, 0.0])]),
        &geometry,
    );
    for confidences in [[0.4, 0.95], [0.95, 0.4]] {
        for (feature, confidence) in features.penalty_spots.iter_mut().zip(confidences) {
            feature.confidence = confidence;
        }
        assert_eq!(
            solver::preprocess(global_input(input(&features, &geometry), &config))
                .unwrap()
                .1
                .len(),
            4
        );
        assert!(
            associate_global_visual_features(global_input(input(&features, &geometry), &config))
                .is_none()
        );
    }
    // Each competing triple alone is certifiable; retaining both must not silently omit either.
    for penalty in features.penalty_spots.clone() {
        let triple = DetectedVisualFeatures {
            goalposts: features.goalposts.clone(),
            penalty_spots: vec![penalty],
            ..Default::default()
        };
        assert_eq!(
            associate_global_visual_features(global_input(input(&triple, &geometry), &config))
                .unwrap()
                .associations
                .len(),
            3
        );
    }
}

#[test]
fn tracking_joint_assignment_keeps_plausible_rivals_outside_the_distance_limit() {
    let mut geometry = geometry();
    geometry.estimate.pose.inner.translation.vector.y = 0.375;
    let features = project(
        [
            stationary_three()[0],
            stationary_three()[1],
            stationary_three()[2],
            (VisualFeatureClass::XSpot, point![0.0, 0.365]),
        ],
        &geometry,
    );
    let mut parameters = FieldMarkAssociationParameters::default();
    parameters.tracking.max_pixel_distance = 340.0;
    // The center and +y crossing have mirror-symmetric image covariances about robot y=0.375.
    // This observation is 332 pixels from the center and 350 from its plausible rival.
    // Pruning that rival at the output ceiling would falsely certify the center match.
    geometry.estimate = tracking_estimate(1.0);
    geometry.estimate.pose.inner.translation.vector.y = 0.375;
    assert!(associate_visual_features(input(&features, &geometry), &parameters).is_none());
}

#[test]
fn tracking_high_covariance_keeps_distinct_opponent_landmarks_and_exact_age_horizon() {
    let mut geometry = geometry();
    let features = project(stationary_three(), &geometry);
    let expected = [
        features.goalposts[0],
        features.goalposts[1],
        features.penalty_spots[0],
    ]
    .into_iter()
    .zip(stationary_three())
    .map(|(feature, (_, point))| (feature.pixel, point))
    .collect::<Vec<_>>();
    let parameters = FieldMarkAssociationParameters::default();
    geometry.estimate.covariance = sparse_retained_covariance();
    for (age, expected_count) in [
        (std::time::Duration::from_millis(3100), 3),
        (parameters.tracking.max_age, 3),
        (
            parameters.tracking.max_age + std::time::Duration::from_nanos(1),
            0,
        ),
    ] {
        let mut input = input(&features, &geometry);
        input.time = input.time + age;
        let result = associate_visual_features(input, &parameters);
        if expected_count == 0 {
            assert!(result.is_none(), "age={age:?}");
            continue;
        }
        let result = result.unwrap();
        assert_eq!(result.associations.len(), expected_count, "age={age:?}");
        for association in &result.associations {
            assert!(expected.contains(&(association.detection, association.field_point.xy())));
            assert!(association.field_point.x() > 0.0);
        }
    }
}

fn sparse_retained_covariance() -> nalgebra::Matrix6<f64> {
    // Retained diagonal from the real sparse simulator immediately before loss at 7.9 s.
    SMatrix::from_diagonal(&nalgebra::Vector6::new(
        0.00066995865,
        0.000579513,
        0.002903463,
        1.0588479,
        0.9564871,
        0.9908409,
    ))
}

#[test]
fn tracking_uses_raw_image_features_even_when_their_rays_reach_the_horizon() {
    let mut geometry = geometry();
    geometry.estimate.pose = Isometry3::from_translation(linear_algebra::vector![0.0, 0.0, 0.45]);
    let robot_to_camera = Isometry3::<Robot, Camera>::wrap(nalgebra::Isometry3::rotation(vector![
        0.0,
        -std::f32::consts::FRAC_PI_2,
        0.0
    ]));
    let observe = |field: Point2<Field>| {
        let pixel = intrinsic().project(
            (robot_to_camera
                * Isometry3::<Robot, Field>::wrap(geometry.estimate.pose.inner.cast()).inverse()
                * field.extend(0.0))
            .coords(),
        );
        // These observed rays are exactly horizontal. Shared uncertain camera height explains
        // their elevation error, but intersecting them with the ground cannot produce a point.
        DetectedVisualFeature {
            pixel: point![320.0, pixel.y()],
            confidence: 0.95,
        }
    };
    let mut features = DetectedVisualFeatures {
        goalposts: vec![
            observe(stationary_three()[0].1),
            observe(stationary_three()[1].1),
        ],
        penalty_spots: vec![observe(stationary_three()[2].1)],
        ..Default::default()
    };
    features.goalposts.push(DetectedVisualFeature {
        pixel: point![f32::NAN, 0.0],
        confidence: 1.0,
    });
    features.penalty_spots.push(DetectedVisualFeature {
        pixel: point![10.0, 10.0],
        confidence: f32::NAN,
    });
    let mut estimate = PoseEstimate {
        pose: geometry.estimate.pose,
        covariance: SMatrix::zeros(),
    };
    estimate.covariance[(5, 5)] = 0.25_f64.powi(2);
    geometry.estimate = estimate;
    let mut input = input(&features, &geometry);
    input.robot_to_camera = robot_to_camera;
    assert!(
        solver::preprocess(global_input(input, &GlobalAssociationConfig::default()))
            .unwrap()
            .1
            .is_empty()
    );
    let result =
        associate_visual_features(input, &FieldMarkAssociationParameters::default()).unwrap();
    assert_eq!(result.associations.len(), 3);
    assert!(result.associations.iter().all(|a| a.field_point.x() > 0.0));
    assert!(
        result.debug.is_none(),
        "pixel residuals must not be labeled as metric RMS"
    );
}

#[test]
fn tracking_filters_duplicates_but_rejects_input_and_retained_overflow() {
    let mut geometry = geometry();
    geometry.estimate = tracking_estimate(0.0001);
    let parameters = FieldMarkAssociationParameters::default();
    let mut features = project(stationary_three(), &geometry);
    let duplicate = features.goalposts[0];
    features.goalposts.push(duplicate);
    assert_eq!(
        associate_visual_features(input(&features, &geometry), &parameters)
            .unwrap()
            .associations
            .len(),
        3
    );
    features
        .goalposts
        .extend(std::iter::repeat_n(duplicate, 125));
    assert_eq!(features.supported_feature_count(), 129);
    assert!(associate_visual_features(input(&features, &geometry), &parameters).is_none());
    let mut features = project(stationary_three(), &geometry);
    features
        .goalposts
        .extend((0..30).map(|i| DetectedVisualFeature {
            pixel: point![40000.0 + i as f32 * 10.0, 40000.0],
            confidence: 0.95,
        }));
    assert_eq!(features.supported_feature_count(), 33);
    assert!(associate_visual_features(input(&features, &geometry), &parameters).is_none());
}
