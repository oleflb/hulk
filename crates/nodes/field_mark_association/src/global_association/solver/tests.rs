use ::types::field_dimensions::FieldDimensions;

use super::*;

#[test]
fn stable_matches_drop_non_symmetric_repeated_class_alternative() -> Result<(), String> {
    let map = LandmarkMap::new(&FieldDimensions::SPL_2025, 0.25);
    let l_goal_box = landmark_id(&map, VisualFeatureClass::LSpot, -3.9, -1.1)?;
    let l_penalty_box = landmark_id(&map, VisualFeatureClass::LSpot, -2.85, -2.0)?;
    let l_corner = landmark_id(&map, VisualFeatureClass::LSpot, 4.5, 3.0)?;
    let t_goal_box = landmark_id(&map, VisualFeatureClass::TSpot, -4.5, -1.1)?;

    let best = candidate(vec![
        (0, l_goal_box, VisualFeatureClass::LSpot),
        (1, l_penalty_box, VisualFeatureClass::LSpot),
        (2, l_corner, VisualFeatureClass::LSpot),
        (3, t_goal_box, VisualFeatureClass::TSpot),
    ]);
    let alternative = candidate(vec![
        (0, map.symmetric_id(l_goal_box), VisualFeatureClass::LSpot),
        (
            1,
            map.symmetric_id(l_penalty_box),
            VisualFeatureClass::LSpot,
        ),
        (2, l_penalty_box, VisualFeatureClass::LSpot),
        (3, map.symmetric_id(t_goal_box), VisualFeatureClass::TSpot),
    ]);

    let near_optimal = vec![&best, &alternative];
    let stable = stable_matches_from_candidates(&best, &near_optimal, &map);
    let stable_detections = stable
        .iter()
        .map(|accepted| accepted.detection_index)
        .collect::<Vec<_>>();

    assert_eq!(stable_detections, vec![0, 1, 3]);
    Ok(())
}

#[test]
fn stable_matches_reject_mixed_symmetry_branch() -> Result<(), String> {
    let map = LandmarkMap::new(&FieldDimensions::SPL_2025, 0.25);
    let l_goal_box = landmark_id(&map, VisualFeatureClass::LSpot, -3.9, -1.1)?;
    let l_penalty_box = landmark_id(&map, VisualFeatureClass::LSpot, -2.85, -2.0)?;
    let l_corner = landmark_id(&map, VisualFeatureClass::LSpot, 4.5, 3.0)?;
    let t_goal_box = landmark_id(&map, VisualFeatureClass::TSpot, -4.5, -1.1)?;

    let best = candidate(vec![
        (0, l_goal_box, VisualFeatureClass::LSpot),
        (1, l_penalty_box, VisualFeatureClass::LSpot),
        (2, l_corner, VisualFeatureClass::LSpot),
        (3, t_goal_box, VisualFeatureClass::TSpot),
    ]);
    let mixed = candidate(vec![
        (0, l_goal_box, VisualFeatureClass::LSpot),
        (
            1,
            map.symmetric_id(l_penalty_box),
            VisualFeatureClass::LSpot,
        ),
        (2, l_penalty_box, VisualFeatureClass::LSpot),
        (3, t_goal_box, VisualFeatureClass::TSpot),
    ]);

    let near_optimal = vec![&best, &mixed];
    let stable = stable_matches_from_candidates(&best, &near_optimal, &map);
    let stable_detections = stable
        .iter()
        .map(|accepted| accepted.detection_index)
        .collect::<Vec<_>>();

    assert_eq!(stable_detections, vec![0, 3]);
    Ok(())
}

fn candidate(matches: Vec<(usize, usize, VisualFeatureClass)>) -> Candidate {
    let matches = matches
        .into_iter()
        .map(|(detection_index, landmark_id, class)| Match {
            detection_index,
            landmark_id,
            class,
            confidence: 0.9,
            residual: 0.0,
        })
        .collect::<Vec<_>>();
    Candidate {
        score: matches.len() as f32,
        matches,
        metric_rms_residual: 0.0,
        transform: Similarity2::new(Vector2::zeros(), 0.0, 0.5),
    }
}

fn landmark_id(
    map: &LandmarkMap,
    class: VisualFeatureClass,
    x: f32,
    y: f32,
) -> Result<usize, String> {
    map.landmarks
        .iter()
        .find(|landmark| {
            landmark.class == class
                && (landmark.xy.x() - x).abs() < 1.0e-4
                && (landmark.xy.y() - y).abs() < 1.0e-4
        })
        .map(|landmark| landmark.id)
        .ok_or_else(|| format!("missing landmark {class:?} at ({x}, {y})"))
}
