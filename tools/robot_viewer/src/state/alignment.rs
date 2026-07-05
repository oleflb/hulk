use std::{sync::Arc, time::Duration};

use ros_z::{cache::CacheInner, time::Time};
use types::time_wrapper::TimeWrapper;

pub(super) fn exact_sample<T>(cache: &CacheInner<T>, time: Time) -> Option<TimeWrapper<Arc<T>>> {
    cache
        .get_exact(time)
        .map(|inner| TimeWrapper { time, inner })
}

pub(super) fn nearest_sample<T>(
    cache: &CacheInner<T>,
    time: Time,
    max_distance: Duration,
) -> Option<TimeWrapper<Arc<T>>> {
    let (sample_time, inner) = cache.get_nearest_with_stamp(time)?;
    (sample_time.abs_diff(time) <= max_distance).then(|| TimeWrapper {
        time: sample_time,
        inner,
    })
}

#[cfg(test)]
mod tests {
    use coordinate_systems::{Camera, Field, Robot};
    use kinematics::robot_kinematics::RobotKinematics;
    use linear_algebra::Isometry3;
    use projection::camera_matrix::CameraMatrix;
    use ros_z::time::Time;
    use types::{
        object_detection::{Object, RobocupObjectLabel},
        visual_localization::VisualLocalizationFrame as FieldMarkAssociations,
    };

    use super::*;
    use crate::state::{
        CAMERA_FRAME_BUFFER_CAPACITY, CameraFrame, MAX_NEAREST_SAMPLE_DISTANCE, ViewerState,
    };

    fn empty_associations() -> FieldMarkAssociations {
        FieldMarkAssociations {
            robot_to_camera: Isometry3::<Robot, Camera>::identity(),
            associations: Vec::new(),
            backend_reset: None,
        }
    }

    fn empty_detected_objects(time: Time) -> TimeWrapper<Vec<Object<RobocupObjectLabel>>> {
        TimeWrapper {
            time,
            inner: Vec::new(),
        }
    }

    fn localization() -> Option<Isometry3<Field, Robot>> {
        Some(Isometry3::<Field, Robot>::identity())
    }

    #[test]
    fn high_rate_alignment_streams_retain_delayed_render_samples() {
        let mut state = ViewerState::default();
        let anchor_time = Time::from_nanos(0);

        for index in 0..600 {
            let time = Time::from_nanos(index * 2_000_000);
            state.push_camera_matrix(time, CameraMatrix::default());
            state.push_robot_kinematics(time, RobotKinematics::default());
        }

        assert!(
            nearest_sample(
                &state.camera_matrices,
                anchor_time,
                MAX_NEAREST_SAMPLE_DISTANCE
            )
            .is_some()
        );
        assert!(
            nearest_sample(
                &state.robot_kinematics,
                anchor_time,
                MAX_NEAREST_SAMPLE_DISTANCE
            )
            .is_some()
        );
    }

    #[test]
    fn aligned_snapshot_waits_for_all_required_exact_streams() {
        let mut state = ViewerState::default();
        let association_time = Time::from_nanos(1_000_000_000);
        let detection_time = Time::from_nanos(1_100_000_000);

        state.objects_status.update_publishers(1);
        state.field_mark_associations_status.update_publishers(1);
        state.push_camera_frame(association_time, CameraFrame::default());
        state.push_camera_frame(detection_time, CameraFrame::default());
        state.push_field_mark_associations(association_time, empty_associations());
        state.push_detected_objects(empty_detected_objects(detection_time));

        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(detection_time));
        assert!(aligned.field_mark_associations.is_none());
        assert!(aligned.detected_objects.is_some());

        state.push_field_mark_associations(detection_time, empty_associations());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(detection_time));
        assert!(aligned.field_mark_associations.is_some());
        assert!(aligned.detected_objects.is_some());
    }

    #[test]
    fn aligned_snapshot_uses_delayed_association_for_buffered_camera_frame() {
        let mut state = ViewerState::default();
        let association_time = Time::from_nanos(1_000_000_000);
        let latest_camera_time = Time::from_nanos(1_100_000_000);

        state.field_mark_associations_status.update_publishers(1);
        state.push_camera_frame(association_time, CameraFrame::default());
        state.push_camera_frame(latest_camera_time, CameraFrame::default());
        state.push_field_mark_associations(association_time, empty_associations());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(association_time));
        assert!(aligned.field_mark_associations.is_some());
    }

    #[test]
    fn aligned_snapshot_does_not_move_display_backwards_for_delayed_association() {
        let mut state = ViewerState::default();
        let old_time = Time::from_nanos(1_000_000_000);
        let new_time = Time::from_nanos(1_100_000_000);

        state.push_camera_frame(old_time, CameraFrame::default());
        state.push_camera_frame(new_time, CameraFrame::default());
        assert_eq!(state.aligned_snapshot().anchor_time, Some(new_time));

        state.push_field_mark_associations(old_time, empty_associations());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(new_time));
        assert!(aligned.field_mark_associations.is_none());
    }

    #[test]
    fn aligned_snapshot_accepts_newer_association_after_display_progresses() {
        let mut state = ViewerState::default();
        let old_time = Time::from_nanos(1_000_000_000);
        let new_time = Time::from_nanos(1_100_000_000);
        let newer_time = Time::from_nanos(1_200_000_000);

        state.push_camera_frame(old_time, CameraFrame::default());
        state.push_camera_frame(new_time, CameraFrame::default());
        assert_eq!(state.aligned_snapshot().anchor_time, Some(new_time));

        state.push_camera_frame(newer_time, CameraFrame::default());
        state.push_field_mark_associations(newer_time, empty_associations());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(newer_time));
        assert!(aligned.field_mark_associations.is_some());
    }

    #[test]
    fn aligned_snapshot_keeps_displayed_association_until_next_required_sample() {
        let mut state = ViewerState::default();
        let association_time = Time::from_nanos(1_000_000_000);
        let latest_camera_time = Time::from_nanos(1_100_000_000);

        state.field_mark_associations_status.update_publishers(1);
        state.push_camera_frame(association_time, CameraFrame::default());
        state.push_field_mark_associations(association_time, empty_associations());
        assert_eq!(state.aligned_snapshot().anchor_time, Some(association_time));

        state.push_camera_frame(latest_camera_time, CameraFrame::default());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(association_time));
        assert!(aligned.field_mark_associations.is_some());

        state.push_field_mark_associations(latest_camera_time, empty_associations());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(latest_camera_time));
        assert!(aligned.field_mark_associations.is_some());
    }

    #[test]
    fn aligned_snapshot_keeps_current_frame_for_delayed_detection_before_advancing() {
        let mut state = ViewerState::default();
        let displayed_time = Time::from_nanos(1_000_000_000);
        let latest_camera_time = Time::from_nanos(1_100_000_000);

        state.objects_status.update_publishers(1);
        state.push_camera_frame(displayed_time, CameraFrame::default());
        assert_eq!(state.aligned_snapshot().anchor_time, Some(displayed_time));

        state.push_detected_objects(empty_detected_objects(displayed_time));
        state.push_camera_frame(latest_camera_time, CameraFrame::default());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(displayed_time));
        assert!(aligned.detected_objects.is_some());

        let aligned = state.aligned_snapshot();
        assert_eq!(aligned.anchor_time, Some(displayed_time));
        assert!(aligned.detected_objects.is_some());

        state.push_detected_objects(empty_detected_objects(latest_camera_time));
        let aligned = state.aligned_snapshot();
        assert_eq!(aligned.anchor_time, Some(latest_camera_time));
        assert!(aligned.detected_objects.is_some());
    }

    #[test]
    fn aligned_snapshot_waits_for_next_detection_when_detector_is_live() {
        let mut state = ViewerState::default();
        let first_time = Time::from_nanos(1_000_000_000);
        let second_time = Time::from_nanos(1_100_000_000);

        state.objects_status.update_publishers(1);
        state.push_camera_frame(first_time, CameraFrame::default());
        state.push_detected_objects(empty_detected_objects(first_time));
        assert_eq!(state.aligned_snapshot().anchor_time, Some(first_time));

        state.push_camera_frame(second_time, CameraFrame::default());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(first_time));
        assert!(aligned.detected_objects.is_some());

        state.push_detected_objects(empty_detected_objects(second_time));
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(second_time));
        assert!(aligned.detected_objects.is_some());
    }

    #[test]
    fn aligned_snapshot_keeps_current_frame_while_waiting_for_localization() {
        let mut state = ViewerState::default();
        let first_time = Time::from_nanos(1_000_000_000);
        let second_time = Time::from_nanos(1_300_000_000);

        state.localization_status.update_publishers(1);
        state.push_camera_frame(first_time, CameraFrame::default());
        state.push_localization(first_time, localization());
        assert_eq!(state.aligned_snapshot().anchor_time, Some(first_time));

        state.push_camera_frame(second_time, CameraFrame::default());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(first_time));
        assert_eq!(
            aligned.localization.as_ref().map(|sample| sample.time),
            Some(first_time)
        );

        state.push_localization(second_time, localization());
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(second_time));
        assert_eq!(
            aligned.localization.as_ref().map(|sample| sample.time),
            Some(second_time)
        );
    }

    #[test]
    fn aligned_snapshot_uses_localization_for_displayed_image_not_latest() {
        let mut state = ViewerState::default();
        let displayed_time = Time::from_nanos(1_000_000_000);
        let latest_time = Time::from_nanos(1_300_000_000);

        state.objects_status.update_publishers(1);
        state.localization_status.update_publishers(1);
        state.push_camera_frame(displayed_time, CameraFrame::default());
        state.push_detected_objects(empty_detected_objects(displayed_time));
        state.push_localization(displayed_time, localization());
        assert_eq!(state.aligned_snapshot().anchor_time, Some(displayed_time));

        state.push_camera_frame(latest_time, CameraFrame::default());
        state.push_localization(latest_time, None);
        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(displayed_time));
        assert_eq!(
            aligned.localization.as_ref().map(|sample| sample.time),
            Some(displayed_time)
        );
        assert!(
            aligned
                .localization
                .and_then(|sample| sample.inner)
                .is_some()
        );
    }

    #[test]
    fn aligned_snapshot_keeps_displayed_camera_frame_after_cache_eviction() {
        let mut state = ViewerState::default();
        let displayed_time = Time::from_nanos(0);

        state.objects_status.update_publishers(1);
        state.push_camera_frame(
            displayed_time,
            CameraFrame {
                sequence: 42,
                ..Default::default()
            },
        );
        state.push_detected_objects(empty_detected_objects(displayed_time));
        assert_eq!(state.aligned_snapshot().anchor_time, Some(displayed_time));

        for index in 1..=(CAMERA_FRAME_BUFFER_CAPACITY + 1) {
            state.push_camera_frame(
                Time::from_nanos(index as i64 * 100_000_000),
                CameraFrame::default(),
            );
        }

        let aligned = state.aligned_snapshot();

        assert_eq!(aligned.anchor_time, Some(displayed_time));
        assert_eq!(
            aligned
                .camera_frame
                .as_ref()
                .map(|frame| frame.inner.sequence),
            Some(42)
        );
    }
}
