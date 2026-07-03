use color_eyre::Result;
use coordinate_systems::{Camera, Field, Robot};
use linear_algebra::Isometry3;
use localization_factrs::{
    VinsFrontend, VinsFrontendError, VisualReprojectionAssociation,
    VisualReprojectionAssociationKind,
};
use ros_z::time::Time;
use types::{
    time_wrapper::TimeWrapper,
    visual_localization::{
        AssociationPoseHint, AssociationPoseHintSource, FieldMarkAssociation,
        FieldMarkAssociationSource, VisualLocalizationFrame,
    },
};

use crate::live_odometry::LiveVisualOdometryLocalization;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum GlobalVisualLock {
    Unlocked,
    WaitingForInitialBackend,
    WaitingForRecoveryBackend,
    Locked,
}

impl GlobalVisualLock {
    pub(crate) fn has_backend_result(self) -> bool {
        matches!(self, Self::Locked)
    }

    pub(crate) fn accepts_association_frame(self, has_global_associations: bool) -> bool {
        match self {
            Self::Locked => true,
            Self::Unlocked => !has_global_associations,
            Self::WaitingForInitialBackend | Self::WaitingForRecoveryBackend => false,
        }
    }

    pub(crate) fn should_ingest_backend_reset(self) -> bool {
        // A locked-state backend reset is the recovery signal emitted by field-mark association
        // after its consecutive-frame recovery gate. Ignore additional reset requests until the
        // backend result for the active initial lock or recovery has arrived.
        !matches!(
            self,
            Self::WaitingForInitialBackend | Self::WaitingForRecoveryBackend
        )
    }

    pub(crate) fn mark_waiting_for_backend(&mut self) {
        *self = match self {
            Self::Unlocked => Self::WaitingForInitialBackend,
            Self::Locked => Self::WaitingForRecoveryBackend,
            Self::WaitingForInitialBackend | Self::WaitingForRecoveryBackend => *self,
        };
    }

    pub(crate) fn mark_backend_result(&mut self) -> bool {
        match self {
            Self::WaitingForInitialBackend | Self::WaitingForRecoveryBackend => {
                *self = Self::Locked;
                true
            }
            Self::Unlocked | Self::Locked => false,
        }
    }

    pub(crate) fn reset_for_damping(&mut self) {
        *self = Self::Unlocked;
    }
}

pub(crate) fn association_pose_hint(
    time: Time,
    field_to_robot: Option<Isometry3<Field, Robot>>,
    source: AssociationPoseHintSource,
) -> TimeWrapper<Option<AssociationPoseHint>> {
    TimeWrapper {
        time,
        inner: field_to_robot.map(|pose| AssociationPoseHint {
            robot_to_field: pose.inverse(),
            source,
        }),
    }
}

pub(crate) fn handle_visual_localization_frame(
    frontend: &mut VinsFrontend,
    live_localization: &mut LiveVisualOdometryLocalization,
    global_visual_lock: &mut GlobalVisualLock,
    frame: TimeWrapper<VisualLocalizationFrame>,
) -> Result<(), VinsFrontendError> {
    let TimeWrapper { time, inner } = frame;
    let VisualLocalizationFrame {
        robot_to_camera,
        associations,
        backend_reset,
    } = inner;

    if let Some(robot_to_field) = backend_reset {
        if !global_visual_lock.should_ingest_backend_reset() {
            return Ok(());
        }

        frontend.ingest_global_pose(time.to_wallclock(), robot_to_field)?;
        global_visual_lock.mark_waiting_for_backend();
        live_localization.clear();
        return Ok(());
    }

    if !global_visual_lock.accepts_association_frame(has_global_associations(&associations)) {
        return Ok(());
    }

    ingest_visual_localization_associations(frontend, time, robot_to_camera, associations)
}

#[cfg(test)]
mod tests {
    use super::GlobalVisualLock;

    #[test]
    fn backend_result_only_anchors_when_waiting_for_initial_backend() {
        let mut lock = GlobalVisualLock::Unlocked;

        lock.mark_waiting_for_backend();

        assert_eq!(lock, GlobalVisualLock::WaitingForInitialBackend);
        assert!(lock.mark_backend_result());
        assert_eq!(lock, GlobalVisualLock::Locked);
        assert!(!lock.mark_backend_result());
        assert_eq!(lock, GlobalVisualLock::Locked);
    }

    #[test]
    fn backend_result_only_anchors_when_waiting_for_recovery_backend() {
        let mut lock = GlobalVisualLock::Locked;

        lock.mark_waiting_for_backend();

        assert_eq!(lock, GlobalVisualLock::WaitingForRecoveryBackend);
        assert!(lock.mark_backend_result());
        assert_eq!(lock, GlobalVisualLock::Locked);
        assert!(!lock.mark_backend_result());
    }

    #[test]
    fn backend_reset_is_ignored_while_waiting_for_backend_result() {
        assert!(GlobalVisualLock::Unlocked.should_ingest_backend_reset());
        assert!(GlobalVisualLock::Locked.should_ingest_backend_reset());
        assert!(!GlobalVisualLock::WaitingForInitialBackend.should_ingest_backend_reset());
        assert!(!GlobalVisualLock::WaitingForRecoveryBackend.should_ingest_backend_reset());
    }

    #[test]
    fn unlocked_or_locked_backend_results_do_not_anchor() {
        let mut unlocked = GlobalVisualLock::Unlocked;
        let mut locked = GlobalVisualLock::Locked;

        assert!(!unlocked.mark_backend_result());
        assert_eq!(unlocked, GlobalVisualLock::Unlocked);
        assert!(!locked.mark_backend_result());
        assert_eq!(locked, GlobalVisualLock::Locked);
    }

    #[test]
    fn association_frames_are_dropped_while_waiting_for_backend_result() {
        assert!(GlobalVisualLock::Unlocked.accepts_association_frame(false));
        assert!(!GlobalVisualLock::Unlocked.accepts_association_frame(true));
        assert!(GlobalVisualLock::Locked.accepts_association_frame(false));
        assert!(GlobalVisualLock::Locked.accepts_association_frame(true));
        assert!(!GlobalVisualLock::WaitingForInitialBackend.accepts_association_frame(false));
        assert!(!GlobalVisualLock::WaitingForInitialBackend.accepts_association_frame(true));
        assert!(!GlobalVisualLock::WaitingForRecoveryBackend.accepts_association_frame(false));
        assert!(!GlobalVisualLock::WaitingForRecoveryBackend.accepts_association_frame(true));
    }
}

fn has_global_associations(associations: &[FieldMarkAssociation]) -> bool {
    associations
        .iter()
        .any(|association| matches!(association.source, FieldMarkAssociationSource::GlobalUnique))
}

fn ingest_visual_localization_associations(
    frontend: &mut VinsFrontend,
    time: Time,
    robot_to_camera: Isometry3<Robot, Camera>,
    associations: Vec<FieldMarkAssociation>,
) -> Result<(), VinsFrontendError> {
    let associations = associations
        .into_iter()
        .map(|association| VisualReprojectionAssociation {
            detection: association.detection,
            field_point: association.field_point,
            kind: match association.source {
                FieldMarkAssociationSource::GlobalUnique => {
                    VisualReprojectionAssociationKind::GlobalUnique
                }
                FieldMarkAssociationSource::PoseHint => VisualReprojectionAssociationKind::PoseHint,
            },
        });
    frontend.ingest_visual_reprojection_associations(
        time.to_wallclock(),
        associations,
        robot_to_camera,
    )
}
