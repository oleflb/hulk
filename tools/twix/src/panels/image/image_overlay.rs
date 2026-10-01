use std::{collections::BTreeSet, sync::Arc, time::Duration};

use color_eyre::{Report, eyre::Context as _};
use coordinate_systems::Pixel;
use eframe::egui::{Popup, PopupCloseBehavior, Ui};
use projection::camera_matrix::CameraMatrix;
use ros_z::{Message, time::Time};
use ros_z_debug::{
    ObservationPolicy, RetentionPolicy, SampleRecord, TargetIdentity, TopicObservation,
    TopicReference,
};
use serde_json::{Value, json};
use types::time_wrapper::TimeWrapper;

use crate::{
    backend::RobotBackend,
    repaint::{ObservationContext, ObservationRepaint, RepaintOnUpdates},
};
use twix_visualization::twix_painter::TwixPainter;

use super::overlays::{
    BallDetectionOverlay, FieldBorderOverlay, HorizonOverlay, LineDetectionOverlay,
    ObjectDetectionOverlay, PoseDetectionOverlay, ProjectedFieldLinesOverlay,
};

const OVERLAY_HISTORY_CAPACITY: usize = 4096;

fn overlay_retention() -> RetentionPolicy {
    RetentionPolicy::time_window_with_max_samples(
        Duration::MAX,
        OVERLAY_HISTORY_CAPACITY.try_into().unwrap(),
    )
    .unwrap()
}

pub(super) struct ImageOverlays {
    line_detection: OverlaySlot<LineDetectionOverlay>,
    ball_detection: OverlaySlot<BallDetectionOverlay>,
    horizon: OverlaySlot<HorizonOverlay>,
    field_border: OverlaySlot<FieldBorderOverlay>,
    object_detection: OverlaySlot<ObjectDetectionOverlay>,
    pose_detection: OverlaySlot<PoseDetectionOverlay>,
    projected_field_lines: OverlaySlot<ProjectedFieldLinesOverlay>,
}

impl ImageOverlays {
    pub(super) fn new<C>(value: Option<&Value>, context: &C) -> Self
    where
        C: ObservationContext,
    {
        Self {
            line_detection: OverlaySlot::new(value, context),
            ball_detection: OverlaySlot::new(value, context),
            horizon: OverlaySlot::new(value, context),
            field_border: OverlaySlot::new(value, context),
            object_detection: OverlaySlot::new(value, context),
            pose_detection: OverlaySlot::new(value, context),
            projected_field_lines: OverlaySlot::new(value, context),
        }
    }

    pub(super) fn ui<C>(&mut self, ui: &mut Ui, context: &C)
    where
        C: ObservationContext,
    {
        Popup::menu(&ui.button("Overlays"))
            .close_behavior(PopupCloseBehavior::CloseOnClickOutside)
            .show(|ui| {
                self.line_detection.checkbox(ui, context);
                self.ball_detection.checkbox(ui, context);
                self.horizon.checkbox(ui, context);
                self.field_border.checkbox(ui, context);
                self.object_detection.checkbox(ui, context);
                self.pose_detection.checkbox(ui, context);
                self.projected_field_lines.checkbox(ui, context);
            });
    }

    pub(super) fn prepare(&self, time: Time) -> OverlaySnapshot {
        OverlaySnapshot {
            objects: self.object_detection.prepare(time),
            poses: self.pose_detection.prepare(time),
            horizon: self.horizon.prepare(time),
            border: self.field_border.prepare(time),
            field: self.projected_field_lines.prepare(time),
            field_unavailable: self
                .projected_field_lines
                .overlay
                .as_ref()
                .is_some_and(|overlay| overlay.unavailable(time)),
        }
    }

    pub(super) fn ready(&self, snapshot: &OverlaySnapshot) -> bool {
        (!self.object_detection.active || snapshot.objects.is_some())
            && (!self.pose_detection.active || snapshot.poses.is_some())
            && (!self.horizon.active || snapshot.horizon.is_some())
            && (!self.field_border.active || snapshot.border.is_some())
            && (!self.projected_field_lines.active
                || snapshot.field.is_some()
                || snapshot.field_unavailable)
    }

    pub(super) fn detection_times(&self) -> Option<BTreeSet<Time>> {
        let objects = self.object_detection.active.then(|| {
            self.object_detection
                .overlay
                .as_ref()
                .map(|o| {
                    o.object_detections
                        .get_all()
                        .iter()
                        .map(|s| s.value.time)
                        .collect::<BTreeSet<_>>()
                })
                .unwrap_or_default()
        });
        let poses = self.pose_detection.active.then(|| {
            self.pose_detection
                .overlay
                .as_ref()
                .map(|o| {
                    o.poses
                        .get_all()
                        .iter()
                        .map(|s| s.value.time)
                        .collect::<BTreeSet<_>>()
                })
                .unwrap_or_default()
        });
        match (objects, poses) {
            (Some(mut objects), Some(poses)) => {
                objects.retain(|time| poses.contains(time));
                Some(objects)
            }
            (objects, poses) => objects.or(poses),
        }
    }

    pub(super) fn retain_enabled(&self, snapshot: &mut OverlaySnapshot) {
        if !self.object_detection.active {
            snapshot.objects = None;
        }
        if !self.pose_detection.active {
            snapshot.poses = None;
        }
        if !self.horizon.active {
            snapshot.horizon = None;
        }
        if !self.field_border.active {
            snapshot.border = None;
        }
        if !self.projected_field_lines.active {
            snapshot.field = None;
        }
    }

    pub(super) fn restore_projection(&self, snapshot: &mut OverlaySnapshot, time: Time) -> bool {
        snapshot.field = self.projected_field_lines.prepare(time);
        snapshot.field.is_some() || !self.projected_field_lines.active
    }

    pub(super) fn enrich_residual(&self, snapshot: &mut OverlaySnapshot, time: Time) {
        if let Some(overlay) = &self.projected_field_lines.overlay
            && let Some(field) = &mut snapshot.field
        {
            overlay.enrich_residual(field, time);
        }
    }

    pub(super) fn invalidate_projection(&self, snapshot: &mut OverlaySnapshot) -> bool {
        if let Some(overlay) = &self.projected_field_lines.overlay
            && snapshot
                .field
                .as_ref()
                .is_some_and(|field| !overlay.valid(field))
        {
            snapshot.field = None;
            return true;
        }
        false
    }

    pub(super) fn save(&self) -> Value {
        json!({
            LineDetectionOverlay::STORAGE_KEY: self.line_detection.save(),
            BallDetectionOverlay::STORAGE_KEY: self.ball_detection.save(),
            HorizonOverlay::STORAGE_KEY: self.horizon.save(),
            FieldBorderOverlay::STORAGE_KEY: self.field_border.save(),
            ObjectDetectionOverlay::STORAGE_KEY: self.object_detection.save(),
            PoseDetectionOverlay::STORAGE_KEY: self.pose_detection.save(),
            ProjectedFieldLinesOverlay::STORAGE_KEY: self.projected_field_lines.save(),
        })
    }
}

#[derive(Default)]
pub(super) struct OverlaySnapshot {
    objects: Option<<ObjectDetectionOverlay as ImageOverlay>::Sample>,
    poses: Option<<PoseDetectionOverlay as ImageOverlay>::Sample>,
    horizon: Option<<HorizonOverlay as ImageOverlay>::Sample>,
    border: Option<<FieldBorderOverlay as ImageOverlay>::Sample>,
    field: Option<<ProjectedFieldLinesOverlay as ImageOverlay>::Sample>,
    pub(super) field_unavailable: bool,
}

impl OverlaySnapshot {
    pub(super) fn paint(&self, painter: &TwixPainter<Pixel>) {
        if let Some(sample) = &self.field {
            ProjectedFieldLinesOverlay::paint(painter, sample);
        }
        if let Some(sample) = &self.horizon {
            HorizonOverlay::paint(painter, sample);
        }
        if let Some(sample) = &self.border {
            FieldBorderOverlay::paint(painter, sample);
        }
        if let Some(sample) = &self.objects {
            ObjectDetectionOverlay::paint(painter, sample);
        }
        if let Some(sample) = &self.poses {
            PoseDetectionOverlay::paint(painter, sample);
        }
    }
}

impl Default for ImageOverlays {
    fn default() -> Self {
        Self {
            line_detection: OverlaySlot::inactive(),
            ball_detection: OverlaySlot::inactive(),
            horizon: OverlaySlot::inactive(),
            field_border: OverlaySlot::inactive(),
            object_detection: OverlaySlot::inactive(),
            pose_detection: OverlaySlot::inactive(),
            projected_field_lines: OverlaySlot::inactive(),
        }
    }
}

struct OverlaySlot<T> {
    active: bool,
    overlay: Option<T>,
    error: Option<String>,
}

impl<T> OverlaySlot<T>
where
    T: ImageOverlay,
{
    fn new<C>(value: Option<&Value>, context: &C) -> Self
    where
        C: ObservationContext,
    {
        let mut slot = Self::inactive();
        slot.active = value
            .and_then(|value| value.get(T::STORAGE_KEY))
            .and_then(|value| value.get("active"))
            .and_then(Value::as_bool)
            .unwrap_or(false);
        if slot.active {
            slot.recreate(context);
        }
        slot
    }

    fn inactive() -> Self {
        Self {
            active: false,
            overlay: None,
            error: None,
        }
    }

    fn checkbox<C>(&mut self, ui: &mut Ui, context: &C)
    where
        C: ObservationContext,
    {
        let changed = ui.checkbox(&mut self.active, T::NAME).changed();
        if changed {
            if self.active {
                self.recreate(context);
            } else {
                self.overlay = None;
                self.error = None;
            }
        }
        if let Some(error) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, error);
        }
    }

    fn recreate<C>(&mut self, context: &C)
    where
        C: ObservationContext,
    {
        match T::new(context) {
            Ok(overlay) => {
                self.overlay = Some(overlay);
                self.error = None;
            }
            Err(error) => {
                self.overlay = None;
                self.error = Some(format!("{}: {error:#}", T::NAME));
            }
        }
    }

    fn prepare(&self, time: Time) -> Option<T::Sample> {
        self.overlay.as_ref()?.prepare(time)
    }

    fn save(&self) -> Value {
        json!({"active": self.active})
    }
}

pub(super) trait ImageOverlay: Sized {
    type Sample;
    const NAME: &'static str;
    const STORAGE_KEY: &'static str;

    fn new<C>(context: &C) -> Result<Self, Report>
    where
        C: ObservationContext;

    fn prepare(&self, time: Time) -> Option<Self::Sample>;
    fn paint(painter: &TwixPainter<Pixel>, sample: &Self::Sample);
}

pub(super) struct OverlayObservation<T> {
    backend: Arc<RobotBackend>,
    topic: TopicReference,
    observation: TopicObservation<T>,
    _repaint: ObservationRepaint,
}

impl<T> OverlayObservation<T>
where
    T: Message + Send + Sync + 'static,
    T::Codec: Send + Sync,
{
    pub(super) fn new<C>(context: &C, topic: &str) -> Result<Self, Report>
    where
        C: ObservationContext,
    {
        Self::with_policy(context, topic, ObservationPolicy::default())
    }

    pub(super) fn with_policy<C>(
        context: &C,
        topic: &str,
        policy: ObservationPolicy,
    ) -> Result<Self, Report>
    where
        C: ObservationContext,
    {
        let (observation, repaint) = create_typed_observation(context, topic, policy)?;
        Ok(Self {
            backend: Arc::clone(context.backend()),
            topic: TopicReference::new(topic)?,
            observation,
            _repaint: repaint,
        })
    }

    pub(super) fn latest(&self) -> Option<Arc<SampleRecord<T>>> {
        let topic = self.resolved_topic()?;
        self.observation
            .latest()
            .filter(|record| record.metadata.resolved_topic == topic)
    }

    pub(super) fn get_all(&self) -> Vec<Arc<SampleRecord<T>>> {
        let Some(topic) = self.resolved_topic() else {
            return Vec::new();
        };
        self.observation
            .get_all()
            .into_iter()
            .filter(|record| record.metadata.resolved_topic == topic)
            .collect()
    }

    fn resolved_topic(&self) -> Option<String> {
        // Retargeting is asynchronous; the observer can still expose the previous cache.
        self.topic
            .resolve(&TargetIdentity::new(self.backend.namespace()).ok()?)
            .ok()
    }
}

impl<T> OverlayObservation<TimeWrapper<T>>
where
    TimeWrapper<T>: Message + Send + Sync + 'static,
    <TimeWrapper<T> as Message>::Codec: Send + Sync,
{
    pub(super) fn at_time(&self, time: Time) -> Option<Arc<SampleRecord<TimeWrapper<T>>>> {
        self.get_all()
            .into_iter()
            .rev()
            .find(|record| record.value.time == time)
    }

    pub(super) fn interpolate<R>(
        &self,
        time: Time,
        interpolate: impl FnOnce(&T, &T, f32) -> Option<R>,
    ) -> Option<R> {
        let samples = self.get_all();
        let (before, after, fraction) = bracket(&samples, time)?;
        interpolate(&before.inner, &after.inner, fraction)
    }
}

pub(super) fn bracket<T>(
    samples: &[Arc<SampleRecord<TimeWrapper<T>>>],
    time: Time,
) -> Option<(&TimeWrapper<T>, &TimeWrapper<T>, f32)> {
    let before = samples
        .iter()
        .filter(|s| s.value.time <= time)
        .max_by_key(|s| s.value.time)?;
    if before.value.time == time {
        return Some((&before.value, &before.value, 0.0));
    }
    let after = samples
        .iter()
        .rev()
        .filter(|s| s.value.time >= time)
        .min_by_key(|s| s.value.time)?;
    let gap = after.value.time.duration_since(before.value.time);
    // Brackets, not transport age, bound interpolation; callers reject discontinuities.
    let fraction = if gap.is_zero() {
        0.0
    } else {
        time.duration_since(before.value.time).as_secs_f32() / gap.as_secs_f32()
    };
    Some((&before.value, &after.value, fraction))
}

pub(super) fn valid_intrinsics(intrinsics: &projection::intrinsic::Intrinsic) -> bool {
    intrinsics.focals.iter().all(|v| v.is_finite() && *v > 0.0)
        && intrinsics
            .optical_center
            .inner
            .iter()
            .all(|v| v.is_finite())
}

pub(super) fn interpolate_transform<From, To>(
    a: linear_algebra::Isometry3<From, To>,
    b: linear_algebra::Isometry3<From, To>,
    t: f32,
) -> Option<linear_algebra::Isometry3<From, To>> {
    if !a
        .inner
        .to_homogeneous()
        .iter()
        .chain(b.inner.to_homogeneous().iter())
        .all(|v| v.is_finite())
    {
        return None;
    }
    Some(linear_algebra::Isometry3::wrap(
        a.inner.lerp_slerp(&b.inner, t),
    ))
}

impl OverlayObservation<TimeWrapper<CameraMatrix>> {
    pub(super) fn camera_at(&self, time: Time) -> Option<CameraSample> {
        self.interpolate(time, |a, b, t| {
            // Calibration changes are discontinuities, not motion to interpolate.
            if a.intrinsics != b.intrinsics || a.image_size != b.image_size {
                return None;
            }
            if !valid_intrinsics(&a.intrinsics) {
                return None;
            }
            let ground_to_robot = interpolate_transform(a.ground_to_robot, b.ground_to_robot, t)?;
            let robot_to_head = interpolate_transform(a.robot_to_head, b.robot_to_head, t)?;
            let head_to_camera = interpolate_transform(a.head_to_camera, b.head_to_camera, t)?;
            let robot_to_camera = head_to_camera * robot_to_head;
            Some(CameraSample {
                robot_to_camera,
                intrinsics: a.intrinsics,
                horizon: projection::horizon::Horizon::from_parameters(
                    robot_to_camera * ground_to_robot,
                    &a.intrinsics,
                ),
            })
        })
    }
}

pub(super) struct CameraSample {
    pub(super) robot_to_camera:
        linear_algebra::Isometry3<coordinate_systems::Robot, coordinate_systems::Camera>,
    pub(super) intrinsics: projection::intrinsic::Intrinsic,
    pub(super) horizon: Option<projection::horizon::Horizon>,
}

fn create_typed_observation<T>(
    context: &impl ObservationContext,
    topic: &str,
    policy: ObservationPolicy,
) -> Result<(TopicObservation<T>, ObservationRepaint), Report>
where
    T: Message + Send + Sync + 'static,
    T::Codec: Send + Sync,
{
    let runtime_handle = context.backend().runtime_handle().clone();
    // ros_z_debug spawns observation tasks internally and needs a current runtime.
    let _runtime_context = runtime_handle.enter();
    let observation = context
        .backend()
        .observer()
        .observe_typed::<T>(topic)
        .wrap_err_with(|| format!("failed to create typed topic observation for {topic}"))?
        .policy(policy)
        .retention(overlay_retention())
        .spawn();
    let repaint = observation.repaint_on_updates(context);
    Ok((observation, repaint))
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    pub(in crate::panels::image) async fn publish_until<T>(
        publisher: &ros_z::pubsub::Publisher<T>,
        value: &T,
        mut received: impl FnMut() -> bool,
    ) where
        T: Message + Send + Sync,
        T::Codec: Send + Sync,
    {
        tokio::time::timeout(Duration::from_secs(8), async {
            while !received() {
                publisher.publish(value).await.unwrap();
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("sample should reach asynchronous observer");
    }
}
