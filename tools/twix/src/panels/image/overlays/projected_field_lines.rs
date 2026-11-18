use std::{f32::consts::TAU, sync::Arc, time::Duration};

use color_eyre::Report;
use coordinate_systems::{Camera, Field, Pixel};
use eframe::egui::{Color32, Stroke};
use linear_algebra::{Isometry3, Point2, Point3, point};
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{
    qos::{QosDurability, QosProfile},
    time::Time,
};
use ros_z_debug::{
    ObservationPolicy, SampleRecord,
    query::{TemporalMatch, TimeProjection, plan::Selection},
};
use types::{
    camera_geometry::{CameraGeometry, interpolate_camera_geometry},
    field_dimensions::{FieldDimensions, Half, Side},
    localization::{LocalizationEstimate, LocalizationStatus},
    time_wrapper::TimeWrapper,
    visual_localization::VisualLocalizationFrame,
};

use super::super::{
    image_overlay::ImageOverlay,
    inputs::{OverlayInput, OverlayInputs},
};
use crate::panels::field_markings;
use twix_visualization::twix_painter::TwixPainter;

const NEAR_Z: f32 = 1.0e-4;
const FIELD_STROKE: Stroke = Stroke {
    width: 2.0,
    color: Color32::from_rgb(80, 220, 255),
};
const RESIDUAL_STROKE: Stroke = Stroke {
    width: 2.0,
    color: Color32::from_rgb(255, 80, 200),
};

#[derive(Default)]
enum Solution {
    #[default]
    Original,
    Comparison,
}

#[derive(Default)]
pub(in crate::panels::image) struct ProjectedFieldLinesOverlay {
    solution: Solution,
    sample: Option<ProjectedFieldSample>,
}

impl ProjectedFieldLinesOverlay {
    pub fn comparison() -> Self {
        Self {
            solution: Solution::Comparison,
            sample: None,
        }
    }

    fn stroke(&self) -> Stroke {
        match self.solution {
            Solution::Original => FIELD_STROKE,
            Solution::Comparison => Stroke::new(FIELD_STROKE.width, Color32::RED),
        }
    }
}

impl ImageOverlay for ProjectedFieldLinesOverlay {
    fn inputs(&self) -> Vec<OverlayInput> {
        let bracket = Selection::Bracket {
            max_gap: Duration::from_secs(2),
        };
        let mut dimensions =
            OverlayInput::new::<FieldDimensions>("field_dimensions", Selection::latest());
        dimensions.source.projection = TimeProjection::SourceTime;
        let (estimate_topic, status_topic) = match self.solution {
            Solution::Original => ("localization/estimate", "localization/status"),
            Solution::Comparison => (
                "comparison/localization/estimate",
                "comparison/localization/status",
            ),
        };
        let mut status = OverlayInput::new::<LocalizationStatus>(status_topic, Selection::latest());
        let latched = ObservationPolicy::default().with_subscriber_qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        });
        dimensions.source.live_policy = latched;
        status.source.live_policy = latched;
        let mut inputs = vec![
            OverlayInput::new::<TimeWrapper<CameraMatrix>>("camera_matrix", bracket.clone()),
            OverlayInput::new::<LocalizationEstimate>(estimate_topic, bracket),
            status,
            dimensions,
        ];
        if matches!(self.solution, Solution::Original) {
            inputs.push(OverlayInput::new::<TimeWrapper<VisualLocalizationFrame>>(
                "field_mark_association/visual_localization_local",
                Selection::Exact,
            ));
        }
        inputs
    }

    fn prepare(&mut self, image_time: Time, inputs: &mut OverlayInputs) -> Result<(), Report> {
        self.sample = None;
        let camera = inputs.decode::<TimeWrapper<CameraMatrix>>(0, image_time)?;
        let poses = inputs.decode::<LocalizationEstimate>(1, image_time)?;
        let status = inputs.decode::<LocalizationStatus>(2, image_time)?;
        let dimensions = inputs
            .decode::<FieldDimensions>(3, image_time)
            .ok()
            .flatten();
        let associations = inputs
            .decode::<TimeWrapper<VisualLocalizationFrame>>(4, image_time)
            .ok()
            .flatten();
        self.sample = (|| {
            let status = status.as_ref()?.last();
            let epoch = status.value.epoch;
            let generation = status.value.generation;
            let (before, after) = match poses.as_ref()? {
                TemporalMatch::Exact(record) => (record, record),
                TemporalMatch::Bracket { before, after } => (before, after),
                TemporalMatch::Before(_) => return None,
            };
            if [before, after]
                .iter()
                .any(|p| p.value.epoch != epoch || p.value.generation != generation)
            {
                return None;
            }
            let gap = after.value.time.duration_since(before.value.time);
            let fraction = if gap.is_zero() {
                0.0
            } else {
                image_time.duration_since(before.value.time).as_secs_f32() / gap.as_secs_f32()
            };
            // Rendering interpolates its own snapshots; the localization node publishes only solves.
            let field_to_robot = super::camera::interpolate_transform(
                Isometry3::wrap(
                    before
                        .value
                        .robot_to_field?
                        .pose
                        .inner
                        .cast::<f32>()
                        .inverse(),
                ),
                Isometry3::wrap(
                    after
                        .value
                        .robot_to_field?
                        .pose
                        .inner
                        .cast::<f32>()
                        .inverse(),
                ),
                fraction,
            );
            let (before, after) = match camera.as_ref()? {
                TemporalMatch::Exact(record) => (record, record),
                TemporalMatch::Bracket { before, after } => (before, after),
                TemporalMatch::Before(_) => return None,
            };
            let geometry = |record: &Arc<SampleRecord<TimeWrapper<CameraMatrix>>>| TimeWrapper {
                time: record.value.time,
                inner: CameraGeometry::from(&record.value.inner),
            };
            let camera = interpolate_camera_geometry(
                &geometry(before),
                &geometry(after),
                image_time,
                Duration::from_secs(2),
            )?;
            let sample = ProjectedFieldSample {
                projection: FieldProjection {
                    field_to_camera: camera.robot_to_camera * field_to_robot,
                    intrinsics: camera.intrinsics,
                },
                dimensions: dimensions
                    .as_ref()
                    .map_or(FieldDimensions::SPL_2025, |p| p.last().value),
                associations: associations.as_ref().map(|p| p.last().clone()).filter(|p| {
                    p.value.inner.epoch == epoch && p.value.inner.generation == generation
                }),
            };
            sample
                .projection
                .field_to_camera
                .inner
                .to_homogeneous()
                .iter()
                .all(|v| v.is_finite())
                .then_some(sample)
        })();
        Ok(())
    }

    fn image_overlay(&self, painter: &TwixPainter<Pixel>) {
        let Some(sample) = &self.sample else {
            return;
        };
        sample
            .projection
            .draw_field(painter, &sample.dimensions, self.stroke());
        if let Some(frame) = &sample.associations {
            for association in &frame.value.inner.associations {
                if !association.detection.inner.iter().all(|v| v.is_finite()) {
                    continue;
                }
                if let Some(projected) = sample.projection.project(association.field_point) {
                    painter.line_segment(association.detection, projected, RESIDUAL_STROKE);
                    painter.circle_filled(projected, 3.5, RESIDUAL_STROKE.color);
                }
            }
        }
    }
}

pub(in crate::panels::image) struct ProjectedFieldSample {
    projection: FieldProjection,
    dimensions: FieldDimensions,
    associations: Option<Arc<SampleRecord<TimeWrapper<VisualLocalizationFrame>>>>,
}

struct FieldProjection {
    field_to_camera: Isometry3<Field, Camera>,
    intrinsics: Intrinsic,
}

impl FieldProjection {
    fn project(&self, point: Point3<Field>) -> Option<Point2<Pixel>> {
        self.project_camera(self.field_to_camera * point)
    }

    fn project_camera(&self, point: Point3<Camera>) -> Option<Point2<Pixel>> {
        if !point.inner.iter().all(|v| v.is_finite())
            || point.z() < NEAR_Z
            || !self.intrinsics.is_valid()
        {
            return None;
        }
        let pixel = self.intrinsics.project(point.coords());
        pixel.inner.iter().all(|v| v.is_finite()).then_some(pixel)
    }

    fn segment(&self, start: Point2<Field>, end: Point2<Field>) -> Option<[Point2<Pixel>; 2]> {
        let mut start = self.field_to_camera * start.extend(0.0);
        let mut end = self.field_to_camera * end.extend(0.0);
        if !start
            .inner
            .iter()
            .chain(end.inner.iter())
            .all(|v| v.is_finite())
            || (start.z() < NEAR_Z && end.z() < NEAR_Z)
        {
            return None;
        }
        if start.z() < NEAR_Z || end.z() < NEAR_Z {
            let t = (NEAR_Z - start.z()) / (end.z() - start.z());
            let mut clipped = start + (end - start) * t;
            // Roundoff in interpolation must not put the clipped endpoint behind the plane.
            clipped.inner.z = NEAR_Z;
            if start.z() < NEAR_Z {
                start = clipped;
            } else {
                end = clipped;
            }
        }
        Some([self.project_camera(start)?, self.project_camera(end)?])
    }

    fn draw_field(&self, painter: &TwixPainter<Pixel>, d: &FieldDimensions, stroke: Stroke) {
        let line = |a, b| {
            if let Some([a, b]) = self.segment(a, b) {
                painter.line_segment(a, b, stroke);
            }
        };
        for [start, end] in field_markings::field_segments(d) {
            line(start, end);
        }
        self.arc(painter, field_markings::center_circle(d), stroke);
        for half in [Half::Own, Half::Opponent] {
            for [start, end] in field_markings::half_segments(d, half) {
                line(start, end);
            }
            let spot = d.penalty_spot(half);
            let r = d.penalty_marker_size / 2.0;
            line(point![spot.x() - r, 0.0], point![spot.x() + r, 0.0]);
            line(point![spot.x(), -r], point![spot.x(), r]);
            for side in [Side::Left, Side::Right] {
                for [start, end] in field_markings::box_segments(d, half, side) {
                    line(start, end);
                }
                self.arc(painter, field_markings::corner_arc(d, half, side), stroke);
            }
        }
    }

    fn arc(
        &self,
        painter: &TwixPainter<Pixel>,
        (center, radius, start, end): (Point2<Field>, f32, f32, f32),
        stroke: Stroke,
    ) {
        if !radius.is_finite() || radius <= 0.0 {
            return;
        }
        let count = ((end - start).abs() / TAU * 256.0).ceil().clamp(1.0, 256.0) as usize;
        let at = |angle: f32| {
            self.project(point![
                center.x() + radius * angle.cos(),
                center.y() + radius * angle.sin(),
                0.0
            ])
        };
        let mut previous = at(start);
        for index in 1..=count {
            let next = at(start + (end - start) * index as f32 / count as f32);
            if let (Some(a), Some(b)) = (previous, next) {
                painter.line_segment(a, b, stroke);
            }
            previous = next;
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;
    use crate::panels::image::tests::raw_sample;
    use linear_algebra::vector;
    use ros_z_debug::query::plan::State;
    use types::localization::{LocalizationState, PoseEstimate};

    fn rendered_strokes(paint: impl Fn(&TwixPainter<Pixel>)) -> Vec<Stroke> {
        use eframe::egui::{Context, Shape};
        use twix_visualization::twix_painter::Orientation;

        let output = Context::default().run_ui(Default::default(), |ui| {
            let (_, painter) = TwixPainter::allocate(
                ui,
                linear_algebra::vector![200.0, 200.0],
                point![0.0, 0.0],
                Orientation::LeftHanded,
            );
            paint(&painter);
        });
        output
            .shapes
            .iter()
            .filter_map(|shape| match shape.shape {
                Shape::LineSegment { stroke, .. } => Some(stroke),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn missing_comparison_topics_do_not_hide_original() -> Result<(), Report> {
        use crate::panels::image::image_overlay::ImageOverlays;
        use ros_z_debug::query::plan::Frame;
        use serde_json::json;

        let mut overlays = ImageOverlays::new(Some(&json!({
            "projected_field_lines": {"active": true},
            "comparison_projected_field_lines": {"active": true},
        })));
        overlays.declarations("/replay", true)?;
        let time = Time::from_nanos(20);
        let camera = raw_sample(
            "/replay/camera_matrix",
            1,
            20,
            20,
            &TimeWrapper {
                time,
                inner: CameraMatrix {
                    head_to_camera: Isometry3::from_translation(vector![0.0, 0.0, 10.0]),
                    intrinsics: Intrinsic::new(
                        nalgebra::vector![100.0, 100.0],
                        point![100.0, 100.0],
                    ),
                    ..Default::default()
                },
            },
        );
        let pose = PoseEstimate {
            pose: Isometry3::identity(),
            covariance: nalgebra::SMatrix::zeros(),
        };
        let estimate = raw_sample(
            "/replay/localization/estimate",
            1,
            20,
            20,
            &LocalizationEstimate {
                time,
                epoch: 1,
                generation: 2,
                robot_to_local: pose,
                robot_to_field: Some(PoseEstimate {
                    pose: Isometry3::identity(),
                    covariance: nalgebra::SMatrix::zeros(),
                }),
            },
        );
        let status = raw_sample(
            "/replay/localization/status",
            1,
            20,
            20,
            &LocalizationStatus {
                time,
                epoch: 1,
                generation: 2,
                state: LocalizationState::Tracking,
                heading: None,
            },
        );
        for missing in [
            State::Pending,
            State::Ready(vec![]),
            State::Failed("missing topic".into()),
        ] {
            overlays.prepare(&Frame {
                primary: raw_sample("/replay/inputs/left_image", 1, 20, 20, &0u64),
                inputs: vec![
                    State::Ready(vec![camera.clone()]),
                    State::Ready(vec![estimate.clone()]),
                    State::Ready(vec![status.clone()]),
                    State::Ready(vec![]),
                    State::Ready(vec![]),
                    State::Ready(vec![camera.clone()]),
                    missing.clone(),
                    missing,
                    State::Ready(vec![]),
                ],
            });
            let strokes = rendered_strokes(|painter| overlays.paint(painter, time));
            assert!(strokes.len() > 256);
            assert!(
                strokes
                    .iter()
                    .all(|stroke| stroke.color == FIELD_STROKE.color)
            );
        }
        Ok(())
    }

    #[test]
    fn both_solutions_draw_lines_and_arcs_in_their_own_color() {
        for (mut overlay, color) in [
            (ProjectedFieldLinesOverlay::default(), FIELD_STROKE.color),
            (ProjectedFieldLinesOverlay::comparison(), Color32::RED),
        ] {
            overlay.sample = Some(ProjectedFieldSample {
                projection: FieldProjection {
                    field_to_camera: Isometry3::from_translation(vector![0.0, 0.0, 10.0]),
                    intrinsics: Intrinsic::new(
                        nalgebra::vector![100.0, 100.0],
                        point![100.0, 100.0],
                    ),
                },
                dimensions: FieldDimensions::SPL_2025,
                associations: None,
            });
            let strokes = rendered_strokes(|painter| overlay.image_overlay(painter));
            assert!(strokes.len() > 256, "field includes tessellated arcs");
            assert!(strokes.iter().all(|stroke| stroke.color == color));
        }
    }

    #[test]
    fn exposure_snapshot_requires_matching_pose_generation_and_recovers_delayed_inputs()
    -> Result<(), Report> {
        for overlay in [
            ProjectedFieldLinesOverlay::default(),
            ProjectedFieldLinesOverlay::comparison(),
        ] {
            check_exposure_snapshot(overlay)?;
        }
        Ok(())
    }

    fn check_exposure_snapshot(mut overlay: ProjectedFieldLinesOverlay) -> Result<(), Report> {
        let declarations = overlay.inputs();
        let epoch = match overlay.solution {
            Solution::Original => 1,
            Solution::Comparison => 42,
        };
        let camera = |time| {
            raw_sample(
                "/camera_matrix",
                time as u64,
                time,
                time,
                &TimeWrapper {
                    time: Time::from_nanos(time),
                    inner: CameraMatrix::default(),
                },
            )
        };
        let mut pose_id = 0;
        let mut pose = |time, epoch, generation, tracked: bool| {
            pose_id += 1;
            raw_sample(
                &declarations[1].source.topic,
                pose_id,
                time,
                time,
                &LocalizationEstimate {
                    time: Time::from_nanos(time),
                    epoch,
                    generation,
                    robot_to_local: PoseEstimate {
                        pose: Isometry3::identity(),
                        covariance: nalgebra::SMatrix::zeros(),
                    },
                    robot_to_field: tracked.then_some(PoseEstimate {
                        pose: Isometry3::from_translation(vector![time as f64, 0.0, 0.0]),
                        covariance: nalgebra::SMatrix::zeros(),
                    }),
                },
            )
        };
        let status = raw_sample(
            &declarations[2].source.topic,
            1,
            0,
            0,
            &LocalizationStatus {
                time: Time::from_nanos(0),
                epoch,
                generation: 2,
                state: LocalizationState::Tracking,
                heading: None,
            },
        );
        let mut states = vec![
            State::Ready(vec![camera(10), camera(30)]),
            State::Pending,
            State::Ready(vec![status]),
            State::Ready(vec![]),
            State::Ready(vec![]),
        ];
        let mut inputs = OverlayInputs::default();
        let time = Time::from_nanos(20);
        inputs.update(&states);
        overlay.prepare(time, &mut inputs)?;
        assert!(overlay.sample.is_none());
        states[1] = State::Ready(vec![pose(10, epoch, 2, true), pose(30, epoch, 2, true)]);
        inputs.update(&states);
        overlay.prepare(time, &mut inputs)?;
        let sample = overlay
            .sample
            .as_ref()
            .ok_or_else(|| color_eyre::eyre::eyre!("missing interpolated field"))?;
        assert_eq!(sample.projection.field_to_camera.inner.translation.x, -20.0);
        assert!(sample.associations.is_none());
        for (after_epoch, generation, tracked) in
            [(epoch, 3, true), (epoch, 2, false), (epoch + 1, 2, true)]
        {
            states[1] = State::Ready(vec![
                pose(10, epoch, 2, true),
                pose(30, after_epoch, generation, tracked),
            ]);
            inputs.update(&states);
            overlay.prepare(time, &mut inputs)?;
            assert!(overlay.sample.is_none());
        }
        Ok(())
    }

    #[test]
    fn projection_rejects_invalid_depth_and_clips_crossing_segments() {
        let projection = FieldProjection {
            field_to_camera: Isometry3::from_axis_angle(vector![0.0, -FRAC_PI_2, 0.0]),
            intrinsics: Intrinsic::default(),
        };
        assert!(projection.project(point![f32::NAN, 0.0, 0.0]).is_none());
        assert!(
            projection
                .segment(point![-2.0, 1.0], point![-1.0, 1.0])
                .is_none()
        );
        let segment = projection
            .segment(point![-1.0, 1.0], point![1.0, 1.0])
            .unwrap();
        assert!(
            segment
                .iter()
                .all(|p| p.inner.iter().all(|v| v.is_finite()))
        );
        assert_eq!(
            projection
                .segment(point![1.0, 1.0], point![-1.0, 1.0])
                .unwrap(),
            [segment[1], segment[0]]
        );
    }
}
