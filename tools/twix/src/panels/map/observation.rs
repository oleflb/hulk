use std::time::Duration;

use color_eyre::Result;
use coordinate_systems::{Field, Ground, Robot};
use linear_algebra::{Isometry2, Isometry3};
use ros_z::qos::{QosDurability, QosProfile};
use ros_z_debug::{
    ObservationPolicy, TargetIdentity,
    query::{
        TimeEncoding, TimeProjection,
        plan::{self, Frame, InputSpec, Selection, Source},
    },
};
use types::{
    field_dimensions::FieldDimensions,
    localization::{LocalizationEstimate, LocalizationStatus},
    time_wrapper::TimeWrapper,
};

use crate::{
    panels::map_observation::{MapObservation, input, nearest, source},
    repaint::ObservationContext,
};

#[derive(Default)]
pub(super) struct MapData {
    pub(super) field_dimensions: Option<FieldDimensions>,
    pub(super) ground_to_field: Option<Isometry2<Ground, Field>>,
}

impl MapData {
    pub(super) fn observe(context: &impl ObservationContext) -> Result<MapObservation> {
        let target = TargetIdentity::new(context.backend().namespace())?;
        let payload_time = TimeProjection::field(".time", TimeEncoding::RosZTime)?;
        let mut dimensions =
            source::<FieldDimensions>(&target, "field_dimensions", TimeProjection::SourceTime)?;
        let mut status =
            source::<LocalizationStatus>(&target, "localization/status", payload_time.clone())?;
        let latched = ObservationPolicy::default().with_subscriber_qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        });
        dimensions.live_policy = latched;
        status.live_policy = latched;
        let estimate =
            source::<LocalizationEstimate>(&target, "localization/estimate", payload_time.clone())?;
        let legacy = source::<Isometry2<Ground, Field>>(
            &target,
            "ground_to_field",
            TimeProjection::SourceTime,
        )?;
        let primary = Source::from(legacy.clone())
            .latest(plan::cursor())
            .or_else(Source::from(estimate.clone()).latest(plan::cursor()))
            .or_else(Source::from(dimensions.clone()).latest(plan::cursor()));
        let inputs = vec![
            InputSpec {
                source: dimensions.into(),
                selection: Selection::latest(),
            },
            InputSpec {
                source: status.into(),
                selection: Selection::latest(),
            },
            InputSpec {
                source: estimate.into(),
                selection: Selection::latest(),
            },
            InputSpec {
                source: source::<TimeWrapper<Option<Isometry3<Ground, Robot>>>>(
                    &target,
                    "ground_to_robot",
                    payload_time,
                )?
                .into(),
                selection: Selection::Bracket {
                    max_gap: Duration::from_millis(200),
                },
            },
            InputSpec {
                source: legacy.into(),
                selection: Selection::latest(),
            },
        ];
        MapObservation::new(context, primary.attach(inputs))
    }

    pub(super) fn from_frame(frame: &Frame) -> Result<Self> {
        let dimensions = input::<FieldDimensions>(frame, 0)?;
        let legacy = input::<Isometry2<Ground, Field>>(frame, 4)?;
        let ground_to_field = match legacy.last() {
            Some(record) => Some(record.value),
            None => {
                let status = input::<LocalizationStatus>(frame, 1)?;
                let estimate = input::<LocalizationEstimate>(frame, 2)?;
                let ground = input::<TimeWrapper<Option<Isometry3<Ground, Robot>>>>(frame, 3)?;
                estimate
                    .last()
                    .zip(status.last())
                    .and_then(|(estimate, status)| {
                        let ground = nearest(
                            ground,
                            estimate.value.time,
                            Duration::from_millis(100),
                            |record| record.value.time,
                        )?;
                        super::ground_to_field_at(&estimate.value, &status.value, &ground.value)
                    })
            }
        };
        Ok(Self {
            field_dimensions: dimensions.last().map(|record| record.value),
            ground_to_field,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::panels::map_observation::{commit, tests::sample};
    use ros_z_debug::query::plan::State;

    #[test]
    fn query_keeps_old_pose_then_commits_missing_pose_without_an_identity_fallback() {
        let dimensions = sample("/dimensions", 1, 0, &FieldDimensions::SPL_2025);
        let mut frame = Frame {
            primary: dimensions.clone(),
            inputs: vec![
                State::Ready(vec![dimensions]),
                State::Ready(vec![]),
                State::Ready(vec![]),
                State::Ready(vec![]),
                State::Ready(vec![sample(
                    "/pose",
                    2,
                    10,
                    &Isometry2::<Ground, Field>::from_parts(linear_algebra::vector![2.0, 3.0], 0.0),
                )]),
            ],
        };
        let mut data = MapData::from_frame(&frame).unwrap();
        commit(&mut data, State::Pending, MapData::from_frame).unwrap();
        assert_eq!(data.ground_to_field.unwrap().translation().x(), 2.0);
        frame.inputs[4] = State::Pending;
        assert!(
            commit(
                &mut data,
                State::Ready(Some(frame.clone())),
                MapData::from_frame
            )
            .is_err()
        );
        assert_eq!(data.ground_to_field.unwrap().translation().x(), 2.0);
        frame.inputs[4] = State::Ready(vec![]);
        commit(&mut data, State::Ready(Some(frame)), MapData::from_frame).unwrap();
        assert!(data.ground_to_field.is_none());
        assert!(data.field_dimensions.is_some());
    }
}
