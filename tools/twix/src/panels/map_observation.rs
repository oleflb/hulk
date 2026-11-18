use std::{sync::Arc, time::Duration};

use color_eyre::{Result, eyre::bail};
use ros_z::{Message, time::Time};
use ros_z_debug::{
    SampleRecord, TargetIdentity, TopicReference,
    query::{
        TimeProjection, TypedPlanObservation, decode_record,
        plan::{Expr, Frame, State, TopicSource},
    },
};

use crate::repaint::{ObservationContext, ObservationRepaint, RepaintOnUpdates};

pub(super) struct MapObservation {
    namespace: String,
    generation: u64,
    observation: TypedPlanObservation<Option<Frame>>,
    displayed_records: Option<Vec<(String, u64, Time)>>,
    _repaint: ObservationRepaint,
}

impl MapObservation {
    pub(super) fn new(
        context: &impl ObservationContext,
        plan: Expr<Option<Frame>>,
    ) -> Result<Self> {
        let connection = context.backend().query_connection();
        let _runtime = context.backend().runtime_handle().enter();
        let observation = plan.observe(connection.backend, connection.cursor)?;
        let repaint = observation.raw().repaint_on_updates(context);
        Ok(Self {
            namespace: context.backend().namespace(),
            generation: context.backend().presentation_identity().0,
            observation,
            displayed_records: None,
            _repaint: repaint,
        })
    }

    pub(super) fn is_bound(&self, context: &impl ObservationContext) -> bool {
        self.namespace == context.backend().namespace()
            && self.generation == context.backend().presentation_identity().0
    }

    pub(super) fn is_querying(&self) -> bool {
        crate::panel::query_pending(Some(self.observation.raw().status()))
    }

    pub(super) fn snapshot(&self) -> State<Option<Frame>> {
        let snapshot = self.observation.snapshot();
        if snapshot.raw.retrying || !snapshot.raw.evaluation.settled() {
            return State::Pending;
        }
        snapshot.state
    }

    pub(super) fn update<T: Default>(
        &mut self,
        displayed: &mut T,
        decode: impl FnOnce(&Frame) -> Result<T>,
    ) -> Result<()> {
        let state = self.snapshot();
        if let State::Ready(Some(frame)) = &state
            && frame
                .inputs
                .iter()
                .any(|input| !matches!(input, State::Ready(_)))
        {
            return commit(displayed, state, decode);
        }
        let records = match &state {
            State::Ready(Some(frame)) => std::iter::once(&frame.primary)
                .chain(
                    frame
                        .inputs
                        .iter()
                        .filter_map(|input| match input {
                            State::Ready(samples) => Some(samples.iter()),
                            _ => None,
                        })
                        .flatten(),
                )
                .map(|sample| {
                    (
                        sample.topic.name.clone(),
                        sample.record.id,
                        sample.record.time,
                    )
                })
                .collect(),
            State::Ready(None) => Vec::new(),
            _ => return commit(displayed, state, decode),
        };
        if self.displayed_records.as_ref() == Some(&records) {
            return Ok(());
        }
        commit(displayed, state, decode)?;
        self.displayed_records = Some(records);
        Ok(())
    }
}

pub(super) fn source<T: Message>(
    target: &TargetIdentity,
    topic: &str,
    projection: TimeProjection,
) -> Result<TopicSource> {
    let mut source = TopicSource::new(TopicReference::new(topic)?.resolve(target)?, projection);
    source.expected_type = Some(T::type_info());
    Ok(source)
}

pub(super) fn input<T: Message>(frame: &Frame, slot: usize) -> Result<Vec<Arc<SampleRecord<T>>>> {
    match frame.inputs.get(slot) {
        Some(State::Ready(samples)) => samples
            .iter()
            .map(|sample| decode_record(&sample.record, &sample.topic).map_err(Into::into))
            .collect(),
        Some(State::Failed(error)) => bail!("{error}"),
        _ => bail!("map input {slot} has not resolved"),
    }
}

// Commit the entire frame, not independently arriving pose/camera/ball components.
pub(super) fn commit<T: Default>(
    displayed: &mut T,
    state: State<Option<Frame>>,
    decode: impl FnOnce(&Frame) -> Result<T>,
) -> Result<()> {
    match state {
        State::Ready(Some(frame)) => *displayed = decode(&frame)?,
        State::Ready(None) => *displayed = T::default(),
        State::Failed(error) => bail!("{error}"),
        State::Pending => {}
    }
    Ok(())
}

pub(super) fn nearest<T>(
    records: impl IntoIterator<Item = T>,
    time: Time,
    tolerance: Duration,
    stamp: impl Fn(&T) -> Time,
) -> Option<T> {
    let record = records
        .into_iter()
        .min_by_key(|record| stamp(record).abs_diff(time))?;
    (stamp(&record).abs_diff(time) <= tolerance).then_some(record)
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use ros_z::message::WireEncoder;
    use ros_z_debug::query::{RawRecord, TopicInfo, plan::Sample};

    pub(in crate::panels) fn sample<T: Message>(
        topic: &str,
        id: u64,
        time: i64,
        value: &T,
    ) -> Sample {
        Sample {
            topic: TopicInfo {
                name: topic.into(),
                type_info: T::type_info(),
                schema: None,
                error: None,
            },
            record: Arc::new(RawRecord {
                publication: None,
                id,
                time: Time::from_nanos(time),
                source_time: Time::from_nanos(time),
                log_time: None,
                data: T::Codec::serialize(value).unwrap().into(),
            }),
        }
    }

    #[test]
    fn pending_and_failed_queries_preserve_the_display_but_resolved_empty_clears_it() {
        let mut displayed = 42;
        let decode = |_: &Frame| -> Result<i32> { panic!("no frame to decode") };
        commit(&mut displayed, State::Pending, decode).unwrap();
        assert_eq!(displayed, 42);
        assert!(commit(&mut displayed, State::Failed("failed".into()), decode).is_err());
        assert_eq!(displayed, 42);
        commit(&mut displayed, State::Ready(None), decode).unwrap();
        assert_eq!(displayed, 0);
    }

    #[test]
    fn decode_failure_does_not_commit_a_partial_frame() {
        let frame = Frame {
            primary: sample("pose", 1, 10, &1u64),
            inputs: vec![],
        };
        let mut displayed = 42;
        assert!(
            commit(&mut displayed, State::Ready(Some(frame.clone())), |_| {
                bail!("missing pose bracket")
            })
            .is_err()
        );
        assert_eq!(displayed, 42);
        commit(&mut displayed, State::Ready(Some(frame)), |_| Ok(7)).unwrap();
        assert_eq!(displayed, 7);
    }

    #[test]
    fn sources_resolve_namespaces_and_absolute_topics() {
        for namespace in ["42", "/42", "/42/"] {
            let target = TargetIdentity::new(namespace).unwrap();
            assert_eq!(
                source::<u64>(&target, "pose", TimeProjection::SourceTime)
                    .unwrap()
                    .topic,
                "/42/pose"
            );
            assert_eq!(
                source::<u64>(&target, "/43/pose", TimeProjection::SourceTime)
                    .unwrap()
                    .topic,
                "/43/pose"
            );
        }
    }
}
