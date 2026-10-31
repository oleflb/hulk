use std::{
    collections::HashMap,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::Duration,
};

use bincode::{DefaultOptions, Options};
use ros_z::{node::Node, time::Time};
use serde::{Deserialize, Serialize};
use tokio::{
    sync::{Semaphore, watch},
    task::JoinHandle,
};
use zenoh::query::ConsolidationMode;

use super::{
    Backend, Cursor, CursorHandle, CursorPosition, CursorRange, PlanObservation, PlanSnapshot,
    RawRecord, TopicInfo, TopicList, TopicSnapshot,
    plan::{Evaluation, QueryPlan, Sample, State},
    plan_observation::PlanSender,
    query_error,
};
use crate::{Error, Result, TopicReference};

pub const MAX_RESPONSE_BYTES: u64 = 64 * 1024 * 1024;
static NEXT_OBSERVATION: AtomicU64 = AtomicU64::new(0);

pub fn query_key(namespace: &str) -> String {
    let namespace = namespace.trim_matches('/');
    if namespace.is_empty() {
        "hulk/mcap/query".into()
    } else {
        format!("hulk/mcap/{namespace}/query")
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordingInfo {
    pub instance: String,
    pub name: String,
    pub start: Time,
    pub end: Time,
    pub topics: Vec<TopicInfo>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Request {
    Catalog,
    Plan {
        instance: String,
        observation: String,
        revision: u64,
        cursor: Time,
        plan: QueryPlan,
    },
    Cancel {
        instance: String,
        observation: String,
        revision: u64,
    },
}

#[derive(Debug)]
pub enum Response {
    RecordingChanged,
    Catalog(RecordingInfo),
    Error(String),
    PlanData(Evaluation),
}

#[derive(Serialize, Deserialize)]
enum WireResponse {
    RecordingChanged,
    Catalog(RecordingInfo),
    Error(String),
    PlanData {
        records: Vec<Arc<RawRecord>>,
        evaluation: Evaluation<(usize, TopicInfo)>,
    },
}

pub fn encode(response: &Response) -> Result<Vec<u8>> {
    let response = match response {
        Response::RecordingChanged => WireResponse::RecordingChanged,
        Response::Catalog(info) => WireResponse::Catalog(info.clone()),
        Response::Error(error) => WireResponse::Error(error.clone()),
        Response::PlanData(evaluation) => {
            let mut records = Vec::new();
            let mut slots = HashMap::new();
            let evaluation = evaluation.clone().try_map_samples(|sample| {
                let slot = *slots.entry(Arc::as_ptr(&sample.record)).or_insert_with(|| {
                    records.push(sample.record);
                    records.len() - 1
                });
                Ok((slot, sample.topic))
            })?;
            WireResponse::PlanData {
                records,
                evaluation,
            }
        }
    };
    DefaultOptions::new()
        .with_limit(MAX_RESPONSE_BYTES)
        .serialize(&response)
        .map_err(query_error)
}

pub fn decode(bytes: &[u8]) -> Result<Response> {
    let response = DefaultOptions::new()
        .with_limit(MAX_RESPONSE_BYTES)
        .deserialize(bytes)
        .map_err(query_error)?;
    Ok(match response {
        WireResponse::RecordingChanged => Response::RecordingChanged,
        WireResponse::Catalog(info) => Response::Catalog(info),
        WireResponse::Error(error) => Response::Error(error),
        WireResponse::PlanData {
            records,
            evaluation,
        } => Response::PlanData(evaluation.try_map_samples(|(slot, topic)| {
            let record = records
                .get(slot)
                .ok_or_else(|| query_error("invalid plan record reference"))?;
            Ok(Sample {
                record: Arc::clone(record),
                topic,
            })
        })?),
    })
}

async fn request_catalog(node: &Node, key: &str) -> Result<RecordingInfo> {
    let payload = serde_json::to_vec(&Request::Catalog).map_err(query_error)?;
    let replies = node
        .session()
        .get(key)
        .payload(payload)
        .consolidation(ConsolidationMode::None)
        .timeout(Duration::from_secs(120))
        .await
        .map_err(query_error)?;
    let reply = replies.recv_async().await.map_err(query_error)?;
    let sample = reply
        .into_result()
        .map_err(|error| query_error(String::from_utf8_lossy(&error.payload().to_bytes())))?;
    match decode(&sample.payload().to_bytes())? {
        Response::Catalog(info) => Ok(info),
        Response::RecordingChanged => Err(Error::RecordingChanged),
        Response::Error(error) => Err(query_error(error)),
        _ => Err(query_error("expected recording catalog")),
    }
}

pub struct RemoteMcap {
    replaced: watch::Sender<bool>,
    catalog_check: JoinHandle<()>,
    bounds: watch::Sender<(Time, Time)>,
    namespace: String,
    node: Arc<Node>,
    key: String,
    info: Arc<RecordingInfo>,
    topics: TopicList,
    decoders: Arc<Semaphore>,
}

impl RemoteMcap {
    pub async fn connect(node: Arc<Node>, namespace: &str) -> Result<Self> {
        let key = query_key(namespace);
        let info = request_catalog(&node, &key).await?;
        if info.start > info.end {
            return Err(query_error("invalid recording bounds"));
        }
        for topic in &info.topics {
            TopicReference::new(&topic.name)?;
            let schema = topic
                .schema
                .as_ref()
                .ok_or_else(|| query_error("recording topic has no schema"))?;
            schema.validate().map_err(query_error)?;
            if ros_z_schema::compute_hash(schema.as_ref()).map_err(query_error)?
                != topic.type_info.hash
            {
                return Err(query_error("recording schema hash mismatch"));
            }
        }
        let (_, receiver) = watch::channel(Arc::new(TopicSnapshot {
            topics: info.topics.clone(),
            loading: false,
            error: None,
        }));
        let replaced = watch::channel(false).0;
        let catalog_check = {
            let (node, key, instance, replaced) = (
                Arc::clone(&node),
                key.clone(),
                info.instance.clone(),
                replaced.clone(),
            );
            tokio::spawn(async move {
                loop {
                    tokio::time::sleep(Duration::from_secs(1)).await;
                    if let Ok(Ok(info)) =
                        tokio::time::timeout(Duration::from_secs(3), request_catalog(&node, &key))
                            .await
                        && info.instance != instance
                    {
                        replaced.send_replace(true);
                        break;
                    }
                }
            })
        };
        Ok(Self {
            replaced,
            catalog_check,
            bounds: watch::channel((info.start, info.end)).0,
            namespace: namespace.into(),
            node,
            key,
            info: Arc::new(info),
            topics: TopicList(receiver),
            decoders: Arc::new(Semaphore::new(4)),
        })
    }

    pub fn recording(&self) -> &Arc<RecordingInfo> {
        &self.info
    }

    pub fn bounds(&self) -> (Time, Time) {
        *self.bounds.borrow()
    }

    pub fn needs_reconnect(&self) -> bool {
        *self.replaced.borrow()
    }
}

impl Drop for RemoteMcap {
    fn drop(&mut self) {
        self.catalog_check.abort();
    }
}

impl Backend for RemoteMcap {
    fn topics(&self) -> TopicList {
        self.topics.clone()
    }

    fn observe(&self, cursor: CursorHandle, mut plan: QueryPlan) -> Result<PlanObservation> {
        plan.resolve_topics(&self.namespace)?;
        let (observation, sender) = PlanObservation::channel(cursor.clone());
        let worker = RemoteObservation {
            observation: format!(
                "{}-{}",
                self.node.session().zid(),
                NEXT_OBSERVATION.fetch_add(1, Ordering::Relaxed)
            ),
            node: Arc::clone(&self.node),
            key: self.key.clone(),
            catalog: Arc::clone(&self.info),
            bounds: self.bounds.clone(),
            replaced: self.replaced.clone(),
            cursor,
            plan,
            sender,
            decoders: self.decoders.clone(),
        };
        tokio::spawn(worker.run());
        Ok(observation)
    }
}

struct RemoteObservation {
    observation: String,
    node: Arc<Node>,
    key: String,
    catalog: Arc<RecordingInfo>,
    bounds: watch::Sender<(Time, Time)>,
    replaced: watch::Sender<bool>,
    cursor: CursorHandle,
    plan: QueryPlan,
    sender: PlanSender,
    decoders: Arc<Semaphore>,
}

impl RemoteObservation {
    async fn run(self) {
        let mut positions = self.cursor.subscribe();
        let mut replacement = self.replaced.subscribe();
        loop {
            let position = *positions.borrow_and_update();
            if *replacement.borrow() {
                self.publish_failure(position, "recording changed; reconnect".into(), false);
                break;
            }
            let Cursor::At(time) = position.cursor else {
                self.cancel(position.revision).await;
                self.publish_failure(position, "recordings require Cursor::At".into(), false);
                tokio::select! {
                    _ = self.sender.closed() => break,
                    _ = positions.changed() => {},
                    _ = replacement.changed() => {},
                }
                continue;
            };
            let previous = self.sender.borrow().clone();
            if matches!(previous.evaluation.state, State::Ready(_))
                && previous.evaluation.settled()
                && previous.evaluation.valid_for.contains(time)
            {
                if previous.revision != position.revision {
                    self.sender.send_replace(Arc::new(PlanSnapshot {
                        revision: position.revision,
                        retrying: false,
                        evaluation: previous.evaluation.clone(),
                    }));
                }
                tokio::select! {
                    _ = self.sender.closed() => break,
                    _ = replacement.changed() => {},
                    changed = positions.changed() => if changed.is_err() { break; },
                }
                continue;
            }
            self.sender.send_replace(Arc::new(PlanSnapshot {
                revision: position.revision,
                retrying: false,
                evaluation: Arc::new(Evaluation::unresolved(time, State::Pending)),
            }));
            // A newer request supersedes this observation's work on the server.
            let result = tokio::select! {
                biased;
                _ = self.sender.closed() => break,
                _ = replacement.changed() => continue,
                changed = positions.changed() => {
                    if changed.is_err() { break; }
                    continue;
                },
                result = self.execute_plan(time, position.revision) => result,
            };
            if let Err(error) = &result
                && self.cursor.position().revision == position.revision
            {
                self.publish_failure(position, error.to_string(), !*self.replaced.borrow());
            }
            if self.cursor.position().revision != position.revision {
                continue;
            }
            tokio::select! {
                _ = self.sender.closed() => break,
                _ = replacement.changed() => {},
                changed = positions.changed() => if changed.is_err() { break; },
                _ = tokio::time::sleep(Duration::from_secs(1)), if result.is_err() => {},
            }
        }
        self.cancel(self.cursor.position().revision).await;
    }

    async fn cancel(&self, revision: u64) {
        // Zenoh query handles do not cancel server work when dropped.
        let command = Request::Cancel {
            instance: self.catalog.instance.clone(),
            observation: self.observation.clone(),
            revision,
        };
        if let Ok(payload) = serde_json::to_vec(&command) {
            let _ = self.node.session().get(&self.key).payload(payload).await;
        }
    }

    async fn execute_plan(&self, time: Time, revision: u64) -> Result<()> {
        let command = Request::Plan {
            instance: self.catalog.instance.clone(),
            observation: self.observation.clone(),
            revision,
            cursor: time,
            plan: self.plan.clone(),
        };
        let payload = serde_json::to_vec(&command).map_err(query_error)?;
        let replies = self
            .node
            .session()
            .get(&self.key)
            .payload(payload)
            .consolidation(ConsolidationMode::None)
            .timeout(Duration::from_secs(120))
            .await
            .map_err(query_error)?;
        let mut settled = false;
        while let Ok(reply) = replies.recv_async().await {
            let sample = reply.into_result().map_err(|error| {
                query_error(String::from_utf8_lossy(&error.payload().to_bytes()))
            })?;
            if self.cursor.position().revision != revision {
                return Ok(());
            }
            let permit = self
                .decoders
                .clone()
                .acquire_owned()
                .await
                .map_err(query_error)?;
            let plan = self.plan.clone();
            let catalog = self.catalog.clone();
            let response = tokio::task::spawn_blocking(move || {
                let _permit = permit;
                let mut response = decode(&sample.payload().to_bytes())?;
                if let Response::PlanData(result) = &mut response {
                    if result.cursor != time {
                        return Err(query_error("plan response cursor mismatch"));
                    }
                    plan.validate_result(result, &catalog.topics)?;
                }
                Ok(response)
            })
            .await
            .map_err(query_error)??;
            match response {
                Response::PlanData(result) => {
                    settled = result.settled();
                    if let Some((start, end)) = result.bounds {
                        self.bounds.send_modify(|bounds| {
                            bounds.0 = bounds.0.min(start);
                            bounds.1 = bounds.1.max(end);
                        });
                    }
                    if self.cursor.position().revision == revision {
                        self.sender.send_replace(Arc::new(PlanSnapshot {
                            revision,
                            retrying: false,
                            evaluation: Arc::new(result),
                        }));
                    }
                }
                Response::RecordingChanged => {
                    self.replaced.send_replace(true);
                    return Err(Error::RecordingChanged);
                }
                Response::Error(error) => return Err(query_error(error)),
                _ => return Err(query_error("expected plan response")),
            }
        }
        if settled {
            Ok(())
        } else {
            Err(query_error("plan response ended before completion"))
        }
    }

    fn publish_failure(&self, position: CursorPosition, error: String, retrying: bool) {
        self.sender.send_replace(Arc::new(PlanSnapshot {
            revision: position.revision,
            retrying,
            evaluation: Arc::new(Evaluation {
                valid_for: CursorRange {
                    start: Time::zero(),
                    end: None,
                },
                ..Evaluation::unresolved(
                    match position.cursor {
                        Cursor::At(time) => time,
                        Cursor::Live => Time::zero(),
                    },
                    State::Failed(error),
                )
            }),
        }));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::query::plan::{self, Frame, Value};
    use ros_z::{Message, context::ContextBuilder};

    #[test]
    fn shared_payloads_round_trip_once_and_invalid_references_are_rejected() {
        let sample = Sample {
            topic: TopicInfo {
                name: "/input".into(),
                type_info: u64::type_info(),
                schema: None,
                error: None,
            },
            record: Arc::new(RawRecord {
                id: 0,
                publication: None,
                time: Time::zero(),
                source_time: Time::zero(),
                log_time: None,
                data: vec![0; 1024 * 1024].into(),
            }),
        };
        let mut evaluation = Evaluation::unresolved(Time::zero(), State::Pending);
        evaluation.state = State::Ready(Value::Frames(
            (0..65)
                .map(|id| Frame {
                    primary: Sample {
                        record: Arc::new(RawRecord {
                            id,
                            data: Arc::from([]),
                            ..(*sample.record).clone()
                        }),
                        ..sample.clone()
                    },
                    inputs: vec![State::Ready(vec![sample.clone()])],
                })
                .collect(),
        ));
        let bytes = encode(&Response::PlanData(evaluation)).unwrap();
        assert!(bytes.len() < 2 * 1024 * 1024);
        let Response::PlanData(result) = decode(&bytes).unwrap() else {
            panic!("plan response")
        };
        let State::Ready(Value::Frames(frames)) = result.state else {
            panic!("frames")
        };
        let State::Ready(first) = &frames[0].inputs[0] else {
            panic!("input")
        };
        for frame in &frames {
            let State::Ready(input) = &frame.inputs[0] else {
                panic!("input")
            };
            assert!(Arc::ptr_eq(&first[0].record, &input[0].record));
            assert_eq!(input[0].record.data.len(), 1024 * 1024);
        }
        let mut evaluation = Evaluation::unresolved(Time::zero(), State::Pending);
        evaluation.state = State::Ready(Value::Records(vec![(0, sample.topic)]));
        let bytes = DefaultOptions::new()
            .serialize(&WireResponse::PlanData {
                records: Vec::new(),
                evaluation,
            })
            .unwrap();
        assert!(
            decode(&bytes)
                .unwrap_err()
                .to_string()
                .contains("invalid plan record reference")
        );
    }

    #[test]
    fn query_key_resolves_namespace() {
        assert_eq!(query_key("/"), "hulk/mcap/query");
        assert_eq!(query_key("/replay/"), "hulk/mcap/replay/query");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn latest_seek_does_not_wait_for_obsolete_reply_and_drop_cancels_observation() {
        let context = ContextBuilder::default()
            .disable_multicast_scouting()
            .with_json("connect/endpoints", serde_json::json!([]))
            .build()
            .await
            .unwrap();
        let node = Arc::new(context.create_node("latest_seek").build().await.unwrap());
        let key = query_key("/replay");
        let queryable = node.session().declare_queryable(key.clone()).await.unwrap();
        let (started, first_received) = tokio::sync::oneshot::channel();
        let (release, released) = tokio::sync::oneshot::channel();
        let (finished, old_finished) = tokio::sync::oneshot::channel();
        let (cancelled, cancellation) = tokio::sync::oneshot::channel();
        let server = tokio::spawn(async move {
            let mut started = Some(started);
            let mut released = Some(released);
            let mut finished = Some(finished);
            let mut old = None;
            let mut observation_id = None;
            while let Ok(incoming) = queryable.recv_async().await {
                let command: Request =
                    serde_json::from_slice(&incoming.payload().unwrap().to_bytes()).unwrap();
                let response = match command {
                    Request::Catalog => Response::Catalog(RecordingInfo {
                        instance: "test".into(),
                        name: "test.mcap".into(),
                        start: Time::zero(),
                        end: Time::from_nanos(100),
                        topics: Vec::new(),
                    }),
                    Request::Plan {
                        cursor,
                        plan,
                        observation,
                        ..
                    } => {
                        let evaluation = plan
                            .evaluate(cursor, &[], |_, _| panic!("no payloads"))
                            .unwrap();
                        if let Some(started) = started.take() {
                            observation_id = Some(observation);
                            old = Some((incoming, evaluation));
                            started.send(()).unwrap();
                            continue;
                        }
                        assert_eq!(Some(observation), observation_id);
                        incoming
                            .reply(&key, encode(&Response::PlanData(evaluation)).unwrap())
                            .await
                            .unwrap();
                        if let Some(released) = released.take() {
                            released.await.unwrap();
                            let (incoming, evaluation) = old.take().unwrap();
                            let _ = incoming
                                .reply(&key, encode(&Response::PlanData(evaluation)).unwrap())
                                .await;
                            finished.take().unwrap().send(()).unwrap();
                        }
                        continue;
                    }
                    Request::Cancel {
                        observation,
                        revision,
                        ..
                    } => {
                        assert_eq!(Some(observation), observation_id);
                        assert_eq!(revision, 1);
                        cancelled.send(()).unwrap();
                        break;
                    }
                };
                incoming
                    .reply(&key, encode(&response).unwrap())
                    .await
                    .unwrap();
            }
        });
        let remote = RemoteMcap::connect(node, "/replay").await.unwrap();
        let cursor = CursorHandle::new(Cursor::At(Time::from_nanos(10)));
        let observation = remote
            .observe(cursor.clone(), plan::cursor().plan())
            .unwrap();
        let mut updates = observation.subscribe_updates();
        tokio::time::timeout(Duration::from_secs(5), first_received)
            .await
            .unwrap()
            .unwrap();
        cursor.set(Cursor::At(Time::from_nanos(20)));
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                updates.borrow_and_update();
                let snapshot = observation.snapshot();
                if observation.is_current(&snapshot)
                    && matches!(snapshot.evaluation.state, State::Ready(_))
                {
                    assert_eq!(snapshot.evaluation.cursor, Time::from_nanos(20));
                    break;
                }
                updates.changed().await.unwrap();
            }
        })
        .await
        .unwrap();
        release.send(()).unwrap();
        old_finished.await.unwrap();
        assert_eq!(
            observation.snapshot().evaluation.cursor,
            Time::from_nanos(20)
        );
        drop(observation);
        tokio::time::timeout(Duration::from_secs(5), cancellation)
            .await
            .unwrap()
            .unwrap();
        server.await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn incomplete_plan_stream_reports_retry_until_recovery() {
        let context = ContextBuilder::default()
            .disable_multicast_scouting()
            .with_json("connect/endpoints", serde_json::json!([]))
            .build()
            .await
            .unwrap();
        let node = Arc::new(
            context
                .create_node("incomplete_plan")
                .build()
                .await
                .unwrap(),
        );
        let key = query_key("/replay");
        let queryable = node.session().declare_queryable(key.clone()).await.unwrap();
        let server = tokio::spawn(async move {
            let mut first_plan = true;
            while let Ok(incoming) = queryable.recv_async().await {
                let command: Request =
                    serde_json::from_slice(&incoming.payload().unwrap().to_bytes()).unwrap();
                let response = match command {
                    Request::Catalog => Response::Catalog(RecordingInfo {
                        instance: "test".into(),
                        name: "test.mcap".into(),
                        start: Time::zero(),
                        end: Time::from_nanos(100),
                        topics: Vec::new(),
                    }),
                    Request::Plan { cursor, plan, .. } => {
                        if std::mem::take(&mut first_plan) {
                            continue;
                        }
                        Response::PlanData(
                            plan.evaluate(cursor, &[], |_, _| panic!("no records expected"))
                                .unwrap(),
                        )
                    }
                    Request::Cancel { .. } => continue,
                };
                incoming
                    .reply(&key, encode(&response).unwrap())
                    .await
                    .unwrap();
            }
        });
        let remote = RemoteMcap::connect(node, "/replay").await.unwrap();
        let time = Time::from_nanos(10);
        let cursor = CursorHandle::new(Cursor::At(time));
        let observation = remote
            .observe(cursor.clone(), plan::cursor().plan())
            .unwrap();
        let mut updates = observation.subscribe_updates();
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let snapshot = observation.snapshot();
                if let State::Failed(error) = &snapshot.evaluation.state {
                    assert_eq!(error, "query: plan response ended before completion");
                    assert_eq!(snapshot.evaluation.cursor, time);
                    assert!(snapshot.retrying);
                    break;
                }
                updates.changed().await.unwrap();
            }
        })
        .await
        .unwrap();
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let snapshot = observation.snapshot();
                if matches!(snapshot.evaluation.state, State::Ready(_)) {
                    assert!(!snapshot.retrying);
                    break;
                }
                updates.changed().await.unwrap();
            }
        })
        .await
        .unwrap();
        cursor.set(Cursor::Live);
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let snapshot = observation.snapshot();
                if let State::Failed(error) = &snapshot.evaluation.state {
                    assert_eq!(error, "recordings require Cursor::At");
                    assert!(!snapshot.retrying);
                    break;
                }
                updates.changed().await.unwrap();
            }
        })
        .await
        .unwrap();
        server.abort();
    }
}
