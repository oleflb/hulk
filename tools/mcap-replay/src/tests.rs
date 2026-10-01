use std::{borrow::Cow, collections::BTreeMap, sync::Arc, time::Duration};

use ros_z::{Message as _, message::WireEncoder};
use ros_z_debug::{RetentionPolicy, TopicObservation, TopicObserver, TopicObserverOptions};

use super::*;

fn fixture() -> tempfile::NamedTempFile {
    fixture_chunks(&[&[(1, 30), (2, 20)], &[(1, 10), (1, 40)]], true)
}

fn fixture_chunks(chunks: &[&[(u16, u64)]], message_indexes: bool) -> tempfile::NamedTempFile {
    let file = tempfile::NamedTempFile::new().unwrap();
    let bundle = String::schema();
    let hash = ros_z_schema::compute_hash(&bundle).unwrap();
    let schema = Arc::new(mcap::Schema {
        id: 1,
        name: String::type_name(),
        encoding: "ros-z-schema-json".into(),
        data: Cow::Owned(serde_json::to_vec(&bundle).unwrap()),
    });
    let channel = |id, topic: &str| {
        Arc::new(mcap::Channel {
            id,
            topic: topic.into(),
            schema: Some(schema.clone()),
            message_encoding: "ros-z-cdr".into(),
            metadata: BTreeMap::from([
                ("ros_z.type_name".into(), String::type_name()),
                ("ros_z.schema_hash".into(), hash.to_hash_string()),
            ]),
        })
    };
    let channels = [channel(1, "a"), channel(2, "b")];
    let mut writer = mcap::WriteOptions::new()
        .chunk_size(None)
        .emit_message_indexes(message_indexes)
        .create(file.reopen().unwrap())
        .unwrap();
    let mut sequence = 0;
    for chunk in chunks {
        for &(id, time) in *chunk {
            writer
                .write(&mcap::Message {
                    channel: channels[usize::from(id - 1)].clone(),
                    sequence,
                    log_time: time,
                    publish_time: time - 1,
                    data: Cow::Owned(
                        <String as ros_z::Message>::Codec::serialize(&time.to_string()).unwrap(),
                    ),
                })
                .unwrap();
            sequence += 1;
        }
        writer.flush().unwrap();
    }
    writer.finish().unwrap();
    file
}

#[test]
fn cached_snapshots_match_indexed_reader_with_ties_and_missing_message_indexes() {
    use mcap::sans_io::indexed_reader::{IndexedReader, IndexedReaderOptions, ReadOrder};
    let chunks: &[&[(u16, u64)]] = &[
        &[(1, 30), (2, 20), (1, 10)],
        &[(1, 5), (1, 30), (1, 30), (2, 50)],
        &[(1, 15), (1, 10)],
        &[(2, 20), (1, 100)],
    ];
    for indexes in [true, false] {
        let file = fixture_chunks(chunks, indexes);
        let summary = mcap::Summary::read(&std::fs::read(file.path()).unwrap())
            .unwrap()
            .unwrap();
        let mut recording = Recording::open(file.path(), "/replay").unwrap();
        for position in (0..=105).chain((0..=105).rev()) {
            let snapshot = recording.snapshot(position).unwrap();
            for (id, topic) in [(1, "a"), (2, "b")] {
                let mut reader = IndexedReader::new_with_options(
                    &summary,
                    IndexedReaderOptions::default()
                        .include_topics([topic])
                        .with_order(ReadOrder::ReverseLogTime)
                        .log_time_before(position + 1),
                )
                .unwrap();
                let expected = recording.next(&mut reader).unwrap();
                let actual = snapshot
                    .iter()
                    .find(|message| message.header.channel_id == id);
                assert_eq!(
                    actual.map(|m| (m.header.log_time, m.header.sequence)),
                    expected
                        .as_ref()
                        .map(|m| (m.header.log_time, m.header.sequence)),
                    "position {position}, channel {id}"
                );
            }
        }
        assert_eq!(
            recording.decoded_chunks,
            chunks.len(),
            "all seeks reuse decoded chunks"
        );
    }
}

#[test]
fn indexed_snapshots_are_inclusive_and_playback_is_ordered() {
    let file = fixture();
    let mut recording = Recording::open(file.path(), "/replay").unwrap();
    assert_eq!((recording.start, recording.end), (10, 40));
    assert!(recording.snapshot(9).unwrap().is_empty());
    let times = |messages: Vec<Message>| {
        messages
            .into_iter()
            .map(|m| m.header.log_time)
            .collect::<Vec<_>>()
    };
    assert_eq!(times(recording.snapshot(10).unwrap()), [10]);
    assert_eq!(times(recording.snapshot(30).unwrap()), [20, 30]);
    assert_eq!(times(recording.snapshot(40).unwrap()), [20, 40]);
    let decoded = recording.decoded_chunks;
    for position in [10, 15, 20, 25, 30, 35, 40, 10] {
        recording.snapshot(position).unwrap();
    }
    assert_eq!(
        recording.decoded_chunks, decoded,
        "scrubbing cached chunks must not decompress again"
    );
    assert_eq!(times(recording.snapshot(10).unwrap()), [10]);
    let mut reader = recording.forward(10).unwrap();
    let mut messages = Vec::new();
    while let Some(message) = recording.next(&mut reader).unwrap() {
        messages.push(message);
    }
    assert_eq!(times(messages), [20, 30, 40]);
}

async fn wait_value(observation: &TopicObservation<String>, expected: &str) {
    let mut updates = observation.subscribe_updates().unwrap();
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if observation
                .latest()
                .is_some_and(|record| record.value == expected)
            {
                return;
            }
            updates.recv().await.unwrap();
        }
    })
    .await
    .unwrap_or_else(|_| panic!("waiting for {expected}: {:?}", observation.status()));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn paused_snapshots_rewind_and_reject_other_generations() {
    let file = fixture();
    let mut recording = Recording::open(file.path(), "/replay").unwrap();
    let context = ContextBuilder::default()
        .disable_multicast_scouting()
        .with_json("connect/endpoints", serde_json::json!([]))
        .build()
        .await
        .unwrap();
    let node = Arc::new(
        context
            .create_node("replay_test")
            .with_namespace("/replay")
            .build()
            .await
            .unwrap(),
    );
    let observer = TopicObserver::new(
        node.clone(),
        TopicObserverOptions::with_namespace("/replay").unwrap(),
    );
    let mut status = ReplayStatus {
        instance: "test".into(),
        recording: "test".into(),
        generation: 0,
        start: 10,
        end: 40,
        position: 40,
        playing: false,
        sources: Default::default(),
    };
    let old = seek(&node, &mut recording, 40).await.unwrap();
    update_sources(&mut status, &old);
    observer.set_replay_sources(Some(status.sources.clone()));
    let observation = observer
        .observe_typed::<String>("a")
        .unwrap()
        .retention(RetentionPolicy::time_window(Duration::from_nanos(5)).unwrap())
        .spawn();
    wait_value(&observation, "40").await;
    assert_eq!(
        observation.latest().unwrap().source_time,
        Time::from_nanos(39)
    );

    // New data precedes the manifest; old data follows it.
    let new = seek(&node, &mut recording, 10).await.unwrap();
    update_sources(&mut status, &new);
    observer.set_replay_sources(Some(status.sources.clone()));
    assert!(observation.latest().is_none());
    assert!(observation.get_all().is_empty());
    wait_value(&observation, "10").await;
    for message in recording.snapshot(40).unwrap() {
        publish(&old, message).await.unwrap();
    }
    let late = observer.observe_typed::<String>("a").unwrap().spawn();
    wait_value(&late, "10").await;
    assert_eq!(observation.latest().unwrap().value, "10");
    assert_eq!(
        observation.latest().unwrap().source_time,
        Time::from_nanos(9)
    );
    assert_eq!(observation.get_all().len(), 1);
    let mut raw = node
        .subscriber::<String>("a")
        .publisher(status.sources["/replay/a"].into())
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .raw()
        .build()
        .await
        .unwrap();
    let sample = tokio::time::timeout(Duration::from_secs(3), raw.recv())
        .await
        .unwrap()
        .unwrap();
    assert_eq!(
        sample.payload().to_bytes().as_ref(),
        <String as ros_z::Message>::Codec::serialize(&"10".to_owned()).unwrap()
    );

    let dynamic = observer.observe_dynamic("a").unwrap().spawn();
    let mut updates = dynamic.subscribe_updates().unwrap();
    tokio::time::timeout(Duration::from_secs(5), async {
        while dynamic.latest_json().is_none() {
            updates.recv().await.unwrap();
        }
    })
    .await
    .unwrap();
    assert_eq!(dynamic.latest_json().unwrap(), serde_json::json!("10"));

    // No sample before the sparse topic's first message; no old value survives.
    let sparse = observer.observe_typed::<String>("b").unwrap().spawn();
    assert!(sparse.latest().is_none());
    observer.set_replay_sources(Some(Default::default()));
    assert!(observation.latest().is_none());
    assert!(dynamic.latest_json().is_none());
}
