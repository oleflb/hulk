use std::{collections::HashMap, sync::Arc};

use color_eyre::eyre::{OptionExt, Result, ensure};
use parking_lot::Mutex;
use ros_z::time::Time;
use ros_z_debug::query::{
    Coverage, MAX_PAYLOAD_BYTES, RawRecord, SourceCoverage, TopicInfo,
    plan::{Evaluation, LoadedRecord, QueryPlan, SourceState, SourceView, State, TopicSource},
    remote::RecordingInfo,
};
use tokio::sync::{OwnedMutexGuard, watch};
use tokio_util::{sync::CancellationToken, task::TaskTracker};
use uuid::Uuid;

use super::{ChunkCache, Recording, SnapshotChunk};
pub use index::IndexCache;
use index::{INDEX_WORKERS, Index, IndexEntry, IndexKey};

mod index;
#[cfg(test)]
pub(super) mod tests;

/// Coordinates progressive queries; each evaluation pins its acquired indexes.
pub struct QueryRecording {
    cancel: CancellationToken,
    recording: Arc<Recording>,
    publication_publisher: [u8; 16],
    cache: Arc<IndexCache>,
    pub(crate) chunks: Arc<ChunkCache>,
    builds: Mutex<HashMap<IndexKey, watch::Receiver<Option<IndexResult>>>>,
    tasks: TaskTracker,
    owner: Uuid,
    index_workers: usize,
    pub info: RecordingInfo,
}

type IndexResult = std::result::Result<Arc<Index>, String>;

pub struct PendingQuery {
    plan: QueryPlan,
    cursor: Time,
    sources: Vec<TopicSource>,
    keys: Vec<Option<IndexKey>>,
    pinned: Vec<Option<Arc<Index>>>,
    failures: Vec<Option<String>>,
}

impl QueryRecording {
    pub(crate) fn cancel_token(&self) -> CancellationToken {
        self.cancel.clone()
    }

    pub fn new(
        recording: Recording,
        name: String,
        cancel: CancellationToken,
        cache: Arc<IndexCache>,
        chunks: Arc<ChunkCache>,
    ) -> Result<Self> {
        let instance = Uuid::new_v4();
        let info = RecordingInfo {
            instance: instance.to_string(),
            name,
            // Summary log-time bounds require no scan; query indexes refine them.
            start: Time::from_nanos(recording.start.try_into()?),
            end: Time::from_nanos(recording.end.try_into()?),
            topics: recording
                .channels
                .values()
                .map(|channel| TopicInfo {
                    name: channel.topic.clone(),
                    type_info: channel.type_info.clone(),
                    schema: Some(Arc::clone(&channel.schema)),
                    error: None,
                })
                .collect(),
        };
        Ok(Self {
            cancel,
            recording: Arc::new(recording),
            publication_publisher: *instance.as_bytes(),
            cache,
            chunks,
            builds: Mutex::new(HashMap::new()),
            tasks: TaskTracker::new(),
            owner: instance,
            index_workers: INDEX_WORKERS,
            info,
        })
    }

    fn channel(&self, topic: &str) -> Result<u16> {
        self.recording
            .channels
            .iter()
            .find(|(_, channel)| channel.topic == topic)
            .map(|(id, _)| *id)
            .ok_or_eyre(format!("recording has no topic {topic}"))
    }

    fn source_view(
        &self,
        key: Option<&IndexKey>,
        pinned: &mut Option<Arc<Index>>,
        failure: Option<&String>,
    ) -> SourceState {
        let Some(key) = key else {
            return SourceState::Absent;
        };
        if let Some(error) = failure {
            return SourceState::Failed(error.clone());
        }
        if pinned.is_none() {
            *pinned = self.cache.get(&self.owner, key);
        }
        let Some(index) = pinned else {
            return SourceState::Pending;
        };
        let channel = &self.recording.channels[&key.0];
        SourceState::Ready(SourceView {
            topic: TopicInfo {
                name: channel.topic.clone(),
                type_info: channel.type_info.clone(),
                schema: None,
                error: None,
            },
            index: index.clone(),
            coverage: Coverage {
                source: SourceCoverage::Complete,
                truncated: false,
            },
            diagnostics: index.diagnostics.clone(),
        })
    }

    fn record(
        &self,
        channel: u16,
        entry: IndexEntry,
        chunk: &SnapshotChunk,
    ) -> Result<Arc<RawRecord>> {
        let (header, data) = chunk.message(entry.record_offset)?;
        ensure!(
            header.channel_id == channel,
            "indexed message channel mismatch"
        );
        let id = (u64::from(entry.chunk_id) << 32) | u64::from(entry.record_offset);
        Ok(Arc::new(RawRecord {
            // Reopening a file changes its publisher; the cast preserves all location bits.
            publication: Some((self.publication_publisher, id as i64)),
            id,
            time: entry.time(),
            source_time: Time::from_nanos(header.publish_time.try_into()?),
            log_time: Some(Time::from_nanos(header.log_time.try_into()?)),
            data: data.as_ref().into(),
        }))
    }

    async fn load_records(
        self: &Arc<Self>,
        keys: &[Option<IndexKey>],
        pinned: &[Option<Arc<Index>>],
        locations: &[(usize, usize)],
    ) -> Result<Vec<LoadedRecord>> {
        let mut ordered: Vec<_> = locations
            .iter()
            .enumerate()
            .map(|(output, &(source, slot))| {
                let index = pinned[source]
                    .as_ref()
                    .ok_or_eyre("selected source is not pinned")?;
                Ok((index.entries[slot].chunk_id, source, slot, output))
            })
            .collect::<Result<_>>()?;
        ordered.sort_unstable();
        let mut results = vec![None; locations.len()];
        let mut bytes = 0;
        for group in ordered.chunk_by(|a, b| a.0 == b.0) {
            if bytes >= MAX_PAYLOAD_BYTES as usize {
                for &(_, _, _, output) in group {
                    results[output] = Some(Err("plan payload exceeds 32 MiB".into()));
                }
                continue;
            }
            let chunk = self
                .chunks
                .get(
                    self.owner,
                    self.recording.clone(),
                    group[0].0,
                    self.cancel.clone(),
                )
                .await;
            let locations: Vec<_> = group
                .iter()
                .map(|&(_, source, slot, output)| {
                    let key = keys[source]
                        .as_ref()
                        .ok_or_eyre("selected source has no index key")?;
                    let index = pinned[source]
                        .as_ref()
                        .ok_or_eyre("selected source is not pinned")?;
                    Ok((key.0, index.entries[slot], output))
                })
                .collect::<Result<_>>()?;
            let recording = Arc::clone(self);
            let loaded = self
                .chunks
                .blocking(move || recording.copy_records(chunk, locations, bytes))
                .await;
            match loaded {
                Ok((total, loaded)) => {
                    bytes = total;
                    for (output, record) in loaded {
                        results[output] = Some(record);
                    }
                }
                Err(error) => {
                    for &(_, _, _, output) in group {
                        results[output] = Some(Err(format!("{error:#}")));
                    }
                }
            }
        }
        results
            .into_iter()
            .map(|record| record.ok_or_eyre("selected payload location was not loaded"))
            .collect()
    }

    fn copy_records(
        &self,
        chunk: Result<Arc<SnapshotChunk>>,
        locations: Vec<(u16, IndexEntry, usize)>,
        mut bytes: usize,
    ) -> (usize, Vec<(usize, LoadedRecord)>) {
        let mut loaded = Vec::with_capacity(locations.len());
        for (channel, entry, output) in locations {
            let record = if bytes >= MAX_PAYLOAD_BYTES as usize {
                Err("plan payload exceeds 32 MiB".into())
            } else {
                match &chunk {
                    Ok(chunk) => self
                        .record(channel, entry, chunk)
                        .map_err(|error| format!("{error:#}")),
                    Err(error) => Err(format!("{error:#}")),
                }
            };
            if let Err(error) = &record {
                tracing::warn!(recording = %self.info.name, %error, "reading query payload failed");
            }
            if let Ok(record) = &record {
                bytes += record.data.len();
            }
            loaded.push((
                output,
                if bytes > MAX_PAYLOAD_BYTES as usize {
                    Err("plan payload exceeds 32 MiB".into())
                } else {
                    record
                },
            ));
        }
        (bytes, loaded)
    }

    pub async fn acquire_build(&self) -> OwnedMutexGuard<()> {
        self.cache.acquire_build().await
    }

    async fn index(self: &Arc<Self>, key: IndexKey) -> IndexResult {
        if let Some(index) = self.cache.get(&self.owner, &key) {
            return Ok(index);
        }
        let mut result = {
            let mut builds = self.builds.lock();
            builds
                .entry(key.clone())
                .or_insert_with(|| {
                    let (sender, result) = watch::channel(None);
                    let recording = Arc::clone(self);
                    self.tasks.spawn(async move {
                        let build = recording.build_index(key.clone()).await;
                        sender.send_replace(Some(build));
                        recording.builds.lock().remove(&key);
                    });
                    result
                })
                .clone()
        };
        loop {
            if let Some(index) = result.borrow_and_update().clone() {
                return index;
            }
            if result.changed().await.is_err() {
                return Err("index builder stopped".into());
            }
        }
    }

    async fn build_index(self: &Arc<Self>, key: IndexKey) -> IndexResult {
        let guard = tokio::select! {
            biased;
            _ = self.cancel.cancelled() => return Err("indexing cancelled".into()),
            guard = self.acquire_build() => guard,
        };
        let recording = Arc::clone(self);
        tokio::task::spawn_blocking(move || {
            recording.cache.index(
                &recording.owner,
                &recording.recording,
                &recording.cancel,
                recording.index_workers,
                &key,
                guard,
            )
        })
        .await
        .map_err(|error| error.to_string())?
        .map_err(|error| format!("{error:#}"))
    }

    pub(crate) async fn shutdown(&self) {
        self.cancel.cancel();
        self.tasks.close();
        self.tasks.wait().await;
        self.chunks.remove_recording(self.owner);
    }
}

impl PendingQuery {
    pub fn new(recording: &QueryRecording, plan: QueryPlan, cursor: Time) -> Result<Self> {
        let sources = plan.sources()?;
        let keys: Vec<_> = sources
            .iter()
            .map(|source| {
                recording
                    .channel(&source.topic)
                    .ok()
                    .map(|channel| (channel, source.projection.clone()))
            })
            .collect();
        Ok(Self {
            pinned: vec![None; sources.len()],
            failures: vec![None; sources.len()],
            plan,
            cursor,
            sources,
            keys,
        })
    }

    /// Evaluate cached inputs before asking the async driver for the build lock.
    pub async fn advance(
        &mut self,
        recording: &Arc<QueryRecording>,
    ) -> Result<(Evaluation, Option<usize>)> {
        ensure!(!recording.cancel.is_cancelled(), "query cancelled");
        let views: Vec<_> = self
            .keys
            .iter()
            .zip(&mut self.pinned)
            .zip(&self.failures)
            .map(|((key, pinned), failure)| {
                recording.source_view(key.as_ref(), pinned, failure.as_ref())
            })
            .collect();
        let plan = self.plan.clone();
        let cursor = self.cursor;
        // Tuple drop order releases index views before recording cleanup, even on panic.
        let input = (views, recording.clone());
        let start = std::time::Instant::now();
        let selected = recording
            .chunks
            .blocking(move || {
                let selected = plan.select(cursor, &input.0);
                drop(input);
                selected
            })
            .await??;
        let selection_us = start.elapsed().as_micros();
        let locations = selected.locations().len();
        let records = recording
            .load_records(&self.keys, &self.pinned, selected.locations())
            .await?;
        let payload_us = start.elapsed().as_micros() - selection_us;
        let result = recording
            .chunks
            .blocking(move || selected.assemble(records))
            .await??;
        tracing::debug!(
            selection_us,
            payload_us,
            locations,
            elapsed_us = start.elapsed().as_micros(),
            "evaluated replay plan"
        );
        let pending = result
            .dependencies
            .iter()
            .find(|dependency| dependency.resolution == State::Pending)
            .and_then(|dependency| {
                self.sources
                    .iter()
                    .position(|source| source == &dependency.source)
            });
        for (source, pin) in self.sources.iter().zip(&mut self.pinned) {
            if !result
                .dependencies
                .iter()
                .any(|dependency| &dependency.source == source)
            {
                *pin = None;
            }
        }
        Ok((result, pending))
    }

    pub async fn build_async(
        &mut self,
        recording: &Arc<QueryRecording>,
        source: usize,
    ) -> Result<()> {
        let key = self
            .keys
            .get(source)
            .and_then(Option::as_ref)
            .ok_or_eyre("missing pending source")?;
        match recording.index(key.clone()).await {
            Ok(index) => self.pinned[source] = Some(index),
            Err(error) => {
                tracing::warn!(recording = %recording.info.name, %error, "building query index failed");
                self.failures[source] = Some(error);
            }
        }
        Ok(())
    }
}

impl Drop for QueryRecording {
    fn drop(&mut self) {
        self.cache.remove_recording(self.owner);
        self.chunks.remove_recording(self.owner);
    }
}
