use std::{collections::HashMap, sync::Arc};

#[cfg(test)]
use std::sync::atomic::{AtomicUsize, Ordering};

use color_eyre::eyre::{Result, ensure, eyre};
use parking_lot::Mutex;
use tokio::{
    sync::{Notify, Semaphore, watch},
    task::spawn_blocking,
};
use tokio_util::{sync::CancellationToken, task::TaskTracker};
use uuid::Uuid;

use super::{MAX_CHUNK, Recording, SnapshotChunk};

type ChunkKey = (Uuid, u32);
type Decoded = std::result::Result<Arc<SnapshotChunk>, String>;

struct Entry {
    result: watch::Receiver<Option<Decoded>>,
    last_used: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recording::query::tests::fixture_records;
    use std::{
        future::{Future, poll_fn},
        task::Poll,
    };

    async fn poll_pending<T>(future: std::pin::Pin<&mut impl Future<Output = T>>) {
        let mut future = future;
        poll_fn(|cx| {
            assert!(future.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
    }

    #[tokio::test]
    async fn simultaneous_waiters_share_decode_after_initiator_is_dropped() {
        let file = fixture_records(true, false, &[(0, 10, 100, 100)]);
        let recording = Arc::new(Recording::open(file.path(), "/replay").unwrap());
        let cache = Arc::new(ChunkCache::new(1024 * 1024).unwrap());
        let gate = cache.workers.clone().acquire_many_owned(4).await.unwrap();
        let owner = Uuid::new_v4();
        let cancel = CancellationToken::new();
        let mut first = Box::pin(cache.get(owner, recording.clone(), 0, cancel.clone()));
        let mut second = Box::pin(cache.get(owner, recording.clone(), 0, cancel.clone()));
        poll_pending(first.as_mut()).await;
        poll_pending(second.as_mut()).await;
        assert_eq!(cache.state.lock().entries.len(), 1);
        drop(first);
        drop(gate);
        let chunk = second.await.unwrap();
        let again = cache.get(owner, recording, 0, cancel).await.unwrap();
        assert!(Arc::ptr_eq(&chunk, &again));
        assert_eq!(cache.decoded_chunks.load(Ordering::Relaxed), 1);
        cache.shutdown().await;
    }

    #[tokio::test]
    async fn failed_decode_wakes_all_waiters_and_can_be_retried() {
        let file = fixture_records(true, false, &[(0, 10, 100, 100)]);
        let recording = Arc::new(Recording::open(file.path(), "/replay").unwrap());
        let original = std::fs::read(file.path()).unwrap();
        file.as_file().set_len(0).unwrap();
        let cache = Arc::new(ChunkCache::new(1024 * 1024).unwrap());
        let owner = Uuid::new_v4();
        let cancel = CancellationToken::new();
        let mut first = Box::pin(cache.get(owner, recording.clone(), 0, cancel.clone()));
        let mut second = Box::pin(cache.get(owner, recording.clone(), 0, cancel.clone()));
        poll_pending(first.as_mut()).await;
        poll_pending(second.as_mut()).await;
        let (a, b) = tokio::join!(first, second);
        assert_eq!(a.err().unwrap().to_string(), b.err().unwrap().to_string());
        std::fs::write(file.path(), original).unwrap();
        assert!(cache.get(owner, recording, 0, cancel).await.is_ok());
        cache.shutdown().await;
    }

    #[tokio::test]
    async fn eviction_keeps_pinned_buffers_charged_until_released() {
        let file = fixture_records(true, true, &[(0, 10, 100, 100), (0, 20, 200, 200)]);
        let recording = Arc::new(Recording::open(file.path(), "/replay").unwrap());
        let sizes: Vec<_> = recording
            .summary
            .chunk_indexes
            .iter()
            .map(|chunk| chunk.uncompressed_size as usize)
            .collect();
        let capacity = *sizes.iter().max().unwrap();
        assert!(sizes.iter().sum::<usize>() > capacity);
        let cache = Arc::new(ChunkCache::new(capacity).unwrap());
        let owner = Uuid::new_v4();
        let cancel = CancellationToken::new();
        let first = cache
            .get(owner, recording.clone(), 0, cancel.clone())
            .await
            .unwrap();
        let mut second = Box::pin(cache.get(owner, recording, 1, cancel));
        poll_pending(second.as_mut()).await;
        assert_eq!(cache.bytes.available_permits(), capacity - sizes[0]);
        drop(first);
        let second = second.await.unwrap();
        assert_eq!(cache.bytes.available_permits(), capacity - sizes[1]);
        cache.remove_recording(owner);
        drop(second);
        assert_eq!(cache.bytes.available_permits(), capacity);
        cache.shutdown().await;
    }

    #[tokio::test]
    async fn loading_entries_make_progress_when_budget_only_fits_one_chunk() {
        let file = fixture_records(true, true, &[(0, 10, 100, 100), (0, 20, 200, 200)]);
        let recording = Arc::new(Recording::open(file.path(), "/replay").unwrap());
        let capacity = recording
            .summary
            .chunk_indexes
            .iter()
            .map(|chunk| chunk.uncompressed_size as usize)
            .max()
            .unwrap();
        let cache = Arc::new(ChunkCache::new(capacity).unwrap());
        let owner = Uuid::new_v4();
        let cancel = CancellationToken::new();
        let first = async {
            drop(
                cache
                    .get(owner, recording.clone(), 0, cancel.clone())
                    .await
                    .unwrap(),
            );
        };
        let second = async {
            drop(
                cache
                    .get(owner, recording.clone(), 1, cancel.clone())
                    .await
                    .unwrap(),
            );
        };
        tokio::time::timeout(std::time::Duration::from_secs(5), async {
            tokio::join!(first, second);
        })
        .await
        .unwrap();
        assert_eq!(cache.decoded_chunks.load(Ordering::Relaxed), 2);
        cache.shutdown().await;
        assert_eq!(cache.bytes.available_permits(), capacity);
    }

    #[tokio::test]
    async fn recording_cancellation_stops_queued_decodes_and_releases_budget() {
        let file = fixture_records(true, false, &[(0, 10, 100, 100)]);
        let recording = Arc::new(Recording::open(file.path(), "/replay").unwrap());
        let cache = Arc::new(ChunkCache::new(1024 * 1024).unwrap());
        let gate = cache.workers.clone().acquire_many_owned(4).await.unwrap();
        let cancel = CancellationToken::new();
        let mut waiting = Box::pin(cache.get(Uuid::new_v4(), recording, 0, cancel.clone()));
        poll_pending(waiting.as_mut()).await;
        cancel.cancel();
        assert!(waiting.await.is_err());
        tokio::time::timeout(std::time::Duration::from_secs(5), cache.shutdown())
            .await
            .unwrap();
        assert_eq!(cache.bytes.available_permits(), cache.capacity);
        assert!(cache.state.lock().entries.is_empty());
        drop(gate);
    }
}

#[derive(Default)]
struct State {
    entries: HashMap<ChunkKey, Entry>,
    clock: u64,
}

/// One shared decode per chunk; the byte reservation lives as long as its buffer.
pub struct ChunkCache {
    state: Mutex<State>,
    capacity: usize,
    bytes: Arc<Semaphore>,
    workers: Arc<Semaphore>,
    tasks: TaskTracker,
    changed: Notify,
    #[cfg(test)]
    pub(super) decoded_chunks: AtomicUsize,
}

impl ChunkCache {
    pub fn new(capacity: usize) -> Result<Self> {
        ensure!(
            capacity > 0 && capacity <= Semaphore::MAX_PERMITS,
            "invalid chunk cache budget"
        );
        Ok(Self {
            state: Mutex::new(State::default()),
            capacity,
            bytes: Arc::new(Semaphore::new(capacity)),
            workers: Arc::new(Semaphore::new(4)),
            tasks: TaskTracker::new(),
            changed: Notify::new(),
            #[cfg(test)]
            decoded_chunks: AtomicUsize::new(0),
        })
    }

    pub(super) async fn get(
        self: &Arc<Self>,
        owner: Uuid,
        recording: Arc<Recording>,
        chunk: u32,
        cancel: CancellationToken,
    ) -> Result<Arc<SnapshotChunk>> {
        ensure!(!cancel.is_cancelled(), "recording closed");
        let size = recording.summary.chunk_indexes[chunk as usize].uncompressed_size;
        ensure!(size <= MAX_CHUNK as u64, "chunk exceeds 64 MiB limit");
        ensure!(
            size <= self.capacity as u64,
            "chunk exceeds --chunk-cache-mib budget"
        );
        let key = (owner, chunk);
        let mut result = {
            let mut state = self.state.lock();
            state.clock = state.clock.saturating_add(1);
            let clock = state.clock;
            match state.entries.get_mut(&key) {
                Some(entry) => {
                    entry.last_used = clock;
                    entry.result.clone()
                }
                None => {
                    let (sender, result) = watch::channel(None);
                    state.entries.insert(
                        key,
                        Entry {
                            result: result.clone(),
                            last_used: clock,
                        },
                    );
                    let cache = Arc::clone(self);
                    let cancel = cancel.clone();
                    self.tasks.spawn(async move {
                        let decoded = cache
                            .load(recording, chunk, size, &cancel)
                            .await
                            .map(Arc::new)
                            .map_err(|error| format!("{error:#}"));
                        {
                            let mut state = cache.state.lock();
                            let failed = decoded.is_err();
                            sender.send_replace(Some(decoded));
                            // Existing waiters receive the error; later calls can retry.
                            if failed {
                                state.entries.remove(&key);
                            }
                        }
                        cache.changed.notify_waiters();
                    });
                    result
                }
            }
        };
        loop {
            if let Some(decoded) = result.borrow_and_update().clone() {
                return decoded.map_err(|error| eyre!(error));
            }
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(eyre!("recording closed")),
                changed = result.changed() => changed.map_err(|_| eyre!("chunk decoder stopped"))?,
            }
        }
    }

    async fn load(
        &self,
        recording: Arc<Recording>,
        chunk: u32,
        size: u64,
        cancel: &CancellationToken,
    ) -> Result<SnapshotChunk> {
        let size = u32::try_from(size)?;
        let reservation = loop {
            let changed = self.changed.notified();
            tokio::pin!(changed);
            changed.as_mut().enable();
            while self.bytes.available_permits() < size as usize {
                let evicted = {
                    let mut state = self.state.lock();
                    // shortcut: eviction scans entries; use an indexed LRU if tiny chunks make this costly.
                    let oldest = state
                        .entries
                        .iter()
                        .filter(|(_, entry)| entry.result.borrow().is_some())
                        .min_by_key(|(_, entry)| entry.last_used)
                        .map(|(key, _)| *key);
                    oldest.and_then(|key| state.entries.remove(&key))
                };
                let Some(evicted) = evicted else { break };
                // Free potentially large buffers outside the cache lock.
                drop(evicted);
            }
            tokio::select! {
                biased;
                _ = cancel.cancelled() => return Err(eyre!("recording closed")),
                reservation = Arc::clone(&self.bytes).acquire_many_owned(size) => break reservation?,
                _ = changed => {},
            }
        };
        let decoder = tokio::select! {
            biased;
            _ = cancel.cancelled() => return Err(eyre!("recording closed")),
            decoder = Arc::clone(&self.workers).acquire_owned() => decoder?,
        };
        let cancel = cancel.clone();
        let decoded = spawn_blocking(move || {
            let _decoder = decoder;
            recording.decode_chunk(chunk, &cancel, reservation)
        })
        .await??;
        #[cfg(test)]
        self.decoded_chunks.fetch_add(1, Ordering::Relaxed);
        Ok(decoded)
    }

    pub(super) fn remove_recording(&self, owner: Uuid) {
        let removed: Vec<_> = self
            .state
            .lock()
            .entries
            .extract_if(|(id, _), _| *id == owner)
            .map(|(_, entry)| entry)
            .collect();
        drop(removed);
    }

    pub async fn shutdown(&self) {
        self.tasks.close();
        self.tasks.wait().await;
        let entries = std::mem::take(&mut self.state.lock().entries);
        drop(entries);
    }

    pub(crate) async fn blocking<T: Send + 'static>(
        &self,
        work: impl FnOnce() -> T + Send + 'static,
    ) -> Result<T> {
        let permit = Arc::clone(&self.workers).acquire_owned().await?;
        Ok(spawn_blocking(move || {
            let _permit = permit;
            work()
        })
        .await?)
    }
}
