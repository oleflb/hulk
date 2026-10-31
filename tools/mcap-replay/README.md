# MCAP replay for Twix

Open an indexed ROS-Z recording:

```sh
cargo run -p mcap-replay -- recording.mcap
```

The server starts a Zenoh router and prints a Twix connection command:

```sh
./twix /replay --router tcp/127.0.0.1:7447
```

The embedded router allows up to 60 seconds of transmit congestion before closing
a connection, so image replies can finish over slow or lossy links. With
`--router`, configure the external router's
`transport/link/tx/queue/congestion_control/block/wait_before_close` separately
(microseconds). This prevents premature disconnects; it does not improve link throughput.

Recordings need a summary and chunk indexes. Recover/reindex incomplete recordings first.

## Watch a directory

```sh
cargo run -p mcap-replay -- /data/recordings --index-cache-mib 1024
```

Existing and newly added `.mcap` files are served recursively using `notify`.
Changes reload recordings; deletions withdraw them.
Symlinks are not followed.
Namespaces follow relative paths: `game1/robot2.mcap` becomes `/replay/game1/robot2`.
`--namespace` changes the prefix, which must be `/replay` or start with `/replay/`.
Letters, digits, underscores, hyphens, and dots are preserved; other filename bytes (including `~`) are escaped as `~xx` to produce valid, collision-free namespaces.
Each recording prints its Twix command.

Unreadable or incomplete files are logged and retried on subsequent filesystem changes without stopping other recordings.
Duplicate open errors are suppressed.
Write recordings to a temporary name and rename them to `.mcap` when complete to avoid exposing partially written files.

The index budget is shared across recordings, including active and building timestamp indexes.
Idle indexes are evicted globally in least-recently-used order; queries that cannot fit return errors.
Index builds are serialized across recordings, with parallel scanning within each build.
Queries share in-progress index builds and await them asynchronously. Warm queries can run while other indexes are built.
Running builds retain their owned mutex guard until they finish and check the same cancellation token while scanning.
This is not a total memory limit: summaries, schemas, selected payloads, replies, and decoder scratch buffers use additional memory.
Dropping a recording queues index removal to one Tokio blocking task, without taking the cache lock.
Scanning and sorting run outside that lock, so cleanup and cached lookups can proceed during builds.
Removed allocations remain charged until freed.
Shutdown signals all recordings before waiting for them, then drains the cleanup queue and awaits the cleanup task.

Decoded chunks have a separate global byte budget, shared across recordings. Concurrent
requests for a chunk share one decode and wait asynchronously. Least-recently-used
completed chunks are evicted; buffers still held by queries remain charged until freed.
In-flight decodes reserve their uncompressed size before allocating. A chunk larger
than the configured budget returns an error; increase `--chunk-cache-mib` to fit it.
Timestamp-index scans bypass this cache so scanning a recording does not evict the
interactive working set.

Queries run concurrently, with four shared blocking-work slots for chunk decoding,
payload copying, evaluation, and response encoding. Each recording admits up to 128
observations and 256 reply tasks. New cursor revisions replace pending/active work
for the same observation without affecting other panels or clients. Shared decodes
and index builds survive an individual query being superseded. Closing an observation
cancels its server work; abandoned idle workers expire after 120 seconds.
Rebuild both the server and Twix together: plan requests now carry observation IDs
and revisions, and the protocol includes explicit cancellation.

To inspect index builds, chunk misses, evaluation phases, queue waits, and response
encoding times, run with `RUST_LOG=mcap_replay=debug`. Use `trace` for cache hits.

## Options

Pass options after `--` in the Cargo command above.

| Option | Purpose |
| --- | --- |
| `--namespace /replay` | Single-file namespace or directory prefix; must be `/replay` or below it. |
| `--listen tcp/127.0.0.1:0` | Choose a router listen address; port `0` selects an available port. |
| `--router tcp/127.0.0.1:7447` | Connect to an existing router instead of starting one. |
| `--index-cache-mib 1024` | Global timestamp-index memory budget; defaults to 1 GiB (1024 MiB). |
| `--chunk-cache-mib 256` | Global decoded-chunk buffer budget, including in-flight and pinned buffers; defaults to 256 MiB. |

`--listen` and `--router` are mutually exclusive. Use `--help` for all options.

## Playback

In Twix, open **+ → Timeline**, then add the panels you want to inspect.

Panels show **Querying server** for queries or retries lasting longer than 250 ms.

- Click or drag to seek; seeking pauses playback.
- Dragging submits the latest position each UI frame, without a fixed preview interval.
- Use the header buttons to play, pause, step, or jump to the start/end.
- Wheel/pinch zooms; middle-drag or Shift + wheel pans. **Frame all** fits the recording.
- **⋯** offers exact-time jumps, step size, and timestamp copying.

Playback is independent for each Twix client.
See the [Twix timeline guide](../../docs/tooling/twix.md#mcap-timeline) for keyboard shortcuts.
