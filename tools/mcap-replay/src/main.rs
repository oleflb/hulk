use std::{
    collections::HashMap,
    future::Future,
    io::{Error, stderr},
    num::NonZeroUsize,
    path::{Path, PathBuf},
    sync::Arc,
    time::Duration,
};

use clap::Parser;
use color_eyre::{
    Report,
    eyre::{Context as _, OptionExt, Result},
};
use parking_lot::Mutex;
use recording::{ChunkCache, IndexCache, PendingQuery, QueryRecording, Recording};
use ros_z::{
    config::RouterConfigBuilder,
    context::{Context, ContextBuilder},
    node::Node,
    time::Time,
};
use ros_z_debug::{
    TargetIdentity,
    query::{
        plan::QueryPlan,
        remote::{Request, Response, encode, query_key},
    },
};
use tokio::{
    signal::ctrl_c,
    sync::{mpsc, watch},
    task::{JoinSet, spawn_blocking},
};
use tokio_util::sync::CancellationToken;
use tracing::instrument;
use zenoh::query::Query;

mod directory;
mod recording;

#[derive(Parser)]
#[command(about = "Serve indexed ROS-Z MCAP queries to Twix over Zenoh")]
struct Arguments {
    /// An indexed MCAP file, or a directory to watch recursively.
    file: PathBuf,
    /// Namespace or directory prefix: /replay or a namespace below it.
    #[arg(long, default_value = "/replay")]
    namespace: String,
    /// Connect to an external router instead of starting an embedded router.
    #[arg(long, conflicts_with = "listen")]
    router: Option<String>,
    /// Embedded router listen endpoint. Use port 0 to pick an available port.
    #[arg(long, default_value = "tcp/0.0.0.0:7447")]
    listen: String,
    /// Global memory budget for lazy timestamp indexes, in MiB. Idle indexes are evicted.
    #[arg(long, default_value = "1024")]
    index_cache_mib: NonZeroUsize,
    /// Global memory budget for decoded chunks, including in-flight and pinned buffers, in MiB.
    #[arg(long, default_value = "256")]
    chunk_cache_mib: NonZeroUsize,
}

impl Arguments {
    async fn connect(&self) -> Result<Context> {
        let builder = if let Some(router) = &self.router {
            ContextBuilder::default().with_router_endpoint(router)?
        } else {
            let config = RouterConfigBuilder::new()
                .with_listen_endpoint(&self.listen)
                .with_override(
                    "transport/link/tx/queue/congestion_control/block/wait_before_close",
                    serde_json::json!(60_000_000),
                    "Allow large recording replies to finish over slow or lossy links",
                )
                .build_config()
                .map_err(Error::other)
                .wrap_err("configuring embedded router")?;
            ContextBuilder::default().with_zenoh_config(config)
        };
        builder.build().await.wrap_err("connecting replay server")
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt().with_writer(stderr).init();

    let args = Arguments::parse();
    let index_budget = args
        .index_cache_mib
        .get()
        .checked_mul(1024 * 1024)
        .ok_or_eyre("index cache size overflow")?;
    let namespace = TargetIdentity::new(args.namespace.clone())?
        .namespace()
        .to_owned();

    let context = args.connect().await?;
    let router_uris = if let Some(router) = &args.router {
        vec![router.clone()]
    } else {
        context
            .session()
            .info()
            .locators()
            .await
            .into_iter()
            .map(|locator| locator.to_string())
            .collect::<Vec<_>>()
    };
    let router = router_uris
        .first()
        .ok_or_eyre("embedded router has no reachable listen URI")?;

    for uri in &router_uris {
        println!("Router URI: {uri}");
    }

    let (cache, cleanup) = IndexCache::new(index_budget)?;
    let cache = Arc::new(cache);
    let chunk_budget = args
        .chunk_cache_mib
        .get()
        .checked_mul(1024 * 1024)
        .ok_or_eyre("chunk cache size overflow")?;
    let chunks = Arc::new(ChunkCache::new(chunk_budget)?);
    let result = async {
        if args.file.is_dir() {
            return directory::watch(
                &args.file,
                &namespace,
                &context,
                Arc::clone(&cache),
                Arc::clone(&chunks),
                router,
            )
            .await;
        }

        let (node, recording, name) =
            open_recording(&context, &args.file, &namespace, router).await?;
        serve_queries(
            &node,
            recording,
            name,
            Arc::clone(&cache),
            Arc::clone(&chunks),
            async {
                let _ = ctrl_c().await;
            },
        )
        .await
    }
    .await;

    chunks.shutdown().await;
    drop(cache);
    cleanup.await.wrap_err("joining index cleanup task")??;
    result
}

async fn open_recording(
    context: &Context,
    path: &Path,
    namespace: &str,
    router: &str,
) -> Result<(Node, Recording, String)> {
    let file = path.to_owned();
    let ns = namespace.to_owned();
    let recording = spawn_blocking(move || Recording::open(&file, &ns))
        .await?
        .wrap_err_with(|| format!("opening recording in {namespace}"))?;
    let node = context
        .create_node("mcap_replay")
        .with_namespace(namespace)
        .build()
        .await
        .wrap_err_with(|| format!("creating node in {namespace}"))?;

    println!("Twix: ./twix {namespace} --router {router}");
    Ok((node, recording, path.display().to_string()))
}

struct PlanRequest {
    cursor: Time,
    plan: QueryPlan,
    reply: mpsc::Sender<Response>,
}

struct Submission {
    revision: u64,
    request: Mutex<Option<PlanRequest>>,
    cancel: CancellationToken,
}

struct ObservationWorker {
    generation: uuid::Uuid,
    latest: watch::Sender<Submission>,
}

const MAX_OBSERVATIONS: usize = 128;

async fn serve_queries(
    node: &Node,
    recording: Recording,
    name: String,
    cache: Arc<IndexCache>,
    chunks: Arc<ChunkCache>,
    shutdown: impl Future<Output = ()>,
) -> Result<()> {
    let cancel = CancellationToken::new();
    let _cancel_on_drop = cancel.clone().drop_guard();
    let recording = Arc::new(QueryRecording::new(
        recording,
        name,
        cancel.clone(),
        cache,
        chunks.clone(),
    )?);
    tokio::pin!(shutdown);

    let key = query_key(node.namespace());
    let queryable = node
        .session()
        .declare_queryable(key.clone())
        .complete(true)
        .await
        .map_err(Error::other)
        .wrap_err("declaring replay queryable")?;
    tracing::info!(path = %recording.info.name, namespace = %node.namespace(), "serving recording");
    let mut replies = JoinSet::new();
    let mut workers = JoinSet::new();
    let mut observations = HashMap::new();

    let result = loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break Ok(()),
            Some(finished) = workers.join_next(), if !workers.is_empty() => {
                match finished {
                    Ok((id, generation)) => {
                        if observations.get(&id).is_some_and(|entry: &ObservationWorker| entry.generation == generation) {
                            observations.remove(&id);
                        }
                    }
                    Err(error) => tracing::error!(%error, "query worker failed"),
                }
            },
            Some(_) = replies.join_next(), if !replies.is_empty() => {},
            incoming = queryable.recv_async() => {
                let incoming = match incoming.map_err(Error::other).wrap_err("receiving replay query") {
                    Ok(incoming) => incoming,
                    Err(error) => break Err(error),
                };
                if replies.len() >= 2 * MAX_OBSERVATIONS {
                    tracing::warn!("query reply limit reached; dropping request");
                    continue;
                }
                let responses = dispatch_request(&incoming, &recording, &mut observations, &mut workers);
                replies.spawn(reply_to_query(incoming, key.clone(), responses, chunks.clone()));
            }
        }
    };

    replies.abort_all();
    cancel.cancel();
    drop(observations);
    while workers.join_next().await.is_some() {}
    recording.shutdown().await;
    spawn_blocking(move || drop(recording)).await?;

    result
}

async fn run_query_worker(
    recording: Arc<QueryRecording>,
    mut requests: watch::Receiver<Submission>,
    stopped: CancellationToken,
) {
    loop {
        let request = requests.borrow_and_update().request.lock().take();
        if let Some(request) = request {
            tokio::select! {
                biased;
                _ = stopped.cancelled() => break,
                changed = requests.changed() => {
                    if changed.is_err() { break; }
                    continue;
                },
                _ = request.reply.closed() => {},
                _ = process_request(&recording, &request) => {},
            }
        }
        tokio::select! {
            biased;
            _ = stopped.cancelled() => break,
            changed = requests.changed() => if changed.is_err() { break; },
            // Abandoned client observations must not occupy admission slots forever.
            _ = tokio::time::sleep(Duration::from_secs(120)) => break,
        }
    }
}

#[instrument(skip_all, fields(path = %recording.info.name))]
async fn process_request(recording: &Arc<QueryRecording>, request: &PlanRequest) {
    let mut query = match PendingQuery::new(recording, request.plan.clone(), request.cursor) {
        Ok(query) => query,
        Err(error) => {
            reply_query_error(&recording.info.name, &request.reply, error).await;
            return;
        }
    };
    let result = loop {
        let (evaluation, source) = match query.advance(recording).await {
            Ok(step) => step,
            Err(error) => break Err(error),
        };
        if request
            .reply
            .send(Response::PlanData(evaluation))
            .await
            .is_err()
        {
            break Ok(());
        }
        let Some(source) = source else {
            break Ok(());
        };
        if let Err(error) = query.build_async(recording, source).await {
            break Err(error);
        }
    };
    drop(query);
    if let Err(error) = result {
        reply_query_error(&recording.info.name, &request.reply, error).await;
    }
}

async fn reply_query_error(name: &str, reply: &mpsc::Sender<Response>, error: Report) {
    tracing::warn!(path = name, error = %format!("{error:#}"), "query failed");
    let _ = reply.send(Response::Error(format!("{error:#}"))).await;
}

fn dispatch_request(
    incoming: &Query,
    recording: &Arc<QueryRecording>,
    observations: &mut HashMap<String, ObservationWorker>,
    workers: &mut JoinSet<(String, uuid::Uuid)>,
) -> Result<(mpsc::Receiver<Response>, CancellationToken)> {
    let payload = incoming
        .payload()
        .filter(|payload| payload.len() <= 64 * 1024)
        .ok_or_eyre("missing or oversized request")?;
    let request: Request = serde_json::from_slice(&payload.to_bytes())?;
    let (reply, responses) = mpsc::channel(2);
    let cancel = recording.cancel_token().child_token();
    match request {
        // Catalog requests must not wait for long-running index builds.
        Request::Catalog => reply
            .try_send(Response::Catalog(recording.info.clone()))
            .wrap_err("sending recording catalog")?,
        Request::Plan {
            instance,
            observation,
            revision,
            cursor,
            plan,
        } => {
            if instance != recording.info.instance {
                reply.try_send(Response::RecordingChanged)?;
                return Ok((responses, cancel));
            }
            color_eyre::eyre::ensure!(
                !observation.is_empty() && observation.len() <= 256,
                "invalid observation ID"
            );
            let submission = Submission {
                revision,
                cancel: cancel.clone(),
                request: Mutex::new(Some(PlanRequest {
                    cursor,
                    plan,
                    reply,
                })),
            };
            if let Some(worker) = observations.get(&observation)
                && !worker.latest.is_closed()
            {
                color_eyre::eyre::ensure!(
                    revision >= worker.latest.borrow().revision,
                    "obsolete query revision"
                );
                worker.latest.send_replace(submission).cancel.cancel();
            } else {
                color_eyre::eyre::ensure!(
                    observations.len() < MAX_OBSERVATIONS
                        || observations.contains_key(&observation),
                    "too many query observations; retry"
                );
                let (latest, requests) = watch::channel(submission);
                let generation = uuid::Uuid::new_v4();
                observations.insert(
                    observation.clone(),
                    ObservationWorker { generation, latest },
                );
                let recording = Arc::clone(recording);
                let stopped = recording.cancel_token();
                workers.spawn(async move {
                    run_query_worker(recording, requests, stopped).await;
                    (observation, generation)
                });
            }
        }
        Request::Cancel {
            instance,
            observation,
            revision,
        } => {
            if instance == recording.info.instance
                && observations
                    .get(&observation)
                    .is_some_and(|worker| worker.latest.borrow().revision <= revision)
                && let Some(worker) = observations.remove(&observation)
            {
                worker.latest.borrow().cancel.cancel();
            }
        }
    }
    Ok((responses, cancel))
}

async fn reply_to_query(
    incoming: Query,
    key: String,
    responses: Result<(mpsc::Receiver<Response>, CancellationToken)>,
    chunks: Arc<ChunkCache>,
) {
    let (mut responses, superseded) = match responses {
        Ok(responses) => responses,
        Err(error) => {
            let _ = incoming.reply_err(error.to_string()).await;
            return;
        }
    };
    let result = async {
        while let Some(response) = responses.recv().await {
            let bytes = chunks.blocking(move || encode(&response)).await??;
            if incoming.reply(&key, bytes).await.is_err() {
                return Ok(());
            }
        }
        Ok::<(), Report>(())
    };
    let result = tokio::select! {
        biased;
        _ = superseded.cancelled() => return,
        result = result => result,
    };

    if let Err(error) = result {
        let _ = incoming.reply_err(error.to_string()).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_preserves_router_options() {
        let defaults = Arguments::try_parse_from(["mcap-replay", "test.mcap"]).unwrap();
        assert_eq!(defaults.listen, "tcp/0.0.0.0:7447");
        assert_eq!(defaults.index_cache_mib.get(), 1024);
        assert_eq!(defaults.chunk_cache_mib.get(), 256);
        assert!(
            Arguments::try_parse_from(["mcap-replay", "test.mcap", "--index-cache-mib", "0"])
                .is_err()
        );
        assert!(
            Arguments::try_parse_from(["mcap-replay", "test.mcap", "--chunk-cache-mib", "0"])
                .is_err()
        );
        assert!(
            Arguments::try_parse_from([
                "mcap-replay",
                "test.mcap",
                "--listen",
                "tcp/127.0.0.1:0",
                "--router",
                "tcp/127.0.0.1:7447"
            ])
            .is_err()
        );
    }
}
