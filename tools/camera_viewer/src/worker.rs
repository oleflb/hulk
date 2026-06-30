use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use color_eyre::{Result, eyre::WrapErr};
use ros_z::{prelude::*, qos::QosHistory};
use tokio::task::JoinSet;
use types::{encoded_frame::EncodedFrame, time_wrapper::TimeWrapper};

use crate::{
    decoder::{DecodeOutcome, StreamDecoder},
    fps::FpsMeter,
    state::SharedCameraState,
};

const FRAME_POLL_INTERVAL: Duration = Duration::from_millis(200);

#[derive(Clone, Debug)]
pub(crate) struct WorkerConfig {
    pub(crate) router: String,
    pub(crate) namespace: String,
    pub(crate) ffmpeg_path: PathBuf,
    pub(crate) frame_timeout: Duration,
}

pub(crate) struct WorkerHandle {
    stop: Arc<AtomicBool>,
    _join_handle: JoinHandle<()>,
}

impl WorkerHandle {
    pub(crate) fn spawn(config: WorkerConfig, states: Vec<Arc<SharedCameraState>>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = stop.clone();
        let join_handle = thread::Builder::new()
            .name("camera-viewer-worker".to_string())
            .spawn(move || run_worker_thread(config, states, worker_stop))
            .expect("failed to spawn camera viewer worker");

        Self {
            stop,
            _join_handle: join_handle,
        }
    }
}

impl Drop for WorkerHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

fn run_worker_thread(
    config: WorkerConfig,
    states: Vec<Arc<SharedCameraState>>,
    stop: Arc<AtomicBool>,
) {
    if let Err(error) = run_worker(config, states.clone(), stop) {
        let message = format!("{error:#}");
        for state in states {
            state.set_error(message.clone());
        }
    }
}

fn run_worker(
    config: WorkerConfig,
    states: Vec<Arc<SharedCameraState>>,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .worker_threads(states.len().max(2))
        .thread_name("camera-viewer-ros")
        .build()
        .wrap_err("create Tokio runtime")?;
    runtime.block_on(run_ros(config, states, stop))
}

async fn run_ros(
    config: WorkerConfig,
    states: Vec<Arc<SharedCameraState>>,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    let context = Arc::new(
        ContextBuilder::default()
            .with_namespace(&config.namespace)
            .with_mode("client")
            .with_router_endpoint(config.router.clone())?
            .build()
            .await?,
    );
    let node = Arc::new(
        context
            .create_node("camera_viewer")
            .without_schema_service()
            .build()
            .await?,
    );

    let mut tasks = JoinSet::new();
    for state in states {
        let node = node.clone();
        let config = config.clone();
        let stop = stop.clone();
        tasks.spawn(async move {
            let result = run_camera(state.clone(), node, config, stop).await;
            if let Err(error) = &result {
                state.set_error(format!("{error:#}"));
            }
            result
        });
    }

    while !stop.load(Ordering::Relaxed) {
        match tokio::time::timeout(FRAME_POLL_INTERVAL, tasks.join_next()).await {
            Ok(Some(Ok(Ok(())))) => {}
            Ok(Some(Ok(Err(error)))) => tracing::error!("camera viewer task failed: {error:#}"),
            Ok(Some(Err(error))) => tracing::error!("camera viewer task panicked: {error}"),
            Ok(None) => break,
            Err(_) => {}
        }
    }

    tasks.abort_all();
    Ok(())
}

async fn run_camera(
    state: Arc<SharedCameraState>,
    node: Arc<Node>,
    config: WorkerConfig,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    let side = state.side();
    state.set_status(format!("subscribing to {}", side.topic()));
    let subscriber = node
        .subscriber::<TimeWrapper<EncodedFrame>>(side.topic())?
        .qos(encoded_input_qos())
        .build()
        .await?;
    state.set_status("waiting for publisher");

    let mut decoder = StreamDecoder::new(config.ffmpeg_path, config.frame_timeout);
    let mut receive_fps = FpsMeter::new();
    let mut decoded_fps = FpsMeter::new();

    while !stop.load(Ordering::Relaxed) {
        let wrapped_frame = match tokio::time::timeout(FRAME_POLL_INTERVAL, subscriber.recv()).await
        {
            Ok(result) => result?,
            Err(_) => {
                state.update_idle(
                    receive_fps.rate(Instant::now()),
                    subscriber.publisher_count(),
                );
                continue;
            }
        };

        let receive_rate = receive_fps.tick(Instant::now());
        state.update_received(receive_rate, subscriber.publisher_count());

        let decoded = tokio::task::block_in_place(|| decoder.decode(&wrapped_frame.inner))
            .wrap_err_with(|| format!("decode {} HEVC frame", side.label()))?;
        match decoded {
            DecodeOutcome::Decoded(frame) => {
                let decode_rate = decoded_fps.tick(Instant::now());
                state.update_decoded(frame, decode_rate);
            }
            DecodeOutcome::TimedOut => state.update_decoder_timeout(),
            DecodeOutcome::WaitingForIrap => state.update_waiting_for_irap(),
        }
    }

    Ok(())
}

fn encoded_input_qos() -> QosProfile {
    QosProfile {
        history: QosHistory::from_depth(1),
        ..Default::default()
    }
}
