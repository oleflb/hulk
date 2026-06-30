use std::{
    boxed::Box, collections::VecDeque, future::Future, path::PathBuf, pin::Pin, sync::Arc,
    time::Duration,
};

use color_eyre::{Result, eyre::WrapErr};
use encoded_frame_decoder::{
    DecodedFrame, DecoderConfig, FfmpegHevcDecoder, OutputPixelFormat,
    hevc_access_unit_contains_irap,
};
use ros_z::{prelude::*, qos::QosHistory, shm::ShmProviderBuilder, time::Time};
use tokio::task::JoinSet;
use types::{
    encoded_frame::{EncodedFrame, EncodedFrameCodec},
    nv12_image::Nv12Image,
    time_wrapper::TimeWrapper,
};
use zenoh::shm::{PosixShmProviderBackend, ShmProvider};

pub const DEFAULT_FFMPEG_PATH: &str = "ffmpeg";
pub const DEFAULT_DECODED_SHM_POOL_SIZE: usize = 128 * 1024 * 1024;
const IDLE_SUBSCRIBER_WAIT: Duration = Duration::from_millis(100);

#[derive(Clone, Debug)]
pub struct DecoderNodeConfig {
    pub ffmpeg_path: PathBuf,
    pub decoded_shm_pool_size: usize,
}

impl Default for DecoderNodeConfig {
    fn default() -> Self {
        Self {
            ffmpeg_path: PathBuf::from(DEFAULT_FFMPEG_PATH),
            decoded_shm_pool_size: DEFAULT_DECODED_SHM_POOL_SIZE,
        }
    }
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx, DecoderNodeConfig::default()))
}

pub async fn run(ctx: Arc<Context>, config: DecoderNodeConfig) -> Result<()> {
    let node = Arc::new(ctx.create_node("encoded_frame_decoder").build().await?);
    let decoded_shm_provider = Arc::new(
        ShmProviderBuilder::new(config.decoded_shm_pool_size)
            .build()
            .wrap_err("create decoded-frame SHM provider")?,
    );
    let left_pub = node
        .publisher::<Nv12Image>("inputs/left_nv12_image")?
        .build()
        .await?;
    let right_pub = node
        .publisher::<Nv12Image>("inputs/right_nv12_image")?
        .build()
        .await?;

    let mut channel_tasks = JoinSet::new();
    channel_tasks.spawn(run_channel(
        "left",
        node.clone(),
        "inputs/left_encoded_frame",
        left_pub,
        config.ffmpeg_path.clone(),
        decoded_shm_provider.clone(),
    ));
    channel_tasks.spawn(run_channel(
        "right",
        node,
        "inputs/right_encoded_frame",
        right_pub,
        config.ffmpeg_path,
        decoded_shm_provider,
    ));

    while let Some(result) = channel_tasks.join_next().await {
        match result {
            Ok(Ok(())) => {}
            Ok(Err(error)) => {
                channel_tasks.abort_all();
                return Err(error);
            }
            Err(error) => {
                channel_tasks.abort_all();
                return Err(error).wrap_err("decoder channel task failed");
            }
        }
    }

    Ok(())
}

async fn run_channel(
    channel: &'static str,
    node: Arc<Node>,
    input_topic: &'static str,
    publisher: Publisher<Nv12Image>,
    ffmpeg_path: PathBuf,
    decoded_shm_provider: Arc<ShmProvider<PosixShmProviderBackend>>,
) -> Result<()> {
    let mut decoder = None;
    loop {
        if !publisher.has_subscribers() {
            decoder = None;
            publisher
                .wait_for_subscribers(1, IDLE_SUBSCRIBER_WAIT)
                .await;
            continue;
        }

        let subscriber = node
            .subscriber::<TimeWrapper<EncodedFrame>>(input_topic)?
            .qos(encoded_input_qos())
            .build()
            .await?;

        while publisher.has_subscribers() {
            let wrapped_frame =
                match tokio::time::timeout(IDLE_SUBSCRIBER_WAIT, subscriber.recv()).await {
                    Ok(result) => result?,
                    Err(_) => continue,
                };

            let frame_time = wrapped_frame.time;
            let frame = wrapped_frame.inner;
            if frame.codec != EncodedFrameCodec::Hevc {
                color_eyre::eyre::bail!("unsupported encoded frame codec {:?}", frame.codec);
            }

            let decoded = {
                let decoder = ensure_decoder(
                    &mut decoder,
                    frame.width,
                    frame.height,
                    &ffmpeg_path,
                    decoded_shm_provider.clone(),
                )
                .wrap_err_with(|| format!("create {channel} decoder"))?;

                if !decoder.is_synchronized {
                    if !hevc_access_unit_contains_irap(&frame.data) {
                        continue;
                    }
                    decoder.is_synchronized = true;
                }

                decoder.pending_metadata.push_back(PendingFrameMetadata {
                    time: frame_time,
                    frame_identifier: frame.frame_identifier,
                    timestamp_ns: frame.timestamp_ns,
                    presentation_timestamp_us: frame.presentation_timestamp_us,
                });

                let decoded = tokio::task::block_in_place(|| decoder.decoder.decode_frame(&frame))
                    .wrap_err_with(|| format!("decode {channel} HEVC frame"))?
                    .map(|decoded| {
                        let metadata = decoder.pending_metadata.pop_front().ok_or_else(|| {
                            color_eyre::eyre::eyre!(
                                "decoded {channel} frame has no queued metadata"
                            )
                        })?;
                        Ok::<_, color_eyre::Report>((metadata, decoded))
                    })
                    .transpose()?;

                if decoded.is_none() {
                    decoder.pending_metadata.clear();
                    decoder.is_synchronized = false;
                }
                decoded
            };
            let Some((metadata, decoded)) = decoded else {
                tracing::warn!(channel, "decoder timed out; restarting FFmpeg decoder");
                decoder = None;
                continue;
            };

            let image = nv12_image_from_decoded_frame(metadata, decoded)
                .wrap_err_with(|| format!("build {channel} NV12 image"))?;
            publisher.publish(&image).await?;
        }
    }
}

fn encoded_input_qos() -> QosProfile {
    QosProfile {
        history: QosHistory::from_depth(1),
        ..Default::default()
    }
}

fn ensure_decoder<'a>(
    decoder: &'a mut Option<DecoderState>,
    width: u32,
    height: u32,
    ffmpeg_path: &PathBuf,
    decoded_shm_provider: Arc<ShmProvider<PosixShmProviderBackend>>,
) -> Result<&'a mut DecoderState> {
    let recreate = decoder
        .as_ref()
        .is_none_or(|decoder| decoder.width != width || decoder.height != height);
    if recreate {
        let config = DecoderConfig::new(width, height)
            .with_ffmpeg_path(ffmpeg_path.clone())
            .with_output_shm_provider(decoded_shm_provider);
        *decoder = Some(DecoderState {
            width,
            height,
            decoder: FfmpegHevcDecoder::spawn(config)?,
            pending_metadata: VecDeque::new(),
            is_synchronized: false,
        });
    }
    Ok(decoder.as_mut().expect("decoder was just initialized"))
}

struct DecoderState {
    width: u32,
    height: u32,
    decoder: FfmpegHevcDecoder,
    pending_metadata: VecDeque<PendingFrameMetadata>,
    is_synchronized: bool,
}

struct PendingFrameMetadata {
    time: Time,
    frame_identifier: u32,
    timestamp_ns: u64,
    presentation_timestamp_us: u64,
}

fn nv12_image_from_decoded_frame(
    metadata: PendingFrameMetadata,
    decoded: DecodedFrame,
) -> Result<Nv12Image> {
    if decoded.pixel_format != OutputPixelFormat::Nv12 {
        color_eyre::eyre::bail!(
            "decoded frame format is not NV12: {:?}",
            decoded.pixel_format
        );
    }

    Ok(Nv12Image {
        time: metadata.time,
        frame_identifier: metadata.frame_identifier,
        timestamp_ns: metadata.timestamp_ns,
        presentation_timestamp_us: metadata.presentation_timestamp_us,
        width: decoded.width,
        height: decoded.height,
        step: decoded.width,
        data: decoded.data,
    })
}
