use std::{
    boxed::Box,
    collections::BTreeMap,
    fs::{self, File},
    future::Future,
    io::BufWriter,
    path::PathBuf,
    pin::Pin,
    sync::Arc,
    time::Duration,
};

use booster::ImuState;
use color_eyre::{
    Result,
    eyre::{WrapErr, eyre},
};
use coordinate_systems::{Field, Robot};
use field_mark_association::{FieldMarkAssociations, GlobalLocalizationDebug};
use kinematics::robot_kinematics::RobotKinematics;
use linear_algebra::Isometry3;
use mcap::{Writer, records::MessageHeader};
use projection::{camera_matrix::CameraMatrix, intrinsic::Intrinsic};
use ros_z::{Message, attachment::Attachment, prelude::*, time::Time};
use serde::{Deserialize, Serialize};
use tokio::{sync::mpsc, task::JoinSet};
use tokio_util::sync::CancellationToken;
use types::{
    field_dimensions::FieldDimensions,
    object_detection::{Object, RobocupObjectLabel},
    stereo_image_pair::StereoImagePair,
    time_wrapper::TimeWrapper,
    visual_odometry::VisualOdometryDelta,
};

type ChannelId = u16;

#[derive(Clone, Debug, Deserialize, Serialize, Message)]
#[serde(deny_unknown_fields)]
pub struct LocalizationRecorderParameters {
    pub enable: bool,
    pub output_path: PathBuf,
    pub max_duration: Option<Duration>,
    pub include_raw_images: bool,
}

impl Default for LocalizationRecorderParameters {
    fn default() -> Self {
        Self {
            enable: false,
            output_path: PathBuf::from("/tmp/localization_recording.mcap"),
            max_duration: None,
            include_raw_images: false,
        }
    }
}

impl LocalizationRecorderParameters {
    fn validate(&self) -> std::result::Result<(), String> {
        if self.output_path.as_os_str().is_empty() {
            return Err("output_path must not be empty".to_string());
        }
        if self.max_duration.is_some_and(|duration| duration.is_zero()) {
            return Err("max_duration must be positive when set".to_string());
        }
        Ok(())
    }
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("localization_recorder").build().await?;
    let parameters =
        node.bind_parameter_as::<LocalizationRecorderParameters>("localization_recorder")?;
    parameters.add_validation_hook(LocalizationRecorderParameters::validate)?;
    let parameters = parameters.snapshot().typed().clone();

    if !parameters.enable {
        std::future::pending::<()>().await;
        return Ok(());
    }

    if let Some(parent) = parameters.output_path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent).wrap_err_with(|| {
            format!(
                "failed to create recorder output directory {}",
                parent.display()
            )
        })?;
    }

    let file = File::create(&parameters.output_path).wrap_err_with(|| {
        format!(
            "failed to create localization recording {}",
            parameters.output_path.display()
        )
    })?;
    let writer = McapWriter::new(BufWriter::new(file))?;
    let (sample_sender, sample_receiver) = mpsc::unbounded_channel();
    let token = CancellationToken::new();
    let writer_task = tokio::spawn(write_mcap(sample_receiver, writer, token.clone()));

    let mut recorders = JoinSet::new();
    spawn_topic::<ImuState>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "inputs/imu_state",
    )
    .await?;
    spawn_topic::<VisualOdometryDelta>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "visual_odometry/current_left_camera_to_previous_left_camera",
    )
    .await?;
    spawn_topic::<TimeWrapper<RobotKinematics>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "robot_kinematics",
    )
    .await?;
    spawn_topic::<TimeWrapper<CameraMatrix>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "camera_matrix",
    )
    .await?;
    spawn_topic::<Vec<Object<RobocupObjectLabel>>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "detected_objects",
    )
    .await?;
    spawn_topic::<FieldDimensions>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "field_dimensions",
    )
    .await?;
    spawn_topic::<Option<Isometry3<Field, Robot>>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "localization",
    )
    .await?;
    spawn_topic::<TimeWrapper<nalgebra::Isometry3<f32>>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "visual_odometry/current_left_camera_to_visual_odometer",
    )
    .await?;
    spawn_topic::<Option<nalgebra::Isometry3<f32>>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "visual_odometry/previous_left_camera_to_current_left_camera",
    )
    .await?;
    spawn_topic::<Option<GlobalLocalizationDebug>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "debug/global_localization",
    )
    .await?;
    spawn_topic::<TimeWrapper<FieldMarkAssociations>>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "field_mark_association/associations",
    )
    .await?;
    spawn_topic::<Intrinsic>(
        &node,
        &mut recorders,
        sample_sender.clone(),
        "debug/calibrated_intrinsics",
    )
    .await?;

    if parameters.include_raw_images {
        spawn_topic::<TimeWrapper<StereoImagePair>>(
            &node,
            &mut recorders,
            sample_sender.clone(),
            "inputs/stereo_image_pair",
        )
        .await?;
    }

    tracing::info!(
        path = %parameters.output_path.display(),
        include_raw_images = parameters.include_raw_images,
        "localization recording started"
    );

    drop(sample_sender);
    if let Some(max_duration) = parameters.max_duration {
        tokio::select! {
            _ = tokio::time::sleep(max_duration) => {
                recorders.abort_all();
            }
            result = recorders.join_next() => {
                handle_recorder_result(result)?;
                recorders.abort_all();
            }
        }
    } else {
        while let Some(result) = recorders.join_next().await {
            result.wrap_err("localization recorder task panicked")??;
        }
    }

    token.cancel();
    drop(recorders);
    let samples_written = writer_task
        .await
        .wrap_err("localization recorder writer task panicked")??;
    tracing::info!(
        path = %parameters.output_path.display(),
        samples_written,
        "localization recording finished"
    );

    Ok(())
}

async fn spawn_topic<T>(
    node: &Node,
    recorders: &mut JoinSet<Result<()>>,
    sample_sender: mpsc::UnboundedSender<RecordedSample>,
    topic: &'static str,
) -> Result<()>
where
    T: Message + Send + Sync + 'static,
{
    let mut subscriber = node.subscriber::<T>(topic)?.raw().build().await?;
    let channel = Arc::new(RecordedChannel::for_message::<T>(topic));

    recorders.spawn(async move {
        loop {
            let sample = subscriber
                .recv()
                .await
                .wrap_err_with(|| format!("failed to receive raw sample from {topic}"))?;
            let attachment = sample
                .attachment()
                .ok_or_else(|| eyre!("sample on {topic} has no ros-z attachment"))
                .and_then(|raw| {
                    Attachment::try_from(raw).map_err(|error| {
                        eyre!("failed to decode ros-z attachment on {topic}: {error}")
                    })
                })?;
            let source_time = attachment.source_time();
            let transport_time = sample
                .timestamp()
                .map(|timestamp| Time::from_wallclock(timestamp.get_time().to_system_time()))
                .unwrap_or(source_time);
            let payload = sample.payload().to_bytes().to_vec();
            let sequence = u32::try_from(attachment.sequence_number).unwrap_or(u32::MAX);

            sample_sender
                .send(RecordedSample {
                    channel: channel.clone(),
                    payload,
                    sequence,
                    log_time: time_to_mcap_nanos(transport_time),
                    publish_time: time_to_mcap_nanos(source_time),
                })
                .map_err(|_| eyre!("localization recorder writer stopped"))?;
        }
    });

    Ok(())
}

fn handle_recorder_result(
    result: Option<std::result::Result<Result<()>, tokio::task::JoinError>>,
) -> Result<()> {
    if let Some(result) = result {
        result.wrap_err("localization recorder task panicked")??;
    }
    Ok(())
}

async fn write_mcap(
    mut sample_receiver: mpsc::UnboundedReceiver<RecordedSample>,
    mut writer: McapWriter<BufWriter<File>>,
    token: CancellationToken,
) -> Result<usize> {
    let mut samples_written = 0;
    while let Some(Some(sample)) = token.run_until_cancelled(sample_receiver.recv()).await {
        writer.write(sample)?;
        samples_written += 1;
    }
    writer.finish()?;
    Ok(samples_written)
}

#[derive(Clone)]
struct RecordedChannel {
    topic: &'static str,
    schema_name: String,
    schema_data: Vec<u8>,
    metadata: BTreeMap<String, String>,
}

impl RecordedChannel {
    fn for_message<T: Message>(topic: &'static str) -> Self {
        let schema_name = T::type_name();
        let schema_data = serde_json::to_vec(&T::schema()).unwrap_or_default();
        let mut metadata = BTreeMap::new();
        metadata.insert("ros_z.type_name".to_string(), schema_name.clone());
        metadata.insert(
            "ros_z.schema_hash".to_string(),
            T::schema_hash().to_hash_string(),
        );

        Self {
            topic,
            schema_name,
            schema_data,
            metadata,
        }
    }
}

struct RecordedSample {
    channel: Arc<RecordedChannel>,
    payload: Vec<u8>,
    sequence: u32,
    log_time: u64,
    publish_time: u64,
}

struct McapWriter<W: std::io::Write + std::io::Seek> {
    writer: Writer<W>,
    channel_mapping: BTreeMap<&'static str, ChannelId>,
}

impl<W> McapWriter<W>
where
    W: std::io::Write + std::io::Seek,
{
    fn new(writer: W) -> Result<Self> {
        Ok(Self {
            writer: Writer::new(writer)?,
            channel_mapping: BTreeMap::new(),
        })
    }

    fn write(&mut self, sample: RecordedSample) -> Result<()> {
        let channel_id = match self.channel_mapping.get(sample.channel.topic).copied() {
            Some(channel_id) => channel_id,
            None => {
                let schema_id = self.writer.add_schema(
                    &sample.channel.schema_name,
                    "ros-z-schema-json",
                    &sample.channel.schema_data,
                )?;
                let channel_id = self.writer.add_channel(
                    schema_id,
                    sample.channel.topic,
                    "ros-z-cdr",
                    &sample.channel.metadata,
                )?;
                self.channel_mapping
                    .insert(sample.channel.topic, channel_id);
                channel_id
            }
        };

        self.writer.write_to_known_channel(
            &MessageHeader {
                channel_id,
                sequence: sample.sequence,
                log_time: sample.log_time,
                publish_time: sample.publish_time,
            },
            &sample.payload,
        )?;

        Ok(())
    }

    fn finish(mut self) -> Result<()> {
        self.writer.finish()?;
        Ok(())
    }
}

fn time_to_mcap_nanos(time: Time) -> u64 {
    u64::try_from(time.as_nanos()).unwrap_or_default()
}
