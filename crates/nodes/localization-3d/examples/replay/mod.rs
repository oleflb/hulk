use std::{
    collections::BTreeMap,
    fs::File,
    io::{Read, Seek, Write},
    path::Path,
};

use color_eyre::{
    Result,
    eyre::{ensure, eyre},
};
use mcap::{
    Writer,
    records::{MessageHeader, Metadata, Record},
    sans_io::{LinearReadEvent, LinearReader},
};
use ros_z::{
    Message,
    message::{SerdeCdrCodec, WireEncoder},
    time::Time,
};
use types::localization::{LocalizationEstimate, LocalizationStatus};

pub const ESTIMATE_TOPIC: &str = "comparison/localization/estimate";
pub const STATUS_TOPIC: &str = "comparison/localization/status";

pub struct Comparison {
    pub delivery: u64,
    pub status: LocalizationStatus,
    pub estimate: Option<LocalizationEstimate>,
}

pub fn write_comparison(
    input: &Path,
    output: &Path,
    rows: &[Comparison],
    audit: &serde_json::Value,
) -> Result<()> {
    // create_new also rejects symlinks/hardlinks to input and never truncates an existing file.
    let output = File::create_new(output)?;
    let mut writer = Writer::new(output)?;
    copy_recording(File::open(input)?, &mut writer)?;
    write_rows(&mut writer, rows, audit)?;
    writer.finish()?;
    Ok(())
}

fn write_rows<W: Write + Seek>(
    writer: &mut Writer<W>,
    rows: &[Comparison],
    audit: &serde_json::Value,
) -> Result<()> {
    let estimate_channel = add_channel::<LocalizationEstimate, _>(writer, ESTIMATE_TOPIC)?;
    let status_channel = add_channel::<LocalizationStatus, _>(writer, STATUS_TOPIC)?;
    for (sequence, row) in rows.iter().enumerate() {
        let sequence = u32::try_from(sequence)?;
        write_message(
            writer,
            status_channel,
            sequence,
            row.delivery,
            Time::from_nanos(i64::try_from(row.delivery)?),
            &row.status,
        )?;
        if let Some(estimate) = row.estimate {
            write_message(
                writer,
                estimate_channel,
                sequence,
                row.delivery,
                estimate.time,
                &estimate,
            )?;
        }
    }
    writer.write_metadata(&Metadata {
        name: "comparison/replay".into(),
        metadata: BTreeMap::from([("kinematic_odometry".into(), serde_json::to_string(audit)?)]),
    })?;
    Ok(())
}

fn add_channel<T: Message, W: Write + Seek>(writer: &mut Writer<W>, topic: &str) -> Result<u16> {
    let name = T::type_name();
    let schema = writer.add_schema(
        &name,
        "ros-z-schema-json",
        &serde_json::to_vec(&T::schema())?,
    )?;
    Ok(writer.add_channel(
        schema,
        topic,
        "ros-z-cdr",
        &BTreeMap::from([
            ("ros_z.type_name".into(), name),
            (
                "ros_z.schema_hash".into(),
                T::schema_hash().to_hash_string(),
            ),
        ]),
    )?)
}

fn write_message<T: serde::Serialize, W: Write + Seek>(
    writer: &mut Writer<W>,
    channel_id: u16,
    sequence: u32,
    log_time: u64,
    time: Time,
    message: &T,
) -> Result<()> {
    writer.write_to_known_channel(
        &MessageHeader {
            channel_id,
            sequence,
            log_time,
            publish_time: u64::try_from(time.as_nanos())?,
        },
        &SerdeCdrCodec::<T>::serialize(message)?,
    )?;
    Ok(())
}

fn copy_recording<W: Write + Seek>(mut input: impl Read, writer: &mut Writer<W>) -> Result<()> {
    let mut reader = LinearReader::new();
    let mut schemas = BTreeMap::from([(0, 0)]);
    let mut channels = BTreeMap::new();
    while let Some(event) = reader.next_event() {
        match event? {
            LinearReadEvent::ReadRequest(n) => {
                let n = input.read(reader.insert(n))?;
                ensure!(n > 0, "MCAP ended before the reader completed");
                reader.notify_read(n);
            }
            LinearReadEvent::Record { opcode, data } => match mcap::parse_record(opcode, data)? {
                Record::Schema { header, data } => {
                    schemas.insert(
                        header.id,
                        writer.add_schema(&header.name, &header.encoding, &data)?,
                    );
                }
                Record::Channel(channel) => {
                    ensure!(
                        channel.topic != ESTIMATE_TOPIC && channel.topic != STATUS_TOPIC,
                        "input already contains comparison channels"
                    );
                    let schema = schemas
                        .get(&channel.schema_id)
                        .ok_or_else(|| eyre!("unknown input schema"))?;
                    channels.insert(
                        channel.id,
                        writer.add_channel(
                            *schema,
                            &channel.topic,
                            &channel.message_encoding,
                            &channel.metadata,
                        )?,
                    );
                }
                Record::Message { mut header, data } => {
                    header.channel_id = *channels
                        .get(&header.channel_id)
                        .ok_or_else(|| eyre!("unknown input channel"))?;
                    writer.write_to_known_channel(&header, &data)?;
                }
                Record::Metadata(metadata) => writer.write_metadata(&metadata)?,
                Record::Attachment { header, data, .. } => {
                    writer.start_attachment(data.len() as u64, header)?;
                    writer.put_attachment_bytes(&data)?;
                    writer.finish_attachment()?;
                }
                _ => (),
            },
        }
    }
    Ok(())
}
