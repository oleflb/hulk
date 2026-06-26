use std::{error::Error, fmt};

use ros_z::{Message, MessageSchema, SchemaBuilder, ZBuf, message::WireEncoder, time::Time};
use ros_z_cdr::{CdrBuffer, CdrReader, CdrWriter, LittleEndian, ZBufWriter};
use serde::{Deserialize, Serialize};
use zenoh_buffers::buffer::Buffer;

/// Fixed-size CDR metadata prefix before the NV12 payload bytes.
const PREFIX_LEN: usize = 52;
const NANOS_PER_SECOND: u64 = 1_000_000_000;

/// Zero-copy friendly NV12 image for high-rate robot vision.
///
/// The payload is stored as a `ZBuf` so receivers can keep Zenoh SHM slices
/// instead of copying the image into an owned byte array.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Nv12Image {
    /// ROS-Z frame time propagated from the encoded input stream.
    pub time: Time,
    /// Monotonic frame identifier from the camera pipeline.
    pub frame_identifier: u32,
    /// Camera-side capture timestamp in nanoseconds.
    pub timestamp_ns: u64,
    /// Encoder presentation timestamp in microseconds.
    pub presentation_timestamp_us: u64,
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Full row length in bytes for the Y and interleaved UV planes.
    pub step: u32,
    /// Packed NV12 bytes: full-resolution Y plane followed by half-height UV.
    pub data: ZBuf,
}

impl Nv12Image {
    /// Returns the expected payload size for this image layout.
    pub fn expected_data_len(&self) -> Result<usize, Nv12ImageCodecError> {
        expected_data_len(self.width, self.height, self.step)
    }
}

impl MessageSchema for Nv12Image {
    fn build_schema(
        builder: &mut SchemaBuilder,
    ) -> Result<ros_z::__private::ros_z_schema::TypeDef, ros_z::__private::ros_z_schema::SchemaError>
    {
        use ros_z::__private::ros_z_schema::{PrimitiveTypeDef, SequenceLengthDef, TypeDef};

        builder.define_message_struct::<Self>(|fields| {
            fields.field::<Time>("time")?;
            fields.field::<u32>("frame_identifier")?;
            fields.field::<u64>("timestamp_ns")?;
            fields.field::<u64>("presentation_timestamp_us")?;
            fields.field::<u32>("width")?;
            fields.field::<u32>("height")?;
            fields.field::<u32>("step")?;
            fields.field_with_shape(
                "data",
                TypeDef::Sequence {
                    element: Box::new(TypeDef::Primitive(PrimitiveTypeDef::U8)),
                    length: SequenceLengthDef::Dynamic,
                },
            );
            Ok(())
        })
    }
}

impl Message for Nv12Image {
    type Codec = Nv12ImageCodec;

    fn type_name() -> String {
        "types::nv12_image::Nv12Image".to_string()
    }
}

/// Custom codec that keeps the large NV12 payload as `ZBuf` slices.
pub struct Nv12ImageCodec;

#[derive(Debug)]
pub struct Nv12ImageCodecError(String);

impl Nv12ImageCodecError {
    fn new(message: impl Into<String>) -> Self {
        Self(message.into())
    }
}

impl fmt::Display for Nv12ImageCodecError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl Error for Nv12ImageCodecError {}

impl From<ros_z_cdr::Error> for Nv12ImageCodecError {
    fn from(error: ros_z_cdr::Error) -> Self {
        Self(error.to_string())
    }
}

impl WireEncoder for Nv12ImageCodec {
    type Input<'a> = &'a Nv12Image;
    type Error = Nv12ImageCodecError;

    fn serialize_to_zbuf(input: &Nv12Image) -> Result<zenoh_buffers::ZBuf, Self::Error> {
        Self::serialize_to_zbuf_with_hint(input, Self::serialized_size_hint(input))
    }

    fn serialize_to_zbuf_with_hint(
        input: &Nv12Image,
        capacity_hint: usize,
    ) -> Result<zenoh_buffers::ZBuf, Self::Error> {
        validate_image(input)?;
        let mut writer = ZBufWriter::with_capacity(capacity_hint.min(PREFIX_LEN));
        write_prefix(input, &mut writer)?;
        Ok(writer.into_zbuf())
    }

    fn serialized_size_hint(input: &Nv12Image) -> usize {
        PREFIX_LEN + input.data.len()
    }

    fn serialize_to_shm(
        input: &Nv12Image,
        _estimated_size: usize,
        _provider: &zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>,
    ) -> ros_z::Result<(zenoh_buffers::ZBuf, usize)> {
        let zbuf = Self::serialize_to_zbuf(input)
            .map_err(|source| ros_z::Error::encode(std::any::type_name::<Nv12Image>(), source))?;
        let size = zbuf.len();
        Ok((zbuf, size))
    }

    fn serialize_to_buf(input: &Nv12Image, buffer: &mut Vec<u8>) -> Result<(), Self::Error> {
        validate_image(input)?;
        buffer.clear();
        buffer.extend_from_slice(&ros_z::message::CDR_HEADER_LE);
        let mut writer = CdrWriter::<LittleEndian, _>::new(buffer);
        write_metadata(input, &mut writer)?;
        writer
            .buffer_mut()
            .extend_from_slice(input.data.contiguous().as_ref());
        Ok(())
    }
}

impl ros_z::message::WireDecoder for Nv12ImageCodec {
    type Input<'a> = &'a [u8];
    type Output = Nv12Image;
    type Error = Nv12ImageCodecError;

    fn deserialize(input: &[u8]) -> Result<Self::Output, Self::Error> {
        let zbuf = ZBuf::from(input);
        <Self as ros_z::WireZBufDecoder>::deserialize_zbuf(&zbuf)
    }
}

impl ros_z::WireZBufDecoder for Nv12ImageCodec {
    fn deserialize_zbuf(input: &zenoh_buffers::ZBuf) -> Result<Nv12Image, Self::Error> {
        let input = ZBuf::from_zenoh(input.clone());
        if input.len() < PREFIX_LEN {
            return Err(Nv12ImageCodecError::new(format!(
                "NV12 image payload too short: {} bytes",
                input.len()
            )));
        }

        let prefix = copy_prefix(&input)?;
        if prefix[0..2] != [0x00, 0x01] {
            return Err(Nv12ImageCodecError::new(format!(
                "expected CDR_LE encapsulation, found {:?}",
                &prefix[0..2]
            )));
        }

        let mut reader = CdrReader::<LittleEndian>::new(&prefix[4..]);
        let time = read_time(&mut reader)?;
        let frame_identifier = reader.read_u32()?;
        let timestamp_ns = reader.read_u64()?;
        let presentation_timestamp_us = reader.read_u64()?;
        let width = reader.read_u32()?;
        let height = reader.read_u32()?;
        let step = reader.read_u32()?;
        let data_len = reader.read_u32()? as usize;

        let expected_len = expected_data_len(width, height, step)?;
        if data_len != expected_len {
            return Err(Nv12ImageCodecError::new(format!(
                "NV12 payload length {data_len} does not match expected {expected_len} for {width}x{height} step {step}"
            )));
        }
        if input.len() != PREFIX_LEN + data_len {
            return Err(Nv12ImageCodecError::new(format!(
                "NV12 message length {} does not match prefix + payload {}",
                input.len(),
                PREFIX_LEN + data_len
            )));
        }

        let data = input
            .subslice(PREFIX_LEN..PREFIX_LEN + data_len)
            .ok_or_else(|| Nv12ImageCodecError::new("failed to borrow NV12 payload slice"))?;

        Ok(Nv12Image {
            time,
            frame_identifier,
            timestamp_ns,
            presentation_timestamp_us,
            width,
            height,
            step,
            data,
        })
    }
}

fn write_prefix(input: &Nv12Image, writer: &mut ZBufWriter) -> Result<(), Nv12ImageCodecError> {
    writer.extend_from_slice(&ros_z::message::CDR_HEADER_LE);
    let mut cdr = CdrWriter::<LittleEndian, _>::new(writer);
    write_metadata(input, &mut cdr)?;
    cdr.buffer_mut().append_zbuf(&input.data);
    Ok(())
}

fn write_metadata<B: CdrBuffer>(
    input: &Nv12Image,
    writer: &mut CdrWriter<'_, LittleEndian, B>,
) -> Result<(), Nv12ImageCodecError> {
    write_time(input.time, writer);
    writer.write_u32(input.frame_identifier);
    writer.write_u64(input.timestamp_ns);
    writer.write_u64(input.presentation_timestamp_us);
    writer.write_u32(input.width);
    writer.write_u32(input.height);
    writer.write_u32(input.step);
    writer.write_u32(u32::try_from(input.data.len()).map_err(|_| {
        Nv12ImageCodecError::new(format!(
            "NV12 payload too large: {} bytes",
            input.data.len()
        ))
    })?);
    Ok(())
}

fn write_time<B: CdrBuffer>(time: Time, writer: &mut CdrWriter<'_, LittleEndian, B>) {
    let nanos = u64::try_from(time.as_nanos()).unwrap_or_default();
    writer.write_u64(nanos / NANOS_PER_SECOND);
    writer.write_u32((nanos % NANOS_PER_SECOND) as u32);
}

fn read_time(reader: &mut CdrReader<'_, LittleEndian>) -> Result<Time, Nv12ImageCodecError> {
    let secs = reader.read_u64()?;
    let nanos = reader.read_u32()?;
    if nanos >= NANOS_PER_SECOND as u32 {
        return Err(Nv12ImageCodecError::new(format!(
            "ROS-Z time nanoseconds out of range: {nanos}"
        )));
    }
    let total_nanos = secs
        .checked_mul(NANOS_PER_SECOND)
        .and_then(|value| value.checked_add(nanos as u64))
        .ok_or_else(|| Nv12ImageCodecError::new("ROS-Z time overflow"))?;
    let total_nanos = i64::try_from(total_nanos)
        .map_err(|_| Nv12ImageCodecError::new("ROS-Z time exceeds i64 nanoseconds"))?;
    Ok(Time::from_nanos(total_nanos))
}

fn copy_prefix(input: &ZBuf) -> Result<[u8; PREFIX_LEN], Nv12ImageCodecError> {
    let prefix = input
        .subslice(0..PREFIX_LEN)
        .ok_or_else(|| Nv12ImageCodecError::new("failed to borrow NV12 image prefix"))?;
    let bytes = prefix.contiguous();
    let bytes = bytes.as_ref();
    if bytes.len() < PREFIX_LEN {
        return Err(Nv12ImageCodecError::new(format!(
            "NV12 image prefix too short: {} bytes",
            bytes.len()
        )));
    }
    let mut prefix = [0; PREFIX_LEN];
    prefix.copy_from_slice(&bytes[..PREFIX_LEN]);
    Ok(prefix)
}

fn validate_image(input: &Nv12Image) -> Result<(), Nv12ImageCodecError> {
    let expected_len = input.expected_data_len()?;
    if input.data.len() != expected_len {
        return Err(Nv12ImageCodecError::new(format!(
            "NV12 payload length {} does not match expected {expected_len}",
            input.data.len()
        )));
    }
    Ok(())
}

fn expected_data_len(width: u32, height: u32, step: u32) -> Result<usize, Nv12ImageCodecError> {
    if width == 0 || height == 0 {
        return Err(Nv12ImageCodecError::new(
            "NV12 image dimensions must be nonzero",
        ));
    }
    if !width.is_multiple_of(2) || !height.is_multiple_of(2) {
        return Err(Nv12ImageCodecError::new(format!(
            "NV12 image dimensions must be even, got {width}x{height}"
        )));
    }
    if step < width {
        return Err(Nv12ImageCodecError::new(format!(
            "NV12 step {step} is smaller than width {width}"
        )));
    }

    let step = step as usize;
    let height = height as usize;
    step.checked_mul(height + height / 2)
        .ok_or_else(|| Nv12ImageCodecError::new("NV12 payload size overflow"))
}

#[cfg(test)]
mod tests {
    use ros_z::WireZBufDecoder;
    use ros_z::message::WireEncoder;

    use super::*;

    #[test]
    fn nv12_image_codec_roundtrips() {
        let image = Nv12Image {
            time: Time::from_nanos(123),
            frame_identifier: 42,
            timestamp_ns: 1_000,
            presentation_timestamp_us: 2_000,
            width: 4,
            height: 4,
            step: 4,
            data: ZBuf::from(vec![7; 24]),
        };

        let encoded = ZBuf::from_zenoh(Nv12ImageCodec::serialize_to_zbuf(&image).unwrap());
        assert_eq!(encoded.slice_count(), 2);

        let decoded = Nv12ImageCodec::deserialize_zbuf(&encoded.clone().into_inner()).unwrap();

        assert_eq!(decoded.frame_identifier, image.frame_identifier);
        assert_eq!(decoded.time, image.time);
        assert_eq!(decoded.timestamp_ns, image.timestamp_ns);
        assert_eq!(
            decoded.presentation_timestamp_us,
            image.presentation_timestamp_us
        );
        assert_eq!(decoded.width, image.width);
        assert_eq!(decoded.height, image.height);
        assert_eq!(decoded.step, image.step);
        assert_eq!(
            decoded.data.contiguous().as_ref(),
            image.data.contiguous().as_ref()
        );
        assert_eq!(decoded.data.slice_count(), 1);
    }

    #[test]
    fn nv12_image_codec_rejects_wrong_payload_size() {
        let image = Nv12Image {
            time: Time::zero(),
            frame_identifier: 0,
            timestamp_ns: 0,
            presentation_timestamp_us: 0,
            width: 4,
            height: 4,
            step: 4,
            data: ZBuf::from(vec![0; 23]),
        };

        assert!(Nv12ImageCodec::serialize_to_zbuf(&image).is_err());
    }
}
