use std::{collections::VecDeque, fmt, num::NonZeroUsize, sync::Arc};

use ros_z::time::Time;
use types::{encoded_frame::EncodedFrame, time_wrapper::TimeWrapper};

pub const TRIPLE_BUFFER_COUNT: usize = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FrameMetadata {
    pub time: Time,
    pub frame_identifier: u32,
    pub timestamp_ns: u64,
    pub presentation_timestamp_us: u64,
}

impl FrameMetadata {
    pub fn from_encoded_frame(frame: &TimeWrapper<EncodedFrame>) -> Self {
        Self {
            time: frame.time,
            frame_identifier: frame.inner.frame_identifier,
            timestamp_ns: frame.inner.timestamp_ns,
            presentation_timestamp_us: frame.inner.presentation_timestamp_us,
        }
    }

    fn timestamp_for_matching(self) -> i128 {
        self.timestamp_ns as i128
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StereoMetadata {
    pub left: FrameMetadata,
    pub right: FrameMetadata,
    pub timestamp_delta_ns: i128,
}

impl StereoMetadata {
    pub fn new(left: FrameMetadata, right: FrameMetadata) -> Self {
        Self {
            left,
            right,
            timestamp_delta_ns: left.timestamp_for_matching() - right.timestamp_for_matching(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DevicePointer {
    address: NonZeroUsize,
}

impl DevicePointer {
    /// Creates a non-null CUDA device pointer wrapper.
    pub fn new(address: usize) -> Result<Self, DeviceStereoError> {
        let address = NonZeroUsize::new(address).ok_or(DeviceStereoError::NullDevicePointer)?;
        Ok(Self { address })
    }

    /// Returns the raw CUDA device pointer address.
    pub fn address(self) -> usize {
        self.address.get()
    }

    /// Returns a checked byte offset from this pointer.
    pub fn offset(self, byte_offset: usize) -> Result<Self, DeviceStereoError> {
        let address = self
            .address
            .get()
            .checked_add(byte_offset)
            .ok_or(DeviceStereoError::PointerOverflow)?;
        Self::new(address)
    }
}

/// Keeps the CUDA allocation backing a `DeviceStereoNv12` alive.
///
/// Implementors are owned behind an `Arc`, so cloned frame handles keep the
/// decoder slot checked out until every consumer drops the frame.
pub trait DeviceBufferGuard: Send + Sync {}

impl<T> DeviceBufferGuard for T where T: Send + Sync {}

/// Stereo NV12 images packed in one CUDA device allocation.
///
/// The byte layout is `[left NV12][right NV12]`. Each NV12 image is contiguous
/// luma followed by interleaved chroma, and `image_size()` returns the size of
/// one image. The guard owns the allocation lifetime; consumers must not retain
/// raw device pointers after dropping the frame.
#[derive(Clone)]
pub struct DeviceStereoNv12 {
    metadata: StereoMetadata,
    width: u32,
    height: u32,
    image_size: usize,
    total_size: usize,
    buffer_index: usize,
    device_ptr: DevicePointer,
    stream_id: Option<usize>,
    ready_event_id: Option<usize>,
    guard: Arc<dyn DeviceBufferGuard>,
}

impl DeviceStereoNv12 {
    pub fn new(
        metadata: StereoMetadata,
        width: u32,
        height: u32,
        buffer_index: usize,
        device_ptr: DevicePointer,
        guard: Arc<dyn DeviceBufferGuard>,
    ) -> Result<Self, DeviceStereoError> {
        let image_size = nv12_image_size(width, height)?;
        let total_size = image_size
            .checked_mul(2)
            .ok_or(DeviceStereoError::SizeOverflow)?;
        Ok(Self {
            metadata,
            width,
            height,
            image_size,
            total_size,
            buffer_index,
            device_ptr,
            stream_id: None,
            ready_event_id: None,
            guard,
        })
    }

    pub fn with_stream_id(mut self, stream_id: usize) -> Self {
        self.stream_id = Some(stream_id);
        self
    }

    pub fn with_ready_event_id(mut self, ready_event_id: usize) -> Self {
        self.ready_event_id = Some(ready_event_id);
        self
    }

    pub fn metadata(&self) -> StereoMetadata {
        self.metadata
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn image_size(&self) -> usize {
        self.image_size
    }

    pub fn total_size(&self) -> usize {
        self.total_size
    }

    pub fn buffer_index(&self) -> usize {
        self.buffer_index
    }

    pub fn stream_id(&self) -> Option<usize> {
        self.stream_id
    }

    pub fn ready_event_id(&self) -> Option<usize> {
        self.ready_event_id
    }

    pub fn left_device_ptr(&self) -> DevicePointer {
        self.device_ptr
    }

    pub fn right_device_ptr(&self) -> Result<DevicePointer, DeviceStereoError> {
        self.device_ptr.offset(self.image_size)
    }

    pub fn guard(&self) -> &Arc<dyn DeviceBufferGuard> {
        &self.guard
    }
}

impl fmt::Debug for DeviceStereoNv12 {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("DeviceStereoNv12")
            .field("metadata", &self.metadata)
            .field("width", &self.width)
            .field("height", &self.height)
            .field("image_size", &self.image_size)
            .field("total_size", &self.total_size)
            .field("buffer_index", &self.buffer_index)
            .field("device_ptr", &self.device_ptr)
            .field("stream_id", &self.stream_id)
            .field("ready_event_id", &self.ready_event_id)
            .finish_non_exhaustive()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum DeviceStereoError {
    #[error("NV12 dimensions must be nonzero and even, got {width}x{height}")]
    InvalidDimensions { width: u32, height: u32 },
    #[error("CUDA device pointer must be non-null")]
    NullDevicePointer,
    #[error("CUDA device pointer offset overflow")]
    PointerOverflow,
    #[error("NV12 stereo buffer size overflow")]
    SizeOverflow,
}

pub fn nv12_image_size(width: u32, height: u32) -> Result<usize, DeviceStereoError> {
    if width == 0 || height == 0 || !width.is_multiple_of(2) || !height.is_multiple_of(2) {
        return Err(DeviceStereoError::InvalidDimensions { width, height });
    }
    (width as usize)
        .checked_mul(height as usize)
        .and_then(|pixels| pixels.checked_mul(3))
        .and_then(|bytes| bytes.checked_div(2))
        .ok_or(DeviceStereoError::SizeOverflow)
}

#[derive(Debug, Clone)]
pub struct StereoTimestampMatcher<T> {
    tolerance_ns: u64,
    max_queue_len: usize,
    left: VecDeque<Timestamped<T>>,
    right: VecDeque<Timestamped<T>>,
}

impl<T> StereoTimestampMatcher<T> {
    pub fn new(tolerance_ns: u64, max_queue_len: usize) -> Self {
        Self {
            tolerance_ns,
            max_queue_len: max_queue_len.max(1),
            left: VecDeque::new(),
            right: VecDeque::new(),
        }
    }

    pub fn push_left(&mut self, metadata: FrameMetadata, value: T) -> Option<StereoPair<T>> {
        self.push(Side::Left, metadata, value)
    }

    pub fn push_right(&mut self, metadata: FrameMetadata, value: T) -> Option<StereoPair<T>> {
        self.push(Side::Right, metadata, value)
    }

    fn push(&mut self, side: Side, metadata: FrameMetadata, value: T) -> Option<StereoPair<T>> {
        let other_queue = match side {
            Side::Left => &mut self.right,
            Side::Right => &mut self.left,
        };

        if let Some(match_index) = best_match_index(other_queue, metadata, self.tolerance_ns) {
            let other = other_queue
                .remove(match_index)
                .expect("matched index must exist");
            return Some(match side {
                Side::Left => StereoPair {
                    metadata: StereoMetadata::new(metadata, other.metadata),
                    left: value,
                    right: other.value,
                },
                Side::Right => StereoPair {
                    metadata: StereoMetadata::new(other.metadata, metadata),
                    left: other.value,
                    right: value,
                },
            });
        }

        let queue = match side {
            Side::Left => &mut self.left,
            Side::Right => &mut self.right,
        };
        queue.push_back(Timestamped { metadata, value });
        while queue.len() > self.max_queue_len {
            queue.pop_front();
        }
        None
    }
}

#[derive(Debug, Clone)]
pub struct StereoPair<T> {
    pub metadata: StereoMetadata,
    pub left: T,
    pub right: T,
}

#[derive(Debug, Clone)]
struct Timestamped<T> {
    metadata: FrameMetadata,
    value: T,
}

#[derive(Clone, Copy)]
enum Side {
    Left,
    Right,
}

fn best_match_index<T>(
    queue: &VecDeque<Timestamped<T>>,
    metadata: FrameMetadata,
    tolerance_ns: u64,
) -> Option<usize> {
    queue
        .iter()
        .enumerate()
        .filter_map(|(index, candidate)| {
            let delta = (metadata.timestamp_for_matching()
                - candidate.metadata.timestamp_for_matching())
            .unsigned_abs();
            if delta <= tolerance_ns as u128 {
                Some((index, delta))
            } else {
                None
            }
        })
        .min_by_key(|(_, delta)| *delta)
        .map(|(index, _)| index)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metadata(timestamp_ns: u64) -> FrameMetadata {
        FrameMetadata {
            time: Time::from_nanos(timestamp_ns as i64),
            frame_identifier: timestamp_ns as u32,
            timestamp_ns,
            presentation_timestamp_us: timestamp_ns / 1_000,
        }
    }

    #[test]
    fn matcher_pairs_closest_timestamp() {
        let mut matcher = StereoTimestampMatcher::new(5, 4);
        assert!(matcher.push_left(metadata(100), "left-100").is_none());
        assert!(matcher.push_right(metadata(90), "right-90").is_none());

        let pair = matcher
            .push_right(metadata(103), "right-103")
            .expect("frame should match");

        assert_eq!(pair.left, "left-100");
        assert_eq!(pair.right, "right-103");
        assert_eq!(pair.metadata.timestamp_delta_ns, -3);
    }

    #[test]
    fn matcher_drops_stale_frames() {
        let mut matcher = StereoTimestampMatcher::new(1, 2);
        assert!(matcher.push_left(metadata(10), 10).is_none());
        assert!(matcher.push_left(metadata(20), 20).is_none());
        assert!(matcher.push_left(metadata(30), 30).is_none());

        assert!(matcher.push_right(metadata(10), 10).is_none());
        let pair = matcher
            .push_right(metadata(30), 30)
            .expect("newest frame should remain matchable");
        assert_eq!(pair.left, 30);
        assert_eq!(pair.right, 30);
    }

    #[test]
    fn stereo_layout_offsets_right_image_after_left_image() {
        let guard: Arc<dyn DeviceBufferGuard> = Arc::new(());
        let frame = DeviceStereoNv12::new(
            StereoMetadata::new(metadata(1), metadata(2)),
            4,
            4,
            1,
            DevicePointer::new(0x1000).unwrap(),
            guard,
        )
        .unwrap();

        assert_eq!(frame.image_size(), 24);
        assert_eq!(frame.total_size(), 48);
        assert_eq!(frame.left_device_ptr().address(), 0x1000);
        assert_eq!(frame.right_device_ptr().unwrap().address(), 0x1000 + 24);
    }
}
