use color_eyre::{Result, eyre::bail};
use encoded_frame_decoder::{DecodedFrame, OutputPixelFormat};

pub(crate) struct FrameMetadata {
    pub(crate) frame_identifier: u32,
    pub(crate) timestamp_ns: u64,
    pub(crate) presentation_timestamp_us: u64,
}

pub(crate) struct ViewerFrame {
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) frame_identifier: u32,
    pub(crate) timestamp_ns: u64,
    pub(crate) presentation_timestamp_us: u64,
    pub(crate) rgb: Vec<u8>,
}

impl ViewerFrame {
    pub(crate) fn from_decoded(metadata: FrameMetadata, decoded: DecodedFrame) -> Result<Self> {
        if decoded.pixel_format != OutputPixelFormat::Rgb24 {
            bail!(
                "decoded frame format is not RGB24: {:?}",
                decoded.pixel_format
            );
        }

        Ok(Self {
            width: decoded.width,
            height: decoded.height,
            frame_identifier: metadata.frame_identifier,
            timestamp_ns: metadata.timestamp_ns,
            presentation_timestamp_us: metadata.presentation_timestamp_us,
            rgb: decoded.data.contiguous().into_owned(),
        })
    }
}

pub(crate) fn expected_rgb_len(width: u32, height: u32) -> Option<usize> {
    (width as usize)
        .checked_mul(height as usize)?
        .checked_mul(3)
}
