use std::{collections::VecDeque, path::PathBuf, time::Duration};

use color_eyre::{
    Result,
    eyre::{ensure, eyre},
};
use encoded_frame_decoder::{DecoderConfig, FfmpegHevcDecoder, hevc_access_unit_contains_irap};
use types::encoded_frame::{EncodedFrame, EncodedFrameCodec};

use crate::frame::{FrameMetadata, ViewerFrame};

pub(crate) enum DecodeOutcome {
    Decoded(ViewerFrame),
    TimedOut,
    WaitingForIrap,
}

pub(crate) struct StreamDecoder {
    ffmpeg_path: PathBuf,
    frame_timeout: Duration,
    state: Option<DecoderState>,
}

impl StreamDecoder {
    pub(crate) fn new(ffmpeg_path: PathBuf, frame_timeout: Duration) -> Self {
        Self {
            ffmpeg_path,
            frame_timeout,
            state: None,
        }
    }

    pub(crate) fn decode(&mut self, frame: &EncodedFrame) -> Result<DecodeOutcome> {
        ensure!(
            frame.codec == EncodedFrameCodec::Hevc,
            "unsupported encoded frame codec {:?}",
            frame.codec
        );

        let outcome = {
            let state = self.ensure_decoder(frame)?;
            if !state.is_synchronized {
                if !hevc_access_unit_contains_irap(&frame.data) {
                    return Ok(DecodeOutcome::WaitingForIrap);
                }
                state.is_synchronized = true;
            }

            state.pending_metadata.push_back(FrameMetadata {
                frame_identifier: frame.frame_identifier,
                timestamp_ns: frame.timestamp_ns,
                presentation_timestamp_us: frame.presentation_timestamp_us,
            });

            match state.decoder.decode_frame(frame)? {
                Some(decoded) => {
                    let metadata = state
                        .pending_metadata
                        .pop_front()
                        .ok_or_else(|| eyre!("decoded frame has no queued metadata"))?;
                    DecodeOutcome::Decoded(ViewerFrame::from_decoded(metadata, decoded)?)
                }
                None => {
                    state.pending_metadata.clear();
                    state.is_synchronized = false;
                    DecodeOutcome::TimedOut
                }
            }
        };

        if matches!(outcome, DecodeOutcome::TimedOut) {
            self.state = None;
        }

        Ok(outcome)
    }

    fn ensure_decoder(&mut self, frame: &EncodedFrame) -> Result<&mut DecoderState> {
        let recreate = self
            .state
            .as_ref()
            .is_none_or(|state| state.width != frame.width || state.height != frame.height);
        if recreate {
            let config = DecoderConfig::new(frame.width, frame.height)
                .with_rgb_output()
                .with_ffmpeg_path(self.ffmpeg_path.clone())
                .with_frame_timeout(self.frame_timeout);
            self.state = Some(DecoderState {
                width: frame.width,
                height: frame.height,
                decoder: FfmpegHevcDecoder::spawn(config)?,
                pending_metadata: VecDeque::new(),
                is_synchronized: false,
            });
        }
        Ok(self.state.as_mut().expect("decoder was just initialized"))
    }
}

struct DecoderState {
    width: u32,
    height: u32,
    decoder: FfmpegHevcDecoder,
    pending_metadata: VecDeque<FrameMetadata>,
    is_synchronized: bool,
}
