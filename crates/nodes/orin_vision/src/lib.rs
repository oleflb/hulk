use std::{boxed::Box, future::Future, pin::Pin, sync::Arc, time::Duration};

use color_eyre::Result;
use ros_z::prelude::*;

#[derive(Clone, Debug)]
pub struct OrinVisionConfig {
    pub stereo_timestamp_tolerance: Duration,
    pub max_stereo_queue_len: usize,
}

impl Default for OrinVisionConfig {
    fn default() -> Self {
        Self {
            stereo_timestamp_tolerance: Duration::from_millis(2),
            max_stereo_queue_len: 2,
        }
    }
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx, OrinVisionConfig::default()))
}

#[cfg(not(feature = "orin-gst-cuda"))]
pub async fn run(_ctx: Arc<Context>, _config: OrinVisionConfig) -> Result<()> {
    color_eyre::eyre::bail!("orin_vision_node was built without the `orin-gst-cuda` feature")
}

#[cfg(feature = "orin-gst-cuda")]
pub async fn run(ctx: Arc<Context>, config: OrinVisionConfig) -> Result<()> {
    imp::run(ctx, config).await
}

#[cfg(feature = "orin-gst-cuda")]
mod imp {
    use std::{ffi::c_void, sync::Arc, time::Duration};

    use color_eyre::{Result, eyre::WrapErr};
    use cudarc::driver::{CudaContext, CudaStream};
    use detection::{
        TimedYoloDetections, create_yolo_session_with_compute_stream,
        postprocess_yolo_outputs_with_timing,
    };
    use encoded_frame_decoder::{
        device_stereo::DeviceStereoNv12,
        orin::{OrinStereoDecoder, OrinStereoDecoderConfig},
    };
    use ort::{
        inputs,
        memory::{AllocationDevice, AllocatorType, MemoryInfo, MemoryType},
        session::Session,
        tensor::Shape,
        value::TensorRefMut,
    };
    use ros_z::{prelude::*, qos::QosHistory};
    use ros_z_streams::{AnnouncingPublisher, CreateAnnouncingPublisher};
    use tokio::time::Instant;
    use types::{
        encoded_frame::EncodedFrame,
        object_detection::YOLOObjectLabel,
        object_detection::{Object, RobocupObjectLabel},
        parameters::DetectionParameters,
        pose_detection::Pose,
        time_wrapper::TimeWrapper,
    };

    use crate::OrinVisionConfig;

    pub async fn run(ctx: std::sync::Arc<Context>, config: OrinVisionConfig) -> Result<()> {
        let node = ctx.create_node("orin_vision").build().await?;
        let left_sub = node
            .subscriber::<TimeWrapper<EncodedFrame>>("inputs/left_encoded_frame")?
            .qos(encoded_input_qos())
            .build()
            .await?;
        let right_sub = node
            .subscriber::<TimeWrapper<EncodedFrame>>("inputs/right_encoded_frame")?
            .qos(encoded_input_qos())
            .build()
            .await?;
        let node_parameters = node.bind_parameter_as::<DetectionParameters>("detection")?;
        let inference_duration_pub = node
            .publisher::<Duration>("inference_duration")?
            .build()
            .await?;
        let post_processing_duration_pub = node
            .publisher::<Duration>("post_processing_duration")?
            .build()
            .await?;
        let non_maximum_suppression_duration_pub = node
            .publisher::<Duration>("non_maximum_suppression_duration")?
            .build()
            .await?;
        let detected_objects_pub = node
            .announcing_publisher::<Vec<Object<RobocupObjectLabel>>>("detected_objects")
            .await?;
        let detected_poses_pub = node
            .announcing_publisher::<Vec<Pose<YOLOObjectLabel>>>("detected_poses")
            .await?;

        let cuda_context = CudaContext::new(0).wrap_err("open CUDA device 0")?;
        let cuda_stream = cuda_context.default_stream();
        let mut decoder = StereoDecoderState::Uninitialized { config };
        let initial_parameters_snapshot = node_parameters.snapshot();
        let mut yolo = DeviceYoloRunner::new(
            initial_parameters_snapshot.typed(),
            cuda_context.clone(),
            cuda_stream.clone(),
        )?;

        loop {
            tokio::select! {
                left = left_sub.recv() => {
                    let left = left?;
                    let parameters_snapshot = node_parameters.snapshot();
                    let parameters = parameters_snapshot.typed().clone();
                    if !parameters.enable {
                        continue;
                    }
                    if let Some(stereo) = ensure_decoder(&mut decoder, &left, &cuda_context, &cuda_stream)?.push_left(left)? {
                        process_stereo(
                            stereo,
                            &mut yolo,
                            &parameters,
                            &inference_duration_pub,
                            &post_processing_duration_pub,
                            &non_maximum_suppression_duration_pub,
                            &detected_objects_pub,
                            &detected_poses_pub,
                        ).await?;
                    }
                }
                right = right_sub.recv() => {
                    let right = right?;
                    let parameters_snapshot = node_parameters.snapshot();
                    let parameters = parameters_snapshot.typed().clone();
                    if !parameters.enable {
                        continue;
                    }
                    if let Some(stereo) = ensure_decoder(&mut decoder, &right, &cuda_context, &cuda_stream)?.push_right(right)? {
                        process_stereo(
                            stereo,
                            &mut yolo,
                            &parameters,
                            &inference_duration_pub,
                            &post_processing_duration_pub,
                            &non_maximum_suppression_duration_pub,
                            &detected_objects_pub,
                            &detected_poses_pub,
                        ).await?;
                    }
                }
            }
        }
    }

    async fn process_stereo(
        stereo: DeviceStereoNv12,
        yolo: &mut DeviceYoloRunner,
        parameters: &DetectionParameters,
        inference_duration_pub: &Publisher<Duration>,
        post_processing_duration_pub: &Publisher<Duration>,
        non_maximum_suppression_duration_pub: &Publisher<Duration>,
        detected_objects_pub: &AnnouncingPublisher<Vec<Object<RobocupObjectLabel>>>,
        detected_poses_pub: &AnnouncingPublisher<Vec<Pose<YOLOObjectLabel>>>,
    ) -> Result<()> {
        if !parameters.enable {
            return Ok(());
        }

        let image_time = stereo.metadata().left.time;
        let detected_objects_pending = detected_objects_pub.announce(image_time).await?;
        let detected_poses_pending = detected_poses_pub.announce(image_time).await?;

        let inference_start = Instant::now();
        let detections = yolo.run(&stereo, parameters)?;
        let inference_duration = inference_start.elapsed();

        inference_duration_pub.publish(&inference_duration).await?;
        post_processing_duration_pub
            .publish(&detections.post_processing_duration)
            .await?;
        non_maximum_suppression_duration_pub
            .publish(&detections.non_maximum_suppression_duration)
            .await?;
        detected_objects_pending
            .publish(&detections.detections.objects)
            .await?;
        detected_poses_pending
            .publish(&detections.detections.poses)
            .await?;

        Ok(())
    }

    fn encoded_input_qos() -> QosProfile {
        QosProfile {
            history: QosHistory::from_depth(1),
            ..Default::default()
        }
    }

    enum StereoDecoderState {
        Uninitialized {
            config: OrinVisionConfig,
        },
        Ready {
            width: u32,
            height: u32,
            config: OrinVisionConfig,
            decoder: OrinStereoDecoder,
        },
    }

    fn ensure_decoder<'a>(
        state: &'a mut StereoDecoderState,
        frame: &TimeWrapper<EncodedFrame>,
        cuda_context: &Arc<CudaContext>,
        cuda_stream: &Arc<CudaStream>,
    ) -> Result<&'a mut OrinStereoDecoder> {
        let needs_recreate = match state {
            StereoDecoderState::Uninitialized { .. } => true,
            StereoDecoderState::Ready { width, height, .. } => {
                *width != frame.inner.width || *height != frame.inner.height
            }
        };

        if needs_recreate {
            let config = match state {
                StereoDecoderState::Uninitialized { config } => config.clone(),
                StereoDecoderState::Ready { config, .. } => config.clone(),
            };
            let decoder_config =
                OrinStereoDecoderConfig::new(frame.inner.width, frame.inner.height)
                    .with_timestamp_tolerance(config.stereo_timestamp_tolerance)
                    .with_max_queue_len(config.max_stereo_queue_len);
            *state = StereoDecoderState::Ready {
                width: frame.inner.width,
                height: frame.inner.height,
                config,
                decoder: OrinStereoDecoder::new_with_cuda(
                    decoder_config,
                    cuda_context.clone(),
                    cuda_stream.clone(),
                )
                .wrap_err("create Orin stereo HEVC decoder")?,
            };
        }

        match state {
            StereoDecoderState::Ready { decoder, .. } => Ok(decoder),
            StereoDecoderState::Uninitialized { .. } => {
                unreachable!("decoder was initialized above")
            }
        }
    }

    struct DeviceYoloRunner {
        session: StreamBoundSession,
    }

    impl DeviceYoloRunner {
        fn new(
            parameters: &DetectionParameters,
            cuda_context: Arc<CudaContext>,
            cuda_stream: Arc<CudaStream>,
        ) -> Result<Self> {
            Ok(Self {
                session: StreamBoundSession::new_yolo(parameters, cuda_context, cuda_stream)?,
            })
        }

        fn run(
            &mut self,
            stereo: &DeviceStereoNv12,
            parameters: &DetectionParameters,
        ) -> Result<TimedYoloDetections> {
            check_stereo(stereo)?;

            let cuda_memory_info = MemoryInfo::new(
                AllocationDevice::CUDA,
                0,
                AllocatorType::Device,
                MemoryType::Default,
            )?;
            let input = left_nv12_input_tensor(stereo, cuda_memory_info)?;
            let outputs = self
                .session
                .session_mut()
                .run(inputs!["raw_bytes_input" => input])?;
            postprocess_yolo_outputs_with_timing(&outputs, parameters)
        }
    }

    struct StreamBoundSession {
        // This field is intentionally kept alive until after `session` is
        // dropped. ONNX Runtime stores `cuda_stream.cu_stream()` as a raw pointer in the
        // CUDA/TensorRT execution provider options.
        session: Option<Session>,
        _cuda_stream: Arc<CudaStream>,
        _cuda_context: Arc<CudaContext>,
    }

    impl StreamBoundSession {
        fn new_yolo(
            parameters: &DetectionParameters,
            cuda_context: Arc<CudaContext>,
            cuda_stream: Arc<CudaStream>,
        ) -> Result<Self> {
            let compute_stream = cuda_stream.cu_stream() as *mut ();
            let session = unsafe {
                create_yolo_session_with_compute_stream(parameters, compute_stream)
                    .wrap_err("create YOLO session on decoder CUDA stream")?
            };

            Ok(Self {
                session: Some(session),
                _cuda_stream: cuda_stream,
                _cuda_context: cuda_context,
            })
        }

        fn session_mut(&mut self) -> &mut Session {
            self.session
                .as_mut()
                .expect("stream-bound ORT session was dropped")
        }
    }

    impl Drop for StreamBoundSession {
        fn drop(&mut self) {
            // Drop the ORT session while the CUDA stream/context Arcs are still
            // alive. This upholds the `with_compute_stream` contract even if the
            // struct field order is changed later.
            drop(self.session.take());
        }
    }

    fn left_nv12_input_tensor<'frame>(
        stereo: &'frame DeviceStereoNv12,
        cuda_memory_info: MemoryInfo,
    ) -> Result<TensorRefMut<'frame, u8>> {
        let shape = Shape::new([
            i64::from(stereo.height() / 2),
            i64::from(stereo.width() / 2),
            6,
        ]);
        let data = stereo.left_device_ptr().address() as *mut c_void;

        // Safety: `DeviceStereoNv12` owns an Arc guard for the CUDA allocation,
        // and this function ties the returned ORT tensor lifetime to the
        // borrowed frame. The tensor is only used synchronously inside
        // `Session::run`, so ONNX Runtime cannot outlive the guarded allocation.
        // The session is configured to execute on the same CUDA stream that
        // filled the buffer, preserving write-before-read ordering without an
        // intermediate D2D copy.
        unsafe { TensorRefMut::from_raw(cuda_memory_info, data, shape) }
            .wrap_err("create ORT input tensor view over decoded CUDA NV12 image")
    }

    fn check_stereo(stereo: &DeviceStereoNv12) -> Result<()> {
        if !stereo.width().is_multiple_of(32) || !stereo.height().is_multiple_of(32) {
            color_eyre::eyre::bail!(
                "image dimensions must be multiples of 32 (got {}x{})",
                stereo.width(),
                stereo.height()
            );
        }
        Ok(())
    }
}
