use std::time::Duration;

#[derive(Clone, Debug)]
pub struct OrinStereoDecoderConfig {
    pub width: u32,
    pub height: u32,
    pub timestamp_tolerance: Duration,
    pub max_queue_len: usize,
}

impl OrinStereoDecoderConfig {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            timestamp_tolerance: Duration::from_millis(2),
            max_queue_len: 2,
        }
    }

    pub fn with_timestamp_tolerance(mut self, tolerance: Duration) -> Self {
        self.timestamp_tolerance = tolerance;
        self
    }

    pub fn with_max_queue_len(mut self, max_queue_len: usize) -> Self {
        self.max_queue_len = max_queue_len;
        self
    }
}

#[cfg(feature = "orin-gst-cuda")]
mod imp {
    use std::{
        collections::VecDeque,
        ffi::c_void,
        ptr,
        sync::{Arc, Mutex},
    };

    use color_eyre::{Result, eyre::WrapErr};
    use cudarc::driver::{CudaContext, CudaSlice, CudaStream, DevicePtrMut, sys as cuda_sys};
    use gstreamer as gst;
    use gstreamer::prelude::*;
    use gstreamer_app::{AppSink, AppSrc};
    use libloading::Library;
    use types::{encoded_frame::EncodedFrame, time_wrapper::TimeWrapper};

    use crate::{
        device_stereo::{
            DevicePointer, DeviceStereoNv12, FrameMetadata, StereoPair, StereoTimestampMatcher,
            TRIPLE_BUFFER_COUNT,
        },
        orin::OrinStereoDecoderConfig,
    };

    #[allow(
        dead_code,
        non_camel_case_types,
        non_snake_case,
        non_upper_case_globals
    )]
    mod jetson {
        include!(concat!(env!("OUT_DIR"), "/orin_jetson_bindings.rs"));
    }

    const CUDA_GRAPHICS_MAP_RESOURCE_FLAGS_READ_ONLY: u32 = 1;
    const MAX_PENDING_DECODER_METADATA: usize = 256;

    pub struct OrinStereoDecoder {
        config: OrinStereoDecoderConfig,
        left: GstHevcDecoder,
        right: GstHevcDecoder,
        matcher: StereoTimestampMatcher<DecodedNvmmFrame>,
        cuda: Arc<CudaContext>,
        stream: Arc<CudaStream>,
        buffers: Vec<Arc<Mutex<Option<CudaSlice<u8>>>>>,
        next_buffer: usize,
        jetson: JetsonInterop,
    }

    impl OrinStereoDecoder {
        pub fn new(config: OrinStereoDecoderConfig) -> Result<Self> {
            let cuda = CudaContext::new(0).wrap_err("open CUDA device 0")?;
            let stream = cuda.default_stream();
            Self::new_with_cuda(config, cuda, stream)
        }

        pub fn new_with_cuda(
            config: OrinStereoDecoderConfig,
            cuda: Arc<CudaContext>,
            stream: Arc<CudaStream>,
        ) -> Result<Self> {
            gst::init().wrap_err("initialize GStreamer")?;
            let image_size = crate::device_stereo::nv12_image_size(config.width, config.height)?;
            let total_size = image_size * 2;
            let mut buffers = Vec::with_capacity(TRIPLE_BUFFER_COUNT);
            for _ in 0..TRIPLE_BUFFER_COUNT {
                buffers.push(Arc::new(Mutex::new(Some(
                    stream
                        .alloc_zeros::<u8>(total_size)
                        .wrap_err("allocate stereo CUDA buffer")?,
                ))));
            }

            Ok(Self {
                matcher: StereoTimestampMatcher::new(
                    config.timestamp_tolerance.as_nanos().min(u64::MAX as u128) as u64,
                    config.max_queue_len,
                ),
                left: GstHevcDecoder::new("left")?,
                right: GstHevcDecoder::new("right")?,
                config,
                cuda,
                stream,
                buffers,
                next_buffer: 0,
                jetson: JetsonInterop::load()?,
            })
        }

        pub fn push_left(
            &mut self,
            frame: TimeWrapper<EncodedFrame>,
        ) -> Result<Option<DeviceStereoNv12>> {
            let metadata = FrameMetadata::from_encoded_frame(&frame);
            self.left.push(&frame.inner, metadata)?;
            let decoded = self.left.try_pull()?;
            self.push_decoded_left(decoded)
        }

        pub fn push_right(
            &mut self,
            frame: TimeWrapper<EncodedFrame>,
        ) -> Result<Option<DeviceStereoNv12>> {
            let metadata = FrameMetadata::from_encoded_frame(&frame);
            self.right.push(&frame.inner, metadata)?;
            let decoded = self.right.try_pull()?;
            self.push_decoded_right(decoded)
        }

        fn push_decoded_left(
            &mut self,
            decoded: Option<DecodedNvmmFrame>,
        ) -> Result<Option<DeviceStereoNv12>> {
            let Some(decoded) = decoded else {
                return Ok(None);
            };
            let Some(pair) = self.matcher.push_left(decoded.metadata, decoded) else {
                return Ok(None);
            };
            self.pack_pair(pair).map(Some)
        }

        fn push_decoded_right(
            &mut self,
            decoded: Option<DecodedNvmmFrame>,
        ) -> Result<Option<DeviceStereoNv12>> {
            let Some(decoded) = decoded else {
                return Ok(None);
            };
            let Some(pair) = self.matcher.push_right(decoded.metadata, decoded) else {
                return Ok(None);
            };
            self.pack_pair(pair).map(Some)
        }

        fn pack_pair(&mut self, pair: StereoPair<DecodedNvmmFrame>) -> Result<DeviceStereoNv12> {
            let buffer_index = self.next_buffer;
            self.next_buffer = (self.next_buffer + 1) % self.buffers.len();
            let slot = self.buffers[buffer_index].clone();
            let mut buffer = CheckedOutCudaBuffer::take(slot, buffer_index)?;

            // Keep Jetson-specific EGL/NvBufSurface interop isolated so portable
            // builds do not depend on the Orin sysroot or CUDA runtime libraries.
            self.jetson.pack_nvmm_pair_into_stereo_cuda_buffer(
                &pair.left,
                &pair.right,
                buffer.as_mut(),
                &self.stream,
                self.config.width,
                self.config.height,
            )?;

            let address = {
                let (device_ptr, sync_on_drop) = buffer.as_mut().device_ptr_mut(&self.stream);
                let address = device_ptr as usize;
                drop(sync_on_drop);
                address
            };
            let device_ptr = DevicePointer::new(address)?;

            let guard = buffer.into_guard(self.cuda.clone());
            Ok(DeviceStereoNv12::new(
                pair.metadata,
                self.config.width,
                self.config.height,
                buffer_index,
                device_ptr,
                guard,
            )?
            .with_stream_id(self.stream.cu_stream() as usize))
        }
    }

    struct CheckedOutCudaBuffer {
        slot: Arc<Mutex<Option<CudaSlice<u8>>>>,
        buffer: Option<CudaSlice<u8>>,
    }

    impl CheckedOutCudaBuffer {
        fn take(slot: Arc<Mutex<Option<CudaSlice<u8>>>>, buffer_index: usize) -> Result<Self> {
            let buffer = slot
                .lock()
                .map_err(|_| color_eyre::eyre::eyre!("stereo CUDA buffer slot lock poisoned"))?
                .take()
                .ok_or_else(|| {
                    color_eyre::eyre::eyre!("stereo CUDA buffer {buffer_index} is still in use")
                })?;
            Ok(Self {
                slot,
                buffer: Some(buffer),
            })
        }

        fn as_mut(&mut self) -> &mut CudaSlice<u8> {
            self.buffer
                .as_mut()
                .expect("checked-out CUDA buffer must be present")
        }

        fn into_guard(mut self, cuda: Arc<CudaContext>) -> Arc<DeviceBufferGuardImpl> {
            Arc::new(DeviceBufferGuardImpl {
                _cuda: cuda,
                slot: self.slot.clone(),
                buffer: Mutex::new(self.buffer.take()),
            })
        }
    }

    impl Drop for CheckedOutCudaBuffer {
        fn drop(&mut self) {
            let Some(buffer) = self.buffer.take() else {
                return;
            };
            if let Ok(mut slot) = self.slot.lock() {
                *slot = Some(buffer);
            }
        }
    }

    struct DeviceBufferGuardImpl {
        _cuda: Arc<CudaContext>,
        slot: Arc<Mutex<Option<CudaSlice<u8>>>>,
        buffer: Mutex<Option<CudaSlice<u8>>>,
    }

    impl Drop for DeviceBufferGuardImpl {
        fn drop(&mut self) {
            let Ok(buffer) = self.buffer.get_mut() else {
                return;
            };
            let Some(buffer) = buffer.take() else {
                return;
            };
            if let Ok(mut slot) = self.slot.lock() {
                *slot = Some(buffer);
            }
        }
    }

    struct GstHevcDecoder {
        name: String,
        appsrc: AppSrc,
        appsink: AppSink,
        bus: gst::Bus,
        pipeline: gst::Pipeline,
        pending_metadata: VecDeque<FrameMetadata>,
    }

    impl GstHevcDecoder {
        fn new(name: &str) -> Result<Self> {
            let pipeline = gst::parse::launch(&format!(
                "appsrc name=src is-live=true do-timestamp=false format=time block=false \
                 ! h265parse disable-passthrough=true \
                 ! nvv4l2decoder enable-max-performance=true disable-dpb=true \
                 ! video/x-raw(memory:NVMM),format=NV12 \
                 ! appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
            ))
            .wrap_err_with(|| format!("create {name} GStreamer pipeline"))?
            .downcast::<gst::Pipeline>()
            .map_err(|_| color_eyre::eyre::eyre!("{name} GStreamer element is not a pipeline"))?;

            let appsrc = pipeline
                .by_name("src")
                .ok_or_else(|| color_eyre::eyre::eyre!("{name} appsrc not found"))?
                .downcast::<AppSrc>()
                .map_err(|_| color_eyre::eyre::eyre!("{name} src is not appsrc"))?;
            appsrc.set_caps(Some(
                &gst::Caps::builder("video/x-h265")
                    .field("stream-format", "byte-stream")
                    .field("alignment", "au")
                    .build(),
            ));
            appsrc.set_format(gst::Format::Time);
            appsrc.set_is_live(true);
            appsrc.set_block(false);

            let appsink = pipeline
                .by_name("sink")
                .ok_or_else(|| color_eyre::eyre::eyre!("{name} appsink not found"))?
                .downcast::<AppSink>()
                .map_err(|_| color_eyre::eyre::eyre!("{name} sink is not appsink"))?;

            pipeline
                .set_state(gst::State::Playing)
                .wrap_err_with(|| format!("start {name} GStreamer decoder"))?;
            let bus = pipeline
                .bus()
                .ok_or_else(|| color_eyre::eyre::eyre!("{name} GStreamer pipeline has no bus"))?;

            Ok(Self {
                name: name.to_string(),
                appsrc,
                appsink,
                bus,
                pipeline,
                pending_metadata: VecDeque::new(),
            })
        }

        fn push(&mut self, frame: &EncodedFrame, metadata: FrameMetadata) -> Result<()> {
            self.check_bus()?;
            let mut buffer =
                gst::Buffer::with_size(frame.data.len()).wrap_err("allocate HEVC GstBuffer")?;
            {
                let buffer_mut = buffer
                    .get_mut()
                    .ok_or_else(|| color_eyre::eyre::eyre!("GstBuffer unexpectedly shared"))?;
                {
                    let mut map = buffer_mut.map_writable().wrap_err("map HEVC GstBuffer")?;
                    map.as_mut_slice().copy_from_slice(&frame.data);
                }
                buffer_mut.set_pts(gst::ClockTime::from_useconds(
                    frame.presentation_timestamp_us,
                ));
            }
            self.appsrc.push_buffer(buffer).map_err(|err| {
                color_eyre::eyre::eyre!("push HEVC access unit into GStreamer: {err:?}")
            })?;
            self.pending_metadata.push_back(metadata);
            while self.pending_metadata.len() > MAX_PENDING_DECODER_METADATA {
                self.pending_metadata.pop_front();
            }
            self.check_bus()?;
            Ok(())
        }

        fn try_pull(&mut self) -> Result<Option<DecodedNvmmFrame>> {
            self.check_bus()?;
            let sample = self.appsink.try_pull_sample(gst::ClockTime::ZERO);
            self.check_bus()?;
            let Some(sample) = sample else {
                return Ok(None);
            };
            let buffer = sample
                .buffer_owned()
                .ok_or_else(|| color_eyre::eyre::eyre!("decoded sample has no buffer"))?;
            let pts = buffer.pts().ok_or_else(|| {
                color_eyre::eyre::eyre!("{} decoded sample has no PTS", self.name)
            })?;
            let metadata = self.take_metadata_for_pts(pts.useconds())?;
            Ok(Some(DecodedNvmmFrame { metadata, buffer }))
        }

        fn take_metadata_for_pts(&mut self, pts_us: u64) -> Result<FrameMetadata> {
            let index = self
                .pending_metadata
                .iter()
                .position(|metadata| metadata.presentation_timestamp_us == pts_us)
                .ok_or_else(|| {
                    color_eyre::eyre::eyre!(
                        "{} decoded sample PTS {pts_us} us has no queued metadata",
                        self.name
                    )
                })?;
            if index > 0 {
                self.pending_metadata.drain(..index);
            }
            self.pending_metadata.pop_front().ok_or_else(|| {
                color_eyre::eyre::eyre!(
                    "{} decoded sample PTS {pts_us} us metadata queue was empty",
                    self.name
                )
            })
        }

        fn check_bus(&self) -> Result<()> {
            while let Some(message) = self.bus.timed_pop(gst::ClockTime::ZERO) {
                match message.view() {
                    gst::MessageView::Error(error) => {
                        let source = error
                            .src()
                            .map(|source| source.path_string())
                            .unwrap_or_else(|| "unknown".into());
                        let debug = error.debug().unwrap_or_else(|| "no debug info".into());
                        color_eyre::eyre::bail!(
                            "{} GStreamer pipeline error from {source}: {} ({debug})",
                            self.name,
                            error.error()
                        );
                    }
                    gst::MessageView::Eos(_) => {
                        color_eyre::eyre::bail!("{} GStreamer pipeline reached EOS", self.name);
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    impl Drop for GstHevcDecoder {
        fn drop(&mut self) {
            let _ = self.pipeline.set_state(gst::State::Null);
        }
    }

    struct DecodedNvmmFrame {
        metadata: FrameMetadata,
        buffer: gst::Buffer,
    }

    struct JetsonInterop {
        _nvbufsurface: Library,
        _cuda: Library,
        map_egl_image: NvBufSurfaceMapEglImage,
        unmap_egl_image: NvBufSurfaceUnMapEglImage,
        register_egl_image: CuGraphicsEglRegisterImage,
        get_mapped_egl_frame: CuGraphicsResourceGetMappedEglFrame,
    }

    type NvBufSurfaceMapEglImage = unsafe extern "C" fn(*mut jetson::NvBufSurface, i32) -> i32;
    type NvBufSurfaceUnMapEglImage = unsafe extern "C" fn(*mut jetson::NvBufSurface, i32) -> i32;
    type CuGraphicsEglRegisterImage = unsafe extern "C" fn(
        *mut cuda_sys::CUgraphicsResource,
        *mut c_void,
        u32,
    ) -> cuda_sys::CUresult;
    type CuGraphicsResourceGetMappedEglFrame = unsafe extern "C" fn(
        *mut jetson::CUeglFrame,
        cuda_sys::CUgraphicsResource,
        u32,
        u32,
    ) -> cuda_sys::CUresult;

    impl JetsonInterop {
        fn load() -> Result<Self> {
            let nvbufsurface = load_library(&[
                "libnvbufsurface.so.1.0.0",
                "libnvbufsurface.so.1",
                "libnvbufsurface.so",
            ])?;
            let cuda = load_library(&["libcuda.so.1", "libcuda.so"])?;

            let map_egl_image = unsafe { *nvbufsurface.get(b"NvBufSurfaceMapEglImage\0")? };
            let unmap_egl_image = unsafe { *nvbufsurface.get(b"NvBufSurfaceUnMapEglImage\0")? };
            let register_egl_image = unsafe { *cuda.get(b"cuGraphicsEGLRegisterImage\0")? };
            let get_mapped_egl_frame =
                unsafe { *cuda.get(b"cuGraphicsResourceGetMappedEglFrame\0")? };

            Ok(Self {
                _nvbufsurface: nvbufsurface,
                _cuda: cuda,
                map_egl_image,
                unmap_egl_image,
                register_egl_image,
                get_mapped_egl_frame,
            })
        }

        fn pack_nvmm_pair_into_stereo_cuda_buffer(
            &self,
            left: &DecodedNvmmFrame,
            right: &DecodedNvmmFrame,
            dst: &mut CudaSlice<u8>,
            stream: &CudaStream,
            width: u32,
            height: u32,
        ) -> Result<()> {
            let (dst_ptr, sync_on_drop) = dst.device_ptr_mut(stream);
            let image_size = crate::device_stereo::nv12_image_size(width, height)?;
            let right_dst = device_ptr_offset(dst_ptr, image_size)?;
            unsafe {
                self.copy_nvmm_frame_to_device(left, dst_ptr, stream, width, height)?;
                self.copy_nvmm_frame_to_device(right, right_dst, stream, width, height)?;
            }
            drop(sync_on_drop);
            Ok(())
        }

        unsafe fn copy_nvmm_frame_to_device(
            &self,
            frame: &DecodedNvmmFrame,
            dst: cuda_sys::CUdeviceptr,
            stream: &CudaStream,
            width: u32,
            height: u32,
        ) -> Result<()> {
            let map = frame
                .buffer
                .map_readable()
                .wrap_err("map decoded NVMM GstBuffer")?;
            let surface = map.as_slice().as_ptr() as *mut jetson::NvBufSurface;
            if surface.is_null() {
                color_eyre::eyre::bail!("decoded NVMM GstBuffer did not contain NvBufSurface");
            }
            unsafe { validate_nvbuf_surface(surface, width, height)? };

            let map_result = unsafe { (self.map_egl_image)(surface, 0) };
            if map_result != 0 {
                color_eyre::eyre::bail!("NvBufSurfaceMapEglImage failed with {map_result}");
            }

            let copy_result =
                unsafe { self.copy_mapped_surface_to_device(surface, dst, stream, width, height) };
            let unmap_result = unsafe { (self.unmap_egl_image)(surface, 0) };
            if unmap_result != 0 && copy_result.is_ok() {
                color_eyre::eyre::bail!("NvBufSurfaceUnMapEglImage failed with {unmap_result}");
            }
            copy_result
        }

        unsafe fn copy_mapped_surface_to_device(
            &self,
            surface: *mut jetson::NvBufSurface,
            dst: cuda_sys::CUdeviceptr,
            stream: &CudaStream,
            width: u32,
            height: u32,
        ) -> Result<()> {
            let params = unsafe { validate_nvbuf_surface(surface, width, height)? };
            let egl_image = unsafe { (*params).mappedAddr.eglImage } as *mut c_void;
            if egl_image.is_null() {
                color_eyre::eyre::bail!("NvBufSurface EGL image is null after mapping");
            }

            let mut resource = ptr::null_mut();
            unsafe {
                (self.register_egl_image)(
                    &mut resource,
                    egl_image,
                    CUDA_GRAPHICS_MAP_RESOURCE_FLAGS_READ_ONLY,
                )
                .result()
                .wrap_err("register NvBufSurface EGL image with CUDA")?;
            }
            if resource.is_null() {
                color_eyre::eyre::bail!("CUDA EGL registration returned a null resource");
            }

            let copy_result = unsafe {
                let mut egl_frame = std::mem::MaybeUninit::<jetson::CUeglFrame>::uninit();
                let mapped_result =
                    (self.get_mapped_egl_frame)(egl_frame.as_mut_ptr(), resource, 0, 0)
                        .result()
                        .wrap_err("get mapped CUDA EGL frame");
                match mapped_result {
                    Ok(()) => copy_egl_frame_nv12_to_device(
                        egl_frame.assume_init(),
                        dst,
                        stream,
                        width,
                        height,
                    ),
                    Err(error) => Err(error),
                }
            };

            let unregister_result =
                unsafe { cuda_sys::cuGraphicsUnregisterResource(resource).result() }
                    .wrap_err("unregister CUDA EGL resource");
            copy_result.and(unregister_result)
        }
    }

    fn load_library(names: &[&str]) -> Result<Library> {
        let mut errors = Vec::new();
        for name in names {
            match unsafe { Library::new(*name) } {
                Ok(library) => return Ok(library),
                Err(error) => errors.push(format!("{name}: {error}")),
            }
        }
        color_eyre::eyre::bail!("failed to load Jetson library: {}", errors.join(", "))
    }

    unsafe fn validate_nvbuf_surface(
        surface: *mut jetson::NvBufSurface,
        width: u32,
        height: u32,
    ) -> Result<*mut jetson::NvBufSurfaceParams> {
        if surface.is_null() {
            color_eyre::eyre::bail!("NvBufSurface is null");
        }
        if unsafe { (*surface).batchSize } == 0 || unsafe { (*surface).numFilled } == 0 {
            color_eyre::eyre::bail!("NvBufSurface has no filled batch entry");
        }

        let params = unsafe { (*surface).surfaceList };
        if params.is_null() {
            color_eyre::eyre::bail!("NvBufSurface surfaceList is null");
        }

        let actual_width = unsafe { (*params).width };
        let actual_height = unsafe { (*params).height };
        if actual_width != width || actual_height != height {
            color_eyre::eyre::bail!(
                "NvBufSurface dimensions {actual_width}x{actual_height} do not match expected {width}x{height}"
            );
        }
        Ok(params)
    }

    fn device_ptr_offset(
        ptr: cuda_sys::CUdeviceptr,
        offset: usize,
    ) -> Result<cuda_sys::CUdeviceptr> {
        let offset = cuda_sys::CUdeviceptr::try_from(offset)
            .map_err(|_| color_eyre::eyre::eyre!("CUDA device pointer offset is too large"))?;
        ptr.checked_add(offset)
            .ok_or_else(|| color_eyre::eyre::eyre!("CUDA device pointer offset overflow"))
    }

    unsafe fn copy_egl_frame_nv12_to_device(
        egl_frame: jetson::CUeglFrame,
        dst: cuda_sys::CUdeviceptr,
        stream: &CudaStream,
        width: u32,
        height: u32,
    ) -> Result<()> {
        if egl_frame.frameType as u32 != 1 {
            color_eyre::eyre::bail!(
                "CUDA EGL frame is not pitch-backed NV12 (frameType={})",
                egl_frame.frameType as u32
            );
        }
        if egl_frame.planeCount < 2 {
            color_eyre::eyre::bail!(
                "CUDA EGL frame exposes {} plane(s), expected at least 2",
                egl_frame.planeCount
            );
        }
        if egl_frame.width < width || egl_frame.height < height {
            color_eyre::eyre::bail!(
                "CUDA EGL frame dimensions {}x{} are smaller than expected {width}x{height}",
                egl_frame.width,
                egl_frame.height
            );
        }

        let y_plane = unsafe { egl_frame.frame.pPitch[0] } as cuda_sys::CUdeviceptr;
        let uv_plane = unsafe { egl_frame.frame.pPitch[1] } as cuda_sys::CUdeviceptr;
        if y_plane == 0 || uv_plane == 0 {
            color_eyre::eyre::bail!("CUDA EGL frame does not expose pitched NV12 planes");
        }

        let pitch = egl_frame.pitch as usize;
        let width = width as usize;
        let height = height as usize;
        if pitch < width {
            color_eyre::eyre::bail!("CUDA EGL frame pitch {pitch} is smaller than width {width}");
        }
        let y_size = width
            .checked_mul(height)
            .ok_or_else(|| color_eyre::eyre::eyre!("NV12 luma plane size overflow"))?;
        let uv_dst = device_ptr_offset(dst, y_size)?;
        copy_2d_device_to_device(y_plane, pitch, dst, width, width, height, stream)
            .wrap_err("copy NV12 luma plane")?;
        copy_2d_device_to_device(uv_plane, pitch, uv_dst, width, width, height / 2, stream)
            .wrap_err("copy NV12 chroma plane")?;
        Ok(())
    }

    fn copy_2d_device_to_device(
        src: cuda_sys::CUdeviceptr,
        src_pitch: usize,
        dst: cuda_sys::CUdeviceptr,
        dst_pitch: usize,
        width_bytes: usize,
        height: usize,
        stream: &CudaStream,
    ) -> Result<()> {
        let copy = cuda_sys::CUDA_MEMCPY2D {
            srcXInBytes: 0,
            srcY: 0,
            srcMemoryType: cuda_sys::CUmemorytype::CU_MEMORYTYPE_DEVICE,
            srcHost: ptr::null(),
            srcDevice: src,
            srcArray: ptr::null_mut(),
            srcPitch: src_pitch,
            dstXInBytes: 0,
            dstY: 0,
            dstMemoryType: cuda_sys::CUmemorytype::CU_MEMORYTYPE_DEVICE,
            dstHost: ptr::null_mut(),
            dstDevice: dst,
            dstArray: ptr::null_mut(),
            dstPitch: dst_pitch,
            WidthInBytes: width_bytes,
            Height: height,
        };
        unsafe { cuda_sys::cuMemcpy2DAsync_v2(&copy, stream.cu_stream()).result() }
            .wrap_err("CUDA memcpy2D device-to-device")
    }
}

#[cfg(not(feature = "orin-gst-cuda"))]
mod imp {
    use color_eyre::{Result, eyre::bail};
    use types::{encoded_frame::EncodedFrame, time_wrapper::TimeWrapper};

    use crate::{device_stereo::DeviceStereoNv12, orin::OrinStereoDecoderConfig};

    pub struct OrinStereoDecoder;

    impl OrinStereoDecoder {
        pub fn new(_config: OrinStereoDecoderConfig) -> Result<Self> {
            bail!("encoded_frame_decoder was built without the `orin-gst-cuda` feature")
        }

        pub fn push_left(
            &mut self,
            _frame: TimeWrapper<EncodedFrame>,
        ) -> Result<Option<DeviceStereoNv12>> {
            bail!("encoded_frame_decoder was built without the `orin-gst-cuda` feature")
        }

        pub fn push_right(
            &mut self,
            _frame: TimeWrapper<EncodedFrame>,
        ) -> Result<Option<DeviceStereoNv12>> {
            bail!("encoded_frame_decoder was built without the `orin-gst-cuda` feature")
        }
    }
}

pub use imp::OrinStereoDecoder;
