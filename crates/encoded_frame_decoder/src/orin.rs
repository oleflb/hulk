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
    use std::sync::{Arc, Mutex};
    use std::{ffi::c_void, ptr};

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
            self.left.push(&frame.inner)?;
            let decoded = self.left.try_pull(metadata)?;
            self.push_decoded_left(decoded)
        }

        pub fn push_right(
            &mut self,
            frame: TimeWrapper<EncodedFrame>,
        ) -> Result<Option<DeviceStereoNv12>> {
            let metadata = FrameMetadata::from_encoded_frame(&frame);
            self.right.push(&frame.inner)?;
            let decoded = self.right.try_pull(metadata)?;
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
            let mut buffer = slot
                .lock()
                .map_err(|_| color_eyre::eyre::eyre!("stereo CUDA buffer slot lock poisoned"))?
                .take()
                .ok_or_else(|| {
                    color_eyre::eyre::eyre!("stereo CUDA buffer {buffer_index} is still in use")
                })?;

            // This is intentionally a hard runtime boundary: getting a CUDA pointer
            // to Jetson NVMM requires platform-specific EGL/NvBufSurface interop.
            // The safe public API and GStreamer low-latency decode path are wired;
            // the interop is isolated here so it can be completed against the Orin
            // sysroot without affecting portable builds.
            self.jetson.pack_nvmm_pair_into_stereo_cuda_buffer(
                &pair.left,
                &pair.right,
                &mut buffer,
                &self.stream,
                self.config.width,
                self.config.height,
            )?;

            let address = {
                let (device_ptr, sync_on_drop) = buffer.device_ptr_mut(&self.stream);
                let address = device_ptr as usize;
                drop(sync_on_drop);
                address
            };

            let guard = Arc::new(DeviceBufferGuardImpl {
                _cuda: self.cuda.clone(),
                slot,
                buffer: Mutex::new(Some(buffer)),
            });
            Ok(DeviceStereoNv12::new(
                pair.metadata,
                self.config.width,
                self.config.height,
                buffer_index,
                DevicePointer::new(address)?,
                guard,
            )?
            .with_stream_id(self.stream.cu_stream() as usize))
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
        appsrc: AppSrc,
        appsink: AppSink,
        _pipeline: gst::Pipeline,
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

            Ok(Self {
                appsrc,
                appsink,
                _pipeline: pipeline,
            })
        }

        fn push(&self, frame: &EncodedFrame) -> Result<()> {
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
            Ok(())
        }

        fn try_pull(&self, metadata: FrameMetadata) -> Result<Option<DecodedNvmmFrame>> {
            let sample = self.appsink.try_pull_sample(gst::ClockTime::ZERO);
            let Some(sample) = sample else {
                return Ok(None);
            };
            let buffer = sample
                .buffer_owned()
                .ok_or_else(|| color_eyre::eyre::eyre!("decoded sample has no buffer"))?;
            Ok(Some(DecodedNvmmFrame { metadata, buffer }))
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
            unsafe {
                self.copy_nvmm_frame_to_device(left, dst_ptr, stream, width, height)?;
                self.copy_nvmm_frame_to_device(
                    right,
                    dst_ptr + image_size as cuda_sys::CUdeviceptr,
                    stream,
                    width,
                    height,
                )?;
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
            let params = unsafe { (*surface).surfaceList };
            if params.is_null() {
                color_eyre::eyre::bail!("NvBufSurface surfaceList is null");
            }
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

    unsafe fn copy_egl_frame_nv12_to_device(
        egl_frame: jetson::CUeglFrame,
        dst: cuda_sys::CUdeviceptr,
        stream: &CudaStream,
        width: u32,
        height: u32,
    ) -> Result<()> {
        let y_plane = unsafe { egl_frame.frame.pPitch[0] } as cuda_sys::CUdeviceptr;
        let uv_plane = unsafe { egl_frame.frame.pPitch[1] } as cuda_sys::CUdeviceptr;
        if y_plane == 0 || uv_plane == 0 {
            color_eyre::eyre::bail!("CUDA EGL frame does not expose pitched NV12 planes");
        }

        let pitch = egl_frame.pitch as usize;
        let width = width as usize;
        let height = height as usize;
        copy_2d_device_to_device(y_plane, pitch, dst, width, width, height, stream)
            .wrap_err("copy NV12 luma plane")?;
        copy_2d_device_to_device(
            uv_plane,
            pitch,
            dst + (width * height) as cuda_sys::CUdeviceptr,
            width,
            width,
            height / 2,
            stream,
        )
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
