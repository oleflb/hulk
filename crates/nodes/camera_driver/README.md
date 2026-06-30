# Camera Driver

`camera_driver` is the ROS-Z node for the X5 SC132GS stereo camera module. It opens the two X5 VIO pipelines in SC132GS normal mode without VIN LPWM trigger generation, rectifies the camera streams with GDC, encodes them with the X5 HEVC encoder, and publishes compressed access units for transport to the robot body.

## Run

```bash
cargo run -p camera_driver -- --router <router-endpoint> --namespace <robot-namespace>
```

The X5 backend is only available for the `x5cam_x5_target` build configuration. Other targets build a stub that reports the backend as unavailable.

For target validation, build the real backend inside the X5 SDK container so the aarch64 linker, sysroot, SDK headers, and SDK libraries all match:

```bash
crates/nodes/camera_driver/build.sh
```

The build script persists Cargo registry and git caches under `$(./scripts/resolve_data_home)/container-cargo-home`, so repeated container builds do not need to redownload Cargo dependencies.

The node configures a 64 MiB ROS-Z shared-memory pool and forces encoded frame payloads through SHM. Containers running this node need at least that much available POSIX shared memory, for example with `--shm-size=128m` or an equivalent `/dev/shm` mount.

The SDK build script accepts `X5_SOURCE_DIR`, `X5_SDK_INCLUDE`, `X5_SDK_LIB_DIR`, and `X5CAM_FORCE_X5_TARGET`. The container sets the target sysroot and SDK paths; outside the container, `X5_SOURCE_DIR` defaults to `/home/ole/hulk-stuff/x5/source`.

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `inputs/left_encoded_frame` | `types::time_wrapper::TimeWrapper<types::encoded_frame::EncodedFrame>` | Left rectified HEVC access units. Published only when subscribers are present. |
| `inputs/right_encoded_frame` | `types::time_wrapper::TimeWrapper<types::encoded_frame::EncodedFrame>` | Right rectified HEVC access units. Published only when subscribers are present. |
| `inputs/camera_info` | `ros2::sensor_msgs::camera_info::CameraInfo` | Legacy raw left camera info for existing consumers. Transient-local. |
| `inputs/left_camera_info` | `ros2::sensor_msgs::camera_info::CameraInfo` | Left rectified camera info. Transient-local. |
| `inputs/right_camera_info` | `ros2::sensor_msgs::camera_info::CameraInfo` | Right rectified camera info. Transient-local. |

## Encoded Layout

The driver publishes `EncodedFrameCodec::Hevc` and one access unit per `EncodedFrame`. The production mode is `1280x1088` at `60 fps` per camera with `12_000 kbps` CBR per camera by default.

The VSE output frame is passed to MediaCodec as an external NV12 input buffer. Encoded SDK output stays leased until the ROS-Z publisher actually needs an owned payload, so the driver avoids copying raw high-resolution frames.

`EncodedFrame.timestamp_ns` uses the VSE/VIN frame timestamp reported by the SDK. `trigger_ts=<frames-with-trigger>/<frames>` remains a diagnostic counter, but normal/no-LPWM mode is expected to run without trigger timestamps. Runtime stats also print left-minus-right timestamp deltas as observability, not as a hardware-sync guarantee.

Decoded NV12, RGB, or YCbCr images are intentionally produced by downstream receivers or debug tools, not by `camera_driver`. Developer and debug runs use `encoded_frame_decoder_node` to decode HEVC into `types::nv12_image::Nv12Image` on `inputs/left_nv12_image` and `inputs/right_nv12_image`. The Orin gameplay path builds `hulk_ros_z` with `--features orin-vision`, decodes the stereo HEVC streams with GStreamer/NVMM/CUDA in `orin_vision_node`, and feeds YOLO from CUDA device memory without publishing decoded pixels through ROS-Z.

`inputs/left_camera_info` and `inputs/right_camera_info` describe the rectified GDC output. The right camera projection matrix uses `Tx = -fx * baseline`. The legacy `inputs/camera_info` topic keeps the previous raw-left calibration shape and `frame_id="x5"` for existing consumers.

## Validation

At startup, the node waits for calibration, normal/no-LPWM camera setup events, and at least 90% of the expected frames in the configured startup window. Runtime statistics print once per second.

Robot validation checklist:

1. Start the node on the X5 with enough `/dev/shm` for the 64 MiB ROS-Z SHM pool.
2. Confirm startup prints both cameras with `mode=Normal lpwm=false` and then `startup validation: ok`.
3. Confirm stats show stable 60 fps per side. `trigger_ts=0/N` is expected in normal/no-LPWM mode.
4. Confirm left-minus-right timestamp deltas are stable enough for the current unsynced camera mode.
5. Subscribe to both HEVC topics and verify both streams decode.

Unstable or sub-60 fps output usually indicates VIO/encoder backpressure, SDK setup problems, or insufficient runtime resources such as `/dev/shm` headroom.
