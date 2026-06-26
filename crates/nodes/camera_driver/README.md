# Camera Driver

`camera_driver` is the ROS-Z node for the X5 SC132GS stereo camera module. It opens the two X5 VIO pipelines in SC132GS `SLAVE_M` mode with VIN LPWM trigger generation, rectifies the camera streams with GDC, encodes them with the X5 HEVC encoder, and publishes compressed access units for transport to the robot body.

## Run

```bash
cargo run -p camera_driver -- --router <router-endpoint> --namespace <robot-namespace>
```

The X5 backend is only available for the `x5cam_x5_target` build configuration. Other targets build a stub that reports the backend as unavailable.

For target validation, build the real backend inside the X5 SDK container so the aarch64 linker, sysroot, SDK headers, and SDK libraries all match:

```bash
podman build -t camera-driver-x5 -f crates/nodes/camera_driver/Containerfile .

podman run --rm \
  -v "$PWD":/work \
  camera-driver-x5 \
  cargo build -p camera_driver --target aarch64-unknown-linux-gnu
```

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

`EncodedFrame.timestamp_ns` uses the VIN trigger timestamp (`trig_tv`) when the SDK reports one, and falls back to the VSE frame timestamp otherwise. Runtime stats print `trigger_ts=<frames-with-trigger>/<frames>` plus left-minus-right `sync` deltas so LPWM hardware sync can be verified on robot hardware.

Decoded NV12, RGB, or YCbCr images are intentionally produced by downstream receivers or debug tools, not by `camera_driver`. Developer and debug runs use `encoded_frame_decoder_node` to decode HEVC into `types::nv12_image::Nv12Image` on `inputs/left_nv12_image` and `inputs/right_nv12_image`. The Orin gameplay path builds `hulk_ros_z` with `--features orin-vision`, decodes the stereo HEVC streams with GStreamer/NVMM/CUDA in `orin_vision_node`, and feeds YOLO from CUDA device memory without publishing decoded pixels through ROS-Z.

`inputs/left_camera_info` and `inputs/right_camera_info` describe the rectified GDC output. The right camera projection matrix uses `Tx = -fx * baseline`. The legacy `inputs/camera_info` topic keeps the previous raw-left calibration shape and `frame_id="x5"` for existing consumers.

## Validation

At startup, the node waits for calibration, `SLAVE_M`/LPWM camera setup events, at least 90% of the expected frames, and matching VIN trigger timestamp coverage in the configured startup window. Runtime statistics print once per second.

Robot validation checklist:

1. Start the node on the X5 with enough `/dev/shm` for the 64 MiB ROS-Z SHM pool.
2. Confirm startup prints both cameras with `mode=Slave lpwm=true` and then `startup validation: ok`.
3. Confirm stats show stable 60 fps per side and `trigger_ts=N/N` rather than `0/N`.
4. Confirm `sync` deltas are stable and close to `0 ms`.
5. Subscribe to both HEVC topics and verify both streams decode.

Bad LPWM outcomes to diagnose: camera open/start failure may indicate deployed SC132GS library or wiring does not support `SLAVE_M`; `trigger_ts=0/N` means frames arrive but SDK trigger timestamps are not reported; unstable or sub-60 fps suggests LPWM period or trigger mode needs hardware-specific adjustment.
