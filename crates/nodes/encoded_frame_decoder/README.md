# Encoded Frame Decoder

`encoded_frame_decoder_node` subscribes to the HEVC camera topics from `camera_driver`, decodes them to NV12, and publishes SHM-friendly `types::nv12_image::Nv12Image` messages for vision inference.

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `inputs/left_encoded_frame` | `types::time_wrapper::TimeWrapper<types::encoded_frame::EncodedFrame>` | Left HEVC input. |
| `inputs/right_encoded_frame` | `types::time_wrapper::TimeWrapper<types::encoded_frame::EncodedFrame>` | Right HEVC input. |
| `inputs/left_nv12_image` | `types::nv12_image::Nv12Image` | Left decoded NV12 image with `ZBuf` payload. |
| `inputs/right_nv12_image` | `types::nv12_image::Nv12Image` | Right decoded NV12 image with `ZBuf` payload. |

## Run Standalone

```bash
cargo run -p encoded_frame_decoder_node -- \
  --router <router-endpoint> \
  --namespace <robot-namespace>
```

The node defaults to `ffmpeg` on `PATH` and a 128 MiB decoded-frame SHM pool. Use `--ffmpeg-path` or `--decoded-shm-pool-size` for local debug setups.

## Integrated Runtime

`hulk_ros_z` starts this decoder automatically. The combined stack therefore also requires an `ffmpeg` binary with HEVC decode support on `PATH`, unless `hulk_ros_z --ffmpeg-path <path>` is used. The deployed runtime container installs `ffmpeg` and reserves `/dev/shm` headroom for the decoded-frame SHM pool.

The gameplay path keeps decoded output as NV12. RGB conversion remains inside the YOLO model/kernel or debug tooling.
