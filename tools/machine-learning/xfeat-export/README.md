# XFeat ONNX Export

Exports XFeat and LighterGlue to fixed-contract ONNX models for later TensorRT conversion.

```bash
uv run export-xfeat-onnx models/xfeat.onnx --height 1088 --width 1280 --keypoints 512
uv run export-xfeat-onnx models/xfeat-b2.onnx --height 1088 --width 1280 --keypoints 512 --batch-size 2
uv run export-lighterglue-onnx models/lighterglue.onnx --keypoints 512
uv run export-xfeat-lighterglue-onnx models/xfeat-lighterglue.onnx --height 1088 --width 1280 --keypoints 512
```

`export-xfeat-onnx` takes NV12 input as `uint8` shaped `(height / 2, width / 2, 6)` and embeds the GPU NV12-to-RGB conversion layer from `../multi-task-yolo/src/utils/nv12_to_rgb.py` into the exported graph.
With `--batch-size`, the XFeat input is `uint8` shaped `(batch_size, height / 2, width / 2, 6)`.
It returns normalized keypoints, descriptors, scores, and valid masks. The keypoints use the LighterGlue normalization `(keypoint - [width, height] / 2) / (max(width, height) / 2)`.
`export-lighterglue-onnx` expects those normalized keypoints directly, so the exported LighterGlue model does not take image-size inputs.

`export-xfeat-lighterglue-onnx` fuses extraction and matching for visual odometry. It takes four zero-copy NV12 inputs named `previous_left`, `previous_right`, `current_left`, and `current_right`, each shaped `(height / 2, width / 2, 6)`. It returns current stereo keypoints and valid masks, current-left-to-current-right matches, and previous-left-to-current-left matches.
