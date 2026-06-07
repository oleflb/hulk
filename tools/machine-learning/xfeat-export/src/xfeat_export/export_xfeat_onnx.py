from __future__ import annotations

import os
from pathlib import Path

import click
import torch
import torch.nn.functional as F
from torch import ByteTensor, Tensor, nn
from torch.onnx import operators as onnx_ops

from modules.model import XFeatModel
from utils.nv12_to_rgb import NV12ToRgb


def default_weights_path() -> Path:
    import modules.model

    return Path(modules.model.__file__).resolve().parents[1] / "weights" / "xfeat.pt"


class ExportableXFeatModel(XFeatModel):
    def _unfold2d(self, x: Tensor, ws: int = 2) -> Tensor:
        return F.pixel_unshuffle(x, downscale_factor=ws)


def load_xfeat_model(weights_path: Path) -> ExportableXFeatModel:
    model = ExportableXFeatModel()
    state_dict = torch.load(weights_path, map_location="cpu")
    model.load_state_dict(state_dict)
    return model.float().eval()


class XFeatNv12TopKWrapper(nn.Module):
    def __init__(
        self,
        weights_path: Path,
        *,
        keypoint_count: int,
        detection_threshold: float,
    ) -> None:
        super().__init__()
        self.keypoint_count = keypoint_count
        self.detection_threshold = detection_threshold
        self.preprocessor = NV12ToRgb(subsample=False)
        self.xfeat = load_xfeat_model(weights_path)

    def forward(self, raw_bytes_input: ByteTensor) -> tuple[Tensor, Tensor, Tensor, Tensor]:
        rgb = self._preprocess(raw_bytes_input)
        descriptors, keypoint_logits, reliability = self.xfeat(rgb)

        descriptors = F.normalize(descriptors, dim=1)
        keypoint_heatmap = self._keypoint_heatmap(keypoint_logits)
        score_map = self._score_map(keypoint_heatmap, reliability)

        keypoints, scores = self._topk_keypoints(score_map)
        sampled_descriptors = self._sample_descriptors(descriptors, keypoints, score_map)
        sampled_descriptors = F.normalize(sampled_descriptors, dim=-1)

        valid = scores > 0.0
        keypoints = self._normalize_keypoints(keypoints, score_map)
        batch_size = score_map.shape[0]

        keypoints = keypoints.reshape(batch_size, self.keypoint_count, 2)
        sampled_descriptors = sampled_descriptors.reshape(batch_size, self.keypoint_count, 64)
        scores = scores.reshape(batch_size, self.keypoint_count)
        valid = valid.reshape(batch_size, self.keypoint_count)

        return keypoints, sampled_descriptors, scores, valid

    def _preprocess(self, raw_bytes_input: ByteTensor) -> Tensor:
        if raw_bytes_input.dim() == 3:
            return self.preprocessor(raw_bytes_input).unsqueeze(0).permute(0, 3, 1, 2)
        return self._batched_nv12_to_rgb(raw_bytes_input).permute(0, 3, 1, 2)

    def _batched_nv12_to_rgb(self, raw_bytes_input: ByteTensor) -> Tensor:
        image_data = raw_bytes_input.to(torch.float32)
        batch_size, half_height, half_width, _ = image_data.shape
        height, width = half_height * 2, half_width * 2
        flat = image_data.reshape(batch_size, -1)
        luminance = flat[:, : width * height].reshape(batch_size, height, width, 1)
        chroma_subsampled = flat[:, width * height :].reshape(batch_size, half_height, half_width, 2)
        chroma = chroma_subsampled.repeat_interleave(2, dim=1).repeat_interleave(2, dim=2)
        yuv = torch.concat([luminance, chroma], dim=-1)
        return torch.matmul(yuv - self.preprocessor.yuv_to_rgb_offset, self.preprocessor.yuv_to_rgb)

    @staticmethod
    def _keypoint_heatmap(keypoint_logits: Tensor) -> Tensor:
        scores = F.softmax(keypoint_logits, dim=1)[:, :64]
        batch_size, _, height, width = scores.shape
        heatmap = scores.permute(0, 2, 3, 1).reshape(batch_size, height, width, 8, 8)
        return heatmap.permute(0, 1, 3, 2, 4).reshape(batch_size, 1, height * 8, width * 8)

    def _score_map(self, keypoint_heatmap: Tensor, reliability: Tensor) -> Tensor:
        local_max = F.max_pool2d(keypoint_heatmap, kernel_size=5, stride=1, padding=2)
        nms_mask = (keypoint_heatmap == local_max) & (keypoint_heatmap > self.detection_threshold)
        keypoint_scores = self._sample_map(keypoint_heatmap, keypoint_heatmap, mode="nearest")
        reliability = self._sample_map(reliability, keypoint_heatmap, mode="bilinear")
        scores = keypoint_scores * reliability
        return torch.where(nms_mask, scores, torch.zeros_like(scores))

    @staticmethod
    def _sample_map(source: Tensor, target_shape: Tensor, *, mode: str) -> Tensor:
        shape = onnx_ops.shape_as_tensor(target_shape)
        height = shape[-2]
        width = shape[-1]
        y = torch.arange(height, device=target_shape.device, dtype=target_shape.dtype)
        x = torch.arange(width, device=target_shape.device, dtype=target_shape.dtype)
        grid_y, grid_x = torch.meshgrid(y, x, indexing="ij")
        normalizer = torch.stack([width, height]).to(dtype=target_shape.dtype) - 1.0
        grid = (2.0 * torch.stack([grid_x, grid_y], dim=-1) / normalizer) - 1.0
        grid = grid.unsqueeze(0).expand(source.shape[0], -1, -1, -1)
        return F.grid_sample(source, grid, mode=mode, align_corners=False)

    def _topk_keypoints(self, score_map: Tensor) -> tuple[Tensor, Tensor]:
        batch_size, _, _, width = score_map.shape
        topk_scores, indices = torch.topk(score_map.reshape(batch_size, -1), k=self.keypoint_count, dim=-1)
        y = torch.div(indices, width, rounding_mode="floor")
        x = indices - y * width
        return torch.stack([x, y], dim=-1).to(dtype=score_map.dtype), topk_scores

    @staticmethod
    def _sample_descriptors(descriptors: Tensor, keypoints: Tensor, score_map: Tensor) -> Tensor:
        shape = onnx_ops.shape_as_tensor(score_map)
        height = shape[-2].to(dtype=keypoints.dtype)
        width = shape[-1].to(dtype=keypoints.dtype)
        normalizer = torch.stack([width - 1.0, height - 1.0]).view(1, 1, 2)
        grid = ((2.0 * keypoints) / normalizer - 1.0).unsqueeze(-2)
        sampled = F.grid_sample(descriptors, grid, mode="bicubic", align_corners=False)
        return sampled.permute(0, 2, 3, 1).squeeze(-2)

    @staticmethod
    def _normalize_keypoints(keypoints: Tensor, score_map: Tensor) -> Tensor:
        shape = onnx_ops.shape_as_tensor(score_map)
        height = shape[-2].to(dtype=keypoints.dtype)
        width = shape[-1].to(dtype=keypoints.dtype)
        image_size = torch.stack([width, height]).view(1, 1, 2)
        scale = torch.maximum(width, height) / 2.0
        return (keypoints - image_size / 2.0) / scale


def validate_image_size(height: int, width: int) -> None:
    if height <= 0 or width <= 0:
        raise click.BadParameter("--height and --width must be > 0")
    if height % 2 != 0 or width % 2 != 0:
        raise click.BadParameter("--height and --width must be even for NV12")
    if height % 32 != 0 or width % 32 != 0:
        raise click.BadParameter("--height and --width must be divisible by 32 for XFeat")


def dynamic_axes(batch_size: int | None) -> dict[str, dict[int, str]]:
    if batch_size is not None:
        return {"raw_bytes_input": {1: "half_height", 2: "half_width"}}
    return {"raw_bytes_input": {0: "half_height", 1: "half_width"}}


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("export-path", type=click.Path(path_type=Path))
@click.option(
    "--weights",
    "weights_path",
    type=click.Path(exists=True, path_type=Path),
    default=None,
    help="Path to the XFeat .pt weights. Defaults to the accelerated-features package weights.",
)
@click.option("--height", default=1088, show_default=True, help="Full-resolution dummy image height.")
@click.option("--width", default=1280, show_default=True, help="Full-resolution dummy image width.")
@click.option("--batch-size", default=None, type=int, help="Static batch size. If omitted, export the legacy unbatched input.")
@click.option("--keypoints", "keypoint_count", default=512, show_default=True, help="Fixed keypoint count.")
@click.option("--threshold", "detection_threshold", default=0.05, show_default=True, help="NMS detection threshold.")
@click.option("--opset", default=20, show_default=True, help="ONNX opset version.")
@click.option("--device", default="cpu", show_default=True, help="Torch export device, e.g. cpu or cuda:0.")
@click.option("--dynamic/--static", "use_dynamic_axes", default=True, show_default=True, help="Mark image dimensions dynamic.")
def main(
    export_path: Path,
    *,
    weights_path: Path | None,
    height: int,
    width: int,
    batch_size: int | None,
    keypoint_count: int,
    detection_threshold: float,
    opset: int,
    device: str,
    use_dynamic_axes: bool,
) -> None:
    validate_image_size(height, width)
    if keypoint_count <= 0:
        raise click.BadParameter("--keypoints must be > 0")
    if keypoint_count > height * width:
        raise click.BadParameter("--keypoints must be <= height * width")
    if batch_size is not None and batch_size <= 0:
        raise click.BadParameter("--batch-size must be > 0")

    wrapper = XFeatNv12TopKWrapper(
        weights_path or default_weights_path(),
        keypoint_count=keypoint_count,
        detection_threshold=detection_threshold,
    ).to(device)
    wrapper.eval()

    input_shape = (height // 2, width // 2, 6) if batch_size is None else (batch_size, height // 2, width // 2, 6)
    dummy_input = torch.zeros(input_shape, dtype=torch.uint8, device=device)
    output_names = ["keypoints", "descriptors", "scores", "valid"]

    export_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        wrapper,
        (dummy_input,),
        export_path,
        input_names=["raw_bytes_input"],
        output_names=output_names,
        dynamic_axes=dynamic_axes(batch_size) if use_dynamic_axes else None,
        opset_version=opset,
        external_data=False,
        dynamo=False,
    )

    click.echo(f"Exported XFeat ONNX model to: {os.path.abspath(export_path)}")


if __name__ == "__main__":
    main()
