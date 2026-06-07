from __future__ import annotations

import os
from pathlib import Path

import click
import torch
from torch import ByteTensor, Tensor, nn

from xfeat_export.export_lighterglue_onnx import (
    LighterGlueFixedWrapper,
    default_weights_path as default_lighterglue_weights_path,
)
from xfeat_export.export_xfeat_onnx import (
    XFeatNv12TopKWrapper,
    default_weights_path as default_xfeat_weights_path,
    validate_image_size,
)


class XFeatLighterGlueWrapper(nn.Module):
    def __init__(
        self,
        xfeat_weights_path: Path,
        lighterglue_weights_path: Path,
        *,
        keypoint_count: int,
        detection_threshold: float,
        min_confidence: float,
    ) -> None:
        super().__init__()
        self.extractor = XFeatNv12TopKWrapper(
            xfeat_weights_path,
            keypoint_count=keypoint_count,
            detection_threshold=detection_threshold,
        )
        self.matcher = LighterGlueFixedWrapper(
            lighterglue_weights_path,
            min_confidence=min_confidence,
        )

    def forward(
        self,
        previous_left: ByteTensor,
        previous_right: ByteTensor,
        current_left: ByteTensor,
        current_right: ByteTensor,
    ) -> tuple[Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor]:
        raw_images = torch.stack(
            [previous_left, previous_right, current_left, current_right], dim=0
        )
        keypoints, descriptors, _, valid = self.extractor(raw_images)

        stereo_matches, _, stereo_scores, _ = self.matcher(
            keypoints[2:3],
            keypoints[3:4],
            descriptors[2:3],
            descriptors[3:4],
            valid[2:3],
            valid[3:4],
        )
        temporal_matches, _, temporal_scores, _ = self.matcher(
            keypoints[0:1],
            keypoints[2:3],
            descriptors[0:1],
            descriptors[2:3],
            valid[0:1],
            valid[2:3],
        )

        return (
            keypoints[2],
            keypoints[3],
            valid[2],
            valid[3],
            stereo_matches.squeeze(0),
            stereo_scores.squeeze(0),
            temporal_matches.squeeze(0),
            temporal_scores.squeeze(0),
        )


def dynamic_axes(input_names: list[str]) -> dict[str, dict[int, str]]:
    return {name: {0: "half_height", 1: "half_width"} for name in input_names}


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("export-path", type=click.Path(path_type=Path))
@click.option(
    "--xfeat-weights",
    type=click.Path(exists=True, path_type=Path),
    default=None,
    help="Path to the XFeat .pt weights. Defaults to the accelerated-features package weights.",
)
@click.option(
    "--lighterglue-weights",
    type=click.Path(exists=True, path_type=Path),
    default=None,
    help="Path to the XFeat LighterGlue .pt weights. Defaults to the accelerated-features package weights.",
)
@click.option("--height", default=1088, show_default=True, help="Full-resolution dummy image height.")
@click.option("--width", default=1280, show_default=True, help="Full-resolution dummy image width.")
@click.option("--keypoints", "keypoint_count", default=512, show_default=True, help="Fixed keypoint count.")
@click.option("--threshold", "detection_threshold", default=0.05, show_default=True, help="NMS detection threshold.")
@click.option("--min-confidence", default=0.1, show_default=True, help="Minimum match confidence.")
@click.option("--opset", default=20, show_default=True, help="ONNX opset version.")
@click.option("--device", default="cpu", show_default=True, help="Torch export device, e.g. cpu or cuda:0.")
@click.option("--dynamic/--static", "use_dynamic_axes", default=True, show_default=True, help="Mark image dimensions dynamic.")
def main(
    export_path: Path,
    *,
    xfeat_weights: Path | None,
    lighterglue_weights: Path | None,
    height: int,
    width: int,
    keypoint_count: int,
    detection_threshold: float,
    min_confidence: float,
    opset: int,
    device: str,
    use_dynamic_axes: bool,
) -> None:
    validate_image_size(height, width)
    if keypoint_count <= 0:
        raise click.BadParameter("--keypoints must be > 0")

    wrapper = XFeatLighterGlueWrapper(
        xfeat_weights or default_xfeat_weights_path(),
        lighterglue_weights or default_lighterglue_weights_path(),
        keypoint_count=keypoint_count,
        detection_threshold=detection_threshold,
        min_confidence=min_confidence,
    ).to(device)
    wrapper.eval()

    input_shape = (height // 2, width // 2, 6)
    dummy_input = torch.zeros(input_shape, dtype=torch.uint8, device=device)
    input_names = ["previous_left", "previous_right", "current_left", "current_right"]
    output_names = [
        "current_left_keypoints",
        "current_right_keypoints",
        "current_left_valid",
        "current_right_valid",
        "stereo_matches",
        "stereo_scores",
        "temporal_matches",
        "temporal_scores",
    ]

    export_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        wrapper,
        (dummy_input, dummy_input, dummy_input, dummy_input),
        export_path,
        input_names=input_names,
        output_names=output_names,
        dynamic_axes=dynamic_axes(input_names) if use_dynamic_axes else None,
        opset_version=opset,
        external_data=False,
        dynamo=False,
    )

    click.echo(f"Exported fused XFeat/LighterGlue ONNX model to: {os.path.abspath(export_path)}")


if __name__ == "__main__":
    main()
