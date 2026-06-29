# ruff: noqa: TRY003

import json
import math
from pathlib import Path
from typing import Any

Annotation = dict[str, Any]

CLASS_NAMES = [
    "Ball",
    "GoalPost",
    "LSpot",
    "PenaltySpot",
    "Robot",
    "TSpot",
    "XSpot",
]
CLASS_MAP = {name: index for index, name in enumerate(CLASS_NAMES)}
IMAGE_EXTENSIONS = {
    f".{extension}"
    for extension in Path(__file__)
    .resolve()
    .parents[1]
    .joinpath("image_extensions.txt")
    .read_text()
    .splitlines()
    if extension
}
POINT_CLASSES = {"GoalPost", "LSpot", "PenaltySpot", "TSpot", "XSpot"}
MIGRATED_POINT_CLASSES = POINT_CLASSES


class LabelSchemaError(ValueError):
    pass


def supported_image_paths(folder: Path) -> list[Path]:
    return sorted(
        path
        for path in folder.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def load_label_file(
    json_path: Path,
    filename: str,
) -> tuple[list[str], list[Annotation]]:
    with open(json_path) as f:
        data = json.load(f)

    if isinstance(data, list):
        annotations = data
        labeled_classes = [
            class_name
            for class_name in CLASS_NAMES
            if any(
                annotation.get("class") == class_name
                for annotation in annotations
            )
        ]
    elif isinstance(data, dict):
        extra_fields = set(data) - {"labeled_classes", "annotations"}
        if extra_fields:
            fields = ", ".join(sorted(extra_fields))
            raise LabelSchemaError(
                f"Unexpected label file fields in {filename}: {fields}"
            )

        labeled_classes = data.get("labeled_classes", [])
        annotations = data.get("annotations", [])
    else:
        raise LabelSchemaError(
            f"Label file {filename} must be an object or annotation array"
        )

    if not isinstance(labeled_classes, list):
        raise LabelSchemaError(f"labeled_classes in {filename} must be a list")
    for class_name in labeled_classes:
        if class_name not in CLASS_MAP:
            raise LabelSchemaError(
                f"Unknown labeled class '{class_name}' in {filename}"
            )

    if not isinstance(annotations, list):
        raise LabelSchemaError(f"annotations in {filename} must be a list")

    return labeled_classes, annotations


def validate_annotation(annotation: Annotation, filename: str) -> None:
    extra_fields = set(annotation) - {
        "class",
        "points",
        "point",
        "migration_skipped",
    }
    if extra_fields:
        fields = ", ".join(sorted(extra_fields))
        raise LabelSchemaError(
            f"Unexpected annotation fields in {filename}: {fields}"
        )

    class_name = annotation.get("class")
    points = annotation.get("points")
    point = annotation.get("point")
    migration_skipped = annotation.get("migration_skipped", False)

    if class_name not in CLASS_MAP:
        raise LabelSchemaError(f"Unknown class '{class_name}' in {filename}")

    if points is None and point is None:
        raise LabelSchemaError(
            f"Annotation in {filename} must contain 'points' or 'point' "
            "geometry"
        )
    if points is not None:
        if len(points) != 2:
            raise LabelSchemaError(f"Invalid points in {filename}")
        for normalized_point in points:
            validate_normalized_point(normalized_point, filename)
    if point is not None:
        validate_normalized_point(point, filename)
        if class_name not in POINT_CLASSES:
            raise LabelSchemaError(
                f"Class '{class_name}' in {filename} does not support point "
                "geometry"
            )
    if (
        points is not None
        and point is not None
        and class_name not in MIGRATED_POINT_CLASSES
    ):
        raise LabelSchemaError(
            f"Class '{class_name}' in {filename} cannot combine points and "
            "point geometry"
        )
    if not isinstance(migration_skipped, bool):
        raise LabelSchemaError(
            f"migration_skipped in {filename} must be a boolean"
        )
    if migration_skipped and point is not None:
        raise LabelSchemaError(
            f"migration_skipped annotation in {filename} cannot contain point geometry"
        )
    if migration_skipped and class_name not in POINT_CLASSES:
        raise LabelSchemaError(
            f"Class '{class_name}' in {filename} does not support point migration skips"
        )
    if migration_skipped and points is None:
        raise LabelSchemaError(
            f"migration_skipped annotation in {filename} must retain points geometry"
        )


def validate_normalized_point(point: Any, filename: str) -> None:
    if not isinstance(point, (list, tuple)) or len(point) != 2:
        raise LabelSchemaError(f"Invalid normalized point in {filename}")

    x, y = point
    if not (
        isinstance(x, (int, float))
        and isinstance(y, (int, float))
        and math.isfinite(x)
        and math.isfinite(y)
        and 0.0 <= x <= 1.0
        and 0.0 <= y <= 1.0
    ):
        raise LabelSchemaError(
            f"Normalized coordinates in {filename} must be finite and between "
            "0 and 1"
        )
