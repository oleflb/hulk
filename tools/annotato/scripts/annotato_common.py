import math
from pathlib import Path

CLASS_NAMES = [
    "Ball",
    "GoalPost",
    "LSpot",
    "PenaltySpot",
    "Robot",
    "TSpot",
    "XSpot",
    "Person",
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
POINT_CLASSES = {"GoalPost", "LSpot", "TSpot", "XSpot"}
MIGRATED_POINT_CLASSES = {"LSpot", "TSpot", "XSpot"}


class LabelSchemaError(ValueError):
    pass


def supported_image_paths(folder):
    return sorted(
        path
        for path in folder.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def validate_annotation(annotation, filename):
    extra_fields = set(annotation) - {"class", "points", "point"}
    if extra_fields:
        fields = ", ".join(sorted(extra_fields))
        raise LabelSchemaError(f"Unexpected annotation fields in {filename}: {fields}")

    class_name = annotation.get("class")
    points = annotation.get("points")
    point = annotation.get("point")

    if class_name not in CLASS_MAP:
        raise LabelSchemaError(f"Unknown class '{class_name}' in {filename}")

    if points is None and point is None:
        raise LabelSchemaError(
            f"Annotation in {filename} must contain 'points' or 'point' geometry"
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
                f"Class '{class_name}' in {filename} does not support point geometry"
            )
    if points is not None and point is not None and class_name not in MIGRATED_POINT_CLASSES:
        raise LabelSchemaError(
            f"Class '{class_name}' in {filename} cannot combine points and point geometry"
        )


def validate_normalized_point(point, filename):
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
            f"Normalized coordinates in {filename} must be finite and between 0 and 1"
        )
