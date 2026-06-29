# /// script
# requires-python = ">=3.13"
# dependencies = [
#   "click",
#   "tqdm",
#   "opencv-python",
#   "ultralytics",
#   "wonderwords",
#   "onnxruntime",
# ]
# ///

# ruff: noqa: FBT001, S311, TRY003

import json
import shutil
from collections.abc import Callable, Iterable, Iterator
from pathlib import Path
from typing import Any
from uuid import uuid4

import click  # ty: ignore[unresolved-import]
from annotato_common import CLASS_MAP, CLASS_NAMES, supported_image_paths

MANIFEST_FILENAME = "source_images.json"
MANIFEST_VERSION = 1
PRELABELED_DATA_FILENAME = "prelabeled-data.json"


Annotation = dict[str, Any]
Manifest = dict[str, Any]


def source_key(source_path: Path) -> str:
    return str(Path(source_path).resolve())


def manifest_entry_for_source(
    manifest: Manifest,
    source_path: Path,
) -> dict[str, str] | None:
    return manifest["images"].get(source_key(source_path))


def validate_unique_source_keys(image_paths: list[Path]) -> None:
    seen_sources = {}
    for image_path in image_paths:
        key = source_key(image_path)
        if key in seen_sources:
            raise click.ClickException(
                "duplicate source image resolves to the same path: "
                f"{seen_sources[key]} and {image_path}"
            )
        seen_sources[key] = image_path


def empty_manifest() -> Manifest:
    return {"version": MANIFEST_VERSION, "images": {}}


def manifest_path(output_path: Path) -> Path:
    return output_path / MANIFEST_FILENAME


def load_manifest(output_path: Path) -> Manifest:
    path = manifest_path(output_path)
    if not path.exists():
        return empty_manifest()

    with open(path) as f:
        manifest = json.load(f)

    if manifest.get("version") != MANIFEST_VERSION:
        raise click.ClickException(
            f"unsupported manifest version in {path}: {manifest.get('version')}"
        )
    if not isinstance(manifest.get("images"), dict):
        raise click.ClickException(f"invalid manifest format in {path}")

    return manifest


def write_manifest(output_path: Path, manifest: Manifest) -> None:
    with open(manifest_path(output_path), "w") as f:
        json.dump(manifest, f)


def parse_yolo_classes(class_spec: str | None) -> set[int] | None:
    if class_spec is None:
        return None

    class_ids = set()
    for raw_part in class_spec.split(","):
        part = raw_part.strip()
        if not part:
            raise click.BadParameter("class list contains an empty item")

        if part in CLASS_NAMES:
            class_ids.add(CLASS_NAMES.index(part))
            continue

        if "-" in part:
            range_start, range_end = part.split("-", 1)
            try:
                start = int(range_start)
                end = int(range_end)
            except ValueError as error:
                raise click.BadParameter(
                    f"invalid class range '{part}'"
                ) from error
            if start > end:
                raise click.BadParameter(
                    f"invalid descending class range '{part}'"
                )
            for class_id in range(start, end + 1):
                validate_class_id(class_id)
                class_ids.add(class_id)
            continue

        try:
            class_id = int(part)
        except ValueError as error:
            raise click.BadParameter(f"unknown class '{part}'") from error
        validate_class_id(class_id)
        class_ids.add(class_id)

    return class_ids


def resolve_sample_size(
    total_images: int,
    sample_count: int | None,
    sample_fraction: float | None,
) -> int | None:
    if sample_count is not None and sample_fraction is not None:
        raise click.UsageError(
            "--sample-count and --sample-fraction are mutually exclusive"
        )

    if sample_count is not None:
        return min(sample_count, total_images)

    if sample_fraction is not None:
        import math

        return max(
            1,
            min(total_images, math.ceil(total_images * sample_fraction)),
        )

    return None


def sample_random(
    image_paths: list[Path],
    sample_size: int | None,
    seed: int,
) -> list[Path]:
    if sample_size is None or sample_size >= len(image_paths):
        return image_paths

    import random

    rng = random.Random(seed)
    selected_indices = sorted(rng.sample(range(len(image_paths)), sample_size))
    return [image_paths[index] for index in selected_indices]


def sample_spread(
    image_paths: list[Path],
    sample_size: int | None,
) -> list[Path]:
    if sample_size is None or sample_size >= len(image_paths):
        return image_paths
    if sample_size == 1:
        return [image_paths[0]]

    last_index = len(image_paths) - 1
    indices = [
        round(position * last_index / (sample_size - 1))
        for position in range(sample_size)
    ]
    return [image_paths[index] for index in indices]


def sample_visual_diversity(
    image_paths: list[Path],
    sample_size: int | None,
    seed: int,
    feature_for_image: Callable[[Path], tuple[float, ...]] | None = None,
) -> list[Path]:
    if sample_size is None or sample_size >= len(image_paths):
        return image_paths

    feature_for_image = feature_for_image or image_fingerprint
    features = {
        path: feature_for_image(path)
        for path in progress(image_paths)
    }
    source_order = {path: index for index, path in enumerate(image_paths)}
    feature_order = sorted(
        image_paths,
        key=lambda path: (features[path], source_order[path]),
    )
    start_index = seed % len(feature_order)
    rotated_order = feature_order[start_index:] + feature_order[:start_index]
    selected = sample_spread(rotated_order, sample_size)
    selected_paths = set(selected)
    return [path for path in image_paths if path in selected_paths]


def image_fingerprint(image_path: Path) -> tuple[float, ...]:
    import cv2  # ty: ignore[unresolved-import]

    image = cv2.imread(str(image_path))
    if image is None:
        raise click.ClickException(f"failed to read image {image_path}")

    thumbnail = cv2.resize(image, (16, 16), interpolation=cv2.INTER_AREA)
    fingerprint = []
    for channel in range(3):
        histogram = cv2.calcHist(
            [thumbnail],
            [channel],
            None,
            [8],
            [0, 256],
        ).flatten()
        total = histogram.sum()
        if total:
            histogram = histogram / total
        fingerprint.extend(float(value) for value in histogram)

    return tuple(fingerprint)


def validate_class_id(class_id: int) -> None:
    if class_id < 0 or class_id >= len(CLASS_NAMES):
        raise click.BadParameter(
            f"class id {class_id} is outside the supported range "
            f"0-{len(CLASS_NAMES) - 1}"
        )


def merge_annotations(
    existing_annotations: list[Annotation],
    new_annotations: list[Annotation],
    replacement_class_ids: set[int],
) -> list[Annotation]:
    preserved_annotations = [
        annotation
        for annotation in existing_annotations
        if CLASS_MAP.get(annotation.get("class")) not in replacement_class_ids
    ]
    return preserved_annotations + new_annotations


def load_data_json(chunk_path: Path) -> dict[str, list[Annotation]]:
    data_path = chunk_path / PRELABELED_DATA_FILENAME
    if not data_path.exists():
        return {}

    with open(data_path) as f:
        return json.load(f)


def write_data_json(
    chunk_path: Path,
    chunk_annotations: dict[str, list[Annotation]],
) -> None:
    with open(chunk_path / PRELABELED_DATA_FILENAME, "w") as f:
        json.dump(chunk_annotations, f)


def generate_random_chunk_name() -> str:
    from wonderwords import RandomWord  # ty: ignore[unresolved-import]

    rng = RandomWord()

    adjective = rng.word(
        include_categories=["adjective"], regex="[a-zA-Z]+"
    ).lower()
    noun = rng.word(include_categories=["noun"], regex="[a-zA-Z]+").lower()

    return f"{adjective}-{noun}"


def chunked(items: list[Path], chunk_size: int) -> Iterator[list[Path]]:
    for start in range(0, len(items), chunk_size):
        yield items[start : start + chunk_size]


def progress(iterable: Iterable[Any]) -> Iterable[Any]:
    from tqdm import tqdm  # ty: ignore[unresolved-import]

    return tqdm(iterable)


def generated_image_name(source_path: Path) -> str:
    return f"{uuid4()}{source_path.suffix.lower()}"


def load_image(image_path: Path, convert_colors: bool) -> Any:
    import cv2  # ty: ignore[unresolved-import]

    image = cv2.imread(str(image_path))
    if image is None:
        raise click.ClickException(f"failed to read image {image_path}")

    if convert_colors:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.cvtColor(image, cv2.COLOR_YCrCb2RGB)

    return image


def write_image(
    image_path: Path,
    output_image_path: Path,
    image: Any,
    convert_colors: bool,
) -> None:
    if not convert_colors:
        shutil.copy2(image_path, output_image_path)
        return

    import cv2  # ty: ignore[unresolved-import]

    if not cv2.imwrite(str(output_image_path), image):
        raise click.ClickException(f"failed to write image {output_image_path}")


def annotations_from_detection(
    detection: Any,
    selected_class_ids: set[int] | None,
) -> list[Annotation]:
    annotations = []

    for box in detection[0].boxes:
        class_id = int(box.cls)
        if (
            selected_class_ids is not None
            and class_id not in selected_class_ids
        ):
            continue
        validate_class_id(class_id)
        annotations.append(
            {
                "class": CLASS_NAMES[class_id],
                "points": box.xyxyn.reshape(2, 2).tolist(),
            }
        )

    return annotations


def sample_images(
    image_paths: list[Path],
    sample_size: int | None,
    method: str,
    seed: int,
) -> list[Path]:
    method = method.lower()

    if sample_size is None or sample_size >= len(image_paths):
        return image_paths

    if method == "random":
        return sample_random(image_paths, sample_size, seed)

    if method == "spread":
        return sample_spread(image_paths, sample_size)

    if method == "visual-diversity":
        return sample_visual_diversity(image_paths, sample_size, seed)

    raise click.UsageError(f"unknown sample method: {method}")


def infer_annotations(
    yolo_model: Any,
    image: Any,
    selected_class_ids: set[int] | None,
) -> list[Annotation]:
    detection = yolo_model(
        image,
        verbose=False,
        conf=0.1,
        end2end=False,
        iou=0.3,
    )
    return annotations_from_detection(detection, selected_class_ids)


def load_yolo_model(yolo_checkpoint: Path) -> Any:
    from ultralytics import YOLO  # ty: ignore[unresolved-import]

    return YOLO(str(yolo_checkpoint))


def create_chunk_path(output_path: Path) -> tuple[str, Path]:
    for _ in range(100):
        chunk_name = generate_random_chunk_name()
        chunk_path = output_path / chunk_name
        if not chunk_path.exists():
            return chunk_name, chunk_path

    raise click.ClickException("failed to generate a unique chunk name")


def create_labelling_chunks(
    image_paths: list[Path],
    output_path: Path,
    manifest: Manifest,
    yolo_model: Any,
    selected_class_ids: set[int] | None,
    chunk_size: int,
    convert_colors: bool,
) -> None:
    duplicate_sources = [
        image_path
        for image_path in image_paths
        if manifest_entry_for_source(manifest, image_path) is not None
    ]
    if duplicate_sources:
        raise click.ClickException(
            "source image already exists in the manifest; "
            f"use --extend to add labels: {duplicate_sources[0]}"
        )

    for chunk in progress(list(chunked(image_paths, chunk_size))):
        chunk_name, chunk_path = create_chunk_path(output_path)
        chunk_path.mkdir(parents=True, exist_ok=False)
        chunk_annotations = {}

        for image_path in chunk:
            image_name = generated_image_name(image_path)
            output_image_path = chunk_path / image_name
            image = None
            if yolo_model is not None or convert_colors:
                image = load_image(image_path, convert_colors)

            if yolo_model is not None:
                chunk_annotations[image_name] = infer_annotations(
                    yolo_model,
                    image,
                    selected_class_ids,
                )

            write_image(image_path, output_image_path, image, convert_colors)
            manifest["images"][source_key(image_path)] = {
                "chunk": chunk_name,
                "image": image_name,
            }

        write_data_json(chunk_path, chunk_annotations)


def extend_labelling_chunks(
    image_paths: list[Path],
    output_path: Path,
    manifest: Manifest,
    yolo_model: Any,
    selected_class_ids: set[int] | None,
    replacement_class_ids: set[int],
    convert_colors: bool,
) -> None:
    targets = collect_extend_targets(image_paths, output_path, manifest)
    chunk_updates = {}

    for image_path, chunk_path, image_name in progress(targets):
        image = load_image(image_path, convert_colors)
        new_annotations = infer_annotations(
            yolo_model,
            image,
            selected_class_ids,
        )

        if chunk_path not in chunk_updates:
            chunk_updates[chunk_path] = load_data_json(chunk_path)
        chunk_annotations = chunk_updates[chunk_path]
        existing_annotations = chunk_annotations.get(image_name, [])
        chunk_annotations[image_name] = merge_annotations(
            existing_annotations,
            new_annotations,
            replacement_class_ids,
        )

    for chunk_path, chunk_annotations in chunk_updates.items():
        write_data_json(chunk_path, chunk_annotations)


def collect_extend_targets(
    image_paths: list[Path],
    output_path: Path,
    manifest: Manifest,
) -> list[tuple[Path, Path, str]]:
    targets = []
    for image_path in image_paths:
        entry = manifest_entry_for_source(manifest, image_path)
        if entry is None:
            raise click.ClickException(
                f"no manifest entry for {image_path}; "
                "run without --extend first"
            )

        chunk_path = output_path / entry["chunk"]
        output_image_path = chunk_path / entry["image"]
        if not output_image_path.exists():
            raise click.ClickException(
                f"manifest points to a missing image: {output_image_path}"
            )

        targets.append((image_path, chunk_path, entry["image"]))

    return targets


@click.command(context_settings={"show_default": True})
@click.option(
    "--image-folder",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    required=True,
    help="The source image folder.",
)
@click.option(
    "--output",
    "output_path",
    type=click.Path(file_okay=False, path_type=Path),
    default=Path("current"),
    help="Output folder for labelling chunks.",
)
@click.option(
    "--extend",
    is_flag=True,
    help="Extend an existing output folder using its source image manifest.",
)
@click.option(
    "--yolo",
    "yolo_checkpoint",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=None,
    help="The YOLO checkpoint used for inference.",
)
@click.option(
    "--yolo-classes",
    default=None,
    help=(
        "Comma-separated class IDs, ranges, or names to use from the YOLO "
        "output."
    ),
)
@click.option(
    "--chunk-size",
    type=click.IntRange(min=1),
    default=200,
    help="Maximum number of images in one labelling task.",
)
@click.option(
    "--sample-count",
    type=click.IntRange(min=1),
    default=None,
    help="Number of images to select before labelling task generation.",
)
@click.option(
    "--sample-fraction",
    type=click.FloatRange(min=0.0, max=1.0, min_open=True),
    default=None,
    help="Fraction of images to select before labelling task generation.",
)
@click.option(
    "--sample-method",
    type=click.Choice(
        ["random", "spread", "visual-diversity"],
        case_sensitive=False,
    ),
    default="spread",
    help="Sampling strategy used with --sample-count or --sample-fraction.",
)
@click.option(
    "--sample-seed",
    type=int,
    default=42,
    help="Seed for deterministic sampling.",
)
@click.option(
    "--convert-colors",
    is_flag=True,
    default=False,
    help="Whether to convert YCbCr to RGB before inference/output.",
)
def main(
    image_folder: Path,
    output_path: Path,
    extend: bool,
    yolo_checkpoint: Path | None,
    yolo_classes: str | None,
    chunk_size: int,
    sample_count: int | None,
    sample_fraction: float | None,
    sample_method: str,
    sample_seed: int,
    convert_colors: bool,
) -> None:
    selected_class_ids = parse_yolo_classes(yolo_classes)
    if selected_class_ids is not None and yolo_checkpoint is None:
        raise click.UsageError("--yolo-classes requires --yolo")
    if extend and yolo_checkpoint is None:
        raise click.UsageError("--extend requires --yolo")
    if extend and not manifest_path(output_path).exists():
        raise click.ClickException(
            f"cannot extend without {manifest_path(output_path)}"
        )

    image_paths = supported_image_paths(image_folder)
    if len(image_paths) == 0:
        raise click.ClickException("No images found in the image folder")
    validate_unique_source_keys(image_paths)

    output_path.mkdir(parents=True, exist_ok=True)
    manifest = load_manifest(output_path)
    yolo_model = (
        load_yolo_model(yolo_checkpoint)
        if yolo_checkpoint is not None
        else None
    )
    replacement_class_ids = selected_class_ids or set(range(len(CLASS_NAMES)))
    sample_size = resolve_sample_size(
        len(image_paths),
        sample_count,
        sample_fraction,
    )
    if sample_size is not None:
        click.echo(
            "Sampling "
            f"{sample_size} of {len(image_paths)} images with "
            f"{sample_method.lower()}"
        )
    image_paths = sample_images(
        image_paths=image_paths,
        sample_size=sample_size,
        method=sample_method,
        seed=sample_seed,
    )

    if extend:
        extend_labelling_chunks(
            image_paths,
            output_path,
            manifest,
            yolo_model,
            selected_class_ids,
            replacement_class_ids,
            convert_colors,
        )
    else:
        create_labelling_chunks(
            image_paths,
            output_path,
            manifest,
            yolo_model,
            selected_class_ids,
            chunk_size,
            convert_colors,
        )

    write_manifest(output_path, manifest)


if __name__ == "__main__":
    main()
