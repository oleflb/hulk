# /// script
# requires-python = ">=3.13"
# dependencies = ["click"]
# ///

import os
import shutil
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path

import click

from annotato_common import (
    CLASS_MAP,
    LabelSchemaError,
    load_label_file,
    TASK_NAMES,
    class_map_for_task,
    class_names_for_task,
    supported_image_paths,
    validate_annotation,
)

PREPARE_MANIFEST_FILENAME = "source_images.json"
PRELABELED_DATA_FILENAME = "prelabeled-data.json"


@dataclass(frozen=True)
class Sample:
    filename: str
    base: str
    json_path: Path
    image_path: Path


def convert_annotations(
    json_path: str | Path,
    filename: str,
    task: str,
    point_box_size: float = 0.05,
) -> list[str]:
    _, annotations = load_label_file(
        json_path,
        filename,
        allow_unknown_classes=True,
    )

    class_map = class_map_for_task(task)
    yolo_lines = []

    for ann in annotations:
        class_name = ann.get("class")
        points = ann.get("points")
        point = ann.get("point")

        if class_name not in CLASS_MAP or class_name not in class_map:
            continue

        try:
            validate_annotation(ann, filename)
        except LabelSchemaError as error:
            raise click.ClickException(str(error)) from error

        if task == "pose" and point is None:
            continue

        if points is None:
            assert point is not None
            x_center, y_center = point
            width = point_box_size
            height = point_box_size
        else:
            (x1, y1), (x2, y2) = points

            # Ensure correct ordering
            x_min = min(x1, x2)
            x_max = max(x1, x2)
            y_min = min(y1, y2)
            y_max = max(y1, y2)

            # Convert to YOLO format (normalized)
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            width = x_max - x_min
            height = y_max - y_min

        class_id = class_map[class_name]
        yolo_line = (
            f"{class_id} {x_center:.6f} {y_center:.6f} "
            f"{width:.6f} {height:.6f}"
        )
        if task in {"pose", "mixed"} and point is not None:
            point_x, point_y = point
            yolo_line += f" {point_x:.6f} {point_y:.6f} 2"

        yolo_lines.append(yolo_line)

    return yolo_lines


def find_images_by_stem(input_dir: str | Path) -> dict[str, Path]:
    images_by_stem: dict[str, Path] = {}

    for image_path in supported_image_paths(Path(input_dir)):
        filename = image_path.name
        stem = image_path.stem

        if stem in images_by_stem:
            current_name = images_by_stem[stem].name
            if filename < current_name:
                images_by_stem[stem] = image_path
            print(
                "Found multiple images for "
                f"'{stem}', using '{images_by_stem[stem].name}'."
            )
            continue

        images_by_stem[stem] = image_path

    return images_by_stem


def is_label_sidecar(path: Path) -> bool:
    return (
        path.is_file()
        and path.suffix == ".json"
        and path.name
        not in {PREPARE_MANIFEST_FILENAME, PRELABELED_DATA_FILENAME}
    )


def collect_samples_from_folder(input_dir: Path) -> list[Sample]:
    images_by_stem = find_images_by_stem(input_dir)
    json_paths = sorted(path for path in input_dir.iterdir() if is_label_sidecar(path))

    samples: list[Sample] = []
    for json_path in json_paths:
        base = json_path.stem
        image_path = images_by_stem.get(base)

        if image_path is None:
            print(f"No matching image found for {json_path.name}. Skipping...")
            continue

        samples.append(
            Sample(
                filename=json_path.name,
                base=base,
                json_path=json_path,
                image_path=image_path,
            )
        )

    return samples


def collect_samples(input_dir: str | Path) -> list[Sample]:
    input_path = Path(input_dir)
    samples = collect_samples_from_folder(input_path)

    if (input_path / PREPARE_MANIFEST_FILENAME).is_file() or not samples:
        for chunk_path in sorted(path for path in input_path.iterdir() if path.is_dir()):
            samples.extend(collect_samples_from_folder(chunk_path))

    return samples


def yaml_quote(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


def dataset_yaml_lines(output_dir: str | Path, task: str) -> list[str]:
    lines = [
        f"path: {yaml_quote(str(Path(output_dir).resolve()))}",
        "train: images/train",
        "val: images/val",
    ]

    if task == "pose":
        lines.extend(
            [
                "kpt_shape: [1, 3]",
                "flip_idx: [0]",
            ]
        )
    elif task == "mixed":
        lines.append("# Mixed output is for inspection only, not direct training.")

    lines.append("names:")
    lines.extend(
        f"  {class_id}: {class_name}"
        for class_id, class_name in enumerate(class_names_for_task(task))
    )
    return lines


def write_dataset_yaml(output_dir: str | Path, task: str) -> None:
    yaml_path = Path(output_dir) / "data.yaml"
    yaml_path.write_text("\n".join(dataset_yaml_lines(output_dir, task)) + "\n")


@click.command()
@click.argument(
    "input_dir",
    type=click.Path(
        exists=True,
        file_okay=False,
        path_type=str,
    ),
)
@click.argument(
    "output_dir",
    type=click.Path(
        file_okay=False,
        path_type=str,
    ),
)
@click.option(
    "--task",
    type=click.Choice(TASK_NAMES, case_sensitive=False),
    required=True,
    help=(
        "YOLO task output: object=Ball/Robot detection, "
        "pose=point-class pose labels, mixed=all annotato classes."
    ),
)
@click.option(
    "--train-split",
    type=click.FloatRange(0.0, 1.0),
    default=0.8,
    show_default=True,
    help="Fraction of matched samples to put into train. Val gets the rest.",
)
@click.option(
    "--seed",
    type=int,
    default=42,
    show_default=True,
    help="Random seed used for shuffling before the train/val split.",
)
@click.option(
    "--point-box-size",
    type=click.FloatRange(0.0, 1.0, min_open=True),
    default=0.05,
    show_default=True,
    help="Normalized dummy box size used for point-only labels.",
)
def main(
    input_dir: str,
    output_dir: str,
    task: str,
    train_split: float,
    seed: int,
    point_box_size: float,
) -> None:
    """Convert Annotato labels to YOLO and create a COCO-style train/val layout.

    INPUT_DIR must contain JSON annotations and image files with matching stems.
    OUTPUT_DIR will be created with images/{train,val} and labels/{train,val}.
    """
    task = task.lower()
    labels_root = os.path.join(output_dir, "labels")
    images_root = os.path.join(output_dir, "images")

    for split_name in ("train", "val"):
        os.makedirs(os.path.join(labels_root, split_name), exist_ok=True)
        os.makedirs(os.path.join(images_root, split_name), exist_ok=True)
    write_dataset_yaml(output_dir, task)

    samples = collect_samples(input_dir)

    if not samples:
        print("No matching JSON/image samples were found.")
        return

    samples.sort(
        key=lambda item: sha256(f"{seed}:{item.base}".encode()).hexdigest()
    )

    train_count = int(len(samples) * train_split)
    split_samples = {
        "train": samples[:train_count],
        "val": samples[train_count:],
    }

    processed = 0
    for split_name, items in split_samples.items():
        split_labels_dir = os.path.join(labels_root, split_name)
        split_images_dir = os.path.join(images_root, split_name)

        for sample in items:
            out_txt_path = os.path.join(split_labels_dir, sample.base + ".txt")
            out_img_path = os.path.join(
                split_images_dir, sample.image_path.name
            )

            try:
                yolo_lines = convert_annotations(
                    sample.json_path,
                    sample.filename,
                    task,
                    point_box_size,
                )

                # Write output TXT (even if empty, to stay consistent)
                with open(out_txt_path, "w") as f:
                    f.write("\n".join(yolo_lines))

                shutil.copy2(sample.image_path, out_img_path)
                processed += 1
            except click.ClickException:
                raise
            except Exception as error:
                print(
                    f"Error processing {sample.filename}: {error}. Skipping..."
                )

    print(
        "Done generating YOLO dataset with split: "
        f"{processed} samples "
        "("
        f"{len(split_samples['train'])} train / "
        f"{len(split_samples['val'])} val"
        f") for task '{task}'."
    )


if __name__ == "__main__":
    main()
