# ruff: noqa: ANN001, ANN201

import importlib.util
import json
import sys
from pathlib import Path

from click.testing import CliRunner  # ty: ignore[unresolved-import]

SCRIPT_DIR = Path(__file__).parent


def load_converter(monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPT_DIR))

    spec = importlib.util.spec_from_file_location(
        "convert_dataset_to_yolo_under_test",
        SCRIPT_DIR / "convert_dataset_to_yolo.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_label(path: Path, annotations: list[dict]) -> None:
    path.write_text(
        json.dumps(
            {
                "labeled_classes": [
                    annotation["class"]
                    for annotation in annotations
                ],
                "annotations": annotations,
            }
        )
    )


def test_convert_annotations_exports_box_and_single_keypoint_pose_rows(
    tmp_path,
    monkeypatch,
):
    converter = load_converter(monkeypatch)
    label_path = tmp_path / "frame.json"
    write_label(
        label_path,
        [
            {"class": "Robot", "points": [[0.1, 0.2], [0.3, 0.6]]},
            {"class": "GoalPost", "point": [0.9, 0.1]},
            {"class": "LSpot", "point": [0.4, 0.5]},
            {
                "class": "TSpot",
                "points": [[0.2, 0.3], [0.6, 0.9]],
                "point": [0.45, 0.7],
            },
        ],
    )

    assert converter.convert_annotations(
        label_path,
        "frame.json",
        "mixed",
        point_box_size=0.05,
    ) == [
        "4 0.200000 0.400000 0.200000 0.400000",
        "1 0.900000 0.100000 0.050000 0.050000 0.900000 0.100000 2",
        "2 0.400000 0.500000 0.050000 0.050000 0.400000 0.500000 2",
        "5 0.400000 0.600000 0.400000 0.600000 0.450000 0.700000 2",
    ]


def test_convert_annotations_skips_person_labels(tmp_path, monkeypatch):
    converter = load_converter(monkeypatch)
    label_path = tmp_path / "person.json"
    write_label(
        label_path,
        [
            {"class": "Robot", "points": [[0.1, 0.2], [0.3, 0.6]]},
            {"class": "Person", "points": [[0.4, 0.5], [0.6, 0.7]]},
        ],
    )

    assert converter.convert_annotations(label_path, "person.json", "mixed") == [
        "4 0.200000 0.400000 0.200000 0.400000",
    ]


def test_collect_samples_reads_prepare_output_tree(tmp_path, monkeypatch):
    converter = load_converter(monkeypatch)
    output_root = tmp_path / "current"
    first_chunk = output_root / "first-chunk"
    second_chunk = output_root / "second-chunk"
    first_chunk.mkdir(parents=True)
    second_chunk.mkdir(parents=True)
    (output_root / "source_images.json").write_text(
        json.dumps({"version": 1, "images": {}})
    )
    (first_chunk / "prelabeled-data.json").write_text("{}")

    for chunk, stem in [(first_chunk, "first"), (second_chunk, "second")]:
        (chunk / f"{stem}.jpg").write_bytes(b"image")
        write_label(
            chunk / f"{stem}.json",
            [{"class": "Robot", "points": [[0.1, 0.2], [0.3, 0.6]]}],
        )

    samples = converter.collect_samples(output_root)

    assert [(sample.base, sample.json_path.name, sample.image_path.name) for sample in samples] == [
        ("first", "first.json", "first.jpg"),
        ("second", "second.json", "second.jpg"),
    ]


def test_collect_samples_reads_chunk_tree_without_manifest(tmp_path, monkeypatch):
    converter = load_converter(monkeypatch)
    output_root = tmp_path / "airplane-labelling"
    chunk = output_root / "chunk-100"
    chunk.mkdir(parents=True)
    (chunk / "frame.webp").write_bytes(b"image")
    write_label(
        chunk / "frame.json",
        [{"class": "Robot", "points": [[0.1, 0.2], [0.3, 0.6]]}],
    )

    samples = converter.collect_samples(output_root)

    assert [(sample.base, sample.json_path.name, sample.image_path.name) for sample in samples] == [
        ("frame", "frame.json", "frame.webp"),
    ]


def test_main_converts_prepare_output_tree_and_point_labels(
    tmp_path,
    monkeypatch,
):
    converter = load_converter(monkeypatch)
    output_root = tmp_path / "current"
    chunk = output_root / "gentle-river"
    chunk.mkdir(parents=True)
    (output_root / "source_images.json").write_text(
        json.dumps({"version": 1, "images": {}})
    )
    (chunk / "frame.jpg").write_bytes(b"image")
    write_label(
        chunk / "frame.json",
        [{"class": "PenaltySpot", "point": [0.25, 0.75]}],
    )
    dataset_path = tmp_path / "yolo"

    result = CliRunner().invoke(
        converter.main,
        [
            str(output_root),
            str(dataset_path),
            "--task",
            "pose",
            "--train-split",
            "1.0",
            "--point-box-size",
            "0.05",
        ],
    )

    assert result.exit_code == 0, result.output
    assert (dataset_path / "labels" / "train" / "frame.txt").read_text() == (
        "2 0.250000 0.750000 0.050000 0.050000 0.250000 0.750000 2"
    )
    assert (dataset_path / "images" / "train" / "frame.jpg").read_bytes() == b"image"
