import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest
from click.testing import CliRunner


SCRIPT_DIR = Path(__file__).parent


def load_prepare_for_labelling(monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPT_DIR))
    monkeypatch.setitem(sys.modules, "cv2", types.ModuleType("cv2"))
    monkeypatch.setitem(sys.modules, "numpy", types.ModuleType("numpy"))

    pil = types.ModuleType("PIL")
    pil.Image = object()
    monkeypatch.setitem(sys.modules, "PIL", pil)

    tqdm = types.ModuleType("tqdm")
    tqdm.tqdm = lambda iterable: iterable
    monkeypatch.setitem(sys.modules, "tqdm", tqdm)

    ultralytics = types.ModuleType("ultralytics")
    ultralytics.YOLO = object
    monkeypatch.setitem(sys.modules, "ultralytics", ultralytics)

    wonderwords = types.ModuleType("wonderwords")
    wonderwords.RandomWord = object
    monkeypatch.setitem(sys.modules, "wonderwords", wonderwords)

    spec = importlib.util.spec_from_file_location(
        "prepare_for_labelling_under_test",
        SCRIPT_DIR / "prepare_for_labelling.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_yolo_classes_accepts_ranges_names_and_ids(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)

    class_ids = prepare_for_labelling.parse_yolo_classes("0-2,Robot,7")

    assert class_ids == {0, 1, 2, 4, 7}


def test_merge_annotations_replaces_only_selected_classes(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    existing_annotations = [
        {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
        {"class": "Robot", "points": [[0.5, 0.6], [0.7, 0.8]]},
        {"class": "Person", "points": [[0.2, 0.3], [0.4, 0.5]]},
    ]
    new_annotations = [
        {"class": "Robot", "points": [[0.11, 0.22], [0.33, 0.44]]},
    ]

    merged_annotations = prepare_for_labelling.merge_annotations(
        existing_annotations,
        new_annotations,
        replacement_class_ids={4},
    )

    assert merged_annotations == [
        {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
        {"class": "Person", "points": [[0.2, 0.3], [0.4, 0.5]]},
        {"class": "Robot", "points": [[0.11, 0.22], [0.33, 0.44]]},
    ]


def test_update_data_json_extends_existing_image_key(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    chunk_path = tmp_path / "gentle-river"
    chunk_path.mkdir()
    data_path = chunk_path / "data.json"
    data_path.write_text(
        json.dumps(
            {
                "frame.jpg": [
                    {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
                    {"class": "Robot", "points": [[0.5, 0.6], [0.7, 0.8]]},
                ]
            }
        )
    )

    prepare_for_labelling.update_data_json(
        chunk_path,
        "frame.jpg",
        [{"class": "Robot", "points": [[0.11, 0.22], [0.33, 0.44]]}],
        replacement_class_ids={4},
    )

    assert json.loads(data_path.read_text()) == {
        "frame.jpg": [
            {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
            {"class": "Robot", "points": [[0.11, 0.22], [0.33, 0.44]]},
        ]
    }


def test_manifest_entry_for_source_uses_resolved_source_path(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    source_path = tmp_path / "images" / "frame.jpg"
    source_path.parent.mkdir()
    source_path.write_bytes(b"image")
    manifest = {
        "version": 1,
        "images": {
            str(source_path.resolve()): {
                "chunk": "gentle-river",
                "image": "generated.jpg",
            }
        },
    }

    entry = prepare_for_labelling.manifest_entry_for_source(manifest, source_path)

    assert entry == {"chunk": "gentle-river", "image": "generated.jpg"}


def test_main_preserves_source_suffix_and_writes_manifest(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    monkeypatch.setattr(
        prepare_for_labelling,
        "generate_random_chunk_name",
        lambda: "gentle-river",
    )
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    source_path = source_dir / "frame.JPG"
    source_path.write_bytes(b"source-image")
    output_path = tmp_path / "current"

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(source_dir),
            "--output",
            str(output_path),
            "--chunk-size",
            "10",
        ],
    )

    assert result.exit_code == 0, result.output
    images = list((output_path / "gentle-river" / "images").iterdir())
    assert len(images) == 1
    assert images[0].suffix == ".jpg"
    assert images[0].read_bytes() == b"source-image"
    assert json.loads((output_path / "gentle-river" / "data.json").read_text()) == {}
    manifest = json.loads((output_path / "source_images.json").read_text())
    assert manifest == {
        "version": 1,
        "images": {
            str(source_path.resolve()): {
                "chunk": "gentle-river",
                "image": images[0].name,
            }
        },
    }


def test_extend_uses_manifest_to_update_existing_data_json(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    source_path = source_dir / "frame.jpg"
    source_path.write_bytes(b"source-image")
    output_path = tmp_path / "current"
    images_path = output_path / "gentle-river" / "images"
    images_path.mkdir(parents=True)
    (images_path / "generated.jpg").write_bytes(b"source-image")
    (output_path / "source_images.json").write_text(
        json.dumps(
            {
                "version": 1,
                "images": {
                    str(source_path.resolve()): {
                        "chunk": "gentle-river",
                        "image": "generated.jpg",
                    }
                },
            }
        )
    )
    data_path = output_path / "gentle-river" / "data.json"
    data_path.write_text(
        json.dumps(
            {
                "generated.jpg": [
                    {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
                    {"class": "Robot", "points": [[0.5, 0.6], [0.7, 0.8]]},
                ]
            }
        )
    )
    yolo_path = tmp_path / "model.pt"
    yolo_path.write_bytes(b"model")
    fake_image = object()
    monkeypatch.setattr(
        prepare_for_labelling,
        "load_image",
        lambda image_path, convert_colors: fake_image,
    )

    class FakeTensor:
        def reshape(self, *shape):
            return self

        def tolist(self):
            return [[0.11, 0.22], [0.33, 0.44]]

    class FakeBox:
        cls = 4
        xyxyn = FakeTensor()

    class FakeYolo:
        def __init__(self, checkpoint):
            self.checkpoint = checkpoint

        def __call__(self, image, **kwargs):
            assert image is fake_image
            return [types.SimpleNamespace(boxes=[FakeBox()])]

    monkeypatch.setattr(
        prepare_for_labelling,
        "load_yolo_model",
        lambda checkpoint: FakeYolo(checkpoint),
    )

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(source_dir),
            "--output",
            str(output_path),
            "--extend",
            "--yolo",
            str(yolo_path),
            "--yolo-classes",
            "Robot",
        ],
    )

    assert result.exit_code == 0, result.output
    assert json.loads(data_path.read_text()) == {
        "generated.jpg": [
            {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
            {"class": "Robot", "points": [[0.11, 0.22], [0.33, 0.44]]},
        ]
    }


def test_validate_unique_source_keys_rejects_duplicate_resolved_paths(
    tmp_path,
    monkeypatch,
):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    source_path = tmp_path / "frame.jpg"
    source_path.write_bytes(b"source-image")

    with pytest.raises(prepare_for_labelling.click.ClickException, match="duplicate"):
        prepare_for_labelling.validate_unique_source_keys([source_path, source_path])


def test_extend_does_not_partially_update_when_later_source_is_missing(
    tmp_path,
    monkeypatch,
):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    first_source = source_dir / "first.jpg"
    second_source = source_dir / "second.jpg"
    first_source.write_bytes(b"first-image")
    second_source.write_bytes(b"second-image")
    output_path = tmp_path / "current"
    images_path = output_path / "gentle-river" / "images"
    images_path.mkdir(parents=True)
    (images_path / "first-generated.jpg").write_bytes(b"first-image")
    (output_path / "source_images.json").write_text(
        json.dumps(
            {
                "version": 1,
                "images": {
                    str(first_source.resolve()): {
                        "chunk": "gentle-river",
                        "image": "first-generated.jpg",
                    }
                },
            }
        )
    )
    data_path = output_path / "gentle-river" / "data.json"
    original_data = {
        "first-generated.jpg": [
            {"class": "Robot", "points": [[0.5, 0.6], [0.7, 0.8]]},
        ]
    }
    data_path.write_text(json.dumps(original_data))
    yolo_path = tmp_path / "model.pt"
    yolo_path.write_bytes(b"model")
    fake_image = object()
    monkeypatch.setattr(
        prepare_for_labelling,
        "load_image",
        lambda image_path, convert_colors: fake_image,
    )

    class FakeTensor:
        def reshape(self, *shape):
            return self

        def tolist(self):
            return [[0.11, 0.22], [0.33, 0.44]]

    class FakeBox:
        cls = 4
        xyxyn = FakeTensor()

    class FakeYolo:
        def __call__(self, image, **kwargs):
            return [types.SimpleNamespace(boxes=[FakeBox()])]

    monkeypatch.setattr(
        prepare_for_labelling,
        "load_yolo_model",
        lambda checkpoint: FakeYolo(),
    )

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(source_dir),
            "--output",
            str(output_path),
            "--extend",
            "--yolo",
            str(yolo_path),
            "--yolo-classes",
            "Robot",
        ],
    )

    assert result.exit_code != 0
    assert json.loads(data_path.read_text()) == original_data
