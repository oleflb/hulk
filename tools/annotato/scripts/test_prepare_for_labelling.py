# ruff: noqa: ANN001, ANN002, ANN003, ANN201, ANN202, ANN204, ARG002, ARG005

import importlib.util
import json
import sys
import types
from pathlib import Path

import pytest  # ty: ignore[unresolved-import]
from click.testing import CliRunner  # ty: ignore[unresolved-import]

SCRIPT_DIR = Path(__file__).parent


def load_prepare_for_labelling(monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPT_DIR))
    monkeypatch.setitem(sys.modules, "cv2", types.ModuleType("cv2"))
    monkeypatch.setitem(sys.modules, "numpy", types.ModuleType("numpy"))

    pil = types.ModuleType("PIL")
    pil.Image = object()  # ty: ignore[unresolved-attribute]
    monkeypatch.setitem(sys.modules, "PIL", pil)

    tqdm = types.ModuleType("tqdm")
    tqdm.tqdm = lambda iterable: iterable  # ty: ignore[unresolved-attribute]
    monkeypatch.setitem(sys.modules, "tqdm", tqdm)

    ultralytics = types.ModuleType("ultralytics")
    ultralytics.YOLO = object  # ty: ignore[unresolved-attribute]
    monkeypatch.setitem(sys.modules, "ultralytics", ultralytics)

    wonderwords = types.ModuleType("wonderwords")
    wonderwords.RandomWord = object  # ty: ignore[unresolved-attribute]
    monkeypatch.setitem(sys.modules, "wonderwords", wonderwords)

    spec = importlib.util.spec_from_file_location(
        "prepare_for_labelling_under_test",
        SCRIPT_DIR / "prepare_for_labelling.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_yolo_classes_accepts_ranges_names_and_ids(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)

    class_ids = prepare_for_labelling.parse_yolo_classes("0-2,Robot,7")

    assert class_ids == {0, 1, 2, 4, 7}


def test_resolve_sample_size_accepts_count(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)

    assert (
        prepare_for_labelling.resolve_sample_size(
            total_images=100,
            sample_count=25,
            sample_fraction=None,
        )
        == 25
    )


def test_resolve_sample_size_accepts_fraction(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)

    assert (
        prepare_for_labelling.resolve_sample_size(
            total_images=100,
            sample_count=None,
            sample_fraction=0.25,
        )
        == 25
    )


def test_resolve_sample_size_keeps_all_when_unspecified(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)

    assert (
        prepare_for_labelling.resolve_sample_size(
            total_images=100,
            sample_count=None,
            sample_fraction=None,
        )
        is None
    )


def test_resolve_sample_size_rejects_count_and_fraction(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)

    with pytest.raises(prepare_for_labelling.click.UsageError):
        prepare_for_labelling.resolve_sample_size(
            total_images=100,
            sample_count=10,
            sample_fraction=0.5,
        )


def test_sample_random_is_seeded_and_preserves_original_order(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    image_paths = [Path(f"image-{index}.jpg") for index in range(10)]

    selected = prepare_for_labelling.sample_random(
        image_paths,
        sample_size=4,
        seed=7,
    )

    assert selected == [
        Path("image-2.jpg"),
        Path("image-5.jpg"),
        Path("image-6.jpg"),
        Path("image-9.jpg"),
    ]


def test_sample_spread_selects_evenly_across_order(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    image_paths = [Path(f"image-{index}.jpg") for index in range(10)]

    selected = prepare_for_labelling.sample_spread(image_paths, sample_size=4)

    assert selected == [
        Path("image-0.jpg"),
        Path("image-3.jpg"),
        Path("image-6.jpg"),
        Path("image-9.jpg"),
    ]


def test_sample_visual_diversity_prefers_distant_features(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    image_paths = [Path(f"image-{index}.jpg") for index in range(5)]
    features = {
        image_paths[0]: (0.0, 0.0),
        image_paths[1]: (0.1, 0.0),
        image_paths[2]: (5.0, 5.0),
        image_paths[3]: (5.1, 5.0),
        image_paths[4]: (10.0, 0.0),
    }

    selected = prepare_for_labelling.sample_visual_diversity(
        image_paths,
        sample_size=3,
        seed=0,
        feature_for_image=lambda path: features[path],
    )

    assert selected == [image_paths[0], image_paths[2], image_paths[4]]


def test_sample_visual_diversity_uses_seed_for_initial_anchor(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    image_paths = [Path(f"image-{index}.jpg") for index in range(4)]

    selected = prepare_for_labelling.sample_visual_diversity(
        image_paths,
        sample_size=1,
        seed=2,
        feature_for_image=lambda path: (float(image_paths.index(path)),),
    )

    assert selected == [image_paths[2]]


def test_sample_visual_diversity_reports_fingerprint_progress(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    image_paths = [Path(f"image-{index}.jpg") for index in range(4)]
    progress_calls = []

    def progress_spy(iterable):
        progress_calls.append(iterable)
        return iterable

    monkeypatch.setattr(prepare_for_labelling, "progress", progress_spy)

    prepare_for_labelling.sample_visual_diversity(
        image_paths,
        sample_size=2,
        seed=0,
        feature_for_image=lambda path: (float(image_paths.index(path)),),
    )

    assert progress_calls == [image_paths]


def test_sample_images_random_dispatches_to_random(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    image_paths = [Path(f"image-{index}.jpg") for index in range(10)]

    result = prepare_for_labelling.sample_images(
        image_paths=image_paths,
        sample_size=4,
        method="random",
        seed=7,
    )

    assert result == [
        Path("image-2.jpg"),
        Path("image-5.jpg"),
        Path("image-6.jpg"),
        Path("image-9.jpg"),
    ]


def test_yolo_model_kind_maps_coco_person_and_sports_ball(monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)

    class FakeTensor:
        def reshape(self, *shape):
            return self

        def tolist(self):
            return [[0.1, 0.2], [0.3, 0.4]]

    detection = [
        types.SimpleNamespace(
            boxes=[
                types.SimpleNamespace(cls=0, xyxyn=FakeTensor()),
                types.SimpleNamespace(cls=32, xyxyn=FakeTensor()),
                types.SimpleNamespace(cls=4, xyxyn=FakeTensor()),
            ]
        )
    ]

    annotations = prepare_for_labelling.annotations_from_detection(
        detection,
        selected_class_ids=None,
        model_kind="yolo",
    )

    assert annotations == [
        {"class": "Person", "points": [[0.1, 0.2], [0.3, 0.4]]},
        {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
    ]


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


def test_manifest_entry_for_source_uses_resolved_source_path(
    tmp_path,
    monkeypatch,
):
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

    entry = prepare_for_labelling.manifest_entry_for_source(
        manifest,
        source_path,
    )

    assert entry == {"chunk": "gentle-river", "image": "generated.jpg"}


def test_main_preserves_source_suffix_and_writes_manifest(
    tmp_path,
    monkeypatch,
):
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
    images = [
        path
        for path in (output_path / "gentle-river").iterdir()
        if path.suffix.lower() == ".jpg"
    ]
    assert len(images) == 1
    assert images[0].suffix == ".jpg"
    assert images[0].read_bytes() == b"source-image"
    assert json.loads(
        (output_path / "gentle-river" / "prelabeled-data.json").read_text()
    ) == {}
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


def test_extend_uses_manifest_to_update_existing_data_json(
    tmp_path,
    monkeypatch,
):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    source_path = source_dir / "frame.jpg"
    source_path.write_bytes(b"source-image")
    output_path = tmp_path / "current"
    chunk_path = output_path / "gentle-river"
    chunk_path.mkdir(parents=True)
    (chunk_path / "generated.jpg").write_bytes(b"source-image")
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
    data_path = output_path / "gentle-river" / "prelabeled-data.json"
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

    with pytest.raises(
        prepare_for_labelling.click.ClickException,
        match="duplicate",
    ):
        prepare_for_labelling.validate_unique_source_keys(
            [source_path, source_path],
        )


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
    chunk_path = output_path / "gentle-river"
    chunk_path.mkdir(parents=True)
    (chunk_path / "first-generated.jpg").write_bytes(b"first-image")
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
    data_path = output_path / "gentle-river" / "prelabeled-data.json"
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


def test_main_subsamples_before_creating_chunks(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    monkeypatch.setattr(
        prepare_for_labelling,
        "generate_random_chunk_name",
        lambda: "gentle-river",
    )
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    for index in range(5):
        (source_dir / f"frame-{index}.jpg").write_bytes(
            f"image-{index}".encode()
        )
    output_path = tmp_path / "current"

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(source_dir),
            "--output",
            str(output_path),
            "--sample-count",
            "2",
            "--sample-method",
            "spread",
        ],
    )

    assert result.exit_code == 0, result.output
    assert "Sampling 2 of 5 images with spread" in result.output
    images = sorted(
        path
        for path in (output_path / "gentle-river").iterdir()
        if path.suffix.lower() == ".jpg"
    )
    assert len(images) == 2
    manifest = json.loads((output_path / "source_images.json").read_text())
    assert set(manifest["images"]) == {
        str((source_dir / "frame-0.jpg").resolve()),
        str((source_dir / "frame-4.jpg").resolve()),
    }


def test_main_samples_each_image_folder_in_order(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    chunk_names = iter(["first-chunk", "second-chunk"])
    monkeypatch.setattr(
        prepare_for_labelling,
        "generate_random_chunk_name",
        lambda: next(chunk_names),
    )
    first_dir = tmp_path / "first"
    second_dir = tmp_path / "second"
    third_dir = tmp_path / "third"
    for source_dir in [first_dir, second_dir, third_dir]:
        source_dir.mkdir()
        for index in range(4):
            (source_dir / f"frame-{index}.jpg").write_bytes(
                f"{source_dir.name}-{index}".encode()
            )
    output_path = tmp_path / "current"

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(first_dir),
            "--image-folder",
            str(second_dir),
            "--image-folder",
            str(third_dir),
            "--output",
            str(output_path),
            "--chunk-size",
            "4",
            "--sample-count",
            "1",
            "--sample-count",
            "2",
            "--sample-method",
            "spread",
        ],
    )

    assert result.exit_code == 0, result.output
    assert (
        "Using final --sample-count value 2 for 1 remaining image folder"
        in result.output
    )
    manifest = json.loads((output_path / "source_images.json").read_text())
    assert list(manifest["images"]) == [
        str((first_dir / "frame-0.jpg").resolve()),
        str((second_dir / "frame-0.jpg").resolve()),
        str((second_dir / "frame-3.jpg").resolve()),
        str((third_dir / "frame-0.jpg").resolve()),
        str((third_dir / "frame-3.jpg").resolve()),
    ]


def test_main_rejects_more_sample_counts_than_image_folders(
    tmp_path,
    monkeypatch,
):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    (source_dir / "frame.jpg").write_bytes(b"image")

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(source_dir),
            "--sample-count",
            "1",
            "--sample-count",
            "2",
        ],
    )

    assert result.exit_code != 0
    assert (
        "more --sample-count values than --image-folder values"
        in result.output
    )


def test_main_samples_each_image_folder_with_fraction(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    monkeypatch.setattr(
        prepare_for_labelling,
        "generate_random_chunk_name",
        lambda: "gentle-river",
    )
    first_dir = tmp_path / "first"
    second_dir = tmp_path / "second"
    for source_dir in [first_dir, second_dir]:
        source_dir.mkdir()
        for index in range(4):
            (source_dir / f"frame-{index}.jpg").write_bytes(b"image")
    output_path = tmp_path / "current"

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(first_dir),
            "--image-folder",
            str(second_dir),
            "--output",
            str(output_path),
            "--sample-fraction",
            "0.25",
            "--sample-fraction",
            "0.5",
            "--sample-method",
            "spread",
        ],
    )

    assert result.exit_code == 0, result.output
    manifest = json.loads((output_path / "source_images.json").read_text())
    assert list(manifest["images"]) == [
        str((first_dir / "frame-0.jpg").resolve()),
        str((second_dir / "frame-0.jpg").resolve()),
        str((second_dir / "frame-3.jpg").resolve()),
    ]


def test_main_chunks_multiple_image_folders_proportionally(
    tmp_path,
    monkeypatch,
):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    chunk_names = iter(["chunk-one", "chunk-two", "chunk-three"])
    monkeypatch.setattr(
        prepare_for_labelling,
        "generate_random_chunk_name",
        lambda: next(chunk_names),
    )
    source_counts = {
        "first": 5,
        "second": 3,
        "third": 1,
    }
    source_dirs = {}
    for source_name, source_count in source_counts.items():
        source_dir = tmp_path / source_name
        source_dir.mkdir()
        source_dirs[source_name] = source_dir
        for index in range(source_count):
            (source_dir / f"frame-{index}.jpg").write_bytes(
                f"{source_name}-{index}".encode()
            )
    output_path = tmp_path / "current"

    result = CliRunner().invoke(
        prepare_for_labelling.main,
        [
            "--image-folder",
            str(source_dirs["first"]),
            "--image-folder",
            str(source_dirs["second"]),
            "--image-folder",
            str(source_dirs["third"]),
            "--output",
            str(output_path),
            "--chunk-size",
            "4",
        ],
    )

    assert result.exit_code == 0, result.output
    manifest = json.loads((output_path / "source_images.json").read_text())
    chunk_sources = {
        "chunk-one": [],
        "chunk-two": [],
        "chunk-three": [],
    }
    for source_name, source_count in source_counts.items():
        for index in range(source_count):
            source_path = source_dirs[source_name] / f"frame-{index}.jpg"
            entry = manifest["images"][str(source_path.resolve())]
            chunk_sources[entry["chunk"]].append(source_name)

    assert chunk_sources == {
        "chunk-one": ["first", "first", "second", "third"],
        "chunk-two": ["first", "first", "second", "second"],
        "chunk-three": ["first"],
    }


def test_extend_subsamples_before_updating_data_json(tmp_path, monkeypatch):
    prepare_for_labelling = load_prepare_for_labelling(monkeypatch)
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    source_paths = [source_dir / f"frame-{index}.jpg" for index in range(3)]
    for path in source_paths:
        path.write_bytes(b"image")

    output_path = tmp_path / "current"
    chunk_path = output_path / "gentle-river"
    chunk_path.mkdir(parents=True)
    generated_names = [f"generated-{index}.jpg" for index in range(3)]
    for name in generated_names:
        (chunk_path / name).write_bytes(b"image")

    (output_path / "source_images.json").write_text(
        json.dumps(
            {
                "version": 1,
                "images": {
                    str(source_path.resolve()): {
                        "chunk": "gentle-river",
                        "image": generated_name,
                    }
                    for source_path, generated_name in zip(
                        source_paths,
                        generated_names,
                        strict=True,
                    )
                },
            }
        )
    )
    data_path = output_path / "gentle-river" / "prelabeled-data.json"
    data_path.write_text(
        json.dumps(
            {
                "generated-0.jpg": [
                    {"class": "Ball", "points": [[0.1, 0.2], [0.3, 0.4]]},
                ],
                "generated-1.jpg": [
                    {"class": "Ball", "points": [[0.2, 0.3], [0.4, 0.5]]},
                ],
                "generated-2.jpg": [
                    {"class": "Ball", "points": [[0.3, 0.4], [0.5, 0.6]]},
                ],
            }
        )
    )
    yolo_path = tmp_path / "model.pt"
    yolo_path.write_bytes(b"model")
    monkeypatch.setattr(
        prepare_for_labelling,
        "load_image",
        lambda image_path, convert_colors: object(),
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
            "--sample-count",
            "2",
            "--sample-method",
            "spread",
        ],
    )

    assert result.exit_code == 0, result.output
    assert json.loads(data_path.read_text()) == {
        "generated-0.jpg": [
            {"class": "Robot", "points": [[0.11, 0.22], [0.33, 0.44]]},
        ],
        "generated-1.jpg": [
            {"class": "Ball", "points": [[0.2, 0.3], [0.4, 0.5]]},
        ],
        "generated-2.jpg": [
            {"class": "Robot", "points": [[0.11, 0.22], [0.33, 0.44]]},
        ],
    }
