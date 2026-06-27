# Annotato Label Format

## Usage

Run annotato with image files, non-recursive image folders, or glob patterns:

```sh
cargo run -p annotato -- images/
cargo run -p annotato -- 'images/*.webp' image.jpg
cargo run -p annotato -- --predictions predictions.json --config config.toml images/
```

Without `--config`, annotato reads `$XDG_CONFIG_HOME/annotato/config.toml` or `~/.config/annotato/config.toml` when present. Supported image inputs are listed in `image_extensions.txt`.

Annotato labels images in chunks of 50. Each chunk is labelled one class at a time in `Class::ALL` order: all 50 images for `Ball`, then the same chunk for `GoalPost`, and so on. `Space` marks the current image as reviewed for the active class, saves, skips images already reviewed for that class, and advances. A reviewed image does not need to contain an annotation for that class.

Hold `F` for temporary 10x focus around the cursor. Removed class-cycling and corner-cycling keybindings from older configs are ignored during config loading.

## Config

```toml
[keybindings]
next = { primary = "Space" }
previous = { primary = "Space", modifiers = ["shift"] }
class_popup = { primary = "C" }
temporary_focus = { primary = "F" }
draw = { primary = "B" }
```

`KeyBind` values accept `primary`, optional `alternatives`, and `modifiers = ["ctrl", "alt", "shift"]`.

## Labels

Annotato stores one JSON sidecar next to each image. Coordinates are normalized to the image size. `labeled_classes` records which classes have already been reviewed for the image, including reviewed-empty classes.

```json
{
  "labeled_classes": ["Robot", "LSpot", "Person"],
  "annotations": [
    { "class": "Robot", "points": [[0.10, 0.20], [0.35, 0.80]] },
    { "class": "LSpot", "point": [0.42, 0.58] },
    { "class": "TSpot", "points": [[0.30, 0.40], [0.45, 0.55]], "point": [0.38, 0.48] }
  ]
}
```

`points` is a bounding box stored as top-left and bottom-right corners. `point` is a single feature location. The schema is strict: each annotation may contain only `class`, `points`, and `point`, and `class` must be one of the built-in class names. `LSpot`, `TSpot`, and `XSpot` use point labels for new annotations; legacy boxes for those classes are preserved and augmented with `point` during migration. `GoalPost` can be annotated as either a box or a point. Legacy sidecar files containing a bare annotation array are still accepted; their `labeled_classes` are inferred from annotation classes when loaded.

The YOLO conversion script is box-only and fails clearly if a label contains `point` annotations.

## Scripts

```sh
uv run --script tools/annotato/scripts/prepare_for_labelling.py --image_folder images --yolo model.pt --chunksize 200
cargo run -p annotato -- --predictions current/<chunk>/data.json current/<chunk>/images
uv run --script tools/annotato/scripts/convert_dataset_to_yolo.py labelled-images yolo-output --train-split 0.8
```

`prepare_for_labelling.py` writes chunked labelling tasks below `current/`. Each chunk contains `images/` and a `data.json` predictions file for `annotato --predictions`.

`convert_dataset_to_yolo.py` expects images and sidecar JSON files with matching stems in one folder. It writes `images/{train,val}` and `labels/{train,val}`.
