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

Press `C` or use `Edit other class` during the chunk workflow to temporarily unlock class selection. After one label create/edit/delete, Annotato returns to the chunk class. These manual cross-class fixes only change annotations; they never add to or remove from `labeled_classes`, so an already reviewed class stays reviewed.

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
uv run --script tools/annotato/scripts/prepare_for_labelling.py --image-folder images --output current --yolo model.pt --yolo-classes 0-5 --chunk-size 200
uv run --script tools/annotato/scripts/prepare_for_labelling.py --image-folder images --output current --yolo yolo11x.pt --model-kind yolo --yolo-classes 0,32
uv run --script tools/annotato/scripts/prepare_for_labelling.py --image-folder images-a --image-folder images-b --output current --sample-count 500 --sample-count 100 --sample-method visual-diversity --sample-seed 123
uv run --script tools/annotato/scripts/prepare_for_labelling.py --image-folder images --output current --extend --yolo second-model.pt --yolo-classes 6-7
cargo run -p annotato -- current/<chunk>
uv run --script tools/annotato/scripts/convert_dataset_to_yolo.py labelled-images yolo-output --train-split 0.8
```

`prepare_for_labelling.py` writes chunked labelling tasks below the selected output folder. Each chunk is a normal annotato image folder containing copied images and a `prelabeled-data.json` predictions file. When passed one chunk folder, annotato automatically uses `prelabeled-data.json` for images that do not have sidecar label files yet. Images are copied with their source file format. Supported formats are JPEG, PNG, WebP, BMP, and TIFF.

`--model-kind` controls how YOLO class IDs are mapped to annotato labels. The default `custom` kind uses annotato's custom class order. The `yolo` kind is for off-the-shelf COCO YOLO models and maps COCO `person` (`0`) to `Person` and `sports ball` (`32`) to `Ball`; all other detections are dropped. `--yolo-classes` accepts class IDs, ranges, and names for the selected model kind, for example `0-5`, `Robot,Person`, or `0-5,Person` with `custom`, and `0,32`, `Person`, or `Ball` with `yolo`. The output folder also contains `source_images.json`, which maps original source images to generated chunk image names. A later run with `--extend` uses that manifest to update existing `prelabeled-data.json` files instead of creating new image keys.

Use `--image-folder` multiple times to build one prepared dataset from several source folders. Use `--sample-count` or `--sample-fraction` to prepare only a deterministic subset before creating chunks or extending existing predictions. These sampling options can also be supplied multiple times; values apply to source folders by order. If fewer values are supplied than folders, the last value is reused for the remaining folders and the script prints an informational message. During chunk creation, selected images from multiple source folders are distributed proportionally across chunks; only the final chunk is partial unless the total selected image count is smaller than `--chunk-size`. `--sample-method` accepts `random`, `spread`, and `visual-diversity`; `spread` is the default when sampling is requested. Change `--sample-seed` to get a different deterministic subset for seeded methods.

`convert_dataset_to_yolo.py` expects images and sidecar JSON files with matching stems in one folder. It writes `images/{train,val}` and `labels/{train,val}`.
