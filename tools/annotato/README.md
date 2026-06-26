# Annotato Label Format

Annotato stores one JSON sidecar next to each image. Coordinates are normalized to the image size.

```json
[
  { "class": "Robot", "points": [[0.10, 0.20], [0.35, 0.80]] },
  { "class": "LSpot", "point": [0.42, 0.58] },
  { "class": "TSpot", "points": [[0.30, 0.40], [0.45, 0.55]], "point": [0.38, 0.48] }
]
```

`points` is a bounding box stored as top-left and bottom-right corners. `point` is a single feature location. `LSpot`, `TSpot`, and `XSpot` use point labels for new annotations; legacy boxes for those classes are preserved and augmented with `point` during migration. `GoalPost` can be annotated as either a box or a point.

The YOLO conversion script is box-only and fails clearly if a label contains `point` annotations.
