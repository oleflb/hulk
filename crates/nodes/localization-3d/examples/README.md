# Localization replay

Replay current association and localization using recorded solve times and epoch
resets:

```sh
cargo run -p localization-3d --example relocalization_replay -- recording.mcap > replay.json
```

Rebuild after implementation/base-parameter changes and replay the same file.
Compare `audit_estimates`, lifecycle transitions, timings and `audit_solves`
(final status/diagnostics, estimate presence and the reported failure reason).
`audit_frames_queued` replaces `audit_frames`: `[frame, queued]` pairs; queue
admission is not visual commitment. Uninitialized solves report that explicitly.
JSON goes to stdout; warnings to stderr. Base parameters are embedded at build
time; no experiment modes or recording-specific checkpoints.

Required topics, each containing messages:

- `field_dimensions`
- `inputs/low_state`
- `robot_kinematics`
- `camera_geometry`
- `visual_odometry/current_left_camera_to_visual_odometer`
- `detected_objects`
- `localization/status`
- `debug/solve_diagnostics`

Missing topics, malformed input and missing required camera/attitude brackets
fail the replay. Field dimensions come exclusively from MCAP; changes within a
recording are rejected. Unrelated topics are ignored. Old-epoch and expired
observations are discarded, matching replay admission rules. Field association
waits for motion initialization; missing field estimates remain legitimate
localization results, not input errors.

This reruns association/localization, not image detection or visual odometry.
Live and replay share exposure-attitude validation/interpolation, coherent pose
selection, and tracking/global matching preparation. Live matching uses a bounded
background worker in the separate `field_mark_association` node; replay evaluates
matching synchronously in recorded delivery order. Replay therefore does not test live worker scheduling
or publication backpressure.
Recorded status supplies epoch resets, not Tracking/LostTrack decisions; those
are recomputed. Events preserve delivery order and source timestamps. Solves run
at recorded diagnostic delivery times regardless of the new solver's runtime.
Timing includes ingestion overhead and host scheduling, not just solver work.
Recorded trajectories are not ground truth. `field_height_above_1m` and
`max_field_height_m` summarize available field posteriors, not contact or safety
validity. Finite poses with positive-definite covariance remain available above
one metre (or below zero); height alone neither rejects visuals nor loses track.
