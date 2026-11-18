# Bounded local reacquisition

Status: **historical experiment; local reacquisition and graph association removed
on 2026-11-08.** Fixed-correspondence localization, motion-history/conflict recovery
and intrinsic optimization remain. The implementation descriptions, commands,
test counts and quantitative results below refer to archived revisions, not the
current executable; no replay rerun is claimed for this removal.

Source, including pre-existing dirty edits and `test_support.rs`, is archived at
`archive/localization-graph-association-20261108`, stash snapshot
`e3e9e8d1e3a322572889e99c3de4a7cdf5c22671` (active HEAD/index unchanged).
The archive includes 79 small ignored replay evidence files from
`loca-rerun/graph-association-20261106/` and
`loca-rerun/graph-association-fagra-1ad5ce7/`. Its 107-artifact inventory is available via:

```sh
git show archive/localization-graph-association-20261108:loca-rerun/graph-association-archive-inventory.json
```

All `loca-rerun` evidence is retained untouched locally, including files over 1 MB
(MCAPs, replay JSON, doctor logs and binaries) not stored in the Git snapshot.

## Historical retained model and cleanup

The retained model uses angular bearing factors and two sole-origin penetration
inequalities sampled every 50 ms before motion-history storage. Equality grounding,
shared odometry-endpoint height, calibrated bearings, their replay switches, and
localization-only contact polygons have been removed. The experiment sections below
describe frozen historical binaries, not options available in the current executable.
Separate odometry/support-foot contact geometry is unchanged.

Foot observations now contain exactly two points. Endpoint maps, height factor
replacement/restoration and height-specific pruning are gone. Graph checkpoints,
the objective probe, initialization/recovery and both association paths remained
at that historical revision. Only fixed correspondences remain after removal.

Removed parameter keys are `grounded_soles`, `pixel_calibrated_bearings`,
`left_sole_contact_vertices` and `right_sole_contact_vertices` under localization's
`model`; strict configuration parsing rejects these keys. Solve diagnostics no
longer contain `sampled_height_count` or `shared_height_count`; visual diagnostics
retain `residual_rms` (pixel RMS) instead of also computing identical `pixel_rms`.
These are wire-schema changes; historical experiment reports remain untouched.

Existing release tests pass: 75 localization, nine replay, 157 factor-library and
two integration tests (seven ignored in total). Scoped Clippy and formatting pass.
Five-recording exact-output and paired runtime verification is recorded under
`loca-rerun/localization-cleanup/`. All five reproduce the frozen winner's estimates,
solve schedules, publication statuses and visual admission exactly, preserving its
11.8093% aggregate outage. Paired replay pipeline time fell from 821.123 s to
782.940 s (4.65%); pipeline time and solve p99 improved on each recording. This is
one paired replay batch, not a hardware real-time or statistically controlled claim.

## Test maintenance

The subsequent deduplication pass leaves the estimator model and admission policies
unchanged. It shares matching fixtures, synthetic camera/parameter setup, global
search construction, visual-rejection retry logic, and bounded subscriber setup.
Relative to `e5c4e4f2d`, net Rust reductions including new fixtures are 464 lines in
`localization-fagra`, 779 in `localization-3d`, and 355 in `field_mark_association`.
These remain below the requested 1500/5000/3000-line reductions.

Removed tests/tooling and retained responsibility:

- Direct state-checkpoint/cache-equivalence tests now rely on the pinned fagra
  `state_checkpoint.rs`, `graph_checkpoint.rs` and `covariance.rs` suites. Local
  rollback, factor identity restoration, and recovery rejection tests remain.
- Parameter tests directly exercise local restart/live-update rules and the
  running estimator, without recreating ROSZ's transactional file/revision/reload
  tests (`crates/ros-z/tests/parameter_integration.rs`).
- Removed print-only Lie/spline/association timings and the old synthetic tracking
  benchmark. The graph matching benchmark's sparse/dense certificate assertions
  now run as a regular deterministic test, without its timing loop. The current
  replay pipeline is the performance measurement path.
- Removed the obsolete synthetic all-factor allocation workload, which omitted
  graph association. The spline allocation regression remains; this intentionally
  drops that workload's separate per-factor allocation guard.
- Removed the ignored fixture re-extraction test and historical wire-layout
  structs. The checked-in incident fixture, its heading/recovery regression, and
  original MCAPs remain. The remaining ignored full divergence-window test was
  subsequently removed at the user's request; none of these three crates retains
  ignored tests.
- Removed historical two-pose covariance equivalence code; independent projection
  finite differences and covariance-correlation checks remain. The failed-debug
  publication case is folded into the existing publication-gate test rather than
  reimplementing the publisher in a test.

The 18 ROSZ parameter integration tests pass independently, retaining the upstream
transactional-update checks removed from the two consumer crates.
The four shipped carrying cases (20/50 ms legs × zero/recorded image latency) now
share one parameterized test, retaining all four runs and their assertions. Repeated
bootstrap setup and half-turn geometry cases are also consolidated.

A further dead-code pass removes the permanently false `motion_rebuilt` diagnostic
(a wire-schema change), the test-only public `associate_tracking_visual_features`
wrapper (at that revision, callers used `diagnose_tracking_visual_features`), and unreachable
heading/innovation/supported-leg branches in startup reconstruction. Production
bootstrap is called only at generation zero, before a trusted heading exists.
The incident fixture now checks optimized startup heading modulo the field
half-turn instead of injecting a trusted startup heading; its strict heading-guided
association checks and the separate recovery continuity tests remain.

The frozen solve/search deduplication candidate reproduces estimates, solve
schedules, publication statuses and visual admission exactly on all five inputs
(`loca-rerun/localization-dedup/`). Its paired pipeline time was 876.520 s versus
847.620 s (+3.41%), predominantly robot 45 (210.154 versus 182.510 s). Builds/tests
overlapped this batch, so it does not isolate runtime effects or establish runtime
neutrality. Subsequent test-only pruning does not change the replay algorithm;
live subscriber construction was consolidated separately and is not exercised by
this replay. The remaining 297 tests pass with none ignored, alongside formatting,
scoped Clippy and the 18 ROSZ parameter integration tests.

## Search, not extra evidence

After 500 ms without an accepted visual frame, a nonempty raw frame may request
one local hypothesis. Requests are separated by at least 500 ms of exposure time,
including failed searches. The estimator requires an exposure within the solved
trajectory and an empty pending-visual queue before extracting covariance. Epoch,
generation, age and finite-input checks run before this path.

The existing tracking matcher uses the exposure-time joint pose covariance,
including trajectory/alignment cross-correlations. Its existing default ambiguity,
work and pixel-distance limits are reused, with the localization pixel-noise
variance. Raw pixels have already been filtered; their wire format has no detector
confidence, so the hypothesis search assigns equal confidence. This is only a
search hypothesis; no covariance-derived pose prior is inserted.

One temporary fixed-correspondence image is optimized for at most six accepted LM
steps, using the existing per-step trial cap. Useful partial results from
`NoConvergence`/`NoProgress` are allowed as seeds; numerical failures are not.
The original association factor is held out during this trial. Temporary factors
are removed before inserting it, so the image is never counted twice. The final
ordinary solve must converge and pass existing heading, tilt, visual and covariance
validation. Final correspondences remain revisable and cannot renew independent
motion-conflict evidence. No generation/heading reset occurs.

The acceptance reference is the **same final objective at the pre-seed state**:
fixed-trial initial total cost minus that image's fixed cost plus its revisable
cost, including clutter. Final cost must not exceed it beyond numerical tolerance.
Marginal-prior constants are included. This is not a comparison of costs from
different observation models, nor a claim of global optimality.

fagra `5a22e73b08c95d0505d0ea918275c1c1f924e572` supplies state checkpoints,
full-graph checkpoints, owned LM outcomes and typed borrowed covariance blocks.
State-only checkpoints replace manual variable-family snapshots. Conflict recovery
uses a full graph checkpoint instead of reconstructing removed motion factors;
application segment/kinematic handles are restored alongside it. IMU preparation
runs before this transaction so its cached handles remain valid on rollback.
Typed bias covariance blocks replace hard-coded offsets and the padded 39×39 copy.
Checkpoint support is now conditional on cloneable stored values in ordinary
`states!`/`factors!` declarations; the former `snapshot` schema modifier is removed.
Independent cloning of association scratch remains necessary for correct rollback.

This fagra revision still lacks a public total-cost query or initial-cost statistics
on failed solves. An initial-gradient-only solve
(`gradient_tolerance = f64::MAX`, checked for zero iterations) reads the objective
without moving states. It performs one extra linearization per proposed seed;
there is no preliminary covariance extraction. A direct nonlinear-cost API would
remove this work without changing semantics.

Rejected trials restore states and discard provisional visual evidence. History
updates occur only on acceptance. Posterior-dependent retirement is deferred to
the next ordinary solve after an accepted reacquisition, preventing provisional
associations from entering a marginal prior. At most one attempt is pending.

## Diagnostics and checks

`SolveDiagnostics.reacquisition` reports exposure, matching/preparation duration,
seed duration/iterations/convergence, matched count, reference/final objective and
typed outcome. LM attempt/rejection counters accumulate across optimizer calls
within an estimator solve instead of overwriting earlier retries. They still do
not aggregate every separate estimator invocation in a higher-level recovery.

Behavioral tests cover accepted revisable-only evidence and independent-anchor
preservation, failed-hypothesis throttling, failed-seed rollback and temporary
factor removal, and use of bounded partial seeds. The objective test evaluates
the actual restored raw-factor graph against the recorded comparison reference.

## Investigation so far

The first robot-45 probe found a local hypothesis at elapsed 647.613 s, but requiring
the six-step preliminary solve to converge discarded that and most subsequent
seeds. There were 200 searches, 81 hypotheses, 69 failed preliminary solves, ten
accepted reacquisitions and two validation rejections over the recording. The
650–703 s height failure persisted. This motivated allowing useful partial seeds
while preserving full final convergence and unchanged-objective validation.

The partial-seed probe produced 162 searches: 117 without a hypothesis, 33 accepted
reacquisitions and 12 final-validation rejections. At 673.330 s, a single-match
trial reduced the unchanged objective from 9178.0002 to 9176.0215. Height was
approximately 0.658 m at 675 s instead of 1.287 m, and Tracking resumed at
682.155 s instead of 702.501 s (20.35 s earlier). The earlier 647.613 s trial still
failed final validation; height remained about 1.15 m at 650 s and 1.33 m at 660 s.
This is a partial recovery improvement, not a complete fix or an accuracy claim.

Artifacts: `/tmp/opencode/reacquisition-20261107/`. The `probe/` run is superseded
by `partial-seed/`; `validate.sh` performs paired baseline/candidate replays of all
five identical preserved inputs, using the previous fagra-1ad5ce7 candidate as
baseline. Do not equate output availability or a low residual with physical accuracy.

The completed paired reacquisition batch reduced non-Damping outage from
601.23/4399.02 s (13.67%) to 527.02/4399.02 s (11.98%). Pipeline time decreased
1.7% overall, but robot 43 regressed 11.3%. These results precede the four-API
fagra integration. Its paired five-recording replay comparison was cancelled by
a server restart under
`/tmp/opencode/fagra-four-api-20261107/`, against the frozen pre-integration
reacquisition binary. Relevant unit/integration tests, replay tests, formatting
and scoped Clippy pass; the partial recording outputs do not establish regression
acceptance.
