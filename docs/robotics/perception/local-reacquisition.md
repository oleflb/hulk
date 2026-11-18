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

## Robot 46 factor experiment

The frozen executable and runner are in `/tmp/opencode/46-factor-experiment/`.
Four sequential replays use identical preserved inputs and one executable:
control, `--grounded-soles`, `--pixel-calibrated-bearings`, and both options.
Both model options default to false. These options are not production-approved.

The completed original robot-46 experiment (857.08 s of non-Damping time):

| Model | Outage | Pipeline time | Solve p99 |
| --- | ---: | ---: | ---: |
| Control | 19.08% (163.51 s) | 227.71 s | 24.27 ms |
| Grounded soles at input rate | 15.37% (131.77 s) | 413.52 s | 49.40 ms |
| Calibrated bearings | 19.18% (164.40 s) | 238.51 s | 25.19 ms |
| Both | 20.79% (178.20 s) | 419.14 s | 52.61 ms |

Grounding reduced second/third outage-window maximum heights from 1.83/3.93 m
to 0.57/0.58 m. Its extra runtime was not explained by more LM attempts:
attempts fell from 169,572 to 162,819, although rejected steps rose from 4,232
to 9,790. Unlike inactive penetration factors, equality factors always evaluate
spline Jacobians and assemble their contributions. Exact cost attribution has
not been profiled.

### Lowest-sole equality

For the configured sole-polygon vertices transformed into Robot coordinates,
the experimental residual is `min_i((T_local_robot * vertex_i).z) / foot_sigma`.
It penalizes floating as well as penetration. The minimum is piecewise smooth;
ties select the first vertex. The control retains the two sole-origin penetration
residuals. Neither model supplies a field-heading observation.

Unconditional equality assumes at least one grounded foot. Enabling it globally
caused ten existing localization test failures, including flight and carried-robot
cases. This experiment is separate from contact-supported height work; those
failures were not resolved by weakening assertions.

### Sampled, shared endpoint implementation

The subsequent implementation samples experimental height at a nominal 50 ms
cadence, without changing `foot_sigma`. When a live odometry endpoint has an
exactly timestamp-matched kinematics sample, its factor evaluates height using
the already computed current pose and spline Jacobians. Height has an independent
quadratic likelihood, outside the displacement Huber weighting.

The estimator retains raw sole geometry within the live window so either message
can arrive first. A shared endpoint replaces standalone height samples in the
preceding 50 ms; short odometry tails within 50 ms of an existing shared height
do not add another height constraint. Without a matching endpoint, standalone
height observations continue at 50 ms. This still uses the experimental
unconditional grounding assumption, not a new contact classifier.

Removing contradicted odometry restores its shared height as a standalone factor;
graph rollback restores the sampling handles alongside other graph handles.
`sampled_height_count` and `shared_height_count` report live observations at solve
entry. The legacy penetration model remains at its original input cadence.

Keeping the per-observation noise unchanged deliberately reduces accumulated
height information versus 500 Hz insertion. The robot-46 comparison at fagra
`5a22e73` measured 19.08% → 14.95% outage and 214.82 → 182.97 s pipeline time;
solve p99 fell from 22.59 to 18.94 ms. Approximately 88.94% of sampled height
observations shared an odometry endpoint. These are replay timings, not isolated
factor profiles; some development compilation overlapped the batch. The combined
calibrated-bearing model still regressed outage to 20.71% despite lower runtime.
Artifacts: `/tmp/opencode/height50-fagra-5a22e73/`.
Existing factor derivative checks
exercise the optional height row; dedicated arrival-order and fallback tests were
offered but not requested.

### Unified graph recovery experiment (reverted)

In this experiment, for an established graph, a global proposal initialized the same bounded,
temporary-fixed-to-revisable trial as local reacquisition. The global geometric
fit retains heading/half-turn checks but does not require consistency with the
incorrect current-mean pixel projection before trying the candidate. Both proposal
sources share the estimator's 500 ms trial cadence and unchanged-objective check.
The reference objective is measured before moving to the candidate geometry.
Final association, depth, heading, tilt, covariance and publication checks remain.

Global proposals no longer diverted established graph-mode frames into a separate
fixed-correspondence visual update. Rejected global trials can still invoke the
existing independently corroborated motion-conflict recovery. Initial alignment
and the legacy fixed-correspondence API retain their existing bootstrap path.
The upstream global proposal generator is not yet throttled; sparse covariance
search still requires complete assignments. Neither limitation is claimed fixed
by this consolidation.

The first four-way batch under `/tmp/opencode/unified-recovery-5a22e73/` stopped
while writing the robot-42 height report; temporary-storage quota exhaustion was
subsequently confirmed. Its robot-46 results exposed a startup routing regression:
unified outage was 36.39% (22.60% with height), primarily from lost confirmation
frames. An allocated alignment/generation does not imply completed initialization.
The corrected dispatcher keeps global confirmation while the state is `Startup`,
and a regression test covers the generation-1 Startup → Tracking handoff through
the raw graph-frame API.

The corrected comparison wrote compressed reports outside the exhausted temporary
filesystem at `loca-rerun/unified-recovery-5a22e73-v2/`: control, sampled height only,
unified recovery only, and both. Experimental bearings are disabled. Availability,
pipeline time, p99, discontinuities and objective acceptance are compared against
identical inputs; passing existing tests is not recording-level acceptance.

The five-recording comparison completed after resuming robot 45 following a server
restart. The restart removed the temporary inputs and helper scripts. Robot 45's
input was re-extracted from the untouched original; the frozen control reproduced
the saved control's complete estimates, queued visual frames and solve schedule
exactly. The analyzer is now self-contained in the persistent experiment directory.

| Model | Outage over 4399.02 eligible seconds | Pipeline time |
| --- | ---: | ---: |
| Control | 527.02 s (11.980%) | 1031.89 s |
| Unified recovery | 528.88 s (12.023%) | 1007.89 s |
| Sampled shared height | 675.86 s (15.364%) | 853.13 s |
| Both | 626.51 s (14.242%) | 853.93 s |

Per-recording outage percentages:

| Robot | Control | Unified | Shared height | Both |
| --- | ---: | ---: | ---: | ---: |
| 42 | 4.689 | 4.711 | 5.414 | 5.425 |
| 43 | 12.366 | 12.570 | 31.788 | 26.009 |
| 44 | 17.846 | 17.832 | 18.183 | 18.169 |
| 45 | 6.303 | 6.319 | 6.383 | 6.392 |
| 46 | 19.078 | 19.058 | 14.953 | 15.228 |

Neither candidate satisfies lower outage without higher runtime. Unified routing
was reverted, retaining the new graph-startup confirmation regression test. Height
and calibrated-bearing experiments remain disabled by default. Timing differences
are replay measurements across a restarted host, not statistically controlled
microbenchmarks. The current recovery baseline remains the 11.980% control.

The consolidation rarely reached candidate trials on proposal-bearing frames:
in the unified run, there were zero such reacquisition audits on robots 42 and 46,
two on 43 (accepted), two on 44 (validation rejected), and ten on 45 (validation
rejected). These counts do not prove which pre-trial gate prevented each attempt,
or whether the accepted trials actually used the global rather than local seed.
The remaining investigation is candidate admission/search and physically valid
contact support, not unconditional height equality or a routing-only refactor.

### Admission and inequality sampling experiment

`loca-rerun/recovery-admission-v3/` isolates two smaller changes with frozen
binaries and identical re-extracted inputs:

- `control`: the validated recovery baseline, original input-rate penetration guard.
- `local`: request local reacquisition while `LostTrack`, retaining the 500 ms
  attempt cadence but removing dependence on time since the last accepted visual.
  Recent single-feature updates can otherwise keep resetting recovery eligibility
  without supplying enough geometry to leave `LostTrack`.
- `rate`: sample the existing two-origin penetration-only guard at 50 ms before
  allocating vertices or storing motion history. The most recent retained foot
  record supplies the sampling cursor; there is no new state or configuration.
  This does not impose ground contact on airborne feet. Per-observation sigma is
  unchanged, so the cumulative strength of the penetration penalty is lower.
- `both`: combine the two changes.

The global proposal/initialization routing remains the baseline in all variants.
Both equality grounding and calibrated bearings are disabled. The existing
reacquisition test fixture exercises recent accepted visual support for the local
candidate. Carry/reconstruction tests, replay tests and Clippy are checked for
each frozen candidate. The analyzer checks re-extracted control estimates against
the previous batch, identical solve schedules, unchanged-objective acceptance,
and reduced graph measurements in sampled variants.

The complete five-recording result selects **`rate` only**:

| Variant | Outage | Pipeline time |
| --- | ---: | ---: |
| Control | 527.015 s (11.9803%) | 951.638 s |
| LostTrack-only recovery scheduling | 546.349 s (12.4198%) | 944.490 s |
| 50 ms penetration sampling | **519.495 s (11.8093%)** | **811.501 s** |
| Both | 538.828 s (12.2488%) | 809.408 s |

Selected-versus-control per recording:

| Robot | Outage before → after | Pipeline reduction | Solve p99 before → after |
| --- | ---: | ---: | ---: |
| 42 | 4.6888% → 4.6888% | 12.25% | 19.51 → 16.26 ms |
| 43 | 12.3658% → 11.8402% | 13.43% | 20.44 → 17.01 ms |
| 44 | 17.8458% → 17.5183% | 17.40% | 19.29 → 15.43 ms |
| 45 | 6.3035% → 6.3035% | 13.02% | 21.01 → 17.95 ms |
| 46 | 19.0776% → 19.0825% | 16.60% | 21.19 → 18.09 ms |

Net outage fell 7.521 s, with a 42 ms regression on robot 46. Pipeline time fell
14.73%, LM attempts 5.18%, rejected steps 27.85%, and summed live measurement
counts over solves 41.74%. The latter measures graph workload, not independent
sensor samples. Reducing repeated penetration observations also reduces their
cumulative weight; this experiment does not isolate weighting from computational
savings. Largest position steps are substantially lower on 43, approximately
unchanged elsewhere, and still reach 5.89 m on 44; discontinuities are not solved.

The working implementation matches the frozen `rate` candidate. No new sampling
parameter or mutable cursor is added. Existing recovery timing, initialization,
publication gates, and per-observation noise remain unchanged. The experimental
equality and bearing options remain disabled. All 75 localization tests, nine
replay tests, formatting and scoped Clippy passed before freezing this candidate.
The graph-startup regression test remains included. This establishes a modest
availability improvement and lower replay runtime, not absolute physical accuracy
without ground truth. Frozen inputs, binaries, source diffs, per-recording results
and aggregate `summary.json` are retained in the experiment directory.

### Locally pixel-calibrated bearing residual

Let `q = (u, v, 1)` be the normalized observed pinhole ray, `n = ||q||`,
`b = q/n`, `d` the predicted unit bearing, and `f = sqrt(fx*fy)`.
Apply the matrix

```text
W = [ fx*n     0      -fx*n*u ]
    [    0  fy*n      -fy*n*v ]
    [ f*u/n f*v/n       f/n  ]
```

to the existing oriented chord `sqrt(3/(2+b.dot(d))) * (d-b)`, then divide
by detector pixel standard deviation. The first two rows preserve local pixel
noise off-axis. The third is second-order near the match and distinguishes opposite
bearings. This is not an exact global pinhole pixel likelihood.

Association costs, factor optimization and final graph visual validation share
this residual when enabled. Positive-depth acceptance remains separate; the
initializer retains actual-pixel validation. Both actual pixel RMS and calibrated
RMS are recorded, alongside proposed/final identities, minimum depth and publication
gates. Enabling this model globally exposed three carried-motion velocity
regressions; passing default-mode tests does not resolve them.

### Remaining recovery-search issue (finding 4)

Proposed next change, not implemented:

1. Let sparse hypothesis search retain clutter subsets with the same one-to-one
   assignment and clutter costs as the final objective. A detection without a
   feasible landmark must not invalidate otherwise useful detections.
2. Keep current-pose visibility for ordinary tracking, but perform bounded recovery
   searches at changed candidate geometry. A wrong current pose must not eliminate
   a landmark solely because it currently lies behind the camera. Validate depth
   at the candidate/final pose.
3. Reuse the existing seed transaction, unchanged-objective comparison and rollback.
   Preserve single-feature information without treating it as proof of a fully
   observable pose or resetting established symmetry.
4. Throttle global recovery search separately: the current 500 ms limit applies
   only to local reacquisition. Repeated proposals from adjacent frames are not
   independent recovery evidence. Measure search cost and recovery delay before
   selecting the global cadence.

Diagnostics currently group ambiguity rejection and search-budget exhaustion as
`AmbiguityGateOrSearchLimit`; they cannot yet distinguish those causes. Without
physical ground truth, residuals and availability alone cannot certify landmark
identity or trajectory accuracy.
