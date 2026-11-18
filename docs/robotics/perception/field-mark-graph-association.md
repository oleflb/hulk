# Field-mark association inside the localization graph

Status: **historical experiment; graph association and local reacquisition removed
on 2026-11-08. Fixed-correspondence localization and intrinsic optimization remain.**

The results and design notes below describe archived code, not the current pipeline.
No replays were rerun for this removal. Source (including pre-existing dirty edits
and `test_support.rs`) is preserved at branch
`archive/localization-graph-association-20261108`, stash snapshot
`e3e9e8d1e3a322572889e99c3de4a7cdf5c22671`; active HEAD/index were unchanged.
The snapshot includes 79 small ignored replay evidence files from
`loca-rerun/graph-association-20261106/` and
`loca-rerun/graph-association-fagra-1ad5ce7/`. The inventory lists 107 artifacts:

```sh
git show archive/localization-graph-association-20261108:loca-rerun/graph-association-archive-inventory.json
```

All local `loca-rerun` artifacts remain untouched, including files over 1 MB
(MCAPs, replay JSON, doctor logs and binaries) retained locally rather than in Git.

Date: 2026-11-06. Historical scope: the then-current `field_mark_association` and
`localization-3d` pipeline, not the legacy localization implementation.

## Goal and decision

Let temporal trajectory optimization reconsider field-mark associations while
keeping total association + ingestion + estimation cost at or below the current
pipeline on the same recordings and hardware. Preserve false-positive rejection,
missed-detection tolerance, exposure-time geometry, and the established field
symmetry branch.

Recommended first implementation: **one atomic association-aware visual factor
per frame**, minimizing a one-to-one assignment against a fixed planar field map.
Use the existing continuous solver and the existing rectangular assignment crate.
Start with hard assignment, not Viterbi, soft mixtures, or a general hybrid solver.

This is a local, piecewise-smooth optimization method. Keeping alternative edges
in the objective does not maintain multiple independently optimized trajectories
or guarantee global convergence.

## Integrated candidate results

### Follow-up: fagra inspection API cleanup

Updated to fagra `1ad5ce752dc8c70d8250a14fcf3580e60d6d1831`, replacing
duplicate evaluators with stored-factor inspection. Relevant release suites,
nine replay tests and scoped Clippy pass. All five benchmarks and full MCAP
exports completed under `loca-rerun/graph-association-fagra-1ad5ce7/`; its README
contains Twix instructions and the per-recording comparison.

Timed preparation + ingestion + estimation totals **976.05 s**, **3.7% below
the original baseline** and **1.4% below the prior graph candidate**. Per-recording
changes against original baseline: 42 -14.1%, 43 -13.2%, 44 +10.9%, 45 -1.3%,
46 -1.9%. Solve p95 is 12.285, 13.712, 14.870, 13.885 and 12.283 ms respectively.
Coverage checks pass; availability is nearly unchanged from the previous graph
candidate, and 44's approximately 6.15 m discontinuity persists. This upstream
revision includes solver changes as well as the API; no isolated cleanup speedup
is claimed. Single-run desktop timing and no independent accuracy validation
remain limitations. The live default remains unchanged.

All five exports preserve the 39 non-comparison channels and original time range;
comparison counts agree with their full-recording replay audits. MCAP doctor
reports only log-time ordering warnings from appended comparison messages.
Twix GUI inspection has not been performed.

### Initial integrated candidate

The final integrated candidate completed all five release replays successfully,
with the same preserved inputs, 157485 solve events, 211555 timed preparation
calls and 39 recorded resets as baseline. No cropped recordings or substituted
detector/VO outputs were used.

**Decision: retain the existing live matcher by default.** Aggregate measured
association/preparation + ingestion + estimation time decreased **2.3%**, but
recording 44 increased **12.5%**, and solve p95 regressed on 44–46. Continuity and
accuracy acceptance are not established. An aggregate average does not satisfy
the per-recording runtime/quality target.

| Recording | Solve p95 baseline → candidate (ms) | Total timed pipeline baseline → candidate (s) | Change | Field-output availability baseline → candidate |
| --- | ---: | ---: | ---: | ---: |
| 42 | 15.509 → 12.124 | 190.15 → 159.35 | -16.2% | 80.44% → 81.84% |
| 43 | 15.000 → 14.202 | 195.60 → 174.79 | -10.6% | 88.53% → 89.68% |
| 44 | 11.717 → 15.065 | 204.53 → 230.20 | +12.5% | 85.00% → 84.92% |
| 45 | 12.459 → 14.660 | 218.43 → 222.96 | +2.1% | 80.45% → 80.57% |
| 46 | 11.285 → 12.620 | 204.38 → 202.55 | -0.9% | 86.46% → 86.88% |

Total timed pipeline: **1013.08 → 989.84 s**. Whole-process user + system CPU:
**1064.55 → 1040.55 s**. Event-loop wall: **1054.98 → 1027.86 s**.
The timed pipeline sums measured durations across the respective event streams,
not their independent percentiles. Solve timing includes graph matching, recovery
attempts and retirement; ingestion is accounted separately. Raw preparation now
takes approximately 0.009–0.010 ms p95, but matching moved into estimation.

Field estimates increased **132481 → 133316** (+835); total estimates increased
**155254 → 156089**. Availability is per solve event, **not accuracy**.

### Continuity and remaining risks

- Recording 44: the same epoch-8/generation-1 discontinuity persists at
  `1792937747329576004` → `1792937747341626741`, both endpoints `LostTrack`:
  **5.8851 → 6.1519 m over 12.05 ms**.
- Recording 45: largest adjacent same-generation step increases
  **0.4650 → 1.4175 m**; candidate transition is `LostTrack` → `Tracking` over
  42.20 ms, timestamps `1792937213182435490` → `1792937213224631418`.
- Recording 43: **1.1745 → 1.6589 m**, but candidate endpoints span a 1.25 s
  output gap. Recording 46 decreases **1.8516 → 0.8675 m**, with the candidate
  step spanning 26.08 ms. These diagnostics are not truth-labelled errors.
- No independent recording ground truth or robot timing. One final run per
  recording; no confidence intervals. Intermediate builds measured -1.4% and
  +0.5% aggregate timing changes, with the same reported availability/largest-step
  figures. Small aggregate differences are not a reliable speedup claim.
- An always-solve integrated replay, target-hardware comparison, and the
  reset-time versus delayed-exposure own-half audit remain outstanding.

### Artifacts and reproduction

Authoritative final artifacts:
`/tmp/opencode/localization-graph-validated-R1huZFZt/`.
Read `REPORT.md`, `summary.json`, `run.sh`, and `compare.py`. The directory also
contains the measured binary, input/binary hashes, source diff, new factor source,
configuration snapshots, and all raw JSON/log/time reports. It supersedes the
intermediate `localization-graph-fh1V09o9` and `localization-graph-final-a2VM2qWG`
directories. Preserve these temporary artifacts before cleanup.

Reproduction requires the archived source/binary, not the current executable.
The experimental replay flag, live switch, raw-observation wire topic and recorder
entry have been removed. The fixed-correspondence wire topic remains available.

### Integrated lifecycle policy and checks

- Startup/global proposals use existing independent validation and fixed factors;
  the source image is inserted once. Established raw frames use the atomic factor.
- Epoch/generation, exposure geometry, stale-job rejection and latest-frame
  scheduling remain enforced. Accepted association history is refreshed only
  after transactional acceptance, including conflict-recovery trials.
- Graph-selected associations cannot replace independent motion-conflict anchors
  or renew supported-leg evidence. This distinction was corrected before the
  authoritative final replay.
- Clutter cost uses existing pixel noise, `max_rms_px` and Huber parameters.
  Retirement requires a conditional assignment gap of one quarter of a clutter
  cost. An ambiguous retiring cohort is removed before constructing a prior;
  remaining constraints are reoptimized, with at most one additional cohort
  retry. Fixed anchors and prior information remain. On failed solves, the
  restored-state motion marginalization path excludes the removed visual factors
  and keeps the window bounded. This is a conservative initial policy, not a
  calibrated association-marginal posterior.
- The initial measured candidate kept a separate diagnostic evaluator because
  fagra lacked factor inspection. The subsequent update to `1ad5ce7` uses
  `with_factor` for validation, accepted-history refresh and retirement, removing
  that duplicate evaluator, its scratch, cloning and the `GraphStates` adapter.
  Matching is still recomputed at the requested state, never recovered from a
  trial cache. This dependency update also includes upstream solver changes;
  its results must be measured separately from the initial candidate above.
- Release checks passed: 60 association unit tests + two integration tests,
  70 localization tests, 157 factor-crate tests + two integration tests + one
  doctest; nine replay-example tests. Scoped Clippy with `-D warnings` passed.
- New integration checks cover midfield crossing, false positives/all clutter,
  duplicate-image rejection, propagation through dropout, mirrored recovery
  without a new generation, single-counted proposals, and independent conflict
  evidence. Full repository tests and robot execution were not run.

## Baseline

**Completed on 2026-11-06:** all five full-recording replays exited successfully.
One subagent ran the existing pipeline; no production code or repository tests
were changed during baseline collection. Candidate progress is recorded below.

### Provenance and artifacts

- Source revision: `2843620801adc2512c623e8dcdbe6c6f13226841`; initially clean.
  The only later repository change was this strategy document.
- Build: `cargo build --locked --release -p localization-3d --example relocalization_replay`.
  Native x86_64 release, clang/mold linker, no target-cpu override;
  rustc/cargo 1.99.1. Compile-time base localization, association, and odometry
  parameters were used and copied into the artifact directory.
- Host: AMD Ryzen AI 9 HX PRO 370, 12 cores/24 threads, 54 GiB reported RAM,
  Linux 7.2.14-200.fc44.x86_64. This is not robot hardware.
- Binary SHA256: `f30c97c2d3c5ae38438053c699ab22796e69eef7c80195f487122e16371ee7d1`.
- Artifacts: `/tmp/opencode/localization-baseline-2843620801/`. Includes
  `REPORT.md`, `commands.md`, `environment.txt`, `summary.json`, source/input
  hashes, channel/reset audits, copied binary/configuration, and per-recording
  replay JSON, logs, `/usr/bin/time` reports, and filtered MCAPs. These are local
  temporary artifacts; preserve them before cleanup if needed for comparison.

One pass per recording, sequential replay processes, no affinity/governor
isolation or repeated-run confidence intervals. Source integrity/reset scans
overlapped early replay work; desktop/background load was uncontrolled. WARN
logging remained enabled with output on tmpfs.

### Recording and reset coverage

Sources are `loca-rerun/10.1.24.{42,43,44,45,46}.mcap`, recorded 2026-10-25.
All five originals passed `mcap doctor`. Each has 41 channels, including historical
comparison outputs rejected by the replay tool. Filtered copies retained all
messages on the nine available replay-input channels, without time cropping;
retained counts were checked against source summaries. Original MCAPs are unchanged.

| Recording suffix | Duration (s) | Detection messages | Solve events | Observed epoch resets | Replayed epochs |
| --- | ---: | ---: | ---: | ---: | --- |
| 42 | 1009.235 | 42027 | 27483 | 3 | 0–3 |
| 43 | 1013.052 | 39985 | 26200 | 8 | 0–8 |
| 44 | 1006.182 | 42996 | 36270 | 8 | 0–8 |
| 45 | 1015.057 | 43898 | 33968 | 9 | 0–9 |
| 46 | 1013.942 | 42785 | 33564 | 11 | 0–11 |

Total: **157485 solve events, 211691 detection messages, 39 observed resets**.
Every recorded epoch appears in replay diagnostics. Initial epoch zero is not a
reset. Replay reapplies recorded epoch changes; live reset triggers/reasons were
not independently exercised.

All recordings contain field dimensions, IMU, kinematics, camera geometry,
detections, VO, localization status, solve diagnostics, and fall status. None
contains `odometry/kinematic_delta`; all runs reconstruct it using current base
parameters. All runs report incomplete reconstruction inputs: missing/stale
aligned updates interrupt delta continuity and those intervals are not fused.

### Host timing

All quantiles below are **milliseconds**, using sorted-sample index `floor(n*p)`
(median `floor(n/2)`). These are measured durations, not throughput-derived latency.

| Recording | Association calls | Association p50 / p95 | Replay localization cycle p50 / p95 | Solve-only p50 / p95 / p99 / max |
| --- | ---: | ---: | ---: | ---: |
| 42 | 42008 | 0.01062 / 0.21848 | 5.772 / 16.894 | 4.922 / 15.509 / 27.056 / 86.026 |
| 43 | 39955 | 0.01262 / 0.27558 | 6.514 / 16.979 | 5.622 / 15.000 / 27.155 / 83.047 |
| 44 | 42966 | 0.00926 / 0.12315 | 4.297 / 13.814 | 3.861 / 11.717 / 21.216 / 66.340 |
| 45 | 43872 | 0.01078 / 0.24147 | 5.839 / 14.167 | 5.064 / 12.459 / 24.998 / 98.222 |
| 46 | 42754 | 0.01008 / 0.27239 | 5.494 / 12.867 | 4.719 / 11.285 / 19.634 / 107.270 |

- **Association:** only `associate_visual_features`; includes no-result calls,
  excludes feature extraction, geometry/history lookup, decoding, and detector
  inference. Ineligible detection events are skipped; 211555 calls were timed.
- **Replay localization cycle:** pending sorting/ingestion, initialization, solve,
  diagnostic JSON construction, and output/history bookkeeping. Includes
  uninitialized attempts; it is not solver-only latency.
- **Solve-only:** `SolveDiagnostics.estimation_duration`, the complete
  `Localization::solve`, including discarded recovery candidates but excluding
  ingestion and outer replay bookkeeping. Six events were uninitialized, leaving
  157479 timed solves. Logging and scheduler delays can affect wall durations.
- `summary.json` additionally contains ingestion-only and ingestion-plus-estimation
  quantiles. Association and solve operate at different event rates: do not add
  their independently computed percentiles as a total-cycle percentile.

| Recording | Event-loop wall (s) | Whole-process elapsed (s) | User / system CPU (s) | Peak RSS (KiB) |
| --- | ---: | ---: | ---: | ---: |
| 42 | 198.223 | 201.25 | 198.36 / 0.58 | 1494688 |
| 43 | 204.871 | 208.00 | 205.92 / 0.57 | 1428596 |
| 44 | 212.879 | 216.17 | 214.53 / 0.67 | 1790460 |
| 45 | 226.450 | 229.64 | 228.29 / 0.63 | 1795472 |
| 46 | 212.555 | 215.73 | 214.38 / 0.62 | 1754916 |

Event-loop wall excludes initial loading/decoding/sorting and final JSON output.
Whole-process measurements include those operations. Neither measures sensor-to-pose
latency or isolated estimator memory; the replay retains recording/report data.

### Availability and continuity

| Recording | Estimates / solve events | Field estimates (% of solve events) | Missing estimates | Longest same-epoch field-output gap (s) | Largest same-generation adjacent XY step (m / elapsed s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 42 | 26840 / 27483 | 22107 (80.44%) | 643 | 2.101 | 0.3674 / 0.2741 |
| 43 | 25782 / 26200 | 23196 (88.53%) | 418 | 1.526 | 1.1745 / 1.4245 |
| 44 | 36075 / 36270 | 30830 (85.00%) | 195 | 0.530 | 5.8851 / 0.01205 |
| 45 | 33639 / 33968 | 27327 (80.45%) | 329 | 1.918 | 0.4650 / 0.1105 |
| 46 | 32918 / 33564 | 29021 (86.46%) | 646 | 2.461 | 1.8516 / 1.1903 |

Total: 155254 estimates, 132481 field estimates, 2231 missing estimates. These are
availability counts, **not accuracy or time-in-Tracking percentages**. Missing
outputs include non-convergence, singular/rank-deficient information, inability
to progress, and six uninitialized attempts. Visual-frame queue admissions total
118940 with 188 rejections; queue admission is not eventual factor acceptance.

Notable baseline anomaly: recording 44 jumps **5.8851 m in 12.05 ms** within epoch 8,
generation 1, both endpoints `LostTrack`, at payload timestamps
`1792937747329576004` to `1792937747341626741` ns. This is not a generation-change
artifact, but without independent truth it is not a measured position error or
proof of a symmetry flip. Inspect this interval during candidate evaluation.
The report also records field heights above 1 m in all five runs; these are sanity
diagnostics, not truth-labelled failures.

### Reproduction and existing checks

Choose a fresh output directory; do not overwrite the preserved run. Core commands:

```sh
OUT=/tmp/opencode/localization-baseline-repeat
mkdir -p "$OUT"
cargo build --locked --release -p localization-3d --example relocalization_replay
cargo test --locked --release -p localization-3d --example relocalization_replay
for ip in 42 43 44 45 46; do
  mcap filter "loca-rerun/10.1.24.$ip.mcap" \
    -y '^(field_dimensions|inputs/low_state|robot_kinematics|camera_geometry|visual_odometry/current_left_camera_to_visual_odometer|detected_objects|localization/status|debug/solve_diagnostics|odometry/kinematic_delta|fall_detection/status)$' \
    -o "$OUT/$ip.inputs.mcap"
  /usr/bin/time -v -o "$OUT/$ip.time.txt" \
    target/release/examples/relocalization_replay "$OUT/$ip.inputs.mcap" \
    > "$OUT/$ip.replay.json" 2> "$OUT/$ip.replay.log"
done
```

Use the recorded revision and configuration for an exact source baseline; the
command otherwise builds the current checkout. Original commands, integrity
checks, reset audit, and aggregation instructions are in artifact `commands.md`.

- Nine existing replay-example tests passed; no tests were added or modified.
- The former ignored `benchmark_tracking_estimation` test also passed. Its
  separate synthetic run reports 121 cycles, 119 field estimates, one failure,
  p50/p95 2.224/4.133 ms, and maximum position/rotation errors
  0.000966 m / 0.080415 degrees. It bypasses association/perception and does not
  establish recording accuracy. Its passing assertion does not mean zero failures.
- Analysis-only `reset_coverage.py` and `summarize.py` live in the artifact
  directory, outside the repository; no new replay or matching implementation.

### Interpretation and limits

**Association is already cheap relative to trajectory optimization.** The main
candidate benefit must be better temporal association/availability, while avoiding
additional graph iterations or costly recovery. An isolated faster assignment
solver provides little demonstrated budget headroom. These timings motivate
profiling, not a claim that every association call is cheap or that integration
will meet runtime parity.

No independent pose/association ground truth was identified. Historical comparison
outputs are estimates, not truth. Absolute accuracy, correspondence precision,
field-side correctness, and ground-truth recovery times remain unmeasured.

Recorded detections and VO are reused, not recomputed. Robot-specific parameter
override histories and calibration provenance are not reproduced. MCAP log time
determines delivery order; IMU uses publish time, other measurements payload time.
Solve events use recorded diagnostic delivery time rather than original solve
start time. Reconstructed odometry and subscriber scheduling are approximations.

Not checked: robot end-to-end latency, repeated-run variability, full repository
tests, or absolute recording accuracy. Incomplete reconstructed odometry and
approximate scheduling must be identical across variants and explicitly reported.

## Current implementation: verified integration points

Paths below are relative to the repository root.

| Component | Existing behavior | Consequence for the redesign |
| --- | --- | --- |
| `crates/linear_sum_assignment/src/lib.rs` | Reusable dense rectangular shortest-augmenting-path solver following Crouse/SciPy; generic float scalar, forbidden edges, deterministic ties | Reuse `AssignmentSolver<f64>`; benchmark before changing algorithms |
| `crates/nodes/field_mark_association/src/assignment.rs` | Solves assignment, then excludes each winning edge and resolves to certify uniqueness | Do not run this full certification wrapper on every factor evaluation |
| `crates/nodes/field_mark_association/src/tracking.rs` | Pose-covariance gating; joint likelihood for up to five detections; marginal assignment for larger frames | Replace tracking-time committed identities, not merely append another matching pass |
| `crates/nodes/field_mark_association/src/global_association/solver.rs` | Bounded hypotheses, two rounds of refinement/reassociation, rival checks, exact final assignment | Retain as initialization/recovery proposal generation initially |
| `crates/nodes/field_mark_association/src/map.rs` | Fixed class-labelled planar map and half-turn partners | Build once per immutable field configuration; preserve a single map definition |
| `crates/nodes/field_mark_association/src/frame_processing.rs` | Exposure-time preparation, latest-frame scheduling, epoch/generation checks, late-job rejection | Preserve these semantics when transporting observations |
| `crates/types/src/visual_localization.rs` | Wire message contains fixed correspondences | Add an explicit observation message; do not reinterpret old recorded messages |
| `crates/localization-fagra/src/factors/reprojection.rs` | Shared frame geometry; robust oriented-bearing residual and analytic Jacobians | Reuse this actual objective, not a newly invented pixel likelihood |
| `crates/nodes/localization-3d/src/estimator/vision.rs` | Innovation checks, fixed observation insertion, pending visual acceptance/rejection | Admit and validate observation frames without treating a minimum-cost assignment as uniqueness evidence |
| `crates/nodes/localization-3d/src/estimator/window.rs` | Marginalizes old controls and retires associated factors | Finalize or exclude unresolved visual factors before marginalization |
| `crates/nodes/localization-3d/src/heading.rs` | Independent IMU-to-field heading reference; no recovery-driven drift ratcheting | Keep this reference and its update restrictions |

Pinned `fagra` revision: `7b37df61ed615c85f5d59265f5df53ff5d5dc1be`.
Its `Factor` trait explicitly permits multiple residual blocks under one factor
identity. Separate `cost` and `linearize` methods permit unmatched costs to be
constant contributions with no Jacobian. A custom hybrid solver is unnecessary.

## Objective and numerical contract

For a frame with detections D and compatible map landmarks L:

```text
frame_cost(state) = min over valid one-to-one assignments A:
    sum(pair_cost(detection, landmark, state) for matched pairs)
  + sum(clutter_cost(detection) for unmatched detections)
```

- Each detection selects at most one landmark; each landmark is used at most once
  within this camera frame. Uniqueness is not imposed across different frames.
- Real field points are `Point2<Field, f64>`, lifted to z = 0 only at the camera
  geometry boundary. Full 3D body/camera motion is retained.
- Pair cost is the existing Huber cost of the whitened oriented-bearing residual.
  Matching and graph evaluation use the same units, normalization, and objective.
- Initially preserve explicit class compatibility, including intentional existing
  global-recovery substitution rules. Do not silently broaden tracking to all
  classes. Any later class penalty is fixed during a solve and must be calibrated.
- Each detection has a finite positive clutter cost. Start with a calibrated
  constant; detector confidence is not a calibrated probability by default.
- Unobserved map landmarks incur no cost. No visibility-based missed-detection
  penalty is introduced without a detection-probability model.
- Pose uncertainty may be used for conservative proposal/admission logic. Do not
  add the graph pose posterior to independent measurement noise inside each
  factor: pose uncertainty is already represented by graph variables and priors.
- This is not numerically identical to the current tracking matcher's joint
  marginalized pixel likelihood. Equivalence claims apply to always-solve versus
  certified-fast-path evaluation of the new objective, not to old versus new
  association decisions. Preserve old quality requirements through replay and
  acceptance checks; do not copy `score_ratio` into a new cost-gap test without
  deriving its meaning in the new units.
- All-clutter is a valid frame assignment, but is not evidence of visual tracking
  or recovery and must not refresh the last successful visual timestamp.
- A tiny residual or a minimum assignment cost is not proof of a unique global
  pose. Existing recovery corroboration and observability checks remain relevant.

### Evaluation and linearization

`cost(states)` computes the current minimum assignment and returns its complete
cost, including unmatched and any fixed class penalties.

`linearize(states)` selects the same deterministic optimum and emits residuals
and Jacobians only for selected real edges. Huber weights use the existing frozen
IRLS convention. Away from assignment ties, these rows give the active branch's
gradient. At ties, the objective can be nonsmooth; correctness checks must not
pretend a unique derivative exists there.

Every LM trial evaluates this same minimized objective at its trial state. Do not
freeze the previous matching for acceptance, cache by timestamp alone, or omit
the cost of unmatched detections. Rejected trials must not modify committed
diagnostics, accepted associations, heading references, or visual freshness.

Use one standalone `Factor` for the whole frame, rather than a batch of mutually
dependent detection factors. Partial `FactorSelection` evaluation would otherwise
change which observations compete for a landmark. Removal and marginalization
must act on the same atomic frame identity.

### Ground-plane computation

For a field landmark (x, y, 0) and current field-to-camera transform:

```text
camera_point = rotation.column(0) * x + rotation.column(1) * y + translation
```

Compute each predicted unit bearing once per landmark, each observed bearing once
per detection, and pair costs from their dot products. Compute analytic Jacobians
only for selected real matches. Do not introduce ray-to-ground intersection into
the tracking likelihood; it is ill-conditioned near the horizon and changes the
noise model. Ground-planarity alone cannot identify a false detection.

Validate finite input geometry at ingestion. Handle invalid range/projection
candidates explicitly as forbidden edges where justified; malformed graph states
must remain evaluation errors rather than being disguised as all-clutter frames.
Preserve the existing oriented-bearing model's smooth behavior around optical
depth zero; final physical visibility checks remain separate acceptance criteria.

## Matching and exact fast path

Construct a D x (L + D) matrix. Real columns contain pair costs; dummy columns
represent clutter. The private result is `Matched(LandmarkId)` or `Unmatched`,
not a sentinel column number exposed to callers.

Before invoking the rectangular solver:

1. Find each row's cheapest compatible real landmark or its own dummy option.
2. If the chosen real landmarks are all distinct, the sum of row minima attains
   the assignment lower bound: this is a globally optimal matching.
3. Otherwise invoke `AssignmentSolver<f64>` with `Objective::Minimize`.

For reuse of an old matching, the same sufficient certificate is valid if every
old selected edge remains a row minimum. Low reprojection error alone is not a
certificate. Equal-cost assignments may differ between exact implementations;
test equal costs and constraints, not identical IDs at genuine ties. Make tie
handling deterministic and prefer unmatched at a clutter tie where practical.

Pair-cost calculation is still needed for this certificate. Initial complexity
is O(DL) for pair costs and approximately O(D^2(L + D)) worst case for rectangular
assignment. Do not add a spatial index, assignment warm-start machinery, or sparse
solver before profiling this small fixed map.

An additional exact simplification is available: a real edge with cost greater
than its row's private clutter cost is dominated by leaving that detection
unmatched. It may be excluded without changing the optimum. This is different
from a stale pose-based radius gate that could remove a future correct match.
The initial reference implementation should use all structurally compatible
landmarks so candidate pruning cannot hide correctness differences.

### Allocation and ownership decision

Preallocate matrix and assignment scratch to validated detection/map bounds.
Reuse capacity, never numerical results from a different graph state.

`fagra::Factor` currently evaluates through `&self` and does not pass a mutable
workspace. Resolve this explicitly in milestone M1 rather than hiding shared
mutation. Preferred small option, if estimator ownership permits: private
per-factor `RefCell<MatchingWorkspace>` with `try_borrow_mut`, used only as scratch
by a single-owner evaluator; return a meaningful error on an invalid reentrant
borrow. Confirm actual Send/Sync requirements first. If Sync is required, evaluate
a narrow explicit workspace API against local scratch allocation; do not add a
global mutex or thread-local cache by default. Any upstream API change requires a
separate scoped decision. Parallel evaluation is not assumed or newly introduced.

## Strict Rust types and module boundaries

The implementation must follow the Writing Rust skill; this section is a design
contract, not production code or a request to add all types immediately.

- Keep `Point2<Pixel>`, `Point2<Field>`, typed transforms, `Time`, and typed
  `StateKey`/`FactorKey` through all public interfaces. Use f64 for the numerical
  assignment/factor; convert detector f32 values once at ingestion.
- Put the shared wire feature-class enum and observation record in `types`.
  Reexport where needed instead of duplicating class definitions.
- Keep map geometry, private `LandmarkId`, `DetectionIndex`, assignment choices,
  and validated candidate relationships in a small numerical domain module in
  `localization-fagra`. Keep raw solver row/column indices inside its adapter.
  Map IDs are scoped to the immutable map owned by the estimator; do not transmit
  unversioned numeric IDs as if they were globally stable identifiers.
- Keep field-dimension/class-aware map construction and global search in the
  existing association module initially. It can populate typed numerical map
  entries and candidate lists without making `localization-fagra` depend on a ROS
  node or on the broad `types` crate. One map construction path serves both.
- The observation message carries detections, exposure-time camera geometry, and
  epoch/generation identity, not optimizer-owned graph handles or pose guesses.
  Reuse `TimeWrapper` rather than introducing a second timestamp convention.
- Give initialization/recovery proposals a separate explicit type from raw
  observations. A proposal seeds optimization; it is not an extra independent
  measurement and must not double-count its source image.
- Use existing lifecycle types where sufficient. Introduce new epoch/generation
  wrappers only at touched boundaries if needed to prevent real mixing errors;
  do not perform an unrelated repository-wide type migration.
- Public constructors validate finite pixels/geometry, confidence bounds, candidate
  membership, size limits, and calibration. Keep invariants private afterward.
- Distinguish valid unmatched results from malformed input, arithmetic failure,
  and invalid state. Propagate contextual errors with `?`; no new `unwrap`,
  `expect`, `unreachable!`, unsafe code, or silent fallback to fake matches.
- Prefer local mutation, borrowed immutable maps, and focused functions. Share
  the existing bearing calculation/Jacobian helper instead of copying it.
- Reuse existing dependencies (`linear_sum_assignment`, `ndarray` as needed).
  No new third-party matching library, blanket generic framework, or new crate is
  justified by this design.

### Observation delivery

The current association node becomes a preparer/proposal producer: all validated
raw frames can reach localization without first passing tracking uniqueness.
Retain its exposure-time interpolation, backpressure, bounded latest-frame queue,
and stale epoch/generation rejection. Global proposals remain available for
startup and recovery. Specify deduplication by frame identity so proposal delivery
and observation delivery cannot insert the same image twice.

Do not rename the node or move all subscriptions merely for architectural purity.
Use an explicit new message/topic or other schema-safe migration; old MCAP payloads
must continue to decode through an explicit replay adapter. Acceptance comparisons
must run both approaches from the same upstream detections, not feed the new
approach only observations already accepted by the old matcher.

### Startup anchor is a deliberate exception

Do not immediately replace every startup correspondence with a symmetric
min-assignment factor. Before any field-conditioned marginal prior exists,
transforming `FieldAlignment` by a half-turn and permuting all landmark choices
can leave the visual objective unchanged while leaving local motion untouched.
Warm-starting one branch is not an explicit record of the own-half evidence.

Smallest initial policy: use the current validated own-half bootstrap to establish
the branch, and keep its accepted source image as fixed-correspondence anchor
evidence using the existing factor path. That image is inserted exactly once,
not also as an association-aware factor or as a separately derived pose prior.
Subsequent frames use revisable matching. Retain the fixed anchor until its
information passes into the ordinary field-conditioned marginal prior. Reusing
the existing fixed factor for this specific purpose is not a second tracking
association implementation.

The own-half assumption applies to the reset-time pose, not arbitrarily to a
much later camera pose after movement. Preserve reset-to-exposure motion when
selecting the startup branch; explicitly audit the existing exposure-time
canonicalization behavior. If available motion/geometry cannot establish that
branch, do not invent certainty. This boundary and the provisional-to-corroborated
heading transition are M4 acceptance cases.

The fixed bootstrap image is still subject to the existing initialization
validation and corroboration requirements; do not anchor on a forced match or an
all-clutter frame. Correcting an erroneous bootstrap must use explicit validated
recovery semantics, not silently delete the only field anchor.

## Symmetry and retained history

1. Only an explicit localization reset creates a new initialization epoch and
   permits the assumption that the robot starts in its own half. It supplies no
   additional heading information. Boundary/underconstrained cases may abstain.
2. Crossing midfield after initialization must never invoke own-half
   canonicalization. Matching remains free to select landmarks in either half.
3. Losing visual track preserves field alignment, propagated motion uncertainty,
   marginal priors, and the established IMU-to-field heading reference.
4. A non-reset recovery proposal is evaluated against that retained history.
   Identical visual costs under a half-turn do not justify a symmetry switch.
5. Do not freely transform both the retained anchor and the candidate trajectory
   into a mirrored explanation. A local/global seed is only an initial iterate;
   every retained prior remains part of acceptance.
6. Ordinary uncertainty growth does not erase history. Broader uncertainty can
   weaken discrimination, but must not silently trigger fresh own-half startup.

Verified current behavior: `Localization::try_bootstrap` routes established
generations through `Estimator::ingest_recovery`; `apply_recovery_seed` leaves
priors and motion factors in force. `HeadingReference` is updated only by fresh
accepted tracking evidence, not recovery or repeated solves.

Important distinction: `bootstrap_candidate` explicitly reconstructs motion
without field-conditioned marginal priors, but its caller currently uses it for
generation-zero bootstrap. Do not accidentally extend it to ordinary recovery.
Conflict recovery has its own transactional handling of contradicted live leg
factors. Preserve its independent evidence requirements; a newly selected graph
matching is not by itself independent corroboration against those motion factors.

## Acceptance, covariance, and marginalization

Cheapest assignment is a numerical step, not an assertion of visual certainty.
Track frame support, physical visibility, geometric conditioning, and ambiguity
separately from factor cost. Retain safe last-good state and transaction semantics
on rejected updates. A single ambiguous mark must not declare global recovery.

Covariance from the active assignment is conditional on that branch, not a full
association-marginal covariance. Do not advertise it as resolving a tied rival.
Initially retain conservative ambiguity checks at visual acceptance/recovery and
at marginalization boundaries, not every LM cost call. Conditional assignment
gaps do not prove absence of a distant rival pose.

Before marginalizing a frame that still has materially ambiguous alternatives:
either establish sufficient support for its branch or remove its visual factor
and re-optimize the remaining active graph before constructing the prior. Keep
motion factors and the previous anchor. Do not merely delete a factor after its
information has been absorbed, and do not solve ambiguity by unbounded window
growth. The exact support policy and its runtime are an explicit M4 gate.
