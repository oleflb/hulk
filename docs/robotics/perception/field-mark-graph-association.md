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
