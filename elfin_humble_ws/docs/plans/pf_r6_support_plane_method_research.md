# PF-R6 Support-Plane Method Research Plan

Date: 2026-09-05

Status: approved by distinct Codex consensus. The Git commit containing this
approved text is the required plan revision for runnable work.

Parent task: `PFH-REMEDIATION-20260904`

Proposed subtask: `PF-R6-RANSAC-RESEARCH`

Base source revision: `18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3`

## Decision

Do not immediately replace support-plane RANSAC with either a z-mode/median
estimator or an assumed automotive algorithm. Current profiling proves that
support-plane RANSAC is the dominant measured geometry cost, but it does not
yet prove which replacement preserves fail-closed behavior.

Run an isolated research comparison first. The current support RANSAC remains
the production baseline, z-mode/median becomes one comparator, and the owner
must identify and evaluate the exact triangle-based or triangle-constrained
RANSAC family recalled from road/ground-plane fitting. Top-plane RANSAC is out
of scope and remains unchanged.

## Research Question

Can a road-ground-plane RANSAC acceleration or another constrained sampling
method reduce support-plane fitting latency enough to unblock PF-R6 while
preserving the current support-Z accuracy and fail-closed safety behavior on a
dense, bounded, partially occluded pickup-platform annulus?

## Scope Boundary

- Research and benchmark only. Do not modify
  `top_support_estimator.py`, `platform_free_pipeline.py`, detector nodes,
  launch files, runtime parameters, topics, or production defaults.
- Put prototypes and benchmark helpers in an isolated research directory.
- Read the exact committed baseline implementation; do not copy uncommitted
  production changes into the research branch.
- Use identical candidate points and preprocessing for every method. A method
  cannot gain speed by receiving a smaller or privileged input unless that
  preprocessing is separately timed and applied to every comparator.
- Simulation/eval truth may score results only. No GT support height, platform
  identity, spawned box identity, or hidden geometry may enter a candidate
  estimator.
- Preserve horizontal-support assumptions. Platform tilt remains out of scope.
- Do not change top-plane fitting. The benchmark measures support estimation
  and may report projected pipeline effect, but it does not claim PF-R6 closure.
- No result from this task authorizes a production replacement. A passing
  result is only an implementation recommendation to the existing PF-R6 owner.

## Required Method Study

The owner must use primary papers and official implementations where
available, and report:

1. The exact name, citation, and algorithmic steps of the automotive method;
   do not report a guessed label as a finding.
2. Whether "triangle" refers to minimal three-point sampling, spatial triangle
   constraints, seed triangles, triangulation, region-growing initialization,
   or another mechanism.
3. Its original sensor type, point count, scene scale, ground continuity,
   slope assumptions, outlier model, complexity, and stopping rule.
4. Which assumptions transfer to this pickup problem and which do not. The
   pickup support is a bounded annulus with central box occlusion, possible
   robot/background clutter, dense RGB-D noise, and one expected horizontal
   support plane; it is not an unbounded road surface.
5. License, maintained implementation availability, dependency cost, and
   whether a small ROS-free prototype can reproduce the core method.

If the remembered method cannot be identified confidently, the report must say
so and benchmark the closest well-supported constrained-RANSAC family without
claiming they are the same algorithm.

## Comparator Set

At minimum compare:

- current production support-plane RANSAC at the exact base revision;
- fail-closed robust z-mode/median over the same candidates;
- the identified automotive triangle/constrained-RANSAC method, or the closest
  explicitly labelled alternative if exact identification fails;
- one additional low-complexity robust plane estimator only when the literature
  review shows a credible fit to this data shape.

The task must attempt a runnable prototype of at least the baseline,
z-mode/median, and one constrained-RANSAC candidate. A literature-only report
does not complete the subtask.

## Dataset And Fixtures

Freeze a machine-readable manifest before final measurement. It must include:

- deterministic synthetic flat support with Gaussian/depth-quantization noise;
- 20k through 35k candidate points to cover the measured PF-R6 range;
- central suitcase occlusion and asymmetric visible annulus sectors;
- sparse or missing side coverage;
- isolated outliers and coherent wrong-height planar clutter;
- multiple competing horizontal planes;
- insufficient-point and non-finite negative controls;
- simulation-derived observations spanning carryon, standard, and large boxes,
  at least 10 trials per size with XY/yaw variation when available.

Synthetic GT is eval-only. Candidate estimators receive only the point set and
the same non-privileged workspace/height bounds available to hardware.

## Metrics

Report per fixture family and aggregate:

- support-Z absolute error median, p95, and maximum;
- valid estimate rate and fail-closed rejection rate;
- false measured-support count on every negative control;
- inlier count/ratio, residual, side-coverage result, and stable reason code;
- p50, p95, and maximum wall-clock latency after warmup;
- candidate-point count, sample/iteration count, early-stop behavior, and seed;
- determinism across three seeds;
- estimated detector critical-path saving relative to the measured PF-R6
  support RANSAC p50 `126.008 ms` and p95 `183.909 ms`.

Do not average away negative-control failures or compare methods on different
accepted subsets.

## Implementation-Candidate Gate

A method may be recommended to the existing PF-R6 owner only when all hold:

1. Support-Z p95 is at most 15 mm and maximum is at most 25 mm on the frozen
   positive set.
2. False measured-support count is zero across wrong-height clutter,
   competing-plane, missing-side, insufficient-point, and non-finite controls.
3. The method preserves explicit side coverage, minimum support points,
   residual, plausible-height, and same-stamp/status validation semantics, or
   demonstrates an equivalent fail-closed check.
4. Support-estimation p95 is at most 75 ms and is at least 2x faster than the
   baseline measured on the same machine and identical point sets.
5. Support-estimation p50 is at most 50 ms.
6. All outputs and reason codes are deterministic for a fixed input and seed.
7. No online privileged input, silent fallback, or production import is used.
8. The simulation-derived three-size matrix does not reduce support-valid or
   full-geometry-valid rate by more than one percentage point relative to the
   exact baseline.

These are research promotion gates, not PF-R6 acceptance. After a positive
recommendation, the current PF-R6 owner must independently implement the
selected method in production scope and rerun focused tests, PF-G6S, and the
original 30-trial PF-R5 Gate 4 regression.

## Required Deliverables

- Method identification and applicability report with primary references.
- Frozen dataset/fixture manifest and exact reproduction commands.
- Isolated ROS-free prototypes and focused tests.
- Machine-readable per-sample metrics and aggregate comparison table.
- Failure examples indexed to fixture IDs.
- CPU/GPU, dependency, version, seed, and revision provenance.
- One final recommendation: `reject`, `continue-research`, or
  `implementation-candidate`, with every gate marked pass/fail/not-measurable.
- Evidence under
  `docs/status/evidence/platform_free_height/pf-r6-ransac-research/<revision>/`.
- A result notification to the existing `eng/codex/gpt-5` PF-R6 owner. The
  research owner must not edit or close the PF-R6 generation-2 thread.

## Ownership

The proposed owner is `eng/cursor-grok-b/grok-4.6/cursor`. Work is isolated
from the active PF-R6 implementation owner. The research owner owns its
prototype, tests, evidence, commit, and subtask closeout end to end.

## Consensus

Distinct Codex requirements consensus was reached by
`reviews/codex-ransac-consensus/gpt-5.5/codex`. It confirmed that the task is
bounded, all comparators receive identical non-privileged inputs, the gates are
measurable, the active PF-R6 dirty checkpoint is excluded, and a positive
result cannot authorize production replacement.

The consensus record is
`docs/agents/discuss/2026-09-05_1950_pf-r6-ransac-research-consensus.md`.
