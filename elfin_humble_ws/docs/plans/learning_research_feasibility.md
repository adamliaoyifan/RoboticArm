# Learning-Based Research Feasibility Plan

Date: 2026-09-05

Status: approved by distinct Codex consensus. The Git commit containing this
approved text is the required plan revision for runnable work.

Parent task: `LRF-20260905`

Base source revision: `218f444406217bab779ce99333ec75fb8b24e1de`

## Objective

Measure whether learning-based methods can improve occlusion-aware 3D
perception, active perception, and placement planning over the maintained
model-based baseline. This plan produces reproducible feasibility evidence; it
does not replace, bypass, or change the default production pipeline.

The model-based pipeline remains the deterministic baseline and safety kernel.
Learning modules may predict uncertainty, propose views, or rank already-safe
placement candidates. Canonical container geometry, UNKNOWN-space semantics,
collision checking, insertion-corridor checks, IK continuity, and motion
execution remain authoritative deterministic gates.

## Non-Negotiable Boundaries

1. Online model inputs are limited to data available on hardware: synchronized
   RGB/depth or point clouds, camera calibration, stamped TF, robot state,
   calibrated container geometry, and prior online outputs. Gazebo state,
   spawned identity, complete mesh, hidden surface, future frames, and eval
   labels are forbidden model inputs.
2. Simulation/eval truth may generate training targets, occlusion buckets, and
   metrics only. The same inference and post-processing functions must run on
   simulated messages and hardware bag messages.
3. Learned completion must preserve an explicit distinction between observed,
   inferred, and unknown geometry. Inferred free space never becomes canonical
   FREE without measured ray evidence.
4. A learned module cannot execute motion, mutate the canonical cargo map, or
   bypass hull, corridor, collision, aperture, support, IK, freshness, or OOD
   gates.
5. Research code and optional dependencies stay isolated from production
   package imports and launch defaults. Downloaded datasets and weights are not
   committed; source, license, version, and content hash are recorded.
6. A failed or unavailable model must produce an explicit unavailable/OOD
   result. Silent fallback presented as a learned result is forbidden.
7. No task in this plan grants production decision authority. Promotion always
   requires a separate reviews decision and a newly dispatched integration
   task.

## Promotion Levels

### L0 - Research-only

The method has a reproducible offline harness, frozen data split, machine-
readable metrics, dependency/weight provenance, and direct comparison with the
exact model-based baseline revision. L0 permits continued research only.

### L1 - Shadow-eligible

In addition to L0, the module meets its module-specific improvement gate,
common safety/calibration gates, and runtime reporting below. It may then be
proposed for a separate shadow-mode adapter that receives the same observations
but cannot affect robot decisions.

Passing L1 does not authorize a production replacement.

### L2 - Replacement-candidate

L2 requires a later, separately approved task and all of:

- held-out real ROS 2 bag replay with no training/test leakage;
- hardware shadow operation with OOD and dropout injection;
- no regression of deterministic safety invariants;
- at least three consecutive boxes completed in the production state machine;
- measured latency/resource compliance on the deployment computer;
- explicit rollback and fail-closed behavior.

No simulation-only result can satisfy L2.

## Common Evaluation Protocol

- Compare the candidate and baseline on identical sensor observations.
- Freeze train/validation/test manifests before reporting final metrics.
- Split by suitcase mesh identity as well as pose; a test mesh must not appear
  in training under another pose.
- Report at least three deterministic seeds and aggregate median, p95, bootstrap
  95% confidence interval, valid-result rate, and failure-reason counts.
- Bucket test samples using eval-only visible-surface ratio:
  - light occlusion: greater than 70%;
  - medium occlusion: 40% through 70%;
  - heavy occlusion: 20% through 40%.
- Include an unoccluded control and sensor-noise/TF-perturbation/OOD fixtures.
- Record CPU/GPU model, peak memory, warmup policy, and p50/p95 inference time.
- Store raw machine-readable output under
  `docs/status/evidence/learning_research/<subtask>/<revision>/`; role notes
  summarize and point to evidence rather than copying logs.

## Common L1 Gates

Every candidate must satisfy all of these before reviews considers shadow mode:

1. No privileged input appears in the inference function, serialized sample,
   feature cache, or normalization statistics.
2. Test-set leakage checks pass, and all reported runs identify baseline code
   revision, model/config hash, data-manifest hash, and seed.
3. OOD, missing, stale, malformed, and non-finite inputs fail closed with stable
   reason codes; they do not emit decision-authoritative geometry or actions.
4. A nominal/unoccluded control metric does not regress by more than 10%
   relative to the baseline unless reviews accepts a documented tradeoff.
5. Uncertainty is empirically calibrated: the nominal 90% interval has 85% to
   95% coverage on held-out simulation. Otherwise the result remains L0.
6. Runtime is reported honestly. Per-frame modules need p95 at or below 200 ms
   on the stated target-class GPU for a 5 Hz path. Slower methods can only be
   considered for asynchronous stop-and-look use.
7. All deterministic hard-gate tests remain unchanged and passing. A learned
   proposal that a hard gate rejects is recorded, never coerced into acceptance.

Thresholds are eligibility gates, not claims of statistical superiority. The
report must also include confidence intervals and sample counts.

## Subtasks

| ID | Owner | Depends on | Scope | Outcome |
|---|---|---|---|---|
| LRF-P1 | `eng/cursor-grok-b/grok-4.6` | none | Occlusion-aware 3D perception feasibility spike | Reproducible baseline/candidate metrics and an L0/L1 recommendation; no production replacement |
| LRF-A1 | `eng/cursor-grok-b/grok-4.6` | LRF-P1 | Task-aware learned NBV feasibility through `ExplorationPolicy` | Same-budget comparison against heuristic NBV; no motion authority |
| LRF-PL1 | `eng/cursor-grok-b/grok-4.6` | LRF-P1, LRF-A1 | Learned ranking of deterministic-safe placement candidates | Three-box simulation ranking metrics; no hard-gate replacement |

Only one runnable row may be assigned to `cursor-grok-b` at a time. Dispatch
LRF-P1 first. LRF-A1 and LRF-PL1 remain planned, not runnable, until reviews
accepts the preceding evidence and confirms their external baseline dependencies
are stable.

## LRF-P1 - Occlusion-Aware 3D Perception Spike

### Research question

Can a learning-based method infer useful full suitcase geometry from partial
RGB-D/point-cloud observations while exposing enough calibrated uncertainty to
remain conservative under occlusion?

### Required work

- Audit existing simulation assets, PF evaluation fixtures, and available bag
  interfaces; define a frozen offline sample and split manifest.
- Select at least one technically plausible trainable or pretrained completion
  method and document why it fits suitcase geometry, available compute, input
  modality, license, and sim-to-real constraints.
- Build the smallest isolated offline adapter necessary to compare that method
  with the current model-based estimator on identical observations.
- Measure top/support Z, width/depth/height, center XY/Z, yaw, full-geometry
  valid rate, uncertainty calibration, unsafe under-bound rate, latency, peak
  memory, and failure reasons by occlusion bucket and mesh split.
- Preserve separate observed, inferred, and unknown outputs. If the selected
  method cannot express uncertainty, wrap it with a measurable ensemble,
  sampling, or residual-calibration method, or conclude that it is not L1
  eligible.
- Include ablations for single-view versus accumulated multi-view input and for
  learning completion versus measured multi-view fusion. This determines
  whether active sensing is more valuable than hallucinated completion.
- Return one of: `stop`, `continue-research`, or `shadow-candidate`. The return
  must be derived from the gates, not qualitative screenshots.

### LRF-P1 L1 improvement gate

On the frozen held-out test set, all common L1 gates must pass and:

- in heavy occlusion, full-geometry valid rate improves by at least 10
  percentage points over the exact baseline;
- the heavy-occlusion composite normalized p95 geometry error improves by at
  least 20%, with no individual safety-critical p95 error worsening by more
  than 10%;
- a conservative uncertainty-expanded box contains the eval GT observable box
  in at least 99% of held-out samples;
- there are zero catastrophic under-bounds greater than 30 mm on any box face;
- nominal/unoccluded top Z, dimensions, center, and yaw each satisfy the common
  no-more-than-10% regression rule.

The observable-box GT used by the PF evaluation remains the primary target.
Complete hidden mesh shape is an auxiliary metric and must not redefine the
production box contract.

### LRF-P1 required evidence

- Frozen data manifest and leakage audit.
- Exact reproduction command and environment/dependency manifest.
- `metrics.json` containing per-sample results and aggregate confidence
  intervals for candidate and baseline.
- Failure gallery indexed to sample IDs, including false-confidence cases.
- Model/weight/data provenance with licenses and hashes.
- A short engineering report mapping every L1 gate to pass/fail/not-measurable.
- Focused tests proving no privileged fields reach inference, malformed/OOD
  inputs fail closed, and research imports do not enter production packages.

### LRF-P1 exit

The owner commits only the isolated harness, tests, manifests, report, and
small evidence summaries. Large models, datasets, generated clouds, and raw
images remain outside Git. The subtask passes when its experiment is
reproducible and reports honest gate outcomes; the learning candidate itself
does not need to pass L1 for the research subtask to complete.

## LRF-A1 - Future Learned Active Perception Spike

LRF-A1 will use the existing ROS-free `ExplorationPolicy` contract. It will
compare against the prior-guided stop-and-look baseline under identical view,
motion, and time budgets. Training may use eval-only information-gain labels;
inference may consume only immutable online snapshots.

L1 requires either at least a 10 percentage-point increase in downstream
measured full-geometry success under heavy occlusion, or at least 20% fewer
views at non-inferior geometry accuracy. Coordinator rejection of unsafe,
unreachable, stale, or mismatched proposals must remain 100% effective.

## LRF-PL1 - Future Learned Placement Ranking Spike

LRF-PL1 may rank only candidates already accepted by canonical hull, support,
aperture, corridor, collision, and IK gates. It may not emit joint trajectories
or convert an infeasible candidate into a feasible one.

L1 requires at least a 10 percentage-point increase in held-out three-box
completion rate, or at least a 5 percentage-point utilization improvement with
non-inferior completion rate. Accepted hard-gate violations and simulated
collisions must both remain zero. TCIG online/offline geometry semantics must be
stable before this task is dispatched.

## Decision Rule

Reviews evaluates each completed report as follows:

- `stop`: leakage, unsafe semantics, unavailable licensing/compute, or no
  credible improvement after controlled experiments;
- `continue-research`: reproducible L0 evidence shows promise but one or more L1
  gates fail or cannot yet be measured;
- `shadow-candidate`: every applicable L1 gate passes and a separate shadow
  integration plan is justified.

No result from this plan directly produces `replacement-candidate` status.

## Consensus

Distinct Codex requirements consensus was reached by
`reviews/codex-lrf-consensus/gpt-5.5/codex`. It confirmed that LRF-P1 is
bounded and measurable, preserves the deterministic safety authority, keeps
observable-box GT primary, and cannot authorize either shadow integration or
production replacement by itself.

The consensus record is
`docs/agents/discuss/2026-09-05_1827_learning-research-feasibility-consensus.md`.
