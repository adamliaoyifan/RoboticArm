# STEP-BPP + Isaac Lab + MuJoCo + Top-K trajectory learning

- date: 2026-10-14
- parent: STEP-TOPK-20261014
- status: approved_for_foundation_only
- owner: eng / cursor / grok-4.7
- upstream: https://github.com/nikitasarawgi/step-bpp
- upstream_revision: 6f362a4e11877555ec923151638aec60eb091ac1
- elfin_reference_revision: 56b2a41ffc00d4e2292fe235863716519ea71d80
- plan_revision: exact containing commit, pinned by dispatch

## 1. Unified Objective

Build an isolated research extension of STEP-BPP with two simulation backends and
a trainable Top-K arm-path proposal module. This joins platform migration and
learning into one data/model/validation architecture, not two disconnected demos.
The current task delivers the initial runnable framework. Detailed research
requirements, throughput targets, large datasets and later experiments are separate
decisions after this foundation report. It does not replace Gazebo or the production
planner, and does not imply novelty, deployment eligibility or improved capacity.

STEP-BPP's documented task selects items and orientations with a placement module;
it is not documented as an Elfin motion generator. Preserve that distinction:

```text
occupancy observation + current box + robot/grasp/scene state
  -> existing STEP/GOPT placement interface (or explicit fixture proposal)
  -> geometry/endpoint screening
  -> Top-K phase-labelled joint paths
  -> deterministic repair + retiming + validation
  -> legal candidate selection
  -> Isaac Lab OR MuJoCo research rollout
  -> release/observation event -> occupancy update -> next decision
```

The existing item-selection policy is optional for the single-next-box case, not
misrepresented as the new trajectory network. Candidate IDs, placements and box yaw
are not trajectories. The model cannot update confirmed occupancy or send ROS motion.

## 2. Dispatch and Phases

| Work package | State | Owner/base | Deliverable |
|---|---|---|---|
| FOUNDATION G1 | dispatch_ready=yes | cursor/grok-4.7; pinned upstream + Elfin reference above | cloned extension, both backends, data/teacher adapters, train/infer/eval smoke, tests and requirement backlog |
| DATASET | not dispatched | to decide after G1 | task distributions, certified diverse teachers, held-out splits |
| LEARNING-EVAL | not dispatched | depends on dataset definition | trained model and same-budget baselines, K ablations and uncertainty |
| SEQUENTIAL-CAPACITY | not dispatched | later | coupled grasp/placement/path and capacity experiments |
| PRODUCT-INTEGRATION | not dispatched | later approval | shadow/revalidation and independent Gazebo/hardware gates |

Only FOUNDATION is executable now. One owner implements, tests and integrates all
its modules; no separate test handoff. This is independent of MASTER-PACK G3 and
does not authorize seed-chain. Read only immutable Elfin assets; coordinate through
the primary workspace docs/agents tree. CPU coding may proceed alongside Claude;
simulation/GPU-intensive work must queue behind its G3 runs.

Claim with agent_start.sh, then clone into
`/home/adamliao/work/elfin_humble_ws/research/step-bpp-topk` as a separate Git repo,
branch `research/dual-sim-topk-g1` from the pinned upstream. Preserve upstream origin,
license and commit history; no force push, no push without permission. Do not stage
the nested repo into the Elfin repo. If the directory already exists, verify identity
and preserve it rather than overwriting. Record clone and dependency manifests.

## 3. Upstream Audit and Licensing Boundary

Before imports/install, inspect entry points, subprocess/install hooks, dependency
versions, tests, observation/actions and geometry assumptions. Record which existing
modules are reused versus adapted; do not construct an unrelated standalone scaffold.
Run a minimal upstream environment step as a baseline and retain its output. Do not
launch the upstream default long PPO training as a dependency check.

The upstream LICENSE file says MIT while README contains an academic-use restriction.
Record both at the pinned commit, including upstream GOPT and asset licenses. This
plan does not resolve that inconsistency or authorize commercial redistribution.
Research-only local adaptation can proceed; product transfer/public redistribution
requires clarification. Preserve attribution. No claim that the upstream README's
published speed/capacity metrics transfer to this system.

Use separate dependency environments for legacy STEP, MuJoCo, and Isaac Lab when
needed; bridge through a versioned serialization schema. Do not downgrade ROS/PyTorch
globally or require all backends to import in one Python interpreter. Pin actual
compatible stable versions after host audit, save locks and installation commands;
do not silently follow latest/main or introduce a preview physics backend.

## 4. Shared Contracts

Implement small modules with explicit interfaces, respecting upstream structure:

- SceneSpec: SI metres/radians/seconds, world/container/base/tool/contact transforms,
  ordered six joints and limits, tool and seven-face container collision geometry,
  pedestal/pickup table, box dimensions, placed objects, gravity/mass/attachment mode,
  source/version/hash and optional observation raster resolution.
- Observation: measured/fixture dimensions with provenance, occupied/free/unknown
  plus confidence, q/dq, fixed installation and geometry IDs, target pose or allowed
  set, T_tcp_box, load phase and map token. Simulator truth is a separate label channel.
- PlacementProposal: box pose/yaw/support/geometry status from STEP/GOPT or named
  fixture provider. Upstream cuboid assumptions cannot silently replace the seven-face
  hull. G1 filters adapted proposals through the declared hull/support contract;
  no claim that unmodified upstream capacity remains valid for that hull.
- RouteSample: request/scene/model/split IDs, placement/grasp, q waypoints, phase mask,
  release event and load transitions, time-parameterized result, validator parameters
  and outcome, teacher provenance, cost and failed boundary. Keep negative/timeouts
  distinct; search timeout is not proven physical infeasibility.
- Backend: load/reset/observe/set_joint_targets/step/attach/release/contacts/close,
  explicit capabilities and BackendUnavailable errors. Mock tests are not runtime
  backend evidence. Switching backend must not change task units or joint order.
- ProposalModel: propose(observation, target, K, seed) -> K paths, scores and provenance.
  Output geometric paths, not motor commands. Validate after interpolation/retiming,
  not only at network knots. Missing validator means not_evaluated, never certified.

G1 starts at an already attached box and declared start q; covers carrying transit,
approach/insert/descend, release, unloaded retreat and return. Pickup search and
grasp optimization are extension points, not faked as demonstrated. T_tcp_box remains
fixed while carrying. Prior boxes remain obstacles in both load states. A ideal-rigid
attachment is acceptable as a declared research abstraction, not validated suction
physics or real load capacity. Do not reproduce a static teleported box pushing the
arm. Allow only the explicitly declared mating contact, never blanket robot/payload
collision suppression. Release makes the box a world/dynamic object.

## 5. Dual Simulation Implementation

MuJoCo uses a versioned MJCF conversion; Isaac Lab uses versioned USD/articulation
assets and a headless task. Both consume the same SceneSpec and Elfin reference
geometry. Preserve collision geometry independently from visuals; a convex hull must
not fill the container's empty interior. Export URDF/xacro dependencies from the exact
Elfin reference, not current dirty master. Conversion manifests identify every mesh,
joint-axis/limit/inertia modification and fixed TF. No hardware TF optimization here.

Start with one environment per backend. Rendering is optional for CPU unit tests,
but each actual backend smoke must save a scene image and a short rollout/state log
showing the robot/container/payload, attach, release, retreat. More environments and
RGBD/occlusion realism are future work. Backend friction/controller parameters must
be declared; do not claim cross-engine dynamics equivalence.

Scene changes are event-driven: release/add/remove/disturb/TF/content change invalidates
the relevant snapshot/certificate. Unchanged time or occlusion alone does not erase
confirmed occupancy. Relevant path changes trigger revalidation; target/support
failure triggers re-selection. Continuous execution-state monitoring remains active.

## 6. Initial Top-K Training Framework

Implement a small conditional multimodal path model, default a CVAE with an occupancy
encoder and robot/box/target/grasp conditioning, plus phase-labelled waypoint decoder.
Keep architecture configurable and separate from STEP's placement network. Default
K=4, expose K=1/4/8. Avoid a deterministic single-path network copied K times.
Use normalized joint paths, explicit phase masks and release boundaries; preserve
real trajectory lengths for validation. Auxiliary smoothness/endpoint losses do not
replace hard gates. Model scores are not collision certificates.

Provide data import/export, immutable split manifests, normalization fitted on train
only, teacher adapter, train/checkpoint/resume/infer/validate/eval entry points.
TeacherAdapter can import saved G2 full certified routes and query a bounded classical
planner through a separate process; never fit on unvalidated paths labelled positive.
Provide a tiny synthetic analytic dataset for trainer smoke, clearly distinct from
robot-route evidence. Multiple seeds/parameter variants of one scene stay in one split.
Training labels may use simulator truth; online input schema must not expose hidden
objects, future occupancy, teacher success or evaluation-only IDs.

Implement two reference proposal providers: classical/imported teacher and nearest
training-route retrieval (with the same repair/validation). Initial smoke compares
schema and reports outcomes, not superiority. Missing compatible teacher data must
be explicit and cannot be replaced with a straight line labelled collision-free.

G1 metrics: raw and repaired valid@K, returned/unique candidate count, phase completeness,
generation/repair/retime/validation/total latency, rejection reasons, validator calls,
scene freshness and OOD/unavailable status. Report denominators including failure
and timeout. Model diversity and held-out success are reported without a G1 improvement
threshold; zero held-out success does not prove benefit and must be stated plainly.

## 7. G1 Acceptance and Exact Procedure

Owner provides scripts implementing these command interfaces (names may be thin wrappers
over native upstream code); config/asset locations are committed before final smokes:

```text
python scripts/topk/doctor.py --out <evidence>/doctor.json
python scripts/topk/check_upstream.py --seed 0 --steps 2 --out <evidence>/upstream
python -m pytest -q tests/topk
python scripts/topk/convert_assets.py --config cfg/topk/elfin.yaml --out <asset-cache>
python scripts/topk/backend_smoke.py --backend mujoco --config cfg/topk/smoke.yaml --seed 0
<Isaac Lab environment Python> scripts/topk/backend_smoke.py --backend isaaclab --config cfg/topk/smoke.yaml --seed 0
python scripts/topk/build_smoke_dataset.py --seed 0 --train 24 --val 4 --test 4 --out <data>
python scripts/topk/train.py --config cfg/topk/smoke_train.yaml --data <data> --steps 200 --seed 0 --out <run>
python scripts/topk/evaluate.py --checkpoint <run>/last.pt --split test --k 1 4 8 --seed 0 --out <evidence>/eval
```

| Gate | Samples | Pass observations |
|---|---|---|
| F0 upstream/provenance | one pinned clone and two upstream steps | exact origin/SHA, licenses/dependencies recorded; original env produces finite observation/reward and coherent done state; no long training |
| F1 shared contracts | each schema/event negative and positive once | round-trip equality; q order/units/xyzw convention explicit; NaN, wrong shape, missing scene, stale token fail closed; unknown stays unknown; no truth leakage; backend absence yields explicit unavailable |
| F2 geometry | both backends, same 10 fixed in-limit q states | contact-frame FK agrees with source FK within 1 mm and 0.1 degree; asset dimensions within 1 mm; five declared interior free points not filled by hull conversion; five explicit wall penetration probes (>1 cm) rejected |
| F3 backend lifecycle | each backend, empty and one-obstacle scenes, seed 0, one reset/attach/step/release/step/close each | actual engine steps >=120 before and >=120 after release, finite q/poses, attached relative pose <=2 mm/1 degree after settle, released box remains world object; occupied obstacle persists; save rendered frame and release trace; no process leak |
| F4 collision controls | each backend, same loaded q with inserted/removed test obstacle | inserted obstacle gives named non-mating collision; removed control valid; no broad ACM/mask bypass; missing collision support is not_evaluated |
| F5 trainer smoke | 32 analytic fixtures, split 24/4/4 by scene family; seed 0; 200 updates | finite loss/gradients, actual parameter update L2>1e-8; reconstruction loss on a fixed training batch decreases >=10%; checkpoint reload fixed-seed outputs within 1e-5; resumed step increments; optimizer state restored |
| F6 proposals/validator | four held-out fixture requests at K=1/4/8; one imported full robot teacher if available | exact candidate counts, finite six-joint paths and correct stages; all go through validation; report duplicate/valid@K metrics even zero; collision/NaN/limit negatives never executable; no teacher import means robot-teacher coverage not_evaluated, not invented |
| F7 integrated research demo | one canonical robot task per backend, seed 0 | request -> placement adapter -> proposal -> validator -> rollout-or-named-refusal trace is joined; additionally one validated teacher route plays through release/retreat/return on each backend; model rejection may use a separately labelled teacher replay, not claim learned success |
| F8 reproducibility | clean final commit per repo, all outcomes | commands/locks/hashes and evidence complete; tests zero new failures vs upstream baseline; owned backend processes closed; detailed next-requirement TODO exists |

F0-F8 are a scaffold functionality AND, not a success-rate campaign. Every mandated
unit must pass once; no population reliability threshold is claimed. Positive and
negative controls prevent both always-rejecting and always-accepting implementations
passing. Missing Isaac/GPU/assets, missing real teacher or too few samples are explicitly
not_evaluated and prevent a full FOUNDATION pass; deliver completed work plus a bounded
blocked report. Pure fake/stub tests never substitute for F2-F4/F7 engine execution.
If source/reference evidence cannot provide a replayable full route, preserve the
reason; do not broaden teacher search indefinitely just to close the framework.

## 8. Scope, Resources and Failure Evidence

Use isolated environments only; no sudo, driver changes, global pip installs, external
API keys or paid cloud. Never assume license acceptance. If Isaac is unavailable,
audit hardware/driver/disk first; record concrete compatible installation command and
required download size. Downloads over 10 GiB or interactive license acceptance need
explicit user approval. Do not claim success for a backend merely because its SDK is
unavailable. Complete independent CPU/MuJoCo work before reporting a platform blocker.

Limits for this generation: setup troubleshooting <=120 active minutes; engine/teacher
smokes <=120 active minutes combined; training <=30 GPU minutes (or <=60 CPU minutes),
one 200-step smoke plus one resumability check, no hyperparameter sweep. A setup retry
before any case executes consumes setup time, not a data sample. Per engine startup
deadline 180 s and per demo 10 minutes. Pause for Claude/Gazebo or other GPU jobs;
do not kill their processes or run competing Isaac/MuJoCo physics smokes concurrently.
Close owned processes before ending the session. No production ROS control endpoints.

Primary evidence root:
`docs/status/evidence/step_topk/2026-10-14_foundation_g1/`; reference large artifacts by
hash/path with recovery instructions. Evidence <=2 GiB/run and <=8 GiB total; datasets,
weights and converted asset cache separate, <=10 GiB before requesting expansion.
Do not delete first failures. T0 manifest always includes both repo revisions, dirty
state, dependency/hardware identity, parameters, seeds, stage timers and teardown.

| Boundary | Trigger | Required preserved data and window | Inspection |
|---|---|---|---|
| setup/conversion | import/asset/identity failure | full command/log, source and converted hashes, transforms and joint table for failed asset | doctor plus convert_assets check mode |
| observation/proposal | schema/OOD/nonfinite/phase failure | exact serialized request, checkpoint/config/seed and all K outputs for failed call | infer same request, compare IDs and shapes |
| validation | collision/timeout/disagreement | exact phase/path/interpolation, scene/load state, contacts, thresholds and model hashes | validator replay command in commands.sh |
| backend rollout | tracking/contact/release failure | failing phase state/action/contact arrays, 2 s before and 1 s after event; attach/release identity | backend_smoke replay mode and frame inspection |
| training | NaN/resume/loss failure | offending batch IDs, RNG, config, checkpoint/optimizer and last 20 updates | train resume/reproduce-batch command |

Capture-health tests must prove required files deserialize, join by case/phase and
survive one write failure with explicit incomplete status. Flush before teardown.
Record capture cost; no claim of model speed-up from unmeasured or incomparable runs.

## 9. Deliverable and Future Requirement TODO

Deliver source commits in the cloned repo, upstream audit/license report, environments,
asset conversions/manifests, two backends, data/teacher import, model/train/infer/eval,
tests and one reproducible demo command. Keep coordination notes in the primary repo.
Use agent_complete only when the full gate set passes; otherwise record blocked with
the exact external requirement, completed subgates and unexecuted gates. Do not close
as pass merely because the skeleton exists.

`NEXT_REQUIREMENTS.md` must separately list undecided product/research choices:

- [ ] Target box distribution, measured-size noise and allowed grasp offsets/yaws.
- [ ] Fixed installation and fixture set; geometry capacity versus executable capacity.
- [ ] Teacher diversity, search budget, minimum real positives and negative labels.
- [ ] Sensor/occlusion and confirmation model beyond the structured fixture observer.
- [ ] Suction/mass/contact fidelity and eventual sim-to-real calibration.
- [ ] K, path representation, conditioning, loss/reward and safe fallback policy.
- [ ] Held-out scenes/sequences, equal-budget baselines and quantitative promotion gates.
- [ ] Cross-engine discrepancy policy and independent final validation authority.
- [ ] Novelty hypothesis, related-work differentiation and publication experiments.
- [ ] License clarification and any redistribution/deployment decision.

Do not turn this backlog into implied authorization for long training or production
integration. Existing LRF promotion gates and MASTER-PACK acceptance are not superseded.

## Sources and Related Plans

- STEP-BPP README and code: https://github.com/nikitasarawgi/step-bpp/tree/6f362a4e11877555ec923151638aec60eb091ac1
- Upstream license: https://github.com/nikitasarawgi/step-bpp/blob/6f362a4e11877555ec923151638aec60eb091ac1/LICENSE
- Isaac Lab official installation: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
- MuJoCo official Python API: https://mujoco.readthedocs.io/en/stable/python.html
- `docs/plans/learned_placement_research/01_method.md`
- `docs/plans/learned_placement_research/02_simulation_and_events.md`
- `docs/plans/learned_placement_research/03_academic_positioning.md`
- `docs/plans/learned_placement_research/04_validation_and_decisions.md`
- `docs/plans/learning_research_feasibility.md` (promotion boundaries remain unchanged)
- `docs/architecture/privilege_boundary.md`
- `docs/agents/reviews/2026-10-14_1124_baseline-g2-review-closure.md`
