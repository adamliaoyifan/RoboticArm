# ULD geometric placement RL initial implementation

- owner: codex/gpt-6
- parent: ULD-RL-20261019
- subtask: CORE
- generation: 1
- workspace_base: a920f4bd0a1549447da059c5a62461d05095b35c
- research_base: a5b4daf3985f40aa983cb2e6790e0bc4461267dc

## Scope

Implement the approved six-down-face/two-yaw geometric-GT-to-occ curriculum
under research/step-bpp-topk/topk/packing_rl, with typed configuration,
separate geometry/world/occupancy/support/candidates/environment/policy/trainer
ownership, real spawn-process parallel sampling, PPO, checkpoint/resume,
independent evaluation, CLI and usage documentation. Preserve unrelated dirty
files. No Gazebo or robot motion; no performance superiority claim.

Reuse the authoritative ULD geometry kernel. All external box poses are
container_link box centres in metres with xyzw quaternions. Policy inputs
exclude truth object lists and unknown future arrivals. Support may consume a
same-source surface companion to occ to avoid inventing exact contact heights
from quantized voxel centres; provenance must remain explicit.

## Acceptance

All gates are engineering correctness gates, not reliability streaks or
learning-quality claims. Every deterministic fixture must pass (per-fixture
expected pass rate 100%); deliberately invalid inputs must be rejected.
No assumption of independent repeated trials is made. No successful-packing
rate threshold is asserted from short smoke training.

1. Geometry: all 12 labelled orientations map the declared face downward;
   rotated extents, centre heights and WDH round trips agree within 1e-9.
   ULD chamfer intrusion, overlap, unsupported stacking, unknown support,
   stale map/action and invalid config are rejected; rejection never commits.
   Actual kernel usable volume, not enclosing cuboid volume, is used.
2. Environment: fixed fixture episodes with at least two legal placements,
   candidate exhaustion and truncation produce finite rewards, consistent
   immutable observations and nonzero successful-volume rewards. STOP cannot
   replace a legal placement. Only successful commits advance boxes/maps.
3. Parallelism: two spawn workers have distinct PIDs/worlds/RNG streams;
   repeated fixed-seed rollouts match; injected worker failure/timeout has a
   bounded response, durable boundary dump and no surviving owned worker.
4. PPO: handcrafted multi-environment GAE checks show no cross-environment or
   reset bootstrap; masked actions have correct probabilities. Two CPU runs,
   seeds 0 and 1, each two workers, at least two optimizer updates and 32
   collected transitions, have finite losses and changed trainable weights.
5. Resume: an update-boundary checkpoint resumes the next update with identical
   CPU tensors/metrics to uninterrupted training under the same configuration;
   incompatible config is refused. Evaluation changes neither training weights
   nor RNG/world state. A fresh-process CLI resume and eval both succeed.
6. Integration: focused packing_rl tests and existing tests/topk pass (or
   existing unrelated failures explicitly attributed). CLI help, preflight,
   train/resume/evaluate documented with exact runnable commands. Training
   output contains resolved config/hash, revisions, metrics and checkpoint.

Commands use research/_artifacts/venvs/topk-core/bin/python from the research
repo: `-m pytest tests/packing_rl -q`, `-m pytest tests/topk -q`, and
`scripts/topk/train_uld_rl.py` subcommands. The final role note records exact
commands, resolved smoke configuration and artifact paths. Rehearsals may
repeat; no scarce scored-run budget is consumed.

## Evidence

Evidence root: primary docs/status/evidence/uld_rl_initial/<run>/; refuse
nonempty train run directories. Max 2 GiB per run, 8 GiB total, CPU only,
two torch/BLAS threads or fewer per process. Test/train failure artifacts are
flushed before cleanup/retry; no full history/tree dumps.

| Boundary | Every-run trace | Failure payload | Window/budget | Inspection |
|---|---|---|---|---|
| configuration/geometry | config/hash/revision, kernel identity | exact config and exception | one case, 2 MiB | JSON/text |
| observation/candidates/commit | episode/map/action ids, counts, reasons | world state, observation, action, RNG, traceback | current transition, 16 MiB/worker | saved NPZ/pickle + replay API |
| parallel IPC | request id, worker pid, operation | pending command, last state/checkpoint, traceback or timeout | current RPC, 32 MiB | worker-failure JSON/pickle |
| PPO update | step, loss, grad norm, reward counts | last checkpoint, rollout tensors, RNG | one update, 256 MiB | torch.load and update replay |

Record missing capture explicitly; worker startup/death before first snapshot
uses its initial seed/config plus pending command. Capture bookkeeping should
be lightweight; no timing superiority comparisons in this engineering gate.
Clean research implementation commit for final smoke; if tracked unrelated
docs remain dirty, record their exact paths and diff hash in provenance.
