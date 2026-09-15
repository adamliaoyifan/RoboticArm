# PF-R7 generation 5 standard-seed scan and bounded live

Date: 2026-09-15

Parent: `PFH-REMEDIATION-20260904`

Subtask: `PF-R7`

Supersedes: PF-R7 generation 4 acceptance only. Generation 4 remains
`inconclusive/perception_availability_blocked` and is not promoted. Generation 3
remains a historical scored failure. PF-R1 through PF-R10 results remain
unchanged.

## Decision and objective

Generation 5 does not retry generation 4 and does not close on a detector-only
sweep. It first records detector availability for standard seeds, then runs one
bounded live campaign whose standard workload is the first six
detector-available standard seeds. Production `0.20`, `0.95`, support-ready
`t_steady`, the half-open `[t_steady, t_steady + 8.000s)` window, and
known-detector-miss conjuncts stay unchanged.

Generation 4 proved `standard_00/01/02` `proposal_available` and
`standard_03/04/05` `known_detector_miss_valid` on clean
`c086a396b974311ce40a6f8a3ab26dc14e9341f2`. Those six classifications are
imported; generation 5 does not rescan them. The live scan starts at
`standard_06` and continues in matrix order until six detector-available
standard seeds exist or the inventory is exhausted.

Generation 5 answers:

> Under the unchanged production detector floor, which predeclared standard
> seeds are detector-available, and does the generation-4 geometry contract
> then pass on a bounded live campaign whose standard cases use only those
> first six available seeds?

## Immutable anchors

- Production acceptance anchor:
  `60dafb7deee50a6f3a76d48076b743bf3e3e1bc8`.
- Generation-4 evaluator/window revision:
  `c086a396b974311ce40a6f8a3ab26dc14e9341f2`.
- Generation-4 live evidence:
  `docs/status/evidence/platform_free_height/2026-09-15_pfr7_g4/rev_c086a396b974311ce40a6f8a3ab26dc14e9341f2/`.
- Accepted proposal confidence floor: `0.20`.
- Conditional valid-top and `FULL_3D` rate floors: `0.95` each.
- Support-ready window: first admitted `support_window_count == 5`, then
  half-open `[t_steady, t_steady + 8.000s)`.
- Recovery bars: `t_first_valid_top`, `t_first_FULL_3D`, and `t_steady` each
  `<= t_proposal + 1.4 s`.

The tested revision must be one clean commit descendant of `c086a39` that
preserves generation-4 evaluator/window behavior and adds only eval-only scan
and seed-matrix controls. The tested worktree and the overlay source must both
report `git status --porcelain --untracked-files=all` empty, and the overlay
must be built from the same exact commit. D555, Livox, scheduler, site runtime
and unrelated planning changes are outside this task.

## Scan classification

Each scanned standard seed is recorded as exactly one of:

- `proposal_available`: production accepts a proposal at the unchanged `0.20`
  floor and the required T1/T2 dump is replayable.
- `known_detector_miss_valid`: the full known-detector-miss proof holds
  (upright GT in frame and workspace, eval IoU at least 0.50, best confidence
  below 0.20, production `detections=[]`, no cargo mask/cloud, healthy
  camera/TF/clock, replayable dump).
- `fixture_invalid`: bounded exclusion; preserve evidence; not available.
- `infrastructure_invalid`: bounded exclusion; preserve evidence; not available.
- `evidence_invalid`: task defect. Repair capture and rescan that seed before
  continuing. Do not close PF-R7.

Do not invent a sixth class. Do not retune the classifier to convert
`standard_03/04/05` into `eligible_fail`.

Imported generation-4 rows (do not rescan):

- `standard_00`, `standard_01`, `standard_02`: `proposal_available`.
- `standard_03`, `standard_04`, `standard_05`: `known_detector_miss_valid`.

## Scan budget and procedure

Environment: exclusive Gazebo slot, `gui:=false`, `use_rviz:=false`,
`ROS_DOMAIN_ID=7`, pidfile `/tmp/elfin_humble_sim.pid`. Clean-room residual
check before launch. Exactly one `/clock` publisher after launch.

Workload: the existing 16-seed G4 standard matrix (`standard_00`–`standard_15`).
Do not invent `standard_16`–`standard_23` on this attempt. The reviews cap of 24
is an upper bound, not a reason to extend `default_seed_matrix()`.

Order: import `00-05`, then live-scan `06` through `15` in deterministic seed
order. Stop as soon as six `proposal_available` seeds exist. Stop as
`perception_availability_blocked` if fewer than six are available after
`standard_15`. That blocked scan is not a PF-R7 fail and must not call
`agent_complete`.

Scan observe: stop when production accepts a proposal and the dump is
replayable, or when the known-detector-miss proof is complete, or at the 1.4 s
proposal deadline plus dump flush. Do not wait for the 8.000 s geometry window
during scan.

Command:

```bash
python3 scripts/pf_r7_bounded_acceptance.py \
  --live --scan-standard \
  --scan-start standard_06 \
  --stop-available 6 \
  --max-scan 24 \
  --import-scan docs/status/evidence/platform_free_height/2026-09-15_pfr7_g5/rev_<commit>/scan/imported_g4.json \
  --out docs/status/evidence/platform_free_height/2026-09-15_pfr7_g5/rev_<commit>/scan \
  --steady-start support-window-ready --steady-window-sec 8.0 \
  --recovery-limit-sec 1.4 --ros-domain-id 7 --overlay /tmp/pfr10_g6
```

Expected observations: `scan_verdict.json` lists every imported and scanned
seed with exactly one class; `available_standard_seeds.json` contains the first
six `proposal_available` seeds in scan order, or fewer with
`reason=perception_availability_blocked`; every scanned seed has
`capture_complete` and `replay_possible` except an `evidence_invalid` row that
stops the scan for repair.

Repetition: one scan campaign. No online best-seed search.

## Live workload and procedure

If and only if six detector-available standard seeds were recorded, run one
bounded live campaign. Carryon and large keep the generation-4
`default_seed_matrix()`. Standard uses only the predeclared six available
seeds, in scan order, via `--seed-matrix-json`. Scoring must not pick a later
unscanned or previously missed standard seed.

Live metrics, budgets, C2 bars, and window semantics are generation 4:

- slots 3, eligible per size 2, max 12 attempts per slot, 6 exclusions per
  slot, 3 consecutive same-size exclusions, 2 stack resets, 36 campaign
  attempts, 2700 s wall timeout;
- `0.95` valid-top and `FULL_3D` rates; 1.4 s recovery; 8.000 s half-open
  window; output `>=4 Hz`; geometry P95/max bars unchanged;
- executor-lag Q4 mean `<=0.20 s` and `<=1.25 * Q1`; workload-adjusted RSS
  `<=2 MiB/min`; teardown residuals 0.

Command:

```bash
python3 scripts/pf_r7_bounded_acceptance.py \
  --live \
  --out docs/status/evidence/platform_free_height/2026-09-15_pfr7_g5/rev_<commit>/live \
  --seed-matrix-json docs/status/evidence/platform_free_height/2026-09-15_pfr7_g5/rev_<commit>/scan/live_seed_matrix.json \
  --slots 3 --eligible-per-size 2 --max-attempts-per-slot 12 \
  --max-exclusions-per-slot 6 --max-consecutive-size-exclusions 3 \
  --max-stack-resets 2 --campaign-timeout-sec 2700 \
  --steady-start support-window-ready --steady-window-sec 8.0 \
  --recovery-limit-sec 1.4 --ros-domain-id 7 --overlay /tmp/pfr10_g6
```

Pass requires three consecutive accepted slots. One `eligible_fail` stops the
campaign. Exhausting the six standard seeds without six eligible standard
passes is `perception_availability_blocked`, not a fail, and must not call
`agent_complete`.

Aggregation: per case first; no cross-case averaging. One live campaign.

## Required offline tests

On the clean G5 revision, before acquiring the simulator:

1. Focused classifier/window/campaign/G4 suite three consecutive times with
   zero failure, error, skip or retry:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=src/luggage_perception:src/luggage_perception/test/eval \
python3 -m pytest -q --tb=short \
  src/luggage_perception/test/eval/test_pf_r7_g4_steady_window.py \
  src/luggage_perception/test/eval/test_pf_r7_classifier.py \
  src/luggage_perception/test/eval/test_pf_r7_campaign.py \
  src/luggage_perception/test/eval/test_pf_r7_score_window.py \
  src/luggage_perception/test/eval/test_pf_r7_g5_scan.py
```

2. Gate4 evaluator 37 passed once: `src/luggage_perception/test/test_pf_g4h_evaluator.py`.
3. PF-R10 RSS 10 passed once: `src/luggage_perception/test/test_pf_r10_g6s_rss.py`.

New focused tests must prove: scan stop-at-six, five-class recording,
`--seed-matrix-json` replaces only `standard`, and carryon/large remain the
generation-4 matrix.

## Evidence and dump matrix

Evidence root:
`docs/status/evidence/platform_free_height/2026-09-15_pfr7_g5/rev_<commit>/`.

Maximum retained size: 200 MiB per scanned seed, 500 MiB per live slot, 1.5 GiB
per live campaign plus 2 GiB for the scan tree. Compress JSONL/NPZ where
helpful. Keep failed, excluded, and available scan cases. Delete only
incomplete aborted launches after the manifest records `capture_complete=false`.
Expected T1 overhead below 5% wall time and 75 MiB RSS. Capture-health gate:
every required attempt manifest has `capture_complete=true` and
`replay_possible=true`, or the run is incomplete evidence.

| Boundary | T1 trace | Failure trigger | T2/T3 payload | Window | Format / budget | Replay or inspection |
|---|---|---|---|---|---|---|
| scan proposal availability | seed_id, xy/yaw, stamps, rates, TF, bbox/class/confidence, production detections, instance/generation, queue/drop counts | no accepted proposal by 1.4 s, clock/rate fault, dump gap | RGB, depth, camera info, TF, raw/eval proposals, separate GT, miss ledger | 1 s before spawn through 2 s after trigger or proposal | MCAP + JSON/PNG, <=50 MiB per seed | `scan_verdict.json` plus dump `manifest.json` |
| sensor/TF to proposal | stamps, rates, TF, bbox/class/confidence, instance/generation, queue/drop counts | no accepted proposal by 1.4 s, clock/rate fault | RGB, depth, camera info, TF, raw/eval proposals and separate GT | 1 s before spawn through 2 s after trigger | MCAP + JSON/PNG, <=50 MiB per attempt | classifier replay and manifest inspection |
| proposal to exact-stamp measure | cargo/raw stamps, source, join/gate reason, instance/generation | accepted proposal but missing/mismatched measure | proposal, mask, synchronized depth/cloud, filtered cargo cloud, join buffers | 1 s before through 2 s after divergence | MCAP + NPZ/JSON, <=60 MiB | boundary replay |
| support readiness and steady window | admitted flag, history owner/count/size, candidate/inlier/residual/side coverage, `t_proposal`, `t_steady`, start/end `/clock` | readiness >1.4 s, incomplete interval, rate/geometry failure | all transition and scored rows, support inputs/candidates, raw/cargo clouds, exact window manifest | proposal through 2 s after fixed window or trigger | JSONL + NPZ, <=100 MiB normal, <=200 MiB failed case | fixed-window replay |
| geometry to verdict | category, errors, numerator/denominator, class decision | threshold or class inconsistency | exact classifier inputs/outputs and reference | full bounded attempt | JSONL, <=10 MiB | verdict replay |
| lifecycle/resources | state transitions, PID/process/clock/GPU, buffers, RSS and lag | timeout, duplicate clock, crash, resource bar | logs, process snapshots, C2 raw/summary and reset ledger | 5 s before trigger through teardown proof | JSONL/text, <=25 MiB | reset tests and verdict inspection |

T0/T1 are mandatory for every scanned seed and every live attempt. T2 is
mandatory for every exclusion or failure. T3 is enabled only after T2
localizes an unresolved module and is not timing-scored. Save GT pose/size
separately from online inputs.

## Completion

PF-R7 generation 5 passes only when the clean-revision rule, offline tests,
scan evidence with six detector-available standard seeds, all generation-4
per-case metrics, RSS/lag/resource bars, three accepted slots, evidence health
and zero-residual teardown all pass. Do not call `agent_complete` until that
pass, or until reviews issues a superseding closure rule.

If generation 5 again exhausts detector availability without an eligible
failure, record `perception_availability_blocked` with scan/live evidence and
ask reviews whether to decouple PF-R7 geometry acceptance from YOLO
standard-class availability.
