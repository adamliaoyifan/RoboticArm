# PF-R7 generation 7 deeper standard matrix and one bounded live

Date: 2026-09-15

Parent: `PFH-REMEDIATION-20260904`

Subtask: `PF-R7`

Generation: `7`

Owner: `test/cursor/grok-4.6`

Base revision: `d08963605c1fdbcb26a5a573ddeb510ad6aa1a73`

Dependencies: `PF-R1,PF-R2,PF-R3,PF-R4,PF-R5,PF-R6,PF-A1,PF-R8,PF-R9,PF-R10`

## Decision and objective

Generation 6 remains `inconclusive/seed_exhausted`; it is neither a pass nor
an eligible product failure. It produced 16 `eligible_pass`, five valid
`known_detector_miss`, zero `eligible_fail`, complete/replayable attempt
evidence and zero residuals. Slots 1 and 2 passed. Slot 3 stopped because the
six imported standard seeds yielded only five eligible standard cases after
`standard_02` became a valid live known-detector-miss.

Generation 7 supersedes G6. It conservatively imports the newest standard
classification for every previously observed seed, scans only the unscanned
tail `standard_10` through `standard_15`, and permits one bounded live campaign
only after at least nine proposal-available standard seeds have been frozen.
Nine is the six unique standard cases required by three slots plus three
predeclared substitutes. The live still fails closed if that reserve is
insufficient; there is no online seed search and no automatic second live.

The task is eval-only. It must not change production detection, geometry,
support readiness, recovery, planning, D555/Livox configuration, scheduler or
site runtime behavior. No normative architecture document changes are needed.

## Immutable production and scoring contract

- Production anchor: `60dafb7deee50a6f3a76d48076b743bf3e3e1bc8`.
- G7 evaluator base: `d08963605c1fdbcb26a5a573ddeb510ad6aa1a73`.
- Production proposal confidence floor: `0.20`.
- Conditional valid-top rate floor: `0.95`.
- Conditional `FULL_3D` rate floor: `0.95`.
- Output rate: `>=4 Hz`.
- Support-ready begins at admitted `support_window_count == 5`.
- Score the half-open `[t_steady, t_steady + 8.000s)` interval.
- `t_first_valid_top`, `t_first_FULL_3D` and `t_steady` must each be
  `<= t_proposal + 1.4s`.
- All G4 geometry P95/max error bars remain unchanged.
- Three consecutive slots; two eligible cases per size per slot.
- At most 12 attempts and six exclusions per slot, three consecutive
  same-size exclusions, two stack resets, 36 campaign attempts and 2700 s
  total.
- Executor-lag Q4 mean `<=0.20s` and `<=1.25 * Q1`.
- Workload-adjusted RSS `<=2 MiB/min`.
- Teardown residuals: exactly `0`.
- One eligible failure stops the campaign. Score per case; do not average away
  a failing case and do not retune a threshold.

## Revision and bounded implementation scope

Create a clean satellite worktree and branch `agent/test/pfr7-g7` from the
exact G7 base. The final evaluator worktree and built overlay must be the same
clean exact descendant commit. Record both full hashes and both dirty counts;
each dirty count must be zero for all tracked and untracked scoped paths.

Allowed implementation scope:

- G7 eval driver and seed-import/scan controls under `scripts/pf_r7_*`;
- PF-R7 campaign/classifier/offline-lock helpers under
  `src/luggage_perception/luggage_perception/eval/`;
- focused PF-R7 eval tests under `src/luggage_perception/test/eval/`;
- G7 evidence and the required test role note.

The owner chooses implementation details inside this scope. The final entry
point must be `/tmp/pfr7_g7/scripts/pf_r7_generation7_live.sh` with `offline`,
`scan`, and `live` modes. Every mode must write its fully expanded commands,
environment and exit code beneath the G7 evidence root.

## Offline lock and strict saved-dump preflight

Before any `ultralytics` import in the driver or a child, set and record:

```text
YOLO_OFFLINE=1
ULTRALYTICS_OFFLINE=1
YOLO_AUTOINSTALL=false
```

The exact overlay preflight must:

1. prove a fresh child sees all three values and
   `ultralytics.utils.AUTOINSTALL == False`;
2. prove the local CLIP/low-confidence dependency is usable without network,
   install or cache/package mutation;
3. replay the G6 `standard_02` miss bundle at
   `docs/status/evidence/platform_free_height/2026-09-15_pfr7_g6/rev_d08963605c1fdbcb26a5a573ddeb510ad6aa1a73/live/slot2/attempt08`
   within `30s`;
4. require a finite projected or saved GT bbox, a finite best confidence
   `<0.20`, and a finite best IoU `>=0.50` for that replay;
5. reject the G6 fallback that accepted `gt_bbox=null` / `iou=null` merely
   because a below-floor cargo box existed;
6. record no `pip`, package-manager, `git clone`, `curl`, downloader or
   installer child and identical before/after cache/package fingerprints.

Any absent/null/non-finite confidence, bbox or IoU, timeout, dependency error,
process audit hit or mutation stops before simulator acquisition as
`offline_dependency_unavailable` or `evidence_invalid`.

## Deterministic standard-seed import and scan

Import prior classifications without rerunning `standard_00` through
`standard_09`. The G6 live observation overrides the older G4/G5 scan when the
same seed differs:

| Imported class | Seeds | Source |
|---|---|---|
| `proposal_available` | `standard_00,01,06,07,09` | accepted G5 scan and G6 eligible live cases |
| `known_detector_miss_valid` | `standard_02` | G6 live attempt 08, confidence 0.0243 and IoU 0.9475 |
| `known_detector_miss_valid` | `standard_03,04,05,08` | accepted G4/G5 scan evidence |

Write these ten rows to `scan/imported_prior_standard.json`, including source
artifact path, source commit and SHA-256. Reject duplicate seed IDs, conflicting
classes, non-finite proof fields or a hash mismatch.

Scan the existing predeclared standard tail exactly once in this order:

| Seed | XY (m) | Yaw (rad) |
|---|---:|---:|
| `standard_10` | `[-0.08, 0.00]` | `-0.15` |
| `standard_11` | `[0.00, 0.08]` | `0.05` |
| `standard_12` | `[0.06, 0.02]` | `-0.05` |
| `standard_13` | `[-0.06, -0.02]` | `0.25` |
| `standard_14` | `[0.02, 0.06]` | `-0.25` |
| `standard_15` | `[-0.02, -0.06]` | `0.00` |

Each scanned seed receives exactly one G5 class:
`proposal_available`, `known_detector_miss_valid`, `fixture_invalid`,
`infrastructure_invalid`, or `evidence_invalid`. The valid known-miss proof is
the unchanged G6 proof: stable upright GT in frame/workspace, healthy
camera/TF/inference/controller/single clock, production `detections=[]`, a
GT-associated low-confidence proposal with finite IoU `>=0.50` and confidence
`<0.20`, no cargo cloud/geometry or false cargo, and a complete replayable
dump.

Start with five imported `proposal_available` seeds and stop scanning as soon
as the total reaches nine. Freeze those first nine, in deterministic seed
order, into `scan/live_seed_matrix.json`. Carryon and large must byte-match the
G6 imported matrices. If fewer than nine are available after `standard_15`,
record `inconclusive/perception_availability_blocked`, tear down cleanly and do
not start live.

Scan environment: exclusive Gazebo, `gui:=false`, `use_rviz:=false`,
`ROS_DOMAIN_ID=7`, PID file `/tmp/elfin_humble_sim.pid`, exactly one `/clock`
publisher after launch. Scan observe ends on a replayable production proposal,
a complete valid known-miss proof, or the 1.4 s proposal deadline plus dump
flush. Do not spend the 8.000 s geometry window in scan mode.

Scan repetition and aggregation: one deterministic scan campaign, at most six
new seeds, no repeated availability vote and no online best-seed selection.
`proposal_available` is a categorical per-seed observation; it is not averaged.

## Live workload and procedure

Only a passing scan with exactly nine frozen proposal-available standard seeds
may unlock live. Standard uses those nine in frozen order. Carryon and large
remain the unchanged 16-seed G6 matrices. Seeds are consumed without wrapping
or reuse across slots.

Run exactly one live campaign. Before it, stop the scan stack with
`scripts/stop_sim.sh`, require no owned sim/bridge/GPU residuals, then acquire
the exclusive slot again. Queue behind any existing user/agent simulator;
never launch a second Gazebo world.

The required workload is three consecutive accepted slots with two eligible
carryon, two eligible standard and two eligible large cases in each slot. A
valid exclusion consumes a frozen same-size substitute. An eligible failure,
evidence-invalid attempt, infrastructure-invalid attempt, duplicate clock,
threshold failure or exhausted budget stops the campaign after evidence is
durable. If nine standard seeds are still exhausted before six eligible
standard cases, record `inconclusive/seed_exhausted`; do not auto-rerun.

Per-case pass observations are the immutable contract above plus every G4
geometry P95/max bar. Campaign pass additionally requires three slot summaries
with `slot_pass=true`, C2 lag/RSS pass, all evidence-health gates pass and zero
teardown residuals.

## Exact commands and ordering

From a clean worktree `/tmp/pfr7_g7` on `agent/test/pfr7-g7`:

```bash
cd /tmp/pfr7_g7
AGENT_COORD_ROOT=/home/adamliao/work/elfin_humble_ws \
  scripts/pf_r7_generation7_live.sh offline
```

The offline mode must build the exact overlay, run the suites below, run the
strict preflight, and exit nonzero before Gazebo if any gate fails. After it
passes and only when the exclusive slot is free:

```bash
cd /tmp/pfr7_g7
AGENT_COORD_ROOT=/home/adamliao/work/elfin_humble_ws \
  scripts/pf_r7_generation7_live.sh scan
```

After scan teardown, hash validation, nine-seed gate and a fresh clean-room
check pass:

```bash
cd /tmp/pfr7_g7
AGENT_COORD_ROOT=/home/adamliao/work/elfin_humble_ws \
  scripts/pf_r7_generation7_live.sh live
```

The driver must derive the evidence path from the full clean tested commit:
`docs/status/evidence/platform_free_height/2026-09-15_pfr7_g7/rev_<full-commit>/`.
It must refuse an evidence directory whose recorded revision differs, and must
never overwrite G5/G6 evidence.

## Required offline tests

Run this exact focused suite three consecutive times with zero failure, error,
skip or retry:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=src/luggage_perception:src/luggage_perception/test/eval \
python3 -m pytest -q --tb=short \
  src/luggage_perception/test/eval/test_pf_r7_g4_steady_window.py \
  src/luggage_perception/test/eval/test_pf_r7_classifier.py \
  src/luggage_perception/test/eval/test_pf_r7_campaign.py \
  src/luggage_perception/test/eval/test_pf_r7_score_window.py \
  src/luggage_perception/test/eval/test_pf_r7_g5_scan.py \
  src/luggage_perception/test/eval/test_pf_r7_g6_offline_lock.py \
  src/luggage_perception/test/eval/test_pf_r7_g7_seed_reserve.py
```

Then run once each with zero failure, error, skip or retry:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_perception \
python3 -m pytest -q --tb=short \
  src/luggage_perception/test/test_pf_g4h_evaluator.py

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_perception \
python3 -m pytest -q --tb=short \
  src/luggage_perception/test/test_pf_r10_g6s_rss.py
```

New focused tests must prove at minimum:

- G6 `standard_02` overrides the older scan class;
- import rejects conflict, duplicate, bad hash and non-finite miss proof;
- only `standard_10..15` may be newly scanned and ordering is stable;
- stop-at-nine, fewer-than-nine blocking and exactly-nine live unlock;
- carryon/large are unchanged and standard has exactly nine frozen entries;
- no seed wrap/reuse and deterministic `seed_exhausted` classification;
- null GT bbox/IoU cannot pass strict offline preflight;
- valid finite IoU/confidence offline replay passes without network/install;
- every child remains offline with `AUTOINSTALL=False`.

Store three focused logs, the two regression logs, exact commands, counts and
exit codes under `offline/`. A test name or role-note summary without those
artifacts is insufficient.

## Evidence and dump matrix

Evidence root:
`docs/status/evidence/platform_free_height/2026-09-15_pfr7_g7/rev_<full-commit>/`.

T0 must include `revision.txt`, dirty counts, overlay provenance,
`commands.txt`, environment/profile, start/end wall and ROS times, imported
source paths and hashes, seed matrices and hashes, test/preflight summaries,
scan/live verdicts, artifact inventory, disk sizes and teardown status. T1 is
mandatory for every scanned/live case. T2 is mandatory for every exclusion,
failure, timeout, invalid case or seed/budget exhaustion. Flush and verify the
manifest before teardown or another attempt.

| Boundary | Always-on T1 trace | Failure trigger | T2 payload | Window | Format / budget | Replay or inspection |
|---|---|---|---|---|---|---|
| prior import and scan selection | source hash, seed ID/class/XY/yaw, override decision, scan cursor and available count | conflict, bad hash, unknown seed/class, non-finite proof | both source rows, normalized row, validation result and frozen matrix candidate | full import transaction | JSON, <=10 MiB | deterministic import unit test plus `scan_verdict.json` |
| sensor/TF to proposal | stamps/frame/generation IDs, rates, TF result, bbox/class/confidence, production/eval proposals, queue/drop counts | no accepted proposal by 1.4 s, clock/rate/TF fault | RGB, depth, camera info, TF, production and low-confidence proposals, separate GT | 1 s before spawn through 2 s after trigger | MCAP + JSON/PNG, <=60 MiB/case | classifier replay and manifest inspection |
| proposal to cloud/geometry | proposal and mask IDs, exact-stamp join, cargo cloud counts, support/geometry request IDs and reasons | missing/mismatched join, unexpected cloud/geometry on miss, false cargo | proposal, masks, synchronized depth/cloud, filtered cargo cloud, join buffers and geometry input/output | 1 s before through 2 s after divergence | MCAP + NPZ/JSON, <=80 MiB/case | boundary replay |
| support readiness and score window | admitted flag, history owner/count, candidates/inliers/residuals/coverage, proposal/steady/window timestamps | readiness >1.4 s, incomplete window, rate or geometry bar failure | transition/scored rows, support inputs/candidates, raw/cargo clouds, exact half-open window manifest | proposal through 2 s after window or trigger | JSONL + NPZ, <=100 MiB normal and <=200 MiB failed case | fixed-window replay |
| classifier to verdict | exact classifier input/output, class, reasons, numerators/denominators and thresholds | invalid class, eligible failure, null proof, seed/budget exhaustion | classifier record, references, miss ledger, slot/campaign state and seed cursor | full bounded attempt plus stop transition | JSON/JSONL, <=15 MiB/case | classifier/verdict replay |
| lifecycle and C2 | process/PID/clock/GPU state, reset ledger, lag/RSS raw samples, buffer/resource summaries | timeout, crash, duplicate clock, residual, lag/RSS failure | launch log, process snapshots, C2 raw/summary and teardown proof | 5 s before trigger through teardown | JSONL/text, <=30 MiB/run | C2 evaluator and `scripts/stop_sim.sh` audit |

Retention limits: `<=200 MiB` per scanned seed, `<=500 MiB` per live slot,
`<=2 GiB` scan tree, `<=1.5 GiB` live campaign and `<=4 GiB` total G7 root.
Expected T1 overhead is `<5%` wall time and `<75 MiB` RSS. Compress bounded
JSONL/NPZ/MCAP as needed. Retain all invalid, excluded and failing cases plus
the frozen passing summaries. T3 is allowed only after T2 localizes an
unresolved module and is not timing-scored.

Every case manifest must name case/seed, stage and first observed failure,
revision/dirty state, config, clock domain, artifacts, capture window, missing
items, sizes, `capture_complete` and `replay_possible`. Missing, truncated or
unjoinable required artifacts make the run incomplete; add capture
instrumentation before another run on that gate.

## Completion and stop rules

PF-R7 G7 passes only if one clean exact descendant revision satisfies all of:

1. three consecutive focused-suite runs and both affected regression suites;
2. strict offline lock/preflight with finite GT-associated replay IoU/conf;
3. valid prior import and at least nine frozen proposal-available standard
   seeds after at most one `standard_10..15` scan;
4. exactly one live campaign with all per-case G4 bars and three accepted
   slots;
5. C2 lag/RSS pass, evidence/capture-health pass and zero residuals.

On pass, commit code/tests, record the exact commit and evidence, write the
test role note, and call `scripts/agent_complete.sh --outcome pass` for the G7
thread. Do not report pass from a dirty worktree or branch name.

If scan inventory is below nine, live exhausts seeds, a valid eligible failure
occurs, infrastructure/evidence is invalid, C2 fails, or a budget expires,
preserve the required evidence, stop the owned stack, record the exact
categorical reason and return to reviews. Do not silently lower the reserve,
reuse seeds, change 0.20 / 0.95 / 1.4 / 8.000 s, or run a second G7 live.
