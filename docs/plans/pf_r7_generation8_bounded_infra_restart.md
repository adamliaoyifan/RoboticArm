# PF-R7 generation 8 bounded infrastructure restart and one scored live

Date: 2026-09-15

Parent: `PFH-REMEDIATION-20260904`

Subtask: `PF-R7`

Generation: `8`

Owner: `test/cursor/grok-4.6`

Base revision: `59b5060cbc18ffcc16aa1faca39ad70fe6b918e2`

Dependencies: `PF-R1,PF-R2,PF-R3,PF-R4,PF-R5,PF-R6,PF-A1,PF-R8,PF-R9,PF-R10`

## Decision and objective

Generation 7 remains `inconclusive/infrastructure_invalid`, not pass and not
an eligible product failure. Its clean offline gate and deterministic
nine-standard-seed scan passed. Its only allowed live stopped before case 1
with `n_attempts=0`: the `gz_ros2_control` request for
`robot_state_publisher/robot_description` did not complete, so
`controller_manager`, both controllers and `/joint_states` never appeared.
There was exactly one `/clock` publisher and teardown residuals were zero.

The same G7 revision and launch configuration had loaded `controller_manager`
in about 2.7 s and activated both controllers in about 4.4 s during scan. G8
therefore treats the G7 live result as a bounded startup/RMW service or
relaunch-isolation defect. It does not waive controller readiness and does not
change PF-R7 product thresholds.

G8 supersedes G7. It imports the hash-locked G7 offline and scan result, does
not run another seed scan, repairs or gates the startup chain, and runs one
scored live campaign. It may perform at most one controlled infrastructure
restart before scoring under the exact whitelist below. At most two Gazebo
launches are permitted: the initial launch and one pre-scoring restart. Only
one launch may enter the scored campaign.

## Immutable workload and acceptance bars

- Production anchor: `60dafb7deee50a6f3a76d48076b743bf3e3e1bc8`.
- G8 base/evaluator ancestor: `59b5060cbc18ffcc16aa1faca39ad70fe6b918e2`.
- Production proposal confidence floor: `0.20`.
- Conditional valid-top and `FULL_3D` rate floors: `0.95` each.
- Output rate: `>=4 Hz`.
- Support-ready starts at admitted `support_window_count == 5`.
- Score the half-open `[t_steady, t_steady + 8.000s)` interval.
- `t_first_valid_top`, `t_first_FULL_3D` and `t_steady` must each be
  `<=t_proposal+1.4s`.
- All G4 geometry P95/max bars remain unchanged.
- Three consecutive slots, two eligible cases per size per slot.
- At most 12 attempts and six exclusions per slot, three consecutive
  same-size exclusions, two stack resets, 36 campaign attempts and 2700 s
  scored-campaign wall timeout.
- Executor-lag Q4 mean `<=0.20s` and `<=1.25*Q1`.
- Workload-adjusted RSS `<=2 MiB/min`.
- Teardown residuals exactly `0`.
- One eligible failure stops the campaign. Per-case scoring only; no averaging
  across failures and no threshold retuning.

The frozen matrices are immutable and seeds are consumed without wrapping or
reuse. Standard is exactly:

```text
standard_00, standard_01, standard_06, standard_07, standard_09,
standard_10, standard_12, standard_13, standard_14
```

Carryon and large remain the exact 16-seed G7 matrices.

## Hash-locked G7 evidence import

G8 must validate and import these exact G7 artifacts before implementation
tests or simulator acquisition:

| Artifact | Required SHA-256 |
|---|---|
| `preflight/offline_preflight.json` | `7abefa60e68997505e6ba7b18cc075ef367ab3873f2beff63fabf449f0eb5b6a` |
| `scan/scan_verdict.json` | `28238d4ea5d6524b921ce3fba304d4022ce03dfeffe720452567b588a495137a` |
| `scan/live_seed_matrix.json` | `1a05fbbce8e000c2112082187f32f95cd3486169b850becb890733fd1529d08b` |
| `scan/available_standard_seeds.json` | `70894e108ce7f4987cc368e92a7b9fa0abb05a821d2bc7dfb86f86e6519da24c` |
| `scan/imported_prior_standard.json` | `07bcb9658ef573dec70fc43eb7442b4883bc043e52b4a82d91d432ba6a139c01` |

Source root:
`docs/status/evidence/platform_free_height/2026-09-15_pfr7_g7/rev_59b5060cbc18ffcc16aa1faca39ad70fe6b918e2/`.

The importer must require source evaluator/overlay commit `59b5060...`, dirty
counts zero, offline outcome pass, scan outcome pass,
`reason=nine_proposal_available`, `live_unlock=true`, exactly nine frozen
standard IDs in the order above, and byte-identical carryon/large matrices.
Write `import/g7_import_manifest.json` with all paths, hashes and validation
results. Any mismatch is `evidence_invalid`; stop before Gazebo. Do not rescan,
replace, reorder or append a seed.

## Revision and bounded implementation scope

Create clean branch/worktree `agent/test/pfr7-g8` at `/tmp/pfr7_g8` from the
exact G8 base. The final evaluator worktree and built overlay must be the same
clean exact descendant commit. Record full hashes and tracked/untracked dirty
counts; both dirty counts must be zero.

Allowed scope:

- a G8 eval driver and startup gate under `scripts/pf_r7_*`;
- PF-R7 eval/startup helpers and focused tests under
  `src/luggage_perception/.../eval/` and `src/luggage_perception/test/eval/`;
- only if required to make ordering deterministic,
  `src/luggage_gazebo/launch/sim_world.launch.py` and a bounded simulation
  readiness helper;
- G8 evidence and the required test role note.

Do not change the robot xacro, controller YAML, production perception/planning
logic, D555/Livox profiles, scheduler or site runtime unless new evidence
proves a deterministic defect there and reviews issues a higher generation.
No normative architecture change is authorized.

The final entry point is
`/tmp/pfr7_g8/scripts/pf_r7_generation8_live.sh` with `offline` and `live`
modes. A `scan` mode must be rejected. Record fully expanded commands,
environment, exit codes and launch-attempt index.

## Offline verification

Every driver/child must set before any Ultralytics import:

```text
YOLO_OFFLINE=1
ULTRALYTICS_OFFLINE=1
YOLO_AUTOINSTALL=false
```

On the exact final G8 commit, rerun the strict saved G6 `standard_02` replay
once without network/install. It must complete within 30 s with a finite GT
bbox, confidence `<0.20`, IoU `>=0.50`, `AUTOINSTALL=False`, no installer
process and no package/cache mutation. Null-bbox/null-IoU fallback remains
forbidden.

Run the focused suite below three consecutive times with zero failure, error,
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
  src/luggage_perception/test/eval/test_pf_r7_g7_seed_reserve.py \
  src/luggage_perception/test/eval/test_pf_r7_g8_startup_restart.py
```

Then run once each, also with zero failure, error, skip or retry:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_perception \
python3 -m pytest -q --tb=short \
  src/luggage_perception/test/test_pf_g4h_evaluator.py

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_perception \
python3 -m pytest -q --tb=short \
  src/luggage_perception/test/test_pf_r10_g6s_rss.py
```

New tests must prove import hash/order rejection, scan-mode refusal, staged
startup transitions and deadlines, whitelist/non-whitelist decisions, exactly
one retry maximum, retry refusal after case start, evidence-health and
residual gates, identical revision/config/matrix hashes across restart, and
one scored-campaign maximum.

Store exact commands, three focused logs, Gate4/RSS logs, strict preflight,
test counts and exit codes under `offline/`.

## Deterministic startup gate

Environment: exclusive Gazebo, `gui:=false`, `use_rviz:=false`, initial
`ROS_DOMAIN_ID=7`, PID file `/tmp/elfin_humble_sim.pid`. Queue behind any live
user/agent simulator. Require zero owned sim/bridge/GPU residuals before each
launch and exactly one `/clock` publisher after launch.

Before case 1, record these stages and wall/ROS timestamps:

1. `robot_state_publisher` exists and its parameter service completes a real
   `get_parameters(robot_description)` round trip with nonempty URDF within
   10 s;
2. `gz_ros2_control` records successful robot-description retrieval and
   `/controller_manager/list_controllers` responds within 30 s;
3. `joint_state_broadcaster` and `elfin_arm_controller` are both `active`, and
   one finite six-joint `/joint_states` sample is received within 60 s total;
4. exactly one `/clock`, expected camera/inference rates, semantic segmenter,
   pickup-box spawner and observe-pose readiness pass before scoring.

The global startup deadline is 75 s. Do not wait 240 s after the controller
manager creation path has already failed. A successful gate writes
`startup/attempt_<n>/ready.json` and hands the same still-running stack to the
single scored campaign; do not tear it down between readiness and case 1.

Implementation may delay or gate robot/controller creation on the successful
robot-description service round trip, but must not bypass the plugin's own
retrieval or fabricate `/joint_states`.

## One bounded pre-scoring infrastructure restart

One restart is permitted only when every condition holds:

- failure occurs before case 1 and `n_attempts == 0`;
- no pickup case was spawned or scored;
- exact commit, overlay, launch args, production config and frozen matrix
  hashes are unchanged;
- exactly one `/clock` publisher existed; duplicate clocks are contamination,
  not whitelisted;
- the first failure is one of:
  `robot_state_publisher_get_parameters_timeout`,
  `controller_manager_service_absent`,
  `controller_spawner_died_controller_manager_absent`, or
  `joint_states_absent_after_controller_startup`;
- T2 startup evidence is flushed and its manifest has
  `capture_complete=true`, no required missing artifacts, and an exact replay
  command/environment;
- `scripts/stop_sim.sh` completes, the PID file is absent, and bounded process
  inspection proves zero owned simulator/bridge/eval residuals;
- a 10 s wall-clock quiescence interval and a fresh clean-room check pass.

The retry uses the same predeclared ROS domain/config. Record restart count 1
and link both startup manifests. It is not a second scored live because no case
was attempted on the first stack. There is no second restart.

Do not restart for dirty/hash mismatch, offline/cache failure, duplicate
clock, user/other-agent simulator conflict, perception/planning node crash,
seed/evidence defect, eligible failure, any attempted case, incomplete startup
capture, teardown residual or unknown reason. Those conditions stop G8.

If startup attempt 2 fails for any reason, record
`inconclusive/infrastructure_invalid`, preserve evidence and stop. Do not run a
higher hidden retry.

## Single scored live procedure

After one startup attempt passes, run exactly one scored campaign on that
stack with the frozen G7 matrix. Required workload, attempt/exclusion/reset
budgets, scoring semantics, recovery/window/geometry thresholds and C2 bars
are exactly the immutable contract above.

One eligible failure, evidence-invalid attempt, infrastructure-invalid attempt
after scoring begins, seed exhaustion, budget/time stop, C2 failure or capture
health failure stops the campaign. Do not restart after case 1, reuse a seed,
average away a failure, retune a threshold or run another scored campaign.

Pass requires six eligible cases per slot, three consecutive `slot_pass=true`
summaries, every per-case G4 bar, C2 lag/RSS, evidence/capture health and zero
residual teardown on one clean exact G8 revision.

## Exact commands and ordering

From `/tmp/pfr7_g8`:

```bash
cd /tmp/pfr7_g8
AGENT_COORD_ROOT=/home/adamliao/work/elfin_humble_ws \
  scripts/pf_r7_generation8_live.sh offline
```

Only after offline/import/tests pass and the exclusive simulator slot is free:

```bash
cd /tmp/pfr7_g8
AGENT_COORD_ROOT=/home/adamliao/work/elfin_humble_ws \
  scripts/pf_r7_generation8_live.sh live
```

The driver derives the evidence path from the full clean tested commit:
`docs/status/evidence/platform_free_height/2026-09-15_pfr7_g8/rev_<full-commit>/`.
It must refuse a revision mismatch and must never overwrite G7 evidence.

## Evidence and dump matrix

T0 includes revision/dirty/overlay provenance, G7 import manifest, exact
commands/environment/profile, config/matrix hashes, startup-attempt ledger,
offline/test results, live/slot/C2 verdicts, artifact inventory, disk sizes and
teardown status. T1 is mandatory for every startup stage and scored case. T2
must be durable before every restart, teardown or return to reviews.

| Boundary | Always-on T1 | Failure trigger | T2 payload | Window | Format / budget | Replay or inspection |
|---|---|---|---|---|---|---|
| G7 evidence import | source path/hash/commit/dirty, parsed outcome, matrix IDs/order | hash/schema/outcome/order mismatch | source and normalized manifests, validation reasons | full import | JSON, <=10 MiB | import unit test and manifest verifier |
| RSP to gz_ros2_control | node/service discovery, request/response timestamps, URDF byte count/hash, plugin state | response timeout/empty URDF/plugin retrieval failure | launch log, ROS graph/service/type snapshots, request result, process/RMW/env snapshot | 5 s before through 5 s after trigger | JSONL/text, <=25 MiB | exact startup command and graph inspection |
| controller manager and spawners | service discovery, controller states, spawner PIDs/exits, joint sample/rate | service absent, spawner death, inactive controller, missing joint states | list/controller responses, spawner logs, joint sample, process/graph snapshots | 5 s before through 5 s after trigger | JSONL/text, <=30 MiB | startup gate replay/inspection |
| sensor/TF to proposal | stamps/frame/generation IDs, camera/inference rates, TF results, proposals, queue/drop counts | no proposal, TF/rate/clock fault | RGB/depth/camera info/TF, production/eval proposals, separate GT | 1 s before through 2 s after trigger | MCAP + JSON/PNG, <=60 MiB/case | classifier/boundary replay |
| proposal to support/geometry | exact-stamp joins, masks/clouds, support candidates, readiness/window timestamps and geometry metrics | join/readiness/window/geometry failure | synchronized inputs, masks/clouds, candidates, classifier input/output and reference | proposal through 2 s after window/trigger | JSONL + NPZ, <=200 MiB failed case | fixed-window replay |
| lifecycle and C2 | launch/restart state, PID/clock/GPU, reset ledger, lag/RSS/resource samples | timeout, duplicate clock, crash, C2/residual failure | both startup manifests, launch/process logs, C2 raw/summary, teardown proof | 5 s before trigger through teardown | JSONL/text, <=40 MiB/launch | verdict inspection and stop audit |

Retention: `<=250 MiB` per failed startup, `<=500 MiB` per live slot,
`<=1.5 GiB` scored campaign and `<=2.5 GiB` total G8 root. Expected T1
overhead `<5%` wall time and `<75 MiB` RSS. T3 is allowed only after T2
localizes an unresolved component and is not timing-scored.

Every failure manifest names launch/case, stage and first failure, exact
revision/config/seed hashes, clock domain, artifacts, capture window, missing
items, sizes, `capture_complete` and replay command/status. Missing, truncated
or unjoinable required evidence forbids restart and makes the run incomplete.

## Completion and stop rules

G8 passes only when the exact clean final descendant satisfies all offline and
import gates, one startup attempt (with at most one permitted pre-scoring
restart) reaches readiness, exactly one scored live passes three slots and all
per-case/C2/evidence/residual bars pass. The owner then commits code/tests,
writes the test role note and calls `scripts/agent_complete.sh --outcome pass`
with the exact commit and evidence root.

Any non-whitelisted or repeated startup failure, scored failure, invalid
evidence/infrastructure, seed/budget exhaustion, C2 failure or residual is not
pass. Preserve the required bundle, stop the owned stack, record the exact
reason and return to reviews. Do not change the frozen seeds or product bars,
and do not run a second scored live.
