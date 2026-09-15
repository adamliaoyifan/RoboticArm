# PF-R7 generation 6 offline-locked low-confidence live

Date: 2026-09-15

Parent: `PFH-REMEDIATION-20260904`

Subtask: `PF-R7`

Base revision: `5f432ac8796fa5cc86c67dfa8d21191cb0c6e9c7`

Owner: `test/cursor/grok-4.6`

## Review decision

Generation 5 remains `inconclusive/time_budget`, not pass and not an eligible
product failure. Its standard-seed scan is accepted: the first six
detector-available standard seeds are `00/01/02/06/07/09`, while `08` is a
valid scan miss. Do not rerun the scan.

Generation 5 live cannot close PF-R7. Ultralytics attempted to install CLIP
during the two `carryon_03/04` failure dumps, consuming 540 s and 426 s. Both
attempt manifests also have `best_confidence=null` and `best_iou=null` while
labelling the attempts `known_detector_miss`. That violates the unchanged full
miss proof requiring a matching eval trace with IoU `>=0.50` and confidence
`<0.20`. These attempts are evidence/infrastructure invalid, not valid
detector-miss exclusions.

Generation 6 supersedes G5 for one offline-locked live campaign. It repairs
the environment and fail-closed evidence boundary before taking the simulator
slot. It does not change production detection, geometry, recovery or scoring
thresholds.

## Immutable workload and thresholds

- Production anchor: `60dafb7deee50a6f3a76d48076b743bf3e3e1bc8`.
- G6 base/evaluator ancestor: `5f432ac8796fa5cc86c67dfa8d21191cb0c6e9c7`.
- Standard seeds, in order: `standard_00`, `standard_01`, `standard_02`,
  `standard_06`, `standard_07`, `standard_09`.
- Carryon and large matrices remain exactly the G4/G5 matrices.
- Production confidence floor `0.20`; valid-top and `FULL_3D` rate floors
  `0.95`; output rate `>=4 Hz`.
- Support-ready starts at admitted support window count `5`; score the
  half-open `[t_steady, t_steady+8.000s)` interval.
- `t_first_valid_top`, `t_first_FULL_3D` and `t_steady` must each be
  `<=t_proposal+1.4s`; all G4 geometry P95/max bars remain unchanged.
- Three consecutive slots, two eligible cases per size per slot; at most 12
  attempts and six exclusions per slot, three consecutive same-size
  exclusions, two stack resets, 36 campaign attempts and 2700 s total.
- Executor-lag Q4 mean `<=0.20s` and `<=1.25*Q1`; workload-adjusted RSS
  `<=2 MiB/min`; teardown residuals `0`.
- One eligible failure stops the campaign. Per-case scoring only; no averaging
  across failures and no threshold retuning.

## Offline-lock requirement

Before any import of `ultralytics`, the driver and every child environment
must set:

```text
YOLO_OFFLINE=1
ULTRALYTICS_OFFLINE=1
YOLO_AUTOINSTALL=false
```

The live harness must have a preflight that runs after sourcing the exact
overlay and before launching Gazebo. It must:

1. import `ultralytics.utils` in a fresh child process and record
   `AUTOINSTALL=False`;
2. record that all three environment values above reached the child;
3. prove the low-confidence dump dependency is already locally usable, or
   stop before simulator acquisition with `offline_dependency_unavailable`;
4. run one saved G5 miss dump through the low-confidence evaluator with
   network disabled and finish within `30s` with a matching bbox trace;
5. record no `pip`, package-manager, `git clone`, `curl` or download child and
   no package/cache mutation during the preflight.

Do not install, download or repair dependencies while Gazebo is running. Do
not silently disable the low-confidence trace to make the preflight pass.

## Fail-closed miss proof

A live attempt may be `known_detector_miss` only when all of these are present:

- stable upright GT is wholly in frame and in the pickup workspace;
- healthy camera, exact-stamp TF, inference, controller and exactly one clock;
- unchanged production floor and production `detections=[]` by the deadline;
- low-confidence eval trace contains a cargo proposal associated to that GT
  with IoU `>=0.50` and best confidence `<0.20`;
- no cargo mask/cloud/geometry entry, no false cargo and a complete replayable
  dump.

Missing/empty eval trace, null/non-finite `best_confidence` or `best_iou`, CLIP
dependency error, evaluator timeout, or unjoinable trace must classify as
`evidence_invalid` or `infrastructure_invalid`, stop the scored campaign, and
preserve T2. It must never advance or consume the known-miss exclusion budget.

Add focused regression cases for absent trace, null fields, IoU `<0.50`,
confidence `>=0.20`, offline dependency failure and valid low-confidence miss.

## Revision and procedure

The tested evaluator worktree and overlay must be the same clean exact commit,
a descendant of the G6 base containing only the offline-lock, preflight,
fail-closed proof and associated tests/evidence changes. Both must report empty
`git status --porcelain --untracked-files=all` for all scoped paths.

Required offline tests before simulator acquisition:

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

Run that focused suite three consecutive times with zero failure, error, skip
or retry. Then run once each:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_perception \
python3 -m pytest -q src/luggage_perception/test/test_pf_g4h_evaluator.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/luggage_perception \
python3 -m pytest -q src/luggage_perception/test/test_pf_r10_g6s_rss.py
```

Run the recorded offline-lock preflight. Only after every offline condition
passes, verify the exclusive slot, clean-room residuals, one expected clock
after launch and normal camera/inference rates. Execute exactly one G6 live
campaign using the accepted G5 seed matrix and the unchanged G5 command/budget
surface. Do not run another seed scan.

## Evidence

Evidence root:
`docs/status/evidence/platform_free_height/2026-09-15_pfr7_g6/rev_<commit>/`.

Required T0 additions are `offline_preflight.json`, child environment values,
Ultralytics version and `AUTOINSTALL` value, dependency provenance, saved-dump
low-confidence result/duration, process audit, cache/package before-after
manifest, exact imported G5 seed-matrix hash, revision/dirty state and command.

For every live case retain the G5 T1 trace. On missing proposal, evaluator
timeout/dependency error, time-budget event or threshold failure, freeze T2
before teardown: RGB/depth/camera info/TF, production and low-confidence
proposals, separate GT, mask/cloud/geometry boundaries, timing/process trace,
classifier input/output and manifest with `capture_complete` and
`replay_possible`.

Limits remain 500 MiB per live slot and 1.5 GiB per campaign. Low-confidence
preflight artifacts are capped at 100 MiB. T1 overhead must remain below 5%
wall time and 75 MiB RSS. T3 is allowed only after T2 localizes an unresolved
failure and is not timing-scored.

## Completion

PF-R7 G6 passes only when offline tests/preflight, the imported matrix, all
per-case G4 bars, C2 lag/RSS, three accepted slots, evidence health and zero
residual teardown pass on one clean exact revision. Then the owner calls
`agent_complete.sh --outcome pass`.

Any eligible failure, evidence-invalid attempt, infrastructure-invalid
attempt, offline dependency failure or budget exhaustion is not a pass. Record
the exact reason and complete evidence, stop cleanly, and return to reviews;
do not auto-rerun.
