# Pack until terminal outcome — 2026-10-23

- owner: eng/codex/gpt-6
- base_revision: e1da5543

## Scope and acceptance

User requests continued packing until termination and observed final count. One frozen full-stack run, same A-prime scene, inch20 sequence, seed0, 180s selection, current attachment envelope and release XY<=5mm with Z observation. Product code/config stays unchanged. Parameterize the existing harness only: max_boxes=999 safety cap, driver wall timeout=43200s. Report MAX_BOXES or timeout as censored, not full capacity. Keep stop-on-first-failure and do not skip boxes. First terminal event ends the experiment; do not fix/retry a scored failure in this campaign. Pre-box startup retries allowed for at most1800s, preserve bundles and clean between attempts.

Acceptance is a complete observed run report, not a prescribed count: count verified committed ledger entries, reconcile suite boxes_packed, identify last successful case and first failed stage or genuine BIN_FULL. Every claimed fully closed box must have release/verify/HOME success and one commit. Occupancy committed before failed HOME must be reported separately. Distinguish planning/candidate/infra stops from geometric capacity. One workload/run is descriptive, no reliability estimate; no minimum count requirement. Freeze commit+dirty state and source/install hashes before launch, no product edits during run.

## Procedure

PHASE2_MAX_BOXES=999 PHASE2_DRIVER_TIMEOUT_S=43200 PHASE2_RUN_OUT=<root>/live_01 bash scripts/g3/n3_functional_run.sh. Harness defaults remain n3/4200s; bash -n validates syntax. Prior successful startup rehearsal and n3 on same product provide rehearsal evidence; new startup still checks gain, URDF, camera, clock and capture health. One Gazebo only domain7 gui=false. Owned teardown immediately at exit. Audit ledger/trials/suite, preserve/copy/hash evidence to primary and commit bounded report/index.

## Dump matrix

| Boundary | T1 trace | Trigger | T2 payload/window | Format/budget | Inspection |
|---|---|---|---|---|---|
| startup/readiness | manifest, preflight, heartbeat ring | startup timeout / scope refusal | logs, peer trace2048rows (~60s), HOME snapshot | JSON/text128MiB | preflight + trial |
| selection/planning | original request/response, candidates, map/scene identity | first terminal rejection | boundary CDR, selected/rejected route, scene/joints and replay manifest for failed case | CDR/JSON/PLY1GiB per case | replay/manifest.json |
| execution/release/HOME | stage/certificate/commit records | failure/timeout | exact trial/checkpoints, release raw90s/8MiB percase, joint plugin existing600s cap | JSON/raw1GiB percase | trial.json |
| final result | suite, ledger, final layout | any terminal event | full failed case, final map/layout + inventory + teardown | total16GiB | REPORT.md |

Keep existing bounded captures; their caps do not establish continuous whole-run physical collision coverage. Missing required artifacts => capture_complete=false explicitly. Monitor total retained16GiB, stop as resource-censored if exceeded, preserve all failure data. No deletes/overwrites, raw data copied primary and hashes indexed. Expected overhead same as n3 harness; timing diagnostic only.
