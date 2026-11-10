# Release presence XY acceptance and one n=3 suite

User directive: height discrepancy, including over50mm, is not a release-position failure; XY displacement is. Parent PHASE2-20261022, subtask RELEASE-FORMAT generation2, eng/codex/gpt-6; supersedes generation1 scope. Base dacc207d. Retain set_pose, identity/freshness/finite-pose checks; change only kinematic release position acceptance from3D distance to planar Euclidean distance<=0.005m. Z signed delta and3D error remain observations, with explicit acceptance_axes=xy and verdict declared_xy_confirmed. No geometry, envelope, trajectory or other settle/verify gates are changed.

Before another scored gate, fill previous release evidence omissions: save exact sampled pose7, declared unrounded pose, signed XYZ delta and XY/3D norms, set_pose request/response host and simulation times, sampled message stamp and decoder evidence. Preserve bounded raw capture. This is observation instrumentation; no added retry or freshness barrier.

Offline acceptance: XY within5mm with positive/negative Z differences over50mm accepted; XY outside5mm (including diagonal combined error) rejected regardless of Z; existing absent/stale/service failures still refused. Exact input pose and separate errors must be logged. Run kinematic release, release pose stream, pose text, capture, payload attachment and release failure contract tests; build luggage_gazebo, require installed/source byte identity. Behavioral tests pass with no skips due to missing peers.

Rehearse startup unscored (20minute total bound); then freeze product commit. Exactly one n=3 suite, stop-on-first-failure, seed0/inch20/A-prime/180s Select/prioroff, unchanged envelope/gain. Command: PHASE2_RUN_OUT=<evidence>/live_01 ELFIN_RELEASE_CAPTURE_DIR=<evidence>/live_01/release_capture bash scripts/g3/n3_functional_run.sh. Gate:3 attempted/picked/packed, verified cargo-map commits, MAX_BOXES and exit0; AND over3 prescribed box units, no calibrated per-unit reliability claim. Startup failure before first box is not scored. No second scored suite or in-run changes.

## Dump matrix

| Boundary | T1 | Trigger | Payload/window | Format/limit | Inspect |
|---|---|---|---|---|---|
| startup | launch/preflight/identity | startup failure | config,URDF,logs;launch→stop | text/JSON64MiB | startup_preflight.json |
| selection/execution | stage/reason/timing | failure | original request/response/scene CDR,failed-stage replay and joint ring | 512MiB | trial.json,replay/manifest.json |
| release | exact sampled/declared pose,XYZ/XY norms,command times | refusal | command/read boundary in trial;raw vacuum-off onward90s/8MiB per stream | JSON/text; byte truncation explicit | trial.extras.release_kinematic,raw decoder replay |
| suite | counts,commit,teardown | any termination | all failed/success trial boundaries | 1GiB/run,2GiB total | suite.json,artifact_manifest.json |

Keep raw immutable, hash/copy primary before discarding anything. Known recorder600s plugin deadline and pose8MiB limit mean these streams cannot prove whole-suite coverage; require exact release sampled inputs in trial even if raw cap expires. Report dropped counts/interval/missing items, never label incomplete raw as a complete race replay. Capture overhead means functional diagnostic timing only. Single sim, oneclock publisher, readiness gate, teardown and residual audit required. Dirty rule: no product modifications at scoring; pre-existing untracked pydeps-frozen symlink recorded separately.
