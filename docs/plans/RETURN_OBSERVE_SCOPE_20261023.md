# Return-observe admission scope — 2026-10-23

- owner: eng/codex/gpt-6
- base_revision: a9175a71

## Contract

New placement requests and next-box entry retain full-graph readiness. Certified unloaded named-pose motion uses its actual peer set (selector, motion, vacuum, scene, cargo map and driver), independently of placement and waypoint generation. Full process identity remains immutable. Each scope observes from startup, retains lease/restart revocation and token generation; unrelated revocation cannot leak into motion scope. A claimed unloaded scope still requires a complete certificate, matching leg and live unloaded vacuum; carrying cannot enter it.

Return planning takes the post-commit map snapshot once, records its identity, and uses it for building/certification; map changes and scene changes still revalidate/refuse before execution. Map mutation is explicit service-driven already; returning performs no integrate/placement request. Do not suppress external physical change events or freeze away collision obstacles. Next placement fetches current map normally.

Persist trial/release/verify and replay inputs before HOME and on HOME exception, so the previous missing-evidence failure cannot repeat.

## Acceptance and procedure

Behavioral fake-clock tests: placement missing/expired/restarted permits scoped unloaded request but denies full graph; expiry/restart of each required peer denies motion, changed identity and endpoint types deny, scope/token mismatch denies. Producer tests: unloaded live vacuum required at execution; wrong load state, scene/map mutation, absent certificate refuse. Existing readiness, typed motion, named-route and release gates regressions pass.

Build luggage_description, luggage_planning and luggage_gazebo against ROS Humble overlays; verify loaded source hashes. Rehearse startup with PHASE2_STARTUP_ONLY=1 scripts/g3/n3_functional_run.sh, freely repair deterministic readiness/wiring before scoring. Then freeze product commit and execute PHASE2_RUN_OUT=<evidence>/live_01 bash scripts/g3/n3_functional_run.sh: A-prime, seed0, inch20, n=3, 180s select, direct portal first, same envelope and XY release<=5mm/Z observation. Gate: all 3 attempted boxes released, verified, returned and committed; AND over3 functional units, no statistical per-unit reliability claim. Pre-first-box startup failure is not scored. Later measured failures are preserved and fixed only between immutable attempts; each attempt/revision reported, no success-rate claim from this campaign.

## Dump matrix

| Boundary | T1 | Trigger | T2 and window | Budget/format | Inspection |
|---|---|---|---|---|---|
| readiness | scope, peer receive times, revocation | denied action | runtime snapshot at HOME and exception, identity reports in existing logs | JSON/text 16MiB | trial.json and launch log |
| planning/motion | request, scene/map/certificate, stage times | refusal/exception | original boundary CDR, scene, joint ring and replay manifest, failed leg window | 512MiB | replay/manifest.json |
| release/verify/HOME | exact sampled poses/times, verdict | failure or exception | trial persisted before HOME and after exception, bounded existing release capture90s/8MiB | JSON/raw64MiB | trial.json |
| suite/startup | config, seed, revisions, counts, teardown | any stop | preflight, all case manifests, logs and hashes | 1GiB/run, 4GiB total | REPORT.md/artifact_manifest.json |

Preserve raw without overwrite, copy to primary evidence before discarding. Existing raw caps and plugin600s deadline cannot prove full-suite coverage; exact boundary snapshots are mandatory. Missing artifacts => capture_complete=false, never infer absence of collisions from empty contact capture. One Gazebo stack only, owned teardown immediately after runs, residual audit. Scoring accepts no product dirty state except pre-existing untracked pydeps-frozen symlink; evidence/config outputs recorded separately. Functional capture timing only. If retained evidence reaches4GiB pause scoring, retain diagnostics and report size; do not delete failure evidence.
