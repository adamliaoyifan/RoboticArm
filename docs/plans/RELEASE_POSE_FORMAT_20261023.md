# Release pose format repair and one frozen closed-loop suite

Owner eng/codex/gpt-6; base4061af22. User authorizes format repair followed by one complete closed-loop simulation. Scope: a single schema-aware text decoder shared by online release cache and offline replay; omitted scalar zeros, field ordering, valid numeric syntax, header reset, named rejection diagnostics; retain freshness/identity and existing geometry/planning/physics gates. No envelope or route policy changes.

Offline acceptance: real captured Pose_V bytes decode published preloaded suitcase into xyz/quaternion; canonical explicit-zero tests retained; reordered fields/scientific notation accepted; zero timestamp components reset; missing stamp, malformed/truncated message, nonfinite coordinate and invalid quaternion rejected by name; stream chunks do not affect decoded result; live and offline yield identical samples. Run focused parser, release stream, kinematic release and capture behavioral tests. Build luggage_gazebo; installed/source identity required.

Rehearse startup without scoring, bounded20minutes; preserve failed startup logs and repair deterministic launch wiring only with measurements before scoring. Freeze implementation commit and run exactly one existing n=3 suite, seed0, A-prime,180s Select budget, direct-first, prioroff, unchanged geometry/gain. Command PHASE2_RUN_OUT=<evidence>/live_01 ELFIN_RELEASE_CAPTURE_DIR=<evidence>/live_01/release_capture bash scripts/g3/n3_functional_run.sh. Suite stops at first product failure; no automatic second scored attempt or mid-run change. Pass=3 attempted/picked/packed and verified occupancy commits, no failure. This is an AND over three box units; no calibrated reliability claim from n=1 suite. Startup failure before first driver box consumes no scored unit; report separately.

## Dump matrix

| Boundary | T1 | Trigger | T2 payload | Window | Budget/format | Inspection |
|---|---|---|---|---|---|---|
| startup | launch,preflight,identity | startup refusal | configs,URDF,logs | launch→teardown | 64MiB text/JSON | startup_preflight.json,launch.log |
| selection/certification | driver stage/route timings | select or motion refusal | request/response/scene CDR,route/collision dump | failed case boundaries | 512MiB | driver_out/dumps,selection_boundary |
| release | presence evidence,decoder counters | missing/invalid/stale/pose mismatch | poses.raw,contacts.raw,chunks/events journals | vacuum-off through settle,90s max | 8MiB per stream | shared decoder replay plus manifest coverage |
| whole suite | trial,suite,capture health | any failure | joint window and failure bundle | existing bounded plugin capture | 1GiB retained per run;2GiB total | manifest hashes and suite.json |

Raw input immutable; derived decoder output separate. Capture ready only means readers started; verify actual raw bytes, completeness and event coverage. Timing is diagnostic with raw capture overhead, not clean benchmark. Preserve files and sha256 manifest in primary and satellite; no cleanup until durable. Teardown each owned stack and audit residual processes before next startup. Claim success only at frozen clean product revision (pre-existing pydeps-frozen symlink excluded); report missing artifacts/not_evaluated explicitly.
