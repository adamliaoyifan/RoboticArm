# 2026-11-12 -- Same-cargo matching and O provenance checkpoint

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: open

## Summary

User accepts conditional unique measured association under static-other-cargo premise; ambiguous/occluded/contradictory evidence retains O. Exact observation partition prerequisite is complete at cf3192f6. Physical matching and O replacement are not implemented or dispatched by that commit. This checkpoint records the next boundaries without inventing a geometry tolerance.

## Evidence now available

The read-only acquisition service gives a frozen complete RGB/depth/instance-mask/calibration/acquisition-TF observation, actual Float32 point order, original pixel indices and frame-local labels. It does not give a persistent box ID. Every post-release candidate must come from measured segmentation/geometry. Actual fixed seal G and release/TCP are search constraints, not a substitute observed box pose. The known planned slot cannot be used to carve occupancy.

Existing suction_contact_model.yaml RMS2.5mm/P954mm are local contact plane quality thresholds. dynamic_suction_acceptance.yaml XY20mm/Z10mm/size30mm P95 are evaluation acceptance targets. livox_d555_table_icp_frozen.yaml is a historical table-patch cross-sensor fit. None establishes the current cargo pose/shape error bound or an acceptable false-association rate. Do not copy these numbers into a matching gate or declare measured accuracy from configuration. Asked user for current deployment recorded multiframe data/report; pending availability. No fabricated defaults or uncalibrated automatic replacement.

## Matching design to freeze next

1. Bind frozen measured pickup identity/shape/source to confirmed actual attachment and release, plus exact before/after observations. Query the full candidate set; do not choose the nearest instance unconditionally.
2. Fit each candidate from actual visible points and image evidence, with the measured pickup shape as a retained observation. Enumerate compatible symmetries; orientation ambiguity within one symmetric box and multiple-object ambiguity are different. Retain competing candidates and residual/support/visibility evidence.
3. Require one supported same-cargo explanation under the user's conditional static premise. Unseen surfaces are bounded by the already measured shape and a qualified pose/uncertainty model, never by the visible points alone. Missing physical signal does not prove no collision.
4. Derive a matching/uncertainty policy from accessible actual data. Multiframe repeatability and cross-view closure can measure relative consistency; they do not measure all systematic calibration bias. Explicitly report unavailable coverage instead of treating it as zero. Freeze workload, false-match/missed-match metrics, thresholds, per-unit reliability, sample-count/power and holdout procedure before dispatch. No scored campaign yet.

## O provenance design to freeze next

O must identify the actual contributing endpoints at its native filter/transform/OctoMap key boundary. A Python approximation of quantization or a latest mask cannot prove which hit caused occupancy. Need frame receipt identity including config/epoch/hull/clock/observed revision and exact source cloud/partition hashes. All writers, including unlabelled sensors and unchanged-occupancy integrations, count toward interval completeness; map_revision alone misses integrations with no probability change. The actual owner increments observed_revision for every accepted frame, established by observer.cpp.

A bounded owner ledger can retain source-frame plus native hit-cell/local-label contributors, with empty-hit frames retained as continuity markers. Raw segmentation labels remain local. Any history gap, eviction, reset/restore, unclassified/mixed contributor, source mismatch or unresolved association preserves the voxel. Do not label a whole voxel from a majority of points. Baseline free/occupied/unknown semantics remain explicit. Avoid retaining every raw point forever; measure enabled memory/latency and maintain zero evidence authority when disabled. This is design, not an implemented/readily dispatchable owner API.

Only source-complete exclusively-own occupied geometry can eventually be replaced in a read-only private planning scene by conservative measured cargo geometry with the existing padding. Other obstacles and unknowns remain. Current-source checks and full native original-suffix validation still precede any controller goal.5cm release gap and original trajectories remain unchanged.

## Review status

No request to change the user's goal or to approve hardware motion. Current requested information is actual-data availability, not permission to discard obstacles. Continue measured evidence/owner-interface work; physical matching thresholds remain unresolved until their empirical basis is reviewed. Keep ST-4-INTEGRATION generation2 open and unchanged; later generation3 requires full preserved acceptance and deploy witness.

## Pointers

- docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md
- docs/agents/eng/2026-11-12_1115_cargo-observation-partition.md
- src/luggage_description/config/suction_contact_model.yaml
- src/luggage_perception/config/dynamic_suction_acceptance.yaml
- src/luggage_perception/config/livox_d555_table_icp_frozen.yaml
- src/uld_occ_observer/src/observer.cpp
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-JOIN/run_03/manifest.json
