# 2026-11-12 -- Read-only cargo bag data audit

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-OBS-DATA-AUDIT
- base_revision: c1ef631125cac8e698eff3c37c618a33bd9f37fb
- started_at: 2026-11-12T11:28:28+08:00

## Summary

Implemented bounded offline source acquisition audit under plan ccb50b36.
Inventoried10 bags; deeply inspected the September16 pickup MCAP and retained
3 source examples. No ROS graph, runtime geometry change, matching threshold,
O replacement, robot/controller or sensor playback. This is data qualification
evidence, not completion of conditional physical matching or full integration.

## Requirement

User provided raw point cloud/RGB-D pickup data without an existing error
report. User further confirms no independent metrology or release/multi-box
recordings. Preserve raw sources; report actual source coverage; do not infer
absolute pose/shape error or physical false-association rate from self-agreement.
Finite synthetic gate is100% required case verdicts, no stochastic reliability
or latency/capacity improvement claim. Historical bag selection/count and3
examples are explicit audit workload parameters, never production constants.

## Changed

ROS-free acquisition header parser/index; native MCAP schema reader, exact
stamp/frame/source-hash joins, full topic count parity, TF/joint/event summaries,
recorded detection export, bounded replay source snapshots and image/CDR capture
health. Dedicated offline CLI records revision/dirty/source hashes and manifest.
Closest architecture clarifies proposal-mask and calibration semantics.

## Verification

- PYTHONPATH=src/luggage_perception PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/luggage_perception/test/eval/test_cargo_bag_audit.py --basetemp=<check_02>/pytest_tmp --junitxml=<check_02>/junit.xml:15 cases pass,0.10s.
- PYTHONPATH=src/luggage_perception python3 src/luggage_perception/scripts/audit_cargo_bags.py --root /home/adamliao/work/robotarm_bags --output <run_02>:10/10 metadata inventories, zero selected topic count mismatches,225 exact four-source joins/237 mask records,3 retained source boundaries;~10MiB output.3.89s instrumented scan is diagnostic only.
- run_01 marked incomplete after native slotted DetectionFrame exported empty
  dictionaries. Preserved exact native CDR/schema plus original source snapshot;
  added slot-aware behavioral regression; retained-case replay restores24fields.
  No raw bag overwritten or edited. run_02 exports all224 complete outputs.
-168 measured/56 held outputs;142 valid measured-height outputs. Two nonempty
  sampled masks fill their entire detector rectangles. Three sampled calibration
  records have nonzero plumb_bob D and cannot pass the current rectified-only
  observation service.225 timestamp joins are not225 qualified geometric frames.
- Absolute pose/size error, verified-static repeatability, current-deployment
  transfer, multi-box and post-release matching rates all not_evaluated. No
  threshold derived from these records; full integration remains open.

## Result

Implementation and bounded audit pass; awaiting lifecycle closure at exact
commit. Next review is real camera-model projection and geometric qualification,
then a separately frozen repeatability/uncertainty workload and native O
provenance. Missing calibration evidence must not become a guessed default.

## Pointers

- docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md
- docs/agents/discuss/2026-11-12_1128_cargo-bag-data-audit.md
- docs/agents/reviews/2026-11-12_1120_released-cargo-matching-checkpoint.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-DATA-AUDIT/run_02/manifest.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-DATA-AUDIT/run_02/coverage_assessment.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-DATA-AUDIT/run_01/detection_decoder_failure/
