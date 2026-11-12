# 2026-11-12 -- Observed full-model Gazebo operation integration

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- base_revision: 5086d84f2873db9b36ec94d8e67efeb83481c74f
- started_at: 2026-11-12T17:35:41+08:00
- completed_at: n/a

## Summary

Integrate an explicit observed O/full-model simulation pipeline. Full-cycle
baseline remains open until an actual observed controller cycle passes.

## Requirement

Follow docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md at983e80e1.
No guessed OCC, no online simulator truth, no unilateral collision margin change.

## Changed

Explicit sim_world pipeline selection; new grouped profile; one O owner and
observed optical depth adapter; same full native model for all operation ports;
explicit start diagnostic driver; installed source checks.

## Verification

- Selection/depth/client/readiness:38 cases pass; DDS Best Effort delivery regression passes.
- scripts/production/build.sh --evidence docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/build_01:13 packages pass,6.48s,zero concurrent source changes.
- Rehearsals04/05/06: full-model sensor/controller startup ready;50 installed checks pass; owned teardown has zero residuals.
- cycles01..03: measured observation motions execute, detection/domain joined; no product-box carry yet. Failures and CDR retained.
- Full-model atlas:12,098 states native/FK requalified; distribution unchanged; hardware asset untouched.
- Map-version policy:133 offline and10 actual O/native service tests pass. Content change recertifies before issuance, duplicate observation preserves original time. Release RPC completion is awaited inside original deadline.
- Known next blocker: measured O only0.919% known active volume,0 placement candidates. Old portal view central ray points outside ULD by full-model FK;14 geometry-derived IK states yield2 valid branches for one camera view in a complete static scene; live optical coverage remains unverified.

- cycle04: first observation goal refused, elfin_link6 versus ULD mesh; no physical motion/impact demonstrated. Early ready scene was empty: previous14-state empty-world validity cannot certify environmental collision. Full-scene replay rejects12/14 states.
- Added static geometry readiness and before/after-goal full-scene capture;24 cases pass, build14 and50 install checks pass. rehearsal07 queued behind a foreign Gazebo stack; no second world launched.

## Result

Open. Composition is implementation, not a baseline success witness.

## Pointers

- docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/

## Collision follow-up -- 2026-11-12 19:23

Read-only exact-q native replay localizes link6 contact at the ULD upper front rim,
not O voxels:8000-triangle mesh, zero link6 padding, max reported triangle contact
depth10.536mm. Wrist origin container z2.116660m is near mesh top2.117807m; camera
origin is lower at1.864800m. Live named pair is recorded; offline contact coordinates
must not be presented as captured live contacts. Uniform2cm shell thickness and
real-model dimensional error are not established. Evidence: collision_review_01/
summary.json and analyze.py. No product changes; full-cycle baseline remains open.
