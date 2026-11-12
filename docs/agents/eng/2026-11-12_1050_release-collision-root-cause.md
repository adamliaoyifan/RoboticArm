# 2026-11-12 -- Released-cargo collision root cause and diagnostic capture

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open

## Summary

Located the preserved synthetic native_cycle_02 refusal. The original sixth leg was never submitted: its sample0 cup sphere intersects four O collision voxel objects after release. This is a collision-model refusal, not an observed physical collision. Whole-cycle integration remains open.

## Findings

Direct observations: TCP z=0.7500000014m; released measured fixture box top approximately0.7000000015m; explicit conservative released-box primitive top approximately0.700m. The fixture O grid is0.1m. Float32 point z=0.7000000477m enters the next voxel, whose top is0.8m. Existing0.05m per-face collision padding raises that top to0.85m. Unknown cell count is0. Four contacts are placement-prior-occ-voxel-0..3 against suction_contact_frame. The synthetic robot has collision geometry only on that cup link, so this does not certify real arm/base clearance. Production O default resolution is0.01m; this fixture does not establish production failure rate.

Controlled native replay: original494 samples/times are held fixed. Original scene refuses; removing only O accepts all494; removing only explicit released-box geometry still refuses; retaining O without its padding still refuses. Each deterministic diagnostic variant once, functional AND, all expected verdicts met. These diagnostic ablations are never production admission policies. Do not snap measurements to voxel faces, erase a planned AABB, reduce margins or enlarge the release gap to force success.

Source-established mechanism: prepare certified fine released-box geometry while O was empty; post-release live O represents the same cargo with coarse voxels. Native phase collision representations differ. Duplicate ownership alone is not the cause: removing the fine object does not fix the refusal. Unknown space is not the cause in this case. A different route cannot fix an already invalid original starting configuration.

## Changed

Committed7b697e00: added route_collision_diagnostics.py and capture at a failed native suffix boundary. Bounded summary binds phase index, first invalid sample/TCP, pair names and actual matched world/attached geometry; full request/response CDR remains authoritative. No scene, trajectory or verdict changes.

## Verification

- source /tmp/joint_operation_o_env.sh; JOINT_CERTIFICATION_EVIDENCE_DIR=<native_diag_01/native> python3 -m pytest -q src/luggage_planning/test/test_joint_operation_revalidation_native.py --junitxml=<native_diag_01/junit.xml>:42 passed in0.80s, isolated native service, no motion/hardware/Gazebo.
- python3 docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_collision_root_04/replay.py:4 expected native verdicts met; raw CDR and revision/model/binary/config manifest preserved. Three preceding harness failures remain preserved in root_01..03 and are not product scores.

## Decisions and remaining work

User chose retain0.05m release gap and add observed current-cargo association/fine collision representation. No existing post-release instance-mask/cloud interface is available: develop the measurement association module this round. O currently carries no per-box provenance; task instance_id does not prove visual identity. Until a new source-bound policy is admitted, live O obstacles remain unchanged. User additionally accepts conditional unique matching; ambiguity retains O. No positive complete-operation, deployment witness or performance gain is claimed.

## Pointers

- docs/agents/discuss/2026-11-11_2057_joint-operation-full-integration-g2.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_collision_root_04/summary.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_collision_root_04/optical_input_summary.json
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/release_collision_root_04/collision_side_view.png
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-INTEGRATION/g2/native_diag_01/
