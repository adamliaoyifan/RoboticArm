# 2026-11-12 -- Ordered aperture routing in whole-operation planning

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-APERTURE-ROUTING
- base_revision: 6fd7338b302c8bde2135765d6e20d2e41fe88a58
- started_at: 2026-11-12T19:47:35+08:00
- completed_at: n/a

## Summary

The whole-operation planning root generates grasp-bound exterior/plane/interior
opening poses and connects them inside the existing carry leg. Callers do not
provide intermediate points. Native crossing constraints and full-scene checks
are retained; this is not a full-model Gazebo execution baseline.

## Requirement

Implement plan `79a96b26`. ComputePlacement continues to supply geometric slots.
Opening geometry comes from the admitted scene; attached geometry comes from the
frozen grasp relationship. Bound candidate/seed work explicitly, preserve directed
headings, exact J6 winding, six public legs, and two attachment/release barriers.
No fabricated occupancy, guessed world waypoint, or new physical margin.

## Changed

- `aperture_geometry.py`: opening-local payload erosion and bounded proposals;
  eccentric/tilted grasp geometry is transformed without flattening.
- `aperture_tool_geometry.py`: fixed tool collision bounds from the same URDF.
- `joint_operation_aperture.py`: qualified atlas/current seed proposals, exact
  native IK, ordered connections, trace, independent plane FK witness.
- `joint_operation_scene_admission.py`: require configured static mesh/pose and
  enabled pedestal before search and on subsequent scene reads.
- Existing preparation/planner/certifier/connector consume these modules.
  `PlanSnapshotJointPathV2` carries standard MoveIt path constraints; native
  endpoint, OMPL, and sampled output checks enforce them. All consumers rebuilt.
- Nearby architecture, frame consumer contract, install checks, launch contract,
  behavioral tests and explicit synthetic fixture geometry updated together.

## Verification

- `scripts/production/build.sh --evidence docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-APERTURE-ROUTING/build_04`: 13 packages pass, 6.877 s, zero concurrent input changes.
- `scripts/production/run.sh /usr/bin/python3 scripts/production/check_install.py --output docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-APERTURE-ROUTING/offline_01/installed_check_04.json`: 54 checks pass.
- Installed pytest commands, explicit native opt-in environment and fresh immutable
  fixture directories: `REPLAY.md` under the evidence root. Focused geometry/root/
  planner/connection: 65 pass; native connector: 10; aperture including blocked
  opening and whole six-leg certification: 3; endpoints: 7; certification: 10;
  G preparation: 16; actual O observer/preparation: 10. Total 121 passing cases.
- `scripts/production/run.sh /usr/bin/python3 docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-APERTURE-ROUTING/offline_01/full_model_geometry_probe.py`: full S20 static scene identity and 72 fixed-tool proposal vertices pass. This is offline geometry replay only.
- Harness-only failures/skips and corrected runs remain separately recorded in
  `summary.json`; none are counted as product successes or reliability estimates.
  Native request/response CDR, scene/model/config hashes and node logs retained.

## Result

Verification passes; implementation commit pending. Complete observed full S20
carry/place/exit execution, observation coverage, and production timing remain
under the open integration task. Existing local release/retreat policy is unchanged;
this subtask does not add or certify a complete retreat to outside the opening.

## Pointers

- `docs/plans/JOINT_OPERATION_APERTURE_ROUTING_20261112.md`
- `docs/agents/discuss/2026-11-12_1947_joint-operation-aperture-routing.md`
- `docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-APERTURE-ROUTING/summary.json`
