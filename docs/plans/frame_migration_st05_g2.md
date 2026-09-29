# ST05 generation 2 -- placement candidates and SlotSpec producer

Parent FRAME-MIGRATION-20260929; owner eng/claude/glm-5.3; prerequisites
ST03=2 (6d98a2d) and ST04=2 (8b0cd863709f10207eb790c0454ec3266a6ab3fd),
both passed. This branch is built on the ST04 head. The user reassigned
ST05 from the queued codex generation 1 to this owner. Supersede queued
generation 1 after this plan is committed. Module-task rules from
`docs/plans/frame_migration_stages.md` ST05 apply. No simulation, hardware
motion, master merge or shared-worktree edits.

## Scope and binding decisions

- **Candidate semantics (F07)**: `generate_candidates` emits
  `center_container` — the slot box center in container_link with
  `z = floor_container + support + h/2` (box top = center + h/2) — plus the
  explicit box-axis `box_yaw` and box-axes `size`/`footprint`. The
  historical volume-centered `center_local`, container-rotated
  `center_base` and the ambiguous `yaw = map_yaw + box_yaw` label stop
  being computed inline; they live only in the new explicit legacy
  decoder. `generate_candidates` accepts the v1 surface dict or the v2
  surface record (`kind == "surface"`); v2 input is normalized locally
  (floor-relative view derived from `floor_z_container`), so support
  quantities stay derived at the consumer, never re-embedded.
- **Legacy decoder**: new `luggage_packing.placement_legacy_v1` with one
  function `v1_candidate_view(candidate, surface_view)` reproducing
  today's exact v1 encoding (volume-relative `center_local`, rotated
  `center_base`, `yaw = map_yaw + box_yaw`, base_link quaternion via the
  planner's existing base transform). `placement_planner_node._
  slot_from_candidate` and `_placed_aabbs` route v1 shapes through it; a
  golden test recorded from pre-ST05 behavior pins v1 `SlotSpec` bytes
  unchanged. No global half-height edits anywhere.
- **Producer dual emission**: new `ComputePlacementV2.srv` — request
  `float64[3] box_wdh`, `PoseContractV2[] placed` (slot profile,
  container_link box centers), `string geometry_hash`,
  `uint64 expected_map_revision`, `uint32 max_candidates`; response
  `PoseContractV2 slot`, `bool success`, `string message`,
  `string reason_code`, `string geometry_hash`,
  `PoseContractV2[] candidates`, `float64[] candidate_scores`,
  `string[] candidate_reasons`. Served beside v1 `_handle`; placed-input
  normalization and replan use the same container-center meaning (v2
  placed[] directly; v1 placed via the legacy decoder).
- **Planner surface ingest**: `_on_surface` accepts the v2 surface record
  (geometry-hash fail-closed preserved); `_floor_prior_surface` emits the
  v2 datum view. The v1 JSON topic keeps working through the same
  normalization.
- **F08 consumer seam**: `waypoint_generator` gains a ROS-free
  `slot_from_contract(record, ...)` converting a slot-profile record to
  the world place target through the declared container transform
  (complete orientation, not xyz-only). The BuildMotionSequenceV2 /
  SelectPlacePathV2 service endpoints and their grasp-aware target
  semantics are ST07 scope (rows F11-F13 pin them); F08 stays pending
  until ST07 — declared, not omitted.
- **Neutral atlas fixture (deferred item, now declared)**: ST05 tests use
  the existing atlas-free path (`candidate_validator=None` is the declared
  neutral fixture); runtime certification through the atlas hook is
  unchanged and a test asserts the hook still gates (no silent disable).
- `placement_reachability.solver_contact_point` and
  `placement_solver.candidate_aabb` migrate to container-origin z
  (`center_container`-based), removing the duplicated volume-center
  inversion.

## Decision-complete acceptance

Fresh worktree `/tmp/elfin-frame-migration-st05`, branch
`integration/frame-migration-st05-20260929`, CPU-only
Ubuntu/Python 3.10/Humble, `env -i HOME=/home/adamliao PATH=/usr/bin:/bin
LANG=C.UTF-8`, source only `/opt/ros/humble/setup.bash`. Commit sources
before verification; record revision/dirty state.

```bash
colcon --log-base log/verified build --build-base build/verified --install-base install/verified --symlink-install --packages-up-to luggage_description luggage_msgs luggage_perception luggage_packing luggage_planning luggage_gazebo
source install/verified/local_setup.bash
python3 -m unittest discover -s src/luggage_packing/test -p test_frame_migration_placement.py -v
python3 -m unittest discover -s src/luggage_perception/test -p test_frame_migration_map.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_pick.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_messages.py -v
python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py -v
python3 scripts/check_frame_contracts.py --mode baseline --output EVIDENCE/st05/baseline
python3 scripts/check_frame_contracts.py --mode migration --output EVIDENCE/st05/migration
```

Matrix: yaw 0, pi/2, pi, atan2(4,3) over the box yaw; container
translation (0,0,0)/(1.5,-0.4,0.2); floor .53/.71; support 0/.30; grasp
offsets inapplicable to slot geometry (named); WDH .80/.50/.32 — the
common 4·2·2·2 = 32 slot cases double over v1-dict and v2-record surface
inputs = 64 cases. Per case: candidates from both surface forms agree
(identical geometry, ≤1e-9); `center_container.z == floor + support + h/2`
exactly; box top = center + h/2; footprint and support ratio unchanged
under the coordinate re-expression; four container yaws (applied through
the container transform) produce identical packing geometry (same cell
occupancy pattern after re-expression); the legacy decoder reproduces the
golden v1 encoding; `slot_from_contract` returns the complete world pose
(orientation included) agreeing with independent transform math; placed
input normalization and replan return the same center/yaw meaning
(v2 placed[] direct, v1 via decoder, identical footprints).

Reason categories: existing documented reasons (stack/unknown support,
clearance, support, capacity) fire identically; a coordinate-expression
error is not BIN_FULL — verified by injecting a mislabelled surface datum
and asserting the geometric reason, not a capacity reason.

Negatives each once with exact codes through the v2 slot records: wrong
frame FRAME_CONTRACT_MISMATCH; missing identity MISSING_IDENTITY; wrong
reference POSE_REFERENCE_MISMATCH; stale scene CALIBRATION_REVISION_
MISMATCH; stale map MAP_REVISION_MISMATCH; wrong hull
GEOMETRY_IDENTITY_MISMATCH; version/units/stamp/nonfinite/nonunit
contract codes where the payload carries them. Inapplicable: contact/
detection profile codes (closed at ST03), surface grid codes (closed at
ST04 — the planner rejects foreign surfaces with its existing
CARGO_MAP_GEOMETRY_MISMATCH reason, which must keep firing).

Existing suites: `test_placement_reason_codes.py`,
`test_compute_placement_fixture.py`, `test_placement_atlas_hook.py`,
`test_placement_scoring.py`, `test_ems.py`, `test_insertion_corridor.py`
must pass; `test_placement_solver.py` floor/stacking/AABB z expectations
and `test_mapper_surface_contract.py` planner floor-prior pin are updated
in-chain to the container-origin meaning (named updates, assertions
strengthened not deleted); ST03/ST04 suites and the ST02 message suite
unchanged; contracts 15/15; baseline 50/50.

Migration-gate expectation: baseline exit 0; migration exit 2 with
**14 pending** in this worktree — F07 flips to verified with an on-branch
runtime proof; F08 and later rows stay pending (F08's last consumer is
ST07). The primary workspace keeps reporting F07 pending until merge.

## Evidence and capture

Primary `docs/status/evidence/frame_migration/ST05-20260929/` in the
stage worktree (mirrored to the primary workspace): plan, revision/dirty
state, build log/exit, unittest logs, per-case JSONL via
FRAME_MIGRATION_CASE_LOG, baseline/migration bundles, summary.md,
manifest.json with artifact hashes. Failure records keep complete
input/expected/actual/traceback; no overwrite; 16 MiB bound per run.
