# ST04 generation 2 -- cargo map and measured commit frame boundaries

Parent FRAME-MIGRATION-20260929; owner eng/claude/glm-5.3; prerequisite ST03=2
passed at 6d98a2d2a003b181ad4967a0bec3f7c1c83f1dcc. This is the exact base.
The user reassigned ST04 from the queued codex generation 1 to this owner.
Supersede queued generation 1 after this plan is committed. Module-task rules
from `docs/plans/frame_migration_stages.md` ST04 apply. No simulation,
hardware motion, master merge or shared-worktree edits.

## Scope and binding decisions

- **F06 surface**: `CargoVolumeMapper.surface_contract_v2(scene_revision,
  request_id)` returns the strict v2 surface record
  (`height_datum=container_origin`, `floor_z_container = center[2] −
  inner_h/2`, `ceiling_z_container = center[2] + inner_h/2`,
  `origin_xy_container`, `resolution_m`, rectangular `surface_z_container`
  with **None at unknown cells**, `state`, `confidence` with
  `unknown/sensor/verified_geometry` (v1 `geometry` → `verified_geometry`,
  v1 `none`+unknown → `unknown`), optional `height_above_floor` derived and
  agreeing ≤1e-9). The node publishes it as `luggage_msgs/SurfaceContractV2`
  via `measured_surface_to_message` on `/luggage/cargo_map/surface_v2`
  beside the byte-stable v1 `/luggage/cargo_map/surface_2d` JSON;
  `surface_map_2d()` itself is unchanged.
- **F19 ledger**: new `AddPlacedBoxV2.srv` / `RemovePlacedBoxV2.srv`
  (`PoseContractV2 slot_pose` in the slot profile — container_link box
  center, box_axis_pi, scene basis — plus `float64[3] size_wdh`,
  `string measured_source`, `uint64 expected_map_revision`,
  `string geometry_hash`; response: success/message/resulting revision).
  Served by `cargo_volume_mapper_node` next to v1. One ROS-free
  `normalize_ledger_slot(...)` in `cargo_volume_mapper.py` is the single
  normalization for add/remove/readback: the v2 path validates the slot
  record and uses container coordinates directly (no second yaw
  subtraction); the one declared v1 shim reproduces today's exact
  world-pose SlotSpec conversion (`_to_container` + container-yaw
  subtraction) for the existing orchestrator. Both v1 callbacks and both
  V2 services route through it. `VerifyPlacedBoxV2` is deferred: v1
  `VerifyPlacedBox` is served by the ROS1 placed-pose verifier (legacy
  scope); its readback commit path is the AddPlacedBox normalization,
  which is shared by construction.
- **F06 reader**: `OccupancySnapshot.from_surface_2d` accepts (a) a v2
  surface record — native input, converted internally to the snapshot's
  floor-relative representation (z − floor_z_container; unknown/None kept
  as state) — or (b) the **single declared v1 import adapter**: a legacy
  dict identified by `frame == "container_local"` with the complete legacy
  datum field set (`height`, `state`, `inner_size`, `center_base`,
  `floor_z`, `resolution`, `nx`, `ny`, `geometry_descriptor`). A dict with
  `height` but missing datum fields, a foreign `frame` label, or neither
  schema raises `OccupancyMismatch` with the documented
  PLACE_PATH_INFEASIBLE reason — no default datum, no magnitude guessing.
  The snapshot's internal representation and all downstream consumers are
  unchanged in this stage.
- **F20 integrate**: `IntegrateCargoView` v1 service and its schema/hash/
  revision/stamp checks are preserved byte-stable; the v2 datum binding is
  the scene-bound surface record (envelope `scene_revision` +
  `geometry_hash`), and integration still never promotes unknown to free
  (`fill_unoccupied_as_free` stays GT-only and uncalled by the node).
- **Raster rule (deferred item, now declared)**: cells are half-open
  `[origin + i·r, origin + (i+1)·r)`. A committed box's occupancy is
  defined as the cells containing at least one of its interior sample
  points (`(k+0.5)/n` per axis over the extent, `n = ceil(extent/r)+1`) —
  the mapper's existing deterministic rule, here computed independently as
  an exact oracle (point-in-oriented-rectangle per sample). Against the
  exact oriented-footprint polygon intersection: every cell whose
  intersection area is ≥ half the cell area must be marked; any extra
  marked cell must have its interior intersected by the footprint boundary
  (conservative skirt). Per-cell comparison, zero unexplained mismatch;
  never only array sums.

## Decision-complete acceptance

Fresh worktree `/tmp/elfin-frame-migration-st04`, branch
`integration/frame-migration-st04-20260929`, CPU-only
Ubuntu/Python 3.10/Humble, `env -i HOME=/home/adamliao PATH=/usr/bin:/bin
LANG=C.UTF-8`, source only `/opt/ros/humble/setup.bash`. Commit sources
before verification; record revision/dirty state. Repairs require separate
source commits and preserved failed evidence.

```bash
colcon --log-base log/verified build --build-base build/verified --install-base install/verified --symlink-install --packages-up-to luggage_description luggage_msgs luggage_perception luggage_packing luggage_planning luggage_gazebo
source install/verified/local_setup.bash
python3 -m unittest discover -s src/luggage_perception/test -p test_frame_migration_map.py -v
python3 -m unittest discover -s src/luggage_perception/test -p test_frame_migration_perception.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_pick.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_messages.py -v
python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py -v
python3 scripts/check_frame_contracts.py --mode baseline --output EVIDENCE/st04/baseline
python3 scripts/check_frame_contracts.py --mode migration --output EVIDENCE/st04/migration
```

Matrix: yaw 0, pi/2, pi, atan2(4,3); translation (0,0,0)/(1.5,-0.4,0.2);
floor .53/.71; support 0/.30; grasp offsets not applicable to the map
boundary (named inapplicable); WDH .80/.50/.32 — 4·2·2·2 = 64 surface
cases. Each case: build a mapper at the case geometry, commit one
non-square placed box through the v2 ledger service record, emit
`surface_contract_v2`, serialize via real `SurfaceContractV2`, decode, and
check per-cell: occupied/free/unknown survive add→serialize→read; v2
surface equals v1 height + floor (dual datum agreement ≤1e-9 on every
non-null cell; unknown cells are None in v2 and exactly the v1 unknown
set); raster occupancy equals the independent sample oracle exactly and
satisfies the declared polygon-intersection bounds (per-cell mismatch
count 0).

Ledger semantics: duplicate v2 commits are idempotent (no revision bump);
`expected_map_revision` mismatch rejects (stale revision, no mutation);
remove restores the sensor-observed cells (an integrated sensor column
under a removed box is not erased — verified by integrating points first,
committing, then removing); readback (v1 shape commit of the verified
actual pose) normalizes identically to the v2 path — same voxels, one
revision semantics. Every rejection happens with zero map mutation (spy
counter on revision/ledger length).

Negatives each once with exact codes: v2 record wrong frame
FRAME_CONTRACT_MISMATCH; missing identity MISSING_IDENTITY; stale scene
CALIBRATION_REVISION_MISMATCH; stale map MAP_REVISION_MISMATCH; wrong hull
GEOMETRY_IDENTITY_MISMATCH; version CONTRACT_VERSION_MISMATCH; units
UNIT_MISMATCH; invalid grid INVALID_GRID; unknown-with-value
UNKNOWN_SURFACE_HAS_VALUE; unverified source UNVERIFIED_SURFACE; datum
bound INVALID_HEIGHT_BOUNDS. Adapter negatives: legacy dict missing
`floor_z`/`center_base` → OccupancyMismatch (declared reason); foreign
`frame` label → OccupancyMismatch; empty height rows → INVALID_GRID at the
v2 boundary. Inapplicable for ST04 and named: contact/detection pose
negatives (F03/F04 closed in ST03), nonunit-quaternion and stamp-basis
cases not carried by the surface/ledger records (rejection surfaces as
INVALID_NUMERIC_PAYLOAD/INVALID_STAMP at the pose payload where present).

Existing suites must pass unchanged: `test_cargo_map_commit_contract.py`,
`test_mapper_surface_contract.py`, `test_cargo_view_integration.py`,
`test_frame_migration_messages.py` (v1 wire), ST03 suites (6/6 each),
`test_frame_contracts.py` 15/15, baseline 50/50. If a legacy
`test_occupancy_place_paths.py` fixture lacks the declared adapter datum
fields, the fixture gains them (datum made explicit) — the assertion set
is otherwise unchanged.

Migration-gate expectation: baseline exit 0; migration exit 2 with
**15 pending** in this worktree — F06, F19, F20 flip to verified with
runtime_boundary proofs validated from this tree; the primary workspace
keeps reporting them pending until merge (recorded).

## Evidence and capture

Primary `docs/status/evidence/frame_migration/ST04-20260929/` in the stage
worktree (mirrored to the primary workspace): plan, revision/dirty state,
build log/exit, unittest logs, per-case JSONL via FRAME_MIGRATION_CASE_LOG,
baseline/migration bundles, summary.md, manifest.json with artifact
hashes. Failure records keep complete input/expected/actual/traceback; no
overwrite; 16 MiB bound per module run.
