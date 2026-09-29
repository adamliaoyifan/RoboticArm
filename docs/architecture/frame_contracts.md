# Frame contracts: staged v2 baseline

This document defines the new **offline boundary contract**, its checker and
migration gate. It does not claim that current ROS messages or live nodes have
adopted v2. Existing live placement height semantics remain those documented in
[placement.md](placement.md) until their producers and consumers migrate
together. Interface-by-interface adoption is tracked in
[frame_interfaces.json](frame_interfaces.json); initially every row is pending.

## Canonical representations

| Quantity | Representation |
|---|---|
| Raw images / sensor points | Truthful optical / sensor frame, original acquisition stamp |
| Detected box center | `world`, measured body-axis dimensions; box-axis orientation |
| Measured suction contact | `world`, surface-contact point; local outward +Z normal |
| Slot / container geometry | `container_link`, box center; full quaternion with geometric π symmetry |
| Cargo surface | `container_link`, z relative to container origin; separate configured floor z |
| Container carry path | `container_link`, suction-contact-frame origin, directed tool orientation |
| Executable Cartesian motion | Current planning frame `world`, explicit suction target link |
| IK target | Model-required frame, currently `elfin_base_link` for the relevant offline model |
| Controller trajectory | Named joint coordinates; no Cartesian frame substitution |

Body-axis width is along box +X and depth along box +Y. Do not infer long/short
axis ownership from a catalog. Box footprint symmetry is π; concrete tool
orientation has 2π periodicity. q and -q represent the same rotation, but yaw
0 and π tool-down poses generally do not. Joint winding and IK seeds are
separate from quaternion sign.

The 2.5D task supports a horizontal container. The transform utility supports
full SE(3), but that does not broaden the gravity/stacking contract. Use
`require_horizontal` at container-geometry admission; reject unsupported tilt.

## Plain boundary records

`luggage_description.frame_contracts` is ROS-free. These are plain validated
records, **not newly generated ROS message layouts**. A future ROS binding must
preserve these semantics and be independently tested across real serialization.

All pose/surface records require:

```text
contract_version: 2
kind: pose | surface
frame_id: explicit frame name
length_unit: m
stamp_ns: integer >= 0
time_basis: acquisition | robot_state | scene
scene_revision: nonempty calibration/scene identity
request_id: nonempty correlation identity
```

Zero is a valid simulation epoch, not a request for latest TF. Acquisition
geometry keeps its measurement stamp. Starting robot state keeps its own state
stamp. Static scene goals bind to the chosen scene snapshot. These times must
not be collapsed into a fabricated common `now()`.

Pose payload: `position[3]`, `quaternion_xyzw[4]`, `reference`, and
`orientation_semantics`. Named checker profiles are:

| Profile | Frame | Reference | Orientation | Time basis |
|---|---|---|---|---|
| detection | world | box_center | box_axis_pi | acquisition |
| contact | world | surface_contact | surface_normal | acquisition |
| slot | container_link | box_center | box_axis_pi | scene |
| map_tcp | container_link | suction_contact_frame | tool_quaternion | scene/state/acquisition |
| motion | world | suction_contact_frame | tool_quaternion | scene/state/acquisition |
| base_tool | elfin_base_link | suction_contact_frame | tool_quaternion | scene/state/acquisition |

`state` in the last column abbreviates the actual value `robot_state`.
Missing/unsupported frame, reference, version, scene identity, unit, time basis,
nonfinite coordinates, malformed vector and non-unit quaternion fail. Do not
coerce numeric strings or silently normalize invalid quaternions. Norm tolerance
is 1e-6. Extra provenance fields may be preserved; they do not override checks.

Surface payload: `geometry_hash`, `map_revision`, `origin_xy_container[2]`,
positive `resolution_m`, `height_datum=container_origin`, `floor_z_container`,
`ceiling_z_container`, and matching rectangular `surface_z_container`, `state`,
`confidence` arrays. The consumer supplies expected geometry and map revision.

- Unknown top: null and confidence `unknown`; not numeric zero or floor z.
- Measured free column: surface equals floor, confidence `sensor` or
  `verified_geometry`. Occupied tops lie within floor/ceiling bounds.
- Unverified planned geometry is not admitted as measured support.
- Optional `height_above_floor` is derived from surface minus floor; if present,
  every element must agree within 1e-9 m, with null at unknown cells.
- Floor-prior policy stays explicit and separate from measured free-space claims.
  This checker does not authorize exploration, pick, placement or motion.

Concrete-tool atlas metadata requires separate `position_frame` and
`orientation_frame` (both container_link), `reference=suction_contact_frame`,
`orientation_semantics=concrete_tool_pose`, `tool_down_reference=container_negative_z`,
metres/radians, container-origin height datum, 2π yaw period, unique yaw bins in
[0,2π), and matching scene/geometry/URDF/payload identities. This verifies metadata,
not that seeds are valid IK solutions. π-aggregated box feasibility is a different
future API, not an alias for a concrete tool-pose atlas.

## Transform and reference-point boundaries

`frame_transforms.transform_pose` accepts an explicit source/target transform:
translation, unit quaternion, scene_revision and stamp_ns. An explicit null
transform stamp means static for that scene revision; a dynamic transform stamp
must match the record exactly. Both position and orientation are transformed.
The operation returns a fresh record and retains acquisition/correlation data.

`box_to_tool_target` changes the represented point using a selected grasp:

```text
T_reference_tool = T_reference_box * T_box_tool
```

The grasp transform names source `suction_contact_frame`, target `box`, and its
scene revision. Its translation includes actual lateral contact offset, not only
half box height. The caller must bind the grasp to the correct measured object
and selected directed box representative; this utility is not an object-identity
or grasp-authorization gate. `rotate_vector` rotates normals/offsets without
adding an origin translation. Covariance and uncertainty transport are not yet
implemented by this baseline and remain part of the relevant migration rows.

## Interface inventory

The JSON is the machine-readable inventory authority. IDs link required
geometry/negative cases, source pointers, current assumptions and target meaning.
Case linkage is **not** evidence that a ROS consumer has installed the check.

| IDs | Boundary |
|---|---|
| F01–F05 | scene/TF; sensor ingress; detector; suction candidates; opening estimate |
| F06–F08 | cargo surface; solver candidates; SlotSpec producer/consumers |
| F09–F11 | pick legs; retry/recovery; place and grasp-to-tool conversion |
| F12–F14 | sweep/grid; selector service; atlas/task-grid artifacts |
| F15–F18 | motion serialization; IK/compensation; collision scene; attach/detach |
| F19–F21 | commit/remove/verify; measured integration; startup/orchestration |
| F22–F24 | eval/fixture drivers; GT/replay/viz; isolated legacy integration |

`migration_state` is pending, verified, or isolated. Marking a row verified alone
does not clear the gate. It also needs `runtime_evidence`, pointing under
`docs/status/evidence/` to an object with interface_id, kind=runtime_boundary,
status=pass, command, nonempty observations, positive sample_count, and a
source_sha256 entries matching **every** row source and both shared contract/
transform modules (`CONTRACT_SOURCES` in the checker), `contract_version=2`, and
`interface_contract_sha256` matching the checker-computed row contract digest.
Inventory v1 requires all F01–F24 rows; deleting a pending row is an error.
Isolation requires
kind=isolation_check and scope=legacy. Missing or stale proof remains pending.
Proof references are not cryptographic attestations; the implementation owner
must produce them from actual behavior. Negative/positive runtime acceptance
criteria must be approved when each row is implemented.

This is a curated boundary inventory, not a claim that static source scanning
can find every dynamically constructed caller. New producers/consumers require
new rows or updates. Actual runtime package paths, message versions, profiles,
and branch coverage must be captured during integration, not inferred from a
source-tree pass.

## Run and interpret the gates

From the workspace root, with Python 3 and the existing description-package
dependencies (PyYAML and ament_index_python for the original scene-helper case).
The validators themselves use only the standard library. Source the installed
Humble environment before comparing the original helper; no ROS process starts:

```bash
source /opt/ros/humble/setup.bash
python3 scripts/check_frame_contracts.py --mode baseline --output docs/status/evidence/frame_contracts/my-baseline
python3 scripts/check_frame_contracts.py --mode migration --output docs/status/evidence/frame_contracts/my-migration
PYTHONPATH=src/luggage_description python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py
```

Use a fresh output directory every time; overwrite is refused.

- Baseline exit 0: inventory is well-formed and all deterministic offline cases
  passed. `runtime_execution=not_evaluated` still applies.
- Migration exit 2: offline checks passed, but pending/stale interface proofs
  block claiming migration complete. This is an expected refusal today.
- Exit 1: malformed inputs, missing sources or failed comparison/check.

Each output directory has exact suite/inventory snapshots, complete case
inputs and expected/actual values, summary and a revision/source-hash manifest.
To replay, pass `--suite <run>/suite.json --inventory <run>/inventory.json` with
a fresh `--output`; compare recorded source hashes first. Source changes mean
a comparison on the new implementation, not reproduction of the old binary.

The fixed analytic cases use absolute position tolerance 1e-9 m and quaternion
sign-equivalent component distance 1e-9. They include nonzero translation and z,
90/180/non-axis-aligned rotation, full RPY mechanics versus tilt rejection,
non-square footprints, off-center TCP, changed floors and stacks, unknown cells,
q/-q, different tool branches, old contracts and stale identities. No random
sampling, IK reliability, physical success rate, or hardware certification is
inferred from this deterministic AND gate.

## Failure diagnosis

Callers add interface_id, producer, consumer and request_id to ContractError's
code/field/expected/received diagnostics. Keep the original and transformed pose
plus transform snapshot at stage boundaries. Never replace a contract failure
with generic IK failure, BIN_FULL, default pose, identity TF or empty map.

Runtime labels cannot prove numeric truth. Independent geometric oracles are
required because even a transform roundtrip can pass when both sides share the
same wrong convention. Live integration and dump windows follow
[debug-evidence](../../.cursor/rules/debug-evidence.mdc).
