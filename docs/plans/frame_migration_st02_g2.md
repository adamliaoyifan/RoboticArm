# ST02 generation 2 -- shared v2 ROS contract substrate

Parent FRAME-MIGRATION-20260929; owner eng/codex/gpt-6; prerequisite ST01=2
passed at 4da9b68047b2fbc2175c11743bf5cd18f62b1fc8. This is the exact base.
The user requested ST02 completion. Supersede queued generation 1 after this
plan is committed. Do not enable live v2 producers/consumers in this stage.

## Scope and binding decisions

Add luggage_msgs types FrameIdentityV2, PoseContractV2, WaypointArrayV2,
SurfaceContractV2, JointTrajectoryContractV2 and BoundaryDiagnosticV2, plus
ValidatePoseV2.srv. Reserve `/frame_contracts/v2/validate_pose`; no server or
endpoint remap is enabled here. Existing v1 messages/services/actions stay
byte-for-byte unchanged and importable. V2 pose profiles retain existing strict
plain-record contracts. Envelope explicitly distinguishes missing stamp from
valid epoch zero. Surface null cells use an explicit validity array, never a
floor/zero default. Waypoints carry complete independent pose identities.
Joint trajectories have named joint order and radians, not a Cartesian frame.

Pure validation and diagnostic records live in luggage_description; generated
message conversion is isolated in luggage_description/ros_frame_bindings, with
thin explicit perception/planning boundary adapters. Add the description-to-msgs
runtime dependency. Complete existing SE(3) math and strict transform identity
validation as necessary. Add vector/covariance rotation without translation;
translation affects positions only. No reference/frame default or v1 inference.
A diagnostic identifies interface, request, producer, consumer, declared/expected
frame/reference/scene/map/geometry/time basis and ContractError code/field.
Failed validation returns no converted payload and causes no downstream effect.

## Decision-complete acceptance

Use a fresh worktree `/tmp/elfin-frame-migration-st02`, branch
`integration/frame-migration-st02-20260929`, CPU-only Ubuntu/Python 3.10/Humble.
Start with env -i HOME=/home/adamliao PATH=/usr/bin:/bin LANG=C.UTF-8, source only
/opt/ros/humble/setup.bash. Empty build/install/log initially. No master changes,
shared WIP edits, simulation, hardware or live migration claims. Commit sources
before verification; record revision/dirty state. Repairs require separate source
commits and preserved failed evidence before rerunning affected gates.

```bash
colcon build --symlink-install --packages-up-to luggage_description luggage_msgs luggage_perception luggage_packing luggage_planning luggage_gazebo
source install/local_setup.bash
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_messages.py -v
python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py -v
python3 scripts/check_frame_contracts.py --mode baseline --output EVIDENCE/baseline
python3 scripts/check_frame_contracts.py --mode migration --output EVIDENCE/migration
```

Matrix: yaw 0, pi/2, pi, atan2(4,3); translation (0,0,0)/(1.5,-0.4,0.2);
floor .53/.71; support 0/.30; grasp (0,0,.16)/(.10,-.05,.16); WDH .80/.50/.32.
Exactly 64 positive geometry cases, each through generated pose, waypoint,
surface, joint and diagnostic messages and serialize_message/deserialize_message.
Analytic position/vector errors <=1e-9 m; quaternion sign-equivalent error
<=1e-6 after ROS serialization, <=1e-9 for pure math; covariance components
<=1e-9. Identity, joint order/names, stamps, map/hull versions preserved exactly.
Adapters deep-copy inputs; mutations after conversion do not affect originals.

Named negatives each once with exact ContractError: missing frame
MISSING_IDENTITY; wrong frame FRAME_CONTRACT_MISMATCH; wrong reference
POSE_REFERENCE_MISMATCH; version CONTRACT_VERSION_MISMATCH; units UNIT_MISMATCH;
stale scene CALIBRATION_REVISION_MISMATCH; absent stamp INVALID_STAMP;
time basis TIME_BASIS_MISMATCH; malformed/nonfinite pose INVALID_NUMERIC_PAYLOAD;
nonunit quaternion INVALID_QUATERNION; stale map MAP_REVISION_MISMATCH; wrong hull
GEOMETRY_IDENTITY_MISMATCH. Also check empty/default generated messages, invalid
surface shapes/validity, empty/mixed-identity paths, duplicate/mismatched joint
names, wrong vector lengths/nonfinite joint values, nonmonotonic times, Cartesian
joint labels, request mismatch, wrong message type and uint64 overflow.
All reject before spy downstream effect count increments. No inapplicable common
negative: surface provides map/hull coverage. Every case emits individual JSONL
results with expected/actual code and context; zero unexpected accepts/identity
loss. Missing samples are not_evaluated, not passing.

q/-q must be equivalent, directed tool yaw 0/pi distinct. Roll .2 admission is
rejected (UNSUPPORTED_CONTAINER_TILT); full 3D math still transforms that rotation.
Injected wrong-frame producer and position-only rotation must each fail an
independent geometric oracle (not only a roundtrip). Test generated v1 and v2
service request and action goal bindings before any v2 consumer is enabled.

All required obligations are ANDed, once per revision, 100% deterministic
per-case correctness. No robot success-rate or statistical reliability claim.
Build exit 0 and no failed packages; matrix >=64, zero unexpected failures;
existing suite 15/15 and 50/50 unchanged; migration expected exit 2/24 pending.
New tests need not claim runtime migration of F01--F24; inventory states stay pending.

## Evidence and capture

Primary docs/status/evidence/frame_migration/ST02-20260929/ only. Save plan,
source commit/hash manifest, clean environment, build logs and exit, unittest
logs and per-case JSONL (via FRAME_MIGRATION_CASE_LOG), baseline/migration bundles,
summary and required artifact hashes. Failure case records include complete
synthetic input/transform/expected/actual/diagnostic and traceback. Preserve whole
invocation before any repair; no rerun discards a failed artifact. 128 MiB per
attempt, 512 MiB total; capture health requires nonempty well-formed JSONL with
all expected IDs and durable failure records. Replay the exact unittest command
from the recorded commit. Import/compile failure includes compiler/traceback log
and pinned source; no rosbag is needed for this in-process serialization gate.
