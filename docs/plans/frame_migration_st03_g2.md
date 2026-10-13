# ST03 generation 2 -- perception, pick and recovery frame boundaries

Parent FRAME-MIGRATION-20260929; owner eng/claude/glm-5.3; prerequisite ST02=2
passed at 304d48c383f30e95b39b0426502b535e843198ca. This is the exact base. The
user reassigned ST03 from the queued codex generation 1 to this owner and
requested completion. Supersede queued generation 1 after this plan is committed.
Module-task rules from `docs/plans/frame_migration_stages.md` ST03 apply. No
simulation, hardware motion, master merge or shared-worktree edits.

## Scope and binding decisions

Migrate the F02-F05/F09/F10 boundaries onto the ST02 substrate. World detection
and pick stay world; camera extrinsics and 2D masks are unchanged. Interface
endpoints enabled in this stage:

- `/luggage/detection/luggage_v2` : `luggage_msgs/PoseContractV2` (profile
  `detection`), published by `luggage_detector_node` alongside the unchanged
  v1 `DetectedLuggage` topic. `PoseContractV2` carries identity, reference,
  orientation semantics and pose only; body-axes dimensions continue to travel
  on the v1 message, byte-stable. The v2 record carries world box center,
  `orientation_semantics=box_axis_pi`, `time_basis=acquisition`, the true cloud
  stamp, scene revision and request id from the detection request. When PCA yaw
  is invalid (near-square footprint), the record uses the declared `yaw_fallback`
  provenance already produced by the estimator; no measured orientation is
  invented.
- `/luggage/suction/candidates_v2` : `luggage_msgs/PoseContractV2` (profile
  `contact`), one message per selected candidate, world surface-contact point,
  `orientation_semantics=surface_normal` (+Z outward normal), observation
  identity fields from the suction record.
- Opening estimate (F05): `ContainerOpeningEstimator.estimate` additionally
  returns an explicit-frame record (world opening pose, nested aperture
  vertices in the opening frame, outward normal, stamp/scene revision). No new
  ROS topic: the only ROS consumer of `ContainerOpeningEstimate` is the ST07
  sweep (F12) and the ROS1 exploration node (F24 legacy, not migrated here).
  This deferral is part of this plan, not an omission.
- Pick entry (F09): `build_candidate_pick_segments` stores the explicit suction
  target link (`suction_contact_frame`), the observation identity, and the
  actual box-to-tool grasp transform via
  `frame_transforms.box_to_tool_target`, bound to the selected candidate and
  its detection. V1 `MotionSegment` publication stays byte-identical.
- Retry (F10): the pick session records the approach snapshot as
  (target_pose, link_name, transform_snapshot, scene_revision);
  `retry_reverse_segment` rebuilds only from that snapshot, and recovery
  verification rejects a changed scene revision (`CALIBRATION_REVISION_MISMATCH`
  at the boundary) with zero vacuum/motion side effects.
- Sensor ingress (F02): `_publish_observation` products keep their own
  acquisition stamps and truthful frame ids; admission rejects an observation
  record with a missing stamp (`INVALID_STAMP`). The joint-state
  `fallback_stamp_sec` path in `sensor_preprocessor_node` is robot-state
  (F15/F16) scope and is documented, not changed. The ROS1 exploration node's
  latest-TF use is F24 legacy scope, recorded not fixed.
- `judge_probe` (F04 consumer admission) accepts a candidate only when the v2
  identity matches frame=world, time_basis=acquisition and the session scene
  revision; the v1 string-compare path is kept for v1 records.

Existing v1 topics, services and messages stay byte-for-byte unchanged
(`test_frame_migration_messages.py` v1 wire assertions must pass unmodified).

## Decision-complete acceptance

Fresh worktree `/tmp/elfin-frame-migration-st03`, branch
`integration/frame-migration-st03-20260929`, CPU-only Ubuntu/Python 3.10/Humble.
Start from `env -i HOME=/home/adamliao PATH=/usr/bin:/bin LANG=C.UTF-8`, source
only `/opt/ros/humble/setup.bash`. Empty build/install/log initially. Commit
sources before verification; record revision/dirty state. Repairs require
separate source commits and preserved failed evidence before rerunning affected
gates. Existing perception/planning suites for touched functions must pass
unchanged unless this plan names the update.

```bash
colcon --log-base log/verified build --build-base build/verified --install-base install/verified --symlink-install --packages-up-to luggage_description luggage_msgs luggage_perception luggage_packing luggage_planning luggage_gazebo
source install/verified/local_setup.bash
python3 -m unittest discover -s src/luggage_perception/test -p test_frame_migration_perception.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_pick.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_messages.py -v
python3 -m unittest discover -s src/luggage_description/test -p test_frame_contracts.py -v
python3 scripts/check_frame_contracts.py --mode baseline --output EVIDENCE/st03/baseline
python3 scripts/check_frame_contracts.py --mode migration --output EVIDENCE/st03/migration
```

Matrix: yaw 0, pi/2, pi, atan2(4,3); translation (0,0,0)/(1.5,-0.4,0.2); floor
.53/.71; support 0/.30; grasp (0,0,.16)/(.10,-.05,.16); WDH .80/.50/.32.
Exactly 64 positive geometry cases across the detection record, contact record
and pick/retry snapshots, each through real generated messages and
serialize_message/deserialize_message. Sensor ingress cases feed a fixed
stamped synthetic point set through the admission seam; YOLO is not rerun and
catalog dimensions are never used as live perception. Analytic position/vector
errors <=1e-9 m; quaternion sign-equivalent error <=1e-6 through ROS
serialization; covariance rotation components <=1e-9 against an analytic
diagonal-anisotropic case. Acquisition stamps are preserved exactly, including
epoch zero as an explicit valid stamp. Contact +Z equals the outward surface
normal and the tool -Z direction for the derived tool orientation on every
case. The grasp transform stores the actual lateral offset: for offset
(.10,-.05,.16) the stored T_box_tool translation equals that offset in box
axes. Adapters deep-copy inputs; post-conversion mutation does not change
emitted messages.

Named negatives each once with exact ContractError, all rejected before any
publish/plan/vacuum/occupancy effect (spy counter stays zero): missing frame
MISSING_IDENTITY; wrong frame FRAME_CONTRACT_MISMATCH; wrong reference
POSE_REFERENCE_MISMATCH; version CONTRACT_VERSION_MISMATCH; units
UNIT_MISMATCH; stale scene CALIBRATION_REVISION_MISMATCH; absent observation
stamp INVALID_STAMP; wrong time basis TIME_BASIS_MISMATCH; malformed/nonfinite
pose INVALID_NUMERIC_PAYLOAD; nonunit quaternion INVALID_QUATERNION.
Inapplicable for ST03 and named here: stale map revision MAP_REVISION_MISMATCH
and wrong hull identity GEOMETRY_IDENTITY_MISMATCH (no map/hull surface on
these boundaries; they enter at ST04/ST05). Session-level rejections keep their
documented reason codes with zero side effects: candidate mismatch (existing
suction identity mismatch reason), stale observation, missing TF
(DETECT_TF_FAILED path unchanged), cross-object grasp, stale calibration.
An injected wrong-frame producer and an injected position-only rotation must
each fail an independent geometric oracle (aperture/covariance/grasp checks
above), not only a roundtrip. q/-q equivalent; tool yaw 0/pi distinct; roll
0.2 rad rejected by require_horizontal while transform math remains SE(3).

Retry acceptance: given a recorded approach snapshot, `retry_reverse_segment`
reproduces the reverse target from the snapshot (exact pose equality) and never
queries TF; mutating live TF state between record and retry changes nothing.
A retry whose scene_revision differs from the snapshot's is rejected with zero
motion/vacuum calls.

Existing suites that must pass unchanged for touched functions:
`test_sensor_preprocessor.py`, `test_suction_patch_evaluator.py`,
`test_container_opening_estimator.py`, `test_ros_message_adapters.py`
(perception), `test_waypoint_generator.py`, `test_suction_candidate_planning.py`,
`test_suction_retry_state_machine.py`, `test_hardware_pick_launch.py`
(planning). Synthetic/GT references stay in test fixtures only; no production
perception input gains fixture data.

Migration-gate expectation: baseline exit 0 (50/50), contracts suite 15/15,
ST02 message suite 134 cases unchanged. F02-F05, F09, F10 rows flip to verified
with runtime_evidence proofs under
`docs/status/evidence/frame_migration/ST03-20260929/` validated by running the
migration mode from this worktree; all other rows remain pending. The primary
workspace migration run still reports these rows pending until merge; that is
recorded, not hidden.

## Evidence and capture

Primary `docs/status/evidence/frame_migration/ST03-20260929/` (created in the
stage worktree for checker-validated proofs, mirrored to the primary workspace
summary). Save plan, source commit/hash manifest, clean environment, build
logs and exit, unittest logs and per-case JSONL via FRAME_MIGRATION_CASE_LOG,
baseline/migration bundles, summary.md and manifest.json with artifact hashes.
Failure records keep complete synthetic input/expected/actual/traceback; no
artifact overwrite; 16 MiB bound per module run. Per-case IDs are stable and
replayable from the recorded commit with the exact unittest command.
