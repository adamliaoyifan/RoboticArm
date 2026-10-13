# ST06 generation 2 -- atlas and concrete tool branches

Parent FRAME-MIGRATION-20260929; owner eng/claude/glm-5.3; prerequisites
ST04=2 (8b0cd863) and ST05=2 (d147035), both passed, plus the accepted
review-repair heads f1d4e92 (round 1, Q-20260929-13),
20048a5a7004f5f06bb99c568b0d04cfe4bbd796 (round 2, Q-20260929-17 reply)
and dff51dc05aca25a4dbe17535b82124aaaa9e4ad6 (round 3, admitted
map_revision). Initially built on 20048a5; rebased onto dff51dc when
round 3 landed, with the F07 proof re-emitted against the round-3
planner sources. The user reassigned ST06 from the queued codex
generation 1 to this owner and scoped the certified artifact set to **all four
profiles** (empty-load + payloads 0.55x0.40x0.25, 0.70x0.45x0.28,
0.80x0.50x0.32). Supersede queued generation 1 after this plan is committed.
Module-task rules from `docs/plans/frame_migration_stages.md` ST06 apply: ST06
is never run against old slot/map semantics; existing NPZ/YAML is not
relabelled as v2; this artifact/FK gate is not an IK success-rate claim or a
full atlas campaign. No scored simulation, hardware motion, master merge or
shared-worktree edits. The bounded builder campaign below is offline artifact
generation against the exclusive atlas stack, not a scored run.

## Scope and binding decisions

- **Directed 2pi yaw bins (F14 target)**: builder `~yaw_bins` default becomes
  `[0.0, pi/2, pi, 3*pi/2]` -- four directed container-frame concrete-tool
  bins (the contract fixture bins). `_construct_base_pose` treats the bin yaw
  as a **container-frame** directed yaw and maps it to base-link yaw through
  the scene-config container rotation (constant Z offset; container admission
  already requires horizontal, so the offset is analytic). Base-frame IK,
  collision and wavefront mechanics are unchanged. Grid and sampling are
  otherwise unchanged: res 0.15, origin/inner dims from the pinned s20 sim
  scene (inner 1.49x1.97x2.01, floor 0.53, ceiling 2.01, opening negative_x),
  z sampling `[max(0, floor-0.03), +0.05]`, `transit_clearance 0.30`,
  `safe_tool_z 1.55`, `min_lift 0.06`, `tool_down_tolerance 0.0`,
  `max_branches 3`, `branch_distance 0.25`, `joint_interpolation_step 0.10`,
  serial deterministic.
- **v2 artifact metadata**: the YAML gains a top-level contract block consumed
  verbatim by `luggage_description.frame_contracts.validate_atlas`:
  `contract_version: 2`, `position_frame/orientation_frame: container_link`,
  `reference: suction_contact_frame`, `orientation_semantics:
  concrete_tool_pose`, `angle_unit: rad`, `length_unit: m`, `height_datum:
  container_origin`, `tool_down_reference: container_negative_z`,
  `yaw_period_rad: 2pi`, `yaw_bins_rad` (the four bins), `scene_revision`
  (the scene_tf_hash hex -- one identity source), `geometry_hash` (md5 over
  the canonical container geometry block), `urdf_hash`, `payload_hash` (md5
  over the payload block; distinct constant for empty-load).
  `atlas_version: "3.0" / schema_version: 3`; same file basenames (git
  history preserves the v1 content); `_load_arrays` keeps v1/v2
  normalization and adds v3 (4 yaw bins, contract block passthrough).
- **Directed query semantics (no pi fold on v2)**: `atlas_runtime` gains a
  directed snap (nearest bin over wrapped 2pi distance, no mod-pi) selected
  automatically when the artifact carries the v3 contract block. A v1
  artifact (no contract block) under `hash_policy: strict` is rejected with
  `ATLAS_SEMANTICS_MISMATCH` before any query/seed use; the deploy `warn`
  profile may keep running v1 with an explicit logged downgrade (declared,
  not silent). `ReachabilityAtlas.is_reachable` (infinite yaw tolerance) is
  v1-only.
- **Placement adapter (F07 seam, consumer side)**:
  `placement_reachability.annotate_with_atlas` queries **both directed
  pi-equivalent grasps** (box_yaw and box_yaw+pi snapped to directed bins);
  the candidate is atlas-reachable if either bin is >= MARGINAL; hard reject
  only when both are UNREACHABLE with `neighbor_confidence >= 0.9` and strict
  cell-interior. Pi aggregation lives at this decision layer only -- the
  artifact keeps concrete directed branches (a failed branch never certifies
  pi-equivalents unreachable).
- **FK certification (new, ROS-free)**: `scripts/certify_atlas_seeds.py`
  loads NPZ+YAML, recomputes the URDF from the installed `luggage_description`
  and asserts `urdf_hash` equality, then FKs **every stored contact/transit
  seed** with `offline_ik.Urdf.tcp` against the advertised container target
  transformed container->base: position <= 0.002 m and orientation angular
  error <= 0.02 rad. Exit nonzero on any breach; per-seed JSONL (cell, bin,
  branch, FK pose, errors, verdict) with thread-pinned BLAS env. The builder
  runs it as a post-build gate; >= 1 sampled reachable seed per directed yaw
  bin per profile or the profile is `not_evaluated` and the task cannot close.
- **F16 seed side**: `motion_planner_node._atlas_seed_provider` /
  `_refine_atlas_seeds` and `motion_executor._ik_joints` take directed yaw
  from a v2 artifact (no fold) and gate on identity + `payload_hash` before
  any seed is returned or refined (extends `identity_verdict`). Consumer-side
  F16 (MoveIt boundary) stays ST08; row F16 remains pending after ST06.
- **Task grid: explicit v1 isolation, not relabelling**:
  `reachable_task_grid.save_workspace/save_task_grid` and
  `scripts/build_ikfast_task_grid.py` emit an explicit `semantics:
  v1_pi_folded` marker in their metadata. The task grid has no runtime
  consumer (verified by grep over src/ and runtime configs) and is **not**
  certified v2; F14 v2 closure rides the reachability atlas. Declared
  deferral, not an omission.
- **Build campaign (bound commands)**: fresh verified colcon install of this
  worktree; exclusive atlas stack (Gazebo + move_group + scene_manager,
  gui=false, ROS_DOMAIN_ID=7), queued behind any live sim, teardown via
  `scripts/stop_sim.sh` after evidence is durable. Per profile:

  ```bash
  python3 scripts/build_reachability_atlas_humble.py \
    --output-dir src/luggage_planning/data/reachability_atlas \
    [--payload "L,W,H"] --gate --old-atlas <same-profile v1 prefix>
  ```

  `--gate` regression checks (iz5/6 opening-connected not regressed, no
  REACHABLE at z >= 1.70, no unexplained low-layer drops) must pass against
  the v1 artifact of the same profile. 4-bin build ~2x the ~2 min v1 build;
  total campaign <= ~30 min, evidence within the 256 MiB atlas-boundary cap.

## Decision-complete acceptance

Fresh worktree `/tmp/elfin-frame-migration-st06`, branch
`integration/frame-migration-st06-20260929`, CPU-only
Ubuntu/Python 3.10/Humble, `env -i HOME=/home/adamliao PATH=/usr/bin:/bin
LANG=C.UTF-8`, source only `/opt/ros/humble/setup.bash` (+ the verified
install for suites). Commit sources before verification; record
revision/dirty state. Builder-stack runs record stack launch/teardown
timestamps and the sim-slot exclusivity check.

```bash
colcon --log-base log/verified build --build-base build/verified --install-base install/verified --symlink-install --packages-up-to luggage_description luggage_msgs luggage_perception luggage_packing luggage_planning luggage_gazebo
source install/verified/local_setup.bash
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_atlas.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_messages.py -v
python3 -m unittest discover -s src/luggage_planning/test -p test_frame_migration_pick.py -v
python3 -m unittest discover -s src/luggage_packing/test -p test_frame_migration_placement.py -v
python3 scripts/certify_atlas_seeds.py --atlas <each of the four new artifacts> --output EVIDENCE/st06/fk/<profile>
python3 scripts/check_frame_contracts.py --mode baseline --output EVIDENCE/st06/baseline
python3 scripts/check_frame_contracts.py --mode migration --output EVIDENCE/st06/migration
```

Matrix (64 analytic target-pose cases): container yaw 0, pi/2, pi,
atan2(4,3); container translation (0,0,0) and (1.5,-0.4,0.2); container floor
z 0.53 and 0.71; support height 0 and 0.30; grasp offset in box axes
(0,0,0.16) and (0.10,-0.05,0.16); payload WDH (0.80,0.50,0.32). Per case the
advertised container-frame target (slot box center z = floor + support + h/2,
TCP through the grasp offset) transformed container->base must match
independent quaternion/rotation math to <= 1e-9 m / 1e-9 quaternion distance
(pure Python; <= 1e-6 through ROS serialization where a message is involved).
Tool yaw 0 and pi are distinct branches (distinct records, not merged); q and
-q share a rotation (sign-equivalent comparison passes, yaw+pi does not).

Artifact and negatives: each rebuilt artifact's contract block passes
`validate_atlas` with the runtime-supplied expected identity. Negatives each
once with exact codes: `orientation_frame: elfin_base_link` ->
`ATLAS_SEMANTICS_MISMATCH`; `yaw_period_rad: pi` (pi fold) ->
`ATLAS_SEMANTICS_MISMATCH`; stale `payload_hash` -> `ATLAS_IDENTITY_MISMATCH`;
missing `urdf_hash` -> `MISSING_IDENTITY`; v1 artifact under strict policy ->
`ATLAS_SEMANTICS_MISMATCH` with zero seeds returned (spy counts the seed
extraction calls); stale scene/URDF identity rejects before seed use
(zero seeds). Unknown cells stay UNKNOWN (a failed branch does not mark
pi-equivalents unreachable; the placement adapter hard-rejects only when both
directed bins are confidently UNREACHABLE interior).

FK gate: every stored seed in all four artifacts <= 0.002 m / <= 0.02 rad;
per directed yaw bin per profile at least one sampled reachable seed, else
the profile is recorded `not_evaluated` and ST06 stays open. Preserve every
sampled seed, FK pose, error and branch id in the evidence JSONL.

Existing suites green: `test_reachability_atlas.py`,
`test_atlas_runtime.py`, `test_atlas_builder_wavefront.py`,
`test_motion_atlas_seeds.py`, `test_reachable_task_grid.py`,
`test_placement_atlas_hook.py`, the planning battery (594 at f1d4e92) and
packing battery (the one live-atlas URDF-identity env failure is documented
pre-existing at f1d4e92; re-run under the verified install and record either
way); ST02-ST05 suites unchanged; contracts 15/15; baseline 50/50.

Migration-gate expectation: baseline exit 0; migration exit 2 with
**13 pending** -- F14 flips to verified with an on-branch runtime proof whose
`source_sha256` covers every F14 row source (the six listed files plus
`scripts/certify_atlas_seeds.py` and the rebuilt artifacts if review binds
them); F16 stays pending (ST08 owns the consumer side). The primary
workspace keeps reporting F14 pending until merge.

## Evidence and capture

Primary `docs/status/evidence/frame_migration/ST06-20260929/` in this
worktree (mirrored to the primary workspace): plan, revision/dirty state,
build log/exit, builder campaign log with per-profile `--gate` output and
deterministic fingerprints, the four rebuilt NPZ/YAML pairs, per-seed FK
certification JSONL per profile, unittest logs, per-case JSONL via
FRAME_MIGRATION_CASE_LOG, baseline/migration bundles, summary.md,
manifest.json with artifact hashes. Failure records keep complete
input/expected/actual/traceback; no overwrite; 256 MiB bound on the atlas
boundary.
