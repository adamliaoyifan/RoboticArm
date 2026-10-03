# Gazebo-flavour deploy-sim atlas cell (2026-10-03, ST09-I S5.4)

## Why this cell exists

`atlas_runtime.yaml` / `atlas_runtime_deploy.yaml` both freeze
`urdf_hash: 22ed00b5...` (hardware/mock expansion). A Gazebo
sim_world stack publishes the gz-flavour robot_description
(GazeboSimSystem plugin blocks) whose MD5 differs, so `hash_policy:
strict` refuses and the P3 atlas prior (reachability filter + IK seeds)
is unavailable in Gazebo. Verified live both ways:

- implicit pick (no override) -> `load_runtime_atlas` refuses with
  `hash_mismatch_strict`;
- explicit `atlas_runtime_config:=...atlas_runtime_deploy_sim.yaml` ->
  loads strict-clean (`scene_tf_ok`, `urdf_ok`, `contract ok`).

## Hash computation method (per S5 review: never from CLI print text)

The URDF hash is computed from the LIVE parameter bytes via the
production hash function, not from `ros2 param get` output (labels,
quotes, and escapes pollute a CLI-print hash):

```python
# rclpy parameter client -> /robot_state_publisher/get_parameters
# -> robot_description (string) -> production hash fn
from luggage_planning.atlas_runtime import compute_urdf_hash  # md5
```

Three-way agreement on 2026-10-03 (the running domain-7 Gazebo deploy
stack):

| source                                   | value                        |
|------------------------------------------|------------------------------|
| live RSP param (rclpy client, md5)       | 689838e5ab5948877ea9c87f388a0474 |
| cell yaml `urdf_hash` (builder-written)  | 689838e5ab5948877ea9c87f388a0474 |
| captured `robot_description.xml` (md5)   | 689838e5ab5948877ea9c87f388a0474 |
| live RSP param (sha256, recorded)        | 2589625c5a8755696ae76206fc26273bed2da9e696cd59ad6903f0741f350c17 |
| scene_tf_hash (src == install, md5)      | 98313fb8b30e51521a4c45a829ff6d5d |

## Build (all four variants, one live Gazebo deploy stack, domain 7)

Stack: the S5.3 bring-up (scene_tf_deploy, gui off, use_vacuum/motion/
semantic on; controller race retried once — bringup_retry1.log).

    python3 scripts/build_reachability_atlas_humble.py \
      --output-dir src/luggage_planning/data/reachability_atlas_deploy_sim \
      --scene-tf src/luggage_description/config/scene_tf_deploy.yaml \
      [--payload L,W,H]

rc=0 for all four; full log `logs/atlas_build.log`. The builder reads
the URDF from the live RSP and writes the capture itself.

| variant                        | reachable | marginal | elapsed |
|--------------------------------|-----------|----------|---------|
| empty                          | 665       | 63       | 171.1 s |
| payload 0.55x0.40x0.25         | 402       | 57       | 133.4 s |
| payload 0.70x0.45x0.28         | 381       | 54       | 133.2 s |
| payload 0.80x0.50x0.32         | 368       | 34       | 120.2 s |

Empty-load 665/63 matches `data/reachability_atlas_deploy/` exactly.

## NPZ byte-identity vs the mock-flavour deploy cell (SHA-256)

| file                          | verdict |
|-------------------------------|---------|
| s20_container_collision_aware.npz                          | SAME |
| ...payload_0.55x0.40x0.25.npz                              | SAME |
| ...payload_0.70x0.45x0.28.npz                              | SAME |
| ...payload_0.80x0.50x0.32.npz                              | SAME |

All four NPZ are byte-identical to `data/reachability_atlas_deploy/`:
the sampling is deterministic and the gz/mock URDF difference touches
only the ros2_control blocks, not kinematics/collision geometry. The
YAMLs differ exactly by the `urdf_hash` rebinding (22ed00b5 ->
689838e5) plus generated_at — an auditor can distinguish a
deterministic resample from a bare hash edit by rebuilding.

## Wiring

- config: `src/luggage_planning/config/atlas_runtime_deploy_sim.yaml`
  (field-for-field atlas_runtime_deploy.yaml; only atlas_dir differs;
  strict). Installed to install-st09.
- launch usage: EXPLICIT
  `atlas_runtime_config:=<install>/.../atlas_runtime_deploy_sim.yaml`
  (the implicit matcher selects atlas_runtime_deploy.yaml by scene
  hash and would strict-refuse). The S5 runner exports
  ATLAS_RUNTIME_CONFIG for the driver-side atlas diagnostics too.

## Recorded collateral (not fixed in S5)

Since the P3 rebinding of the SIM cell to 22ed00b5, any sim-scene
Gazebo run with `--atlas on` also strict-refuses (the old motion_occ
lineage). That predates S5; recorded here, not repaired. Long-term
direction (not S5): split atlas identity into a geometry-semantic hash
and a runtime-backend hash so one cell serves every backend.
