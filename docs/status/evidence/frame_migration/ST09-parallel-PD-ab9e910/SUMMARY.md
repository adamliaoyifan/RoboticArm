# ST09-PD deploy-profile asset preparation -- evidence summary

- subtask: ST09-PD (FRAME-MIGRATION-20260929), base ab9e910
- owner: eng/claude-deploy/glm-5.3 (claude-code)
- date: 2026-10-02
- worktree: /tmp/elfin-st09-deploy-assets, branch implementation/st09-deploy-assets

## What ran

1. `scripts/build_deploy_assets_campaign.sh` -- no-Gazebo planning-scene
   stack (`deploy_asset_stack.launch.py`: RSP + static joint_states +
   move_group + scene_manager, use_sim_time=false, planning only), then
   four atlas builds against `scene_tf_deploy.yaml` via
   `scripts/build_reachability_atlas_humble.py` (the ST06 humble
   driver), FK certification of each artifact
   (`scripts/certify_atlas_seeds.py`), the four directed route graphs
   (`scripts/build_deploy_route_graphs.py`), and the identity manifest
   (`scripts/generate_deploy_asset_manifest.py`).
2. `scripts/pd_evidence.sh` -- focused suites, graph structural
   validation, asset hash inventory, source-vs-fresh-install
   resolution, site-pose gating report.

The campaign aborted once at graph-build (grid meta stores 3 axes, the
builder unpacked 4); atlases and FK certifications were complete and
untouched, the graph/manifest stages resumed manually against a fresh
stack instance. All stages and the resume are recorded in
`lifecycle.jsonl`.

## Measured identities

- scene_tf_deploy.yaml md5 98313fb8b30e51521a4c45a829ff6d5d
- expanded hardware URDF (byte-identical to the hardware_pick
  expansion) md5 22ed00b5..., captured at
  data/reachability_atlas_deploy/robot_description.xml
- four atlas/graph pairs strict-load under hash_policy strict
  (concrete-tool v2 contract, directed_container_2pi)

## Results

- PD battery: 44 passed (launch resolution x4 from source AND a
  copied fresh install, five one-byte identity mutations each a named
  rejection with zero usable seeds/edges, graph endpoints/direction/
  costs/q-uniqueness/release twins/start-class route coverage,
  site-pose negatives incl. stale/wrong-scene/wrong-joint-order/sim
  provenance/valid:false).
- Graphs: empty 31 nodes/31 edges; payload variants 39-40 nodes/
  34-37 edges; 8/8 fixture targets chained per graph, 0 prior gaps
  (graph_validation.json).
- Suites: luggage_planning 791 passed; luggage_description 200
  passed. Remaining failures are the ab9e910 baseline set
  (frame_contracts x2, wrist j6 x1), reproduced on a stashed tree --
  not introduced by ST09-PD.
- Fresh colcon build (copied install prefix `install_fresh`, 13
  packages) resolves the same manifest hashes as the source tree
  (pytest_install_resolution_fresh.log).

## Honest boundaries

- The deploy cell container placement ([-1.5, 0.6, 0] yaw pi) yields
  11.9% empty-load reachability vs the sim cell's 26.9%; the sampled
  result is what the assets freeze.
- Payload contact layers required the floor-clamp contact rule (cell
  centre >= floor + box height); without it the lowest payload layers
  read unreachable by discretisation, not by geometry.
- Site poses are UNMEASURED: every deploy entry is valid:false and
  `current` stays the only executable fallback. That is a
  hardware-calibration prerequisite before any motion; it is not a
  validated pose (site_pose_report.json).
- Graph edges are offline priors consumed by B's route planner; the
  store loads them as candidates that production must re-verify in
  session. released_box_modeled is false on retreat edges (in-session
  re-verification adds the placed box).
