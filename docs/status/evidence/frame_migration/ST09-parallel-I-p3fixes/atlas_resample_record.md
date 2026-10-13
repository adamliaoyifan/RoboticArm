# Atlas resample record (2026-10-03, evidence for the P3 fix commits)

## Command (all four variants, sequential, one planning stack session)

    bash /tmp/run_atlas_stack.sh      # conftest-template stack, domain 69
    bash /tmp/run_probe.sh            # graph heartbeat probe (lease keepalive)
    bash /tmp/run_atlas_build.sh      # exit 0 — wraps:
      python3 scripts/build_reachability_atlas_humble.py         --output-dir src/luggage_planning/data/reachability_atlas         --scene-tf  src/luggage_description/config/scene_tf.yaml.example         [--payload L,W,H]

Preconditions enforced by the builder: scene sync through
/scene_manager/sync_static_scene (PEER leases live via the probe),
payload detach before each build, planning scene = container mesh
(airport_container_real, 8000 triangles) via move_group.

## Per-variant results (from the build log)

| variant                        | connected (yaw 4/4) | elapsed |
|--------------------------------|---------------------|---------|
| empty                          | 391                 | 219.7 s |
| payload 0.55x0.40x0.25         | 222                 | 176.8 s |
| payload 0.70x0.45x0.28         | 192                 | 165.7 s |
| payload 0.80x0.50x0.32         | 166                 | 155.6 s |

Payload attach (builder-side, envelope semantics): link
suction_contact_frame, offset [0,0,h/2], touch_links suction_panel.

## SHA-256 — resampled vs pre-resample (bd9aec5)

| file                          | current            | bd9aec5            | verdict |
|-------------------------------|--------------------|--------------------|---------|
| ...aware.npz                  | 15a102c8fb5ca8d7…  | 15a102c8fb5ca8d7…  | SAME    |
| ...payload_0.55x0.40x0.25.npz | 4f5699eefc13bf6d…  | 4f5699eefc13bf6d…  | SAME    |
| ...payload_0.70x0.45x0.28.npz | f57578cb2eca9bbf…  | f57578cb2eca9bbf…  | SAME    |
| ...payload_0.80x0.50x0.32.npz | 28d7157490aa3120…  | 28d7157490aa3120…  | SAME    |
| ...aware.yaml                 | 631eaab2d6faf33c…  | 9964386c2f2d7d83…  | DIFF    |
| ...(all payload .yaml)         | (see git diff)     | (see git diff)     | DIFF    |

## Explicit statement

The four NPZ binaries are BYTE-IDENTICAL before and after the
resample: the sampling is deterministic and the URDF change between
the two runs (git diff robot_description.xml) affects only the
ros2_control plugin/initial-value blocks, not the kinematic/collision
geometry the atlas samples. The YAML diffs are identity rebinding
only: urdf_hash e9577fe5… -> 22ed00b5…, generated_at, and the
robot_description.xml snapshot. This record exists so an auditor can
distinguish a deterministic resample from a bare hash edit.

## Identity fields (post-resample, from the YAML contract block)

- scene_revision: b40c85ae6eb2eabc47d7da2ffb23b72c (unchanged)
- urdf_hash: 22ed00b5577d4b57421d9f02cf3fb2bf (rebound)
- position_frame / orientation_frame: container_link
- reference: suction_contact_frame
- orientation_semantics: concrete_tool_pose
- yaw_period: 2*pi, yaw bins [0, pi/2, pi, 3pi/2]

## Paths

- source: src/luggage_planning/data/reachability_atlas/
- install: install-st09/luggage_planning/share/luggage_planning/data/reachability_atlas/
  (plain colcon copy; the loader resolves src first, then share)
