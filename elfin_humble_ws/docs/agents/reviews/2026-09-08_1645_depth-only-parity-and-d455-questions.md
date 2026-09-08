# 2026-09-08 -- Depth-only parity argument and D455 swap questions

- role: reviews
- agent: cursor
- model: opus5
- cli: cursor
- status: done

## Summary

Two communication actions, no code change and no dispatch.

First, appended a `## Post` to the open F3 thread
`2026-09-08_1310_pixel-space-masking-consensus-trigger.md` (`Q-20260908-1`)
with two arguments that were missing from the 2026-09-08 13:11 request, both
independent of the simulation throughput bars. Payload arithmetic: at 640x480,
depth `16UC1` is 0.61 MB per frame against 3.69-9.83 MB for the equivalent
cloud, so F3 is properly "delete the cloud from the contract", not "shrink the
cloud" -- `cloud_decimation_stride` already tried shrinking and stride 3 broke
B4. Parity: gz `rgbd_camera` emits a cloud unconditionally while
`realsense2_camera` can set `pointcloud.enable:=false`, so a depth-primary
contract is the only option that keeps one perception codebase for sim and
hardware. Depth-primary also deletes the existing `input_cloud_data_frame`
gz workaround rather than adding a new one, and an XYZ cloud at 30 Hz
(0.9-2.4 Gbps) cannot cross the planned 1 GbE deployment bus at all.

The mailbox row for `Q-20260908-1` was deliberately left untouched.
`upsert_open_row` replaces every cell after the id, which would have
overwritten `eng/claude`'s `from_*` fields and original request text. A thread
event is the correct carrier for added argument; routing and target are
unchanged.

Second, opened `Q-20260908-2` as a new question thread
`2026-09-08_1643_d455-sim-parity-and-min-z-conflict.md` for the D435-to-D455
simulation swap. The stated intent was that hardware is a D455 and sim should
match, but `docs/plans/d455_replace_d435/` records three conflicting facts, so
this needs a reviews decision rather than direct implementation. This was kept
as a separate thread because F3 is a distinct question lineage and the format
contract forbids a parallel thread on the same question.

## Risks

- The 2026-09-02 unit snapshot records a **D555 PoE** (SN `419222302385`) in the
  `arm_realsense_v1.3` pocket, not D435i and not D455. If the goal is parity
  with hardware, the sim target may be D555; pursuing D455 would introduce a
  third camera body. This must be confirmed before any URDF change.
- D455 full-resolution Min-Z is about 0.52 m. `pickup_observe` puts the optical
  frame at world z = 1.9 m over a 0.86 m platform, so a 0.80 m box top sits
  about 0.24 m from the camera, inside the blind zone. D435's 0.105 m near clip
  is why height estimation works at this pose. This is geometric and cannot be
  fixed in software; re-qualifying the pose changes metres-per-pixel and
  invalidates existing top/support thresholds as regression baselines.
- D455's 124 mm long edge does not fit the printed 90 mm D435 pocket shared with
  the Mid-360 via `eef_mount_adapter`. D555 fitting that pocket does not prove
  D455 fits.
- Sequencing: F3 changes the cost of the camera swap, not the reverse. A
  depth-primary contract removes the `replacement.md` section 4 aligned-versus-
  unaligned cloud-frame trap and lifts the phase 1 resolution constraint, so F3
  should be decided first.
- If F3 is accepted as depth-primary,
  `docs/plans/platform_free_height_gate5_bag_contract.md` needs amending: it
  lists `/luggage/preprocessed/camera/depth/points` as a required online topic,
  which no hardware recording would contain.

## Pointers

- `docs/agents/discuss/2026-09-08_1310_pixel-space-masking-consensus-trigger.md`
- `docs/agents/discuss/2026-09-08_1643_d455-sim-parity-and-min-z-conflict.md`
- `docs/plans/d455_replace_d435/replacement.md`
- `docs/plans/platform_free_height_gate5_bag_contract.md`
- `docs/agents/eng/2026-09-08_1512_d455-official-tf.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/RESULT.md`
- base revision `985a43ba2dd811683f9ca3ff5ad8ad371cd88ea6`
