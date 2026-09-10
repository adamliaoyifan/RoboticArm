# HE-1 generation 2 result

Blocked. The unique bar-hole pair and seating pose were derived, but
`^eef_mount_adapter T_d555_link` cannot be closed: official D555 CAD and
datasheet give no housing-to-left-IR datum.

## Tested revision

- Git HEAD at run: `42f97af0b5329326ae399ab0809a905e33d28906`
- Dirty-file count: 33 (workspace-wide; HE-1 files are the `he1/` package,
  capture/solve scripts, this evidence tree, and the HE-1 discuss thread)

## Commands

```bash
python3 -m pytest src/luggage_description/test/test_he1_cad_seed.py \
  src/luggage_perception/test/test_handeye_solve.py -q
python3 src/luggage_description/scripts/he1_d555_cad_seed.py \
  --cad-step /tmp/d555_he1_cad/D555_SOLID_SOC.STEP \
  --mount-stl src/luggage_gazebo/models/arm_realsense/arm_realsense_v1.3.stl \
  --out-dir docs/status/evidence/d555_handeye/20260909_he1_g2 \
  --cad-zip /tmp/d555_he1_cad/cad_41953.bin \
  --datasheet /tmp/d555_he1_cad/ds_42013.bin \
  --tessellation-glb /tmp/d555_he1_cad/d555_tol0.2.glb \
  --tessellation-glb /tmp/d555_he1_cad/d555_tol0.05.glb
```

Pytest: 10 passed. Pipeline outcome: blocked, unique mechanical pair: true.

## Mechanical seed

`^eef_mount_adapter T_D555-mechanical` in
`T_eef_mount_adapter_D555_mechanical.json`.

D555-mechanical is the SolidWorks export frame of `D555_SOLID_SOC.STEP`
(millimetres, origin at the front-face centre). It is not `d555_link`.

## Why blocked

STEP has no named IR/DEPTH/OPTICAL/camera origin. Datasheet v1.1 gives 2x M4
at 125.40 ± 0.20 mm and 167 x 42 x 48 mm, not a left-IR offset. There is no
official D555 URDF. D455 `mesh_x_offset` / `zero_depth_to_glass` were not
applied. HB-1 only verifies the ROS convention that `d555_link` coincides with
left IR / depth.

## Not applied

No URDF, xacro, or YAML was edited.
