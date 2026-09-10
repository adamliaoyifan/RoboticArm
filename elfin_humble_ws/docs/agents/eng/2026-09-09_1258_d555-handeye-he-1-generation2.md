# 2026-09-09 -- HE-1 generation 2 CAD seed blocked

- role: eng
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: open
- parent: D555-HANDEYE-20260909
- subtask: HE-1
- base_revision: 42f97af0b5329326ae399ab0809a905e33d28906
- started_at: 2026-09-09T12:27:22+08:00
- completed_at: n/a

## Summary

Claimed HE-1 generation 2 and derived a unique outer-pair mechanical seed
`^eef_mount_adapter T_D555-mechanical` from official D555 STEP plus
`arm_realsense_v1.3.stl`. HE-1 remains blocked because no authoritative
housing-to-`d555_link` (left IR) datum exists. Board, capture, and solve
scripts are in tree. URDF/xacro/yaml were not edited.

## Requirement

- Derive and validate `^eef_mount_adapter T_d555_link` from official D555 CAD,
  the mount STL, and the operator-confirmed assembly constraints at plan
  revision `b45c4e875b76f28ffb323ef2faddc7ef8e9400c7`.
- Enumerate the bar's four holes; exclude Mid360 and EEF flange holes.
- Fail closed if housing-to-`d555_link` cannot be established from official
  D555/D450 material plus driver TF.
- Emit the true-scale ChArUco board and offline capture/solve scripts.
- Report the seed; do not apply it.

## Changed

- `src/luggage_description/luggage_description/he1/`
- `src/luggage_description/scripts/he1_d555_cad_seed.py`
- `src/luggage_description/test/test_he1_cad_seed.py`
- `src/luggage_description/CMakeLists.txt`
- `src/luggage_perception/scripts/handeye_capture.py`
- `src/luggage_perception/scripts/handeye_solve.py`
- `src/luggage_perception/test/test_handeye_solve.py`
- `src/luggage_perception/CMakeLists.txt`
- `docs/status/evidence/d555_handeye/20260909_he1_g2/`

## Verification

- `python3 -m pytest src/luggage_description/test/test_he1_cad_seed.py src/luggage_perception/test/test_handeye_solve.py -q`: 10 passed.
- `python3 src/luggage_description/scripts/he1_d555_cad_seed.py --cad-step /tmp/d555_he1_cad/D555_SOLID_SOC.STEP --mount-stl src/luggage_gazebo/models/arm_realsense/arm_realsense_v1.3.stl --out-dir docs/status/evidence/d555_handeye/20260909_he1_g2 ...`: outcome=blocked, unique=True.

## Result

- blocked: unique `STL_BAR_H0`/`STL_BAR_H3` mechanical seed exists, but official CAD/datasheet do not give housing-to-left-IR, so `^eef_mount_adapter T_d555_link` is not reported.

## Pointers

- `docs/plans/d555_handeye_calibration.md`
- `docs/agents/discuss/2026-09-09_1218_d555-handeye-he-1-generation2.md`
- `docs/status/evidence/d555_handeye/20260909_he1_g2/`
