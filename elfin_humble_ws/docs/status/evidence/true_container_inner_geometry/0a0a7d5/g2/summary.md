# TCIG-2 Gate G2 evidence

- tested_revision: `0a0a7d5ee72359089b090c3d57d1a687092dede6`
- dirty_file_count: 0
- geometry_hash: `cb41f55f243786d50ab30997291083ab40467507abcf78a00e55da77ffc00856`
- exact_usable_volume_m3: 4.22433625
- outcome: pass

## Verification

- `python3 -m pytest src/luggage_description/test -q`: 141 passed, 2 skipped.
- Focused G2 mapper/surface suites: 23 passed.
- `python3 -m pytest src/luggage_perception/test --ignore=src/luggage_perception/test/test_vintage_pose_regression.py -q`: 568 passed, 44 subtests passed.
- `python3 -m pytest src/luggage_packing/test/test_mapper_surface_contract.py -q`: 3 passed.
- `colcon build --packages-select luggage_msgs luggage_description luggage_perception luggage_packing --symlink-install`: pass, 4 packages built.
- `colcon test` passed `luggage_description` and `luggage_packing`; `luggage_perception` reached 95% before its existing 60-second CTest timeout.
- `git diff --check`: pass.
- `scripts/check_agent_contract.sh`: pre-existing base/mailbox mismatch; four `OPEN.md` rows reference DSIM threads added only in the primary worktree and absent from this isolated base.

## Known unrelated test issue

The excluded vintage-pose regression has a pre-existing ineffective `pytest.mark.skipif` call and attempts to initialize YOLO-World despite no usable CUDA/CLIP setup. It raises `ModuleNotFoundError: clip` and is outside TCIG-2 scope. All other perception tests pass.

No Gazebo or robot motion was started.
