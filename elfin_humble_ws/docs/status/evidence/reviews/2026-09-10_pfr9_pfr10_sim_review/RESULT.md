# PF-R9/PF-R10 simulation review evidence

- review time: 2026-09-10 14:25-14:35 +08:00
- PF-R9 accepted revision: `bcb54c9770dd555ccd4e36da8bbda63e76215bca`
- current PF-R10 review HEAD: `0001c413147e4a2f00a01801d6a8ac16ca92ba93`
- current shared-worktree dirty count at final check: 25
- outcome: PF-R9 remains pass; PF-R10 remains open

## Static review

- PF-R9 preprocessor subscribes to RGB, aligned depth, and CameraInfo, not the
  camera-native point cloud.
- Semantic support and detector support are locally deprojected from aligned
  depth and same-grid CameraInfo.
- Semantic and detector world transforms request TF at the acquisition stamp
  and return no geometry when the historical transform is unavailable.
- The active Gazebo launch still bridges `/d435/points`; this is the held
  DSIM-1 backend gap, not part of the accepted PF-R9 consumer boundary.

## Commands and results

PF-R9/current-HEAD focused contract suite:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest \
  src/luggage_perception/test/test_pf_r9_g2_d5_fixtures.py \
  src/luggage_perception/test/test_pf_r9_preprocessor_split.py \
  src/luggage_perception/test/test_sensor_preprocessor.py \
  src/luggage_perception/test/test_semantic_point_filter.py \
  src/luggage_perception/test/test_ros_message_adapters.py \
  src/luggage_perception/test/test_pf_g2a_stamped_status_tf.py -q
```

Result: `127 passed, 44 subtests passed`.

PF-R9 isolated source snapshot at `bcb54c9`: 85 pure tests and 42 subtests
passed. Four ROS-message/TF cases could not collect without a built snapshot
overlay; they were rerun successfully through the current built overlay above.

PF-R10 affected focused suite:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest \
  src/luggage_gazebo/test/test_pf_r5a_fix1_spawn_fail_closed.py \
  src/luggage_perception/test/test_sensor_preprocessor.py \
  src/luggage_perception/test/test_semantic_point_filter.py \
  src/luggage_perception/test/test_pf_r6_detector_instrumentation.py -q
```

Result: `46 passed, 3 failed`. The failures are stale direct-node fixtures:
one omits `_set_pose_cli`; two omit `_scratch_lock`/scratch-buffer state.
Production constructors initialize those members, but the regression suite
must be repaired and pass before PF-R10 closure.

Repository checks:

- `git diff --check`: pass.
- `scripts/check_agent_contract.sh`: fail with one issue because
  `docs/agents/eng/2026-09-10_0030_pf-r10-integration-session.md` uses
  `- status: open（...）` instead of the allowed exact value `open`.

## Acceptance decision

PF-R10 must retain all raw artifacts for three consecutive six-trial Gate-4
passes on one clean commit, pass PF-G6S and teardown, close the test/contract
failures above, and demonstrate bounded fail-closed Gazebo pose verification.
Hardware calibration and deployed TF-tree changes are deferred and are not
part of this simulation result.
