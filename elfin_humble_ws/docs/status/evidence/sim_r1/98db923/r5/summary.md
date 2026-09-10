# SIM-R1-5 Gate R5 evidence

- tested_revision: `98db9236ab0204aa5fe4f81dc46b05f20d7d65b9`
- dirty_file_count: 0
- ros_domain: isolated per-test domain `120 + pid % 80`
- outcome: pass

## Verification

- `python3 -m pytest src/luggage_planning/test/test_sim_r1_contracts.py -q`: 14 passed, 14 subtests passed.
- `python3 -m pytest src/luggage_bringup/test -q`: 9 passed, including the 60-second no-Start graph.
- `colcon build --packages-select luggage_msgs luggage_planning luggage_bringup --symlink-install`: pass, 3 packages built.
- `colcon test --packages-select luggage_msgs luggage_planning luggage_bringup --event-handlers console_direct+`: pass.
- `colcon test-result --verbose`: 260 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: pass.
- `scripts/check_agent_contract.sh`: pre-existing base/mailbox mismatch; four `OPEN.md` rows reference DSIM threads added only in the primary worktree and absent from this isolated base.

## No-Start graph

The isolated ROS 2 graph ran for 60.21 seconds before Start with zero reset calls, exploration goals, motion goals, named-pose goals, and vacuum calls. One explicit Start then produced exactly one reset followed by one declarative initial-exploration goal; duplicate Start was rejected without another effect.

No Gazebo stack or robot motion was started.
