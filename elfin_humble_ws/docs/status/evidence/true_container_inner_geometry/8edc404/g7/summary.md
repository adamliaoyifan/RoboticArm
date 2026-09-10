# TCIG-7 Gate G7 evidence

- tested_revision: `8edc4044e19844aa6815d961b118040325e44704`
- dirty_file_count: 0
- geometry_hash: `cb41f55f243786d50ab30997291083ab40467507abcf78a00e55da77ffc00856`
- outcome: pass

## Verification

- `python3 -m pytest src/luggage_description/test -q`: 143 passed.
- `python3 -m pytest src/luggage_planning/test -q`: 258 passed, 14 subtests passed.
- `colcon build --packages-select luggage_msgs luggage_description luggage_planning --symlink-install`: pass, 3 packages built.
- `colcon test --packages-select luggage_msgs luggage_description luggage_planning --event-handlers console_direct+`: pass.
- `colcon test-result --verbose`: 417 tests, 0 errors, 0 failures, 0 skipped.
- Four checked-in `s20_container_collision_aware*` atlas pairs load as schema v3 with the exact geometry hash above.
- `git diff --check`: pass.
- `scripts/check_agent_contract.sh`: pre-existing base/mailbox mismatch; four `OPEN.md` rows reference DSIM threads added only in the primary worktree and absent from this isolated base.

## Notes

No Gazebo or robot motion was started. The deterministic small-grid fixture verifies that hull-invalid cells never invoke the injected IK callback.

## Post-close hardening

- revision: `101e1c14507e121de57c2616f15ec24e2894dd5f`
- verification: planning suite 258 passed, 14 subtests passed; dirty file count 0 before this evidence update.
- change: distinguish active, allocated, and inactive atlas totals and reject incomplete or inconsistent payload metadata.
