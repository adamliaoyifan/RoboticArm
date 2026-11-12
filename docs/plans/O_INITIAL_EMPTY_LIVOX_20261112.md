# O initial empty declaration and independent LiDAR input

- status: approved
- dispatch_ready: yes
- parent: O-INITIAL-LIVOX-20261112
- subtask: SOURCE-INPUT
- generation: 1
- owner: codex/gpt-6
- base_revision: 36d9e62c4fe94ca5bf14d92752c01a14dc2a46df
- authorization: User requests source-owned initial empty state/provenance and independent Livox cloud integration into O.

## Implementation and boundaries

One native O owner/grid. Explicit once-per-process initial-empty declaration atomically starts a new epoch; reset never re-declares it. Keep exact normalized hull active mask; publish known free source blocks separately from the sensor probability tree so an initial prior never suppresses a real first hit. Record declaration ID, declaring authority, and ROS declaration time independently of sensor observation revision/time, including snapshots, checkpoints and operation-source binding. Consumers use the same pages. Preserve full static ULD admission and pickup observation; diagnostic driver declares initial empty instead of mandatory portal motion. No planned cargo commit, clearance change or simulator truth.

LiDAR is a distinct XYZ input to the existing source writer: finite, nonzero returns, stamped physical-frame TF to container_link, same existing ray fusion. No optical-Z filtering or invented optical frame. A separate stationary adapter keeps acquisition header/fields and uses the existing joint motion gate; motion scans remain refused until deskew is validated. Topic/frame are explicit launch parameters. Both modalities have separate sensor IDs/receipts. Same resolution, epoch, snapshots, validity and content-version checks.

Initial state provenance persists as history after observed updates, never renews sensor acquisition age. First initialized snapshot may have observed_revision=0/last_integrated_stamp_ns=0; its declaration is an initial condition, not stale camera evidence. Subsequent sensor snapshots retain normal age admission. Initial declaration can be issued only once in one owner lifetime; identical replay in its epoch is idempotent, conflict/reuse after reset refuses. Release/adoption remains the existing measured path; this task does not qualify a complete Gazebo box cycle or change release policy.

## Acceptance and verification

Environment ROS Humble production prefix; unrelated dirty sources preserved and hashed. All deterministic cases run once; AND over cases, no per-cycle reliability/speed claim. No Gazebo is needed for source/interface validation.

- Native C++ observer_initial_input_checks covers initial active free/inactive mask, provenance and zero sensor counters, first-hit override, invalid/conflicting/replayed declarations, reset/lease invalidation, checkpoint round trip, depth-vs-LiDAR negative-Z semantics, mixed-source integration, validity declaration versus observation publication. All checks return zero failures.
- Existing observer CTest core regressions run once; all selected tests pass. Invocation: ctest --test-dir <build>/uld_occ_observer --output-on-failure, excluding external acceptance campaigns.
- Python source tests: test_observer_initial_source.py, test_observer_lidar.py, test_operation_sim_driver.py, test_operation_launch.py and relevant existing observer-source/preparation tests. All cases pass; no new skip is accepted as exercised coverage.
- Native ROS integration: test_initial_lidar_native.py starts one actual observer in isolated domain178, calls declaration service, reads complete small-hull surface/cells, integrates a transformed LiDAR cloud and RGB-D cloud, verifies named source receipt, expected occupied endpoint, unchanged header and source snapshot provenance. Missing acquisition TF/wrong frame must not mutate content. Durable request/response CDR, inputs, logs and teardown required. All cases pass; no Gazebo/controller claim.
- scripts/production/build.sh --evidence <root>/build_N: all selected packages pass, no concurrent input changes; installed checker passes with exact source origins. Fresh evidence directories for repairs; archive failed runs before retry.

## Evidence

Root docs/status/evidence/o_initial_livox/20261112/. Manifest includes revision/dirty state, exact commands, input config hashes, stage boundaries, outcomes and missing coverage. Native failure capture includes full source inputs/TF, snapshot pages, receipts and node logs; <=256MiB/native run, <=1GiB total. No owned nodes left running. If capture is incomplete, repair instrumentation before another live attempt. Simulator truth is never an online input.
