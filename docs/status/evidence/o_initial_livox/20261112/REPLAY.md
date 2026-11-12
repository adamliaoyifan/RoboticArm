# Source/input verification replay

Code f03b27366137b4ad875630b4bd708909a157d597; plan5febcde5.
Tests use the exact shared dirty-tree package inputs hashed by build_receipt/manifest.json.
This is not an isolated clean-checkout/full-simulation success claim. source_snapshot.zip
retains effective changed files, including parallel working changes excluded from commit.

From workspace root, build with scripts/production/build.sh --evidence <fresh-dir>,
then run scripts/production/run.sh /usr/bin/python3 scripts/production/check_install.py
--output <fresh-file>. Required source tests and exact outcomes are in python_required.xml.
Source test runner is scripts/production/test_source.py; use the file list in the plan
plus test_placement_volume_evidence.py, test_observer_depth.py, test_frame_contracts.py.

Native source/interface replay:

```bash
O_INITIAL_NATIVE_EVIDENCE=/tmp/o_initial_replay_fresh ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 scripts/production/run.sh /usr/bin/python3 scripts/production/test_source.py -q docs/status/evidence/o_initial_livox/20261112/test_initial_lidar_native.py
```

It runs fresh O owners in domains178 and179, freezes its harness/config, writes
typed CDR requests/responses/point inputs/TF and stops both owners. Expect2passes,
known-free initial cells with source2 and sensor counter/time0, transformed LiDAR
occupied cell[1,2,1] with source1, mixed camera update, frame/TF rejection, and reset
refusing declaration reuse. Manifests require exit0/executor_joined=true.

Native C++: cmake -S src/uld_occ_observer -B /tmp/o_initial_release_fresh
-DCMAKE_BUILD_TYPE=Release -DOBSERVER_WITH_ROS=OFF -DBUILD_TESTING=ON;
cmake --build that directory; ctest --test-dir that directory --timeout300
--output-on-failure. Expect9checks pass. Release is required by the existing
hot-path disassembly gate. The existing free-space test takes about90s here.

Earlier failures remain durable. native_01 was a camera test using a disabled
service in subscription mode, and its early manifest is incomplete. Final runs
use actual subscriptions and complete capture. legacy_baseline/replay.py shows
3old solve_placement-monkeypatch failures against the base-revision node; those
are excluded from required-pass counts, not silently repaired or counted as skips.

No Gazebo, controller execution, actual sensor calibration, moving-scan deskew,
new self segmentation or CustomMsg conversion is certified by these tests.
Current Mid-360S launch defaults to PointCloud2; historical CustomMsg bags need
conversion/configuration rather than type/frame relabelling.
