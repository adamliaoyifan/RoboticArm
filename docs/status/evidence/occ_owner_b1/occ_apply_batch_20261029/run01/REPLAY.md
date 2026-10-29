# ApplyBatch replay and identity

Baseline core: 7007b352; common canonical CLI adapter:57a893f5. A:c4407bc0. B:186cdf15. Exact commits/binary SHA in VERSIONS.json. No new worktree: builds use git archive snapshots, not mutable product source. Default observer.yaml unchanged. Shared fixture/config hashes are in each INPUT_MANIFEST.json and window manifest.

Build each archived package with `cmake -S <archive>/src/uld_occ_observer -B <build> -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release`, then `cmake --build <build> -j4` and `ctest --test-dir <build> --output-on-failure`. Exact archive commits listed in VERSIONS.json. ROS build adds `source /opt/ros/humble/setup.bash`, ON and Python3_EXECUTABLE/PYTHON_EXECUTABLE=/usr/bin/python3, target observer_node. Sanitizer Debug flags `-fsanitize=address,undefined -fno-omit-frame-pointer`, run observer_profile_checks and observer_keyset_checks with ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1.

Paired suite orchestration is replay_campaigns.sh. It invokes the permanent FusionBenchmarkSuite, not a separate worker. Each window preserves argv.json and manifest.yaml, including the full cumulative 33-frame input sequence and lossless input hashes. To replay a window, invoke argv.json with a NEW --out path; never overwrite samples.yaml. The CLI verifies every input SHA. Use state_digest=false in a copied manifest only for separately labeled RSS replay.

Raw and canonical hashes are both retained. Canonical validation checks parent consistency before lossless prune on a copy; it is outside timed integrate/query calls and may affect process peak RSS. No-digest memory campaign isolates that effect. Missing canonical/content data is not a pass.

All previous failure/overshoot windows remain archived. Formal O5/O6/O7 not evaluated.

Closeout test/comment commit: 4c8f1862ec357cb01616f4b04be636227461cf44. Measured runtime core unchanged from B. Campaign artifact times are preserved in TIMELINE.json; exact per-frame monotonic start stamps are in samples.yaml.
