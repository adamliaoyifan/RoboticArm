#!/usr/bin/env bash
# usage: run_suites.sh <outdir>   (G2 affected suites + B0/B1 unit gates; JUnit per suite)
set +u
source /tmp/elfin-master-pack-baseline-g3/scripts/g3/g3_env.sh
OUT="$1"; mkdir -p "$OUT"; cd /tmp/elfin-master-pack-baseline-g3
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
for suite in src/luggage_planning/test tests/st09_parallel/B tests/st09_parallel/C tests/st09_parallel/I \
             src/luggage_description/test \
             src/luggage_gazebo/test/test_baseline_enablement.py \
             src/luggage_gazebo/test/test_planning_request_capture.py \
             src/luggage_gazebo/test/test_commit_to_cargo_map.py \
             src/luggage_gazebo/test/test_handoff_driver_gate.py \
             src/luggage_gazebo/test/test_leg_admission_none.py \
             src/luggage_gazebo/test/test_retreat_start_contract.py \
             src/luggage_gazebo/test/test_m3_place_event_order.py; do
  name=$(echo "$suite" | tr '/' '_')
  mode=--import-mode=importlib; [ "$suite" = tests/st09_parallel/B ] && mode=--import-mode=prepend
  timeout 3000 python3 -m pytest -q -p no:cacheprovider $mode "$suite" \
    --junitxml="$OUT/$name.xml" > "$OUT/$name.log" 2>&1
  echo "$suite rc=$?" >> "$OUT/suite_rc.txt"
done
python3 scripts/g3/junit_ids.py "$OUT"
