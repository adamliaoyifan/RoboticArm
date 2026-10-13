#!/usr/bin/env bash
# usage: run_suites.sh <outdir>   (affected suites; JUnit per suite)
set +u
source /tmp/elfin-master-pack-baseline-g2/scripts/g2/g2_env.sh
OUT="$1"; mkdir -p "$OUT"; cd /tmp/elfin-master-pack-baseline-g2
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
for suite in src/luggage_planning/test tests/st09_parallel/B tests/st09_parallel/C tests/st09_parallel/I; do
  name=$(echo "$suite" | tr '/' '_')
  mode=--import-mode=importlib; [ "$suite" = tests/st09_parallel/B ] && mode=--import-mode=prepend
  timeout 3000 python3 -m pytest -q -p no:cacheprovider $mode "$suite" \
    --junitxml="$OUT/$name.xml" > "$OUT/$name.log" 2>&1
  echo "$suite rc=$?" >> "$OUT/suite_rc.txt"
done
