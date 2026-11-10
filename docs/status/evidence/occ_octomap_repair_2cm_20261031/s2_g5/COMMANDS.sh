#!/bin/bash
# S2 gen5. Default check does not score O5-O8 and does not start another O9.
# Scheduler d5a309e1. O9 remains the 7537b033 measurement. S3 stays closed.
set -euo pipefail
ROOT=/tmp/elfin-occ-octomap-repair-2cm-g1
CODE=d5a309e1071ae54047fc91c6e587d36e33f73ba9
O9_CODE=7537b033daf20561c247bb7e09b6ee8805358a93
EXECUTOR_SHA=1d2427fbf88dde97913bf85220f510527cea73af61dcfb01a190a2509dc1e19d
G5_TEST_SHA=b76cfbb6d815a20632fca06a2eb2ef2cf84362c611582d3c96e50adf7dc3f07a
PRUNE_SHA=1300812a5a2d37b7fd22d8deab02145e9b0b11c114fb5db5e253b056103872b0

blob_sha() {
  git -C "$ROOT" show "$CODE:$1" | sha256sum | awk '{print $1}'
}

check() {
  test "$(blob_sha src/uld_occ_observer/test/benchmark/campaign_executor.py)" = "$EXECUTOR_SHA"
  test "$(blob_sha src/uld_occ_observer/test/test_s2_g5_offline.py)" = "$G5_TEST_SHA"
  test "$(blob_sha src/uld_occ_observer/test/apply_batch_checks.cpp)" = "$PRUNE_SHA"
  git -C "$ROOT" diff --quiet "$O9_CODE" -- src/uld_occ_observer/src src/uld_occ_observer/include src/uld_occ_observer/CMakeLists.txt
  python3 -m pytest -q --tb=line \
    "$ROOT/src/uld_occ_observer/test/test_s2_g3_offline.py" \
    "$ROOT/src/uld_occ_observer/test/test_s2_g4_offline.py" \
    "$ROOT/src/uld_occ_observer/test/test_s2_g5_offline.py" \
    -k "not materialize_and_live"
}

case "${1:-check}" in
  check) check ;;
  o5|o6|o7|o8|all)
    python3 "$ROOT/src/uld_occ_observer/test/benchmark/campaign_executor.py" \
      --mode stage --stage "$1" --out /tmp/s2-g5-stage-refused
    ;;
  judge)
    python3 "$ROOT/src/uld_occ_observer/test/benchmark/campaign_executor.py" \
      --mode judge --judge-file "${2:?judge file}" --out /tmp/s2-g5-judge
    ;;
  replay)
    python3 "$ROOT/src/uld_occ_observer/test/benchmark/campaign_executor.py" \
      --mode replay --bundle "${2:?bundle}" --out "${3:-/tmp/s2-g5-replay}"
    ;;
  s3) echo "S3 is closed; official O5-O8 samples are not started" >&2; exit 2 ;;
  *) echo "unknown stage $1" >&2; exit 2 ;;
esac
