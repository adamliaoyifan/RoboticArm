#!/bin/bash
# S2 gen4 frozen entry. Default check does not score O5-O8 or start another O9.
# Candidate 7537b033. Resource stays NOT_EVALUATED. S3 stays closed.
set -euo pipefail
ROOT=/tmp/elfin-occ-octomap-repair-2cm-g1
CODE=7537b033daf20561c247bb7e09b6ee8805358a93
GEOM=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_chamfer.yaml
CFG=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_chamfer_r0p02.yaml
FIXTURES=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures
INSTALL=/tmp/s2-install
CLI="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_profile_cli"
NATIVE="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_eval_native"
NODE="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node"
CLI_SHA=8e668c95956ae97cfe4a2f44fd9f4ebfb5b31e649745426f066361275ebfa025
NODE_SHA=d486f3791c61e76b3f1f3ca2c6f2be02029c9379ce717de2fc59f8ba2f23d8d8
NATIVE_SHA=e87408f3986d53a8c9998669c0a2cd7d10ee34b3d5a8f5b3ee232a30fc8354d2
EXECUTOR_SHA=edc79ba1766da9aeda65e66aaba29fd70f2df8c196419c48a968a606b2ea8201
REPLAY_SHA=3d4d81beab3b6e44c711effd1d70a95b29270269cd3d9c76778d6b0f12232f80
ENTRY_SHA=02149603f3233bc9a36dcae96a9934be793d07b4a337481c75ab86252acc7c25
JUDGE_SHA=b4a4dd56f4134a50a45adf16fabe45f8589dc7fac61a757ea1a9db5c6abdba1d

blob_sha() {
  git -C "$ROOT" show "$CODE:$1" | sha256sum | awk '{print $1}'
}

check() {
  test "$(blob_sha src/uld_occ_observer/test/benchmark/campaign_executor.py)" = "$EXECUTOR_SHA"
  test "$(blob_sha src/uld_occ_observer/test/benchmark/capture_replay.py)" = "$REPLAY_SHA"
  test "$(blob_sha src/uld_occ_observer/test/benchmark/campaign_entry.py)" = "$ENTRY_SHA"
  test "$(blob_sha src/uld_occ_observer/test/benchmark/repair_s0_judge.py)" = "$JUDGE_SHA"
  test "$(sha256sum "$CLI" | awk '{print $1}')" = "$CLI_SHA"
  test "$(sha256sum "$NODE" | awk '{print $1}')" = "$NODE_SHA"
  test "$(sha256sum "$NATIVE" | awk '{print $1}')" = "$NATIVE_SHA"
  git -C "$ROOT" diff --quiet "$CODE" -- \
    src/uld_occ_observer/src src/uld_occ_observer/include src/uld_occ_observer/CMakeLists.txt \
    src/uld_occ_observer/test/benchmark/campaign_executor.py \
    src/uld_occ_observer/test/benchmark/capture_replay.py \
    src/uld_occ_observer/test/benchmark/campaign_entry.py \
    src/uld_occ_observer/test/benchmark/repair_s0_judge.py \
    src/uld_occ_observer/test/s2_o9.py src/uld_occ_observer/test/test_s2_g4_offline.py
  python3 -m pytest -q --tb=line \
    "$ROOT/src/uld_occ_observer/test/test_s2_g3_offline.py" \
    "$ROOT/src/uld_occ_observer/test/test_s2_g4_offline.py" \
    -k "not materialize_and_live"
  python3 - <<PY
import sys
sys.path.insert(0, "$ROOT/src/uld_occ_observer/test/benchmark")
import campaign_executor
import campaign_entry
plan = campaign_entry.load_plan("$FIXTURES")
result = campaign_executor.schedule(plan, campaign_executor.OfflineAdapter(), "/tmp/s2-g4-check-proof")
assert result["formal_pass"] is False
assert result["official_samples"] == "not_run"
assert result["product_resource_certification"] == "NOT_EVALUATED"
assert result["started"] == {"o5": 132, "o6": 3168, "o7": 2820, "o8": 108}
assert all(item["judge"]["pass"] for item in result["stages"])
closed = campaign_executor.schedule(plan, campaign_executor.OfflineAdapter(), "/tmp/s2-g4-check-s3", scored=True)
assert closed["reason"] == "s3_closed" and closed["started"]["o5"] == 0
print("offline-proof started", result["started"])
PY
}

stage() {
  python3 "$ROOT/src/uld_occ_observer/test/benchmark/campaign_executor.py" \
    --mode stage --stage "$1" --out /tmp/s2-g4-stage-refused
}

judge() {
  python3 "$ROOT/src/uld_occ_observer/test/benchmark/campaign_executor.py" \
    --mode judge --judge-file "$1" --out /tmp/s2-g4-judge
}

replay() {
  python3 "$ROOT/src/uld_occ_observer/test/benchmark/campaign_executor.py" \
    --mode replay --bundle "$1" --out "${2:-/tmp/s2-g4-replay}"
}

o9() {
  python3 "$ROOT/src/uld_occ_observer/test/s2_o9.py" \
    --cli "$CLI" --eval-native "$NATIVE" --geometry "$GEOM" --config "$CFG" \
    --out /tmp/s2-o9-g4 --pairs 30
}

case "${1:-check}" in
  check) check ;;
  o5|o6|o7|o8|all) stage "$1" ;;
  judge) judge "${2:?judge file}" ;;
  replay) replay "${2:?bundle}" "${3:-/tmp/s2-g4-replay}" ;;
  o9) o9 ;;
  s3) echo "S3 is closed; official O5-O8 samples are not started" >&2; exit 2 ;;
  *) echo "unknown stage $1" >&2; exit 2 ;;
esac
