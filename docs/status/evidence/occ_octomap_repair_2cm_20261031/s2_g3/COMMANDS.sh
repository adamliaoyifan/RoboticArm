#!/bin/bash
# S2 gen3 frozen entry. Default runs checks and does not start O5-O8 or a new O9.
# Product C++ is 16dda822. Replay and campaign tools are d653c9c8.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
CPP=16dda82286b345b0a6f4208ac9c3f43621b8d8fd
TOOLS=d653c9c84f2c47dc0daa0046ac57f26b30a9bb3b
GEOM=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_chamfer.yaml
CFG=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_chamfer_r0p02.yaml
FIXTURES=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures
INSTALL=/tmp/s2-install
CLI="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_profile_cli"
NATIVE="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_eval_native"
NODE="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node"
CLI_SHA=19703076fb0c2fca86e5ee1b2ee6ee87423b528f633a24ac94f5cfac75388146
NODE_SHA=35716c796a762c2d74c35e959861e158078364dee07b896e499b1b5139738637

cd "$ROOT"
git diff --quiet "$CPP" -- src/uld_occ_observer/src src/uld_occ_observer/include src/uld_occ_observer/CMakeLists.txt
git diff --quiet "$TOOLS" -- src/uld_occ_observer/test/s2_o9.py src/uld_occ_observer/test/s2_rehearsal.py \
  src/uld_occ_observer/test/benchmark/capture_replay.py src/uld_occ_observer/test/benchmark/campaign_entry.py \
  src/uld_occ_observer/test/test_s2_g3_offline.py
test "$(sha256sum "$CLI" | awk '{print $1}')" = "$CLI_SHA"
test "$(sha256sum "$NODE" | awk '{print $1}')" = "$NODE_SHA"
git status --short -- src/uld_occ_observer/src src/uld_occ_observer/include src/uld_occ_observer/CMakeLists.txt | grep -q . && exit 1 || true

check() {
  python3 -m pytest -q --tb=line "$ROOT/src/uld_occ_observer/test/test_s2_g3_offline.py"
  python3 - <<PY
import json, sys
sys.path.insert(0, "$ROOT/src/uld_occ_observer/test/benchmark")
import campaign_entry
plan = campaign_entry.load_plan("$FIXTURES")
o5 = campaign_entry.resolve_stage(plan, "o5")
o7 = campaign_entry.resolve_stage(plan, "o7")
assert o5["groups"] == 4 and o5["roi_cells"] == 4096 and o5["resolution_m"] == [0.02]
assert o7["runner"] == "ros_windows" and o7["normal_windows"] == 9
assert o7["overload_inputs"] == 600 and o7["recovery_inputs"] == 60
stopped = campaign_entry.continue_after({"o5": "fail"})
assert stopped["continue_o6"] is False and stopped["continue_o7"] is False
print("stages o5=%s o7=%s o5_fail_stops_o6_o7" % (o5["groups"], o7["runner"]))
PY
}

rehearsal() {
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$INSTALL/setup.bash"
  python3 "$ROOT/src/uld_occ_observer/test/s2_rehearsal.py" \
    --node "$NODE" --geometry "$GEOM" --config "$CFG" --out /tmp/s2-g3-rehearsal
}

o9() {
  python3 "$ROOT/src/uld_occ_observer/test/s2_o9.py" \
    --cli "$CLI" --eval-native "$NATIVE" --geometry "$GEOM" --config "$CFG" \
    --out /tmp/s2-o9-g3 --pairs 30
}

# Frozen S3 bodies. This default entry does not call them.
o5() { python3 -c 'import sys; sys.path.insert(0,"'"$ROOT"'/src/uld_occ_observer/test/benchmark"); import campaign_entry; print(campaign_entry.resolve_stage(campaign_entry.load_plan("'"$FIXTURES"'"), "o5"))'; }
o6() { python3 -c 'import sys; sys.path.insert(0,"'"$ROOT"'/src/uld_occ_observer/test/benchmark"); import campaign_entry; print(campaign_entry.resolve_stage(campaign_entry.load_plan("'"$FIXTURES"'"), "o6"))'; }
o7() { python3 -c 'import sys; sys.path.insert(0,"'"$ROOT"'/src/uld_occ_observer/test/benchmark"); import campaign_entry; print(campaign_entry.resolve_stage(campaign_entry.load_plan("'"$FIXTURES"'"), "o7"))'; }
o8() { python3 -c 'import sys; sys.path.insert(0,"'"$ROOT"'/src/uld_occ_observer/test/benchmark"); import campaign_entry; print(campaign_entry.resolve_stage(campaign_entry.load_plan("'"$FIXTURES"'"), "o8"))'; }

case "${1:-check}" in
  check) check ;;
  rehearsal) rehearsal ;;
  o9) o9 ;;
  s3) echo "S3 is closed; o5 o6 o7 o8 are not started" >&2; exit 2 ;;
  *) echo "unknown stage $1" >&2; exit 2 ;;
esac
