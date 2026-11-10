#!/bin/bash
# Frozen S2 entry. This script executes build, behavior, rehearsal, and O9.
# O5-O8, the judge campaign, and failure replay of a scored run are defined
# below and are not executed until S3 is opened.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
GEOM=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_cuboid.yaml
CFG=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_cuboid_r0p05.yaml
INSTALL=/tmp/s2-install
CLI="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_profile_cli"
NODE="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node"
EVIDENCE="$ROOT/docs/status/evidence/occ_octomap_repair_2cm_20261031/s2"

build() {
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  cmake -S "$ROOT/src/uld_occ_observer" -B /tmp/observer-s2-core -DOBSERVER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release
  cmake --build /tmp/observer-s2-core --target observer_repair_s1_behavior observer_profile_cli -j"$(nproc)"
  cd "$ROOT"
  colcon build --packages-select uld_occ_observer \
    --build-base /tmp/s2-build --install-base "$INSTALL" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
  # shellcheck disable=SC1091
  source "$INSTALL/setup.bash"
}

behavior() {
  /tmp/observer-s2-core/observer_repair_s1_behavior
  python3 -m pytest -q --tb=line "$ROOT/src/uld_occ_observer/test/test_repair_s0.py"
}

rehearsal() {
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$INSTALL/setup.bash"
  python3 "$ROOT/src/uld_occ_observer/test/s2_rehearsal.py" \
    --node "$NODE" --geometry "$GEOM" --config "$CFG" --out /tmp/s2-rehearsal
}

o9() {
  python3 "$ROOT/src/uld_occ_observer/test/s2_o9.py" \
    --cli "$CLI" --geometry "$GEOM" --config "$CFG" \
    --out /tmp/s2-o9-full --pairs 30
}

# Frozen for S3. Not called by this S2 entry. Official point-cloud bytes are
# still not_emitted; these commands name the measured CLI, geometry, and config.
o5() {
  python3 "$ROOT/src/uld_occ_observer/test/benchmark/fusion_benchmark.py" \
    --campaign baseline --candidate-cli "$CLI" --baseline-cli "$CLI" \
    --fixture-root "$EVIDENCE/o5_inputs" --geometry "$GEOM" --config "$CFG" \
    --out "$EVIDENCE/o5" --timeout-s 900 --budget-s 28800 \
    --lock-path /tmp/elfin-occ-backend-perf.lock --require-content
}
o6() {
  python3 "$ROOT/src/uld_occ_observer/test/benchmark/fusion_benchmark.py" \
    --campaign baseline --candidate-cli "$CLI" --baseline-cli "$CLI" \
    --fixture-root "$EVIDENCE/o6_inputs" --geometry "$GEOM" --config "$CFG" \
    --out "$EVIDENCE/o6" --timeout-s 900 --budget-s 28800 \
    --lock-path /tmp/elfin-occ-backend-perf.lock --require-content
}
o7() {
  python3 "$ROOT/src/uld_occ_observer/test/benchmark/fusion_benchmark.py" \
    --campaign cold --candidate-cli "$CLI" --baseline-cli "$CLI" \
    --fixture-root "$EVIDENCE/o7_inputs" --geometry "$GEOM" --config "$CFG" \
    --out "$EVIDENCE/o7" --timeout-s 900 --budget-s 28800 \
    --lock-path /tmp/elfin-occ-backend-perf.lock --require-content
}
o8() {
  python3 "$ROOT/src/uld_occ_observer/test/benchmark/fusion_benchmark.py" \
    --campaign memory --candidate-cli "$CLI" \
    --fixture-root "$EVIDENCE/o8_inputs" --geometry "$GEOM" --config "$CFG" \
    --out "$EVIDENCE/o8" --timeout-s 900 --budget-s 28800 \
    --lock-path /tmp/elfin-occ-backend-perf.lock
}
judge() {
  python3 -m pytest -q --tb=line "$ROOT/src/uld_occ_observer/test/test_repair_s0.py"
}
replay() {
  python3 - <<PY
import json, sys
sys.path.insert(0, "$ROOT/src/uld_occ_observer/test/benchmark")
import repair_s0_judge as judge
ops = json.load(open("$EVIDENCE/o9_pairs.json"))
verdict = judge.judge_case({"gate": "o9", "operations": ops})
print(verdict["status"], verdict.get("reasons"))
raise SystemExit(0 if verdict.get("pass") else 1)
PY
}

build
behavior
rehearsal
o9
echo "s2 stopped after o9; o5 o6 o7 o8 judge replay are frozen and not executed"
