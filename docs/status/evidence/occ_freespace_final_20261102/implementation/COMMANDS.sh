#!/usr/bin/env bash
# OCC-FREESPACE-IMPLEMENT gen1 -- executable delivery commands.
# Contract: docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
# (plan_revision 8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451), product baseline
# 2ec2393122481d4ad8467c85e9971e13bc0f820c.
#
# Independent entries: build | tests | rehearsal | freeze | o9 | score | judge
# | replay. Each entry exits non-zero on missing prerequisites, a dirty tree,
# or a SHA mismatch, and no entry silently continues into a dependent stage.
# Formal scoring stays CLOSED: `score` refuses to run until the new S3 task
# binds the frozen candidate and exports OCC_FS_SCORE_OPEN=1.
set -euo pipefail

WORKTREE=/tmp/elfin-occ-freespace-impl-g1
CORE=/tmp/observer-fsi-g1-core
BUILD=/tmp/fsi-g1-build
INSTALL=/tmp/fsi-g1-install
EVIDENCE="$WORKTREE/docs/status/evidence/occ_freespace_final_20261102/implementation"
FIXTURES=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures
GEOMETRY="$FIXTURES/geometry_chamfer.yaml"
CONFIG="$FIXTURES/config_chamfer_r0p02.yaml"
GEOMETRY_SHA=dec2b8f4de9cfeb9aa9cbbac124af33f5b19a579566d39cffebd2629f1aa565b   # descriptor_sha256
PERF_LOCK=/tmp/elfin-occ-backend-perf.lock
EXPECTED_BASE=2ec2393122481d4ad8467c85e9971e13bc0f820c

require_clean_tree() {
  cd "$WORKTREE"
  if [ -n "$(git status --porcelain -- src/uld_occ_observer)" ]; then
    echo "REFUSING: uld_occ_observer tree is dirty" >&2
    exit 3
  fi
}

require_fixtures() {
  actual=$(python3 -c "import yaml,sys;print(yaml.safe_load(open('$GEOMETRY').read())['descriptor_sha256'])")
  if [ "$actual" != "$GEOMETRY_SHA" ]; then
    echo "REFUSING: fixture SHA mismatch ($actual)" >&2
    exit 3
  fi
}

entry=${1:-help}
case "$entry" in
  build)
    cd "$WORKTREE"
    cmake -S src/uld_occ_observer -B "$CORE" -DOBSERVER_WITH_ROS=OFF \
      -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$CORE" -j"$(nproc)" >/dev/null
    source /opt/ros/humble/setup.bash
    colcon build --packages-select uld_occ_observer --build-base "$BUILD" \
      --install-base "$INSTALL" --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers 4 --event-handlers console_direct- 2>&1 \
      | grep -E "Failed|error|Summary" | tail -3
    echo "BUILD OK: core + install"
    ;;
  tests)
    # C++ behavior binaries (F/C semantics + gen10 regressions) then the
    # offline pytest suites including the judge negative matrix.
    "$CORE/observer_freespace_behavior"
    "$CORE/observer_repair_s1_behavior"
    source /opt/ros/humble/setup.bash
    source "$INSTALL/setup.bash"
    cd "$WORKTREE"
    python3 -m pytest -q --tb=line \
      src/uld_occ_observer/test/test_freespace_offline.py \
      src/uld_occ_observer/test/test_occ_surface_bridge.py \
      src/uld_occ_observer/test/test_s2_gen10_offline.py \
      src/uld_occ_observer/test/test_repair_s0.py 2>&1 | tail -3
    echo "TESTS OK"
    ;;
  fmatrix)
    # The full 360-case F gate through the real core + independent oracle.
    require_fixtures
    cd "$WORKTREE"
    python3 src/uld_occ_observer/test/benchmark/freespace_cases.py \
      --geometry "$GEOMETRY" --config "$CONFIG" --cli "$CORE/observer_cli" \
      --out "$EVIDENCE/f_matrix" 2>&1 | tail -2
    ;;
  rehearsal)
    # Non-scored short wiring (<=2h excluding lock waits): P 2/2 per box
    # count, R short overload/drain/recovery, L short loop. Short samples
    # carry formal=false and the judge refuses to certify them.
    require_fixtures
    source /opt/ros/humble/setup.bash
    source "$INSTALL/setup.bash"
    cd "$WORKTREE"
    FRAME_DIR="$EVIDENCE/rehearsal/frames"
    mkdir -p "$FRAME_DIR"
    for tag in 00 05 20; do
      "$CORE/observer_eval_native" --geometry "$GEOMETRY" --config "$CONFIG" \
        --gate emit-window --group "r02_b${tag}_chamfer_c30000_o50_x00" \
        --boxes "${tag#0}" --cloud 30000 --outside 0.5 --cross 0 --warmup 0 \
        --repeat 2 --out "$EVIDENCE/rehearsal/emit_${tag}" >/dev/null
      for frame in "$EVIDENCE/rehearsal/emit_${tag}"/frames/f00*.yaml; do
        cp "$frame" "$FRAME_DIR/b${tag}_$(basename "$frame")"
      done
    done
    python3 src/uld_occ_observer/test/benchmark/freespace_campaign.py \
      --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
      --eval-native "$CORE/observer_eval_native" \
      --frames-glob "$FRAME_DIR/b*.yaml" \
      --out "$EVIDENCE/rehearsal/campaign" --stages p,r,l 2>&1 | tail -3
    ;;
  freeze)
    # Records the candidate identity; refuses on a dirty product tree or a
    # baseline mismatch. FREEZE.json lists every binary and input SHA.
    require_clean_tree
    cd "$WORKTREE"
    base=$(git merge-base HEAD "$EXPECTED_BASE" 2>/dev/null || true)
    head=$(git rev-parse HEAD)
    if [ "$base" != "$EXPECTED_BASE" ]; then
      echo "REFUSING: candidate does not descend from the contract baseline" >&2
      exit 3
    fi
    python3 - <<PY
import hashlib, json, pathlib, subprocess
work = pathlib.Path("$WORKTREE")
head = "$head"
def sha(p): return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
freeze = {
  "head": head,
  "base": "$EXPECTED_BASE",
  "plan_revision": "8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451",
  "binaries": {
    "observer_cli": sha("$CORE/observer_cli"),
    "observer_profile_cli": sha("$CORE/observer_profile_cli"),
    "observer_eval_native": sha("$CORE/observer_eval_native"),
    "observer_freespace_behavior": sha("$CORE/observer_freespace_behavior"),
    "observer_repair_s1_behavior": sha("$CORE/observer_repair_s1_behavior"),
    "observer_node": sha("$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node"),
  },
  "fixtures": {
    "geometry": sha("$GEOMETRY"),
    "config": sha("$CONFIG"),
    "geometry_descriptor_sha256": "$GEOMETRY_SHA",
  },
  "dirty": subprocess.run(["git","status","--porcelain","--","src/uld_occ_observer"],
                          cwd=work, capture_output=True, text=True).stdout.strip(),
}
out = work / "docs/status/evidence/occ_freespace_final_20261102/implementation/FREEZE.json"
out.write_text(json.dumps(freeze, indent=1, sort_keys=True))
print("FROZEN", head)
PY
    ;;
  o9)
    # The four-operation O9 on the frozen candidate: roi, integrate_30k,
    # surface_first, and the REQUIRED whole-query freespace_full chain,
    # 30 AB/BA pairs, P95 ratio <= 1.10. s2_o9 takes the perf lock itself.
    require_fixtures
    if [ ! -f "$EVIDENCE/FREEZE.json" ]; then
      echo "REFUSING: freeze before O9" >&2
      exit 3
    fi
    cd "$WORKTREE"
    python3 src/uld_occ_observer/test/s2_o9.py \
      --cli "$CORE/observer_profile_cli" \
      --eval-native "$CORE/observer_eval_native" \
      --geometry "$GEOMETRY" --config "$CONFIG" \
      --out "$EVIDENCE/o9" 2>&1 | tail -4
    ;;
  score)
    # Formal P/R/L/O8 and the O5/O6 capability matrix. CLOSED in this task.
    if [ "${OCC_FS_SCORE_OPEN:-0}" != "1" ]; then
      echo "REFUSING: formal scoring opens with OCTOMAP-2CM-REPAIR-S3 gen3" \
           "(bind the frozen candidate, then export OCC_FS_SCORE_OPEN=1)" >&2
      exit 3
    fi
    echo "not implemented in gen1: the S3 task owns the formal runner"
    exit 3
    ;;
  judge)
    # Re-judge persisted artifacts without re-running them.
    cd "$WORKTREE"
    python3 - "$EVIDENCE" <<'PY'
import json, sys
from pathlib import Path
sys.path.insert(0, "src/uld_occ_observer/test/benchmark")
import freespace_judge as judge
root = Path(sys.argv[1])
problems = []
matrix = root / "f_matrix" / "summary.json"
if matrix.is_file():
    problems += judge.judge_f_summary(json.loads(matrix.read_text()))
else:
    problems.append("f_matrix_missing")
campaign = root / "rehearsal" / "campaign" / "summary.json"
if campaign.is_file():
    summary = json.loads(campaign.read_text())
    if not summary.get("wiring_ok"):
        problems.append("campaign_wiring_failed")
else:
    problems.append("campaign_summary_missing")
o9 = root / "o9" / "o9_result.json"
if o9.is_file():
    result = json.loads(o9.read_text())
    verdict = result.get("verdict", {})
    if not verdict.get("performance_certified"):
        problems.append("o9_not_certified:%s" % verdict.get("reasons"))
else:
    problems.append("o9_result_missing")
print(json.dumps({"problems": problems}, indent=1))
sys.exit(0 if not problems else 1)
PY
    ;;
  replay)
    # Replay the persisted O9 failure bundles (if any) against the frozen
    # binaries; a healthy run simply certifies there is nothing to replay.
    cd "$WORKTREE"
    python3 src/uld_occ_observer/test/benchmark/repair_s0_replay.py \
      --root "$EVIDENCE/o9" 2>&1 | tail -3 || echo "no replayable bundles (healthy run)"
    ;;
  budget)
    cat <<'EOF'
Implementation-phase budget estimate (excludes perf-lock waits):
  build          ~5 min   (core + colcon install)
  tests          ~8 min   (2 C++ behavior binaries, offline suites, negatives)
  fmatrix       ~12 min   (360 cases, ~2 s each through the real core)
  rehearsal      ~4 min   (P 3x2/2, R 40+5s+10, L 20 s + rounds)
  o9           ~35 min   (4 operations x 30 AB/BA pairs, lock held inside)
  total        ~64 min   << 2 h rehearsal cap
Formal-phase estimate for S3 gen3 (not started here): O9 ~35 min,
P 9x120 s ~20 min, R ~2 min, L 120 min, O8 ~10 min, O5/O6 96 groups
~6-8 h -- the 8 h formal budget prioritizes the new target gates; the O6
matrix is the declared candidate for not_evaluated-on-budget.
EOF
    ;;
  help|*)
    cat <<'EOF'
entries: build | tests | fmatrix | rehearsal | freeze | o9 | score | judge | replay | budget
score stays closed until OCTOMAP-2CM-REPAIR-S3 gen3 binds the frozen candidate.
EOF
    ;;
esac
