#!/usr/bin/env bash
# OCC-FREESPACE-IMPLEMENT gen2 -- executable delivery commands.
# Contract: docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
# (plan_revision 8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451); gen1 candidate
# 6576650297518c35861349a702ce3869d10ff743; this generation repairs the
# review's five tool groups (2026-11-02_1112 review).
#
# Entries: build | tests | fmatrix | rehearsal | freeze | o9 | score | judge
# | replay | verify | budget. No entry silently continues into a dependent
# stage; wrong SHA / dirty tree / missing inputs refuse before any scoring.
set -eo pipefail
# -u off: ROS setup scripts read optional AMENT_* variables.

WORKTREE=/tmp/elfin-occ-freespace-impl-g2
CORE=/tmp/observer-fsi-g2-core
BUILD=/tmp/fsi-g2-build
INSTALL=/tmp/fsi-g2-install
EVIDENCE="$WORKTREE/docs/status/evidence/occ_freespace_final_20261102/implementation_g2"
FIXTURES=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures
GEOMETRY="$FIXTURES/geometry_chamfer.yaml"
CONFIG="$FIXTURES/config_chamfer_r0p02.yaml"
PERF_LOCK=/tmp/elfin-occ-backend-perf.lock
EXPECTED_BASE=6576650297518c35861349a702ce3869d10ff743
CAMPAIGN_PY="$WORKTREE/src/uld_occ_observer/test/benchmark/freespace_campaign.py"
EXECUTOR_PY="$WORKTREE/src/uld_occ_observer/test/benchmark/campaign_executor.py"

file_sha() { sha256sum "$1" | cut -d' ' -f1; }

require_clean_tree() {
  cd "$WORKTREE"
  if [ -n "$(git status --porcelain -- src/uld_occ_observer)" ]; then
    echo "REFUSING: uld_occ_observer tree is dirty" >&2
    exit 3
  fi
}

require_freeze() {
  # The freeze must exist and every recorded SHA must match the actual file
  # bytes on disk (self-reported descriptors never stand in for a hash).
  if [ ! -f "$EVIDENCE/FREEZE.json" ]; then
    echo "REFUSING: no FREEZE.json (run the freeze entry first)" >&2
    exit 3
  fi
  python3 - "$EVIDENCE/FREEZE.json" <<'PY'
import hashlib, json, sys
from pathlib import Path
freeze = json.loads(Path(sys.argv[1]).read_text())
problems = []
def sha(p):
    path = Path(p)
    if not path.is_file():
        problems.append("missing:%s" % p)
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()
for group in ("binaries", "tools", "fixtures", "inputs"):
    for name, expected in (freeze.get(group) or {}).items():
        path = expected["path"] if isinstance(expected, dict) else None
        if path is None:
            continue
        actual = sha(path)
        if actual is not None and actual != expected["sha256"]:
            problems.append("sha_mismatch:%s" % name)
if freeze.get("dirty"):
    problems.append("frozen_dirty_tree")
if problems:
    print("REFUSING: freeze verification failed: %s" % problems, file=sys.stderr)
    sys.exit(3)
PY
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
    "$CORE/observer_freespace_behavior"
    "$CORE/observer_repair_s1_behavior"
    source /opt/ros/humble/setup.bash
    source "$INSTALL/setup.bash"
    cd "$WORKTREE"
    python3 -m pytest -q --tb=line \
      src/uld_occ_observer/test/test_freespace_offline.py \
      src/uld_occ_observer/test/test_occ_surface_bridge.py \
      src/uld_occ_observer/test/test_freespace_g2_offline.py \
      src/uld_occ_observer/test/test_s2_gen10_offline.py \
      src/uld_occ_observer/test/test_repair_s0.py 2>&1 | tail -3
    echo "TESTS OK"
    ;;
  fmatrix)
    cd "$WORKTREE"
    python3 src/uld_occ_observer/test/benchmark/freespace_cases.py \
      --geometry "$GEOMETRY" --config "$CONFIG" --cli "$CORE/observer_cli" \
      --out "$EVIDENCE/f_matrix" 2>&1 | tail -2
    ;;
  rehearsal)
    # Non-scored short wiring (<=2h excluding lock waits). Same source as the
    # formal runner; short artifacts carry formal=false and cannot certify.
    source /opt/ros/humble/setup.bash
    source /home/adamliao/work/elfin_humble_ws/install/setup.bash
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
    OCC_FS_NODE="$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
      python3 -m pytest -q --tb=line -p no:cacheprovider \
      src/uld_occ_observer/test/test_occ_surface_bridge.py 2>&1 | tail -2
    python3 "$CAMPAIGN_PY" \
      --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
      --eval-native "$CORE/observer_eval_native" \
      --frames-glob "$FRAME_DIR/b*.yaml" \
      --out "$EVIDENCE/rehearsal/campaign" --stages p,r,l 2>&1 | tail -3
    ;;
  freeze)
    # Records the candidate identity: product head, every binary, the TOOLS
    # (python modules + static libs + typesupport), the consumer test, the
    # fixture/input files by their actual bytes, config, and this script.
    require_clean_tree
    cd "$WORKTREE"
    base=$(git merge-base HEAD "$EXPECTED_BASE" 2>/dev/null || true)
    head=$(git rev-parse HEAD)
    if [ "$base" != "$EXPECTED_BASE" ]; then
      echo "REFUSING: candidate does not descend from the gen1 candidate" >&2
      exit 3
    fi
    FRAME_DIR="$EVIDENCE/rehearsal/frames"
    if [ ! -d "$FRAME_DIR" ] || ! ls "$FRAME_DIR"/b*.yaml >/dev/null 2>&1; then
      echo "REFUSING: no input frames to freeze (run the rehearsal entry)" >&2
      exit 3
    fi
    python3 - "$head" "$FRAME_DIR" "$EVIDENCE" "$CORE" "$INSTALL" "$GEOMETRY" "$CONFIG" "$WORKTREE" <<'PY'
import hashlib, json, pathlib, subprocess, sys
head, frame_dir, evidence, core, install, geometry, config, work = sys.argv[1:9]
frame_dir, evidence, core, install, work = map(pathlib.Path, (frame_dir, evidence, core, install, work))
def entry(path):
    path = pathlib.Path(path)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
bench = work / "src/uld_occ_observer/test/benchmark"
typesupport = sorted(install.glob("uld_occ_observer/lib/libuld_occ_observer__rosidl_typesupport_cpp.so"))
if not typesupport:
    raise SystemExit("REFUSING: typesupport library missing from install")
frames = sorted(frame_dir.glob("b*.yaml"))
freeze = {
  "head": head, "base": sys.argv[1] and "6576650297518c35861349a702ce3869d10ff743",
  "plan_revision": "8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451",
  "binaries": {
    "observer_cli": entry(core / "observer_cli"),
    "observer_profile_cli": entry(core / "observer_profile_cli"),
    "observer_eval_native": entry(core / "observer_eval_native"),
    "observer_freespace_behavior": entry(core / "observer_freespace_behavior"),
    "observer_repair_s1_behavior": entry(core / "observer_repair_s1_behavior"),
    "observer_node": entry(install / "uld_occ_observer/lib/uld_occ_observer/observer_node"),
  },
  "tools": {
    "freespace_campaign.py": entry(bench / "freespace_campaign.py"),
    "freespace_judge.py": entry(bench / "freespace_judge.py"),
    "freespace_contract.py": entry(bench / "freespace_contract.py"),
    "freespace_cases.py": entry(bench / "freespace_cases.py"),
    "freespace_oracle.py": entry(bench / "freespace_oracle.py"),
    "campaign_executor.py": entry(bench / "campaign_executor.py"),
    "repair_s0_judge.py": entry(bench / "repair_s0_judge.py"),
    "s2_o9.py": entry(work / "src/uld_occ_observer/test/s2_o9.py"),
    "observer_core_static": entry(core / "libuld_occ_observer_core.a"),
    "observer_io_static": entry(core / "libuld_occ_observer_io.a"),
    "typesupport_cpp": entry(typesupport[0]),
    "consumer_test": entry(work / "src/uld_occ_observer/test/test_occ_surface_bridge.py"),
  },
  "fixtures": {
    "geometry": entry(geometry),
    "config": entry(config),
  },
  "inputs": {"frame_%s" % frame.name: entry(frame) for frame in frames},
  "commands": entry(evidence / "COMMANDS.sh"),
  "dirty": subprocess.run(["git", "status", "--porcelain", "--", "src/uld_occ_observer"],
                          cwd=work, capture_output=True, text=True).stdout.strip(),
}
(evidence / "FREEZE.json").write_text(json.dumps(freeze, indent=1, sort_keys=True))
print("FROZEN", head, "frames:", len(frames), "tools:", len(freeze["tools"]))
PY
    ;;
  o9)
    # Four-operation O9 on the frozen candidate. The freespace_full chain is
    # clocked from BEFORE the acquire (full chain: acquire -> all pages ->
    # release). s2_o9 takes the perf lock itself.
    require_freeze
    cd "$WORKTREE"
    python3 src/uld_occ_observer/test/s2_o9.py \
      --cli "$CORE/observer_profile_cli" \
      --eval-native "$CORE/observer_eval_native" \
      --geometry "$GEOMETRY" --config "$CONFIG" \
      --out "$EVIDENCE/o9" 2>&1 | tail -4
    ;;
  score)
    # The formal runner. CLOSED until the new S3 task binds this candidate
    # (OCC_FS_SCORE_OPEN=1). OCC_FS_SCORE_REHEARSAL=1 exercises the SAME
    # entry with bounded counts -- never a formal sample, never certified.
    if [ "${OCC_FS_SCORE_REHEARSAL:-0}" != "1" ] \
       && [ "${OCC_FS_SCORE_OPEN:-0}" != "1" ]; then
      echo "REFUSING: formal scoring opens with OCTOMAP-2CM-REPAIR-S3 gen3" \
           "(bind the frozen candidate, then export OCC_FS_SCORE_OPEN=1;" \
           "OCC_FS_SCORE_REHEARSAL=1 runs the bounded same-source path)" >&2
      exit 3
    fi
    require_freeze
    source /opt/ros/humble/setup.bash
    source /home/adamliao/work/elfin_humble_ws/install/setup.bash
    source "$INSTALL/setup.bash"
    cd "$WORKTREE"
    FRAME_DIR="$EVIDENCE/rehearsal/frames"
    if [ "${OCC_FS_SCORE_REHEARSAL:-0}" = "1" ]; then
      # Bounded path: same runner, same judges, reduced counts.
      python3 "$CAMPAIGN_PY" \
        --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
        --eval-native "$CORE/observer_eval_native" \
        --frames-glob "$FRAME_DIR/b*.yaml" \
        --out "$EVIDENCE/score_rehearsal/campaign" --stages p,r,l 2>&1 | tail -3
      # O8 through the established stage runner with bounded inputs.
      ULD_OCC_S2_ADAPTER=live ULD_OCC_S3_OPEN=1 ULD_OCC_S2_CONFIRM_OFFICIAL=1 \
        python3 "$EXECUTOR_PY" --mode stage --stage o8 --input-limit 3 \
          --cli "$CORE/observer_profile_cli" \
          --eval-native "$CORE/observer_eval_native" \
          --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
          --geometry "$GEOMETRY" --config "$CONFIG" \
          --out "$EVIDENCE/score_rehearsal/o8" 2>&1 | tail -3
      echo "SCORE REHEARSAL OK (non-scored, cannot certify)"
      exit 0
    fi
    # Formal path, contract order: P/H -> R -> L -> O8 -> O5 -> O6.
    python3 "$CAMPAIGN_PY" --formal \
      --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
      --eval-native "$CORE/observer_eval_native" \
      --frames-glob "$FRAME_DIR/b*.yaml" \
      --out "$EVIDENCE/scoring/campaign" --stages p,r,l 2>&1 | tail -3
    ULD_OCC_S2_ADAPTER=live ULD_OCC_S3_OPEN=1 ULD_OCC_S2_CONFIRM_OFFICIAL=1 \
      python3 "$EXECUTOR_PY" --mode stage --stage o8 \
        --cli "$CORE/observer_profile_cli" \
        --eval-native "$CORE/observer_eval_native" \
        --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
        --geometry "$GEOMETRY" --config "$CONFIG" \
        --out "$EVIDENCE/scoring/o8" 2>&1 | tail -3
    ULD_OCC_S2_ADAPTER=live ULD_OCC_S3_OPEN=1 ULD_OCC_S2_CONFIRM_OFFICIAL=1 \
      python3 "$EXECUTOR_PY" --mode stage --stage o5 \
        --cli "$CORE/observer_profile_cli" \
        --eval-native "$CORE/observer_eval_native" \
        --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
        --geometry "$GEOMETRY" --config "$CONFIG" \
        --out "$EVIDENCE/scoring/o5" 2>&1 | tail -3
    ULD_OCC_S2_ADAPTER=live ULD_OCC_S3_OPEN=1 ULD_OCC_S2_CONFIRM_OFFICIAL=1 \
      python3 "$EXECUTOR_PY" --mode stage --stage o6 \
        --cli "$CORE/observer_profile_cli" \
        --eval-native "$CORE/observer_eval_native" \
        --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
        --geometry "$GEOMETRY" --config "$CONFIG" \
        --out "$EVIDENCE/scoring/o6" 2>&1 | tail -3
    echo "FORMAL SCORE COMPLETE -- judge entry decides the verdicts"
    ;;
  judge)
    # Recompute from the artifact FILES. Summary flags are never trusted:
    # every F case is re-judged from its responses, every stage artifact is
    # re-judged from its JSON, and the O9 trace set is re-certified.
    cd "$WORKTREE"
    python3 - "$EVIDENCE" <<'PY'
import json, sys
from pathlib import Path
sys.path.insert(0, "src/uld_occ_observer/test/benchmark")
import yaml
import freespace_judge as judge
root = Path(sys.argv[1])
problems = []
matrix = root / "f_matrix"
manifest_path = matrix / "manifest.json"
if manifest_path.is_file():
    manifest = json.loads(manifest_path.read_text())
    cases = {case["case_id"]: case for case in manifest["cases"]}
    passed = 0
    for case_dir in sorted((matrix / "cases").iterdir()):
        case = cases.get(case_dir.name)
        responses_path = case_dir / "responses.yaml"
        if case is None or not responses_path.is_file():
            problems.append("f_case_incomplete:%s" % case_dir.name)
            continue
        verdict = judge.judge_f_case(case, yaml.safe_load(responses_path.read_text()),
                                     json.loads(manifest["descriptor_json"]),
                                     manifest["resolution"])
        if verdict["reasons"]:
            problems.append("f_case_fail:%s:%s" % (case_dir.name,
                                                   verdict["reasons"][:2]))
        else:
            passed += 1
    if passed != 360:
        problems.append("f_matrix_passed_%d" % passed)
else:
    problems.append("f_matrix_missing")
campaign_dir = root / "rehearsal" / "campaign"
stage_files = {"p": sorted(campaign_dir.glob("p_b*.json")),
               "r": [campaign_dir / "r.json"],
               "l": [campaign_dir / "l.json"]}
for name, files in stage_files.items():
    for path in files:
        if not path.is_file():
            problems.append("stage_missing:%s" % path.name)
            continue
        artifact = json.loads(path.read_text())
        judge_fn = {"p": judge.judge_p_window, "r": judge.judge_r_stage,
                    "l": judge.judge_l_stage}[name]
        reasons = judge_fn(artifact)["reasons"]
        allowed = [r for r in reasons if r == "short_sample_not_formal"]
        if reasons != allowed:
            problems.append("stage_fail:%s:%s" % (path.name, reasons[:3]))
o9 = root / "o9" / "o9_result.json"
if o9.is_file():
    result = json.loads(o9.read_text())
    # The verdict block is the authoritative judge output; the top-level
    # convenience flag is not the gate result.
    verdict = result.get("verdict") or {}
    if not verdict.get("performance_certified"):
        problems.append("o9_not_certified:%s" % verdict.get("reasons"))
else:
    problems.append("o9_result_missing")
print(json.dumps({"problems": problems[:20], "total": len(problems)}, indent=1))
sys.exit(0 if not problems else 1)
PY
    ;;
  replay)
    # Re-executes the product on persisted failure inputs. Exit codes are
    # preserved verbatim -- a replay failure fails this entry.
    cd "$WORKTREE"
    python3 src/uld_occ_observer/test/benchmark/repair_s0_replay.py \
      --root "$EVIDENCE/o9" 2>&1 | tail -5
    ;;
  verify)
    # Negative self-checks: the guards themselves must refuse. Each block
    # MUST fail; a guard that passes is the bug.
    cd "$WORKTREE"
    if OCC_FS_SCORE_OPEN=0 bash "$EVIDENCE/COMMANDS.sh" score >/dev/null 2>&1; then
      echo "BUG: score ran without authorization" >&2; exit 3
    fi
    if [ -f "$EVIDENCE/FREEZE.json" ]; then
      cp "$EVIDENCE/FREEZE.json" /tmp/freeze_backup_g2.json
      python3 - "$EVIDENCE/FREEZE.json" <<'PY'
import json, sys, pathlib
path = pathlib.Path(sys.argv[1])
freeze = json.loads(path.read_text())
freeze["binaries"]["observer_cli"]["sha256"] = "0" * 64
path.write_text(json.dumps(freeze, indent=1))
PY
      if bash "$EVIDENCE/COMMANDS.sh" o9 >/dev/null 2>&1; then
        cp /tmp/freeze_backup_g2.json "$EVIDENCE/FREEZE.json"
        echo "BUG: o9 ran against a tampered SHA" >&2; exit 3
      fi
      cp /tmp/freeze_backup_g2.json "$EVIDENCE/FREEZE.json"
    fi
    echo "VERIFY OK: guards refuse as required"
    ;;
  budget)
    cat <<'EOF'
gen2 implementation-phase budget (excludes perf-lock waits):
  build ~5 min | tests ~10 min | fmatrix ~12 min | rehearsal ~5 min
  o9 ~35 min (four operations, chain incl. acquire) | verify <1 min
  total ~67 min << 2 h rehearsal cap
Formal phase (score, closed): P 9x120 s ~20 min, R ~95 s, L 120 min,
O8 ~10 min, O5 ~5 min, O6 96 groups ~6-8 h -- the 8 h formal budget
prioritizes the new target gates; O6 is the declared budget casualty.
EOF
    ;;
  help|*)
    cat <<'EOF'
entries: build | tests | fmatrix | rehearsal | freeze | o9 | score | judge | replay | verify | budget
score is closed until the new S3 binds this candidate; OCC_FS_SCORE_REHEARSAL=1
runs the bounded same-source path (never certified).
EOF
    ;;
esac
