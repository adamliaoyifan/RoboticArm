#!/usr/bin/env bash
# OCC-FREESPACE-IMPLEMENT gen3 -- executable delivery commands.
# Contract: docs/plans/OCC_SURFACE_FREESPACE_FINAL_DELIVERY_20261101.md
# (plan_revision 8caa0d44a3c6ce3b8adec02c4bb7bf1dd612e451); gen2 candidate
# ca28c885ce435dbda0453cf99ab0cc21e11c0b62; this generation closes the 1621
# review's four groups A-D (per-record judges, real scheduling/lease
# closure, freeze integrity + disk judge + failure capture, matrix tests).
#
# Entries: build | tests | fmatrix | rehearsal | freeze | o9 | score | judge
# | replay | verify | budget. No entry silently continues into a dependent
# stage; wrong SHA / moved HEAD / dirty tree / missing inputs refuse before
# any scoring, and the judge entry re-judges every artifact from its FILES.
set -eo pipefail
# -u off: ROS setup scripts read optional AMENT_* variables.

WORKTREE=/tmp/elfin-occ-freespace-impl-g3
CORE=/tmp/observer-fsi-g3-core
BUILD=/tmp/fsi-g3-build
INSTALL=/tmp/fsi-g3-install
EVIDENCE="$WORKTREE/docs/status/evidence/occ_freespace_final_20261102/implementation_g3"
FIXTURES=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures
GEOMETRY="$FIXTURES/geometry_chamfer.yaml"
CONFIG="$FIXTURES/config_chamfer_r0p02.yaml"
EXPECTED_BASE=ca28c885ce435dbda0453cf99ab0cc21e11c0b62
CAMPAIGN_PY="$WORKTREE/src/uld_occ_observer/test/benchmark/freespace_campaign.py"
EXECUTOR_PY="$WORKTREE/src/uld_occ_observer/test/benchmark/campaign_executor.py"
THIS_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/COMMANDS.sh"

file_sha() { sha256sum "$1" | cut -d' ' -f1; }

require_clean_tree() {
  cd "$WORKTREE"
  if [ -n "$(git status --porcelain -- src/uld_occ_observer)" ]; then
    echo "REFUSING: uld_occ_observer tree is dirty" >&2
    exit 3
  fi
}

require_freeze() {
  # The freeze must exist, every group must be NON-EMPTY, every recorded SHA
  # must match the actual file bytes (self-reported descriptors never stand
  # in), the commands entry must match THIS script, and the worktree HEAD and
  # cleanliness are re-measured now -- freeze.dirty is a record, not proof.
  if [ ! -f "$EVIDENCE/FREEZE.json" ]; then
    echo "REFUSING: no FREEZE.json (run the freeze entry first)" >&2
    exit 3
  fi
  python3 - "$EVIDENCE/FREEZE.json" "$WORKTREE" "$THIS_SCRIPT" <<'PY'
import hashlib, json, subprocess, sys
from pathlib import Path
freeze = json.loads(Path(sys.argv[1]).read_text())
work, script = Path(sys.argv[2]), Path(sys.argv[3])
problems = []


def sha(path):
    path = Path(path)
    if not path.is_file():
        problems.append("missing:%s" % path)
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


for group in ("binaries", "tools", "fixtures", "inputs"):
    entries = freeze.get(group) or {}
    if not entries:
        problems.append("missing_group:%s" % group)
    for name, expected in entries.items():
        path = expected.get("path") if isinstance(expected, dict) else None
        if path is None:
            problems.append("entry_without_path:%s:%s" % (group, name))
            continue
        actual = sha(path)
        if actual is not None and actual != expected.get("sha256"):
            problems.append("sha_mismatch:%s:%s" % (group, name))
# commands is ONE entry (this script itself), not a name -> entry map.
commands = freeze.get("commands")
if not (isinstance(commands, dict) and commands.get("path")):
    problems.append("missing_group:commands")
else:
    if Path(commands["path"]) != script:
        problems.append("commands_path_is_not_this_script")
    actual = sha(script)
    if actual is not None and actual != commands.get("sha256"):
        problems.append("commands_sha_mismatch")
head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=work,
                      capture_output=True, text=True).stdout.strip()
if head != freeze.get("head"):
    # The freeze precedes its own evidence commit, so HEAD may advance --
    # but ONLY through the evidence path. Any product, tool, or command
    # change after the candidate is a moved candidate and refuses.
    diff = subprocess.run(["git", "diff", "--name-only",
                           str(freeze.get("head")), head], cwd=work,
                          capture_output=True, text=True)
    if diff.returncode != 0:
        problems.append("head_unresolvable:%s" % str(freeze.get("head"))[:12])
    else:
        outside = [name for name in diff.stdout.split()
                   if not name.startswith("docs/status/evidence/")]
        if outside:
            problems.append("head_moved:%s!=%s:%s"
                            % (head[:12], str(freeze.get("head"))[:12],
                               outside[:3]))
dirty = subprocess.run(["git", "status", "--porcelain", "--",
                        "src/uld_occ_observer"], cwd=work,
                       capture_output=True, text=True).stdout.strip()
if dirty:
    problems.append("tree_dirty_now")
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
      src/uld_occ_observer/test/test_freespace_g3_offline.py \
      src/uld_occ_observer/test/test_s2_gen10_offline.py \
      src/uld_occ_observer/test/test_repair_s0.py 2>&1 | tail -3
    python3 "$EVIDENCE/audit_baseline.py" >/dev/null
    echo "TESTS OK (incl. audit replay: controls green, 8 mutants rejected)"
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
    # A stage failure STOPS the dependents inside the runner, and the budget
    # is enforced by the scheduler (--budget-minutes), not by text.
    source /opt/ros/humble/setup.bash
    source /home/adamliao/work/elfin_humble_ws/install/setup.bash
    source "$INSTALL/setup.bash"
    cd "$WORKTREE"
    FRAME_DIR="$EVIDENCE/rehearsal/frames"
    rm -rf "$EVIDENCE/rehearsal/emit_"* "$FRAME_DIR"
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
      --out "$EVIDENCE/rehearsal/campaign" --stages p,r,l \
      --budget-minutes 20 2>&1 | tail -3
    ;;
  freeze)
    # Records the candidate identity in SEPARATELY BOUND groups: product
    # binaries, tools (python modules + static libs + typesupport + the
    # consumer AND matrix tests), fixtures and every input frame by their
    # actual bytes, and this script itself. HEAD and dirty state are
    # recorded AND re-measured by require_freeze at every later entry.
    require_clean_tree
    cd "$WORKTREE"
    base=$(git merge-base HEAD "$EXPECTED_BASE" 2>/dev/null || true)
    head=$(git rev-parse HEAD)
    if [ "$base" != "$EXPECTED_BASE" ]; then
      echo "REFUSING: candidate does not descend from the gen2 candidate" >&2
      exit 3
    fi
    FRAME_DIR="$EVIDENCE/rehearsal/frames"
    if [ ! -d "$FRAME_DIR" ] || ! ls "$FRAME_DIR"/b*.yaml >/dev/null 2>&1; then
      echo "REFUSING: no input frames to freeze (run the rehearsal entry)" >&2
      exit 3
    fi
    python3 - "$head" "$FRAME_DIR" "$EVIDENCE" "$CORE" "$INSTALL" "$GEOMETRY" "$CONFIG" "$WORKTREE" "$THIS_SCRIPT" <<'PY'
import hashlib, json, pathlib, subprocess, sys
head, frame_dir, evidence, core, install, geometry, config, work, script = sys.argv[1:10]
frame_dir, evidence, core, install, work = map(pathlib.Path,
    (frame_dir, evidence, core, install, work))
def entry(path):
    path = pathlib.Path(path)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
bench = work / "src/uld_occ_observer/test/benchmark"
tests = work / "src/uld_occ_observer/test"
typesupport = sorted(install.glob("uld_occ_observer/lib/libuld_occ_observer__rosidl_typesupport_cpp.so"))
if not typesupport:
    raise SystemExit("REFUSING: typesupport library missing from install")
frames = sorted(frame_dir.glob("b*.yaml"))
freeze = {
  "head": head, "base": "ca28c885ce435dbda0453cf99ab0cc21e11c0b62",
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
    "s2_o9.py": entry(tests / "s2_o9.py"),
    "observer_core_static": entry(core / "libuld_occ_observer_core.a"),
    "observer_io_static": entry(core / "libuld_occ_observer_io.a"),
    "typesupport_cpp": entry(typesupport[0]),
    "consumer_test": entry(tests / "test_occ_surface_bridge.py"),
    "g2_fault_tests": entry(tests / "test_freespace_g2_offline.py"),
    "g3_matrix_tests": entry(tests / "test_freespace_g3_offline.py"),
    "audit_replay": entry(evidence / "audit_baseline.py"),
  },
  "fixtures": {
    "geometry": entry(geometry),
    "config": entry(config),
  },
  "inputs": {"frame_%s" % frame.name: entry(frame) for frame in frames},
  "commands": entry(script),
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
    # The budget is enforced by the runner's scheduler; a failed stage stops
    # its dependents and the failure prefix is persisted as failure.json.
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
      # Bounded path: same runner, same judges, reduced counts, budget bound.
      python3 "$CAMPAIGN_PY" \
        --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
        --eval-native "$CORE/observer_eval_native" \
        --frames-glob "$FRAME_DIR/b*.yaml" \
        --out "$EVIDENCE/score_rehearsal/campaign" --stages p,r,l \
        --budget-minutes 20 2>&1 | tail -3
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
    # Formal path, contract order: P/H -> R -> L -> O8 -> O5 -> O6, inside
    # the 8 h formal budget (L's 120 min soak dominates; the scheduler
    # enforces the remainder between stages).
    python3 "$CAMPAIGN_PY" --formal \
      --node "$INSTALL/uld_occ_observer/lib/uld_occ_observer/observer_node" \
      --eval-native "$CORE/observer_eval_native" \
      --frames-glob "$FRAME_DIR/b*.yaml" \
      --out "$EVIDENCE/scoring/campaign" --stages p,r,l \
      --budget-minutes 420 2>&1 | tail -3
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
    # Recompute EVERYTHING from the artifact FILES. Summary flags are never
    # trusted; the P windows are checked as a SET (9 distinct ids for
    # scoring), stage artifacts are re-judged from JSON in BOTH the
    # rehearsal and scoring directories, failure prefixes are findings, and
    # O9 is re-judged from the raw per-pair samples.yaml -- the stored
    # ratios and verdict must match the recomputation.
    # Optional second argument: an alternate evidence root (verify entry).
    cd "$WORKTREE"
    python3 - "$EVIDENCE" "${2:-}" <<'PY'
import json, sys
from pathlib import Path
sys.path.insert(0, "src/uld_occ_observer/test/benchmark")
import yaml
import freespace_contract as contract
import freespace_judge as judge
# argv[1] is this evidence root; argv[2] (optional) overrides it -- the
# verify entry points the SAME code at a tampered copy.
root = Path(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] \
    else Path(sys.argv[1])
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
        verdict = judge.judge_f_case(
            case, yaml.safe_load(responses_path.read_text()),
            json.loads(manifest["descriptor_json"]), manifest["resolution"])
        if verdict["reasons"]:
            problems.append("f_case_fail:%s:%s" % (case_dir.name,
                                                   verdict["reasons"][:2]))
        else:
            passed += 1
    if passed != 360:
        problems.append("f_matrix_passed_%d" % passed)
else:
    problems.append("f_matrix_missing")

for label, formal in (("rehearsal", False), ("scoring", True)):
    campaign_dir = root / label / "campaign"
    if not campaign_dir.is_dir():
        if label == "rehearsal":
            problems.append("rehearsal_campaign_missing")
        continue
    if (campaign_dir / "failure.json").is_file():
        problems.append("%s_failure_prefix_present" % label)
    p_files = sorted(campaign_dir.glob("p_b*_w*.json"))
    if not p_files:
        problems.append("%s_p_windows_missing" % label)
    windows = [json.loads(path.read_text()) for path in p_files]
    if formal:
        problems.extend(judge.judge_p_matrix(windows))
    else:
        for artifact in windows:
            reasons = judge.judge_p_window(artifact)["reasons"]
            if reasons != ["short_sample_not_formal"]:
                problems.append("rehearsal_p_fail:%s:%s"
                                % (artifact.get("window_id"), reasons[:3]))
        # A rehearsal runs one window per box count: deleting any window
        # drops a box count from the set and is a finding.
        boxes = {artifact.get("box_count") for artifact in windows}
        if boxes != set(contract.P_BOX_COUNTS):
            problems.append("rehearsal_p_box_coverage:%s" % sorted(boxes, key=str))
    for name, judge_fn in (("r", judge.judge_r_stage),
                           ("l", judge.judge_l_stage)):
        path = campaign_dir / ("%s.json" % name)
        if not path.is_file():
            problems.append("%s_%s_stage_missing" % (label, name))
            continue
        reasons = judge_fn(json.loads(path.read_text()))["reasons"]
        allowed = [] if formal else ["short_sample_not_formal"]
        if formal:
            if reasons:
                problems.append("scoring_%s_fail:%s" % (name, reasons[:3]))
        elif reasons != allowed:
            problems.append("rehearsal_%s_fail:%s" % (name, reasons[:3]))

o9_dir = root / "o9"
o9_result_path = o9_dir / "o9_result.json"
if o9_result_path.is_file():
    result = json.loads(o9_result_path.read_text())
    pairs = []
    index = 0
    while True:
        sides = {}
        ok = True
        for side in ("on", "off"):
            path = o9_dir / ("pair_%02d_%s" % (index, side)) / "samples.yaml"
            if not path.is_file():
                ok = False
                break
            document = next(yaml.safe_load_all(path.read_text()))
            doc_problems = judge.o9_document_problems(document)
            if doc_problems:
                problems.append("o9_sample_%02d_%s:%s" % (index, side,
                                                          doc_problems[:2]))
                ok = False
                break
            first = document["surface_first_call"]
            sides[side] = {
                "roi": float(document["roi_ms"]),
                "integrate_30k": float(document["integrate_external_ms"]),
                "surface_first": float(first["external_ms"]),
                "freespace_full": float(document["freespace_full_external_ms"]),
            }
            sides["freespace_" + side] = document["freespace_content_sha256"]
            sides["content_" + side] = document["roi_surface_content_sha256"]
        if not ok:
            break
        pairs.append(sides)
        index += 1
    problems.extend(judge.judge_o9_samples(pairs, result))
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
    # Negative self-checks: every guard must REFUSE and the disk judge must
    # catch tampering. Each block MUST fail; a guard that passes is the bug.
    cd "$WORKTREE"
    if OCC_FS_SCORE_OPEN=0 bash "$THIS_SCRIPT" score >/dev/null 2>&1; then
      echo "BUG: score ran without authorization" >&2; exit 3
    fi
    if [ -f "$EVIDENCE/FREEZE.json" ]; then
      cp "$EVIDENCE/FREEZE.json" /tmp/freeze_backup_g3.json
      # tamper 1: a frozen binary SHA
      python3 - "$EVIDENCE/FREEZE.json" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
freeze = json.loads(path.read_text())
freeze["binaries"]["observer_cli"]["sha256"] = "0" * 64
path.write_text(json.dumps(freeze, indent=1))
PY
      if bash "$THIS_SCRIPT" o9 >/dev/null 2>&1; then
        cp /tmp/freeze_backup_g3.json "$EVIDENCE/FREEZE.json"
        echo "BUG: o9 ran against a tampered binary SHA" >&2; exit 3
      fi
      # tamper 2: the commands SHA (a judge-entry edit after the freeze)
      cp /tmp/freeze_backup_g3.json "$EVIDENCE/FREEZE.json"
      python3 - "$EVIDENCE/FREEZE.json" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
freeze = json.loads(path.read_text())
freeze["commands"]["sha256"] = "1" * 64
path.write_text(json.dumps(freeze, indent=1))
PY
      if bash "$THIS_SCRIPT" o9 >/dev/null 2>&1; then
        cp /tmp/freeze_backup_g3.json "$EVIDENCE/FREEZE.json"
        echo "BUG: o9 ran with a tampered commands SHA" >&2; exit 3
      fi
      # tamper 3: a moved HEAD recorded in the freeze
      cp /tmp/freeze_backup_g3.json "$EVIDENCE/FREEZE.json"
      python3 - "$EVIDENCE/FREEZE.json" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
freeze = json.loads(path.read_text())
freeze["head"] = "0" * 40
path.write_text(json.dumps(freeze, indent=1))
PY
      if bash "$THIS_SCRIPT" o9 >/dev/null 2>&1; then
        cp /tmp/freeze_backup_g3.json "$EVIDENCE/FREEZE.json"
        echo "BUG: o9 ran against a moved HEAD" >&2; exit 3
      fi
      cp /tmp/freeze_backup_g3.json "$EVIDENCE/FREEZE.json"
    fi
    # tamper 4: a deleted P window + a forged summary + a tampered O9 verdict
    if [ -d "$EVIDENCE/rehearsal/campaign" ]; then
      rm -rf /tmp/g3_verify_root
      mkdir -p /tmp/g3_verify_root
      cp -r "$EVIDENCE/rehearsal" /tmp/g3_verify_root/
      cp -r "$EVIDENCE/o9" /tmp/g3_verify_root/
      cp -r "$EVIDENCE/f_matrix" /tmp/g3_verify_root/
      window=$(ls /tmp/g3_verify_root/rehearsal/campaign/p_b*_w*.json | head -1)
      rm "$window"
      python3 - <<'PY'
import json, pathlib, sys
path = pathlib.Path("/tmp/g3_verify_root/rehearsal/campaign/summary.json")
if path.is_file():
    summary = json.loads(path.read_text())
    summary["wiring_ok"] = True
    for value in summary.get("stages", {}).values():
        if isinstance(value, dict):
            value["reasons"] = []
    path.write_text(json.dumps(summary, indent=1))
o9 = pathlib.Path("/tmp/g3_verify_root/o9/o9_result.json")
result = json.loads(o9.read_text())
result["ratios"]["roi"] = 0.5  # a lied ratio the disk judge must catch
o9.write_text(json.dumps(result, indent=1))
PY
      if bash "$THIS_SCRIPT" judge /tmp/g3_verify_root >/dev/null 2>&1; then
        echo "BUG: disk judge accepted a deleted window + forged summary + lied ratio" >&2
        exit 3
      fi
      rm -rf /tmp/g3_verify_root
    fi
    echo "VERIFY OK: guards refuse, disk judge catches tampering"
    ;;
  budget)
    cat <<'EOF'
gen3 implementation-phase budget (excludes perf-lock waits):
  build ~5 min | tests ~12 min | fmatrix ~12 min | rehearsal ~6 min
  o9 ~35 min (four operations, chain incl. acquire) | verify <1 min
  total ~71 min << 2 h rehearsal cap
Formal phase (score, closed): the runner enforces --budget-minutes 420
between stages (scheduler-enforced, not text); P 9x120 s ~20 min,
R ~95 s, L 120 min, O8 ~10 min, O5 ~5 min -- O6 stays the declared budget
casualty as in gen2.
EOF
    ;;
  help|*)
    cat <<'EOF'
entries: build | tests | fmatrix | rehearsal | freeze | o9 | score | judge [root] | replay | verify | budget
score is closed until the new S3 binds this candidate; OCC_FS_SCORE_REHEARSAL=1
runs the bounded same-source path (never certified). judge re-judges every
artifact from disk (9-window set, stage JSONs, O9 from raw per-pair samples);
verify negative-tests the guards AND the disk judge against tampering.
EOF
    ;;
esac
