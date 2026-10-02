#!/usr/bin/env bash
# ST09-PD evidence runner: focused suites, asset inventory with measured
# hashes, and source-vs-fresh-install resolution, all recorded under the
# evidence directory. Run after build_deploy_assets_campaign.sh and a
# fresh colcon build (default-base install).
# Usage: bash scripts/pd_evidence.sh <ws-root> <evidence-dir>
set -o pipefail
WS="$(cd "$1" && pwd)"
EV="$(cd "$2" && pwd)"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/local_setup.bash"
export PYTHONPATH="$WS/src/luggage_planning:$WS/src/luggage_description:${PYTHONPATH:-}"

mkdir -p "$EV"
STATUS=0

echo "== focused suites (with the ROS env; asset-gated tests active) =="
if python3 -m pytest tests/st09_parallel/PD -q \
    > "$EV/pytest_PD.log" 2>&1; then
  tail -2 "$EV/pytest_PD.log"
else
  STATUS=1; tail -30 "$EV/pytest_PD.log"
fi

for suite in src/luggage_description/test src/luggage_planning/test; do
  name="$(basename "$(dirname "$suite")")_$(basename "$suite")"
  if python3 -m pytest "$suite" -q > "$EV/pytest_$name.log" 2>&1; then
    tail -1 "$EV/pytest_$name.log"
  else
    STATUS=1; tail -30 "$EV/pytest_$name.log"
  fi
done

echo "== deploy graph structural validation report =="
python3 - "$EV" <<'PYEOF' > "$EV/graph_validation.json" 2> "$EV/graph_validation.err" || STATUS=1
import json, os, sys
sys.path.insert(0, "src/luggage_planning")
sys.path.insert(0, "src/luggage_description")
from luggage_planning.deploy_graph_assets import (load_deploy_graph,
                                                   validate_graph_structure)
data = "src/luggage_planning/data/reachability_atlas_deploy"
summary = json.load(open(os.path.join(data, "graphs", "summary.json")))
report = {"schema": "st09.pd-graph-validation/1", "graphs": []}
for entry in summary["graphs"]:
    graph = load_deploy_graph(os.path.join(data, "graphs",
                                           entry["graph_id"] + ".json"))
    result = validate_graph_structure(graph)
    result["graph_id"] = entry["graph_id"]
    result["digest"] = entry["digest"]
    report["graphs"].append(result)
print(json.dumps(report, indent=1, sort_keys=True))
if not all(g["ok"] for g in report["graphs"]):
    sys.exit(1)
PYEOF
[ -s "$EV/graph_validation.json" ] && echo "graph validation: $(python3 -c "
import json; d=json.load(open('$EV/graph_validation.json'));
print('; '.join('%s ok=%s nodes=%d edges=%d gaps=%d' % (g['graph_id'], g['ok'], g['node_count'], g['edge_count'], len(g['not_evaluated'])) for g in d['graphs']))")"

echo "== asset inventory with measured hashes =="
{
  md5sum src/luggage_description/config/scene_tf_deploy.yaml
  sha256sum src/luggage_description/config/S20_with_camera.srdf
  sha256sum src/luggage_description/config/deploy_asset_manifest.yaml
  sha256sum src/luggage_description/config/robot_poses.deploy.yaml
  sha256sum src/luggage_planning/data/reachability_atlas_deploy/robot_description.xml
  for f in src/luggage_planning/data/reachability_atlas_deploy/*.npz \
           src/luggage_planning/data/reachability_atlas_deploy/*.yaml \
           src/luggage_planning/data/reachability_atlas_deploy/graphs/*.json; do
    sha256sum "$f"
  done
} > "$EV/asset_inventory.sha256"

echo "== source vs fresh-install resolution =="
if python3 -m pytest tests/st09_parallel/PD/test_install_resolution.py -q \
    -p no:cacheprovider \
    > "$EV/pytest_install_resolution.log" 2>&1; then
  tail -1 "$EV/pytest_install_resolution.log"
else
  STATUS=1; tail -30 "$EV/pytest_install_resolution.log"
fi
ELFIN_PD_INSTALL_ROOT="${ELFIN_PD_INSTALL_ROOT:-$WS/install}" python3 -m pytest \
    tests/st09_parallel/PD/test_install_resolution.py -q \
    > "$EV/pytest_install_resolution_fresh.log" 2>&1 || STATUS=1
tail -1 "$EV/pytest_install_resolution_fresh.log"

echo "== hardware pose gating: unmeasured site poses stay non-executable =="
python3 - "$EV" > "$EV/site_pose_report.json" 2>&1 <<'PYEOF' || STATUS=1
import json, sys
sys.path.insert(0, "src/luggage_planning")
sys.path.insert(0, "src/luggage_description")
from luggage_planning import deploy_profile as dp
from luggage_planning.site_pose_schema import (executable_site_pose,
                                               load_site_poses,
                                               validate_site_poses)
record = dp.resolve_deploy_profile()
report = validate_site_poses(
    load_site_poses("src/luggage_description/config/robot_poses.deploy.yaml"),
    expected_scene_revision=record["scene_tf_hash"])
print(json.dumps({
    "schema": "st09.pd-site-poses/1",
    "file_ok": report["ok"],
    "issues": report["issues"],
    "executable_fallback": report["executable_fallback"],
    "poses": {kind: {"valid": entry["valid"], "outcome": entry["outcome"]}
              for kind, entry in report["poses"].items()},
    "hardware_calibration_prerequisite": (
        "measured site poses required before any hardware motion; "
        "absence is a prerequisite, not a validated pose"),
}, indent=1, sort_keys=True))
if not report["ok"] or any(executable_site_pose(report, kind) is not None
                           for kind in report["poses"]):
    sys.exit(1)
PYEOF

echo "pd_evidence status: $STATUS"
exit $STATUS
