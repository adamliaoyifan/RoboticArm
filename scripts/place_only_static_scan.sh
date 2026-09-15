#!/usr/bin/env bash
# POS-1 static isolation scan (plan acceptance item 5).
#
# Proves, relative to the execution base revision:
#   1. production planning/orchestration files gained no Gazebo/eval-truth
#      imports (the pre-existing sim-boundary vacuum adapters are the
#      designed Gazebo edge and are unchanged);
#   2. the eval-only mode is not a production default (sim_world defaults
#      keep use_perception=true; the place-only profile sets it false).
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE="${POS_BASE_REVISION:-88ae2db}"
status=0

echo "== 1. no NEW Gazebo/eval-truth imports in production planning/packing"
added=$(cd "$WS" && git diff "$BASE" -- \
    src/luggage_planning src/luggage_packing src/luggage_bringup \
    src/luggage_msgs \
  | grep -E '^\+' \
  | grep -v -E '^\+\+\+' \
  | grep -E -e 'from luggage_gazebo' -e 'import luggage_gazebo' \
      -e 'ros_gz_interfaces' -e 'gazebo_msgs' -e 'ign gazebo' -e 'gz sim' \
      -e 'place_only_fixture' -e 'place_only_eval_driver' \
  || true)
if [[ -n "$added" ]]; then
  echo "VIOLATIONS (added lines):"; echo "$added"; status=1
else
  echo "clean: no new Gazebo/eval-truth imports vs $BASE"
fi

echo "== 2. eval-only mode is not a production default"
defaults=$(python3 - "$WS/src/luggage_gazebo/launch/sim_world.launch.py" <<'PY'
import re
import sys
text = open(sys.argv[1], encoding="utf-8").read()
block = re.search(r"def _default_launch_values\(.*?\n    \}\n", text, re.S)
values = dict(re.findall(r'"([a-z_]+)": "([^"]*)"', block.group(0)))
print(values.get("use_perception"), values.get("use_semantic"))
PY
)
if [[ "$defaults" == "true false" ]]; then
  echo "clean: production defaults keep use_perception=true"
else
  echo "VIOLATION: unexpected sim_world defaults use_perception/use_semantic: $defaults"
  status=1
fi

profile="$WS/src/luggage_gazebo/config/place_only_profile.yaml"
if grep -q 'use_perception: false' "$profile"; then
  echo "clean: place-only profile disables perception explicitly"
else
  echo "VIOLATION: $profile missing use_perception: false"
  status=1
fi

exit $status
