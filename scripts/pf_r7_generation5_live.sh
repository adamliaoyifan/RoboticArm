#!/usr/bin/env bash
# PF-R7 generation-5: standard-seed scan, then one bounded live campaign.
# Overlay HEAD must equal evaluator HEAD. Worktree evaluator paths must be clean.
set -euo pipefail

PRIMARY="$(git rev-parse --show-toplevel)"
OVERLAY="${OVERLAY:-/tmp/pfr10_g6}"
MODE="${1:-scan}"
EVAL_SHA="$(git -C "$PRIMARY" rev-parse HEAD)"
EV="$PRIMARY/docs/status/evidence/platform_free_height/2026-09-15_pfr7_g5/rev_${EVAL_SHA}"
export ROS_DOMAIN_ID=7
export ELFIN_SIM_PIDFILE=/tmp/elfin_humble_sim.pid
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

mkdir -p "$EV/scan" "$EV/live"

overlay_sha="$(git -C "$OVERLAY" rev-parse HEAD)"
overlay_dirty="$(git -C "$OVERLAY" status --porcelain --untracked-files=all | wc -l)"
eval_dirty="$(git -C "$PRIMARY" status --porcelain --untracked-files=all -- \
  scripts/pf_r7_bounded_acceptance.py \
  scripts/pf_r7_live_backend.py \
  scripts/platform_free_height_gate4_eval.py \
  src/luggage_perception/luggage_perception/eval/pf_r7_classifier.py \
  src/luggage_perception/luggage_perception/eval/pf_r7_campaign.py \
  src/luggage_perception/luggage_perception/eval/gate4_scoring.py \
  src/luggage_gazebo/scripts/pickup_box_spawner_node.py \
  src/luggage_msgs/msg/DetectionFrame.msg \
  src/luggage_perception/scripts/luggage_detector_node.py \
  src/luggage_perception/luggage_perception/platform_free_pipeline.py \
  src/luggage_perception/luggage_perception/top_support_estimator.py \
  scripts/pf_r7_generation3_live.sh \
  scripts/pf_r7_generation4_live.sh \
  scripts/pf_r7_generation5_live.sh \
  src/luggage_perception/test/eval/test_pf_r7_classifier.py \
  src/luggage_perception/test/eval/test_pf_r7_campaign.py \
  src/luggage_perception/test/eval/test_pf_r7_score_window.py \
  src/luggage_perception/test/eval/test_pf_r7_g4_steady_window.py \
  src/luggage_perception/test/eval/test_pf_r7_g5_scan.py \
  src/luggage_perception/test/eval/pf_r7_fixtures.py \
  src/luggage_perception/test/eval/data/pfr7_g3_carryon00_scores.jsonl \
  src/luggage_perception/test/eval/data/pfr7_g4_standard00_scores.jsonl \
  src/luggage_perception/test/test_platform_free_pipeline.py \
  | wc -l)"

{
  echo "evaluator_commit=$EVAL_SHA"
  echo "production_anchor=60dafb7deee50a6f3a76d48076b743bf3e3e1bc8"
  echo "overlay=$overlay_sha"
  echo "overlay_dirty=$overlay_dirty"
  echo "evaluator_dirty=$eval_dirty"
  echo "mode=$MODE"
} | tee "$EV/revision.txt"

if [ "$overlay_sha" != "$EVAL_SHA" ]; then
  echo "overlay HEAD != evaluator HEAD; rebuild overlay from $EVAL_SHA" >&2
  exit 4
fi
if [ "$overlay_dirty" != "0" ] || [ "$eval_dirty" != "0" ]; then
  echo "dirty worktree; G5 live requires a clean same-commit overlay" >&2
  exit 4
fi

source_overlay() {
  unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$OVERLAY/install/setup.bash"
  set -u
  export PYTHONPATH="$PRIMARY/src/luggage_perception:${PYTHONPATH:-}"
  export ROS_DOMAIN_ID=7
  export ELFIN_SIM_PIDFILE=/tmp/elfin_humble_sim.pid
}

slot_busy() {
  pgrep -f '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' >/dev/null && return 0
  pgrep -f 'scripts/place_only_run.sh' >/dev/null && return 0
  pgrep -f 'place_only_eval_driver.py' >/dev/null && return 0
  return 1
}

if slot_busy; then
  echo "sim slot busy; refusing second world" | tee "$EV/wait_slot.log"
  exit 2
fi

echo "==== cleanup leftovers ====" | tee "$EV/cleanup.log"
"$PRIMARY/scripts/stop_sim.sh" | tee -a "$EV/cleanup.log" || true
sleep 5
if slot_busy; then
  echo "slot reoccupied during cleanup; abort" | tee -a "$EV/cleanup.log"
  exit 2
fi

source_overlay
export PYTHONPATH="$PRIMARY/src/luggage_perception:${PYTHONPATH:-}"
set +e
if [ "$MODE" = "scan" ]; then
  echo "==== G5 standard-seed scan ====" | tee "$EV/scan/start.txt"
  python3 "$PRIMARY/scripts/pf_r7_bounded_acceptance.py" \
    --live --scan-standard --import-g4 \
    --scan-start standard_06 --stop-available 6 --max-scan 24 \
    --out "$EV/scan" \
    --steady-start support-window-ready --steady-window-sec 8.0 \
    --recovery-limit-sec 1.4 --observe-sec 5 \
    --ros-domain-id 7 \
    --overlay "$OVERLAY" \
    --production-commit 60dafb7deee50a6f3a76d48076b743bf3e3e1bc8 \
    --git-commit "$EVAL_SHA" \
    --pidfile "$ELFIN_SIM_PIDFILE"
  echo "scan_exit=$?" | tee "$EV/scan/exit.txt"
elif [ "$MODE" = "live" ]; then
  MATRIX="$EV/scan/live_seed_matrix.json"
  if [ ! -f "$MATRIX" ]; then
    echo "missing $MATRIX; run scan first" >&2
    exit 4
  fi
  echo "==== G5 live campaign ====" | tee "$EV/live/start.txt"
  python3 "$PRIMARY/scripts/pf_r7_bounded_acceptance.py" \
    --live \
    --seed-matrix-json "$MATRIX" \
    --out "$EV/live" \
    --slots 3 --eligible-per-size 2 \
    --max-attempts-per-slot 12 --max-exclusions-per-slot 6 \
    --max-consecutive-size-exclusions 3 --max-stack-resets 2 \
    --max-attempts-campaign 36 --campaign-timeout-sec 2700 \
    --steady-start support-window-ready --steady-window-sec 8.0 \
    --recovery-limit-sec 1.4 --observe-sec 20 \
    --ros-domain-id 7 \
    --overlay "$OVERLAY" \
    --production-commit 60dafb7deee50a6f3a76d48076b743bf3e3e1bc8 \
    --git-commit "$EVAL_SHA" \
    --pidfile "$ELFIN_SIM_PIDFILE"
  echo "live_exit=$?" | tee "$EV/live/exit.txt"
else
  echo "usage: $0 scan|live" >&2
  exit 2
fi
set -e
"$PRIMARY/scripts/stop_sim.sh" | tee "$EV/${MODE}/stop_sim.log" || true
pgrep -af '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' \
  | grep -v pgrep | tee "$EV/${MODE}/residual_launch.txt" || true
