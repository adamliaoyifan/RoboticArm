#!/usr/bin/env bash
# PF-R7 generation-4: one bounded live campaign after a clean same-commit overlay.
# Does not copy files into the overlay. Overlay HEAD must equal evaluator HEAD.
set -euo pipefail

PRIMARY="/home/adamliao/work/elfin_humble_ws"
OVERLAY="${OVERLAY:-/tmp/pfr10_g6}"
EVAL_SHA="$(git -C "$PRIMARY" rev-parse HEAD)"
EV="$PRIMARY/docs/status/evidence/platform_free_height/2026-09-15_pfr7_g4/rev_${EVAL_SHA}"
export ROS_DOMAIN_ID=7
export ELFIN_SIM_PIDFILE=/tmp/elfin_humble_sim.pid
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

mkdir -p "$EV/live"

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
  src/luggage_perception/test/eval/test_pf_r7_classifier.py \
  src/luggage_perception/test/eval/test_pf_r7_campaign.py \
  src/luggage_perception/test/eval/test_pf_r7_score_window.py \
  src/luggage_perception/test/eval/test_pf_r7_g4_steady_window.py \
  src/luggage_perception/test/eval/pf_r7_fixtures.py \
  src/luggage_perception/test/eval/data/pfr7_g3_carryon00_scores.jsonl \
  src/luggage_perception/test/test_platform_free_pipeline.py \
  | wc -l)"

{
  echo "evaluator_commit=$EVAL_SHA"
  echo "production_anchor=60dafb7deee50a6f3a76d48076b743bf3e3e1bc8"
  echo "overlay=$overlay_sha"
  echo "overlay_dirty=$overlay_dirty"
  echo "evaluator_dirty=$eval_dirty"
} | tee "$EV/revision.txt"

if [ "$overlay_sha" != "$EVAL_SHA" ]; then
  echo "overlay HEAD != evaluator HEAD; rebuild overlay from $EVAL_SHA" >&2
  exit 4
fi
if [ "$overlay_dirty" != "0" ] || [ "$eval_dirty" != "0" ]; then
  echo "dirty worktree; G4 live requires a clean same-commit overlay" >&2
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

echo "==== live G4 campaign ====" | tee "$EV/live/start.txt"
source_overlay
export PYTHONPATH="$PRIMARY/src/luggage_perception:${PYTHONPATH:-}"
set +e
python3 "$PRIMARY/scripts/pf_r7_bounded_acceptance.py" \
  --live \
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
set -e
"$PRIMARY/scripts/stop_sim.sh" | tee "$EV/live/stop_sim.log" || true
pgrep -af '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo' \
  | grep -v pgrep | tee "$EV/live/residual_launch.txt" || true
