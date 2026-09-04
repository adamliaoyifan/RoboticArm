#!/usr/bin/env bash
# Stop a luggage_gazebo sim_world stack the same way a Ctrl+C on the launch
# terminal does, then reap leftovers by PID.
#
#   SIGINT  ros2 launch            # rclcpp signal_handler, not SIGKILL
#   wait    WAIT_INT seconds       # gz GUI / MoveIt teardown is slow
#   SIGTERM leftover PIDs          # ign gazebo is often reparented to init
#   wait    WAIT_TERM seconds
#   SIGKILL last resort
#
# Never `pkill -f "ign gazebo"`: that string matches the calling shell and
# can kill the stopper. Eval failures belong in dumps/, not a live GUI.
#
# Usage:
#   scripts/stop_sim.sh           # pidfile, else a single luggage_gazebo launch
#   scripts/stop_sim.sh --all     # every luggage_gazebo launch + leftover stack
#   ELFIN_SIM_PIDFILE=/tmp/foo.pid scripts/stop_sim.sh
set -euo pipefail

PIDFILE="${ELFIN_SIM_PIDFILE:-/tmp/elfin_humble_sim.pid}"
WAIT_INT="${WAIT_INT:-12}"
WAIT_TERM="${WAIT_TERM:-4}"
ALL=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all) ALL=1; shift ;;
    --pidfile) PIDFILE="$2"; shift 2 ;;
    --wait) WAIT_INT="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,18p' "$0"
      exit 0
      ;;
    *)
      echo "unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

alive() {
  local pid
  for pid in "$@"; do
    [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null && return 0
  done
  return 1
}

wait_gone() {
  local timeout="$1"
  shift
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    alive "$@" || return 0
    sleep 0.2
  done
  alive "$@" && return 1
  return 0
}

send() {
  local sig="$1"
  shift
  local pid
  for pid in "$@"; do
    [[ -n "${pid:-}" ]] || continue
    kill -"$sig" "$pid" 2>/dev/null || true
  done
}

children_of() {
  local pid="$1" kid
  echo "$pid"
  for kid in $(ps -o pid= --ppid "$pid" 2>/dev/null || true); do
    kid="${kid// /}"
    [[ -n "$kid" ]] && children_of "$kid"
  done
}

is_launch_python() {
  local cmd
  cmd="$(ps -o args= -p "$1" 2>/dev/null || true)"
  [[ "$cmd" == *"/opt/ros/humble/bin/ros2 launch luggage_gazebo"* ]] ||
    [[ "$cmd" == *"ros2 launch luggage_gazebo"* ]]
}

resolve_launch() {
  local pid="$1" kid cmd
  if is_launch_python "$pid"; then
    echo "$pid"
    return 0
  fi
  cmd="$(ps -o args= -p "$pid" 2>/dev/null || true)"
  if [[ "$cmd" == *"ros2 launch luggage_gazebo"* ]]; then
    echo "$pid"
    for kid in $(ps -o pid= --ppid "$pid" 2>/dev/null || true); do
      kid="${kid// /}"
      if is_launch_python "$kid"; then
        echo "$kid"
      fi
    done
    return 0
  fi
  return 1
}

# PIDs whose command line belongs to this world's stack, not an unrelated ROS
# graph. Bracket the final letter so `pgrep -f` cannot match this script.
stack_pids() {
  local pid cmd
  while read -r pid cmd; do
    [[ -n "${pid:-}" ]] || continue
    case "$cmd" in
      *stop_sim.sh*) continue ;;
    esac
    echo "$pid"
  done < <(
    pgrep -a -f 'airport_loading\.sdf' 2>/dev/null || true
    pgrep -a -f 'ros2 launch luggage_gazebo' 2>/dev/null || true
    pgrep -a -f 'luggage_gazebo/rviz/sim_full\.rviz' 2>/dev/null || true
    pgrep -a -f 'ign gazeb[o]' 2>/dev/null || true
    pgrep -a -f '/install/luggage_(gazebo|perception|planning|description|packing)/' 2>/dev/null || true
  ) | awk 'NF && !seen[$1]++'
}

unique_pids() {
  awk 'NF && !seen[$1]++' | tr '\n' ' '
}

find_launches() {
  local pid cmd
  while read -r pid cmd; do
    [[ -n "${pid:-}" ]] || continue
    if [[ "$cmd" == *"/opt/ros/humble/bin/ros2 launch luggage_gazebo"* ]] ||
       [[ "$cmd" == *"python3 /opt/ros/humble/bin/ros2 launch luggage_gazebo"* ]]; then
      echo "$pid"
    fi
  done < <(pgrep -a -f 'ros2 launch luggage_gazebo' 2>/dev/null || true)
}

LAUNCH_PIDS=()

if [[ -f "$PIDFILE" ]]; then
  while read -r raw; do
    raw="${raw// /}"
    [[ "$raw" =~ ^[0-9]+$ ]] || continue
    if alive "$raw"; then
      mapfile -t resolved < <(resolve_launch "$raw" || echo "$raw")
      LAUNCH_PIDS+=("${resolved[@]}")
    fi
  done < "$PIDFILE"
fi

if [[ ${#LAUNCH_PIDS[@]} -eq 0 ]]; then
  mapfile -t found < <(find_launches)
  if [[ ${#found[@]} -eq 0 ]]; then
    leftovers="$(stack_pids | unique_pids)"
    if [[ -z "${leftovers// /}" ]]; then
      echo "stop_sim: no luggage_gazebo launch and no leftover stack"
      rm -f "$PIDFILE"
      exit 0
    fi
    echo "stop_sim: no launch PID; reaping leftover stack: $leftovers"
    send TERM $leftovers
    wait_gone "$WAIT_TERM" $leftovers || send KILL $leftovers
    wait_gone 2 $leftovers || true
    rm -f "$PIDFILE"
    echo "stop_sim: leftover stack gone"
    exit 0
  fi
  if [[ ${#found[@]} -gt 1 && "$ALL" -eq 0 ]]; then
    echo "stop_sim: ${#found[@]} launches running (${found[*]}). Re-run with --all or set ELFIN_SIM_PIDFILE." >&2
    exit 3
  fi
  LAUNCH_PIDS=("${found[@]}")
fi

# Dedup.
mapfile -t LAUNCH_PIDS < <(printf '%s\n' "${LAUNCH_PIDS[@]}" | awk 'NF && !seen[$1]++')

TREE=()
for lp in "${LAUNCH_PIDS[@]}"; do
  mapfile -t branch < <(children_of "$lp")
  TREE+=("${branch[@]}")
done

echo "stop_sim: SIGINT launch ${LAUNCH_PIDS[*]} (wait ${WAIT_INT}s)"
send INT "${LAUNCH_PIDS[@]}"
if wait_gone "$WAIT_INT" "${LAUNCH_PIDS[@]}" "${TREE[@]}"; then
  echo "stop_sim: launch tree exited after SIGINT"
else
  echo "stop_sim: SIGTERM leftovers still alive after SIGINT"
  still=()
  for pid in "${LAUNCH_PIDS[@]}" "${TREE[@]}"; do
    alive "$pid" && still+=("$pid")
  done
  extra="$(stack_pids | unique_pids)"
  send TERM "${still[@]}" $extra
  wait_gone "$WAIT_TERM" "${still[@]}" $extra || send KILL "${still[@]}" $extra
  wait_gone 2 "${still[@]}" $extra || true
fi

left="$(stack_pids | unique_pids)"
if [[ -n "${left// /}" ]]; then
  echo "stop_sim: sweeping remaining stack PIDs: $left"
  send TERM $left
  wait_gone "$WAIT_TERM" $left || send KILL $left
  wait_gone 2 $left || true
fi

rm -f "$PIDFILE"
if left="$(stack_pids | unique_pids)"; [[ -n "${left// /}" ]]; then
  echo "stop_sim: still alive: $left" >&2
  exit 1
fi
echo "stop_sim: stack stopped"
exit 0
