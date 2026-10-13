#!/usr/bin/env bash
# Paired rehearsal replay. Never publishes onto the live domain (7).
set -euo pipefail

BAG=""
TRAJECTORY=""
SNAPSHOT=""
DOMAIN=42
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --trajectory) TRAJECTORY="$2"; shift 2 ;;
    --snapshot) SNAPSHOT="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
if [[ -z "$BAG" && -z "$TRAJECTORY" ]]; then
  echo "--bag or --trajectory required" >&2
  exit 2
fi
if [[ -n "$BAG" && ! -e "$BAG" ]]; then echo "bag not found: $BAG" >&2; exit 2; fi
if [[ -n "$TRAJECTORY" && ! -e "$TRAJECTORY" ]]; then echo "trajectory not found: $TRAJECTORY" >&2; exit 2; fi
if [[ "$DOMAIN" == "7" ]]; then
  echo "refusing ROS_DOMAIN_ID=7 (live rehearsal). Use --domain 42." >&2
  exit 2
fi

unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$WS/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID="$DOMAIN"

if [[ -n "$TRAJECTORY" ]]; then
  exec python3 "$WS/scripts/stage_perf_probe.py" \
    --trajectory "$TRAJECTORY" ${SNAPSHOT:+--snapshot "$SNAPSHOT"} --domain "$DOMAIN"
fi

exec python3 "$WS/scripts/stage_perf_probe.py" \
  --bag "$BAG" --repeats 3 --domain "$DOMAIN"
