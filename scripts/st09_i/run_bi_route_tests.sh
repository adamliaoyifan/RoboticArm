#!/usr/bin/env bash
# Run the ST09-B and ST09-I route suites in separate interpreters.
#
# B's mock_graph and I's whole_chain each call rclpy.init() on a different
# ROS domain. A single pytest process that collects both reports nine
# whole_chain errors ("context already initialized") before any I assertion
# runs. This script is the documented gate: one process per suite, same
# extra pytest arguments forwarded to both.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

# ROS setup scripts read optional variables that are unset in this shell.
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$ROOT/install-st09/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/install-st09/setup.bash"
fi
set -u

export ST09_ROUTE_TEST_CHILD=1
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"

status=0
python3 -m pytest -q -p no:cacheprovider "$@" tests/st09_parallel/B || status=$?
python3 -m pytest -q -p no:cacheprovider "$@" tests/st09_parallel/I || status=$?
exit "$status"
