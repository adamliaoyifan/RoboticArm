#!/usr/bin/env bash
# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
#
# Print live elfin joints as a robot_poses YAML snippet. Paste into
# src/luggage_description/config/robot_poses.site.yaml as pickup_observe,
# then set defaults.observe_pose: pickup_observe (or OBSERVE_POSE=pickup_observe).
#
#   source scripts/env_jazzy_real.sh
#   ./scripts/dump_observe_pose.sh
#
set -euo pipefail

DEPLOY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_SH="${DEPLOY}/env_jazzy_real.sh"
POSE_NAME="${1:-pickup_observe}"

set +u
# shellcheck disable=SC1090
source "${ENV_SH}"
set -u

python3 - "${POSE_NAME}" <<'PY'
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

from luggage_planning.named_robot_poses import JOINTS, format_pose_yaml

pose_name = sys.argv[1]
rclpy.init()
node = rclpy.create_node("dump_observe_pose")
held = []

def _cb(msg):
    by_name = dict(zip(msg.name, msg.position))
    if set(JOINTS) <= set(by_name):
        held.append([float(by_name[j]) for j in JOINTS])

node.create_subscription(JointState, "/joint_states", _cb, 10)
deadline = time.monotonic() + 5.0
while rclpy.ok() and not held and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if not held:
    sys.stderr.write("no /joint_states with elfin joints\n")
    sys.exit(1)
sys.stdout.write(
    "# measured live joints — paste under poses:\n"
    + format_pose_yaml(pose_name, held[0], observe_default=pose_name)
)
PY
