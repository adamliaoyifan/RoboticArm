#!/usr/bin/env python3
"""Gate 2: URDF livox frames exist; livox_ros_driver2 overlay built for Jazzy."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
MOUNT = REPO / "elfin_humble_ws" / "src" / "luggage_description" / "urdf" / "eef_sensor_mount.urdf.xacro"
ORIGIN = REPO / "elfin_humble_ws" / "src" / "luggage_description" / "config" / "mid360_origin.xacro"
CFG = (
    REPO
    / "elfin_humble_ws"
    / "src"
    / "luggage_description"
    / "config"
    / "MID360s_config.json.example"
)


def main() -> int:
    failures = []
    for path in (MOUNT, ORIGIN, CFG):
        if not path.is_file():
            print("missing %s" % path)
            failures.append("files")
            continue
        text = path.read_text(encoding="utf-8")
        print("ok %s" % path.relative_to(REPO))
        if path == MOUNT:
            for name in ("livox_frame", "livox_imu_frame", "mid360_mount_frame"):
                if name not in text:
                    print("  missing frame %s" % name)
                    failures.append(name)
        if path == ORIGIN and "0.047" not in text:
            print("  missing 47 mm optical height")
            failures.append("optical")
        if path == CFG:
            data = json.loads(text)
            if "Mid360s" not in data:
                print("  config must use Mid360s (official schema)")
                failures.append("json")

    overlay = ROOT / "livox_ws" / "install" / "setup.bash"
    sdk_so = ROOT / "livox_ws" / "sdk_prefix" / "lib" / "liblivox_lidar_sdk_shared.so"
    if overlay.is_file():
        print("livox overlay: %s" % overlay)
    else:
        print("livox overlay: not built (run scripts/setup_livox_driver.sh)")
        failures.append("overlay")
    if sdk_so.is_file():
        print("livox sdk: %s" % sdk_so)
    else:
        print("livox sdk: missing")
        failures.append("sdk")

    if os.environ.get("ROS_DISTRO"):
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("livox_ros_driver2")
            print("livox_ros_driver2 share: %s" % share)
        except Exception as exc:
            print("livox_ros_driver2: not on AMENT_PREFIX_PATH (%s)" % exc)
            print("  source livox_ws/install/setup.bash after the overlay builds")

    if failures:
        print("gate2: FAIL (%s)" % ",".join(failures))
        return 1
    print("gate2 software: PASS (live /livox/lidar still needs the Mid-360 on 192.168.1.0/24)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
