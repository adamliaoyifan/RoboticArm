#!/usr/bin/env python3
"""Gate 3 assets: YOLO, CLIP, D435 launch, dump script, hardware scene launch.

Camera dump and measured scene_tf are hardware. This check only verifies
the files needed to run those steps.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PERC = REPO / "elfin_humble_ws" / "src" / "luggage_perception"
DESC = REPO / "elfin_humble_ws" / "src" / "luggage_description"
YOLO = PERC / "yolov8s-world.pt"
CLIP = PERC / "vendor" / "clip_models" / "ViT-B-32.pt"
REAL = PERC / "config" / "semantic_segmenter.real.yaml"
DUMP = PERC / "scripts" / "dump_camera_frames.py"
RS = PERC / "launch" / "realsense_d435.launch.py"
SCENE = DESC / "launch" / "scene_hardware.launch.py"
EXAMPLE = DESC / "config" / "scene_tf.yaml.example"


def main() -> int:
    checks = {
        "yolov8s-world.pt": YOLO,
        "CLIP ViT-B-32.pt": CLIP,
        "semantic_segmenter.real.yaml": REAL,
        "dump_camera_frames.py": DUMP,
        "realsense_d435.launch.py": RS,
        "scene_hardware.launch.py": SCENE,
        "scene_tf.yaml.example": EXAMPLE,
    }
    missing = []
    for name, path in checks.items():
        ok = path.is_file()
        print("%s: %s" % (name, path if ok else "MISSING"))
        if not ok:
            missing.append(name)
    if CLIP.is_file():
        size_mb = CLIP.stat().st_size / (1024 * 1024)
        print("CLIP size: %.1f MB" % size_mb)
        if size_mb < 300:
            print("CLIP weight looks truncated")
            missing.append("clip-size")
    if YOLO.is_file():
        print("YOLO size: %.1f MB" % (YOLO.stat().st_size / (1024 * 1024)))
    if REAL.is_file():
        text = REAL.read_text(encoding="utf-8")
        if "extrinsics_source: config" not in text:
            print("real overlay missing extrinsics_source: config")
            missing.append("extrinsics")
    if os.environ.get("ROS_DISTRO"):
        try:
            from ament_index_python.packages import get_package_share_directory

            rs = get_package_share_directory("realsense2_camera")
            print("realsense2_camera share: %s" % rs)
        except Exception as exc:
            print("realsense2_camera: not on AMENT_PREFIX_PATH (%s)" % exc)
            print("  sudo apt install ros-jazzy-realsense2-camera")
            missing.append("realsense2_camera")
    if missing:
        print("gate3 assets: FAIL (%s)" % ",".join(missing))
        return 1
    print("gate3 assets: PASS")
    print("hardware still needed: measured scene_tf.yaml, live D435, dump_camera_frames.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
