#!/usr/bin/env python3
"""ChArUco eye-in-hand capture (live or replay). Joints from CPS, never /joint_states."""

from __future__ import division

import argparse
import json
import os
import sys
import time


def detect_charuco(image_bgr, square_length_m, marker_length_m, dictionary_name="DICT_5X5_100"):
    import cv2
    import numpy as np
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    board = cv2.aruco.CharucoBoard((10, 8), square_length_m, marker_length_m, dictionary)
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.CharucoDetector(board)
    corners, ids, _, _ = detector.detectBoard(gray)
    if corners is None or ids is None:
        return None
    return {
        "corner_ids": [int(v) for v in ids.reshape(-1)],
        "corners_px": np.asarray(corners).reshape(-1, 2).tolist(),
        "count": int(len(ids)),
    }


def read_cps_joints(host="192.168.0.10", port=10003):
    sdk = os.environ.get(
        "HUAYAN_SDK_PATH",
        "/home/adamliao/work/RoboticArm/third_party/huayan_python_sdk",
    )
    if sdk not in sys.path:
        sys.path.insert(0, sdk)
    from CPS import CPSClient
    client = CPSClient()
    nret = client.HRIF_Connect(0, host, int(port))
    if nret != 0:
        raise RuntimeError("HRIF_Connect failed nRet=%s" % nret)
    try:
        result = []
        nret = client.HRIF_ReadActACS(0, 0, result)
        if nret != 0 or not result:
            # Some firmware returns joint pose through ReadActJointPos (deg).
            result = []
            nret = client.HRIF_ReadActJointPos(0, 0, result)
        if nret != 0:
            raise RuntimeError("HRIF_ReadActACS/JointPos failed nRet=%s" % nret)
        return [float(v) for v in result[:6]]
    finally:
        client.HRIF_DisConnect(0)


def replay_dir(path, square_length_m, marker_length_m):
    import cv2
    poses = []
    names = sorted(
        n for n in os.listdir(path)
        if n.startswith("pose_") and n.endswith(".json")
    )
    for name in names:
        meta_path = os.path.join(path, name)
        with open(meta_path, "r", encoding="utf-8") as handle:
            meta = json.load(handle)
        image_path = meta.get("image") or name.replace(".json", ".png")
        if not os.path.isabs(image_path):
            image_path = os.path.join(path, image_path)
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("missing image %s" % image_path)
        det = detect_charuco(image, square_length_m, marker_length_m)
        rec = dict(meta)
        rec["detection"] = det
        rec["source"] = "replay"
        poses.append(rec)
    out = os.path.join(path, "capture_index.json")
    with open(out, "w", encoding="utf-8") as handle:
        json.dump({"poses": poses, "square_length_m": square_length_m}, handle, indent=2)
        handle.write("\n")
    return out


def live_once(out_dir, pose_id, square_length_m, marker_length_m, settle_sec, color_topic):
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    time.sleep(max(0.0, settle_sec))
    joints = read_cps_joints()
    # Image from ROS if available; otherwise require --image.
    raise RuntimeError(
        "live ROS grab is HE-2; this script records CPS joints=%s. "
        "Pass --image PNG from dump_camera_frames or use --replay"
        % joints
    )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay")
    parser.add_argument("--out-dir")
    parser.add_argument("--pose-id")
    parser.add_argument("--image")
    parser.add_argument("--square-length-m", type=float, default=0.050)
    parser.add_argument("--marker-length-m", type=float, default=0.0375)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--color-topic", default="/camera/d555/color/image_raw")
    args = parser.parse_args(argv)
    if args.replay:
        print(replay_dir(args.replay, args.square_length_m, args.marker_length_m))
        return 0
    if args.image and args.out_dir and args.pose_id:
        import cv2
        import shutil
        os.makedirs(args.out_dir, exist_ok=True)
        image = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit("cannot read %s" % args.image)
        det = detect_charuco(image, args.square_length_m, args.marker_length_m)
        joints = None
        try:
            joints = read_cps_joints()
            joint_source = "CPS HRIF_ReadActACS"
        except Exception as exc:
            joint_source = "unavailable: %s" % exc
        dest_img = os.path.join(args.out_dir, "pose_%s.png" % args.pose_id)
        shutil.copy2(args.image, dest_img)
        rec = {
            "pose_id": args.pose_id,
            "image": os.path.basename(dest_img),
            "joints_rad_or_deg": joints,
            "joint_source": joint_source,
            "detection": det,
            "square_length_m": args.square_length_m,
            "marker_length_m": args.marker_length_m,
            "note": "Do not use /joint_states; HB-3 recorded all-zero executor states",
        }
        dest_json = os.path.join(args.out_dir, "pose_%s.json" % args.pose_id)
        with open(dest_json, "w", encoding="utf-8") as handle:
            json.dump(rec, handle, indent=2)
            handle.write("\n")
        print(dest_json)
        return 0
    parser.error("use --replay DIR or --image/--out-dir/--pose-id")
    return 2


if __name__ == "__main__":
    sys.exit(main())
