#!/usr/bin/env python3
"""Solve AX=XB from recorded ChArUco poses. Uses measured square pitch, not print nominal."""

from __future__ import division

import argparse
import json
import os
import sys

import numpy as np


def _R_t_from_rtvec(rvec, tvec):
    import cv2
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    return rotation, np.asarray(tvec, dtype=np.float64).reshape(3)


def board_pose(corners, ids, camera_matrix, dist, square_length_m, marker_length_m):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((10, 8), square_length_m, marker_length_m, dictionary)
    obj = []
    img = []
    chess = board.getChessboardCorners()
    id_set = {int(v): i for i, v in enumerate(np.asarray(ids).reshape(-1))}
    for ident, uv in zip(np.asarray(ids).reshape(-1), np.asarray(corners).reshape(-1, 2)):
        ident = int(ident)
        if ident < 0 or ident >= len(chess):
            continue
        obj.append(chess[ident])
        img.append(uv)
    if len(obj) < 6:
        return None
    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(obj, dtype=np.float32),
        np.asarray(img, dtype=np.float32),
        camera_matrix,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    return _R_t_from_rtvec(rvec, tvec)


def rpy_to_R(rpy):
    roll, pitch, yaw = [float(v) for v in rpy]
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def load_poses(path, square_length_m):
    with open(path, "r", encoding="utf-8") as handle:
        blob = json.load(handle)
    poses = blob["poses"] if isinstance(blob, dict) and "poses" in blob else blob
    return poses, float(blob.get("square_length_m", square_length_m) if isinstance(blob, dict) else square_length_m)


def solve(poses, camera_matrix, dist, square_length_m, marker_length_m, methods=None):
    import cv2
    methods = methods or {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    used = []
    for pose in poses:
        det = pose.get("detection") or {}
        if not det or det.get("count", 0) < 6:
            continue
        board = board_pose(
            det["corners_px"], det["corner_ids"],
            camera_matrix, dist, square_length_m, marker_length_m,
        )
        if board is None:
            continue
        if "T_base_flange" in pose:
            T = np.asarray(pose["T_base_flange"], dtype=np.float64)
            Rg, tg = T[:3, :3], T[:3, 3]
        elif "rpy_base_flange" in pose:
            Rg = rpy_to_R(pose["rpy_base_flange"])
            tg = np.asarray(pose["xyz_base_flange"], dtype=np.float64)
        else:
            continue
        Rc, tc = board
        R_gripper2base.append(Rg)
        t_gripper2base.append(tg.reshape(3, 1))
        R_target2cam.append(Rc)
        t_target2cam.append(tc.reshape(3, 1))
        used.append(pose.get("pose_id"))
    if len(used) < 3:
        raise RuntimeError("need >=3 posed detections with T_base_flange, got %d" % len(used))
    results = {}
    for name, flag in methods.items():
        R, t = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=flag)
        results[name] = {
            "R": np.asarray(R).tolist(),
            "t_m": np.asarray(t).reshape(3).tolist(),
        }
    spread = None
    if "TSAI" in results and "PARK" in results:
        t0 = np.asarray(results["TSAI"]["t_m"])
        t1 = np.asarray(results["PARK"]["t_m"])
        r0 = np.asarray(results["TSAI"]["R"])
        r1 = np.asarray(results["PARK"]["R"])
        rel = r0.T.dot(r1)
        ang = np.degrees(np.arccos(min(1.0, max(-1.0, (np.trace(rel) - 1.0) * 0.5))))
        spread = {
            "translation_mm": float(np.linalg.norm(t0 - t1) * 1000.0),
            "rotation_deg": float(ang),
        }
    return {"methods": results, "used_poses": used, "spread": spread, "n": len(used)}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="capture_index.json or pose directory")
    parser.add_argument("--square-length-m", type=float, default=0.050,
                        help="Measured physical pitch; do not hard-code printer scale")
    parser.add_argument("--marker-length-m", type=float, default=0.0375)
    parser.add_argument("--fx", type=float)
    parser.add_argument("--fy", type=float)
    parser.add_argument("--cx", type=float)
    parser.add_argument("--cy", type=float)
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    path = args.input
    if os.path.isdir(path):
        path = os.path.join(path, "capture_index.json")
    poses, stored = load_poses(path, args.square_length_m)
    square = args.square_length_m if args.square_length_m else stored
    cam = poses[0].get("camera_matrix") if poses else None
    dist = np.zeros(5)
    if cam is None:
        if None in (args.fx, args.fy, args.cx, args.cy):
            raise SystemExit("camera_matrix missing; pass --fx --fy --cx --cy")
        cam = [[args.fx, 0.0, args.cx], [0.0, args.fy, args.cy], [0.0, 0.0, 1.0]]
    cam = np.asarray(cam, dtype=np.float64)
    result = solve(poses, cam, dist, square, args.marker_length_m)
    text = json.dumps(result, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
