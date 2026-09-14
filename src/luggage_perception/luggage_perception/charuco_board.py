"""calib.io CC600 Coarse ChArUco spec and detection (ROS-free)."""

from __future__ import division

import os

CC600_CALIBIO_COARSE = {
    "name": "cc600_calibio_coarse",
    "squares_x": 14,
    "squares_y": 9,
    "square_length_m": 0.040,
    "marker_length_m": 0.030,
    "dictionary": "DICT_5X5_100",
    "legacy_pattern": False,
    "min_corners": 12,
}

_CONFIG_NAME = "charuco_cc600_calibio_coarse.yaml"


def default_config_path():
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("luggage_perception"),
            "config", _CONFIG_NAME))
    except Exception:
        pass
    pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    candidates.append(os.path.join(pkg, "config", _CONFIG_NAME))
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[-1]


def load_spec(path=None):
    spec = dict(CC600_CALIBIO_COARSE)
    path = path or default_config_path()
    if path and os.path.isfile(path):
        import yaml
        with open(path, "r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}
        for key, value in loaded.items():
            if key in spec or key in (
                "name", "squares_x", "squares_y", "square_length_m",
                "marker_length_m", "dictionary", "legacy_pattern", "min_corners",
            ):
                spec[key] = value
    spec["squares_x"] = int(spec["squares_x"])
    spec["squares_y"] = int(spec["squares_y"])
    spec["square_length_m"] = float(spec["square_length_m"])
    spec["marker_length_m"] = float(spec["marker_length_m"])
    spec["legacy_pattern"] = bool(spec.get("legacy_pattern", False))
    spec["min_corners"] = int(spec.get("min_corners", 12))
    spec["dictionary"] = str(spec.get("dictionary") or "DICT_5X5_100")
    return spec


def apply_overrides(spec, square_length_m=None, marker_length_m=None):
    out = dict(spec)
    if square_length_m is not None:
        out["square_length_m"] = float(square_length_m)
    if marker_length_m is not None:
        out["marker_length_m"] = float(marker_length_m)
    return out


def interior_corner_count(spec):
    return (int(spec["squares_x"]) - 1) * (int(spec["squares_y"]) - 1)


def make_board(spec):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, spec["dictionary"]))
    board = cv2.aruco.CharucoBoard(
        (int(spec["squares_x"]), int(spec["squares_y"])),
        float(spec["square_length_m"]),
        float(spec["marker_length_m"]),
        dictionary,
    )
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(bool(spec.get("legacy_pattern", False)))
    return board, dictionary


def detect_charuco(image, spec):
    import cv2
    import numpy as np
    board, _dictionary = make_board(spec)
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    detector = cv2.aruco.CharucoDetector(board)
    corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
    if corners is None or ids is None:
        return None
    markers = []
    if marker_ids is not None:
        markers = [int(v) for v in np.asarray(marker_ids).reshape(-1)]
    return {
        "corner_ids": [int(v) for v in ids.reshape(-1)],
        "corners_px": np.asarray(corners).reshape(-1, 2).tolist(),
        "count": int(len(ids)),
        "marker_ids": markers,
        "marker_count": int(len(markers)),
    }


def draw_detection(image_bgr, det):
    import cv2
    import numpy as np
    out = image_bgr.copy()
    if not det:
        return out
    pts = np.asarray(det["corners_px"], dtype=np.float32).reshape(-1, 1, 2)
    ids = np.asarray(det["corner_ids"], dtype=np.int32).reshape(-1, 1)
    try:
        cv2.aruco.drawDetectedCornersCharuco(out, pts, ids)
    except Exception:
        for uv in pts.reshape(-1, 2):
            cv2.circle(out, (int(round(uv[0])), int(round(uv[1]))), 4, (0, 255, 0), -1)
    return out
