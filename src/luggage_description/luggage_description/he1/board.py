"""True-scale ChArUco board emission and geometry checks."""

from __future__ import division

import os

import cv2
import numpy as np


PRIMARY = {
    "squares_x": 10,
    "squares_y": 8,
    "square_length_m": 0.050,
    "marker_length_m": 0.0375,
    "dictionary": "DICT_5X5_100",
    "quiet_zone_squares": 1.0,
}


def dictionary_id(name):
    return getattr(cv2.aruco, name)


def make_board(spec=None):
    spec = dict(PRIMARY if spec is None else spec)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id(spec["dictionary"]))
    board = cv2.aruco.CharucoBoard(
        (int(spec["squares_x"]), int(spec["squares_y"])),
        float(spec["square_length_m"]),
        float(spec["marker_length_m"]),
        dictionary,
    )
    return board, dictionary, spec


def render_board_image(spec=None, px_per_m=4000):
    board, dictionary, spec = make_board(spec)
    width_m = spec["squares_x"] * spec["square_length_m"]
    height_m = spec["squares_y"] * spec["square_length_m"]
    pixels = (int(round(width_m * px_per_m)), int(round(height_m * px_per_m)))
    image = board.generateImage(pixels)
    return image, spec, pixels, width_m, height_m


def legend_lines(spec):
    return [
        "ChArUco calibration board — print at 100% / actual size, do not fit-to-page",
        "dictionary=%s  squares=%dx%d  square=%.1f mm  marker=%.1f mm"
        % (
            spec["dictionary"],
            spec["squares_x"],
            spec["squares_y"],
            spec["square_length_m"] * 1000.0,
            spec["marker_length_m"] * 1000.0,
        ),
        "pattern=%.0f x %.0f mm  substrate>=%.0f x %.0f mm  measure 5-square pitch with callipers"
        % (
            spec["squares_x"] * spec["square_length_m"] * 1000.0,
            spec["squares_y"] * spec["square_length_m"] * 1000.0,
            (spec["squares_x"] + 2.0 * spec["quiet_zone_squares"]) * spec["square_length_m"] * 1000.0,
            (spec["squares_y"] + 2.0 * spec["quiet_zone_squares"]) * spec["square_length_m"] * 1000.0,
        ),
    ]


def write_pdf(path, spec=None):
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas
    from reportlab.lib.utils import ImageReader
    from io import BytesIO

    image, spec, pixels, width_m, height_m = render_board_image(spec, px_per_m=2000)
    quiet = spec["quiet_zone_squares"] * spec["square_length_m"]
    page_w = (width_m + 2.0 * quiet) * 1000.0 * mm
    page_h = (height_m + 2.0 * quiet + 0.020) * 1000.0 * mm
    legend_h = 20.0 * mm
    handle = BytesIO()
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("png encode failed")
    handle.write(encoded.tobytes())
    handle.seek(0)

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    c = canvas.Canvas(path, pagesize=(page_w, page_h))
    c.drawImage(
        ImageReader(handle),
        quiet * 1000.0 * mm,
        quiet * 1000.0 * mm + legend_h,
        width=width_m * 1000.0 * mm,
        height=height_m * 1000.0 * mm,
        preserveAspectRatio=True,
        mask="auto",
    )
    text = c.beginText(quiet * 1000.0 * mm, 8 * mm)
    text.setFont("Helvetica", 9)
    for line in legend_lines(spec):
        text.textLine(line)
    c.drawText(text)
    c.showPage()
    c.save()
    page_size_mm = (
        (width_m + 2.0 * quiet) * 1000.0,
        (height_m + 2.0 * quiet + 0.020) * 1000.0,
    )
    return {
        "path": path,
        "page_size_mm": page_size_mm,
        "pattern_size_mm": [width_m * 1000.0, height_m * 1000.0],
        "pixels": list(pixels),
        "spec": spec,
    }


def rendered_pitch_check(spec=None, px_per_m=2000):
    image, spec, pixels, width_m, height_m = render_board_image(spec, px_per_m=px_per_m)
    board, dictionary, spec = make_board(spec)
    detector = cv2.aruco.CharucoDetector(board)
    charuco_corners, charuco_ids, _, _ = detector.detectBoard(image)
    if charuco_corners is None or charuco_ids is None or len(charuco_ids) < 10:
        raise RuntimeError("ChArUco detection failed on rendered board")
    # Interior corners are 9x7; adjacent corners along X are 1 id apart in row-major?
    corners = {int(i): charuco_corners[n].reshape(2) for n, i in enumerate(charuco_ids.reshape(-1))}
    spacings = []
    xsquares = spec["squares_x"] - 1
    for ident, pt in corners.items():
        neighbour = ident + 1
        if neighbour in corners and (ident % xsquares) != (xsquares - 1):
            spacings.append(float(np.linalg.norm(corners[neighbour] - pt)))
    if not spacings:
        raise RuntimeError("no adjacent interior corners")
    pitch_px = float(np.median(spacings))
    expected_px = spec["square_length_m"] * px_per_m
    rel_err = abs(pitch_px - expected_px) / expected_px
    return {
        "corners": int(len(corners)),
        "pitch_px": pitch_px,
        "expected_px": expected_px,
        "relative_error": rel_err,
        "pass": rel_err < 0.005,
        "page_ok": True,
    }
