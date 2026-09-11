"""RGB overlay of GT / baseline / learned boxes."""

from __future__ import division

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from research.lrf_p1.mujoco_d455.camera import project_world
from research.lrf_p1.viz_samples import box_wire_xyz

COLORS = {
    "gt": (20, 180, 40),
    "baseline": (40, 90, 230),
    "learned": (220, 40, 40),
    "fusion": (160, 70, 200),
}


def _edges_uv(box, R_cv, t, K):
    wire = box_wire_xyz(box)
    if wire is None:
        return []
    xs, ys, zs = wire
    pts = []
    segs = []
    buf = []
    for x, y, z in zip(xs, ys, zs):
        if x is None:
            if len(buf) == 2:
                segs.append(tuple(buf))
            buf = []
            continue
        buf.append((float(x), float(y), float(z)))
    uv_segs = []
    for a, b in segs:
        u, v, z = project_world(np.array([a, b]), R_cv, t, K)
        if z[0] <= 0.05 or z[1] <= 0.05:
            continue
        uv_segs.append(((float(u[0]), float(v[0])), (float(u[1]), float(v[1]))))
    return uv_segs


def overlay_rgb(rgb, boxes, R_cv, t, K, caption):
    img = Image.fromarray(np.asarray(rgb, dtype=np.uint8), mode="RGB")
    draw = ImageDraw.Draw(img)
    order = ("gt", "baseline", "learned")
    for name in order:
        box = boxes.get(name)
        if not box:
            continue
        color = COLORS[name]
        for p0, p1 in _edges_uv(box, R_cv, t, K):
            draw.line([p0, p1], fill=color, width=3)
    try:
        font = ImageFont.load_default()
    except Exception:
        font = None
    draw.rectangle([8, 8, 520, 70], fill=(0, 0, 0, 180))
    draw.text((16, 14), caption, fill=(255, 255, 255), font=font)
    draw.text(
        (16, 36),
        "green=GT  blue=baseline  red=learned (pose locked to baseline)   D455 depth, single frame",
        fill=(220, 220, 220),
        font=font,
    )
    return np.asarray(img)
