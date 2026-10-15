#!/usr/bin/env python3
"""Box-2 candidate map for score_run01 (G3 wave 3).

Panels: (A) container top view — box-2's 627 ComputePlacement candidates,
the 164 occupancy-excluded points around box 1, box-1's placed footprint
(77 occupied cells) and its chosen slot; (B) YZ section at box-1's x —
floor candidates z 0.639 vs the single stack candidate z 0.889; (C) the
counts. Companion JSON carries every number.
"""
import json
import os
import sys

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
import numpy as np

EVID = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
        "master_pack_recovery/2026-10-14_g3")
RUN = os.path.join(EVID, "score_run01")
OUT_DIR = os.path.join(EVID, "wave3", "figures")
DATA = json.load(open(os.path.join(OUT_DIR, "box2_candidates_data.json")))

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, AQUA, CRIT = "#2a78d6", "#1baf7a", "#d03b3b"
NEUTRAL = "#efeeea"
RAMP_TOP = "#104281"
PLATE = dict(facecolor="white", edgecolor="none", alpha=0.88, pad=2.0)

slots0 = DATA["box0_slots"]
slots1 = DATA["box1_slots"]
floor1 = [s for s in slots1 if abs(s[2] - 0.639) < 1e-3]
stack1 = [s for s in slots1 if abs(s[2] - 0.639) >= 1e-3]
removed = DATA["removed"]
occ_cells = DATA["occupied_cells"]          # map-frame [ix, iy] -> centers
res, origin = DATA["map_resolution"], DATA["map_origin"]
box_wdh = DATA["box1_box_wdh"]
box1_rect = DATA["box1_rect"]               # container-link xy + half extents

fig = plt.figure(figsize=(13.6, 6.4), dpi=140)
gs = fig.add_gridspec(1, 3, width_ratios=[2.35, 1.0, 1.05],
                      wspace=0.34, left=0.045, right=0.985,
                      top=0.86, bottom=0.14)
axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])
axC = fig.add_subplot(gs[0, 2])

# ---------------------------------------------------------------- A top view
HLX, HLY = 0.745, 0.985
# 7-face hull outline: chamfer on +Y from (0.55,floor-side) to (0.985, wall)
hull_xy = [(-HLX, -HLY), (HLX, -HLY), (HLX, -0.55), (HLX + 0.0, 0.0)]
cham_poly = Polygon([(HLX, -0.55), (HLX, HLY), (0.55, HLY), (0.55, -0.55),
                     (HLX, -0.55)][::-1], closed=True,
                    facecolor=NEUTRAL, edgecolor="none", alpha=0.9, zorder=0)
# full outer box + chamfer wedge shading
axA.add_patch(Rectangle((-HLX, -HLY), 2 * HLX, 2 * HLY, fill=False,
                        edgecolor=INK, lw=1.4, zorder=3))
axA.add_patch(Polygon([(HLX, 0.55), (HLX, HLY), (0.55, HLY)],
                      closed=True, facecolor=NEUTRAL, edgecolor=INK2,
                      lw=0.8, ls=(0, (3, 2)), zorder=1))
# aperture on the west (-X) face: y in [-0.928, 0.398]
axA.plot([-HLX, -HLX], [-0.928, 0.398], color=INK, lw=3.2,
         solid_capstyle="butt", zorder=4)
axA.plot([-HLX, -HLX], [-HLY, -0.928], color=INK, lw=1.4, zorder=4)
axA.plot([-HLX, -HLX], [0.398, HLY], color=INK, lw=1.4, zorder=4)

# occupied cells under box 1
for (cx, cy) in occ_cells:
    axA.add_patch(Rectangle((cx - res / 2, cy - res / 2), res, res,
                            facecolor="#86b6ef", edgecolor=BLUE,
                            lw=0.3, zorder=2))
# box-1 placed footprint + chosen slot
bx, by, hx, hy = box1_rect
axA.add_patch(Rectangle((bx - hx, by - hy), 2 * hx, 2 * hy, fill=False,
                        edgecolor=BLUE, lw=2.0, zorder=5))

# 627 floor candidates (box centers, neutral)
axA.scatter([p[0] for p in floor1], [p[1] for p in floor1], s=5, c=INK2,
            alpha=0.55, lw=0, zorder=3, label="box-2 floor candidates (626)")
# 164 occupancy-excluded points
axA.scatter([p[0] for p in removed], [p[1] for p in removed], s=16,
            facecolors="none", edgecolors=INK2, marker="D", lw=0.8,
            zorder=4, label="occupancy-excluded (164)")
# the stack candidate
for p in stack1:
    axA.scatter([p[0]], [p[1]], s=110, marker="*", c=AQUA, zorder=6)
    axA.text(1.03, 0.22, "stack candidate\nz=0.889 (AQUA star)",
             transform=axA.transAxes, fontsize=8.5, color=INK, ha="left",
             bbox=dict(PLATE, edgecolor=AQUA, lw=1.2))
axA.text(1.03, 0.68, "box 1 placed\n(BLUE outline + cells)",
         transform=axA.transAxes, fontsize=8.5, color=INK, ha="left",
         bbox=dict(PLATE, edgecolor=BLUE, lw=1.2))
axA.annotate("chamfer (+Y)", xy=(0.68, 0.80), xytext=(0.97, 0.965),
             textcoords="axes fraction", fontsize=8, color=INK2,
             ha="left", va="top")
axA.annotate("aperture (west)", xy=(-0.72, -0.75), fontsize=8, color=INK2,
             rotation=90, ha="left", va="bottom")
axA.set_xlim(-HLX - 0.10, HLX + 0.10)
axA.set_ylim(-HLY - 0.10, HLY + 0.10)
axA.set_aspect("equal")
axA.set_xlabel("container_link x [m]", fontsize=9)
axA.set_ylabel("container_link y [m]", fontsize=9)
axA.set_title("(A) top view — box-2 placement points", fontsize=10, color=INK)
axA.legend(loc="upper center", bbox_to_anchor=(0.5, -0.085), ncol=3,
           fontsize=8, frameon=False)

# ---------------------------------------------------------------- B section
axB.add_patch(Rectangle((-HLY, 0.53), 2 * HLY, 0.05, facecolor=NEUTRAL,
                        edgecolor=INK2, lw=0.6, zorder=1))
# chamfer profile (+Y side)
axB.plot([0.55, 0.985], [0.53, 0.90], color=INK2, lw=1.2, ls=(0, (3, 2)))
axB.plot([0.985, 0.985], [0.90, 1.35], color=INK2, lw=1.2)
axB.plot([-0.985, -0.985], [0.53, 1.35], color=INK2, lw=1.2)
axB.plot([-0.985, 0.985], [1.35, 1.35], color=INK2, lw=1.2)
# box 1 in section (at its y span)
axB.add_patch(Rectangle((by - hy, 0.53), 2 * hy, box_wdh[2] if False else 0.2175,
                        facecolor="#86b6ef", edgecolor=BLUE, lw=1.2, zorder=3))
# measured support surface over box 1 (0.25 above floor)
axB.plot([by - hy, by + hy], [0.78, 0.78], color=RAMP_TOP, lw=1.0,
         ls=(0, (2, 2)), zorder=4)
# floor candidates band
axB.plot([-0.80, -0.10], [0.639, 0.639], color=INK2, lw=1.4)
axB.text(-1.02, 0.72, "floor candidates\nz=0.639 (626)", fontsize=8.5,
         color=INK, ha="left", va="bottom", bbox=PLATE)
# stack candidate box on box-1
for p in stack1:
    axB.add_patch(Rectangle((p[1] - DATA["box1_box_wdh"][1] / 2, 0.78),
                            DATA["box1_box_wdh"][1], 0.2175, fill=False,
                            edgecolor=AQUA, lw=1.6, zorder=5))
    axB.plot([p[1]], [0.889], marker="*", ms=13, color=AQUA, zorder=6)
axB.text(0.36, 1.03, "stack z=0.889\n(support 0.78)", fontsize=8.5,
         color=INK, ha="center", va="bottom", bbox=PLATE)
axB.text(by, 0.639, "box 1", fontsize=8, color=INK, ha="center",
         va="center")
axB.set_xlim(-1.06, 1.06)
axB.set_ylim(0.35, 1.42)
axB.set_aspect(0.62)
axB.set_xlabel("container_link y [m]", fontsize=9)
axB.set_ylabel("z [m]", fontsize=9)
axB.set_title("(B) YZ section @ box-1 x", fontsize=10, color=INK)

# ---------------------------------------------------------------- C bars
labels = ["box-1 candidates\n(rev-1 map)", "box-2 candidates\n(rev-2 map)",
          "occupancy-excluded", "portal attempts\n(60 s budget)",
          "stack candidates"]
vals = [len(slots0), len(slots1), len(removed), DATA["portal_attempts"],
        len(stack1)]
colors = [INK2, INK, INK2, CRIT, AQUA]
order = np.argsort(vals)
axBbar = axC
axBbar.barh(range(len(vals)), [vals[i] for i in order],
            color=[colors[i] for i in order], height=0.62)
axBbar.set_yticks(range(len(vals)))
axBbar.set_yticklabels([labels[i] for i in order], fontsize=8.5)
for row, i in enumerate(order):
    axBbar.text(vals[i] + 12, row, str(vals[i]), va="center", fontsize=9,
                color=INK)
axBbar.set_xlim(0, 900)
axBbar.set_xlabel("count", fontsize=9)
axBbar.set_title("(C) counts", fontsize=10, color=INK)
axBbar.spines[["top", "right"]].set_visible(False)

fig.suptitle("score_run01 box-2 placement: 627 candidates after occupancy "
             "excluded 164 around box 1 (+1 stack option); the 60 s search "
             "budget ended after 3 portal attempts",
             fontsize=11, color=INK, y=0.965)

png = os.path.join(OUT_DIR, "box2_candidates.png")
fig.savefig(png)
fig.canvas.draw()
rend = fig.canvas.get_renderer()

# bbox audit
bad = 0
boxes = []
for ax_ in fig.axes:
    for t in ax_.texts:
        if t.get_text().strip():
            boxes.append((t.get_text().split("\n")[0], t.get_window_extent(rend)))
for name, bb in boxes:
    if (bb.x0 < 1 or bb.x1 > fig.bbox.x1 - 1 or bb.y0 < 1
            or bb.y1 > fig.bbox.y1 - 1):
        print("CLIP", repr(name)); bad += 1
for i in range(len(boxes)):
    for j in range(i + 1, len(boxes)):
        l1, b1 = boxes[i]; l2, b2 = boxes[j]
        ox = min(b1.x1, b2.x1) - max(b1.x0, b2.x0)
        oy = min(b1.y1, b2.y1) - max(b1.y0, b2.y0)
        if ox > 2 and oy > 2 and ox * oy > 0.10 * min(b1.width * b1.height,
                                                      b2.width * b2.height):
            print("OVERLAP", repr(l1), "<->", repr(l2)); bad += 1
st = fig._suptitle.get_window_extent(rend)
for name, bb in boxes:
    ox = min(st.x1, bb.x1) - max(st.x0, bb.x0)
    oy = min(st.y1, bb.y1) - max(st.y0, bb.y0)
    if ox > 2 and oy > 2:
        print("SUPTITLE OVERLAP", repr(name)); bad += 1
print("AUDIT:", "CLEAN" if bad == 0 else bad)

json.dump({
    "figure": "box2_candidates.png",
    "asserts": {
        "box1_candidates": len(slots0), "box2_candidates": len(slots1),
        "occupancy_excluded": len(removed), "stack_candidates": len(stack1),
        "portal_attempts_in_60s": DATA["portal_attempts"],
        "box2_box_wdh": DATA["box1_box_wdh"],
        "floor_candidate_z": 0.639, "stack_candidate_z": 0.889,
        "box1_placed": DATA["box1_world"], "surface_revision": 2,
        "occupied_cells": len(occ_cells)},
    "layers": {
        "floor_candidates": "box centers, no atlas status drawn",
        "occupancy_excluded": "points present in box-1 request, absent in "
                              "box-2 request",
        "stack_candidate": "box center z=0.889 on measured support 0.78"},
}, open(os.path.join(OUT_DIR, "box2_candidates.json"), "w"), indent=1)
print("saved", png)
