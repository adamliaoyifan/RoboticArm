#!/usr/bin/env python3
"""score_run02 box-2: where the search went and why it failed.

(A) top view — all 582 candidates, the 17 attempted slots in visit order
    (all in the deepest corner cluster, boxed), box-1 occupancy;
(B) budget timeline — per-attempt durations and outcomes inside the 60 s;
(C) depth distribution — attempted cluster vs all candidates.
"""
import json
import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
import numpy as np

DATA = json.load(open("/tmp/box2_run02_data.json"))
EVID = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
        "master_pack_recovery/2026-10-14_g3")
OUT = os.path.join(EVID, "wave4", "figures")

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, AQUA, CRIT = "#2a78d6", "#1baf7a", "#d03b3b"
NEUTRAL = "#efeeea"
PLATE = dict(facecolor="white", edgecolor="none", alpha=0.9, pad=2.0)

slots = {int(k): v for k, v in DATA["slots"].items()}
attempted = DATA["attempted"]
occ = DATA["occupied"]
res = DATA["res"]

# timeline from launch.log (extracted): (index, dur_s, outcome)
timeline = [
    (580, 7.87, "miss"), (579, 5.31, "miss"), (578, 3.05, "miss"),
    (574, 1.51, "miss"), (573, 1.49, "miss"), (577, 1.54, "miss"),
    (572, 1.46, "miss"), (576, 2.93, "miss"), (571, 1.48, "miss"),
    (575, 0.61, "miss"), (570, 1.44, "miss"), (569, 1.25, "miss"),
    (565, 1.05, "miss"), (568, 6.11, "miss"), (564, 1.33, "miss"),
    (567, 4.71, "budget_cut"), (563, 1.21, "miss")]

fig = plt.figure(figsize=(13.8, 6.6), dpi=140)
gs = fig.add_gridspec(1, 3, width_ratios=[2.3, 1.35, 1.0], wspace=0.36,
                      left=0.045, right=0.982, top=0.87, bottom=0.13)
axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])
axC = fig.add_subplot(gs[0, 2])

# ---------------------------------------------------------------- A top view
HLX, HLY = 0.745, 0.985
axA.add_patch(Rectangle((-HLX, -HLY), 2 * HLX, 2 * HLY, fill=False,
                        edgecolor=INK, lw=1.4, zorder=3))
axA.add_patch(Polygon([(HLX, 0.55), (HLX, HLY), (0.55, HLY)], closed=True,
                      facecolor=NEUTRAL, edgecolor=INK2, lw=0.8,
                      ls=(0, (3, 2)), zorder=1))
axA.plot([-HLX, -HLX], [-0.928, 0.398], color=INK, lw=3.2,
         solid_capstyle="butt", zorder=4)
axA.plot([-HLX, -HLX], [-HLY, -0.928], color=INK, lw=1.4, zorder=4)
axA.plot([-HLX, -HLX], [0.398, HLY], color=INK, lw=1.4, zorder=4)
for cx, cy in occ:
    axA.add_patch(Rectangle((cx - res / 2, cy - res / 2), res, res,
                            facecolor="#86b6ef", edgecolor=BLUE, lw=0.3,
                            zorder=2))
all_xy = [(v[0], v[1]) for v in slots.values()]
axA.scatter([p[0] for p in all_xy], [p[1] for p in all_xy], s=5, c=INK2,
            alpha=0.45, lw=0, zorder=3,
            label="box-2 candidates (582; 581 floor + 1 stack)")
# attempted cluster zone
axA.add_patch(Rectangle((0.38, -0.12), 0.20, 0.37, fill=False,
                        edgecolor=CRIT, lw=1.6, ls=(0, (4, 2)), zorder=5))
for order, idx in enumerate(attempted):
    x, y, _z, _yaw = slots[idx]
    axA.scatter([x], [y], s=42, marker="x", c=CRIT, lw=1.6, zorder=6)
    if order % 6 == 0:
        dy = 4 if (order // 6) % 2 == 0 else -10
        axA.annotate(str(idx), (x, y), xytext=(3, dy),
                     textcoords="offset points", fontsize=6.5, color=INK,
                     zorder=7)
axA.text(1.03, 0.30, "attempted slots (17)\nred X, index labels\nALL inside the\nboxed deepest-corner\ncluster 0.43..0.53 x\n-0.06..0.19 (m)",
         transform=axA.transAxes, fontsize=8.5, color=INK, ha="left",
         bbox=dict(PLATE, edgecolor=CRIT, lw=1.1))
axA.text(1.03, 0.72, "box 1 placed\n(BLUE cells,\ncentroid 0.53,-0.51)",
         transform=axA.transAxes, fontsize=8.5, color=INK, ha="left",
         bbox=dict(PLATE, edgecolor=BLUE, lw=1.1))
axA.annotate("opening (west, x=-0.745)", xy=(-0.70, -0.75), fontsize=8,
             color=INK2, rotation=90, ha="left", va="bottom")
axA.annotate("deepest wall (east)", xy=(0.70, 0.55), fontsize=8, color=INK2,
             rotation=90, ha="right", va="top")
axA.set_xlim(-HLX - 0.12, HLX + 0.12)
axA.set_ylim(-HLY - 0.12, HLY + 0.12)
axA.set_aspect("equal")
axA.set_xlabel("container_link x [m]", fontsize=9)
axA.set_ylabel("container_link y [m]", fontsize=9)
axA.set_title("(A) where the search went — 17 attempts, one corner",
              fontsize=10, color=INK)
axA.legend(loc="upper center", bbox_to_anchor=(0.5, -0.085), ncol=2,
           fontsize=8, frameon=False)

# ---------------------------------------------------------------- B timeline
t = 0.0
for idx, dur, outcome in timeline:
    color = CRIT if outcome == "miss" else INK2
    axB.barh(idx, dur, left=t, height=0.72, color=color)
    t += dur
axB.axvline(60.0, color=INK, lw=1.2, ls=(0, (4, 2)))
axB.text(60.0, 582.5, " budget 60 s", fontsize=8, color=INK, va="top")
axB.set_ylim(560, 584)
axB.set_xlim(0, 66)
axB.set_xlabel("route-budget time [s] (cumulative)", fontsize=9)
axB.set_ylabel("candidate index (visit order bottom→top)", fontsize=9)
axB.set_title("(B) budget eaten by 16 sampler misses\nportal-edge OMPL found no valid state (1 redraw each)",
              fontsize=10, color=INK)
axB.invert_yaxis()
axB.text(2, 561, "total planned: 43.7 s\n+ anchor screens/overhead -> 60 s\ndurations 0.6-7.9 s/attempt\n(offline isolated stack: <1 s/attempt,\nfirst-try success 10-16 s)",
         fontsize=8, color=INK, bbox=PLATE, va="bottom")

# ---------------------------------------------------------------- C depth
depths_all = np.array([0.745 + v[0] for v in slots.values()])
depths_att = np.array([0.745 + slots[i][0] for i in attempted])
axC.hist(depths_all, bins=np.arange(0.0, 1.45, 0.1), color=NEUTRAL,
         edgecolor=INK2, lw=0.6, label="all candidates")
axC.hist(depths_att, bins=np.arange(0.0, 1.45, 0.1), color=CRIT,
         edgecolor=INK, lw=0.6, label="attempted (17)")
axC.set_xlabel("opening depth of candidate [m]", fontsize=9)
axC.set_ylabel("count", fontsize=9)
axC.legend(fontsize=8, frameon=False, loc="upper left")
axC.set_title("(C) attempts concentrated at\nthe deepest 1.18-1.28 m band",
              fontsize=10, color=INK)

fig.suptitle("score_run02 box-2: atlas healthy (seeded=582) yet budget-exhausted — "
             "the seeded order spent all 17 attempts on one deepest-corner "
             "cluster and every portal sample missed",
             fontsize=11, color=INK, y=0.97)

png = os.path.join(OUT, "box2_run02_attempts.png")
fig.savefig(png)
fig.canvas.draw()
rend = fig.canvas.get_renderer()
bad = 0
boxes = []
for ax_ in fig.axes:
    for tt in ax_.texts:
        if tt.get_text().strip():
            boxes.append((tt.get_text().split("\n")[0],
                          tt.get_window_extent(rend)))
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
    "figure": "box2_run02_attempts.png",
    "asserts": {
        "request_slots": 582, "attempted": 17, "sampler_miss": 16,
        "budget_cut_mid_attempt": 1,
        "attempted_region_container": "x 0.43..0.53, y -0.06..0.19",
        "attempted_depth_band_m": "1.18..1.28 (deepest)",
        "box1_centroid": [0.53, -0.51],
        "durations_s": [d for _, d, _ in timeline],
        "atlas": "seeded=582 online (healthy)",
        "failure_mode": "portal-edge OMPL sampler found no valid state "
                       "(one seeded redraw per candidate), all in one "
                       "deepest-corner cluster",
    },
}, open(os.path.join(OUT, "box2_run02_attempts.json"), "w"), indent=1)
print("saved", png)
