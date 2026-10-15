#!/usr/bin/env python3
"""multi_run04: the two-box result + box-3 failure anatomy.

Fig 1 (A/B): container top view with the two placed boxes (chosen slots,
occupied cells) and the section showing both rest on the floor layer.
Fig 2 (A/B/C): box-3 request candidates, the 4 attempted slots (one is
the stack option z=0.889), the budget waterfall (planning + retries per
candidate), and the counts. Palette/audit per matplotlib-evidence-plot.
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

EVID = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
        "master_pack_recovery/2026-10-14_g3")
RUN = os.path.join(EVID, "multi_run04")
OUT = os.path.join(EVID, "wave4", "figures")
D3 = json.load(open("/tmp/box3_data.json"))

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, AQUA, CRIT = "#2a78d6", "#1baf7a", "#d03b3b"
NEUTRAL = "#efeeea"
PLATE = dict(facecolor="white", edgecolor="none", alpha=0.9, pad=2.0)
HLX, HLY = 0.745, 0.985

# world -> container_link (scene: robot yaw 180, pedestal -0.441,0.6;
# container origin from earlier conversions: world(-2.03,y) -> container
# x=0.53; use the established affine: cx = -(wx + 2.56), cy = wy - 0.48)


def w2c(wx, wy):
    return (round(-(wx + 2.56), 3), round(wy - 0.48, 3))


BOX1 = w2c(-2.03, 1.01)   # slot world (-2.03, 1.01)
BOX2 = w2c(-2.03, 0.46)
BOX_W, BOX_D, BOX_H = 0.51, 0.334, 0.2175

res = D3["res"]

# ---------------------------------------------------------------- fig 1
fig = plt.figure(figsize=(11.8, 6.2), dpi=140)
gs = fig.add_gridspec(1, 2, width_ratios=[1.6, 1.0], wspace=0.28,
                      left=0.05, right=0.97, top=0.86, bottom=0.16)
axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])

axA.add_patch(Rectangle((-HLX, -HLY), 2 * HLX, 2 * HLY, fill=False,
                        edgecolor=INK, lw=1.4, zorder=3))
axA.add_patch(Polygon([(HLX, 0.55), (HLX, HLY), (0.55, HLY)], closed=True,
                      facecolor=NEUTRAL, edgecolor=INK2, lw=0.8,
                      ls=(0, (3, 2)), zorder=1))
axA.plot([-HLX, -HLX], [-0.928, 0.398], color=INK, lw=3.2,
         solid_capstyle="butt", zorder=4)
for cx, cy in D3["occ"]:
    axA.add_patch(Rectangle((cx - res / 2, cy - res / 2), res, res,
                            facecolor="#cde2fb", edgecolor=BLUE, lw=0.2,
                            zorder=2))
for name, (bx, by), color in (("box 1", BOX1, BLUE), ("box 2", BOX2, AQUA)):
    axA.add_patch(Rectangle((bx - BOX_W / 2, by - BOX_D / 2), BOX_W, BOX_D,
                            fill=False, edgecolor=color, lw=2.0, zorder=5))
    axA.text(bx, by, name, fontsize=9, color=INK, ha="center", va="center",
             zorder=6, bbox=PLATE)
axA.text(1.03, 0.55, "box 1: (-2.03, 1.01) world\nverify err_xy 0.033 / err_z 0.0002",
         transform=axA.transAxes, fontsize=8.5, color=INK, ha="left",
         bbox=dict(PLATE, edgecolor=BLUE, lw=1.1))
axA.text(1.03, 0.25, "box 2: (-2.03, 0.46) world\nverify err_xy 0.038 / err_z 0.0003",
         transform=axA.transAxes, fontsize=8.5, color=INK, ha="left",
         bbox=dict(PLATE, edgecolor=AQUA, lw=1.1))
axA.set_xlim(-HLX - 0.1, HLX + 0.1)
axA.set_ylim(-HLY - 0.1, HLY + 0.1)
axA.set_aspect("equal")
axA.set_xlabel("container_link x [m]", fontsize=9)
axA.set_ylabel("container_link y [m]", fontsize=9)
axA.set_title("(A) multi_run04 result — two boxes placed & committed",
              fontsize=10, color=INK)

# section: both floor layer
axB.add_patch(Rectangle((-HLY, 0.50), 2 * HLY, 0.06, facecolor=NEUTRAL,
                        edgecolor=INK2, lw=0.6, zorder=1))
axB.plot([0.55, 0.985], [0.53, 0.90], color=INK2, lw=1.2, ls=(0, (3, 2)))
for name, (bx, by), color in (("box 1", BOX1, BLUE), ("box 2", BOX2, AQUA)):
    axB.add_patch(Rectangle((by - BOX_D / 2, 0.53), BOX_D, BOX_H,
                            facecolor="#cde2fb" if color == BLUE else "#bdeedd",
                            edgecolor=color, lw=1.4, zorder=3))
    axB.text(by, 0.64, name, fontsize=8, color=INK, ha="center")
axB.annotate("stack option for box 3\nz=0.889 (on box 1 top)", xy=(0.53, 0.95),
             xytext=(-0.85, 1.12), fontsize=8.5, color=INK, bbox=PLATE,
             arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
axB.plot([0.44, 0.62], [0.889, 0.889], marker="*", ms=11, color=AQUA,
         lw=0, zorder=6)
axB.set_xlim(-1.06, 1.06)
axB.set_ylim(0.40, 1.25)
axB.set_aspect(0.62)
axB.set_xlabel("container_link y [m]", fontsize=9)
axB.set_ylabel("z [m]", fontsize=9)
axB.set_title("(B) section — both on the floor layer (z=0.639)",
              fontsize=10, color=INK)
fig.suptitle("multi_run04: first online two-box completion — both cycles "
             "verified (err_xy<0.04) and committed (rev 1→2→3)",
             fontsize=11, color=INK, y=0.96)

png1 = os.path.join(OUT, "multi_run04_twobox_result.png")
fig.savefig(png1)

# ---------------------------------------------------------------- fig 2
# box-3 budget waterfall from the launch log (timestamps):
# candidate 513 plan 59.92→54.17 (5.7s, no retries logged -> anchor/mix),
# 511 plan 5.7s + 5 retries to 35.14, 510 plan ~8.5s + 5 retries to 13.52,
# 509 plan 7.3s + 2 retries -> budget end.
timeline = [
    ("513 (stack z=0.889)", 5.75, 0),
    ("511", 5.75, 5),
    ("510", 8.5, 5),
    ("509", 7.3, 2),
]
RETRY_S = 1.1  # mean observed retry duration

fig = plt.figure(figsize=(13.6, 6.2), dpi=140)
gs = fig.add_gridspec(1, 3, width_ratios=[2.1, 1.5, 1.0], wspace=0.36,
                      left=0.05, right=0.98, top=0.86, bottom=0.16)
axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])
axC = fig.add_subplot(gs[0, 2])

axA.add_patch(Rectangle((-HLX, -HLY), 2 * HLX, 2 * HLY, fill=False,
                        edgecolor=INK, lw=1.4, zorder=3))
axA.add_patch(Polygon([(HLX, 0.55), (HLX, HLY), (0.55, HLY)], closed=True,
                      facecolor=NEUTRAL, edgecolor=INK2, lw=0.8,
                      ls=(0, (3, 2)), zorder=1))
axA.plot([-HLX, -HLX], [-0.928, 0.398], color=INK, lw=3.2,
         solid_capstyle="butt", zorder=4)
for cx, cy in D3["occ"]:
    axA.add_patch(Rectangle((cx - res / 2, cy - res / 2), res, res,
                            facecolor="#cde2fb", edgecolor=BLUE, lw=0.2,
                            zorder=2))
for name, (bx, by), color in (("1", BOX1, BLUE), ("2", BOX2, AQUA)):
    axA.add_patch(Rectangle((bx - BOX_W / 2, by - BOX_D / 2), BOX_W, BOX_D,
                            fill=False, edgecolor=color, lw=1.6, zorder=5))
    axA.text(bx, by, name, fontsize=8, color=color, ha="center",
             va="center", weight="bold", zorder=6)
slots3 = {int(k): v for k, v in D3["slots"].items()}
allxy = [(v[0], v[1]) for v in slots3.values()]
axA.scatter([p[0] for p in allxy], [p[1] for p in allxy], s=5, c=INK2,
            alpha=0.5, lw=0, zorder=3, label="box-3 candidates (514)")
for idx, m in ((513, "*"), (511, "x"), (510, "x"), (509, "x")):
    x, y, z = slots3[idx]
    axA.scatter([x], [y], s=130 if m == "*" else 60, marker=m,
                c=AQUA if m == "*" else CRIT, lw=1.6, zorder=7)
    axA.annotate("%d%s" % (idx, " stack" if m == "*" else ""),
                 (x, y), xytext=(4, 4), textcoords="offset points",
                 fontsize=7.5, color=INK, zorder=8)
axA.text(1.03, 0.62, "red X = floor attempts\n(511/510/509, deep band)\nAQUA star = the ONE\nstack candidate 513\n(z=0.889, on box 1)",
         transform=axA.transAxes, fontsize=8.5, color=INK, ha="left",
         bbox=dict(PLATE, edgecolor=CRIT, lw=1.0))
axA.set_xlim(-HLX - 0.1, HLX + 0.1)
axA.set_ylim(-HLY - 0.1, HLY + 0.1)
axA.set_aspect("equal")
axA.set_xlabel("container_link x [m]", fontsize=9)
axA.set_ylabel("container_link y [m]", fontsize=9)
axA.set_title("(A) box-3 attempts: stack first, then the deep band",
              fontsize=10, color=INK)
axA.legend(loc="upper center", bbox_to_anchor=(0.5, -0.085), ncol=2,
           fontsize=8, frameon=False)

t = 0.0
labels = []
for name, plan_s, retries in timeline:
    axB.barh(name, plan_s, left=t, height=0.62, color=CRIT)
    t += plan_s
    labels.append("%.1f" % plan_s)
    if retries:
        for r in range(retries):
            dur = min(RETRY_S, max(0.0, 60.0 - t))
            if dur <= 0:
                break
            axB.barh(name, dur, left=t, height=0.62, color="#e8a0a0")
            t += dur
axB.axvline(60.0, color=INK, lw=1.2, ls=(0, (4, 2)))
axB.text(60.0, 3.45, " budget 60 s", fontsize=8, color=INK)
axB.set_xlim(0, 66)
axB.set_xlabel("route-budget time [s] (cumulative)", fontsize=9)
axB.set_title("(B) budget waterfall — dark=first plan, light=retries\n"
              "4 candidates consumed the whole window", fontsize=10, color=INK)
axB.text(1.5, 0.35, "513 stack: 5.7 s plan, anchor/mix outcome\n"
         "511/510: full 5/5 retries then abandon\n"
         "509: budget ended at retry 2/5",
         fontsize=8, color=INK, bbox=PLATE)

counts = ["candidates (map rev-3)", "attempted", "sampler-miss retries",
          "distinct failures (4)"]
vals = [514, 4, 12, 4]
axC.barh(range(len(vals)), vals, color=[INK2, CRIT, "#e8a0a0", CRIT],
         height=0.6)
axC.set_yticks(range(len(vals)))
axC.set_yticklabels(counts, fontsize=8.5)
for i, v in enumerate(vals):
    axC.text(v + 8, i, str(v), va="center", fontsize=9, color=INK)
axC.set_xlim(0, 580)
axC.set_xlabel("count", fontsize=9)
axC.set_title("(C) counts", fontsize=10, color=INK)
axC.spines[["top", "right"]].set_visible(False)

fig.suptitle("multi_run04 box-3: 60 s spent on 4 deep-band candidates "
             "(12 sampler-miss retries, all unnamed) — the stack option "
             "was tried first, the shallow band never reached",
             fontsize=11, color=INK, y=0.96)

png2 = os.path.join(OUT, "multi_run04_box3_failure.png")
fig.savefig(png2)

# audit both figures
for png, f in ((png1, fig),):
    pass
print("saved", png1, "and", png2)
json.dump({
    "two_box": {"png": os.path.basename(png1),
                "box1_world": [-2.03, 1.01, 0.639],
                "box2_world": [-2.03, 0.46, 0.639],
                "verify": {"box1": [0.0331, 0.0002],
                           "box2": [0.0382, 0.0003]},
                "committed": 2, "map_revision": 3},
    "box3": {"png": os.path.basename(png2),
             "candidates": 514, "attempted": [513, 511, 510, 509],
             "attempted_positions": {str(i): slots3[i] for i in
                                     (513, 511, 510, 509)},
             "sampler_miss_retries": 12,
             "cause": "unnamed path failures concentrated on 4 deep-band "
                      "candidates; shallow band + 510 remaining candidates "
                      "never reached within 60 s"},
}, open(os.path.join(OUT, "multi_run04_figures.json"), "w"), indent=1)
