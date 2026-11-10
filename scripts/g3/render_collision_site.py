#!/usr/bin/env python3
"""Where exactly the portal path collides with the container.

Two captured failing PORTAL_TRANSIT paths (isolated replay, exact online
goal/scene/attachment) are forward-kinematiced sample by sample; the
samples from first_invalid onward are drawn red with the payload
footprint at the first invalid sample. Panels: XY top view + XZ section.
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

import sys
sys.path.insert(0, "/tmp/elfin-master-pack-baseline-g3/src/luggage_planning")
sys.path.insert(0, "/tmp/elfin-master-pack-baseline-g3/src/luggage_description")
from luggage_planning.offline_ik import load_urdf
from luggage_planning.reachable_graph import make_offline_fk

WS = "/tmp/elfin-master-pack-baseline-g3"
EVID = ("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
        "master_pack_recovery/2026-10-14_g3")
DATA = json.load(open(os.path.join(
    EVID, "wave4", "portal_edge_paths", "edge_replay.json")))
OUT = os.path.join(EVID, "wave4", "figures")

urdf = load_urdf(
    xacro_path=os.path.join(
        WS, "install/luggage_description/share/luggage_description/urdf/"
        "elfin_s20_with_camera.urdf.xacro"),
    scene_tf_config=os.path.join(
        WS, "src/luggage_description/config/scene_tf_deploy_update.yaml"))
fk = make_offline_fk(urdf, urdf.path_to("suction_contact_frame"))

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, AQUA, CRIT = "#2a78d6", "#1baf7a", "#d03b3b"
NEUTRAL = "#efeeea"
PLATE = dict(facecolor="white", edgecolor="none", alpha=0.9, pad=2.0)

PAYLOAD_H = 0.2175  # box half-height; box hangs below the TCP by h/2+gap

fig = plt.figure(figsize=(13.2, 6.4), dpi=140)
gs = fig.add_gridspec(1, 2, width_ratios=[1.5, 1.1], wspace=0.27,
                      left=0.05, right=0.97, top=0.86, bottom=0.15)
axA = fig.add_subplot(gs[0, 0])
axB = fig.add_subplot(gs[0, 1])

# container geometry in world frame (from scene_tf: opening west face at
# x=-0.745 world... but the actual world coords from the run: opening
# plane x=-0.745 is container-frame; the run showed slots at world
# x=-2.03 = container x 0.53, so container x=-0.745 -> world -2.83,
# i.e. world = container_x - 2.56 shifted; container half-y 0.985 about
# world y~0.48 per w2c. Draw container in world frame:
# world frame: opening (east face) at x=-0.745; interior extends west;
# container y band approx -0.94..1.03 (scene_tf aperture y -0.928..0.398
# sits inside it); opening z band 0.597..2.009.
CX0, CX1 = -2.235, -0.745          # interior x (west wall .. opening face)
CY0, CY1 = -0.94, 1.03             # interior y band
OPEN_Y0, OPEN_Y1 = -0.928, 0.398   # aperture y band (world)
FLOOR_Z, CEIL_Z = 0.53, 2.009

for ax, xl, yl in ((axA, "world x [m]", "world y [m]"),
                   (axB, "world x [m]", "world z [m]")):
    pass

axA.add_patch(Rectangle((CX0, CY0), CX1 - CX0, CY1 - CY0, fill=False,
                        edgecolor=INK, lw=1.4, zorder=3))
axA.plot([CX1, CX1], [OPEN_Y0, OPEN_Y1], color=INK, lw=3.4,
         solid_capstyle="butt", zorder=4, label="opening (east face)")
axA.plot([CX1, CX1], [CY0, OPEN_Y0], color=INK, lw=1.4, zorder=4)
axA.plot([CX1, CX1], [OPEN_Y1, CY1], color=INK, lw=1.4, zorder=4)
# box1/box2 world positions (multi_run04)
for (bx, by), lab in (((-2.03, 1.01), "box1"), ((-2.03, 0.46), "box2")):
    axA.add_patch(Rectangle((bx - 0.255, by - 0.167), 0.51, 0.334,
                            fill=False, edgecolor=BLUE, lw=1.4, zorder=3))
    axA.text(bx, by, lab, fontsize=7.5, color=BLUE, ha="center",
             va="center")

styles = [(4, "-"), (9, "--")]
for attempt, ls in styles:
    a = next(x for x in DATA["attempts"] if x["attempt"] == attempt)
    pts = [fk(q)[0] for q in a["joint_path"]]  # (pos, quat)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    fi = a.get("first_invalid_sample", 0)
    color_ok = INK2 if attempt == 4 else BLUE
    axA.plot(xs[:fi + 1], ys[:fi + 1], ls=ls, color=color_ok, lw=1.4,
             zorder=5)
    axA.plot(xs[fi:], ys[fi:], ls=ls, color=CRIT, lw=2.0, zorder=6)
    axB.plot(xs[:fi + 1], zs[:fi + 1], ls=ls, color=color_ok, lw=1.4,
             zorder=5)
    axB.plot(xs[fi:], zs[fi:], ls=ls, color=CRIT, lw=2.0, zorder=6)
    # payload footprint at the first invalid sample (TCP -> box center
    # hangs below by h/2)
    cx, cy, cz = xs[fi], ys[fi], zs[fi] - PAYLOAD_H
    contact = (a.get("contacts") or [["?", "?"]])[0]
    axA.annotate(
        "att %d first invalid #%d\n%s × %s" % (
            attempt, fi, contact[0], contact[1]),
        xy=(cx, cy), xytext=(0.97 if attempt == 4 else 0.97,
                             0.80 if attempt == 4 else 0.60),
        textcoords="axes fraction", fontsize=8.5, color=INK, bbox=PLATE,
        arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    axB.annotate("#%d" % fi, xy=(cx, cz), xytext=(4, 6),
                 textcoords="offset points", fontsize=8, color=CRIT)

axB.axhspan(FLOOR_Z, FLOOR_Z + 0.02, color=NEUTRAL, zorder=1)
axB.add_patch(Rectangle((CX0, FLOOR_Z), CX1 - CX0, 0.02, facecolor=NEUTRAL,
                        edgecolor=INK2, lw=0.6, zorder=1))
axB.plot([CX0, CX0], [FLOOR_Z, CEIL_Z], color=INK, lw=1.4)
axB.plot([CX0, CX1], [CEIL_Z, CEIL_Z], color=INK, lw=1.4)
axB.plot([CX1, CX1], [FLOOR_Z, 0.597], color=INK, lw=1.6)
axB.plot([CX1, CX1], [2.009, CEIL_Z], color=INK, lw=1.6)
axB.plot([CX1, CX1], [0.597, 2.009], color=INK, lw=3.4,
         solid_capstyle="butt", label="opening band")
axB.plot([CX0, CX1], [CEIL_Z, CEIL_Z], color=INK, lw=1.4)

axA.set_xlim(-2.6, 0.75)
axA.set_ylim(-1.25, 1.75)
axA.set_aspect("equal")
axA.set_xlabel("world x [m]", fontsize=9)
axA.set_ylabel("world y [m]", fontsize=9)
axA.set_title("(A) top view — TCP path; red = invalid samples", fontsize=10,
              color=INK)
axA.legend(loc="lower left", fontsize=8, frameon=False)

axB.set_xlim(-2.6, 0.75)
axB.set_ylim(0.2, 2.25)
axB.set_aspect(1.0)
axB.set_xlabel("world x [m]", fontsize=9)
axB.set_ylabel("world z [m]", fontsize=9)
axB.set_title("(B) XZ section — where the arm dips into the wall",
              fontsize=10, color=INK)

fig.suptitle("Failing portal paths (exact online goal/scene): the red "
             "segments collide OUTSIDE the container — on the east-face "
             "rim beside the opening (mm-scale grazes: camera mount / "
             "payload corner)", fontsize=11, color=INK, y=0.965)

png = os.path.join(OUT, "portal_collision_site.png")
fig.savefig(png)
fig.canvas.draw()
rend = fig.canvas.get_renderer()
bad = 0
boxes = []
for ax_ in fig.axes:
    for t in ax_.texts:
        if t.get_text().strip():
            boxes.append((t.get_text().split("\n")[0],
                          t.get_window_extent(rend)))
for name, bb in boxes:
    if bb.x0 < 1 or bb.x1 > fig.bbox.x1 - 1 or bb.y0 < 1 or bb.y1 > fig.bbox.y1 - 1:
        print("CLIP", repr(name)); bad += 1
print("AUDIT:", "CLEAN" if bad == 0 else bad)
json.dump({
    "figure": "portal_collision_site.png",
    "paths": {str(a["attempt"]): {
        "n_waypoints": a["n_waypoints"],
        "first_invalid_sample": a.get("first_invalid_sample"),
        "contacts": a.get("contacts")} for a in DATA["attempts"]
        if a.get("joint_path")},
}, open(os.path.join(OUT, "portal_collision_site.json"), "w"), indent=1)
print("saved", png)
