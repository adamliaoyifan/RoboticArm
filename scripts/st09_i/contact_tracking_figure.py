"""Figure for contact_tracking_analysis."""
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, CRIT = "#2a78d6", "#d03b3b"
AQUA = "#1baf7a"


def render(result, bag, contacts, follow, path):
    ct = bag["ct"]
    if not len(ct):
        return
    err = np.abs(bag["ref"][:, 1] - bag["fb"][:, 1])
    fig, axes = plt.subplots(2, 1, figsize=(14.5, 7.2), sharex=False)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.08, hspace=0.38)
    ax = axes[0]
    ax.plot(ct, err, color=INK, lw=1.2, label="|joint2 reference - actual|")
    ax.axhline(0.03, color=INK2, lw=0.8, ls="--", label="0.03 rad")
    colors = {"suction_interface": AQUA, "robot_body": CRIT, "other": BLUE}
    for klass, color in colors.items():
        ts = [c["stamp"] for c in contacts if c.get("klass") == klass]
        if not ts:
            continue
        ax.plot(ts, np.interp(ts, ct, err), "o", ms=3, color=color, label=klass)
    ax.set_ylabel("rad")
    ax.set_title("A  joint2 error and contact times", loc="left", fontsize=10, color=INK)
    ax.legend(fontsize=7.5, frameon=False, loc="upper right", ncol=2)
    bx = axes[1]
    if result.get("legs"):
        names, vals = [], []
        for leg in result["legs"]:
            for klass, color in (("suction_interface", AQUA), ("robot_body", CRIT)):
                frac = leg["contacts"][klass]["fraction_of_undertrack_samples"]
                if frac is None:
                    continue
                names.append("%s\n%s" % (leg["name"], klass.replace("_", " ")))
                vals.append(frac)
        if names:
            bx.bar(range(len(vals)), vals,
                   color=[CRIT if "robot" in n else AQUA for n in names])
            bx.set_xticks(range(len(names)))
            bx.set_xticklabels(names, fontsize=7)
            bx.set_ylim(0, 1.05)
            bx.axhline(0.5, color=INK2, lw=0.8, ls="--")
            for i, v in enumerate(vals):
                bx.text(i, v + 0.02, "%.2f" % v, ha="center", fontsize=7, color=INK)
    bx.set_ylabel("fraction of under-track samples")
    bx.set_title("B  contact class present while |joint2 error| > 0.03 rad",
                 loc="left", fontsize=10, color=INK)
    fig.suptitle(
        "static-box contact vs joint2 under-tracking  |  contacts %d, follow ticks %d"
        % (result.get("contact_rows", 0), result.get("follow_ticks", 0)),
        fontsize=11, color=INK, y=0.97)
    fig.canvas.draw()
    rend = fig.canvas.get_renderer()
    bad = 0
    for ax in fig.axes:
        boxes = [(t.get_text().split("\n")[0], t.get_window_extent(rend))
                 for t in ax.texts if t.get_text().strip()]
        for label, bb in boxes:
            if bb.x0 < 1 or bb.x1 > fig.bbox.x1 - 1 or bb.y0 < 1 or bb.y1 > fig.bbox.y1 - 1:
                print("CLIP", repr(label))
                bad += 1
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                (l1, b1), (l2, b2) = boxes[i], boxes[j]
                ox = min(b1.x1, b2.x1) - max(b1.x0, b2.x0)
                oy = min(b1.y1, b2.y1) - max(b1.y0, b2.y0)
                if ox > 2 and oy > 2 and ox * oy > 0.1 * min(
                        b1.width * b1.height, b2.width * b2.height):
                    print("OVERLAP", repr(l1), repr(l2))
                    bad += 1
    print("AUDIT:", "CLEAN" if bad == 0 else bad)
    fig.savefig(path, dpi=140)
    plt.close(fig)
