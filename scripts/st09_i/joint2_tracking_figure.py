"""Evidence figure for joint2_tracking_analysis (matplotlib, Agg)."""
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

INK, INK2 = "#0b0b0b", "#52514e"
BLUE, AQUA, CRIT = "#2a78d6", "#1baf7a", "#d03b3b"
GREY = "#8c8b87"
PHASE_WASH = {"accel": "#cde2fb", "cruise": "#86b6ef", "decel": "#cde2fb",
              "hold": "#efeeea"}
JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))


def _shade(ax, bounds, hold_end, label_y=None):
    ranges = dict(bounds)
    ranges["hold"] = (bounds["decel"][1], hold_end)
    for name in ("accel", "cruise", "decel", "hold"):
        a, b = ranges[name]
        ax.axvspan(a, b, color=PHASE_WASH[name], alpha=0.16, lw=0)
        if label_y is not None:
            ax.text((a + b) / 2, label_y, name, ha="center", va="top",
                    fontsize=8, color=INK2,
                    transform=ax.get_xaxis_transform())


def render(result, bag, ctx, path):
    ct, fb, ref = bag["ct"], bag["fb"], bag["ref"]
    t0, ref_end = ctx["t0"], ctx["ref_end"]
    bounds, hold_end = ctx["bounds"], ctx["hold_end"]
    ref_fn, common_tau, gain = ctx["ref_fn"], ctx["common_tau"], ctx["gain"]
    lo, hi = t0 - 0.3, hold_end + 0.4
    m = (ct >= lo) & (ct <= hi)
    t = ct[m]
    fig, axes = plt.subplots(2, 3, figsize=(17.5, 9.4))
    fig.subplots_adjust(left=0.05, right=0.985, top=0.88, bottom=0.09,
                        wspace=0.27, hspace=0.36)
    a, b, c, d, e, f = axes.ravel()

    # A: joint2 reference / delayed reference / actual
    _shade(a, bounds, hold_end, 0.985)
    a.plot(t, ref_fn(t, 1), color=BLUE, lw=1.8, label="reference")
    a.plot(t, ref_fn(t - common_tau, 1), color=BLUE, lw=1.0, ls="--",
           label="reference delayed %.3f s (joints 1,4,6 fit)" % common_tau)
    a.plot(t, fb[m][:, 1], color=INK, lw=1.8,
           label="actual (= /joint_states)")
    a.set_title("A  joint2 position", loc="left", fontsize=10, color=INK)
    a.set_ylabel("rad")
    a.legend(fontsize=7.5, loc="lower left", frameon=False)

    # B: residual after common-tau compensation, joint2 vs healthy joints
    _shade(b, bounds, hold_end, 0.985)
    for j, col in ((0, AQUA), (3, GREY), (5, "#9aa6b2")):
        b.plot(t, fb[m][:, j] - ref_fn(t - common_tau, j), color=col, lw=1.1,
               label=JOINTS[j].replace("elfin_", ""))
    b.plot(t, fb[m][:, 1] - ref_fn(t - common_tau, 1), color=INK, lw=2.0,
           label="joint2")
    b.axhline(0, color=INK2, lw=0.6)
    b.set_title("B  residual after %.3f s compensation" % common_tau,
                loc="left", fontsize=10, color=INK)
    b.set_ylabel("actual - delayed reference, rad")
    b.legend(fontsize=7.5, loc="center left", frameon=False, ncol=1)

    # C: delivered speed vs model demand
    vel = np.full_like(fb, np.nan)
    half = 5
    vel[half:-half] = (fb[2 * half:] - fb[:-2 * half]) / (
        ct[2 * half:] - ct[:-2 * half])[:, None]
    _shade(c, bounds, hold_end, 0.985)
    demand = gain * np.abs((ref - fb)[:, 1])
    c.plot(t, demand[m], color=BLUE, lw=1.4,
           label="model demand K*|e|, K=%.1f/s" % gain)
    c.plot(t, np.abs(vel[m][:, 1]), color=INK, lw=2.0,
           label="delivered |speed| (position difference)")
    c.plot(t, np.abs(vel[m][:, 0]), color=AQUA, lw=1.0,
           label="joint1 delivered (follows its demand)")
    c.set_yscale("log")
    c.set_ylim(3e-3, 2.0)
    c.set_title("C  joint2 delivered speed vs demand", loc="left",
                fontsize=10, color=INK)
    c.set_ylabel("rad/s (log)")
    c.legend(fontsize=7.5, loc="lower left", frameon=False)

    # D: hold decay
    hd = result["hold_decay_joint2"]
    td, yd = np.array(hd["t"]), np.array(hd["abs_error"])
    d.plot(td, yd, "o", color=INK, ms=3.5, label="|reference - actual|")
    tt = np.linspace(td[0], td[-1], 100)
    lin_rate = hd["linear_rate_rad_s"]
    d.plot(tt, yd[0] - lin_rate * (tt - td[0]), color=BLUE, lw=1.6,
           label="linear, %.3f rad/s (rms %.4f)" % (
               lin_rate, hd["linear_fit_rms_rad"]))
    if hd["exponential_tau_sec"]:
        d.plot(tt, yd[0] * np.exp(-(tt - td[0]) / hd["exponential_tau_sec"]),
               color=CRIT, lw=1.2, ls="--",
               label="exponential tau %.2f s (rms %.4f)" % (
                   hd["exponential_tau_sec"], hd["exponential_fit_rms_rad"]))
    d.plot(tt, yd[0] * np.exp(-(tt - td[0]) * gain), color=GREY, lw=1.0,
           ls=":", label="what K=%.1f/s alone would give" % gain)
    d.set_ylim(-0.003, yd[0] * 1.15)
    d.set_title("D  end hold: how the error decays", loc="left", fontsize=10,
                color=INK)
    d.set_xlabel("sim s")
    d.set_ylabel("rad")
    d.legend(fontsize=7.5, loc="upper right", frameon=False)

    # E: per-leg delivered/demand for joint2, payload flag
    legs = result["per_leg_dependence"]
    first_retreat = next(l["index"] for l in legs if l["name"] == "retreat")
    xs, ys, cols, labels = [], [], [], []
    for leg in legs:
        item = leg["delivered_vs_model_demand"].get("elfin_joint2")
        if not item:
            continue
        direction = "+" if leg["ref_excursion_rad"][1] > 0 else "-"
        xs.append(len(xs))
        ys.append(item["median"])
        cols.append(CRIT if item["median"] < 0.5 else BLUE)
        labels.append("%s leg%d\n%s%.2f rad\n%s" % (
            leg["name"], leg["index"], direction,
            abs(leg["ref_excursion_rad"][1]),
            "payload" if leg["index"] < first_retreat else "no payload"))
    e.bar(xs, ys, color=cols, width=0.6)
    e.axhline(1.0, color=INK2, lw=0.8, ls="--")
    for x, y in zip(xs, ys):
        e.text(x, y + 0.03, "%.2f" % y, ha="center", fontsize=8, color=INK)
    e.set_xticks(xs)
    e.set_xticklabels(labels, fontsize=7)
    e.set_ylim(0, 1.25)
    e.set_title("E  joint2 delivered/demand, median per leg", loc="left",
                fontsize=10, color=INK)
    e.set_ylabel("ratio (1 = follows demand)")

    # F: delivered speed vs q2 for samples following demand, by leg group
    err2 = (ref - fb)[:, 1]
    spans = [(l["submit_sim"] if "submit_sim" in l else None) for l in legs]
    f.set_title("F  joint2 delivered speed vs position, |e|>0.01 rad",
                loc="left", fontsize=10, color=INK)
    drawn = set()
    for leg in legs:
        item = leg["delivered_vs_model_demand"].get("elfin_joint2")
        if not item:
            continue
        lo_t, hi_t = leg["_t_lo"], leg["_t_hi"]
        mm = (ct > lo_t) & (ct < hi_t) & np.isfinite(vel[:, 1]) & (
            np.abs(err2) > 0.01)
        bad = item["median"] < 0.5
        label = ("below demand (leg1 transit, leg4 descend)" if bad
                 else "follows demand")
        f.plot(fb[mm, 1], np.abs(vel[mm, 1]), ".", ms=4,
               color=CRIT if bad else BLUE,
               label=None if label in drawn else label)
        drawn.add(label)
    f.set_xlabel("joint2 position, rad")
    f.set_ylabel("delivered |speed|, rad/s")
    f.legend(fontsize=7.5, loc="upper left", frameon=False)

    fig.suptitle(
        "capture02, 2nd transit leg (3.16 s): joint2 lags far beyond its "
        "~0.1 s first-order tracking lag; every healthy joint stays within "
        "0.003 rad after compensation", fontsize=11, color=INK, y=0.965)
    fig.canvas.draw()
    rend = fig.canvas.get_renderer()
    bad_count = 0
    for name, ax in zip("ABCDEF", axes.ravel()):
        boxes = [(t_.get_text().split("\n")[0], t_.get_window_extent(rend))
                 for t_ in ax.texts if t_.get_text().strip()]
        for label, bb in boxes:
            if (bb.x0 < 1 or bb.x1 > fig.bbox.x1 - 1 or bb.y0 < 1
                    or bb.y1 > fig.bbox.y1 - 1):
                print(name, "CLIP", repr(label))
                bad_count += 1
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                (l1, b1), (l2, b2) = boxes[i], boxes[j]
                ox = min(b1.x1, b2.x1) - max(b1.x0, b2.x0)
                oy = min(b1.y1, b2.y1) - max(b1.y0, b2.y0)
                if ox > 2 and oy > 2 and ox * oy > 0.1 * min(
                        b1.width * b1.height, b2.width * b2.height):
                    print(name, "OVERLAP", repr(l1), repr(l2))
                    bad_count += 1
    sup = fig._suptitle.get_window_extent(rend)
    if sup.x0 < 1 or sup.x1 > fig.bbox.x1 - 1:
        print("SUPTITLE CLIP")
        bad_count += 1
    print("AUDIT:", "CLEAN" if bad_count == 0 else bad_count)
    fig.savefig(path, dpi=140)
    plt.close(fig)
