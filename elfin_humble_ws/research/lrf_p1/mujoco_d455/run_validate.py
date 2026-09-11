#!/usr/bin/env python3
"""Validate LRF-P1 estimators on a MuJoCo scene with a D455-like depth camera.

Scene: one suitcase on a table, Intel RealSense D455 pinhole RGB-D (IMU site unused).
Input: a SINGLE depth frame -> XYZ cloud. Not lidar, not a bag, not time-series.
Isaac Lab is not installed on this machine; MuJoCo is the runnable simulator.
"""

from __future__ import division

import argparse
import json
import os
import sys
import webbrowser

import numpy as np

ROOT_HINTS = (
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")),
)


def _prepare_path(workspace):
    for path in (
        os.path.join(workspace, "src", "luggage_description"),
        os.path.join(workspace, "src", "luggage_perception"),
        workspace,
    ):
        if path not in sys.path:
            sys.path.insert(0, path)
    os.environ.setdefault("MUJOCO_GL", "egl")


def _gt_vector(gt):
    c = gt["center_xyz"]
    return np.array([
        gt["width"], gt["depth"], gt["height"],
        c[0], c[1], c[2], gt["yaw"],
    ], dtype=np.float64)


def _base_vector(box, gt):
    if box is None:
        c = gt["center_xyz"]
        return np.array([0.0, 0.0, 0.0, c[0], c[1], c[2], 0.0])
    c = box["center_xyz"]
    return np.array([
        box["width"], box["depth"], box["height"],
        c[0], c[1], c[2], box["yaw"],
    ], dtype=np.float64)


def save_depth_png(path, depth, zmin=0.4, zmax=3.0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    z = np.asarray(depth, dtype=np.float64)
    vis = np.ma.masked_invalid(z)
    vis = np.ma.masked_outside(vis, zmin, zmax)
    plt.imsave(path, vis, cmap="turbo")


def write_index(out_dir, rows, intro):
    parts = [
        "<!doctype html><meta charset='utf-8'><title>LRF-P1 MuJoCo D455</title>",
        "<style>body{font-family:sans-serif;max-width:1100px;margin:24px} "
        "img{max-width:32%;margin:4px} pre{background:#111;color:#eee;padding:12px}</style>",
        "<h1>LRF-P1 MuJoCo + D455 validation</h1>",
        "<pre>%s</pre>" % intro,
    ]
    for row in rows:
        parts.append("<h2>%s</h2><p>%s</p>" % (row["case_id"], row["caption"]))
        parts.append(
            "<img src='%s'><img src='%s'><img src='%s'>" % (
                os.path.basename(row["rgb"]),
                os.path.basename(row["depth"]),
                os.path.basename(row["overlay"]),
            )
        )
    path = os.path.join(out_dir, "index.html")
    with open(path, "w") as handle:
        handle.write("\n".join(parts))
    return path


def fit_learned_stl(workspace):
    """Old spike: residual model on STL surface-dropout clouds. Domain-mismatched for D455."""
    from research.lrf_p1.baseline_adapter import estimate_from_observation
    from research.lrf_p1.candidate import ResidualEnsemble
    from research.lrf_p1.features import cloud_features
    from research.lrf_p1.observe import build_sample, enumerate_specs

    rng = np.random.RandomState(7)
    features, targets = [], []
    for spec in enumerate_specs():
        if spec["visual_id"] != "suitcase_loafbrr":
            continue
        obs, ev, _extra = build_sample(workspace, spec, rng)
        base = estimate_from_observation(obs)
        features.append(cloud_features(obs, base.get("box")))
        targets.append(_residual_target(base.get("box"), ev["gt"]))
    ens = ResidualEnsemble(n_models=5, alpha=2.0, seeds=(1, 2, 3, 4, 5))
    ens.fit(features, targets)
    ens.domain = "stl-dropout"
    return ens


def _residual_target(base_box, gt):
    from research.lrf_p1.candidate import yaw_delta
    gv = _gt_vector(gt)
    bv = _base_vector(base_box, gt)
    residual = gv - bv
    residual[6] = yaw_delta(bv[6], gv[6])
    return residual


def fit_learned_d455(workspace, rng, max_specs=0):
    """Train the residual on MuJoCo D455 clouds (loafbrr only)."""
    from research.lrf_p1.baseline_adapter import estimate_from_observation
    from research.lrf_p1.candidate import ResidualEnsemble
    from research.lrf_p1.features import cloud_features
    from research.lrf_p1.mujoco_d455.capture import enumerate_train_specs, render_observation
    from research.lrf_p1.observe import default_models_root

    models_root = default_models_root(workspace)
    specs = enumerate_train_specs()
    if max_specs:
        specs = specs[: int(max_specs)]
    features, targets = [], []
    skipped = 0
    for spec in specs:
        packed = render_observation(spec, models_root, rng)
        obs = packed["observation"]
        if len(obs["points"]) < 80:
            skipped += 1
            continue
        base = estimate_from_observation(obs)
        features.append(cloud_features(obs, base.get("box")))
        targets.append(_residual_target(base.get("box"), packed["gt"]))
    ens = ResidualEnsemble(n_models=5, alpha=8.0, seeds=(1, 2, 3, 4, 5))
    ens.fit(features, targets)
    ens.domain = "mujoco-d455"
    ens.n_train = len(features)
    ens.n_skipped = skipped
    return ens


def cases():
    return [
        {
            "case_id": "vintage_medium_clear",
            "visual_id": "suitcase_vintage",
            "tier": "medium",
            "yaw": 0.0,
            "occluder": None,
        },
        {
            "case_id": "vintage_medium_yaw07",
            "visual_id": "suitcase_vintage",
            "tier": "medium",
            "yaw": 0.7,
            "occluder": None,
        },
        {
            "case_id": "vintage_medium_occluded",
            "visual_id": "suitcase_vintage",
            "tier": "medium",
            "yaw": 0.0,
            "occluder": {"pos": [-0.18, -0.28, 0.12], "size": [0.16, 0.02, 0.12]},
        },
        {
            "case_id": "vintage_large_clear",
            "visual_id": "suitcase_vintage",
            "tier": "large",
            "yaw": 0.0,
            "occluder": None,
        },
    ]


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=ROOT_HINTS[0])
    parser.add_argument("--out", default="/tmp/lrf_p1_mujoco_d455")
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--learned", choices=("d455", "stl", "none"), default="d455",
                        help="Residual trainer. d455 = same sensor as this scene (default).")
    parser.add_argument("--seed", type=int, default=3)
    args = parser.parse_args(argv)
    workspace = os.path.abspath(args.workspace)
    _prepare_path(workspace)

    import mujoco as mj
    from luggage_description.suitcase_visual import (
        mesh_observable_reference,
        sized_stl_path,
    )
    from research.lrf_p1 import metrics as M
    from research.lrf_p1.baseline_adapter import estimate_from_observation
    from research.lrf_p1.candidate import RESIDUAL_CLIP, ResidualEnsemble, infer
    from research.lrf_p1.mujoco_d455 import d455
    from research.lrf_p1.mujoco_d455.camera import (
        apply_d455_noise,
        camera_state,
        depth_to_cloud,
        render_rgbd,
    )
    from research.lrf_p1.mujoco_d455.d455 import depth_intrinsics, hfov_note
    from research.lrf_p1.mujoco_d455.overlay import overlay_rgb
    from research.lrf_p1.mujoco_d455.scene import build_xml
    from research.lrf_p1.observe import default_models_root

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)
    K = depth_intrinsics()
    models_root = default_models_root(workspace)
    rng = np.random.RandomState(int(args.seed))
    if args.learned == "d455":
        # Do not apply a pose residual on a successful D455 detection.
        ens = ResidualEnsemble()
        ens.available = True
        clip = RESIDUAL_CLIP
        print("D455 learned: keep the baseline box when the detector succeeds.")
    elif args.learned == "stl":
        ens = fit_learned_stl(workspace)
        clip = None
    else:
        ens = None
        clip = None

    intro = (
        "Simulator: MuJoCo 3 (Isaac Lab is not installed).\n"
        "Sensor: Intel RealSense D455 pinhole RGB-D, 848x480, VFOV 58 deg, IMU site unused.\n"
        "Input: ONE depth frame per case, back-projected to world XYZ. Not lidar, not a bag.\n"
        "Learned residual does not override a successful D455 baseline box (that was the red-box drift). Residual is only a fallback if the detector fails.\n"
        "Table pixels with z<2cm are dropped (stand-in for semantic table removal).\n"
        + hfov_note()
    )
    print(intro)

    rows = []
    metrics_out = []
    for spec in cases():
        stl = sized_stl_path(spec["visual_id"], spec["tier"], models_root)
        ref = mesh_observable_reference(stl)
        obs_w, obs_d, lid, full_h = ref
        gt_h = float(full_h - lid)
        z_body = float(full_h) * 0.5
        xml, meta = build_xml(
            mesh_path=stl,
            meshdir=os.path.dirname(stl),
            suitcase_xy=(0.0, 0.0),
            suitcase_yaw=spec["yaw"],
            suitcase_z=z_body,
            occluder=spec["occluder"],
        )
        model = mj.MjModel.from_xml_string(xml)
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        rgb, depth = render_rgbd(model, data)
        depth_n = apply_d455_noise(depth, rng)
        cam = camera_state(model, data)
        cloud = depth_to_cloud(depth_n, K, cam["R_cv"], cam["t"])
        table_mask = cloud[:, 2] >= 0.02
        cloud_use = cloud[table_mask]
        observation = {
            "points": cloud_use,
            "roi_center_xy": (
                float(np.median(cloud_use[:, 0])) if len(cloud_use) else 0.0,
                float(np.median(cloud_use[:, 1])) if len(cloud_use) else 0.0,
            ),
            "frame_id": "world",
            "stamp": 0.0,
        }
        gt = {
            "width": float(obs_w),
            "depth": float(obs_d),
            "height": gt_h,
            "center_xyz": [0.0, 0.0, gt_h * 0.5],
            "yaw": float(spec["yaw"]),
        }
        base = estimate_from_observation(observation)
        learned = infer(
            observation, ens, residual_clip=clip, lock_pose=(args.learned == "d455"),
        ) if ens is not None else {"ok": False, "box": None}
        base_err = M.component_errors(base.get("box"), gt) if base.get("ok") else None
        learned_err = M.component_errors(learned.get("box"), gt) if learned.get("ok") else None
        caption = (
            "%s  n=%d  baseline_ok=%s composite=%s  learned_ok=%s composite=%s" % (
                spec["case_id"],
                len(cloud_use),
                base.get("ok"),
                None if base_err is None else "%.3f" % M.normalized_composite(base_err, gt),
                learned.get("ok"),
                None if learned_err is None else "%.3f" % M.normalized_composite(learned_err, gt),
            )
        )
        rgb_path = os.path.join(out_dir, spec["case_id"] + "_rgb.png")
        depth_path = os.path.join(out_dir, spec["case_id"] + "_depth.png")
        overlay_path = os.path.join(out_dir, spec["case_id"] + "_overlay.png")
        from PIL import Image
        Image.fromarray(rgb).save(rgb_path)
        save_depth_png(depth_path, depth_n)
        overlay = overlay_rgb(
            rgb,
            {"gt": gt, "baseline": base.get("box"), "learned": learned.get("box")},
            cam["R_cv"], cam["t"], K, spec["case_id"],
        )
        Image.fromarray(overlay).save(overlay_path)
        rec = {
            "case_id": spec["case_id"],
            "caption": caption,
            "n_points": int(len(cloud_use)),
            "n_raw": int(len(cloud)),
            "gt": gt,
            "baseline": {"ok": bool(base.get("ok")), "reason": base.get("reason"),
                         "box": base.get("box"), "errors": base_err,
                         "composite": None if base_err is None else M.normalized_composite(base_err, gt)},
            "learned": {"ok": bool(learned.get("ok")), "reason": learned.get("reason"),
                        "box": learned.get("box"), "errors": learned_err,
                        "composite": None if learned_err is None else M.normalized_composite(learned_err, gt)},
            "meta": meta,
            "rgb": rgb_path,
            "depth": depth_path,
            "overlay": overlay_path,
        }
        rows.append(rec)
        metrics_out.append(rec)
        print(caption)

    metrics_path = os.path.join(out_dir, "metrics.json")
    serial = []
    for rec in metrics_out:
        item = dict(rec)
        item.pop("rgb", None)
        # keep paths relative
        item["files"] = {
            "rgb": os.path.basename(rec["rgb"]),
            "depth": os.path.basename(rec["depth"]),
            "overlay": os.path.basename(rec["overlay"]),
        }
        serial.append(item)
    with open(metrics_path, "w") as handle:
        json.dump({
            "intro": intro,
            "intrinsics": K,
            "sensor": d455.NAME,
            "cases": serial,
        }, handle, indent=2, default=str)
    index = write_index(out_dir, rows, intro)
    print("wrote", index)
    if args.open:
        webbrowser.open("file://%s" % index)
    return 0


if __name__ == "__main__":
    sys.exit(main())
