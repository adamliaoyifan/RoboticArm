"""HE-1 generation-2 pipeline: features, seed, board, evidence (no URDF edits)."""

from __future__ import division

import hashlib
import json
import os
from datetime import datetime

import numpy as np

from luggage_description.he1.board import rendered_pitch_check, write_pdf
from luggage_description.he1.geometry import fit_cylinder_from_points
from luggage_description.he1.mount_features import extract_mount_features, load_mount_mesh
from luggage_description.he1.register import (
    DATASHEET_SPACING_MM,
    gui_delta,
    monte_carlo_uncertainty,
    roundtrip_points,
    solve_correspondences,
)
from luggage_description.he1.step_features import load_step_features


def _json(obj):
    if isinstance(obj, (np.bool_, bool)):
        return bool(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, dict):
        return {str(k): _json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json(v) for v in obj]
    return obj


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_source_manifest(out_dir, cad_zip, cad_step, datasheet, retrieved_at):
    rows = []
    for path, url, kind in (
        (cad_zip, "https://dev.realsenseai.com/download/41953", "D555 CAD archive"),
        (cad_step, "extracted from 41953", "D555_SOLID_SOC.STEP"),
        (datasheet, "https://dev.realsenseai.com/download/42013/", "D555 datasheet PDF"),
    ):
        if not path or not os.path.isfile(path):
            continue
        rows.append({
            "kind": kind,
            "url": url,
            "filename": os.path.basename(path),
            "bytes": os.path.getsize(path),
            "sha256": sha256_file(path),
            "retrieved_at": retrieved_at,
        })
    path = os.path.join(out_dir, "source_manifest.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"entries": rows, "licence_note": "upstream CAD not committed"}, handle, indent=2)
        handle.write("\n")
    return path, rows


def _scene_vertices_near(scene, centre_m, radius_m):
    chunks = []
    geoms = scene.geometry.values() if hasattr(scene, "geometry") else [scene]
    for geom in geoms:
        if not hasattr(geom, "vertices"):
            continue
        verts = np.asarray(geom.vertices, dtype=np.float64)
        dist = np.linalg.norm(verts - centre_m, axis=1)
        selected = verts[dist < radius_m]
        if len(selected):
            chunks.append(selected)
    if not chunks:
        return np.zeros((0, 3), dtype=np.float64)
    return np.vstack(chunks)


def tessellation_checks(cad_holes, glb_paths):
    """Envelope and M4 sensitivity at two CAD tessellation tolerances (GLB in metres)."""
    import trimesh
    rows = []
    for path in glb_paths or []:
        if not path or not os.path.isfile(path):
            continue
        scene = trimesh.load(path)
        extents_m = np.asarray(scene.extents, dtype=np.float64)
        bounds_m = np.asarray(scene.bounds, dtype=np.float64)
        hole_rows = []
        for hole in cad_holes:
            centre_mm = np.asarray(hole["centre"], dtype=np.float64)
            nearby_mm = _scene_vertices_near(scene, centre_mm / 1000.0, 0.005) * 1000.0
            if len(nearby_mm) < 12:
                hole_rows.append({
                    "id": hole["id"],
                    "support": int(len(nearby_mm)),
                    "delta_mm": None,
                })
                continue
            fitted = fit_cylinder_from_points(nearby_mm, hole["axis"])
            delta = fitted["centre"] - centre_mm
            hole_rows.append({
                "id": hole["id"],
                "support": int(len(nearby_mm)),
                "tess_centre_mm": fitted["centre"].tolist(),
                "tess_radius_mm": float(fitted["radius"]),
                "delta_mm": delta.tolist(),
                "delta_norm_mm": float(np.linalg.norm(delta)),
                "xy_delta_mm": float(np.linalg.norm(delta[:2])),
            })
        rows.append({
            "path": os.path.basename(path),
            "extents_mm": (extents_m * 1000.0).tolist(),
            "bounds_mm": (bounds_m * 1000.0).tolist(),
            "holes": hole_rows,
        })
    repeat = None
    if len(rows) >= 2:
        c0 = [h.get("tess_centre_mm") for h in rows[0]["holes"]]
        c1 = [h.get("tess_centre_mm") for h in rows[1]["holes"]]
        if all(c0) and all(c1) and len(c0) == len(c1):
            delta = np.asarray(c1, dtype=np.float64) - np.asarray(c0, dtype=np.float64)
            change = float(np.max(np.linalg.norm(delta, axis=1)))
            xy_change = float(np.max(np.linalg.norm(delta[:, :2], axis=1)))
            repeat = {
                "centre_change_mm": change,
                "xy_change_mm": xy_change,
                "pass_0_2mm_0_1deg": change <= 0.2,
                "note": "GLB samples the rear boss, not the M4 thread; XY is the registration-sensitive part",
            }
    envelope = None
    if rows:
        extents = np.asarray(rows[0]["extents_mm"], dtype=np.float64)
        datasheet = np.array([167.0, 42.0, 48.0])
        envelope = {
            "cad_extents_mm": extents.tolist(),
            "cad_aabb_mm": rows[0]["bounds_mm"],
            "datasheet_mm": datasheet.tolist(),
            "extent_error_mm": (extents - datasheet).tolist(),
            "source": rows[0]["path"],
        }
    return {"envelope": envelope, "tolerances": rows, "repeat": repeat}


def render_views(mesh, cad_aabb, T_mm, out_dir, bar_holes, cad_holes, extra=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mapped = np.asarray([h["centre"] for h in cad_holes], dtype=np.float64)
    mapped = mapped.dot(T_mm[:3, :3].T) + T_mm[:3, 3]
    stl = np.asarray([h["centre"] for h in bar_holes], dtype=np.float64)
    extra = extra or {}
    eef = np.asarray(extra.get("eef", []), dtype=np.float64).reshape(-1, 3)
    mid = np.asarray(extra.get("mid360", []), dtype=np.float64).reshape(-1, 3)
    pocket = extra.get("pocket")
    verts = mesh.vertices[:: max(1, len(mesh.vertices) // 4000)]
    origin = T_mm[:3, 3]
    axes = T_mm[:3, :3] * 25.0

    views = {
        "top": (0, 2, "x (mm)", "z (mm)", "top (looking +Y)"),
        "side": (1, 2, "y (mm)", "z (mm)", "side (looking +X)"),
        "camera_facing": (0, 1, "x (mm)", "y (mm)", "camera-facing (looking +Z)"),
        "eef_facing": (0, 2, "x (mm)", "z (mm)", "EEF-facing (x-z, EEF at -Y)"),
    }
    paths = []
    for name, (i, j, xlabel, ylabel, title) in views.items():
        fig = plt.figure(figsize=(6, 5))
        ax = fig.add_subplot(111)
        ax.scatter(verts[:, i], verts[:, j], s=1, c="#ccccdd", linewidths=0, label="mount")
        ax.scatter(stl[:, i], stl[:, j], c="tab:blue", s=40, label="bar holes", zorder=3)
        ax.scatter(mapped[:, i], mapped[:, j], c="tab:red", s=40, label="D555 M4", zorder=3)
        if len(eef):
            ax.scatter(eef[:, i], eef[:, j], c="tab:green", s=30, label="EEF holes", zorder=3)
        if len(mid):
            ax.scatter(mid[:, i], mid[:, j], c="tab:orange", s=30, label="Mid360", zorder=3)
        if pocket is not None:
            p = np.asarray(pocket, dtype=np.float64)
            ax.scatter([p[i]], [p[j]], c="k", s=50, marker="x", label="Mid360 pocket", zorder=4)
        for k, color in enumerate(("r", "g", "b")):
            ax.plot(
                [origin[i], origin[i] + axes[i, k]],
                [origin[j], origin[j] + axes[j, k]],
                color=color, linewidth=1.5, zorder=5,
            )
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(loc="upper right", fontsize=7)
        fig.tight_layout()
        path = os.path.join(out_dir, "view_%s.png" % name)
        fig.savefig(path, dpi=120)
        plt.close(fig)
        paths.append(path)
    return paths


def housing_to_d555_link_verdict(cad_features):
    """Fail closed: CAD origin is not d555_link without an official datum."""
    reasons = [
        "D555_SOLID_SOC.STEP is a SolidWorks AP214 solid with no named IR, DEPTH, OPTICAL, or camera_link coordinate system",
        "Datasheet v1.1 publishes 2x M4 rear-face fasteners at 125.40 mm ± 0.20 mm and envelope 167 x 42 x 48 mm; it does not give a left-IR offset from the CAD origin or the M4 pair",
        "realsense2_description _d455.urdf.xacro maps a different aluminium housing; official_params.md records there is no official D555 URDF",
        "HB-1 driver TF verifies d555_link coincides with left IR / depth, which is the ROS convention, not a housing-CAD measurement",
    ]
    return {
        "authoritative": False,
        "d555_link_meaning": "ROS camera root = left IR / depth (driver + D400 wrapper), verified HB-1 identity d555_link <- depth_frame",
        "cad_frame_meaning": "SolidWorks export of D555_SOLID_SOC, millimetres, origin at the front-face centre of the 167x42x48 mm envelope",
        "blocked_reason": reasons,
        "not_used": [
            "enclosure half-width as a left-IR guess",
            "D455 mesh_x_offset / zero_depth_to_glass applied to D555 IP65 glass",
            "GUI-tuned camera_link as a substitute optical datum",
        ],
    }


def run_he1(
    cad_step, mount_stl, out_dir, cad_zip=None, datasheet=None,
    retrieved_at=None, tessellation_glbs=None,
):
    os.makedirs(out_dir, exist_ok=True)
    retrieved_at = retrieved_at or datetime.now().astimezone().isoformat(timespec="seconds")
    manifest_path, manifest_rows = write_source_manifest(
        out_dir, cad_zip, cad_step, datasheet, retrieved_at)

    cad = load_step_features(cad_step)
    mesh = load_mount_mesh(mount_stl)
    mount = extract_mount_features(mesh)
    solved = solve_correspondences(cad, mount, mount_mesh=mesh)
    tess = tessellation_checks(cad["holes"], tessellation_glbs or [])
    if tess.get("envelope"):
        cad["envelope"]["cad_aabb_mm"] = tess["envelope"]["cad_aabb_mm"]
        cad["envelope"]["cad_extents_mm"] = tess["envelope"]["cad_extents_mm"]

    datum = housing_to_d555_link_verdict(cad)
    board_pdf = os.path.join(out_dir, "charuco_10x8_50mm_DICT_5X5_100.pdf")
    board_meta = write_pdf(board_pdf)
    pitch = rendered_pitch_check()

    T_m = solved.get("transform_m")
    checks = {
        "roundtrip_m": None,
        "feature_recompute_m": None,
        "geometry_14": None,
        "tessellation": tess,
        "envelope": tess.get("envelope"),
    }
    uncertainty = None
    gui = None
    views = []
    if T_m is not None:
        pts = np.array([
            [0.0, 0.0, 0.0],
            [0.08, 0.0, 0.0],
            [0.0, 0.04, 0.0],
            [0.0, 0.0, 0.03],
        ])
        checks["roundtrip_m"] = roundtrip_points(T_m, pts)
        # Recompute from saved centres.
        from luggage_description.he1.register import _candidate_transform
        cad_a = np.asarray(cad["holes"][0]["centre"])
        cad_b = np.asarray(cad["holes"][1]["centre"])
        ids = solved["stl_ids"]
        by_id = {h["id"]: h for h in mount["bar_holes"]}
        stl_a = np.asarray(by_id[ids[0]]["centre"])
        stl_b = np.asarray(by_id[ids[1]]["centre"])
        nsign = solved["normal_sign"]
        T_again = _candidate_transform(
            cad_a, cad_b, stl_a, stl_b,
            np.asarray(cad["seating"]["normal"]),
            nsign * np.asarray(mount["bar_seating"]["normal"]),
        )
        T_again_m = np.array(T_again, dtype=np.float64)
        T_again_m[:3, 3] /= 1000.0
        checks["feature_recompute_m"] = float(np.max(np.abs(T_again_m - T_m)))
        checks["geometry_14"] = {
            "centre_rms_mm": solved["metrics"]["centre_rms_mm"],
            "centre_max_mm": solved["metrics"]["centre_max_mm"],
            "axis_err_deg": solved["metrics"]["axis_err_deg"],
            "penetration_mm": solved["metrics"]["penetration_mm"],
        }
        uncertainty = monte_carlo_uncertainty(
            cad_a, cad_b, stl_a, stl_b,
            np.asarray(cad["seating"]["normal"]),
            nsign * np.asarray(mount["bar_seating"]["normal"]),
        )
        uncertainty["datasheet_spacing_tol_mm"] = 0.20
        uncertainty["stl_vs_datasheet_spacing_mm"] = abs(
            float(np.linalg.norm(stl_b - stl_a)) - DATASHEET_SPACING_MM)
        uncertainty["seating_plane_rms_mm"] = float(mount["bar_seating"]["rms"])
        uncertainty["cad_cylinder_radius_mm"] = float(cad["holes"][0]["radius"])
        uncertainty["stl_used_hole_radius_mm"] = [
            float(by_id[ids[0]]["radius"]), float(by_id[ids[1]]["radius"])]
        if tess.get("repeat"):
            uncertainty["tessellation_xy_change_mm"] = tess["repeat"]["xy_change_mm"]
            uncertainty["tessellation_centre_change_mm"] = tess["repeat"]["centre_change_mm"]
        uncertainty["optical_datum"] = "unavailable; not propagated into d555_link"
        gui = gui_delta(T_m)
        views = render_views(
            mesh, None, solved["transform_mm"], out_dir,
            mount["bar_holes"], cad["holes"],
            extra={
                "eef": [h["centre"] for h in mount["eef_holes"]],
                "mid360": [h["centre"] for h in mount["mid360_holes"]],
                "pocket": mount["mid360_pocket"]["centroid"],
            },
        )

    outcome = "blocked"
    if (
        solved.get("unique")
        and datum["authoritative"]
        and pitch["pass"]
        and checks["roundtrip_m"] is not None
        and checks["roundtrip_m"] < 1e-9
        and checks["feature_recompute_m"] < 1e-12
    ):
        outcome = "pass"

    report = {
        "outcome": outcome,
        "plan": "docs/plans/d555_handeye_calibration.md",
        "notation": "^eef_mount_adapter T_d555_link was required; mechanical seed is ^eef_mount_adapter T_D555-mechanical",
        "cad": {
            "units": cad["units"],
            "solids": cad["solids"],
            "spacing_mm": cad["spacing_mm"],
            "holes": cad["holes"],
            "seating": cad["seating"],
            "envelope": cad["envelope"],
            "header": cad["header"],
        },
        "mount": mount,
        "registration": {k: v for k, v in solved.items() if k != "transform_mm"},
        "tessellation": tess,
        "housing_to_d555_link": datum,
        "gui_comparison": gui,
        "uncertainty": uncertainty,
        "checks": checks,
        "board": {"meta": board_meta, "pitch": pitch},
        "datasheet_spacing_mm": DATASHEET_SPACING_MM,
        "source_manifest": manifest_rows,
        "renders": views,
    }
    # numpy-safe dump
    dumped = _json(report)
    report_path = os.path.join(out_dir, "derivation_report.json")
    with open(report_path, "w", encoding="utf-8") as handle:
        json.dump(dumped, handle, indent=2)
        handle.write("\n")
    features_path = os.path.join(out_dir, "feature_table.json")
    with open(features_path, "w", encoding="utf-8") as handle:
        json.dump(_json({
            "cad_holes": cad["holes"],
            "bar_holes": mount["bar_holes"],
            "eef_holes": mount["eef_holes"],
            "mid360_holes": mount["mid360_holes"],
            "mid360_pocket": mount["mid360_pocket"],
            "bar_seating": mount["bar_seating"],
            "cad_seating": cad["seating"],
        }), handle, indent=2)
        handle.write("\n")
    if T_m is not None:
        with open(os.path.join(out_dir, "T_eef_mount_adapter_D555_mechanical.json"), "w", encoding="utf-8") as handle:
            json.dump(_json(solved["report_m"]), handle, indent=2)
            handle.write("\n")
    summary_path = os.path.join(out_dir, "SUMMARY.md")
    with open(summary_path, "w", encoding="utf-8") as handle:
        handle.write("# HE-1 generation 2 — D555 mechanical registration\n\n")
        handle.write("- outcome: %s\n" % outcome)
        handle.write("- CAD M4 spacing: %.3f mm (datasheet 125.40 ± 0.20)\n" % cad["spacing_mm"])
        handle.write("- unique correspondence: %s\n" % solved.get("unique"))
        if solved.get("stl_ids"):
            handle.write("- used bar holes: %s\n" % ", ".join(solved["stl_ids"]))
        handle.write("- housing-to-d555_link: not authoritative; seed stops at D555-mechanical\n")
        handle.write("- reported transform is NOT ^eef_mount_adapter T_d555_link\n")
        handle.write("- board: `%s`\n" % os.path.relpath(board_pdf, out_dir))
        handle.write("- pitch check relative error: %.4g pass=%s\n" % (pitch["relative_error"], pitch["pass"]))
        if checks.get("roundtrip_m") is not None:
            handle.write("- round-trip max error: %.3g m\n" % checks["roundtrip_m"])
        if tess.get("envelope"):
            handle.write("- CAD envelope vs datasheet (mm): %s\n" % tess["envelope"]["extent_error_mm"])
        if tess.get("repeat"):
            handle.write("- tessellation centre change: %.3f mm (XY %.3f mm)\n" % (
                tess["repeat"]["centre_change_mm"], tess["repeat"]["xy_change_mm"]))
        if solved.get("yaw_symmetry_180_deg"):
            handle.write("- 180 deg yaw of the 2x M4 pattern remains; canonical assignment is CAD +X along STL_BAR_H0 -> H3\n")
        if gui:
            handle.write(
                "- GUI comparison (mechanical vs camera_link, not d555_link): "
                "%.1f mm, %.2f deg\n" % (
                    gui["translation_norm_mm"], gui["rotation_angle_deg"]))
        handle.write("\nDo not apply this transform to URDF/xacro/yaml.\n")
    return dumped, out_dir
