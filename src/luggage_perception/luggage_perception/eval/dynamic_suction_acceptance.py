#!/usr/bin/env python3
"""DYNAMIC-SUCTION ST-1 acceptance runner: gates A0-A4 (A5 deferred).

CLI (D1-fixed entry point)::

    python3 -m luggage_perception.eval.dynamic_suction_acceptance \
        --config <locked-config> --out <evidence-run>
    python3 -m luggage_perception.eval.dynamic_suction_acceptance \
        --config <locked-config> --out <evidence-run> \
        --replay <case-dir> --stage {ingest,segmentation,cargo,top}

Evidence contract (.cursor/rules/debug-evidence.mdc): T0 manifest for every
run, T1 boundary trace for every scored case, T2 failure bundle frozen only
for failing cases (bounded NPZ + JSON), replay re-runs a frozen case and
diffs. Exit code 0 iff every executed gate passes; gate A5 reports
``deferred`` (user decision 2026-09-15: real-cell recording scheduled
separately) and does not affect the exit code.

The runner is ROS-free: it must import and execute without any ROS
environment, which is itself gate A0's isolation check.
"""

from __future__ import division

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np

from luggage_perception.instance_depth_component import (
    ComponentConfig,
    isolate_depth_component,
)
from luggage_perception.dynamic_top_surface import (
    DynamicTopConfig,
    estimate_dynamic_top_surface,
)
from luggage_perception.platform_free_pipeline import PlatformFreeDetector
from luggage_perception.top_support_estimator import TopSupportConfig
from luggage_perception.eval.dynamic_suction_renderer import (
    DEFAULT_CATALOG,
    NadirRenderer,
    make_a2_case,
    make_a3_case,
    make_a4_case,
    make_near_square_case,
    make_platform_only_case,
)

STAGES = ("ingest", "segmentation", "cargo", "top")


def _load_yaml(path):
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - site guarantee
        raise SystemExit("PyYAML required for --config: %s" % exc)
    with open(path) as fh:
        return yaml.safe_load(fh)


def _sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _git_state():
    """(commit, dirty_count) or ('unknown', -1) outside a repository."""
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL,
            text=True).strip()
        dirty = int(subprocess.check_output(
            ["git", "status", "--porcelain"], stderr=subprocess.DEVNULL,
            text=True).count("\n"))
        return commit, dirty
    except (subprocess.CalledProcessError, OSError):
        return "unknown", -1


def _configs_from_yaml(cfg):
    camera = dict(cfg["camera"])
    component = ComponentConfig(**{
        k: v for k, v in cfg["component"].items()})
    dynamic = DynamicTopConfig(**{
        k: v for k, v in cfg["dynamic_top"].items()})
    return camera, component, dynamic


def _intrinsics(camera):
    return SimpleNamespace(fx=camera["fx"], fy=camera["fy"],
                           cx=camera["cx"], cy=camera["cy"])


def _run_case(case, component_cfg, dynamic_cfg):
    """One scored case: component isolation + dynamic top estimate.

    Returns ``(component, result)``; ``result`` may carry a fail-closed
    reason (never None).
    """
    comp = isolate_depth_component(
        case.depth_mm, bbox=case.bbox, config=component_cfg)
    if not comp.ok:
        return comp, None
    x0, y0, x1, y1 = case.bbox
    region = np.zeros(case.depth_mm.shape, dtype=bool)
    region[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = True
    result = estimate_dynamic_top_surface(
        case.depth_mm, comp.mask, _intrinsics(case.camera), case.mat4,
        stamp=case.stamp, frame=case.frame, instance_region=region,
        config=dynamic_cfg)
    return comp, result


def _yaw_error_deg(yaw_rad, gt_yaw_deg):
    err = (math.degrees(yaw_rad) - gt_yaw_deg + 90.0) % 180.0 - 90.0
    return abs(err)


class EvidenceWriter(object):
    """T0/T1/T2 evidence under ``<out>`` per the dump matrix."""

    def __init__(self, out_dir, config_path, config):
        self.out = out_dir
        self.failed_dir = os.path.join(out_dir, "failed")
        os.makedirs(self.failed_dir, exist_ok=True)
        self.traces_path = os.path.join(out_dir, "traces_T1.jsonl")
        self._traces = open(self.traces_path, "w")
        self.manifest = {
            "run_kind": "dynamic_suction_acceptance",
            "plan": "docs/plans/dynamic_top_surface_and_suction_patch.md",
            "gates": "A0-A4 (A5 deferred by user decision 2026-09-15)",
            "config_path": os.path.abspath(config_path),
            "config_sha256": _sha256_file(config_path),
            "config": config,
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "capture_complete": False,
            "replay_possible": True,
        }
        commit, dirty = _git_state()
        self.manifest["git_commit"] = commit
        self.manifest["git_dirty_count"] = dirty

    def t1(self, record):
        self._traces.write(json.dumps(record, sort_keys=True,
                                      default=_json_default) + "\n")
        self._traces.flush()

    def freeze_t2(self, case, comp, result, failure):
        """Freeze the failing case bundle (bounded NPZ + JSON)."""
        case_dir = os.path.join(self.failed_dir, case.case_id)
        os.makedirs(case_dir, exist_ok=True)
        arrays = {
            "depth_mm": case.depth_mm,
            "bbox": np.asarray(case.bbox, dtype=np.int64),
        }
        if comp is not None and comp.ok:
            arrays["component_mask"] = comp.mask
        payload = {
            "case": _case_json(case),
            "failure": failure,
            "component_reason": None if comp is None else comp.reason,
            "component_diagnostics":
                None if comp is None else _plain(comp.diagnostics),
            "result": None if result is None else _result_json(result),
            "frozen_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "capture_complete": True,
            "replay_possible": True,
            "replay_command": (
                "python3 -m luggage_perception.eval."
                "dynamic_suction_acceptance --config %s --out %s "
                "--replay %s --stage top"
                % (self.manifest["config_path"], self.out, case_dir)),
        }
        np.savez_compressed(
            os.path.join(case_dir, "case.npz"), **arrays)
        with open(os.path.join(case_dir, "T2.json"), "w") as fh:
            json.dump(payload, fh, indent=1, sort_keys=True,
                      default=_json_default)
        return case_dir

    def finish(self, gate_results, passed):
        self._traces.close()
        self.manifest["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.manifest["capture_complete"] = True
        self.manifest["gates"] = gate_results
        with open(os.path.join(self.out, "manifest_T0.json"), "w") as fh:
            json.dump(self.manifest, fh, indent=1, sort_keys=True,
                      default=_json_default)
        with open(os.path.join(self.out, "summary.json"), "w") as fh:
            json.dump({"pass": bool(passed), "gates": gate_results}, fh,
                      indent=1, sort_keys=True, default=_json_default)
        with open(os.path.join(self.out, "RESULT.md"), "w") as fh:
            fh.write(_result_md(gate_results, passed, self.manifest))


def _json_default(obj):
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    return str(obj)


def _plain(value):
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, (np.integer, np.floating, np.bool_)):
        return _json_default(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def _case_json(case):
    return {
        "case_id": case.case_id, "gate": case.gate, "bbox": list(case.bbox),
        "stamp": case.stamp, "frame": case.frame, "seed": case.seed,
        "generation": case.generation, "instance_id": case.instance_id,
        "gt_center_xy": case.gt_center_xy, "gt_top_z": case.gt_top_z,
        "gt_yaw_deg": case.gt_yaw_deg, "gt_size_wh": case.gt_size_wh,
        "meta": _plain(case.meta),
    }


def _result_json(result):
    return {
        "reason": result.reason, "stamp": result.stamp,
        "frame": result.frame, "top_z": result.top_z, "yaw": result.yaw,
        "yaw_valid": result.yaw_valid, "aspect_ratio": result.aspect_ratio,
        "width": result.width, "depth": result.depth,
        "confidence": result.confidence,
        "center_xy": list(result.center_xy),
        "pca_yaw": getattr(result, "pca_yaw", 0.0),
        "pixel_count": getattr(result, "pixel_count", 0),
        "plane_candidates": [
            {"plane_id": c.plane_id, "normal": list(c.normal),
             "inliers": c.inliers, "rms_residual": c.rms_residual,
             "height": c.height, "connected_area": c.connected_area,
             "mask_support": c.mask_support,
             "inlier_fraction": c.inlier_fraction, "score": c.score,
             "eligible": c.eligible, "rejected_reason": c.rejected_reason}
            for c in getattr(result, "plane_candidates", ())],
    }


def _metrics_of(case, result):
    xy_err = math.hypot(result.center_xy[0] - case.gt_center_xy[0],
                        result.center_xy[1] - case.gt_center_xy[1])
    return {
        "xy_err_mm": xy_err * 1000.0,
        "top_z_err_mm": abs(result.top_z - case.gt_top_z) * 1000.0,
        "width_err_mm": abs(result.width - case.gt_size_wh[0]) * 1000.0,
        "depth_err_mm": abs(result.depth - case.gt_size_wh[1]) * 1000.0,
        "yaw_err_deg": _yaw_error_deg(result.yaw, case.gt_yaw_deg),
        "aspect_ratio": float(result.aspect_ratio),
        "yaw_valid": bool(result.yaw_valid),
        "confidence": float(result.confidence),
    }


# --- gates -----------------------------------------------------------------

def gate_a0(cfg, camera, component_cfg, dynamic_cfg, writer):
    """Import isolation, stamp/frame identity, buffer contract, zero stale
    fusion, determinism."""
    checks = {}
    checks["rclpy_absent"] = not any(
        m.startswith("rclpy") for m in sys.modules)
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    for name in ("instance_depth_component.py", "dynamic_top_surface.py"):
        with open(os.path.join(pkg, name)) as fh:
            src = fh.read()
        checks["%s_ros_free" % name] = not any(
            tok in src for tok in ("rclpy", "rospy", "tf2_ros",
                                   "sensor_msgs", "luggage_msgs"))
        checks["%s_no_io" % name] = not any(
            tok in src for tok in ("open(", "np.load", "np.save",
                                   "yaml."))
    # Buffer contract: the shared ROS-free ring honours 15 entries and the
    # 1.0 s camera-clock horizon with exact integer-stamp identity.
    from luggage_perception.stamp_ring_buffer import StampRingBuffer
    ring = StampRingBuffer(maxlen=15, horizon_sec=1.0)
    base = 1_000_000_000
    for i in range(30):
        ring.insert_ns(base + i * 100_000_000, i)
    checks["ring_occupancy_bounded"] = len(ring) <= 15
    # Stamp/frame identity + determinism on a representative case.
    renderer = NadirRenderer(camera)
    case = make_a2_case(renderer, "carryon", DEFAULT_CATALOG[0][1],
                        (-1.0, 0.0), 30.0, 11, 100.0)
    comp, r1 = _run_case(case, component_cfg, dynamic_cfg)
    comp2, r2 = _run_case(case, component_cfg, dynamic_cfg)
    determinism = (
        r1 is not None and r2 is not None
        and r1.reason == r2.reason
        and r1.top_z == r2.top_z and r1.yaw == r2.yaw
        and r1.width == r2.width and r1.depth == r2.depth
        and bool((r1.center_xy == r2.center_xy).all())
        and comp.reason == comp2.reason)
    checks["determinism"] = determinism
    checks["stamp_frame_carried"] = bool(
        r1 is not None and r1.reason == "ok"
        and r1.stamp == case.stamp and r1.frame == case.frame)
    writer.t1({"gate": "A0", "case_id": case.case_id, "checks": checks})
    return {"gate": "A0", "pass": all(checks.values()), "checks": checks}


def gate_a1(cfg, camera, component_cfg, dynamic_cfg, writer):
    """Scene independence: identical observation replayed with pickup XY
    and workspace-extent variations (crop/predicate stay false)."""
    spec = cfg["matrices"]["a1"]
    tol = float(spec["numeric_tolerance"])
    renderer = NadirRenderer(camera)
    case = make_a2_case(renderer, "standard", DEFAULT_CATALOG[1][1],
                        (-1.0, 0.1), 60.0, 29, 200.0)
    comp, top = _run_case(case, component_cfg, dynamic_cfg)
    if top is None or top.reason != "ok":
        writer.freeze_t2(case, comp, top, "a1_reference_failed")
        return {"gate": "A1", "pass": False,
                "reason": "reference estimate failed",
                "component_reason": None if comp is None else comp.reason}
    records = []
    identical = True
    for xy in spec["pickup_xy_variants"]:
        for half in spec["half_extent_variants"]:
            ws_cfg = TopSupportConfig(
                workspace_center_xy=[float(xy[0]), float(xy[1])],
                workspace_half_extents=[float(half), float(half)],
                crop_to_workspace=False)
            pipeline = PlatformFreeDetector(config=ws_cfg,
                                            support_mode="auto")
            pts = np.asarray([[0.0, 0.0, top.top_z]])
            pts = np.repeat(pts, 64, axis=0)
            result = pipeline.update(
                pts, None, source="measure", geometry_ok=True,
                raw_same_stamp=False, stamp_sec=case.stamp,
                cargo_segmented=True, top_surface=top)
            rec = {
                "pickup_xy": [float(xy[0]), float(xy[1])],
                "half_extent": float(half),
                "top_valid": bool(result.top_valid),
                "top_source": result.top_source,
                "box_width": result.box.width,
                "box_depth": result.box.depth,
                "box_top_z": float(result.box.top.top_z),
                "center_xy": [float(result.box.top.center_xy[0]),
                              float(result.box.top.center_xy[1])],
            }
            same = (
                result.top_valid and result.top_source == "dynamic"
                and abs(rec["box_width"] - top.width) <= tol
                and abs(rec["box_depth"] - top.depth) <= tol
                and abs(rec["box_top_z"] - top.top_z) <= tol
                and abs(rec["center_xy"][0] - float(top.center_xy[0]))
                <= tol
                and abs(rec["center_xy"][1] - float(top.center_xy[1]))
                <= tol)
            rec["identical"] = bool(same)
            identical = identical and same
            records.append(rec)
    writer.t1({"gate": "A1", "case_id": case.case_id,
               "variants": records})
    return {"gate": "A1", "pass": bool(identical),
            "variant_count": len(records),
            "tolerance": tol}


def gate_a2(cfg, camera, component_cfg, dynamic_cfg, writer):
    """324-case synthetic dynamic-position matrix + negative + aspect
    probes."""
    spec = cfg["matrices"]["a2"]
    th = spec["thresholds"]
    renderer = NadirRenderer(camera)
    sizes = spec["sizes"]
    stamps = iter(_stamp_seq(1000.0))
    rows = []
    cells = {}
    for size_id, whd in DEFAULT_CATALOG:
        if size_id not in sizes:
            continue
        for dx in spec["offsets_m"]:
            for dy in spec["offsets_m"]:
                for yaw in spec["yaws_deg"]:
                    for seed in spec["seeds"]:
                        case = make_a2_case(
                            renderer, size_id, tuple(whd),
                            (-1.0 + float(dx), float(dy)), float(yaw),
                            int(seed), next(stamps))
                        t0 = time.monotonic()
                        comp, result = _run_case(
                            case, component_cfg, dynamic_cfg)
                        latency_ms = (time.monotonic() - t0) * 1000.0
                        valid = result is not None and result.reason == "ok"
                        row = {"case_id": case.case_id, "valid": valid,
                               "latency_ms": latency_ms,
                               "component_reason":
                                   None if comp is None else comp.reason}
                        if valid:
                            row["metrics"] = _metrics_of(case, result)
                            row["result"] = _result_json(result)
                            for key, value in row["metrics"].items():
                                if isinstance(value, float):
                                    row[key] = value
                        else:
                            row["result_reason"] = (
                                "COMPONENT_" + comp.reason
                                if comp is not None and not comp.ok
                                else (result.reason if result else "?"))
                            row["replay_dir"] = writer.freeze_t2(
                                case, comp, result, "a2_invalid_top")
                        rows.append(row)
                        cell = (size_id, dx, dy)
                        cells.setdefault(cell, []).append(valid)
                        writer.t1({"gate": "A2", **{
                            k: v for k, v in row.items()
                            if k != "result"}})
    valid_flags = [r["valid"] for r in rows]
    valid_rate = (sum(valid_flags) / len(rows)) if rows else 0.0
    cell_rates = {
        "%s|%+d|%+d" % cell: (sum(v) / len(v))
        for cell, v in cells.items()}
    worst_cell = min(cell_rates.values()) if cell_rates else 0.0
    m = [r["metrics"] for r in rows if r["valid"]]

    def p95(key):
        return float(np.percentile([x[key] for x in m], 95)) if m else 0.0

    def mx(key):
        return float(max([x[key] for x in m])) if m else 0.0

    # Negative probes: platform/background-only ROIs must never be valid.
    neg = []
    for center in spec["negative_probes"]["centers"]:
        for seed in spec["negative_probes"]["seeds"]:
            case = make_platform_only_case(
                renderer, tuple(center), int(seed), next(stamps))
            comp, result = _run_case(case, component_cfg, dynamic_cfg)
            bad = result is not None and result.reason == "ok"
            neg.append(bad)
            if bad:
                row = writer.freeze_t2(
                    case, comp, result, "a2_false_top")
            writer.t1({"gate": "A2-negative", "case_id": case.case_id,
                       "false_top": bool(bad),
                       "component_reason":
                           None if comp is None else comp.reason})
    # Aspect probe: near-square top must report yaw_valid=False.
    aspect_ok = []
    for seed in (11, 29):
        case = make_near_square_case(renderer, seed, next(stamps))
        comp, result = _run_case(case, component_cfg, dynamic_cfg)
        ok = (result is not None and result.reason == "ok"
              and not result.yaw_valid)
        aspect_ok.append(bool(ok))
        writer.t1({"gate": "A2-aspect", "case_id": case.case_id,
                   "yaw_valid": None if result is None
                   else bool(result.yaw_valid),
                   "pass": bool(ok)})

    pass_all = (
        len(rows) == 324
        and valid_rate >= float(th["valid_rate_overall"])
        and worst_cell >= float(th["valid_rate_per_cell"])
        and p95("xy_err_mm") <= th["xy_p95_mm"]
        and mx("xy_err_mm") <= th["xy_max_mm"]
        and p95("top_z_err_mm") <= th["top_z_p95_mm"]
        and mx("top_z_err_mm") <= th["top_z_max_mm"]
        and p95("width_err_mm") <= th["size_p95_mm"]
        and mx("width_err_mm") <= th["size_max_mm"]
        and p95("depth_err_mm") <= th["size_p95_mm"]
        and mx("depth_err_mm") <= th["size_max_mm"]
        and p95("yaw_err_deg") <= th["yaw_p95_deg"]
        and mx("yaw_err_deg") <= th["yaw_max_deg"]
        and sum(neg) == th["false_top_count"]
        and all(aspect_ok))
    summary = {
        "gate": "A2", "pass": bool(pass_all), "cases": len(rows),
        "valid_rate": valid_rate, "worst_cell_rate": worst_cell,
        "cell_rates": cell_rates,
        "xy_p95_mm": p95("xy_err_mm"), "xy_max_mm": mx("xy_err_mm"),
        "top_z_p95_mm": p95("top_z_err_mm"),
        "top_z_max_mm": mx("top_z_err_mm"),
        "width_p95_mm": p95("width_err_mm"),
        "width_max_mm": mx("width_err_mm"),
        "depth_p95_mm": p95("depth_err_mm"),
        "depth_max_mm": mx("depth_err_mm"),
        "yaw_p95_deg": p95("yaw_err_deg"), "yaw_max_deg": mx("yaw_err_deg"),
        "false_top_count": int(sum(neg)),
        "aspect_probes_pass": all(aspect_ok),
        "latency_p50_ms": float(np.percentile(
            [r["latency_ms"] for r in rows], 50)) if rows else 0.0,
        "latency_p95_ms": float(np.percentile(
            [r["latency_ms"] for r in rows], 95)) if rows else 0.0,
        "failures": [r["case_id"] for r in rows if not r["valid"]],
    }
    return summary


def gate_a3(cfg, camera, component_cfg, dynamic_cfg, writer):
    """36-case contamination + multi-plane selection matrix."""
    spec = cfg["matrices"]["a3"]
    renderer = NadirRenderer(camera)
    stamps = iter(_stamp_seq(2000.0))
    rows = []
    for size_id, whd in DEFAULT_CATALOG:
        for frac in spec["fractions"]:
            for seed in spec["seeds"]:
                case = make_a3_case(renderer, size_id, tuple(whd),
                                    float(frac), int(seed), next(stamps))
                t0 = time.monotonic()
                comp, result = _run_case(case, component_cfg, dynamic_cfg)
                latency_ms = (time.monotonic() - t0) * 1000.0
                valid = result is not None and result.reason == "ok"
                correct = False
                wrong_plane = False
                if valid:
                    metrics = _metrics_of(case, result)
                    correct = (
                        metrics["top_z_err_mm"] <= 15.0
                        and metrics["xy_err_mm"] <= 35.0
                        and metrics["width_err_mm"] <= 50.0
                        and metrics["depth_err_mm"] <= 50.0)
                    wrong_plane = metrics["top_z_err_mm"] > 15.0
                    if not correct:
                        writer.freeze_t2(
                            case, comp, result, "a3_wrong_or_invalid")
                else:
                    writer.freeze_t2(case, comp, result, "a3_fail_closed")
                row = {
                    "case_id": case.case_id, "valid": valid,
                    "correct": correct, "wrong_plane_valid": wrong_plane,
                    "latency_ms": latency_ms,
                    "component_reason":
                        None if comp is None else comp.reason,
                    "result_reason":
                        None if result is None else result.reason,
                    "background_frac": float(frac),
                }
                rows.append(row)
                writer.t1({"gate": "A3", **row})
    correct_count = sum(1 for r in rows if r["correct"])
    wrong_count = sum(1 for r in rows if r["wrong_plane_valid"])
    pass_all = (
        len(rows) == 36
        and correct_count >= int(spec["thresholds"]["min_correct"])
        and wrong_count == int(spec["thresholds"]["wrong_plane_valid"]))
    return {
        "gate": "A3", "pass": bool(pass_all), "cases": len(rows),
        "correct": correct_count,
        "min_correct": int(spec["thresholds"]["min_correct"]),
        "wrong_plane_valid": wrong_count,
        "fail_closed": sum(1 for r in rows if not r["valid"]),
        "failures": [r["case_id"] for r in rows if not r["correct"]],
    }


def gate_a4(cfg, camera, component_cfg, dynamic_cfg, writer):
    """48-case PCA-bias rectangle-recovery matrix."""
    spec = cfg["matrices"]["a4"]
    th = spec["thresholds"]
    renderer = NadirRenderer(camera)
    stamps = iter(_stamp_seq(3000.0))
    rows = []
    for size_id, whd in DEFAULT_CATALOG:
        for bias in spec["biases_deg"]:
            for seed in spec["seeds"]:
                case = make_a4_case(renderer, size_id, tuple(whd),
                                    float(bias), int(seed), next(stamps),
                                    component_cfg=component_cfg)
                comp, result = _run_case(case, component_cfg, dynamic_cfg)
                valid = result is not None and result.reason == "ok"
                yaw_err = size_err = None
                if valid:
                    metrics = _metrics_of(case, result)
                    yaw_err = metrics["yaw_err_deg"]
                    size_err = max(metrics["width_err_mm"],
                                   metrics["depth_err_mm"])
                    recovered = (yaw_err <= float(th["yaw_err_deg"])
                                 and size_err <= float(th["size_err_mm"]))
                    never_ok = (yaw_err <= float(th["never_yaw_err_deg"])
                                and size_err
                                <= float(th["never_size_err_mm"]))
                    if not recovered or not never_ok:
                        writer.freeze_t2(
                            case, comp, result, "a4_out_of_tolerance")
                else:
                    recovered = False
                    never_ok = True
                    writer.freeze_t2(case, comp, result, "a4_fail_closed")
                rows.append({
                    "case_id": case.case_id, "valid": valid,
                    "recovered": bool(valid and recovered),
                    "within_never_bounds": bool(never_ok),
                    "yaw_err_deg": yaw_err, "size_err_mm": size_err,
                    "measured_bias_deg":
                        case.meta.get("measured_bias_deg"),
                    "component_reason":
                        None if comp is None else comp.reason,
                })
                writer.t1({"gate": "A4", **rows[-1]})
    recovered = sum(1 for r in rows if r["recovered"])
    violations = sum(1 for r in rows if not r["within_never_bounds"])
    pass_all = (len(rows) == 48
                and recovered >= int(th["min_recovered"])
                and violations == 0)
    return {
        "gate": "A4", "pass": bool(pass_all), "cases": len(rows),
        "recovered": recovered, "min_recovered": int(th["min_recovered"]),
        "never_bound_violations": violations,
        "failures": [r["case_id"] for r in rows if not r["recovered"]],
    }


def _stamp_seq(start):
    stamp = float(start)
    while True:
        stamp += 0.1
        yield round(stamp, 3)


def _result_md(gate_results, passed, manifest):
    lines = [
        "# dynamic_suction_acceptance — gates A0-A4",
        "",
        "- commit: `%s` (dirty: %d)" % (manifest["git_commit"],
                                        manifest["git_dirty_count"]),
        "- config sha256: `%s`" % manifest["config_sha256"],
        "- overall: **%s**" % ("pass" if passed else "FAIL"),
        "- A5: deferred (user decision 2026-09-15; real-cell recording "
        "scheduled separately)",
        "",
        "| gate | pass | key metrics |",
        "|---|---|---|",
    ]
    for g in gate_results:
        key = []
        for k in ("valid_rate", "correct", "recovered", "cases",
                  "wrong_plane_valid", "false_top_count"):
            if k in g:
                key.append("%s=%s" % (k, g[k]))
        lines.append("| %s | %s | %s |" % (
            g.get("gate"), "yes" if g.get("pass") else "NO",
            ", ".join(key) or "-"))
    lines.append("")
    return "\n".join(lines)


def run_acceptance(config_path, out_dir, soak_seconds=None):
    cfg = _load_yaml(config_path)
    camera, component_cfg, dynamic_cfg = _configs_from_yaml(cfg)
    writer = EvidenceWriter(out_dir, config_path, cfg)
    gate_results = []
    try:
        gate_results.append(gate_a0(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_a1(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_a2(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_a3(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_a4(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_b0(
            cfg, camera, component_cfg, dynamic_cfg, writer, config_path))
        gate_results.append(gate_b1(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_b2(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_b3(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_b4(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_b5(
            cfg, camera, component_cfg, dynamic_cfg, writer))
        gate_results.append(gate_b6(
            cfg, camera, component_cfg, dynamic_cfg, writer,
            soak_seconds=soak_seconds))
        gate_results.append(gate_b7(
            cfg, camera, component_cfg, dynamic_cfg, writer, config_path))
        gate_results.append({
            "gate": "A5", "pass": None, "status": "deferred",
            "reason": "real-cell RGB-D recording deferred by user "
                      "decision 2026-09-15; not scored in this run"})
    finally:
        passed = all(
            g.get("pass") is True for g in gate_results
            if g.get("pass") is not None)
        writer.finish(gate_results, passed)
    return passed, gate_results


def replay_case(config_path, case_dir, stage):
    """Re-run one frozen T2 case at the requested stage and diff."""
    if stage not in STAGES:
        raise SystemExit("--stage must be one of %s" % (STAGES,))
    cfg = _load_yaml(config_path)
    camera, component_cfg, dynamic_cfg = _configs_from_yaml(cfg)
    with open(os.path.join(case_dir, "T2.json")) as fh:
        payload = json.load(fh)
    data = np.load(os.path.join(case_dir, "case.npz"))
    case_payload = payload["case"]
    renderer = NadirRenderer(camera)
    from luggage_perception.eval.dynamic_suction_renderer import (
        SyntheticCase)
    case = SyntheticCase(
        case_id=case_payload["case_id"], gate=case_payload["gate"],
        depth_mm=data["depth_mm"], bbox=tuple(int(v) for v in data["bbox"]),
        camera=camera, gt_center_xy=case_payload["gt_center_xy"],
        gt_top_z=case_payload["gt_top_z"],
        gt_yaw_deg=case_payload["gt_yaw_deg"],
        gt_size_wh=case_payload["gt_size_wh"],
        stamp=case_payload["stamp"], seed=case_payload["seed"],
        meta=case_payload["meta"])
    if stage == "ingest":
        ok = (case.depth_mm.shape == (camera["height"], camera["width"])
              and len(case.bbox) == 4)
        print(json.dumps({"stage": stage, "bundle_ok": bool(ok)}))
        return 0
    comp = isolate_depth_component(
        case.depth_mm, bbox=case.bbox, config=component_cfg)
    if stage == "segmentation":
        stored = payload.get("component_reason")
        agree = (stored is None and comp is None) or (
            stored is not None and comp is not None
            and stored == comp.reason)
        print(json.dumps({"stage": stage, "reason": comp.reason,
                          "stored_reason": stored, "agree": bool(agree)}))
        return 0 if agree else 2
    comp2, result = _run_case(case, component_cfg, dynamic_cfg)
    stored_result = payload.get("result") or {}
    if stage in ("cargo", "top"):
        agree = (
            (comp2 is not None and comp2.reason == (
                payload.get("component_reason") or comp2.reason))
            and ((result is None and not stored_result)
                 or (result is not None and stored_result
                     and result.reason == stored_result.get("reason")
                     and abs(float(result.top_z)
                             - float(stored_result.get("top_z", 0.0)))
                     <= 1e-12
                     and abs(float(result.yaw)
                             - float(stored_result.get("yaw", 0.0)))
                     <= 1e-12)))
        print(json.dumps({
            "stage": stage,
            "reason": None if result is None else result.reason,
            "stored_reason": stored_result.get("reason"),
            "agree": bool(agree)}))
        return 0 if agree else 2
    return 2


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="python3 -m luggage_perception.eval.dynamic_suction_acceptance")
    parser.add_argument("--config", required=True,
                        help="locked acceptance YAML")
    parser.add_argument("--out", required=True,
                        help="evidence output directory")
    parser.add_argument("--replay", default=None,
                        help="frozen T2 case directory to replay")
    parser.add_argument("--stage", default="top", choices=STAGES,
                        help="replay boundary stage")
    parser.add_argument("--soak-seconds", type=float, default=None,
                        help="override the B6 RSS soak duration")
    args = parser.parse_args(argv)
    if args.replay:
        return replay_case(args.config, args.replay, args.stage)
    passed, gate_results = run_acceptance(
        args.config, args.out, soak_seconds=args.soak_seconds)
    for g in gate_results:
        print("%s: %s" % (g.get("gate"),
                          "pass" if g.get("pass") else (
                              "deferred" if g.get("pass") is None else
                              "FAIL")))
    print("evidence: %s" % os.path.abspath(args.out))
    return 0 if passed else 1




# --- ST-2 gates (B0-B7) ----------------------------------------------------

def _model_from_cfg(cfg, footprint=None):
    from luggage_description.suction_contact_model import (
        ContactModel,
        validate_contact_model_fields,
    )
    fields = {
        "model_version": 1,
        "contact_frame": "suction_contact_frame",
        "footprint_type": "rectangle",
        "boundary_margin_m": 0.015,
        "cell_size_m": 0.005,
        "candidate_grid_m": 0.010,
        "min_valid_cell_fraction": 0.90,
        "min_mask_coverage": 0.95,
        "min_connected_plane_fraction": 0.90,
        "max_rms_residual_m": 0.0025,
        "max_p95_residual_m": 0.0040,
        "max_peak_to_valley_m": 0.0060,
        "max_normal_deviation_p95_deg": 5.0,
        "max_adjacent_step_m": 0.0040,
        "adjacent_step_distance_m": 0.010,
        "bimodal_min_separation_m": 0.0050,
        "bimodal_min_fraction": 0.15,
        "max_normal_tilt_deg": 8.0,
        "max_candidates": 5,
        "min_candidate_separation_m": 0.050,
        "max_candidate_iou": 0.25,
        "max_rejected_diagnostics": 64,
    }
    size = footprint if footprint is not None else 0.18
    fields["footprint_size_xy_m"] = [size, size]
    return ContactModel(**validate_contact_model_fields(fields))


def _run_suction(case, model, camera, component_cfg, dynamic_cfg,
                 instance_id="box-1", generation=7):
    """component -> dynamic top -> suction evaluation for one case."""
    comp = isolate_depth_component(
        case.depth_mm, bbox=case.bbox, config=component_cfg)
    if not comp.ok:
        return comp, None, None
    x0, y0, x1, y1 = case.bbox
    region = np.zeros(case.depth_mm.shape, dtype=bool)
    region[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = True
    intr = _intrinsics(case.camera)
    top = estimate_dynamic_top_surface(
        case.depth_mm, comp.mask, intr, case.mat4, stamp=case.stamp,
        frame=case.frame, instance_region=region, config=dynamic_cfg)
    if top.reason != "ok":
        return comp, top, None
    from luggage_perception.suction_patch_evaluator import (
        SuctionPatchEvaluator,
    )
    evaluator = SuctionPatchEvaluator(model)
    evaluator.update(case.depth_mm, intr, case.mat4, top,
                     instance_region=region, stamp=case.stamp,
                     frame_id=case.frame, instance_id=instance_id,
                     generation=generation)
    return comp, top, evaluator.copy_output()


def _polygon_region(case):
    """Rasterize the case's true top polygon into an instance mask."""
    from luggage_perception.eval.dynamic_suction_renderer import (
        box_polygon,
    )
    meta = case.meta or {}
    size = case.gt_size_wh
    poly = box_polygon(size, case.gt_center_xy, case.gt_yaw_deg)
    cam = case.camera
    oz = cam["optical_to_world"][2][3] - case.gt_top_z
    uu, vu = np.meshgrid(np.arange(cam["width"]),
                         np.arange(cam["height"]))
    wx = cam["optical_to_world"][0][3] + (uu - cam["cx"]) * oz / cam["fx"]
    wy = cam["optical_to_world"][1][3] - (vu - cam["cy"]) * oz / cam["fy"]
    region = np.zeros(case.depth_mm.shape, dtype=bool)
    inside_pos = np.ones(region.shape, dtype=bool)
    inside_neg = np.ones(region.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cross = (x1 - x0) * (wy - y0) - (y1 - y0) * (wx - x0)
        inside_pos &= cross >= 0.0
        inside_neg &= cross <= 0.0
    region |= inside_pos | inside_neg
    del meta
    return region


def _run_suction_region(case, model, camera, component_cfg, dynamic_cfg,
                        region, instance_id="box-1", generation=7):
    """_run_suction with an explicit instance-region override."""
    comp = isolate_depth_component(
        case.depth_mm, bbox=case.bbox, config=component_cfg)
    if not comp.ok:
        return comp, None, None
    intr = _intrinsics(case.camera)
    top = estimate_dynamic_top_surface(
        case.depth_mm, comp.mask, intr, case.mat4, stamp=case.stamp,
        frame=case.frame, instance_region=region, config=dynamic_cfg)
    if top.reason != "ok":
        return comp, top, None
    from luggage_perception.suction_patch_evaluator import (
        SuctionPatchEvaluator,
    )
    evaluator = SuctionPatchEvaluator(model)
    evaluator.update(case.depth_mm, intr, case.mat4, top,
                     instance_region=region, stamp=case.stamp,
                     frame_id=case.frame, instance_id=instance_id,
                     generation=generation)
    return comp, top, evaluator.copy_output()


def gate_b0(cfg, camera, component_cfg, dynamic_cfg, writer, config_path):
    """B0: schema rejections with stable reasons + shipped config load."""
    from luggage_description.suction_contact_model import (
        SuctionContactModelError,
        load_suction_contact_model,
    )
    import tempfile
    checks = {}
    bad_yaml = {
        "missing": "model_version: 1\ncontact_frame: f\n",
        "dup": "model_version: 1\nmodel_version: 2\n",
        "nan": "model_version: 1\ncontact_frame: f\n"
               "footprint_type: rectangle\nfootprint_size_xy_m: "
               "[.nan, .nan]\n",
        "oversize": "model_version: 1\ncontact_frame: f\n"
                    "footprint_type: rectangle\nfootprint_size_xy_m: "
                    "[0.31, 0.31]\n",
    }
    for name, text in bad_yaml.items():
        with tempfile.NamedTemporaryFile("w", suffix=".yaml",
                                         delete=False) as fh:
            fh.write(text)
            path = fh.name
        try:
            try:
                load_suction_contact_model(path)
                checks["reject_%s" % name] = False
            except SuctionContactModelError:
                checks["reject_%s" % name] = True
        finally:
            os.unlink(path)
    shipped = os.path.join(os.path.dirname(os.path.abspath(config_path)),
                           os.pardir, os.pardir, "luggage_description",
                           "config", "suction_contact_model.yaml")
    try:
        model = load_suction_contact_model(shipped)
        checks["shipped_loads"] = (
            model.model_version == 1
            and model.footprint_size_xy_m == (0.18, 0.18)
            and len(model.identity_hash) == 64)
    except (SuctionContactModelError, OSError):
        checks["shipped_loads"] = False
    writer.t1({"gate": "B0", "checks": checks})
    return {"gate": "B0", "pass": all(checks.values()), "checks": checks}


def gate_b1(cfg, camera, component_cfg, dynamic_cfg, writer):
    """B1: 243-case uniformly planar matrix."""
    spec = cfg["matrices"]["b1"]
    th = spec["thresholds"]
    renderer = NadirRenderer(camera)
    from luggage_perception.eval.dynamic_suction_renderer import make_b1_case
    stamps = iter(_stamp_seq(5000.0))
    rows = []
    for fp in spec["footprints_m"]:
        model = _model_from_cfg(cfg, footprint=fp)
        for tilt in spec["tilts_deg"]:
            for noise in spec["noise_sigma_mm"]:
                for yaw in spec["yaws_deg"]:
                    for seed in spec["seeds"]:
                        case, gt_normal = make_b1_case(
                            renderer, float(tilt), float(yaw),
                            float(noise), int(seed), next(stamps))
                        # B1 uses the true pixel instance mask (the
                        # plan's preferred backend input), so mask
                        # coverage forces the shrunk footprint inside
                        # the true surface.
                        t0 = time.monotonic()
                        comp, top, out = _run_suction_region(
                            case, model, camera, component_cfg, dynamic_cfg,
                            region=_polygon_region(case))
                        latency_ms = (time.monotonic() - t0) * 1000.0
                        present = out is not None and len(out.accepted) >= 1
                        row = {
                            "case_id": case.case_id, "present": present,
                            "latency_ms": latency_ms,
                            "noise": float(noise), "tilt": float(tilt),
                        }
                        if present:
                            rec = out.accepted[0]
                            normal = np.asarray(rec.normal_world)
                            err = math.degrees(math.acos(min(
                                1.0, abs(float(normal @ gt_normal)))))
                            row["normal_err_deg"] = err
                            # Plan B1: the selected position lies inside
                            # the true surface eroded by the physical
                            # boundary margin (15 mm).
                            row["on_surface"] = bool(
                                abs(rec.center_world[0] + 1.0)
                                <= 0.55 / 2 - 0.015
                                and abs(rec.center_world[1])
                                <= 0.40 / 2 - 0.015)
                        else:
                            writer.freeze_t2(
                                case, comp, top, "b1_not_present")
                        rows.append(row)
                        writer.t1({"gate": "B1", **{
                            k: v for k, v in row.items()}})
    present_flags = [r["present"] for r in rows]
    low_noise = [r["present"] for r in rows if r["noise"] <= 1.0]
    normal_errs = [r["normal_err_deg"] for r in rows
                   if "normal_err_deg" in r]
    on_surface = [r.get("on_surface", False) for r in rows]
    pass_all = (
        len(rows) == 243
        and (sum(present_flags) / len(rows)) >= float(
            th["present_rate_overall"])
        and (sum(low_noise) / max(1, len(low_noise)))
        >= float(th["present_rate_low_noise"])
        and all(on_surface)
        and float(np.percentile(normal_errs, 95)) <= th["normal_p95_deg"]
        and max(normal_errs) <= th["normal_max_deg"])
    return {
        "gate": "B1", "pass": bool(pass_all), "cases": len(rows),
        "present_rate": sum(present_flags) / max(1, len(rows)),
        "low_noise_present_rate": sum(low_noise) / max(1, len(low_noise)),
        "on_surface_rate": sum(on_surface) / max(1, len(on_surface)),
        "normal_p95_deg": float(np.percentile(normal_errs, 95)),
        "normal_max_deg": float(max(normal_errs)),
        "failures": [r["case_id"] for r in rows if not r["present"]],
    }


def gate_b2(cfg, camera, component_cfg, dynamic_cfg, writer):
    """B2: 162-case cross-plane rejection matrix."""
    spec = cfg["matrices"]["b2"]
    th = spec["thresholds"]
    renderer = NadirRenderer(camera)
    from luggage_perception.eval.dynamic_suction_renderer import make_b2_case
    model = _model_from_cfg(cfg, footprint=float(spec["footprint_m"]))
    half = float(spec["footprint_m"]) / 2.0
    stamps = iter(_stamp_seq(6000.0))
    unsafe = 0
    rows = []
    for step in spec["steps_mm"]:
        for angle in spec["boundary_angles_deg"]:
            for offset in spec["boundary_offsets_mm"]:
                for seed in spec["seeds"]:
                    case, _gt = make_b2_case(
                        renderer, float(step), float(angle),
                        float(offset), int(seed), next(stamps))
                    comp, top, out = _run_suction(
                        case, model, camera, component_cfg, dynamic_cfg)
                    crossing_accepted = 0
                    ang = math.radians(float(angle))
                    # Same convention as the renderer: n = (-sin, cos),
                    # boundary through centre + n * offset.
                    line_nrm = np.array([-math.sin(ang), math.cos(ang)])
                    c_line = float(offset) * 0.001
                    # Plan B2: only a step >= 6 mm makes a crossing
                    # unsafe; below-threshold crossings are diagnostics.
                    if out is not None and float(step) >= 6.0:
                        for rec in out.accepted:
                            dist = abs(rec.center_world[0] * line_nrm[0]
                                       + rec.center_world[1] * line_nrm[1]
                                       - c_line)
                            if dist < half - 1e-6:
                                crossing_accepted += 1
                    unsafe += crossing_accepted
                    if crossing_accepted:
                        writer.freeze_t2(
                            case, comp, top, "b2_unsafe_accept")
                    rows.append({
                        "case_id": case.case_id,
                        "unsafe": crossing_accepted,
                        "step_mm": float(step)})
                    writer.t1({"gate": "B2", **rows[-1]})
    pass_all = len(rows) == 162 and unsafe == int(th["unsafe_accept_count"])
    return {"gate": "B2", "pass": bool(pass_all), "cases": len(rows),
            "unsafe_accept_count": unsafe,
            "failures": [r["case_id"] for r in rows if r["unsafe"]]}


def gate_b3(cfg, camera, component_cfg, dynamic_cfg, writer):
    """B3: 45-case island matrix (36 interior + 9 edge)."""
    spec = cfg["matrices"]["b3"]
    th = spec["thresholds"]
    renderer = NadirRenderer(camera)
    from luggage_perception.eval.dynamic_suction_renderer import make_b3_case
    model = _model_from_cfg(cfg, footprint=float(spec["footprint_m"]))
    stamps = iter(_stamp_seq(7000.0))
    on_island = 0
    edge_closed = 0
    rows = []
    for pos in spec["interior_positions"]:
        for yaw in spec["yaws_deg"]:
            for seed in spec["seeds"]:
                case = make_b3_case(renderer, tuple(pos), float(yaw),
                                    int(seed), next(stamps))
                comp, top, out = _run_suction(
                    case, model, camera, component_cfg, dynamic_cfg)
                ok = out is not None and len(out.accepted) >= 1 and all(
                    abs(rec.center_world[0] - pos[0]) <= 0.32 / 2 - 0.09
                    + 0.01 and abs(rec.center_world[1] - pos[1])
                    <= 0.32 / 2 - 0.09 + 0.01 for rec in out.accepted)
                on_island += int(ok)
                if not ok:
                    writer.freeze_t2(case, comp, top, "b3_island_miss")
                rows.append({"case_id": case.case_id, "ok": bool(ok),
                             "kind": "interior"})
    for yaw in spec["yaws_deg"]:
        for seed in spec["seeds"]:
            case = make_b3_case(renderer, tuple(spec["edge_position"]),
                                float(yaw), int(seed), next(stamps),
                                island_size=0.15)
            comp, top, out = _run_suction(
                case, model, camera, component_cfg, dynamic_cfg)
            closed = out is None or len(out.accepted) == 0
            edge_closed += int(closed)
            if not closed:
                writer.freeze_t2(case, comp, top, "b3_edge_accepted")
            rows.append({"case_id": case.case_id, "ok": bool(closed),
                         "kind": "edge"})
    for row in rows:
        writer.t1({"gate": "B3", **row})
    pass_all = (
        len(rows) == 45
        and on_island >= int(th["interior_on_island"])
        and edge_closed >= int(th["edge_fail_closed"]))
    return {"gate": "B3", "pass": bool(pass_all), "cases": len(rows),
            "interior_on_island": on_island, "edge_fail_closed": edge_closed,
            "failures": [r["case_id"] for r in rows if not r["ok"]]}


def gate_b4(cfg, camera, component_cfg, dynamic_cfg, writer):
    """B4: 30-case no-seal matrix (zero candidates everywhere)."""
    spec = cfg["matrices"]["b4"]
    renderer = NadirRenderer(camera)
    from luggage_perception.eval.dynamic_suction_renderer import make_b4_case
    model = _model_from_cfg(cfg, footprint=float(spec["footprint_m"]))
    stamps = iter(_stamp_seq(8000.0))
    zero = 0
    rows = []
    for kind in spec["kinds"]:
        for seed in spec["seeds"]:
            case = make_b4_case(renderer, kind, int(seed), next(stamps))
            comp, top, out = _run_suction(
                case, model, camera, component_cfg, dynamic_cfg)
            n_acc = 0 if out is None else len(out.accepted)
            zero += int(n_acc == 0)
            if n_acc:
                writer.freeze_t2(case, comp, top, "b4_candidate_leak")
            rows.append({"case_id": case.case_id, "candidates": n_acc,
                         "kind": kind})
            writer.t1({"gate": "B4", **rows[-1]})
    pass_all = (len(rows) == 30
                and zero >= int(spec["thresholds"]["zero_candidate_cases"]))
    return {"gate": "B4", "pass": bool(pass_all), "cases": len(rows),
            "zero_candidate_cases": zero,
            "failures": [r["case_id"] for r in rows if r["candidates"]]}


def gate_b5(cfg, camera, component_cfg, dynamic_cfg, writer):
    """B5: identity/time injection via the consumer-side gate."""
    from luggage_perception.suction_patch_evaluator import (
        suction_identity_mismatch,
    )
    renderer = NadirRenderer(camera)
    from luggage_perception.eval.dynamic_suction_renderer import make_b1_case
    case, _n = make_b1_case(renderer, 3.0, 30.0, 1.0, 11, 9.0)
    model = _model_from_cfg(cfg)
    comp, top, out = _run_suction(case, model, camera, component_cfg,
                                  dynamic_cfg, instance_id="obs-1",
                                  generation=4)
    if out is None or not out.accepted:
        writer.freeze_t2(case, comp, top, "b5_reference_failed")
        return {"gate": "B5", "pass": False,
                "reason": "reference case produced no candidate"}
    record = out.accepted[0]
    base = dict(stamp=case.stamp, frame_id=case.frame,
                instance_id="obs-1", generation=4)
    mismatches = []
    for name, override in (("stamp", {"stamp": case.stamp + 1.1}),
                           ("frame", {"frame_id": "other"}),
                           ("generation", {"generation": 5}),
                           ("instance", {"instance_id": "obs-2"})):
        fields = dict(base)
        fields.update(override)
        result = suction_identity_mismatch(record, **fields)
        mismatches.append(result is not None
                          and result[0] == "SUCTION_CANDIDATE_"
                          "IDENTITY_MISMATCH")
    clean = suction_identity_mismatch(record, **base) is None
    pass_all = all(mismatches) and clean
    writer.t1({"gate": "B5", "injections": mismatches, "clean": clean})
    return {"gate": "B5", "pass": bool(pass_all),
            "injection_rejections": sum(mismatches),
            "clean_accept": clean}


def gate_b6(cfg, camera, component_cfg, dynamic_cfg, writer,
            soak_seconds=None):
    """B6: determinism, bounds, latency, bounded RSS soak."""
    spec = cfg["matrices"]["b6"]
    renderer = NadirRenderer(camera)
    from luggage_perception.eval.dynamic_suction_renderer import (
        make_b1_case,
        make_b4_case,
    )
    from luggage_perception.suction_patch_evaluator import (
        SuctionPatchEvaluator,
    )
    model = _model_from_cfg(cfg)
    cases = [make_b1_case(renderer, 0.0, 30.0, 2.0, 11, 10.0)[0],
             make_b1_case(renderer, 6.0, 0.0, 1.0, 29, 10.1)[0],
             make_b4_case(renderer, "ridge", 11, 10.2)]
    repeats = int(spec["repeat_count"])
    identical = True
    bounds_ok = True
    latencies = []
    intr = _intrinsics(camera)
    for case in cases:
        comp, top, _first = _run_suction(
            case, model, camera, component_cfg, dynamic_cfg)
        if top is None or top.reason != "ok":
            continue
        x0, y0, x1, y1 = case.bbox
        region = np.zeros(case.depth_mm.shape, dtype=bool)
        region[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = True
        evaluator = SuctionPatchEvaluator(model)
        signatures = []
        for _ in range(repeats):
            # Plan B6 measures the EVALUATOR (height map + candidates),
            # not the upstream dynamic-top chain.
            t0 = time.monotonic()
            evaluator.update(
                case.depth_mm, intr, case.mat4, top,
                instance_region=region, stamp=case.stamp,
                frame_id=case.frame, instance_id="b6", generation=1)
            latencies.append((time.monotonic() - t0) * 1000.0)
            out = evaluator.copy_output()
            signatures.append((
                tuple((c.candidate_id, c.rank) for c in out.accepted),
                tuple((r[0], r[1]) for r in out.rejected)))
            if (len(out.accepted) > int(spec["max_accepted"])
                    or len(out.rejected) > int(spec["max_rejected"])):
                bounds_ok = False
        identical = identical and len(set(signatures)) == 1
    p95_ms = float(np.percentile(latencies, 95)) if latencies else 0.0
    # Bounded RSS soak: evaluate repeatedly and measure the slope.
    soak = float(soak_seconds if soak_seconds is not None
                 else spec.get("soak_seconds", 1200))
    import resource
    case = cases[0]
    comp, top, _first = _run_suction(
        case, model, camera, component_cfg, dynamic_cfg)
    x0, y0, x1, y1 = case.bbox
    region = np.zeros(case.depth_mm.shape, dtype=bool)
    region[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = True
    evaluator = SuctionPatchEvaluator(model)
    start_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    start_t = time.monotonic()
    runs = 0
    while time.monotonic() - start_t < soak:
        evaluator.update(
            case.depth_mm, intr, case.mat4, top,
            instance_region=region, stamp=case.stamp,
            frame_id=case.frame, instance_id="b6", generation=1)
        runs += 1
    end_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    minutes = max(1e-6, (time.monotonic() - start_t) / 60.0)
    slope = (end_rss - start_rss) / 1024.0 / minutes
    pass_all = (identical and bounds_ok
                and p95_ms <= float(spec["evaluator_p95_ms"])
                and slope <= float(spec["rss_slope_mib_per_min"]))
    result = {
        "gate": "B6", "pass": bool(pass_all),
        "determinism": bool(identical), "bounds_ok": bool(bounds_ok),
        "evaluator_p95_ms": p95_ms,
        "soak_seconds": soak, "soak_runs": runs,
        "rss_slope_mib_per_min": slope,
        "rss_note": "ru_maxrss high-water slope; %d s soak" % int(soak),
    }
    writer.t1({"gate": "B6", **{k: v for k, v in result.items()
                                if k != "rss_note"}})
    return result


def gate_b7(cfg, camera, component_cfg, dynamic_cfg, writer,
            config_path):
    """B7: fixed 12-case overlay fixture vs golden JSON (2 px)."""
    spec = cfg["matrices"]["b7"]
    fixture_rel = spec["fixture"]
    fixture = os.path.join(os.path.dirname(os.path.abspath(config_path)),
                           os.pardir, "test", "fixtures",
                           "suction_b7", "golden.json")
    if not os.path.exists(fixture):
        return {"gate": "B7", "pass": False,
                "reason": "golden fixture missing at %s" % fixture}
    with open(fixture) as fh:
        golden = json.load(fh)
    from luggage_perception.eval.suction_debug_overlay import (
        overlay_for_case,
    )
    renderer = NadirRenderer(camera)
    model = _model_from_cfg(cfg)
    half = (model.footprint_size_xy_m[0] / 2.0,
            model.footprint_size_xy_m[1] / 2.0)
    tol = float(spec["pixel_tolerance"])
    max_err = 0.0
    ok = True
    for entry in golden["cases"]:
        case = _case_from_overlay_entry(renderer, entry)
        comp, top, out = _run_suction(
            case, model, camera, component_cfg, dynamic_cfg)
        overlay = overlay_for_case(case, top, out, half)
        got_by_id = {c["candidate_id"]: c
                     for c in overlay["candidates"]}
        for want in entry["expected"]:
            got = got_by_id.get(want["candidate_id"])
            if got is None or got["colour"] != want["colour"]:
                ok = False
                break
            if want["corner_pixels"] is not None:
                if got["corner_pixels"] is None:
                    ok = False
                    break
                for got_p, want_p in zip(got["corner_pixels"],
                                         want["corner_pixels"]):
                    max_err = max(max_err, math.hypot(
                        got_p[0] - want_p[0], got_p[1] - want_p[1]))
        if not ok or max_err > tol:
            writer.freeze_t2(case, comp, top, "b7_overlay_mismatch")
            ok = ok and max_err <= tol
            break
    writer.t1({"gate": "B7", "cases": len(golden["cases"]),
               "max_pixel_error": max_err, "pass": bool(ok)})
    return {"gate": "B7", "pass": bool(ok),
            "cases": len(golden["cases"]),
            "max_pixel_error": max_err}


def _case_from_overlay_entry(renderer, entry):
    from luggage_perception.eval.dynamic_suction_renderer import (
        SyntheticCase,
        make_b1_case,
        make_b2_case,
        make_b3_case,
    )
    kind = entry["kind"]
    if kind == "b1":
        case, _ = make_b1_case(renderer, float(entry["tilt_deg"]),
                               float(entry["yaw_deg"]),
                               float(entry["noise_sigma_mm"]),
                               int(entry["seed"]), float(entry["stamp"]))
    elif kind == "b2":
        case, _ = make_b2_case(renderer, float(entry["step_mm"]),
                               float(entry["boundary_angle_deg"]),
                               float(entry["boundary_offset_mm"]),
                               int(entry["seed"]), float(entry["stamp"]))
    else:
        case = make_b3_case(renderer, tuple(entry["island_xy"]),
                            float(entry.get("yaw_deg", 0.0)),
                            int(entry["seed"]), float(entry["stamp"]),
                            island_size=float(entry.get("island_size",
                                                        0.32)))
    return case


if __name__ == "__main__":
    sys.exit(main())
