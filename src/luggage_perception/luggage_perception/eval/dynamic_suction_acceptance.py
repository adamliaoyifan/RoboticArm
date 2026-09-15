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


def run_acceptance(config_path, out_dir):
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
    args = parser.parse_args(argv)
    if args.replay:
        return replay_case(args.config, args.replay, args.stage)
    passed, gate_results = run_acceptance(args.config, args.out)
    for g in gate_results:
        print("%s: %s" % (g.get("gate"),
                          "pass" if g.get("pass") else (
                              "deferred" if g.get("pass") is None else
                              "FAIL")))
    print("evidence: %s" % os.path.abspath(args.out))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
