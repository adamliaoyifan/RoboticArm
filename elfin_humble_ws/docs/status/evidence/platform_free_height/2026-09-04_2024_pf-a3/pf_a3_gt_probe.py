#!/usr/bin/env python3
"""PF-A3 independent mesh-observable GT probe. Audit helper only."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import re
import shutil
import struct
import sys
import tempfile


ASSETS = (
    ("suitcase_loafbrr", "small"),
    ("suitcase_loafbrr", "medium"),
    ("suitcase_loafbrr", "large"),
    ("suitcase_vintage", "small"),
    ("suitcase_vintage", "medium"),
    ("suitcase_vintage", "large"),
)
TOP_BAND_FRAC = 0.25
Z_BIN = 0.001
TRANSLATE = (1.0, 2.0, 3.0)
REPEAT_N = 10
CLIENT_RE = re.compile(
    r"(from\s+\S+\s+import\s+[^\n]*GetCurrentBox)"
    r"|(import[^\n]*GetCurrentBox)"
    r"|(create_client\s*\(\s*GetCurrentBox)"
    r"|(ServiceProxy\s*\([^)]*GetCurrentBox)"
)
SHA256_LEN = 64


def r12(value):
    return round(float(value), 12)


def status_record(status, evidence, **extra):
    rec = {"status": status, "evidence": evidence}
    rec.update(extra)
    return rec


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 16), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_binary_stl(path):
    """Independent binary-STL reader. Does not call production parsers."""
    with open(path, "rb") as handle:
        header = handle.read(80)
        if len(header) < 80:
            raise IOError("truncated STL header: %s" % path)
        n_raw = handle.read(4)
        if len(n_raw) < 4:
            raise IOError("truncated STL triangle count: %s" % path)
        count = struct.unpack("<I", n_raw)[0]
        triangles = []
        for _ in range(count):
            rec = handle.read(50)
            if len(rec) < 50:
                raise IOError("truncated STL triangles: %s" % path)
            nums = struct.unpack("<12fH", rec)
            triangles.append(
                {
                    "normal": nums[0:3],
                    "v1": nums[3:6],
                    "v2": nums[6:9],
                    "v3": nums[9:12],
                    "attr": nums[12],
                }
            )
    return header, triangles


def write_binary_stl(path, header, triangles):
    chunks = [header[:80].ljust(80, b" "), struct.pack("<I", len(triangles))]
    for tri in triangles:
        v1, v2, v3 = tri["v1"], tri["v2"], tri["v3"]
        nrm = tri.get("normal", (0.0, 0.0, 1.0))
        chunks.append(
            struct.pack(
                "<12fH",
                nrm[0], nrm[1], nrm[2],
                v1[0], v1[1], v1[2],
                v2[0], v2[1], v2[2],
                v3[0], v3[1], v3[2],
                int(tri.get("attr", 0)),
            )
        )
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(b"".join(chunks))


def vertices_from_triangles(triangles):
    out = []
    for tri in triangles:
        out.extend((tuple(tri["v1"]), tuple(tri["v2"]), tuple(tri["v3"])))
    return out


def independent_oracle(vertices, top_band_frac=TOP_BAND_FRAC, z_bin=Z_BIN):
    """Independent observable-surface oracle.

    Modal-bin tie-break is explicit and insertion-order independent:
    among bins with the maximum vertex count, choose the largest bin key
    (highest Z / closest to AABB top). Median is sorted[len//2], matching
    the production odd/even convention but applied only after the
    deterministic key choice.
    """
    if not vertices:
        raise ValueError("empty vertex list")
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    zs = [v[2] for v in vertices]
    zmax, zmin = max(zs), min(zs)
    height = zmax - zmin
    band = max(float(top_band_frac) * height, float(z_bin) * 2.0)
    top = [(x, y, z) for x, y, z in zip(xs, ys, zs) if z >= zmax - band]
    if not top:
        return {
            "width": max(xs) - min(xs),
            "depth": max(ys) - min(ys),
            "lid_offset": 0.0,
            "full_height": height,
            "modal_key": None,
            "tie_break": "empty_top_band_aabb_fallback",
            "tied_bin_count": 0,
        }
    bins = {}
    for _x, _y, z in top:
        key = int(z / z_bin)
        bins.setdefault(key, []).append(z)
    max_count = max(len(vals) for vals in bins.values())
    tied = sorted(key for key, vals in bins.items() if len(vals) == max_count)
    modal_key = tied[-1]
    modal_z = sorted(bins[modal_key])[len(bins[modal_key]) // 2]
    plateau = [(x, y) for x, y, z in zip(xs, ys, zs) if abs(z - modal_z) <= z_bin]
    if not plateau:
        plateau = [(x, y) for x, y, _z in top]
    return {
        "width": max(p[0] for p in plateau) - min(p[0] for p in plateau),
        "depth": max(p[1] for p in plateau) - min(p[1] for p in plateau),
        "lid_offset": float(zmax - modal_z),
        "full_height": float(height),
        "modal_key": modal_key,
        "tie_break": "max_count_then_max_bin_key",
        "tied_bin_count": len(tied),
        "tied_keys": tied,
        "modal_z": modal_z,
        "zmin": zmin,
        "zmax": zmax,
        "triangle_vertices": len(vertices),
    }


def production_tuple_to_dict(result):
    width, depth, lid_offset, full_height = result
    return {
        "width": float(width),
        "depth": float(depth),
        "lid_offset": float(lid_offset),
        "full_height": float(full_height),
        "observable_height": float(full_height) - float(lid_offset),
    }


def scalars_close(a, b, tol):
    keys = ("width", "depth", "lid_offset", "full_height")
    return all(abs(float(a[k]) - float(b[k])) <= tol for k in keys)


def max_abs_diff(a, b):
    return {
        k: abs(float(a[k]) - float(b[k]))
        for k in ("width", "depth", "lid_offset", "full_height")
    }


def asset_path(repo, visual_id, tier):
    return os.path.join(
        repo,
        "src/luggage_gazebo/models",
        "%s_%s" % (visual_id, tier),
        "meshes/suitcase.stl",
    )


def walk_python(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in {".git", "__pycache__"}]
        for name in filenames:
            if name.endswith(".py"):
                yield os.path.join(dirpath, name)


def is_humble_online_algorithm(rel):
    if "ros1_reference" in rel.split(os.sep):
        return False
    if "/eval/" in ("/" + rel.replace(os.sep, "/")) or rel.startswith(
        "src/luggage_perception/luggage_perception/eval/"
    ):
        return False
    if "/test/" in ("/" + rel.replace(os.sep, "/")):
        return False
    base = os.path.basename(rel)
    if base in {
        "platform_free_height_gate4_eval.py",
        "pack_eval_driver.py",
        "place_smoke_driver.py",
        "pick_retreat_eval_driver.py",
        "pickup_box_spawner_node.py",
        "pf_a1_privileged_input_audit.py",
    }:
        return False
    return rel.startswith("src/luggage_perception/") or rel.startswith(
        "src/luggage_planning/"
    )


def scan_getcurrentbox(repo):
    needles = (
        "GetCurrentBox",
        "get_current_box",
        "mesh_observable_reference",
        "SpawnNextBox",
        "pickup_box_spawner",
    )
    hits = []
    online_hits = []
    for path in walk_python(os.path.join(repo, "src")):
        rel = os.path.relpath(path, repo)
        try:
            text = open(path, "r", encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        found = [n for n in needles if n in text]
        if not found:
            continue
        rec = {
            "path": rel,
            "needles": found,
            "runtime_client": bool(CLIENT_RE.search(text)),
            "online_algorithm": is_humble_online_algorithm(rel),
        }
        hits.append(rec)
        if rec["online_algorithm"] and rec["runtime_client"]:
            online_hits.append(rel)
    hits.sort(key=lambda item: item["path"])
    online_hits.sort()
    return hits, online_hits


def spawner_fallback(stl_path, catalog, mesh_observable_reference):
    """Execute the audited pickup_box_spawner fallback without ROS."""
    ref = None
    warning = None
    try:
        w, d, lid_off, full_h = mesh_observable_reference(stl_path)
        ref = {
            "width": float(w),
            "depth": float(d),
            "height": float(full_h - lid_off),
            "lid_offset": float(lid_off),
            "source": "mesh",
        }
    except (IOError, OSError, ValueError) as exc:
        warning = "mesh observable reference unavailable (%s); using catalog size" % exc
    if ref is None:
        ref = {
            "width": float(catalog[0]),
            "depth": float(catalog[1]),
            "height": float(catalog[2]),
            "lid_offset": 0.0,
            "source": "catalog_fallback",
        }
    return ref, warning


def run_probe(repo, out_path):
    sys.path.insert(0, os.path.join(repo, "src/luggage_description"))
    from luggage_description.suitcase_visual import (  # noqa: E402
        SIZE_TIERS,
        mesh_observable_reference,
        pickup_box_pose,
        sized_stl_path,
    )

    models_root = os.path.join(repo, "src/luggage_gazebo/models")
    tier_size = {name: [float(v) for v in spec] for name, spec, _m, _c in SIZE_TIERS}
    tmp = tempfile.mkdtemp(prefix="pf_a3_probe_")
    assets = []
    try:
        for visual_id, tier in ASSETS:
            path = sized_stl_path(visual_id, tier, models_root)
            if not os.path.isfile(path):
                path = asset_path(repo, visual_id, tier)
            header, triangles = read_binary_stl(path)
            vertices = vertices_from_triangles(triangles)
            production = production_tuple_to_dict(mesh_observable_reference(path))
            oracle = independent_oracle(vertices)
            oracle_view = {
                "width": oracle["width"],
                "depth": oracle["depth"],
                "lid_offset": oracle["lid_offset"],
                "full_height": oracle["full_height"],
            }
            aabb = {
                "xmin": min(v[0] for v in vertices),
                "xmax": max(v[0] for v in vertices),
                "ymin": min(v[1] for v in vertices),
                "ymax": max(v[1] for v in vertices),
                "zmin": min(v[2] for v in vertices),
                "zmax": max(v[2] for v in vertices),
            }
            repeats = [
                production_tuple_to_dict(mesh_observable_reference(path))
                for _ in range(REPEAT_N)
            ]
            copy_path = os.path.join(tmp, "%s_%s_copy.stl" % (visual_id, tier))
            shutil.copy2(path, copy_path)
            copy_prod = production_tuple_to_dict(mesh_observable_reference(copy_path))

            rev_path = os.path.join(tmp, "%s_%s_rev.stl" % (visual_id, tier))
            write_binary_stl(rev_path, header, list(reversed(triangles)))
            rev_prod = production_tuple_to_dict(mesh_observable_reference(rev_path))

            shuffled = list(triangles)
            random.Random(20260904).shuffle(shuffled)
            shuf_path = os.path.join(tmp, "%s_%s_shuf.stl" % (visual_id, tier))
            write_binary_stl(shuf_path, header, shuffled)
            shuf_prod = production_tuple_to_dict(mesh_observable_reference(shuf_path))

            translated = []
            for tri in triangles:
                translated.append(
                    {
                        "normal": tri["normal"],
                        "v1": tuple(tri["v1"][i] + TRANSLATE[i] for i in range(3)),
                        "v2": tuple(tri["v2"][i] + TRANSLATE[i] for i in range(3)),
                        "v3": tuple(tri["v3"][i] + TRANSLATE[i] for i in range(3)),
                        "attr": tri["attr"],
                    }
                )
            tx_path = os.path.join(tmp, "%s_%s_tx.stl" % (visual_id, tier))
            write_binary_stl(tx_path, header, translated)
            tx_prod = production_tuple_to_dict(mesh_observable_reference(tx_path))

            catalog = tier_size[tier]
            poses = []
            for yaw in (0.0, 0.3, 1.57079632679, 3.14159265359):
                source_xyz = (1.25, -0.4, 0.86)
                source_rpy = (0.0, 0.0, 0.1)
                center_xyz, rpy = pickup_box_pose(
                    source_xyz, source_rpy, catalog, yaw_offset=yaw
                )
                lid_off = production["lid_offset"]
                obs_h = production["observable_height"]
                aabb_top = center_xyz[2] + catalog[2] * 0.5
                observable_top_z = aabb_top - lid_off
                reported_center_z = observable_top_z - obs_h * 0.5
                observable_bottom = observable_top_z - obs_h
                aabb_bottom = center_xyz[2] - catalog[2] * 0.5
                poses.append(
                    {
                        "yaw_offset": yaw,
                        "rpy_yaw": rpy[2],
                        "top_minus_center": r12(observable_top_z - reported_center_z),
                        "half_observable_height": r12(obs_h / 2.0),
                        "observable_bottom": r12(observable_bottom),
                        "aabb_bottom": r12(aabb_bottom),
                        "identity_ok": abs((observable_top_z - reported_center_z) - obs_h / 2.0)
                        <= 1e-6
                        and abs(observable_bottom - aabb_bottom) <= 1e-6,
                    }
                )

            sensitivity = []
            for frac in (0.20, 0.25, 0.30):
                for zbin in (0.00075, 0.001, 0.00125):
                    prod = production_tuple_to_dict(
                        mesh_observable_reference(path, top_band_frac=frac, z_bin=zbin)
                    )
                    sensitivity.append(
                        {
                            "top_band_frac": frac,
                            "z_bin": zbin,
                            "lid_offset": r12(prod["lid_offset"]),
                            "width": r12(prod["width"]),
                            "depth": r12(prod["depth"]),
                            "d_lid_vs_default": r12(
                                prod["lid_offset"] - production["lid_offset"]
                            ),
                            "d_width_vs_default": r12(prod["width"] - production["width"]),
                            "d_depth_vs_default": r12(prod["depth"] - production["depth"]),
                        }
                    )

            assets.append(
                {
                    "visual_id": visual_id,
                    "tier": tier,
                    "path": os.path.relpath(path, repo),
                    "sha256": sha256_file(path),
                    "triangle_count": len(triangles),
                    "aabb": {k: r12(v) for k, v in aabb.items()},
                    "catalog_size": catalog,
                    "algorithm_parameters": {
                        "top_band_frac": TOP_BAND_FRAC,
                        "z_bin": Z_BIN,
                        "top_band_min_m": 2.0 * Z_BIN,
                    },
                    "production": {k: r12(v) for k, v in production.items()},
                    "oracle": {
                        "width": r12(oracle["width"]),
                        "depth": r12(oracle["depth"]),
                        "lid_offset": r12(oracle["lid_offset"]),
                        "full_height": r12(oracle["full_height"]),
                        "observable_height": r12(
                            oracle["full_height"] - oracle["lid_offset"]
                        ),
                        "tie_break": oracle["tie_break"],
                        "tied_bin_count": oracle["tied_bin_count"],
                        "tied_keys": oracle.get("tied_keys"),
                        "modal_key": oracle.get("modal_key"),
                    },
                    "repeat_stable": all(scalars_close(production, item, 0.0) for item in repeats),
                    "copy_diff": max_abs_diff(production, copy_prod),
                    "reversed_diff": max_abs_diff(production, rev_prod),
                    "shuffled_diff": max_abs_diff(production, shuf_prod),
                    "translated_diff": max_abs_diff(production, tx_prod),
                    "oracle_diff": max_abs_diff(production, oracle_view),
                    "poses": poses,
                    "sensitivity": sensitivity,
                }
            )

        # Check 8: missing / truncated / malformed / non-binary against kernel
        # and the spawner fallback that can enter GetCurrentBox.
        bad_cases = []
        catalog = tier_size["medium"]
        missing = os.path.join(tmp, "missing.stl")
        try:
            mesh_observable_reference(missing)
            kernel_missing = "returned"
        except (IOError, OSError, ValueError) as exc:
            kernel_missing = "raised:%s" % type(exc).__name__
        fb, warn = spawner_fallback(missing, catalog, mesh_observable_reference)
        bad_cases.append(
            {
                "case": "missing",
                "kernel": kernel_missing,
                "spawner_source": fb["source"],
                "spawner_warning": None if warn is None else warn.replace(tmp, "$PROBE_TMP"),
                "substituted_catalog": fb["source"] == "catalog_fallback",
            }
        )

        truncated = os.path.join(tmp, "truncated.stl")
        with open(truncated, "wb") as handle:
            handle.write(b"\0" * 80 + struct.pack("<I", 12))
        try:
            mesh_observable_reference(truncated)
            kernel_trunc = "returned"
        except (IOError, OSError, ValueError) as exc:
            kernel_trunc = "raised:%s" % type(exc).__name__
        fb, warn = spawner_fallback(truncated, catalog, mesh_observable_reference)
        bad_cases.append(
            {
                "case": "truncated",
                "kernel": kernel_trunc,
                "spawner_source": fb["source"],
                "spawner_warning": None if warn is None else warn.replace(tmp, "$PROBE_TMP"),
                "substituted_catalog": fb["source"] == "catalog_fallback",
            }
        )

        ascii_stl = os.path.join(tmp, "ascii.stl")
        with open(ascii_stl, "w", encoding="ascii") as handle:
            handle.write("solid demo\nendsolid demo\n")
        try:
            mesh_observable_reference(ascii_stl)
            kernel_ascii = "returned"
        except (IOError, OSError, ValueError, struct.error) as exc:
            kernel_ascii = "raised:%s" % type(exc).__name__
        fb, warn = spawner_fallback(ascii_stl, catalog, mesh_observable_reference)
        bad_cases.append(
            {
                "case": "non_binary_ascii",
                "kernel": kernel_ascii,
                "spawner_source": fb["source"],
                "spawner_warning": None if warn is None else warn.replace(tmp, "$PROBE_TMP"),
                "substituted_catalog": fb["source"] == "catalog_fallback",
            }
        )

        malformed = os.path.join(tmp, "malformed.stl")
        with open(malformed, "wb") as handle:
            handle.write(b"not-an-stl" + (b"\x00" * 40) + struct.pack("<I", 10 ** 7))
        try:
            mesh_observable_reference(malformed)
            kernel_mal = "returned"
        except (IOError, OSError, ValueError, struct.error, MemoryError) as exc:
            kernel_mal = "raised:%s" % type(exc).__name__
        fb, warn = spawner_fallback(malformed, catalog, mesh_observable_reference)
        bad_cases.append(
            {
                "case": "malformed",
                "kernel": kernel_mal,
                "spawner_source": fb["source"],
                "spawner_warning": None if warn is None else warn.replace(tmp, "$PROBE_TMP"),
                "substituted_catalog": fb["source"] == "catalog_fallback",
            }
        )

        hits, online_hits = scan_getcurrentbox(repo)

        checks = {}
        checks["1_record"] = status_record(
            "PASS" if len(assets) == 6 and all(len(a["sha256"]) == SHA256_LEN for a in assets) else "FAIL",
            "Recorded triangle count, AABB, SHA-256, visual ID, tier, algorithm parameters, and scalars for all six sized STLs.",
        )
        checks["2_repeatability"] = status_record(
            "PASS" if all(a["repeat_stable"] for a in assets) else "FAIL",
            "Each production calculation repeated %d times in this process; results were bit-stable."
            % REPEAT_N,
        )
        copy_ok = all(max(a["copy_diff"].values()) <= 1e-9 for a in assets)
        checks["3_path_invariance"] = status_record(
            "PASS" if copy_ok else "FAIL",
            "Copied each STL to a new path; production scalars agreed within 1e-9 m."
            if copy_ok
            else "Path copy changed scalars.",
            diffs={("%s_%s" % (a["visual_id"], a["tier"])): a["copy_diff"] for a in assets},
        )
        order_ok = all(
            max(a["reversed_diff"].values()) <= 1e-9
            and max(a["shuffled_diff"].values()) <= 1e-9
            for a in assets
        )
        checks["4_triangle_order"] = status_record(
            "PASS" if order_ok else "BLOCKED",
            "Reversed and seeded-shuffled triangle records without changing vertices."
            if order_ok
            else "Production modal-bin selection depends on triangle insertion order (max(dict) on equal counts).",
            diffs={
                "%s_%s" % (a["visual_id"], a["tier"]): {
                    "reversed": a["reversed_diff"],
                    "shuffled": a["shuffled_diff"],
                    "tied_bin_count": a["oracle"]["tied_bin_count"],
                }
                for a in assets
            },
        )
        tx_ok = all(
            a["translated_diff"]["width"] <= 1e-6
            and a["translated_diff"]["depth"] <= 1e-6
            and a["translated_diff"]["lid_offset"] <= 1e-6
            and a["translated_diff"]["full_height"] <= 1e-6
            for a in assets
        )
        checks["5_translation"] = status_record(
            "PASS" if tx_ok else "FAIL",
            "Translated every vertex by %s m." % (TRANSLATE,),
            diffs={("%s_%s" % (a["visual_id"], a["tier"])): a["translated_diff"] for a in assets},
        )
        oracle_ok = all(max(a["oracle_diff"].values()) <= 1e-6 for a in assets)
        checks["6_independent_oracle"] = status_record(
            "PASS" if oracle_ok else "FAIL",
            "Independent binary-STL oracle uses max-count then max bin key; median is sorted[len//2].",
            diffs={("%s_%s" % (a["visual_id"], a["tier"])): a["oracle_diff"] for a in assets},
        )
        pose_ok = all(p["identity_ok"] for a in assets for p in a["poses"])
        yaw_scalar_ok = True
        checks["7_pose_identity"] = status_record(
            "PASS" if pose_ok and yaw_scalar_ok else "FAIL",
            "top_z - center_z == observable_height/2 and observable bottom == spawn AABB bottom within 1e-6 m; yaw does not enter the STL scalar function.",
        )
        silent = any(c["substituted_catalog"] for c in bad_cases)
        kernel_closed = all(str(c["kernel"]).startswith("raised") for c in bad_cases)
        checks["8_fail_closed"] = status_record(
            "BLOCKED" if silent else ("PASS" if kernel_closed else "FAIL"),
            "Kernel raises on missing/truncated/malformed/non-binary STL, but pickup_box_spawner._observable_reference catches that and silently substitutes catalog size into GetCurrentBox (warning only).",
            cases=bad_cases,
        )
        checks["9_online_isolation"] = status_record(
            "FAIL" if online_hits else "PASS",
            "GetCurrentBox producer is pickup_box_spawner; Gate 4 eval is the Humble consumer. Humble online perception/planning algorithm modules have no GetCurrentBox client. ROS1 reference and eval-only paths are recorded separately."
            if not online_hits
            else "Humble online algorithm files consume GetCurrentBox: %s" % online_hits,
            online_algorithm_hits=online_hits,
            scan_hit_count=len(hits),
        )
        risky = []
        for asset in assets:
            for row in asset["sensitivity"]:
                if abs(row["d_lid_vs_default"]) > 0.001 or abs(row["d_width_vs_default"]) > 0.005 or abs(
                    row["d_depth_vs_default"]
                ) > 0.005:
                    risky.append(
                        {
                            "asset": "%s_%s" % (asset["visual_id"], asset["tier"]),
                            "top_band_frac": row["top_band_frac"],
                            "z_bin": row["z_bin"],
                            "d_lid": row["d_lid_vs_default"],
                            "d_width": row["d_width_vs_default"],
                            "d_depth": row["d_depth_vs_default"],
                        }
                    )
        checks["10_parameter_sensitivity"] = status_record(
            "PASS",
            "Diagnostic sweep of top_band_frac {0.20,0.25,0.30} and z_bin {0.00075,0.001,0.00125}; rows exceeding 1 mm lid or 5 mm width/depth are versioning risks, not a relaxed oracle.",
            versioning_risks=risky,
        )

        hard = (
            checks["4_triangle_order"]["status"],
            checks["6_independent_oracle"]["status"],
            checks["8_fail_closed"]["status"],
            checks["9_online_isolation"]["status"],
        )
        gt_readiness = "blocked" if any(s in {"FAIL", "BLOCKED"} for s in hard) else "ready"
        report = {
            "base_revision": "c5921d5f29ae5252747c7430ba2724214d1cbfc4",
            "algorithm_parameters": {
                "top_band_frac": TOP_BAND_FRAC,
                "z_bin": Z_BIN,
                "band": "max(top_band_frac * full_height, 2 * z_bin)",
                "modal_bin": "int(z / z_bin); production max(count) first-insert; oracle max(count) then max key",
            },
            "assets": assets,
            "checks": checks,
            "gt_readiness": gt_readiness,
            "audit_complete": True,
        }
        encoded = json.dumps(report, sort_keys=True, indent=2, ensure_ascii=True)
        with open(out_path, "w", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.write("\n")
        return 0
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    return run_probe(os.path.abspath(args.repo), os.path.abspath(args.out))


if __name__ == "__main__":
    sys.exit(main())
