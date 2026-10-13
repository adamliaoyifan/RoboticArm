#!/usr/bin/env python3
"""FK-certify every seed stored in a reachability atlas artifact (ST06/F14).

ROS-free and deterministic. The certified kinematic model IS the
build-time robot_description capture: its bytes are md5-checked against
the artifact contract ``urdf_hash`` AND parsed for the FK chain, so a
capture that does not contain the arm chain cannot certify anything.

Every stored seed carries an exact advertised target, reconstructed
independently of the solver (cell centre or per-branch
``transit_target``, directed bin yaw, scene-config container transform)
and must agree with FK within the ST06 limits:

  position    <= 0.002 m
  orientation <= 0.02 rad (quaternion sign-equivalent)

A v3 artifact without per-branch ``transit_lift``/``transit_target``
arrays fails structurally -- certification never silently downgrades to a
weaker check. Every yaw bin must carry at least one certified reachable
seed, else the profile is ``not_evaluated``. Exits 0 only when every row
passes and coverage holds; a breach is never masked by the coverage
verdict.
"""

from __future__ import division

import argparse
import hashlib
import json
import math
import os
import sys

# Pin BLAS threads before numpy/scipy import (small-matrix matmul must not
# fan out over every core on this host).
for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
             "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import numpy as np  # noqa: E402
import yaml  # noqa: E402

_SRC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "src"))
for _pkg in ("luggage_planning", "luggage_description"):
    _path = os.path.join(_SRC, _pkg)
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)

from luggage_planning.atlas_frame_contracts import (  # noqa: E402
    tool_target_in_base,
)
from luggage_planning.offline_ik import JOINTS, Urdf  # noqa: E402

POS_TOL_M = 0.002
ROT_TOL_RAD = 0.02
IK_LINK = "suction_contact_frame"
ARM_FIRST_JOINT = JOINTS[0]


def _quat_from_matrix(R):
    from scipy.spatial.transform import Rotation as Rot
    return Rot.from_matrix(R).as_quat()  # x, y, z, w


def _rot_error_rad(qa, qb):
    """Sign-equivalent angular distance between two quaternions."""
    dot = abs(float(np.dot(qa, qb)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def certify(npz_path, meta_path, scene_tf=None, urdf_file=None):
    meta = yaml.safe_load(open(meta_path, "r", encoding="utf-8")) or {}
    contract = meta.get("contract") or {}
    if not isinstance(contract, dict) or not contract:
        return {
            "artifact": npz_path, "verdict": "not_evaluated",
            "reasons": ["no v2 contract block; v1 artifacts are not "
                        "certifiable (ST06 refuses to relabel them)"],
        }
    stored_urdf_hash = str(contract.get("urdf_hash") or "")
    if not stored_urdf_hash:
        return {"artifact": npz_path, "verdict": "not_evaluated",
                "reasons": ["contract urdf_hash missing"]}

    # The identity capture is the FK model: md5 the bytes, then parse THOSE
    # bytes for the chain. A capture without the arm chain fails closed.
    if not urdf_file or not os.path.isfile(urdf_file):
        return {"artifact": npz_path, "verdict": "not_evaluated",
                "reasons": ["build-time robot_description capture not "
                            "supplied"],
                "urdf_hash_ok": None}
    with open(urdf_file, "rb") as handle:
        capture_bytes = handle.read()
    live_hash = hashlib.md5(capture_bytes).hexdigest()
    if live_hash != stored_urdf_hash:
        return {
            "artifact": npz_path, "verdict": "fail",
            "reasons": ["urdf identity mismatch (captured=%s stored=%s)"
                        % (live_hash[:8], stored_urdf_hash[:8])],
            "urdf_hash_ok": False,
        }
    try:
        urdf = Urdf(capture_bytes.decode("utf-8"))
        full_path = urdf.path_to(IK_LINK)
    except Exception as exc:  # noqa: BLE001 - unusable capture fails closed
        return {
            "artifact": npz_path, "verdict": "fail",
            "reasons": ["capture is not a usable kinematic model (%s)"
                        % exc],
            "urdf_hash_ok": True,
        }
    if ARM_FIRST_JOINT not in full_path:
        return {
            "artifact": npz_path, "verdict": "fail",
            "reasons": ["capture model lacks the arm chain (%s not under %s)"
                        % (ARM_FIRST_JOINT, IK_LINK)],
            "urdf_hash_ok": True,
        }
    tcp_path = full_path[full_path.index(ARM_FIRST_JOINT):]

    grid = meta.get("grid") or {}
    container = meta.get("container") or {}
    resolution = float(grid["resolution_xyz"])
    origin = [float(v) for v in grid["origin"]]
    yaw_bins = [float(v) for v in grid["yaw_bins"]]
    container_xyz = [float(v) for v in container["base_xyz"]]
    container_rpy = [float(v) for v in container["base_rpy"]]

    data = np.load(npz_path, allow_pickle=False)
    # Per-branch transit provenance is structural for a v3 artifact: a
    # missing array must refuse certification, never weaken it.
    missing = [key for key in ("transit_lift", "transit_target")
               if key not in getattr(data, "files", [])]
    if missing:
        return {
            "artifact": npz_path, "verdict": "fail",
            "urdf_hash_ok": True,
            "reasons": ["v3 artifact lacks %s; refusing to certify without "
                        "per-branch transit provenance" % ", ".join(missing)],
        }
    status = np.asarray(data["status"])
    contact = np.asarray(data["contact_seeds"], dtype=np.float64)
    transit = np.asarray(data["transit_seeds"], dtype=np.float64)
    transit_lift = np.asarray(data["transit_lift"], dtype=np.float32)
    transit_target = np.asarray(data["transit_target"], dtype=np.float32)
    if transit_target.shape != transit.shape[:-1] + (3,):
        return {"artifact": npz_path, "verdict": "fail",
                "urdf_hash_ok": True,
                "reasons": ["transit_target shape %r does not match the "
                            "seed grid %r" % (
                                transit_target.shape,
                                transit.shape[:-1] + (3,))]}

    nx, ny, nz, nyaw = status.shape
    rows = []
    breaches = []
    per_bin_certified = [0] * nyaw
    counts = {
        "contact_rows": 0, "transit_vertical_rows": 0,
        "transit_predecessor_rows": 0,
    }
    max_pos_err = 0.0
    max_rot_err = 0.0

    seeded = np.argwhere(
        np.logical_and(
            np.isfinite(contact).any(axis=-1),
            np.abs(contact).sum(axis=-1) > 0))
    seeded_t = np.argwhere(
        np.logical_and(
            np.isfinite(transit).any(axis=-1),
            np.abs(transit).sum(axis=-1) > 0))
    keys = {tuple(k) for k in seeded} | {tuple(k) for k in seeded_t}
    for ix, iy, iz, iyaw, branch in sorted(keys):
        x = origin[0] + (ix + 0.5) * resolution
        y = origin[1] + (iy + 0.5) * resolution
        z = origin[2] + (iz + 0.5) * resolution
        bin_yaw = yaw_bins[iyaw]
        row_common = {
            "cell": [int(ix), int(iy), int(iz), int(iyaw)],
            "bin_yaw_rad": round(bin_yaw, 9),
            "branch": int(branch),
            "status": int(status[ix, iy, iz, iyaw]),
        }

        def certify_row(kind, q, target_container):
            """Exact position+orientation gate for one stored seed."""
            pos, quat = tool_target_in_base(
                target_container[0], target_container[1],
                target_container[2], bin_yaw, container_xyz, container_rpy)
            tf = urdf.tcp(q, tcp_path)
            fk_pos = tf[:3, 3]
            fk_quat = _quat_from_matrix(tf[:3, :3])
            pos_err = float(np.linalg.norm(fk_pos - np.array(pos)))
            rot_err = _rot_error_rad(fk_quat, np.array(quat))
            row = {
                "kind": kind, **row_common,
                "target_container": [
                    round(float(v), 9) for v in target_container],
                "pos_err_m": round(pos_err, 9),
                "rot_err_rad": round(rot_err, 9),
                "verdict": "pass" if (
                    pos_err <= POS_TOL_M and rot_err <= ROT_TOL_RAD)
                else "fail",
            }
            breach = None
            if row["verdict"] == "fail":
                breach = {
                    **row,
                    "target_pos_base": [round(float(v), 9) for v in pos],
                    "target_quat_base": [round(float(v), 12) for v in quat],
                    "fk_pos_base": [round(float(v), 9) for v in fk_pos],
                    "fk_quat_base": [round(float(v), 12) for v in fk_quat],
                    "limits": [POS_TOL_M, ROT_TOL_RAD],
                }
            return row, breach, pos_err, rot_err

        c_q = contact[ix, iy, iz, iyaw, branch]
        if np.isfinite(c_q).all() and np.abs(c_q).sum() > 0:
            row, breach, pe, re_ = certify_row("contact", c_q, (x, y, z))
            rows.append(row)
            counts["contact_rows"] += 1
            max_pos_err = max(max_pos_err, pe)
            max_rot_err = max(max_rot_err, re_)
            if breach:
                breaches.append(breach)
            elif row["status"] >= 2:
                per_bin_certified[iyaw] += 1

        t_q = transit[ix, iy, iz, iyaw, branch]
        if np.isfinite(t_q).all() and np.abs(t_q).sum() > 0:
            target = transit_target[ix, iy, iz, iyaw, branch]
            lift = float(transit_lift[ix, iy, iz, iyaw, branch])
            if not np.isfinite(target).all():
                # A seeded row without recorded provenance is a structural
                # breach: certification never guesses the target.
                rows.append({
                    "kind": "transit_target_missing", **row_common,
                    "verdict": "fail",
                })
                breaches.append({
                    "kind": "transit_target_missing", **row_common,
                    "limits": ["provenance"],
                })
                counts["transit_predecessor_rows"] += 1
                continue
            kind = ("transit_vertical" if math.isfinite(lift)
                    else "transit_predecessor")
            row, breach, pe, re_ = certify_row(kind, t_q, tuple(target))
            rows.append(row)
            counts[kind + "_rows"] += 1
            max_pos_err = max(max_pos_err, pe)
            max_rot_err = max(max_rot_err, re_)
            if breach:
                breaches.append(breach)

    empty_bins = [i for i, n in enumerate(per_bin_certified) if n == 0]
    reasons = []
    if empty_bins:
        reasons.append(
            "directed bins without a certified reachable seed: %s"
            % empty_bins)
    if breaches:
        # A breach must never be masked by the coverage verdict.
        reasons.append("%d seed breaches" % len(breaches))
    verdict = "pass"
    if breaches:
        verdict = "fail"
    elif empty_bins:
        verdict = "not_evaluated"
    return {
        "artifact": npz_path,
        "urdf_hash_ok": True,
        "verdict": verdict,
        "reasons": reasons,
        "yaw_bins_rad": [round(b, 9) for b in yaw_bins],
        "per_bin_certified_seeds": per_bin_certified,
        "counts": counts,
        "max_pos_err_m": round(max_pos_err, 9),
        "max_rot_err_rad": round(max_rot_err, 9),
        "rows": rows,
        "breaches": breaches,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--atlas", required=True,
                        help="npz path (yaml sibling) or prefix without ext")
    parser.add_argument("--scene-tf", default=None,
                        help=argparse.SUPPRESS)
    parser.add_argument("--urdf-file", default=None,
                        help="build-time robot_description capture whose md5 "
                             "must equal the artifact contract urdf_hash; "
                             "its parsed model is the FK model")
    parser.add_argument("--output", default="",
                        help="output directory for certification JSON/JSONL")
    args = parser.parse_args()

    npz = args.atlas
    if not npz.endswith(".npz"):
        npz = npz + ".npz"
    meta = os.path.splitext(npz)[0] + ".yaml"
    result = certify(npz, meta, scene_tf=args.scene_tf,
                     urdf_file=args.urdf_file)

    if args.output:
        os.makedirs(args.output, exist_ok=True)
        rows = result.pop("rows", [])
        breaches = result.pop("breaches", [])
        with open(os.path.join(args.output, "seeds.jsonl"), "w") as handle:
            for row in rows:
                handle.write(json.dumps(row) + "\n")
        result["n_rows"] = len(rows)
        result["n_breaches"] = len(breaches)
        with open(os.path.join(args.output, "breaches.json"), "w") as handle:
            json.dump(breaches, handle, indent=1)
        with open(os.path.join(args.output, "summary.json"), "w") as handle:
            json.dump(result, handle, indent=1)
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ("rows", "breaches")}, indent=1))
    return 0 if result.get("verdict") == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
