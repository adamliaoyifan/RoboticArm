#!/usr/bin/env python3
"""FK-certify every seed stored in a reachability atlas artifact (ST06/F14).

ROS-free and deterministic. For each stored seed the advertised container
target is reconstructed independently of the solver (cell centre, directed
bin yaw, scene-config container transform) and the seed is forwarded through
the offline FK chain; the pose must agree within the ST06 limits:

  position    <= 0.002 m
  orientation <= 0.02 rad (quaternion sign-equivalent)

Contact seeds and vertical-branch transit seeds carry exact cell-local
targets (the per-branch ``transit_lift`` array records the lift the transit
seed was solved at). Internal-lift transit seeds ARE the predecessor carry:
their advertised contract is the tool-down orientation at the bin yaw plus a
position inside the container hull, certified as such and counted
separately. The URDF is recomputed from the working tree and its md5 must
equal the artifact's ``urdf_hash`` before any seed is trusted.

Also enforces the directed-bin coverage rule: every yaw bin must carry at
least one certified reachable seed, else the profile is ``not_evaluated``
and the gate fails. Exits 0 only when every row passes and coverage holds.
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
from luggage_planning.offline_ik import JOINTS, load_urdf  # noqa: E402

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


def _base_to_container(xyz, container_xyz, container_R):
    local = (
        xyz[0] - container_xyz[0],
        xyz[1] - container_xyz[1],
        xyz[2] - container_xyz[2],
    )
    return (
        container_R[0][0] * local[0] + container_R[1][0] * local[1]
        + container_R[2][0] * local[2],
        container_R[0][1] * local[0] + container_R[1][1] * local[1]
        + container_R[2][1] * local[2],
        container_R[0][2] * local[0] + container_R[1][2] * local[1]
        + container_R[2][2] * local[2],
    )


def certify(npz_path, meta_path, scene_tf=None, urdf_file=None):
    urdf = load_urdf(scene_tf_config=scene_tf)
    full_path = urdf.path_to(IK_LINK)
    if ARM_FIRST_JOINT not in full_path:
        raise RuntimeError(
            "URDF path to %s lacks %s" % (IK_LINK, ARM_FIRST_JOINT))
    tcp_path = full_path[full_path.index(ARM_FIRST_JOINT):]

    meta = yaml.safe_load(open(meta_path, "r", encoding="utf-8")) or {}
    contract = meta.get("contract") or {}
    if not isinstance(contract, dict) or not contract:
        return {
            "artifact": npz_path, "verdict": "not_evaluated",
            "reason": "no v2 contract block; v1 artifacts are not "
                      "certifiable (ST06 refuses to relabel them)",
        }
    stored_urdf_hash = str(contract.get("urdf_hash") or "")
    if not stored_urdf_hash:
        return {"artifact": npz_path, "verdict": "not_evaluated",
                "reason": "contract urdf_hash missing"}

    # The URDF identity is the build-time robot_description BYTES (what the
    # builder hashed from the live stack). The campaign captures that exact
    # string next to the artifacts; without it the gate cannot claim the
    # model identity and stays not_evaluated.
    if not urdf_file or not os.path.isfile(urdf_file):
        return {"artifact": npz_path, "verdict": "not_evaluated",
                "reason": "build-time robot_description capture not supplied",
                "urdf_hash_ok": None}
    with open(urdf_file, "rb") as handle:
        live_hash = hashlib.md5(handle.read()).hexdigest()
    if live_hash != stored_urdf_hash:
        return {
            "artifact": npz_path, "verdict": "fail",
            "reason": "urdf identity mismatch (captured=%s stored=%s)"
                      % (live_hash[:8], stored_urdf_hash[:8]),
            "urdf_hash_ok": False,
        }

    grid = meta.get("grid") or {}
    container = meta.get("container") or {}
    resolution = float(grid["resolution_xyz"])
    origin = [float(v) for v in grid["origin"]]
    size = [int(v) for v in grid["size"]]
    yaw_bins = [float(v) for v in grid["yaw_bins"]]
    container_xyz = [float(v) for v in container["base_xyz"]]
    container_rpy = [float(v) for v in container["base_rpy"]]
    inner = [float(v) for v in container["inner_dimensions"]]

    from luggage_planning.atlas_frame_contracts import _rpy_to_matrix
    container_R = _rpy_to_matrix(container_rpy)

    data = np.load(npz_path, allow_pickle=False)
    status = np.asarray(data["status"])
    contact = np.asarray(data["contact_seeds"], dtype=np.float64)
    transit = np.asarray(data["transit_seeds"], dtype=np.float64)
    transit_lift = np.asarray(
        data["transit_lift"], dtype=np.float32) if "transit_lift" in data \
        else np.full(transit.shape[:-1], np.nan, dtype=np.float32)

    nx, ny, nz, nyaw = status.shape
    rows = []
    breaches = []
    per_bin_certified = [0] * nyaw
    counts = {
        "contact_rows": 0, "transit_vertical_rows": 0,
        "transit_internal_rows": 0, "nan_rows_skipped": 0,
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

        def certify_row(kind, q, target_z, exact_pose):
            pos, quat = tool_target_in_base(
                x, y, target_z, bin_yaw, container_xyz, container_rpy)
            tf = urdf.tcp(q, tcp_path)
            fk_pos = tf[:3, 3]
            fk_quat = _quat_from_matrix(tf[:3, :3])
            pos_err = float(np.linalg.norm(fk_pos - np.array(pos)))
            rot_err = _rot_error_rad(fk_quat, np.array(quat))
            verdict = "pass"
            breach = None
            if exact_pose:
                if pos_err > POS_TOL_M or rot_err > ROT_TOL_RAD:
                    verdict = "fail"
                    breach = {
                        "kind": kind, **row_common,
                        "target_pos_base": [round(float(v), 9) for v in pos],
                        "target_quat_base": [round(float(v), 12) for v in quat],
                        "fk_pos_base": [round(float(v), 9) for v in fk_pos],
                        "fk_quat_base": [round(float(v), 12)
                                         for v in fk_quat],
                        "pos_err_m": round(pos_err, 9),
                        "rot_err_rad": round(rot_err, 9),
                        "limits": [POS_TOL_M, ROT_TOL_RAD],
                    }
            else:
                # Internal transit: orientation contract + hull containment.
                cx, cy, cz = _base_to_container(
                    fk_pos, container_xyz, container_R)
                inside = (
                    abs(cx) <= inner[0] * 0.5 + 1e-6
                    and abs(cy) <= inner[1] * 0.5 + 1e-6
                    and -0.05 <= cz <= inner[2] + 0.05)
                if rot_err > ROT_TOL_RAD or not inside:
                    verdict = "fail"
                    breach = {
                        "kind": kind, **row_common,
                        "fk_pos_base": [round(float(v), 9) for v in fk_pos],
                        "fk_container": [
                            round(float(cx), 9), round(float(cy), 9),
                            round(float(cz), 9)],
                        "pos_err_m": round(pos_err, 9),
                        "rot_err_rad": round(rot_err, 9),
                        "inside_hull": bool(inside),
                        "limits": ["hull", ROT_TOL_RAD],
                    }
            return {
                "kind": kind, **row_common,
                "pos_err_m": round(pos_err, 9),
                "rot_err_rad": round(rot_err, 9),
                "verdict": verdict,
            }, breach, pos_err, rot_err

        c_q = contact[ix, iy, iz, iyaw, branch]
        if np.isfinite(c_q).all() and np.abs(c_q).sum() > 0:
            row, breach, pe, re_ = certify_row("contact", c_q, z, True)
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
            lift = float(transit_lift[ix, iy, iz, iyaw, branch])
            if math.isfinite(lift):
                row, breach, pe, re_ = certify_row(
                    "transit_vertical", t_q, z + lift, True)
                counts["transit_vertical_rows"] += 1
            else:
                row, breach, pe, re_ = certify_row(
                    "transit_internal", t_q, z, False)
                counts["transit_internal_rows"] += 1
            rows.append(row)
            max_pos_err = max(max_pos_err, pe)
            max_rot_err = max(max_rot_err, re_)
            if breach:
                breaches.append(breach)

    empty_bins = [i for i, n in enumerate(per_bin_certified) if n == 0]
    verdict = "pass"
    reasons = []
    if empty_bins:
        reasons.append(
            "directed bins without a certified reachable seed: %s"
            % empty_bins)
    if breaches:
        # A breach must never be masked by the coverage verdict.
        verdict = "fail"
        reasons.append("%d seed breaches" % len(breaches))
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
                        help="scene_tf yaml for offline_urdf loading")
    parser.add_argument("--urdf-file", default=None,
                        help="build-time robot_description capture whose md5 "
                             "must equal the artifact contract urdf_hash")
    parser.add_argument("--output", default="",
                        help="output directory for certification JSON/JSONL")
    args = parser.parse_args()

    npz = args.atlas
    if not npz.endswith(".npz"):
        npz = npz + ".npz"
    meta = os.path.splitext(npz)[0] + ".yaml"
    result = certify(npz, meta, scene_tf=args.scene_tf, urdf_file=args.urdf_file)

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
