"""sfree_o_rehearsal — O-side live rehearsal of the multi-view fixture
(adjudication 2026-11-08 item 3; reviews 2026-11-08_1751 round).

Two phases, each on its OWN observer process (no store mixing):

positive — the full 12-origin frozen fixture through the PRODUCT chain:
live observer (IntegrateObservation with each frame's own cloud+TF) ->
conservative bridge v2 -> generate_candidates WITH the same-snapshot
volume verifier (paged ReadCells over the full-container ROI, identity
bound, assembled by placement_solver.volume_evidence_from_pages). The
authorized sensor candidates are then re-verified by an INDEPENDENT
external scan of each candidate's COMPLETE box volume — every cell with
positive XYZ overlap, boundary layers included (a box at floor-relative
[0.20, 0.40) on a 2cm grid reads layers 10..19; the earlier version
trimmed to 11..18 and could hide unknown/occupied in the boundary
layers — reviews finding 1). ROI insufficiency, an empty range, or a
short/identity-mismatched page read fails the phase, never passes it.

negative — the single-origin (0.0, 1.20) subset: the surface ALONE
still offers stacking candidates (the counterfactual: without the
volume gate the product would authorize them), and the product gate
rejects exactly those whose volumes hold unobserved voxels. The
external scan agrees on every flip (reviews finding 2: the healthy
fixture passing never proved the product rejects unknown-above).

The native QueryFreespace enumeration is recorded as an AUXILIARY
observation only (±1-cell proximity): it does not prove the same
position/yaw/volume and cannot substitute for the binding check.

Usage: OCC_FS_NODE=<observer_node> python3 -m occ_compat.sfree_o_rehearsal
       --geometry <yaml> --config <yaml> --out <json>
"""
from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import sys
import time
from pathlib import Path

from . import gen_sfree

PROBE = (0.30, 0.20, 0.20)
RES = 0.02
#: The negative-control origin: one mid-height view. Offline probe on
#: the G projection (same frames): the surface offers 1125 stacking
#: candidates, 334 of whose volumes hold unobserved voxels — exactly
#: the "same support surface, unknown above" state reviews asked to
#: see the product reject.
NEGATIVE_ORIGINS = [(0.0, 1.20)]


def _quat(rotation9):
    """Row-major 3x3 -> (x, y, z, w)."""
    m = [[rotation9[0], rotation9[1], rotation9[2]],
         [rotation9[3], rotation9[4], rotation9[5]],
         [rotation9[6], rotation9[7], rotation9[8]]]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def _cloud_msg(frame):
    from sensor_msgs.msg import PointCloud2, PointField
    cloud = PointCloud2()
    cloud.header.frame_id = frame["optical_frame"]
    stamp = int(frame["stamp_ns"])
    cloud.header.stamp.sec = stamp // 1_000_000_000
    cloud.header.stamp.nanosec = stamp % 1_000_000_000
    cloud.height = 1
    cloud.width = len(frame["points_m"])
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * cloud.width
    for index, name in enumerate(("x", "y", "z")):
        field = PointField()
        field.name = name
        field.offset = index * 4
        field.datatype = PointField.FLOAT32
        field.count = 1
        cloud.fields.append(field)
    cloud.data = b"".join(
        struct.pack("<fff", *point) for point in frame["points_m"])
    return cloud


def _transform_msg(frame):
    from geometry_msgs.msg import TransformStamped
    transform = TransformStamped()
    stamp = int(frame["stamp_ns"])
    transform.header.stamp.sec = stamp // 1_000_000_000
    transform.header.stamp.nanosec = stamp % 1_000_000_000
    transform.header.frame_id = "container_link"
    transform.child_frame_id = frame["optical_frame"]
    transform.transform.translation.x = frame["transform"]["translation"][0]
    transform.transform.translation.y = frame["transform"]["translation"][1]
    transform.transform.translation.z = frame["transform"]["translation"][2]
    qx, qy, qz, qw = _quat(frame["transform"]["rotation"])
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


def _wait_service(client, name, deadline_s=20.0):
    deadline = time.monotonic() + deadline_s
    while not client.wait_for_service(timeout_sec=0.2):
        if time.monotonic() > deadline:
            raise RuntimeError("service never appeared: %s" % name)


def _call(node, client, request, timeout_s=30.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            raise RuntimeError("service call timeout")
        time.sleep(0.02)
    return future.result()


def _cells_checked(center, footprint, box_h, floor_z):
    """Reporting-only count of the complete positive-overlap cell range
    the frozen geometry rules assign to one candidate volume (the same
    formulas placement_solver.volume_state_in_evidence scans)."""
    z0 = center[2] - box_h * 0.5 - floor_z
    z1 = center[2] + box_h * 0.5 - floor_z
    iz_lo = math.floor(z0 / RES + 1e-9)
    iz_hi = math.floor(z1 / RES - 1e-9)
    layers = max(0, int(iz_hi - iz_lo + 1))
    span_x = max(0, int(math.floor(footprint[0] / RES + 0.5)))
    span_y = max(0, int(math.floor(footprint[1] / RES + 0.5)))
    return span_x * span_y * layers


def run_phase(node_binary, args, hull, geometry_path, origins, phase):
    """One observer process + one snapshot; returns the phase report."""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from uld_occ_observer.srv import (GetSnapshot, IntegrateObservation,
                                      QueryFreespace, ReadCells, ReadSurface,
                                      ReleaseSnapshot)
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                           / "src" / "uld_occ_observer" / "scripts"))
    import occ_surface_bridge as bridge  # noqa: E402
    worktree = Path(__file__).resolve().parents[2] / "src"
    sys.path.insert(0, str(worktree / "luggage_packing"))
    sys.path.insert(0, str(worktree / "luggage_description"))
    from luggage_packing import placement_solver as ps  # noqa: E402

    raw_frames = gen_sfree.generate_frames(hull, origins_yz=origins)
    # Re-base fixture stamps onto the CURRENT clock (the campaign's
    # integrate_scene does the same): freshness windows judge the node
    # clock, and the frozen epoch-relative stamps would be rejected as
    # ancient. Inter-frame deltas are preserved; the snapshot query then
    # WAITS until the wall clock passes the last frame stamp
    # (INVALID_QUERY_TIME otherwise).
    base = int(raw_frames[0]["stamp_ns"])
    origin_time = time.time_ns() + 120_000_000
    frames = []
    for frame in raw_frames:
        stamp = origin_time + (int(frame["stamp_ns"]) - base)
        rebased = dict(frame)
        rebased["stamp_ns"] = stamp
        transform = dict(frame["transform"])
        transform["stamp_ns"] = stamp
        rebased["transform"] = transform
        frames.append(rebased)

    log = open(args.out + ".%s.node.log" % phase, "w")
    install_lib = Path(node_binary).resolve().parent.parent.parent / "lib"
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "%s:%s" % (
        install_lib, env.get("LD_LIBRARY_PATH", ""))
    process = subprocess.Popen(
        [node_binary, "--ros-args", "-p", "geometry_file:=%s" % geometry_path,
         "-p", "config_file:=%s" % args.config, "-p", "input_mode:=service"],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    rclpy.init()
    node = Node("sfree_o_rehearsal_%s" % phase)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    import threading
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    report = {"phase": phase, "frames": len(frames), "checks": {}}
    try:
        integrate = node.create_client(
            IntegrateObservation, "/uld_occ_observer/integrate_observation")
        get_snapshot = node.create_client(
            GetSnapshot, "/uld_occ_observer/get_snapshot")
        read_surface = node.create_client(
            ReadSurface, "/uld_occ_observer/read_surface")
        read_cells = node.create_client(
            ReadCells, "/uld_occ_observer/read_cells")
        freespace = node.create_client(
            QueryFreespace, "/uld_occ_observer/query_freespace")
        release = node.create_client(
            ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
        for client in (integrate, get_snapshot, read_surface, read_cells,
                       freespace, release):
            _wait_service(client, client.srv_name)

        # 1. integrate the frames (each carries its own TF)
        accepted = accepted_points = 0
        for frame in frames:
            request = IntegrateObservation.Request()
            request.sensor_id = "sfree_cam"
            request.observation_id = frame["observation_id"]
            request.clock_domain = "ros"
            request.geometry_hash = str(hull["geometry_hash"])
            request.epoch = 1
            request.cloud = _cloud_msg(frame)
            request.uld_from_optical = _transform_msg(frame)
            receipt = _call(node, integrate, request)
            status = str(receipt.receipt.status)
            accepted_points += int(receipt.receipt.accepted_points)
            if status == "INTEGRATED":
                accepted += 1
            else:
                report["checks"]["integrate"] = {
                    "pass": False, "status": status,
                    "frame": frame["observation_id"]}
                report["pass"] = False
                return report
        report["checks"]["integrate"] = {
            "pass": accepted == len(frames) and accepted_points > 0,
            "accepted": accepted, "accepted_points": accepted_points}

        # 2. snapshot -> ReadSurface -> bridge v2 (wait for the wall
        # clock to pass the last frame stamp first)
        last_stamp = max(int(f["stamp_ns"]) for f in frames)
        while time.time_ns() < last_stamp + 50_000_000:
            time.sleep(0.05)
        lease = _call(node, get_snapshot, GetSnapshot.Request(
            max_age_ms=60000))
        assert str(lease.status) == "OK", lease.status
        lease_identity = {
            "geometry_hash": str(lease.identity.geometry_hash),
            "map_revision": int(lease.identity.map_revision),
            "epoch": int(lease.identity.epoch),
            "observed_revision": int(lease.identity.observed_revision),
        }
        columns = []
        cursor, done = 0, False
        while not done:
            response = _call(node, read_surface, ReadSurface.Request(
                handle=lease.handle, cursor=cursor, limit=65536))
            assert str(response.status) == "OK", response.status
            identity = {
                "geometry_hash": response.identity.geometry_hash,
                "map_revision": response.identity.map_revision,
                "epoch": response.identity.epoch,
                "observed_revision": response.identity.observed_revision,
                "last_integrated_stamp_ns":
                    response.identity.last_integrated_stamp_ns,
                "clock_domain": response.identity.clock_domain,
            }
            for i in range(len(response.x)):
                columns.append({
                    "x": response.x[i], "y": response.y[i],
                    "active_count": response.active_count[i],
                    "known_count": response.known_count[i],
                    "has_occupied": response.has_occupied[i],
                    "top_z": response.top_z[i]})
            cursor, done = response.next_cursor, response.done
        geometry = bridge.load_geometry(geometry_path)
        geometry["_resolution"] = RES
        surface = bridge.build_surface_map(columns, geometry, identity)
        assert "error" not in surface, surface.get("error")
        for field, expected in lease_identity.items():
            if str(surface.get(field)) != str(expected):
                raise RuntimeError(
                    "surface/lease identity mismatch on %s" % field)

        # 3. same-snapshot volume evidence: paged ReadCells over the
        # FULL container ROI (x-fastest), assembled fail-closed.
        nx, ny = int(surface["nx"]), int(surface["ny"])
        inner_h = float(surface["inner_size"][2])
        nz = int(math.ceil(inner_h / RES - 1e-9))
        pages = []
        cursor, done = 0, False
        while not done:
            page = _call(node, read_cells, ReadCells.Request(
                handle=lease.handle, begin=[0, 0, 0], end=[nx, ny, nz],
                cursor=cursor, limit=65536))
            assert str(page.status) == "OK", page.status
            pages.append({
                "status": str(page.status),
                "states": page.states,
                "identity": {
                    "geometry_hash": str(page.identity.geometry_hash),
                    "map_revision": int(page.identity.map_revision),
                    "epoch": int(page.identity.epoch),
                    "observed_revision":
                        int(page.identity.observed_revision),
                }})
            cursor, done = int(page.next_cursor), bool(page.done)
            if len(pages) > 64:
                raise RuntimeError("read_cells paging did not finish")
        floor = float(hull["floor_z"])
        evidence = ps.volume_evidence_from_pages(
            pages, resolution=RES, nx=nx, ny=ny, nz=nz,
            origin_xy=[-nx * RES * 0.5, -ny * RES * 0.5],
            floor_z=floor, identity=lease_identity)
        if "error" in evidence:
            report["checks"]["volume_evidence"] = {
                "pass": False, "error": evidence["error"]}
            report["pass"] = False
            return report
        report["checks"]["volume_evidence"] = {
            "pass": True, "pages": len(pages), "cells": nx * ny * nz,
            "identity": lease_identity}

        # 4. PRODUCT chain: plain (surface only — the counterfactual)
        # vs guarded (same-snapshot volume verifier).
        solver_surface = dict(surface)
        params = dict(ps._default_params())
        params["top_n"] = 8000
        # the enumerated set is ~11k windows; keep every rejected row so
        # the gate's volume_unknown rejects are all in the returned list
        params["keep_rejected"] = 50000
        plain = ps.generate_candidates(
            solver_surface, list(PROBE), allowed_yaws=[0.0, math.pi / 2.0],
            params=params)
        verifier = ps.make_volume_verifier(evidence)
        guarded = ps.generate_candidates(
            solver_surface, list(PROBE), allowed_yaws=[0.0, math.pi / 2.0],
            params=params, volume_verifier=verifier)

        def stacking(candidates):
            # stacking = resting on an observed support top (center
            # above the floor-placement level by a safe margin)
            return [c for c in candidates
                    if c.get("feasible") and c.get("support_source") == "sensor"
                    and c["center_container"][2] > floor + 0.15]

        plain_stack = stacking(plain)
        guarded_stack = stacking(guarded)

        # 5. INDEPENDENT external scan: complete XYZ ranges over the
        # same evidence, per candidate. Agreement must be exact: every
        # authorized candidate reads known-free; every plain candidate
        # the gate dropped reads unknown/occupied.
        def external(candidates):
            outcomes = []
            for c in candidates:
                center = c["center_container"]
                outcomes.append(ps.volume_state_in_evidence(
                    evidence, center, c["footprint"],
                    float(center[2]) - PROBE[2] * 0.5,
                    float(center[2]) + PROBE[2] * 0.5))
            return outcomes

        authorized_outcomes = external(guarded_stack)
        guarded_ids = {c["candidate_id"] for c in guarded_stack}
        dropped = [c for c in plain_stack
                   if c["candidate_id"] not in guarded_ids]
        dropped_outcomes = external(dropped)
        volume_unknown_rejects = [
            c for c in guarded
            if str(c.get("reason", "")).startswith(ps.REASON_VOLUME_UNKNOWN)]
        checked_voxels = sum(
            _cells_checked(c["center_container"], c["footprint"],
                           PROBE[2], floor)
            for c in guarded_stack)
        agree_authorized = all(o is None
                               for o in authorized_outcomes)
        agree_dropped = all(o is not None for o in dropped_outcomes)
        if phase == "positive":
            gate_pass = (bool(guarded_stack)
                         and len(guarded_stack) == len(plain_stack)
                         and agree_authorized)
        else:
            gate_pass = (bool(plain_stack)
                         and len(dropped) > 0
                         and bool(guarded_stack)
                         and agree_authorized and agree_dropped
                         and volume_unknown_rejects)
        report["checks"]["product_volume_gate"] = {
            "pass": bool(gate_pass),
            "plain_surface_offers": len(plain_stack),
            "authorized_with_verifier": len(guarded_stack),
            "rejected_by_gate": len(dropped),
            "volume_unknown_rejected": len(volume_unknown_rejects),
            "external_agreement_authorized": agree_authorized,
            "external_agreement_dropped": agree_dropped,
            "voxels_verified": checked_voxels,
            "sample_dropped": [
                {"center": [round(v, 4) for v in c["center_container"]],
                 "external": dropped_outcomes[i]}
                for i, c in enumerate(dropped[:3])],
        }

        # 6. AUXILIARY only: native freespace proximity (±1 cell / ±1
        # layer). Not a binding check — proximity does not prove the
        # same position/yaw/volume (reviews 2026-11-08_1751).
        page = _call(node, freespace, QueryFreespace.Request(
            handle=lease.handle, box_length=PROBE[0], box_width=PROBE[1],
            box_height=PROBE[2], yaw_mode=2, cursor=0, limit=65536))
        native = set()
        if str(page.status) == "OK":
            native = {(int(page.cell_x[i]), int(page.cell_y[i]),
                       int(page.cell_z[i])) for i in range(len(page.cell_x))}
        native_by_z = {}
        for cell in native:
            native_by_z.setdefault(cell[2], set()).add((cell[0], cell[1]))
        near = 0
        for c in guarded_stack:
            center = c["center_container"]
            key_z = int(round((center[2] - PROBE[2] * 0.5 - floor) / RES))
            if any(abs(key[0] - int(c["ix0"])) <= 1
                   and abs(key[1] - int(c["iy0"])) <= 1
                   for z_band in (key_z - 1, key_z, key_z + 1)
                   for key in native_by_z.get(z_band, ())):
                near += 1
        report["checks"]["native_freespace_auxiliary"] = {
            "pass": True, "binding": False,
            "role": "auxiliary cross-observation",
            "native_cells": len(native), "near_authorized": near}
        release.call_async(ReleaseSnapshot.Request(handle=lease.handle))
        report["pass"] = all(
            v["pass"] for v in report["checks"].values()
            if v.get("binding", True))
        return report
    finally:
        process.terminate()
        rclpy.shutdown()


def main():
    import argparse
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--skip-negative", action="store_true",
                        help="positive phase only (debug)")
    args = parser.parse_args()
    node_binary = os.environ.get("OCC_FS_NODE")
    if not node_binary:
        print("OCC_FS_NODE not set (exit 2)")
        return 2

    document = yaml.safe_load(open(args.geometry).read())
    hull = json.loads(document["descriptor_json"])
    phases = [("positive", list(gen_sfree.ORIGINS_YZ))]
    if not args.skip_negative:
        phases.append(("negative", list(NEGATIVE_ORIGINS)))
    reports = {}
    ok = True
    for phase, origins in phases:
        report = run_phase(
            node_binary, args, hull, args.geometry, origins, phase)
        reports[phase] = report
        ok = ok and bool(report.get("pass"))
        print("sfree_o_rehearsal[%s]: %s %s" % (
            phase, "PASS" if report.get("pass") else "FAIL",
            {k: v["pass"] for k, v in report["checks"].items()}))
    reports["pass"] = ok
    Path(args.out).write_text(json.dumps(reports, indent=1))
    print("sfree_o_rehearsal: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
