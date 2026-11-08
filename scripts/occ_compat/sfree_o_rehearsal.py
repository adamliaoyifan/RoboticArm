"""sfree_o_rehearsal — O-side live rehearsal of the multi-view fixture
(adjudication 2026-11-08 item 3, reviews follow-up).

The G-side proof is offline; the O positive needs a REAL rehearsal:
integrate the 12 frozen fixture frames through the live observer
(IntegrateObservation with each frame's own cloud+transform), pull the
snapshot surface through the conservative bridge (v2), run the (footprint-
fixed) solver, then bind EVERY sensor-supported candidate to the SAME
snapshot's voxel evidence via ReadCells (0 unknown / 2 occupied inside
the candidate's own box volume rejects), and cross-check the candidate
positions against the NATIVE QueryFreespace enumerator.

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


def main():
    import argparse
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    node_binary = os.environ.get("OCC_FS_NODE")
    if not node_binary:
        print("OCC_FS_NODE not set (exit 2)")
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import PointCloud2  # noqa: F401 (import check)
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

    document = yaml.safe_load(open(args.geometry).read())
    hull = json.loads(document["descriptor_json"])
    raw_frames = gen_sfree.generate_frames(hull)
    # Re-base fixture stamps onto the CURRENT clock (the campaign's
    # integrate_scene does the same: freshness windows judge the node
    # clock, and the frozen epoch-relative stamps would be rejected as
    # ancient — the first rehearsal attempt integrated 0 points and the
    # snapshot read NO_DATA). Inter-frame deltas are preserved.
    base = int(raw_frames[0]["stamp_ns"])
    # Near-future start (the freshness window rejects stamps seconds
    # in the past); the snapshot query then WAITS until the wall clock
    # passes the last frame stamp (INVALID_QUERY_TIME otherwise).
    origin = time.time_ns() + 120_000_000
    frames = []
    for frame in raw_frames:
        rebased = dict(frame)
        stamp = origin + (int(frame["stamp_ns"]) - base)
        rebased["stamp_ns"] = stamp
        transform = dict(frame["transform"])
        transform["stamp_ns"] = stamp
        rebased["transform"] = transform
        frames.append(rebased)

    log = open(args.out + ".node.log", "w")
    install_lib = Path(node_binary).resolve().parent.parent.parent / "lib"
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "%s:%s" % (
        install_lib, env.get("LD_LIBRARY_PATH", ""))
    process = subprocess.Popen(
        [node_binary, "--ros-args", "-p", "geometry_file:=%s" % args.geometry,
         "-p", "config_file:=%s" % args.config, "-p", "input_mode:=service"],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    rclpy.init()
    node = Node("sfree_o_rehearsal")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    import threading
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    report = {"node_pid": process.pid, "frames": len(frames), "checks": {}}
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

        # 1. integrate the 12 frozen frames (each carries its own TF)
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
                print("sfree_o_rehearsal: integrate %s -> %s" % (
                    frame["observation_id"], status))
                return 1
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
        geometry = bridge.load_geometry(args.geometry)
        geometry["_resolution"] = RES
        surface = bridge.build_surface_map(columns, geometry, identity)
        assert "error" not in surface, surface.get("error")
        fully_known = sum(
            1 for c in columns
            if c["known_count"] == c["active_count"] > 0
            and not c["has_occupied"])
        partial = sum(
            1 for c in columns
            if 0 < c["known_count"] < c["active_count"])
        # informational: floor-level whole-column free is NOT the
        # positive (same as the G proof — the positive is the observed
        # support top; partial columns are expected under the fixture's
        # single-side visibility)
        report["checks"]["bridge_v2_columns"] = {
            "pass": True, "binding": False,
            "fully_known_free_columns": fully_known,
            "partial_known_columns": partial,
            "bridge_version": surface.get("bridge_version")}

        # 3. solver on the bridged surface
        floor = float(hull["floor_z"])
        solver_surface = dict(surface)
        solver_surface["geometry_descriptor"] = dict(hull)
        params = dict(ps._default_params())
        params["top_n"] = 8000
        result = ps.generate_candidates(
            solver_surface, list(PROBE),
            allowed_yaws=[0.0, math.pi / 2.0], params=params)
        cands = result if isinstance(result, list) else result["candidates"]
        sensor = [c for c in cands
                  if c.get("feasible") and c.get("support_source") == "sensor"]

        # 4. native freespace enumeration on the same snapshot
        page = _call(node, freespace, QueryFreespace.Request(
            handle=lease.handle, box_length=PROBE[0], box_width=PROBE[1],
            box_height=PROBE[2], yaw_mode=2, cursor=0, limit=65536))
        assert str(page.status) == "OK", page.status
        native = {(int(page.cell_x[i]), int(page.cell_y[i]),
                   int(page.cell_z[i])) for i in range(len(page.cell_x))}

        # 5. per-candidate voxel binding: ONE ReadCells ROI covering
        # every candidate volume on this snapshot (per-candidate calls
        # would outlive the lease), then local per-candidate lookup.
        # states are x-fastest: idx = x + dx * (y + dy * z).
        region_lo = [15, 14, 11]      # covers the support top + margin
        region_hi = [70, 55, 19]      # exclusive; z strictly inside the
        #                               box band [0.20, 0.40] floor-rel
        roi = _call(node, read_cells, ReadCells.Request(
            handle=lease.handle, begin=region_lo, end=region_hi,
            cursor=0, limit=65536))
        assert str(roi.status) == "OK", roi.status
        dx = region_hi[0] - region_lo[0]
        dy = region_hi[1] - region_lo[1]
        states = roi.states
        native_by_z = {}
        for cell in native:
            native_by_z.setdefault(cell[2], set()).add((cell[0], cell[1]))
        voxel_ok = native_ok = 0
        failures = []
        nx = int(surface["nx"])
        ny = int(surface["ny"])
        for c in sensor:
            center = [float(v) for v in c["center_container"]]
            fw, fd = float(c["footprint"][0]), float(c["footprint"][1])
            ix_lo = max(0, int(math.floor(
                (center[0] - fw / 2 + nx * RES / 2) / RES)))
            ix_hi = min(nx - 1, int(math.floor(
                (center[0] + fw / 2 + nx * RES / 2 - 1e-9) / RES)))
            iy_lo = max(0, int(math.floor(
                (center[1] - fd / 2 + ny * RES / 2) / RES)))
            iy_hi = min(ny - 1, int(math.floor(
                (center[1] + fd / 2 + ny * RES / 2 - 1e-9) / RES)))
            z0 = center[2] - PROBE[2] / 2.0
            iz_lo = max(region_lo[2], int(math.ceil((z0 - floor) / RES)) + 1)
            iz_hi = min(region_hi[2] - 1,
                        int(math.floor((z0 - floor + PROBE[2]) / RES)) - 1)
            bad = []
            if iz_lo <= iz_hi:
                for iz in range(iz_lo, iz_hi + 1):
                    for iy in range(iy_lo, iy_hi + 1):
                        for ix in range(ix_lo, ix_hi + 1):
                            if not (region_lo[0] <= ix < region_hi[0]
                                    and region_lo[1] <= iy < region_hi[1]):
                                bad.append("outside-roi")
                                break
                            state = int(states[
                                (ix - region_lo[0])
                                + dx * ((iy - region_lo[1])
                                        + dy * (iz - region_lo[2]))])
                            if state != 1:
                                bad.append(state)
                        if bad:
                            break
                    if bad:
                        break
            if not bad:
                voxel_ok += 1
            key_z = int(round((z0 - floor) / RES))
            near = any(
                abs(key[0] - int(c["ix0"])) <= 1
                and abs(key[1] - int(c["iy0"])) <= 1
                for z_band in (key_z - 1, key_z, key_z + 1)
                for key in native_by_z.get(z_band, ()))
            if near:
                native_ok += 1
            if (bad or not near) and len(failures) < 3:
                failures.append({"center": [round(v, 4) for v in center],
                                 "bad_states": sorted(set(
                                     str(b) for b in bad))[:4],
                                 "native_near": near})
        release.call_async(ReleaseSnapshot.Request(handle=lease.handle))
        report["checks"]["O_positive"] = {
            "pass": (len(sensor) > 0 and voxel_ok == len(sensor)
                     and native_ok == len(sensor)),
            "sensor_supported_feasible": len(sensor),
            "voxel_volume_verified": voxel_ok,
            "native_freespace_near": native_ok,
            "native_freespace_cells": len(native),
            "failures": failures}
        report["pass"] = all(v["pass"] for v in report["checks"].values()
                         if v.get("binding", True))
        Path(args.out).write_text(json.dumps(report, indent=1))
        print("sfree_o_rehearsal: %s %s" % (
            "PASS" if report["pass"] else "FAIL",
            {k: v["pass"] for k, v in report["checks"].items()}))
        return 0 if report["pass"] else 1
    finally:
        process.terminate()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
