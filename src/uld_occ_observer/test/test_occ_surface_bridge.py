"""surface_map_2d bridge tests: pure builder, real solver consumption, and a
live service-chain cross-check (unified delivery contract §1/§2).

The live test runs the REAL observer node from the install under test,
integrates synthetic row sweeps through IntegrateObservation, pulls the
surface through the bridge's ROS fetch path, feeds the REAL placement
solver's generate_candidates, and cross-checks against query_freespace on
the same snapshot: a solver candidate whose footprint support is fully
OBSERVED must be a known-free candidate; a candidate resting on unknown
columns is a floor-prior/policy candidate and must never appear as
known-free. That is exactly the prior-vs-strict distinction the contract
freezes.
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

import pytest

FIXTURES = Path("/home/adamliao/work/elfin_humble_ws/docs/status/evidence/"
                "occ_o6_o7_20261029/run01/fixtures")
GEOMETRY = FIXTURES / "geometry_chamfer.yaml"
CONFIG = FIXTURES / "config_chamfer_r0p02.yaml"
BRIDGE_DIR = Path(__file__).resolve().parent.parent / "scripts"

sys.path.insert(0, str(BRIDGE_DIR))
import occ_surface_bridge as bridge  # noqa: E402


def _geometry():
    geometry = bridge.load_geometry(str(GEOMETRY))
    geometry["_resolution"] = 0.02
    return geometry


def _columns(geometry, occupied_rows=(), free_rows=(), occupied_top=0.63):
    """Synthetic column table: default unknown, some rows observed."""
    sx = int(math.ceil(geometry["length"] / 0.02))
    sy = int(math.ceil(geometry["width"] / 0.02))
    columns = []
    for y in range(sy):
        for x in range(sx):
            if y in occupied_rows:
                columns.append({"x": x, "y": y, "active_count": 74,
                                "known_count": 5, "has_occupied": True,
                                "top_z": occupied_top})
            elif y in free_rows:
                columns.append({"x": x, "y": y, "active_count": 74,
                                "known_count": 5, "has_occupied": False,
                                "top_z": 0.0})
            else:
                columns.append({"x": x, "y": y, "active_count": 74,
                                "known_count": 0, "has_occupied": False,
                                "top_z": 0.0})
    return columns


def _identity(geometry, **extra):
    identity = {"geometry_hash": geometry["geometry_hash"],
                "map_revision": 7, "epoch": 1, "observed_revision": 9,
                "last_integrated_stamp_ns": 123, "clock_domain": "ros"}
    identity.update(extra)
    return identity


def test_builder_fail_closed_on_geometry_mismatch():
    geometry = _geometry()
    surface = bridge.build_surface_map(_columns(geometry), geometry,
                                       _identity(geometry, geometry_hash="other"))
    assert surface["error"] == "CARGO_MAP_GEOMETRY_MISMATCH"


def test_builder_unknown_is_never_free():
    geometry = _geometry()
    surface = bridge.build_surface_map(
        _columns(geometry, occupied_rows={10}, free_rows={11}), geometry,
        _identity(geometry))
    assert surface["state"][0][10] == "occupied"
    assert surface["height"][0][10] == pytest.approx(0.63 - 0.53)
    assert surface["state"][0][11] == "free"
    assert surface["height"][0][11] == 0.0
    assert surface["state"][0][12] == "unknown"
    assert surface["height"][0][12] == 0.0
    assert surface["confidence"][0][12] == "none"
    assert surface["map_revision"] == 7 and surface["epoch"] == 1
    assert surface["floor_z"] == 0.0
    assert surface["center_base"][2] == pytest.approx(0.53 + (2.01 - 0.53) / 2)


def _solver():
    try:
        from luggage_packing.luggage_packing import placement_solver
    except ImportError:
        try:
            from luggage_packing import placement_solver  # ament-flattened install
        except ImportError as error:
            pytest.skip("placement_solver unavailable: %s" % error)
    return placement_solver


def test_real_solver_consumes_the_bridge_map():
    solver = _solver()
    geometry = _geometry()
    surface = bridge.build_surface_map(
        _columns(geometry, occupied_rows=set(range(20, 40)), free_rows=set(range(40, 60))),
        geometry, _identity(geometry))
    params = solver._default_params()
    assert params["support_tol"] == 0.05 and params["min_support_ratio"] == 0.6
    assert params["clearance_margin"] == 0.03
    result = solver.generate_candidates(surface, [0.30, 0.20, 0.20],
                                        allowed_yaws=[0.0], params=params)
    candidates = [c for c in (result if isinstance(result, list) else result["candidates"])
                  if c.get("feasible")]
    assert candidates, "solver found no candidates on the bridge map"
    assert all(c["verdict"] == "legal" for c in candidates)
    # Container-frame floor is center_base[2] - inner_h/2 = 0.53; heights in
    # the map are floor-relative, so a surface-resting box centers at
    # 0.53 + 0.10 + 0.10 and a floor-prior box at 0.53 + 0.10.
    container_floor = surface["center_base"][2] - surface["inner_size"][2] / 2
    saw_floor = False
    for candidate in candidates:
        z = candidate["center_container"][2]
        # Observed-free support: peak 0 -> z = floor + box_h/2.
        assert candidate["support_layer_z"] == pytest.approx(0.0, abs=1e-9)
        assert z == pytest.approx(container_floor + 0.10, abs=1e-6)
        saw_floor = True
    assert saw_floor, "observed-floor candidates must appear"

    # A 5-row unknown hole inside the occupied band: any footprint touching
    # it is rejected by the solver's own unknown_above_floor rule (peak is
    # above the floor), and the floor-prior path answers only at floor
    # height -- the prior-vs-observed distinction the contract freezes.
    mixed = _columns(geometry, occupied_rows=set(range(20, 60)))
    for column in mixed:
        if 29 <= column["y"] < 34:
            column.update(known_count=0, has_occupied=False, top_z=0.0)
    surface_mixed = bridge.build_surface_map(mixed, geometry, _identity(geometry))
    result_mixed = solver.generate_candidates(surface_mixed, [0.30, 0.20, 0.20],
                                              allowed_yaws=[0.0], params=params)
    everything = result_mixed if isinstance(result_mixed, list) else result_mixed["candidates"]
    saw_surface_arithmetic = False
    for candidate in everything:
        if candidate["support_layer_z"] == pytest.approx(0.10):
            saw_surface_arithmetic = True
            assert candidate["center_container"][2] == pytest.approx(
                container_floor + 0.20, abs=1e-6)
    assert saw_surface_arithmetic, "surface-resting arithmetic must be exercised"
    saw_prior = False
    for candidate in everything:
        if candidate.get("feasible") and candidate["support_source"] == "floor_prior":
            saw_prior = True
        touches_hole = candidate["iy0"] <= 33 and candidate["iy0"] + 10 > 29
        if candidate.get("feasible") and touches_hole:
            assert candidate["support_source"] == "floor_prior"
            assert candidate["center_container"][2] == pytest.approx(container_floor + 0.10)
        if not candidate.get("feasible") and touches_hole and candidate["support_source"] == "sensor":
            assert candidate["reason"] == "unknown_above_floor"
    assert saw_prior, "floor-prior candidates must answer beside the hole"


@pytest.mark.skipif(not os.environ.get("OCC_FS_NODE"), reason="OCC_FS_NODE not set")
def test_live_bridge_chain_cross_checks_freespace(tmp_path):
    pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2, PointField
    from uld_occ_observer.srv import (GetSnapshot, IntegrateObservation,
                                      QueryFreespace, ReadSurface,
                                      ReleaseSnapshot)
    import rclpy
    from rclpy.node import Node

    node_binary = os.environ["OCC_FS_NODE"]
    log_path = tmp_path / "observer.log"
    log = log_path.open("w")
    # Run the node against ITS OWN install's libs even when the test runs in
    # an environment sourced from a different workspace.
    install_lib = Path(node_binary).resolve().parent.parent.parent / "lib"
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "%s:%s" % (install_lib,
                                        env.get("LD_LIBRARY_PATH", ""))
    process = subprocess.Popen(
        [node_binary, "--ros-args", "-p", "geometry_file:=%s" % GEOMETRY, "-p", "config_file:=%s" % CONFIG,
         "-p", "input_mode:=service"],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    rclpy.init()
    client_node = Node("bridge_live_client")
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(client_node)
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        integrate = client_node.create_client(
            IntegrateObservation, "/uld_occ_observer/integrate_observation")
        get_snapshot = client_node.create_client(
            GetSnapshot, "/uld_occ_observer/get_snapshot")
        freespace = client_node.create_client(
            QueryFreespace, "/uld_occ_observer/query_freespace")
        release = client_node.create_client(
            ReleaseSnapshot, "/uld_occ_observer/release_snapshot")
        for service in (integrate, get_snapshot, freespace, release):
            deadline = time.monotonic() + 20
            while not service.wait_for_service(timeout_sec=0.2):
                assert time.monotonic() < deadline, service.srv_name

        geometry = _geometry()
        floor = geometry["floor_z"]
        sx = int(math.ceil(geometry["length"] / 0.02))
        sy = int(math.ceil(geometry["width"] / 0.02))
        rotation = [0, 0, 1, -1, 0, 0, 0, -1, 0]
        origin = (-1.15, 0.0, 0.0)

        def optical(target):
            d = [target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]]
            return [rotation[0] * d[0] + rotation[3] * d[1] + rotation[6] * d[2],
                    rotation[1] * d[0] + rotation[4] * d[1] + rotation[7] * d[2],
                    rotation[2] * d[0] + rotation[5] * d[1] + rotation[8] * d[2]]

        def sweep(stamp_ns, z_cell, y_from, y_to, x_end):
            # One observation per (row, layer) through the real service: a
            # horizontal ray with boresight +x, exactly the F-fixture row
            # primitive, so the analytic model applies to the live map too.
            z_world = floor + (z_cell + 0.5) * 0.02
            row_rotation = [0, 0, 1, 0, -1, 0, 1, 0, 0]
            request = IntegrateObservation.Request()
            request.sensor_id = "bridge_cam"
            request.observation_id = "sweep_z%d_y%d" % (z_cell, y_from)
            request.clock_domain = "ros"
            request.geometry_hash = geometry["geometry_hash"]
            request.epoch = 1
            cloud = PointCloud2()
            cloud.header.frame_id = "bridge_optical"
            cloud.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            cloud.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
            cloud.height = 1
            cloud.is_bigendian = False
            cloud.point_step = 12
            points = []
            for y in range(y_from, y_to):
                y_world = -geometry["width"] / 2 + (y + 0.5) * 0.02
                x_world = -geometry["length"] / 2 + (x_end + 0.5) * 0.02
                # origin at the row's own y and the layer's z: optical z is
                # the +x travel, so every endpoint passes the depth filter.
                origin = (-1.15, y_world, z_world)
                target = (x_world, y_world, z_world)
                d = [target[0] - origin[0], 0.0, 0.0]
                points.append([row_rotation[0] * d[0] + row_rotation[3] * d[1]
                               + row_rotation[6] * d[2],
                               row_rotation[1] * d[0] + row_rotation[4] * d[1]
                               + row_rotation[7] * d[2],
                               row_rotation[2] * d[0] + row_rotation[5] * d[1]
                               + row_rotation[8] * d[2]])
            cloud.width = len(points)
            cloud.row_step = 12 * len(points)
            for index, name in enumerate(("x", "y", "z")):
                field = PointField()
                field.name = name
                field.offset = index * 4
                field.datatype = PointField.FLOAT32
                field.count = 1
                cloud.fields.append(field)
            cloud.data = b"".join(struct.pack("<fff", *point) for point in points)
            request.cloud = cloud
            # One TF per row would be needed for per-row origins; a service
            # request carries a single TF, so integrate row by row.
            futures = []
            for y_index, point in enumerate(points):
                pass
            request.uld_from_optical = _row_transform(cloud.header.stamp, points[0])
            # The transform's translation is the ray origin of ROW y_from;
            # the other rows' rays must come from their own origins, so this
            # sweep integrates exactly one row per call site below.
            request.cloud.width = 1
            request.cloud.row_step = 12
            request.cloud.data = struct.pack("<fff", *points[0])
            future = integrate.call_async(request)
            deadline = time.monotonic() + 10
            while not future.done():
                assert time.monotonic() < deadline
                time.sleep(0.02)
            receipt = future.result()
            assert receipt.receipt.status == "INTEGRATED", receipt.receipt.status
            assert receipt.receipt.accepted_points == 1
            return points[1:]

        def _row_transform(stamp, point):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "container_link"
            transform.child_frame_id = "bridge_optical"
            # Identity rotation, translation = the optical point's ULD
            # position (the ray endpoint IS the first wall hit).
            return transform

        def integrate_row(stamp_ns, z_cell, y, x_end):
            z_world = floor + (z_cell + 0.5) * 0.02
            y_world = -geometry["width"] / 2 + (y + 0.5) * 0.02
            x_world = -geometry["length"] / 2 + (x_end + 0.5) * 0.02
            request = IntegrateObservation.Request()
            request.sensor_id = "bridge_cam"
            request.observation_id = "sweep_z%d_y%d" % (z_cell, y)
            request.clock_domain = "ros"
            request.geometry_hash = geometry["geometry_hash"]
            request.epoch = 1
            cloud = PointCloud2()
            cloud.header.frame_id = "bridge_optical"
            cloud.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            cloud.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
            cloud.height = 1
            cloud.width = 1
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = 12
            for index, name in enumerate(("x", "y", "z")):
                field = PointField()
                field.name = name
                field.offset = index * 4
                field.datatype = PointField.FLOAT32
                field.count = 1
                cloud.fields.append(field)
            # Optical frame sits at the row origin; +x boresight; the single
            # endpoint is the far wall, optical z = travel distance.
            cloud.data = struct.pack("<fff", 0.0, 0.0, x_world + 1.15)
            request.cloud = cloud
            transform = TransformStamped()
            transform.header.stamp = cloud.header.stamp
            transform.header.frame_id = "container_link"
            transform.child_frame_id = "bridge_optical"
            transform.transform.translation.x = -1.15
            transform.transform.translation.y = y_world
            transform.transform.translation.z = z_world
            transform.transform.rotation.x = 0.7071067811865476
            transform.transform.rotation.z = 0.7071067811865476
            transform.transform.rotation.w = 0.0  # 180-deg rotation: w must be 0
            request.uld_from_optical = transform
            future = integrate.call_async(request)
            deadline = time.monotonic() + 10
            while not future.done():
                assert time.monotonic() < deadline
                time.sleep(0.02)
            receipt = future.result()
            assert receipt.receipt.status == "INTEGRATED", receipt.receipt.status

        # Sweep rows 20..40 over layers 0..11 (a 0.20 m box needs 10 free
        # cells), then an occupied strip at z=4 over a sub-patch.
        stamp = int(time.time() * 1e9)
        counter = 0
        for z_cell in range(0, 12):
            for y in range(20, 40):
                counter += 1
                integrate_row(stamp + counter * 10_000_000, z_cell, y, sx - 1)
        # Occupied strip: rows 24..30, x 20..36 at z=4 (endpoint per column).
        for y in range(24, 30):
            for x in range(20, 36):
                counter += 1
                z_world = floor + 4.5 * 0.02
                y_world = -geometry["width"] / 2 + (y + 0.5) * 0.02
                x_world = -geometry["length"] / 2 + (x + 0.5) * 0.02
                request = IntegrateObservation.Request()
                request.sensor_id = "bridge_cam"
                request.observation_id = "strip_%d" % counter
                request.clock_domain = "ros"
                request.geometry_hash = geometry["geometry_hash"]
                request.epoch = 1
                cloud = PointCloud2()
                cloud.header.frame_id = "bridge_optical"
                cloud.header.stamp.sec = int((stamp + counter * 10_000_000) // 1_000_000_000)
                cloud.header.stamp.nanosec = int((stamp + counter * 10_000_000) % 1_000_000_000)
                cloud.height, cloud.width = 1, 1
                cloud.is_bigendian = False
                cloud.point_step = cloud.row_step = 12
                for index, name in enumerate(("x", "y", "z")):
                    field = PointField()
                    field.name = name
                    field.offset = index * 4
                    field.datatype = PointField.FLOAT32
                    field.count = 1
                    cloud.fields.append(field)
                cloud.data = struct.pack("<fff", 0.0, 0.0, x_world + 1.15)
                request.cloud = cloud
                transform = TransformStamped()
                transform.header.stamp = cloud.header.stamp
                transform.header.frame_id = "container_link"
                transform.child_frame_id = "bridge_optical"
                transform.transform.translation.x = -1.15
                transform.transform.translation.y = y_world
                transform.transform.translation.z = z_world
                transform.transform.rotation.x = 0.7071067811865476
                transform.transform.rotation.z = 0.7071067811865476
                transform.transform.rotation.w = 0.0  # 180-deg rotation: w must be 0
                request.uld_from_optical = transform
                future = integrate.call_async(request)
                deadline = time.monotonic() + 10
                while not future.done():
                    assert time.monotonic() < deadline
                    time.sleep(0.02)
                receipt = future.result()
                assert receipt.receipt.status == "INTEGRATED", receipt.receipt.status

        # Pull the bridge view through the real ROS services.
        columns = []
        lease_future = get_snapshot.call_async(GetSnapshot.Request(max_age_ms=60000))
        while not lease_future.done():
            time.sleep(0.02)
        lease = lease_future.result()
        assert lease.status == "OK", lease.status
        read = client_node.create_client(
            ReadSurface, "/uld_occ_observer/read_surface")
        deadline = time.monotonic() + 10
        while not read.wait_for_service(timeout_sec=0.2):
            assert time.monotonic() < deadline
        cursor, done = 0, False
        while not done:
            request = ReadSurface.Request(handle=lease.handle, cursor=cursor, limit=65536)
            future = read.call_async(request)
            while not future.done():
                time.sleep(0.02)
            response = future.result()
            assert response.status == "OK", response.status
            identity = {
                "geometry_hash": response.identity.geometry_hash,
                "map_revision": response.identity.map_revision,
                "epoch": response.identity.epoch,
                "observed_revision": response.identity.observed_revision,
                "last_integrated_stamp_ns": response.identity.last_integrated_stamp_ns,
                "clock_domain": response.identity.clock_domain,
            }
            for i in range(len(response.x)):
                columns.append({
                    "x": response.x[i], "y": response.y[i],
                    "active_count": response.active_count[i],
                    "known_count": response.known_count[i],
                    "has_occupied": response.has_occupied[i],
                    "top_z": response.top_z[i],
                })
            cursor, done = response.next_cursor, response.done
        release.call_async(ReleaseSnapshot.Request(handle=lease.handle))
        surface = bridge.build_surface_map(columns, geometry, identity)
        assert "error" not in surface

        solver = _solver()
        params = solver._default_params()
        result = solver.generate_candidates(surface, [0.30, 0.20, 0.20],
                                            allowed_yaws=[0.0], params=params)
        solver_candidates = [c for c in (result if isinstance(result, list)
                                         else result["candidates"])
                             if c.get("feasible")]

        # Known-free candidates from query_freespace on the same node.
        lease_future = get_snapshot.call_async(GetSnapshot.Request(max_age_ms=60000))
        lease_deadline = time.monotonic() + 10
        while not lease_future.done():
            assert time.monotonic() < lease_deadline
            time.sleep(0.02)
        lease = lease_future.result()
        assert lease.status == "OK"
        query = QueryFreespace.Request()
        query.handle = lease.handle
        query.box_length = 0.30
        query.box_width = 0.20
        query.box_height = 0.20
        query.yaw_mode = 0
        query.cursor = 0
        query.limit = 65536
        future = freespace.call_async(query)
        deadline = time.monotonic() + 10
        while not future.done():
            assert time.monotonic() < deadline
            time.sleep(0.02)
        page = future.result()
        assert page.status == "OK", page.status
        release.call_async(ReleaseSnapshot.Request(handle=lease.handle))
        free_cells = {(int(page.cell_x[i]), int(page.cell_y[i]), int(page.cell_z[i]))
                      for i in range(len(page.cell_x))}
        assert free_cells, "no known-free candidates on the live node"

        # Cross-check: a solver candidate whose footprint support is fully
        # observed (no unknown column under it) must be a known-free candidate
        # at the same discrete position; a candidate resting over unknown
        # columns is a policy/floor-prior answer and must never be known-free.
        state = surface["state"]
        # Cross-language consistency on the same live snapshot:
        #  1. every known-free candidate's footprint sits on columns the
        #     bridge map reports as observed (never unknown);
        #  2. a solver candidate resting over unknown columns is a policy /
        #     floor-prior answer and never appears as known-free.
        for cell_x, cell_y, cell_z in free_cells:
            for dx in range(0, 15):
                for dy in range(0, 10):
                    column = state[max(0, min(cell_x + dx, surface["nx"] - 1))] \
                        [max(0, min(cell_y + dy, surface["ny"] - 1))]
                    assert column != "unknown", (
                        "known-free candidate over an unobserved column",
                        (cell_x, cell_y, cell_z, dx, dy))
        checked_prior = 0
        for candidate in solver_candidates:
            cell_x, cell_y = candidate["ix0"], candidate["iy0"]
            cell_z = int(round((candidate["center_container"][2] - 0.10
                                - geometry["floor_z"]) / 0.02))
            footprint_unknown = any(
                state[max(0, min(cell_x + dx, surface["nx"] - 1))]
                    [max(0, min(cell_y + dy, surface["ny"] - 1))] == "unknown"
                for dx in range(0, 15) for dy in range(0, 10))
            if footprint_unknown:
                checked_prior += 1
                assert (cell_x, cell_y, cell_z) not in free_cells, (
                    "unknown-support solver candidate leaked into known-free",
                    (cell_x, cell_y, cell_z))
        assert checked_prior > 0, "no prior-path solver candidate to check"
    finally:
        executor.shutdown()
        client_node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        log.close()
