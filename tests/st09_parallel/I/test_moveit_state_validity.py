"""ST09-I S4 offline: the ValidateRouteV2 certification adapter.

Pure-python layers only (no ROS): the sampling oracles (C-parity grid +
controller interpolation), the frozen-scene ACM composition (SRDF
defaults preserved, payload entries stripped, declared structure pair
added, recorded separately), and the shared link-relative attach
construction — including the behavioral guard that refuses a nonzero
link pose expressed in any other frame (the [ST09-I] frame bug class).
The certified trajectory is contact-free (release gap before the
landing): there is no landing-contact legality judge to test.
"""

from __future__ import division

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
# APPEND, never insert at 0 (same overlay-shadowing hazard as
# test_route_producer: a front src entry flips in-process package
# resolution off the install overlay and the whole-chain probe's
# module provenance is then rejected by every stack node).
for entry in (str(ROOT / "src" / "luggage_planning"), str(ROOT)):
    if entry not in sys.path:
        sys.path.append(entry)

from luggage_planning.planning_contract.fixtures import make_timed_trajectory  # noqa: E402

from luggage_planning.motion.moveit_state_validity import (  # noqa: E402
    DECLARED_STRUCTURE_PAIRS,
    PAYLOAD_OBJECT_ID,
    SUCTION_INTERFACE_TOUCH_LINKS,
    FrozenSceneComposer,
    interpolated_positions,
    sample_grid,
)
from luggage_planning.motion.planning_scene_client import (  # noqa: E402
    build_attach_scene,
)
from luggage_planning.motion.trajectory_certifier import (  # noqa: E402
    MAX_CERT_SAMPLE_DT_NS,
    TrajectoryCertifier,
)


def timed(rows, dt_ms=50.0, velocities=None, accelerations=None):
    """A TimedTrajectory-shaped object with optional derivatives."""
    times = tuple(int(dt_ms * 1e6 * index) for index in range(len(rows)))
    n = len(rows[0])
    zeros = lambda: tuple((0.0,) * n for _ in rows)  # noqa: E731
    return type("T", (), {
        "joint_names": tuple("elfin_joint%d" % i for i in range(1, n + 1)),
        "positions": tuple(tuple(float(v) for v in row) for row in rows),
        "velocities": velocities if velocities is not None else zeros(),
        "accelerations": accelerations if accelerations is not None
        else zeros(),
        "time_from_start_ns": times})()


class TestSamplingOracle:
    """The sampled states must be the states the controller tracks."""

    def test_grid_matches_the_certifiers_iteration(self):
        trajectory = timed([(0.0,), (0.5,), (0.4,)], dt_ms=25.0)
        grid = sample_grid(trajectory)
        # replicate the certifier's own iteration exactly
        expected = []
        stamps = [0, 25_000_000, 50_000_000]
        for t0, t1 in zip(stamps, stamps[1:]):
            dt = t1 - t0
            subdivisions = max(1, int((dt + MAX_CERT_SAMPLE_DT_NS - 1)
                                      // MAX_CERT_SAMPLE_DT_NS))
            for step in range(subdivisions + 1):
                expected.append(t0 + dt * step / subdivisions)
        assert grid == expected

    def test_grid_bounds(self):
        trajectory = timed([(0.0,), (1.0,)], dt_ms=100.0)
        grid = sample_grid(trajectory)
        assert all(b - a <= MAX_CERT_SAMPLE_DT_NS + 1e-6
                   for a, b in zip(grid, grid[1:]))
        assert grid[0] == 0.0 and grid[-1] == 100_000_000.0

    def test_knots_and_endpoints_included_quintic(self):
        trajectory = timed(
            [(0.0,), (0.6,)], dt_ms=40.0,
            velocities=((0.0,), (0.0,)),
            accelerations=((0.0,), (0.0,)))
        rows = interpolated_positions(trajectory)
        assert rows[0] == (0.0,)
        assert rows[-1] == (0.6,)
        # rest-to-rest quintic is symmetric: the temporal midpoint hits
        # exactly half the travel (40 ms span -> 4 subdivisions)
        assert len(rows) == 5
        assert abs(rows[2][0] - 0.3) < 1e-9
        assert all(-1e-12 <= row[0] <= 0.6 + 1e-12 for row in rows)

    def test_quintic_matches_analytic_hermite(self):
        left, right = (0.2,), (0.8,)
        velocity_l, velocity_r = (0.1,), (-0.1,)
        accel_l, accel_r = (0.0,), (0.2,)
        trajectory = timed(
            [left, right], dt_ms=40.0,
            velocities=(velocity_l, velocity_r),
            accelerations=(accel_l, accel_r))
        rows = interpolated_positions(trajectory)
        t_total = 0.04
        for s, row in zip([index / 4.0 for index in range(5)], rows):
            s2, s3, s4, s5 = s ** 2, s ** 3, s ** 4, s ** 5
            expected = (
                left[0] * (1 - 10 * s3 + 15 * s4 - 6 * s5)
                + right[0] * (10 * s3 - 15 * s4 + 6 * s5)
                + velocity_l[0] * t_total * (s - 6 * s3 + 8 * s4 - 3 * s5)
                + velocity_r[0] * t_total
                * (-4 * s3 + 7 * s4 - 3 * s5)
                + accel_l[0] * t_total ** 2 / 2
                * (s2 - 3 * s3 + 3 * s4 - s5)
                + accel_r[0] * t_total ** 2 / 2
                * (s3 - 2 * s4 + s5))
            assert abs(row[0] - expected) < 1e-12

    def test_cubic_and_linear_fallbacks(self):
        cubic = timed([(0.0,), (1.0,)], dt_ms=40.0,
                      velocities=((0.0,), (0.0,)),
                      accelerations=())
        rows = interpolated_positions(cubic)
        assert len(rows) == 5
        assert abs(rows[2][0] - 0.5) < 1e-9
        linear = timed([(0.0,), (1.0,)], dt_ms=40.0,
                       velocities=(), accelerations=())
        rows = interpolated_positions(linear)
        assert len(rows) == 5
        assert abs(rows[2][0] - 0.5) < 1e-9
        assert abs(rows[1][0] - 0.25) < 1e-9


class TestSharedAttachConstruction:
    def test_link_relative_pose_defaults_to_link_frame(self):
        scene = build_attach_scene(
            PAYLOAD_OBJECT_ID, (0.0, 0.0, 0.1), (-1.0, 0.0, 0.0, 0.0),
            [0.35, 0.25, 0.20], "suction_contact_frame",
            touch_links=SUCTION_INTERFACE_TOUCH_LINKS)
        attached = scene.robot_state.attached_collision_objects[0]
        assert attached.link_name == "suction_contact_frame"
        # THE frame bug guard: the measured T_tcp_box rides in the LINK
        # frame, never the planning frame
        assert attached.object.header.frame_id == \
            "suction_contact_frame"
        pose = attached.object.primitive_poses[0]
        assert (pose.position.x, pose.position.y, pose.position.z) == \
            (0.0, 0.0, 0.1)
        assert (pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w) == \
            (-1.0, 0.0, 0.0, 0.0)

    def test_nonzero_pose_in_foreign_frame_is_refused(self):
        try:
            build_attach_scene(
                PAYLOAD_OBJECT_ID, (0.0, 0.0, 0.1),
                (0.0, 0.0, 0.0, 1.0), [0.35, 0.25, 0.2],
                "suction_contact_frame", frame_id="world")
        except ValueError as error:
            assert "ATTACH_FRAME_MISMATCH" in str(error)
        else:
            raise AssertionError("foreign-frame attach was accepted")

    def test_rotated_pose_in_foreign_frame_is_refused(self):
        try:
            build_attach_scene(
                PAYLOAD_OBJECT_ID, (0.0, 0.0, 0.0),
                (-1.0, 0.0, 0.0, 0.0), [0.35, 0.25, 0.2],
                "suction_contact_frame", frame_id="world")
        except ValueError as error:
            assert "ATTACH_FRAME_MISMATCH" in str(error)
        else:
            raise AssertionError("foreign-frame attach was accepted")

    def test_move_world_object_transmits_canonical_zeros(self):
        scene = build_attach_scene(
            "pickup_box", (1.2, -0.4, 0.9), (0.1, 0.2, 0.3, 0.9),
            [0.3, 0.2, 0.2], "suction_contact_frame",
            pose_semantics="move_world_object")
        attached = scene.robot_state.attached_collision_objects[0]
        pose = attached.object.primitive_poses[0]
        assert (pose.position.x, pose.position.y, pose.position.z) == \
            (0.0, 0.0, 0.0)
        assert pose.orientation.w == 1.0
        assert attached.object.header.frame_id == "world"


def fake_live_acm():
    """A live ACM carrying SRDF adjacent defaults + a runtime payload."""
    from moveit_msgs.msg import (
        AllowedCollisionEntry,
        AllowedCollisionMatrix,
    )

    names = ["elfin_base", "elfin_link1", "elfin_link2", PAYLOAD_OBJECT_ID,
             "suction_panel"]
    allowed = {
        "elfin_base": {"elfin_link1"},
        "elfin_link1": {"elfin_base", "elfin_link2"},
        "elfin_link2": {"elfin_link1"},
        PAYLOAD_OBJECT_ID: {"suction_panel"},
        "suction_panel": {PAYLOAD_OBJECT_ID, "elfin_link6"},
    }
    acm = AllowedCollisionMatrix()
    acm.entry_names = list(names)
    for name in names:
        entry = AllowedCollisionEntry()
        entry.enabled = [bool(other in allowed.get(name, ()))
                         for other in names]
        acm.entry_values.append(entry)
    return acm


class TestFrozenSceneComposition:
    def test_acm_preserves_srdf_defaults_and_strips_payload(self):
        composer = FrozenSceneComposer(None)
        composed = composer._composed_acm(fake_live_acm())  # noqa: SLF001
        names = list(composed.entry_names)
        assert PAYLOAD_OBJECT_ID not in names
        matrix = {
            name: {other for other, flag in zip(names, entry.enabled)
                   if flag}
            for name, entry in zip(names, composed.entry_values)}
        # SRDF defaults survive
        assert "elfin_link1" in matrix["elfin_base"]
        assert "elfin_link2" in matrix["elfin_link1"]
        # declared structure allowance added, symmetric
        for first, second in DECLARED_STRUCTURE_PAIRS:
            assert second in matrix.get(first, set())
            assert first in matrix.get(second, set())
        # stale payload whitelist entries gone
        assert "suction_panel" not in matrix.get(PAYLOAD_OBJECT_ID, set())

    def test_support_resolution_prefers_the_container(self):
        from moveit_msgs.msg import CollisionObject

        class Obj(object):
            def __init__(self, object_id):
                self.id = object_id

        live = type("L", (), {"world": type("W", (), {
            "collision_objects": [
                Obj("robot_pedestal"), Obj("airport_container_real")]})()})()
        composer = FrozenSceneComposer(None)
        assert composer._resolve_support(live) == \
            "airport_container_real"  # noqa: SLF001

    def test_missing_support_fails_closed(self):
        class Obj(object):
            def __init__(self, object_id):
                self.id = object_id

        live = type("L", (), {"world": type("W", (), {
            "collision_objects": [Obj("robot_pedestal")]})()})()
        composer = FrozenSceneComposer(None)
        try:
            composer._resolve_support(live)  # noqa: SLF001
        except Exception as error:  # noqa: BLE001
            assert "CERT_SUPPORT_UNRESOLVED" in str(error)
        else:
            raise AssertionError("missing support did not fail closed")


class TestCertifierCompatibility:
    def test_certifier_iteration_parity_with_grid(self):
        """The certifier's sample requests align with sample_grid 1:1."""
        trajectory = timed(
            [(0.0, 0.1), (0.3, -0.2), (0.1, 0.4)], dt_ms=30.0,
            velocities=tuple((0.01, -0.01) for _ in range(3)),
            accelerations=tuple((0.0, 0.0) for _ in range(3)))
        grid = sample_grid(trajectory)
        positions = interpolated_positions(trajectory)
        # C iterates intervals, subdivisions + 1 requests each
        count = 0
        stamps = [0, 30_000_000, 60_000_000]
        for t0, t1 in zip(stamps, stamps[1:]):
            dt = t1 - t0
            subdivisions = max(1, int((dt + MAX_CERT_SAMPLE_DT_NS - 1)
                                      // MAX_CERT_SAMPLE_DT_NS))
            count += subdivisions + 1
        assert count == len(grid) == len(positions)
        # grid times are non-decreasing and knots appear
        assert all(b >= a for a, b in zip(grid, grid[1:]))
        assert 30_000_000.0 in grid and 60_000_000.0 in grid

    def test_make_timed_trajectory_shapes_still_admit(self):
        trajectory = make_timed_trajectory()
        rows = interpolated_positions(trajectory)
        assert len(rows) == len(sample_grid(trajectory)) > 1
        for row, knot in zip(rows[::len(rows) - 1],
                             (trajectory.positions[0],
                              trajectory.positions[-1])):
            assert all(abs(a - b) < 1e-9 for a, b in zip(row, knot))
