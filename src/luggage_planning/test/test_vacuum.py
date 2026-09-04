"""Vacuum gate + backend + planning-scene message tests (no ROS graph)."""

import math
import unittest

import pytest

from luggage_planning.vacuum_backend import (
    SimVacuumBackend,
    StubVacuumBackend,
    relative_offset,
    compose_pose,
)
from luggage_planning.vacuum_gate import (
    VACUUM_NOT_IN_CONTACT,
    VACUUM_NO_BOX,
    VACUUM_RETENTION_MARGIN,
    VACUUM_TILT_EXCEEDED,
    VacuumGate,
)

PANEL_XYZ = (-1.0, 0.0, 1.15)
PANEL_QUAT = (1.0, 0.0, 0.0, 0.0)  # tool Z down
BOX_XYZ = (-1.0, 0.0, 1.01)
BOX_QUAT = (0.0, 0.0, 0.0, 1.0)
BOX_SIZE = (0.70, 0.45, 0.28)
BOX_MASS = 15.0
BOX_RADIUS = 0.5 * math.sqrt(0.70 ** 2 + 0.45 ** 2 + 0.28 ** 2)


class TestVacuumGate(unittest.TestCase):

    def test_contact_ok_passes(self):
        gate = VacuumGate()
        result = gate.evaluate(PANEL_XYZ, PANEL_QUAT, BOX_XYZ, BOX_SIZE,
                               BOX_MASS, BOX_RADIUS)
        self.assertTrue(result.ok, result.reason)
        self.assertGreater(result.retention_margin, 2.0)

    def test_no_box(self):
        result = VacuumGate().evaluate(
            PANEL_XYZ, PANEL_QUAT, None, None, 0.0, 0.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, VACUUM_NO_BOX)

    def test_far_box_rejected(self):
        result = VacuumGate().evaluate(
            (0.0, 0.0, 1.7), PANEL_QUAT, BOX_XYZ, BOX_SIZE,
            BOX_MASS, BOX_RADIUS)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, VACUUM_NOT_IN_CONTACT)

    def test_hover_30cm_rejected(self):
        hover = (PANEL_XYZ[0], PANEL_XYZ[1], BOX_XYZ[2] + 0.5 * BOX_SIZE[2] + 0.30)
        result = VacuumGate().evaluate(
            hover, PANEL_QUAT, BOX_XYZ, BOX_SIZE, BOX_MASS, BOX_RADIUS)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, VACUUM_NOT_IN_CONTACT)
        self.assertAlmostEqual(result.contact_distance, 0.30, places=5)

    def test_side_30cm_rejected(self):
        side = (BOX_XYZ[0] + 0.5 * BOX_SIZE[0] + 0.30, BOX_XYZ[1], PANEL_XYZ[2])
        result = VacuumGate().evaluate(
            side, PANEL_QUAT, BOX_XYZ, BOX_SIZE, BOX_MASS, BOX_RADIUS)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, VACUUM_NOT_IN_CONTACT)

    def test_tilt_rejected(self):
        # 90-degree tilt: quaternion (0, 0, 0, 1) means +Z points up-ish
        result = VacuumGate().evaluate(
            PANEL_XYZ, (0.0, 0.0, 0.0, 1.0), BOX_XYZ, BOX_SIZE,
            BOX_MASS, BOX_RADIUS)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, VACUUM_TILT_EXCEEDED)

    def test_overweight_rejected(self):
        result = VacuumGate().evaluate(
            PANEL_XYZ, PANEL_QUAT, BOX_XYZ, BOX_SIZE,
            200.0, BOX_RADIUS)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, VACUUM_RETENTION_MARGIN)


class FakeGzClient(object):
    def __init__(self):
        self.poses = []

    def set_model_pose(self, model, xyz, quat):
        self.poses.append((model, tuple(xyz), tuple(quat)))
        return True, ""


class FakeScene(object):
    def __init__(self, fail_attach=False, fail_detach=False):
        self.attached = None
        self.detached = None
        self.fail_attach = fail_attach
        self.fail_detach = fail_detach

    def attach_pickup_box(self, model, xyz, quat, size, link):
        if self.fail_attach:
            return False, "scene down"
        self.attached = (model, tuple(xyz), tuple(quat), tuple(size), link)
        return True, "ok"

    def detach_and_remove(self, model):
        if self.fail_detach:
            return False, "scene down"
        self.detached = model
        return True, "ok"


class TestSimBackend(unittest.TestCase):

    def _context(self):
        return {
            "model_name": "pickup_box_0001",
            "panel_xyz": PANEL_XYZ, "panel_quat": PANEL_QUAT,
            "box_xyz": BOX_XYZ, "box_quat": BOX_QUAT,
            "box_size": BOX_SIZE,
        }

    def test_attach_detach_roundtrip(self):
        gz, scene = FakeGzClient(), FakeScene()
        backend = SimVacuumBackend(gz, scene)
        ok, _ = backend.attach(self._context())
        self.assertTrue(ok)
        self.assertTrue(backend.is_attached())
        self.assertEqual(scene.attached[0], "pickup_box_0001")

        ok, _ = backend.detach({})
        self.assertTrue(ok)
        self.assertFalse(backend.is_attached())
        self.assertEqual(scene.detached, "pickup_box_0001")

    def test_follow_moves_box_with_panel(self):
        gz, scene = FakeGzClient(), FakeScene()
        backend = SimVacuumBackend(gz, scene)
        backend.attach(self._context())
        # Panel lifts 0.35 m: the box must move by the same world offset.
        ok, msg = backend.follow_step(
            (PANEL_XYZ[0], PANEL_XYZ[1], PANEL_XYZ[2] + 0.35), PANEL_QUAT)
        self.assertTrue(ok, msg)
        model, xyz, _ = gz.poses[-1]
        self.assertEqual(model, "pickup_box_0001")
        self.assertAlmostEqual(xyz[2], BOX_XYZ[2] + 0.35, places=6)
        self.assertAlmostEqual(xyz[0], BOX_XYZ[0], places=6)

    def test_follow_without_attach_is_noop(self):
        gz, scene = FakeGzClient(), FakeScene()
        backend = SimVacuumBackend(gz, scene)
        backend.follow_step(PANEL_XYZ, PANEL_QUAT)
        self.assertEqual(gz.poses, [])

    def test_scene_failure_aborts_attach(self):
        gz, scene = FakeGzClient(), FakeScene(fail_attach=True)
        backend = SimVacuumBackend(gz, scene)
        ok, message = backend.attach(self._context())
        self.assertFalse(ok)
        self.assertIn("VACUUM_BACKEND_ERROR", message)
        self.assertFalse(backend.is_attached())

    def test_detach_failure_keeps_attached(self):
        gz, scene = FakeGzClient(), FakeScene(fail_detach=True)
        backend = SimVacuumBackend(gz, scene)
        self.assertTrue(backend.attach(self._context())[0])
        ok, message = backend.detach({})
        self.assertFalse(ok)
        self.assertIn("VACUUM_BACKEND_ERROR", message)
        self.assertTrue(backend.is_attached())

    def test_offset_roundtrip_identity(self):
        offset = relative_offset(PANEL_XYZ, PANEL_QUAT, BOX_XYZ, BOX_QUAT)
        xyz, quat = compose_pose(PANEL_XYZ, PANEL_QUAT, offset[0], offset[1])
        for got, want in zip(xyz, BOX_XYZ):
            self.assertAlmostEqual(got, want, places=6)


class TestStubBackend(unittest.TestCase):

    def test_state_only(self):
        backend = StubVacuumBackend()
        self.assertFalse(backend.is_attached())
        backend.attach({})
        self.assertTrue(backend.is_attached())
        ok, _ = backend.follow_step(PANEL_XYZ, PANEL_QUAT)
        self.assertTrue(ok)
        backend.detach({})
        self.assertFalse(backend.is_attached())


class TestPlanningSceneMessages(unittest.TestCase):

    def test_build_shapes(self):
        pytest.importorskip("moveit_msgs")
        from luggage_planning.planning_scene_client import (
            build_add_scene,
            build_attach_scene,
            build_detach_scene,
        )
        from moveit_msgs.msg import CollisionObject
        add = build_add_scene("pickup_box", BOX_XYZ, BOX_QUAT, BOX_SIZE)
        self.assertTrue(add.is_diff)
        obj = add.world.collision_objects[0]
        self.assertEqual(obj.id, "pickup_box")
        self.assertEqual(list(obj.primitives[0].dimensions),
                         [0.70, 0.45, 0.28])

        attach = build_attach_scene(
            "pickup_box", BOX_XYZ, BOX_QUAT, BOX_SIZE,
            "suction_contact_frame")
        self.assertEqual(
            attach.robot_state.attached_collision_objects[0].link_name,
            "suction_contact_frame")
        self.assertEqual(
            list(attach.allowed_collision_matrix.entry_names), [])

        detach = build_detach_scene("pickup_box")
        self.assertEqual(
            detach.world.collision_objects[0].operation,
            CollisionObject.REMOVE)

    def test_mesh_collision_object_casts_int_indices(self):
        pytest.importorskip("moveit_msgs")
        from luggage_planning.planning_scene_client import (
            build_mesh_collision_object,
            mesh_msg_from_arrays,
        )
        mesh = mesh_msg_from_arrays(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
            [(0, 1, 2)])
        obj = build_mesh_collision_object(
            "airport_container_real", (1.5, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0), mesh)
        self.assertEqual(obj.id, "airport_container_real")
        self.assertEqual(len(obj.meshes[0].triangles), 1)
        self.assertEqual(
            list(obj.meshes[0].triangles[0].vertex_indices), [0, 1, 2])

    def test_acm_pair_roundtrip(self):
        pytest.importorskip("moveit_msgs")
        from moveit_msgs.msg import AllowedCollisionMatrix
        from luggage_planning.planning_scene_client import (
            acm_pair_allowed,
            set_acm_pair,
        )
        acm = AllowedCollisionMatrix()
        set_acm_pair(acm, "pickup_box", "airport_container_real", True)
        self.assertTrue(acm_pair_allowed(
            acm, "pickup_box", "airport_container_real"))
        set_acm_pair(acm, "pickup_box", "airport_container_real", False)
        self.assertFalse(acm_pair_allowed(
            acm, "pickup_box", "airport_container_real"))

    def test_apply_wraps_planning_scene_request(self):
        pytest.importorskip("moveit_msgs")
        from moveit_msgs.srv import ApplyPlanningScene
        from luggage_planning.planning_scene_client import (
            PlanningSceneClient,
            build_add_scene,
        )

        class _Future(object):
            def add_done_callback(self, cb):
                cb(self)

            def result(self):
                response = ApplyPlanningScene.Response()
                response.success = True
                return response

        class _FakeClient(object):
            def __init__(self):
                self.requests = []

            def wait_for_service(self, timeout_sec=0.0):
                del timeout_sec
                return True

            def call_async(self, request):
                self.requests.append(request)
                return _Future()

        class _FakeNode(object):
            def create_client(self, srv, name, **kwargs):
                del srv, name, kwargs
                return client

        client = _FakeClient()
        scene_client = PlanningSceneClient(_FakeNode())
        scene_client._client = client
        ok, _ = scene_client.add_pickup_box(BOX_XYZ, BOX_QUAT, BOX_SIZE)
        self.assertTrue(ok)
        self.assertEqual(len(client.requests), 1)
        self.assertIsInstance(client.requests[0], ApplyPlanningScene.Request)
        self.assertTrue(client.requests[0].scene.is_diff)
        add = build_add_scene("pickup_box", BOX_XYZ, BOX_QUAT, BOX_SIZE)
        self.assertEqual(
            client.requests[0].scene.world.collision_objects[0].id,
            add.world.collision_objects[0].id)


if __name__ == "__main__":
    unittest.main()
