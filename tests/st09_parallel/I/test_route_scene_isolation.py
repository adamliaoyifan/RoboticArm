"""Request-scene isolation for carrying and unloaded certification.

The live planning scene may change while edges are built. Certification
rebuilds both load states from the request surface, so that order cannot
change the checked world. Occupancy columns stay on the carrying scene.
"""

from __future__ import division

import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "src" / "luggage_planning"
# This file tests the source under edit. The install overlay would hide it.
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

from luggage_planning.routing.moveit_edge_adapter import (  # noqa: E402
    BoxSpec,
    OccupancyColumnError,
    PayloadSpec,
    SceneState,
    occupancy_boxes_from_surface,
)
from luggage_planning.motion.moveit_state_validity import (  # noqa: E402
    FrozenSceneComposer,
    _scene_digest,
)
from luggage_planning.motion.planning_scene_client import (  # noqa: E402
    build_collision_object,
)
from luggage_planning.routing.producer import RouteProducer  # noqa: E402
from moveit_msgs.msg import (  # noqa: E402
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
)
from luggage_planning.planning_contract.attachment import transform_point  # noqa: E402


FLOOR_Z = 0.53
TRANSFORM = {
    "translation": [-1.5, 0.6, 0.0],
    "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
}


def _surface(height):
    return {
        "frame_id": "container_link",
        "origin_xy_container": [-0.745, -0.985],
        "resolution_m": 0.05,
        "floor_z_container": FLOOR_Z,
        "state": [["occupied"]],
        "height_above_floor": [[height]],
    }


def _column(height=0.25):
    boxes = occupancy_boxes_from_surface(_surface(height), TRANSFORM)
    assert len(boxes) == 1
    return boxes[0]


def _live(objects, extra_acm=()):
    scene = PlanningScene()
    scene.is_diff = False
    scene.name = "live"
    scene.world.collision_objects = list(objects)
    names = ["airport_container_real"] + list(extra_acm)
    acm = AllowedCollisionMatrix()
    acm.entry_names = names
    for _name in names:
        entry = AllowedCollisionEntry()
        entry.enabled = [False] * len(names)
        acm.entry_values.append(entry)
    scene.allowed_collision_matrix = acm
    return scene


def _container():
    return build_collision_object(
        "airport_container_real", [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 1.0], [1.0, 1.0, 1.0], "world",
        CollisionObject.ADD)


def _payload():
    return PayloadSpec(
        dimensions_wdh=(0.30, 0.20, 0.20),
        t_tcp_box_translation=(0.0, 0.0, 0.10),
        t_tcp_box_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))


def _edge(name):
    class _Edge(object):
        def __init__(self):
            self.edge_id = name
            self.load_state = "carrying"

    return _Edge()


def _carrying(box):
    return SceneState(
        load_state="carrying", payload=_payload(), placed_boxes=(box,))


def _records(scene):
    rows = []
    for obj in scene.world.collision_objects:
        pose = obj.primitive_poses[0]
        rows.append({
            "id": str(obj.id),
            "xyz": (pose.position.x, pose.position.y, pose.position.z),
            "size": tuple(float(v) for v in obj.primitives[0].dimensions),
        })
    attached = [
        str(item.object.id)
        for item in scene.robot_state.attached_collision_objects
        if item.object]
    return rows, attached


def _inside(box, point):
    from luggage_planning.planning_contract.attachment import invert_transform

    inverse_t, inverse_q = invert_transform(box.center, box.quaternion_xyzw)
    local = transform_point(inverse_t, inverse_q, point)
    half = [0.5 * float(v) for v in box.size_whd]
    return all(abs(local[axis]) <= half[axis] + 1e-9 for axis in range(3))


class TestOccupancyColumns:

    def test_column_uses_container_floor_and_world_transform(self):
        short = _column(0.20)
        tall = _column(0.25)
        assert short.size_whd[2] == 0.20
        assert tall.size_whd[2] == 0.25
        local = (-0.745 + 0.5 * 0.05, -0.985 + 0.5 * 0.05,
                 FLOOR_Z + 0.5 * 0.25)
        world = transform_point(
            TRANSFORM["translation"], TRANSFORM["quaternion_xyzw"], local)
        assert tall.center == world
        assert tall.center != (-0.17, -0.21, 0.15)
        assert local[2] == 0.655

    def test_missing_height_does_not_invent_a_slab(self):
        surface = _surface(0.25)
        del surface["height_above_floor"]
        try:
            occupancy_boxes_from_surface(surface, TRANSFORM)
        except OccupancyColumnError as exc:
            assert str(exc).startswith("OCCUPANCY_HEIGHT_MISSING")
        else:
            raise AssertionError("missing height invented a column")

    def test_legacy_floor_zero_is_not_the_container_floor(self):
        surface = {
            "origin_xy_container": [-0.745, -0.985],
            "resolution_m": 0.05,
            "floor_z": 0.0,
            "state": [["occupied"]],
            "height": [[0.25]],
        }
        try:
            occupancy_boxes_from_surface(surface, TRANSFORM)
        except OccupancyColumnError as exc:
            assert str(exc).startswith("OCCUPANCY_FLOOR_MISSING")
        else:
            raise AssertionError("floor_z 0 was accepted as container floor")


class TestCertSceneIsolation:

    def test_carrying_scene_is_unchanged_by_unloaded_edge_build(self):
        column = _column(0.25)
        state = _carrying(column)
        edge = _edge("carry")
        clean = _live([_container()])
        polluted = _live(
            [_container(), build_collision_object(
                "snapshot-cell-0-0", [-0.17, -0.21, 0.15],
                [0.0, 0.0, 0.0, 1.0], [0.15, 0.15, 0.30], "world",
                CollisionObject.ADD),
             build_collision_object(
                 "route-released-box", [1.0, 1.0, 1.0],
                 [0.0, 0.0, 0.0, 1.0], [0.4, 0.3, 0.2], "world",
                 CollisionObject.ADD)],
            extra_acm=("snapshot-cell-0-0", "route-released-box"))
        composer = FrozenSceneComposer(None)
        before, _indices, _support = composer.compose(
            clean, (0.0,) * 6, {"carry": state}, [edge])
        after, _indices, _support = composer.compose(
            polluted, (0.0,) * 6, {"carry": state}, [edge])
        assert _records(before[1]) == _records(after[1])
        assert _scene_digest([before[1]]) == _scene_digest([after[1]])
        ids = [row["id"] for row in _records(after[1])[0]]
        assert "snapshot-cell-0-0" in ids
        assert ids.count("snapshot-cell-0-0") == 1
        assert "route-released-box" not in ids
        kept = next(
            row for row in _records(after[1])[0]
            if row["id"] == "snapshot-cell-0-0")
        assert kept["xyz"] == column.center
        assert kept["size"][2] == 0.25
        assert kept["xyz"] != (-0.17, -0.21, 0.15)

    def test_occupancy_only_obstacle_blocks_the_carry_point(self):
        column = _column(0.25)
        composer = FrozenSceneComposer(None)
        blocked = _carrying(column)
        clear = SceneState(
            load_state="carrying", payload=_payload(), placed_boxes=())
        edge = _edge("carry")
        live = _live([_container()])
        with_cargo, _, _ = composer.compose(
            live, (0.0,) * 6, {"carry": blocked}, [edge])
        without, _, _ = composer.compose(
            live, (0.0,) * 6, {"carry": clear}, [edge])
        cargo_ids = [row["id"] for row in _records(with_cargo[1])[0]]
        clear_ids = [row["id"] for row in _records(without[1])[0]]
        assert "snapshot-cell-0-0" in cargo_ids
        assert "snapshot-cell-0-0" not in clear_ids
        assert _inside(column, column.center)
        assert not any(
            _inside(BoxSpec(
                object_id=row["id"], center=row["xyz"],
                size_whd=row["size"]), column.center)
            for row in _records(without[1])[0])

    def test_both_load_states_keep_the_same_columns(self):
        request = {
            "surface": _surface(0.25),
            "container_transform": TRANSFORM,
        }

        class _Load(object):
            def __init__(self, load):
                self.edge_id = load
                self.load_state = load

        states = RouteProducer.edge_scene_states(
            object.__new__(RouteProducer), None,
            [_Load("carrying"), _Load("unloaded")],
            None, None, None, request)
        assert states["carrying"].placed_boxes == states["unloaded"].placed_boxes
        assert len(states["carrying"].placed_boxes) == 1
        assert states["carrying"].placed_boxes[0].size_whd[2] == 0.25
        digest = hashlib.sha256(repr(
            states["carrying"].placed_boxes).encode("utf-8")).hexdigest()
        assert digest == hashlib.sha256(repr(
            states["unloaded"].placed_boxes).encode("utf-8")).hexdigest()
