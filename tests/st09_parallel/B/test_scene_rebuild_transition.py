"""A certification redraw must not carry its hypothetical release into entry."""
from types import SimpleNamespace

import pytest

from luggage_planning.routing.moveit_edge_adapter import (
    BoxSpec, MoveItEdgeAdapter, PayloadSpec, SceneState,
)


class SceneClient:
    def __init__(self):
        self.world = {'external-obstacle'}
        self.attached = set()
        self.removed = []
        self.refuse_remove = False

    def add_collision_box(self, object_id, *args, **kwargs):
        self.world.add(object_id)
        return True

    def remove_object(self, object_id):
        self.removed.append(object_id)
        if self.refuse_remove:
            return False
        self.world.discard(object_id)
        return True

    def detach_and_remove(self, *, box_id):
        self.attached.discard(box_id)
        self.world.discard(box_id)
        return True

    def apply_diff(self, diff, **kwargs):
        for attached in diff.robot_state.attached_collision_objects:
            self.attached.add(attached.object.id)
        return True

    def get_scene(self, *args, **kwargs):
        return SimpleNamespace(robot_state=SimpleNamespace(
            attached_collision_objects=[SimpleNamespace(object=SimpleNamespace(id=i))
                                        for i in self.attached]))


def fixture():
    adapter = MoveItEdgeAdapter.__new__(MoveItEdgeAdapter)
    adapter._scene = SceneClient()
    adapter._scene_state_applied = None
    adapter._scene_world_ids_added = []
    adapter._scene_dumps = []
    adapter._call_counts = {'scene_apply': 0}
    adapter._link = 'suction_contact_frame'
    placed = BoxSpec('confirmed-previous-box', (1., 0., 0.), (.3, .2, .2))
    released = BoxSpec('route-released-box', (2., 0., 0.), (.3, .2, .2))
    payload = PayloadSpec((.3, .2, .2), (0., 0., .1), (0., 0., 0., 1.))
    carrying = SceneState('carrying', payload=payload, placed_boxes=(placed,))
    unloaded = SceneState('unloaded', released_box=released, placed_boxes=(placed,))
    return adapter, carrying, unloaded


def test_rebuild_removes_only_obsolete_owned_world_geometry():
    adapter, carrying, unloaded = fixture()
    for _ in range(2):
        adapter._ensure_scene(carrying)
        assert adapter._scene.world == {'external-obstacle', 'confirmed-previous-box'}
        assert adapter._scene.attached == {'b-payload'}
        adapter._ensure_scene(unloaded)
        assert 'route-released-box' in adapter._scene.world
        assert not adapter._scene.attached
    adapter._ensure_scene(carrying)
    assert adapter._scene.world == {'external-obstacle', 'confirmed-previous-box'}
    assert adapter._scene_world_ids_added == ['confirmed-previous-box']
    assert adapter._scene.removed == ['route-released-box', 'route-released-box']


def test_failed_removal_does_not_admit_carrying_scene_and_can_retry():
    adapter, carrying, unloaded = fixture()
    adapter._ensure_scene(carrying)
    adapter._ensure_scene(unloaded)
    previous = adapter._scene_state_applied
    adapter._scene.refuse_remove = True
    with pytest.raises(RuntimeError, match='scene remove failed'):
        adapter._ensure_scene(carrying)
    assert adapter._scene_state_applied == previous
    assert 'route-released-box' in adapter._scene_world_ids_added
    assert not adapter._scene.attached
    adapter._scene.refuse_remove = False
    adapter._ensure_scene(carrying)
    assert 'route-released-box' not in adapter._scene.world
    assert adapter._scene.attached == {'b-payload'}
