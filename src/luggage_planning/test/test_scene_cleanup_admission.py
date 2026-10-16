"""Cleanup refusal must remain retryable and must revoke Select success."""
import json
from types import SimpleNamespace

import pytest

from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter


@pytest.mark.parametrize('raises', [False, True])
def test_partial_cleanup_retains_failed_objects_for_retry(raises):
    calls = []
    reject = [True]

    def remove(name):
        calls.append(name)
        if name == 'blocked' and reject[0]:
            if raises:
                raise RuntimeError('transport failure')
            return False, 'rejected'
        return True, 'removed'

    adapter = object.__new__(MoveItEdgeAdapter)
    adapter._scene = SimpleNamespace(detach_and_remove=lambda **kw: remove(kw['box_id']),
                                    remove_object=remove)
    adapter._scene_state_applied = 'cached'
    adapter._scene_world_ids_added = ['blocked', 'removable']
    adapter._failure_log = []
    with pytest.raises(RuntimeError, match='scene cleanup failed'):
        adapter.reset_scene()
    assert calls == ['b-payload', 'blocked', 'removable']
    assert adapter._scene_world_ids_added == ['blocked']
    assert adapter._scene_state_applied is None
    assert adapter._failure_log[-1]['reason'] == 'scene_cleanup_failed'
    reject[0] = False
    adapter.reset_scene()
    assert adapter._scene_world_ids_added == []


@pytest.mark.parametrize('planning_raises', [False, True])
def test_cleanup_failure_never_emits_executable_route(monkeypatch, planning_raises):
    from luggage_planning import place_service, route_producer
    monkeypatch.setattr(place_service, 'request_dict', lambda _: {'box_wdh': [0.5, 0.3, 0.2]})
    monkeypatch.setattr(route_producer, 'register_attach_from_request', lambda *a, **kw: None)

    def plan(request):
        if planning_raises:
            raise RuntimeError('planning failed')
        return {'success': True, 'rows': [{'reason': 'OK'}], 'stage_timings': {'total_ms': 12}}

    def reset():
        raise RuntimeError('remove rejected')

    producer = SimpleNamespace(attachment_provider=object(), plan_place_route=plan,
                              edge_builder=SimpleNamespace(reset_scene=reset))
    from luggage_msgs.srv import SelectPlacePathV2
    response = place_service.fill_select_response_route(None, SelectPlacePathV2.Response(), producer)
    assert not response.success
    assert response.reason_code == 'SCENE_CLEANUP_FAILED'
    assert not response.motion_segments and not response.carry_segments
    row = json.loads(response.rows_json)[-1]
    assert row['detail'] == 'remove rejected'
    assert bool(row['planning_error']) == planning_raises


def test_scene_digest_is_stable_and_covers_real_field_changes():
    from moveit_msgs.msg import PlanningScene
    from luggage_planning.moveit_state_validity import _scene_digest
    from luggage_planning.planning_scene_client import build_collision_object
    scene = PlanningScene()
    scene.world.collision_objects = [build_collision_object(
        'box', [0., 0., 1.], [0., 0., 0., 1.], [0.5, 0.3, 0.2], 'world', 0)]
    original = _scene_digest([scene])
    assert {_scene_digest([scene]) for _ in range(100)} == {original}
    scene.world.collision_objects[0].primitive_poses[0].position.x = 1e-12
    assert _scene_digest([scene]) != original
