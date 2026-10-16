"""A rejected scene update must stop before edge planning or scene verification."""
from types import SimpleNamespace

import pytest

from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter


@pytest.mark.parametrize('failure', ['remove', 'add', 'live_detach', 'attach', 'unloaded_detach'])
def test_scene_rejection_stops_at_failed_operation(failure):
    calls = []

    def operation(name):
        def invoke(*args, **kwargs):
            calls.append(name)
            return (False, 'injected rejection') if name == failure else (True, 'applied')
        return invoke

    adapter = object.__new__(MoveItEdgeAdapter)
    adapter._scene = SimpleNamespace(
        remove_object=operation('remove'), add_collision_box=operation('add'),
        detach_and_remove=operation('unloaded_detach' if failure == 'unloaded_detach' else 'live_detach'),
        apply_diff=operation('attach'))
    adapter._link = 'suction_contact_frame'
    adapter._scene_state_applied = 'previous' if failure in ('remove', 'unloaded_detach') else None
    adapter._scene_world_ids_added = ['obsolete'] if failure == 'remove' else []
    adapter._call_counts = {'scene_apply': 0}
    adapter._scene_dumps = []
    adapter._verify_scene = lambda *args: calls.append('verify')
    box = SimpleNamespace(object_id='occupied', center=(0., 0., 0.),
                          quaternion_xyzw=(0., 0., 0., 1.), size_whd=(1., 1., 1.))
    payload = SimpleNamespace(t_tcp_box_translation=(0., 0., 0.1),
        t_tcp_box_quaternion_xyzw=(0., 0., 0., 1.), dimensions_wdh=(0.5, 0.3, 0.2))
    scene = SimpleNamespace(load_state='unloaded' if failure == 'unloaded_detach' else 'carrying',
                            payload=payload, released_box=None, placed_boxes=(box,))
    before = adapter._scene_state_applied
    with pytest.raises(RuntimeError, match='injected rejection'):
        adapter._ensure_scene(scene)
    assert calls[-1] == failure
    assert 'verify' not in calls
    assert adapter._scene_state_applied == before
    assert adapter._scene_dumps == []


def test_scene_rejection_is_named_and_never_calls_motion_planner():
    from luggage_planning.grid_path_search import RoutePlanningError
    adapter = object.__new__(MoveItEdgeAdapter)
    adapter._failure_log = []
    calls = []

    def reject(scene):
        raise RuntimeError('scene attach failed: injected rejection')

    adapter._ensure_scene = reject
    adapter._plan_joint = lambda *args: calls.append('motion')
    with pytest.raises(RoutePlanningError):
        adapter.build_edge(None, None, 'PORTAL_TRANSIT', scene=object())
    assert calls == []
    assert adapter._failure_log[-1]['reason'] == 'scene_apply_error'
    assert 'injected rejection' in adapter._failure_log[-1]['error']
