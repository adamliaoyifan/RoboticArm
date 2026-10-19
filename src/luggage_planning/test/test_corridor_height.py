import math
from types import SimpleNamespace
import pytest
from luggage_planning.corridor_height import required_corridor_height


def payload(offset=(0, 0, -.1)):
    return SimpleNamespace(dimensions_wdh=(.4, .2, .2),
                           t_tcp_box_translation=offset,
                           t_tcp_box_quaternion_xyzw=(0, 0, 0, 1))


def obstacle(name, center, size, quat=(0, 0, 0, 1)):
    return SimpleNamespace(object_id=name, center=center, size_whd=size,
                           quaternion_xyzw=quat)


def test_only_obstacles_under_swept_payload_set_height():
    boxes = [obstacle('crossed', (.5, 0, .7), (.1, .1, .4)),
             obstacle('away', (.5, 2, 5), (.1, .1, 1))]
    result = required_corridor_height((0, 0, 1), (1, 0, 1), (0, 0, 0, 1),
                                      payload(), boxes, .05)
    assert result['obstacle_ids'] == ['crossed']
    assert result['top_world_z'] == pytest.approx(.9)
    assert result['required_tcp_world_z'] == pytest.approx(1.15)


def test_grasp_offset_and_rotation_move_swept_footprint():
    boxes = [obstacle('offset_contact', (.5, .35, .7), (.1, .1, .4))]
    assert required_corridor_height((0, 0, 1), (1, 0, 1), (0, 0, 0, 1),
                                    payload(), boxes, .05) is None
    result = required_corridor_height((0, 0, 1), (1, 0, 1),
                                      (0, 0, math.sqrt(.5), math.sqrt(.5)),
                                      payload((.3, 0, -.1)), boxes, .05)
    assert result['obstacle_ids'] == ['offset_contact']


def test_empty_scene_needs_no_raise():
    assert required_corridor_height((0, 0, 1), (1, 0, 1), (0, 0, 0, 1),
                                    payload(), [], .05) is None
