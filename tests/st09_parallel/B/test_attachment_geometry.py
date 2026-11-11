"""Consumer coordinate semantics for a single attachment-owned geometry."""
from dataclasses import FrozenInstanceError
import math

import pytest

from st09_contracts.attachment_geometry import freeze_geometry
from st09_contracts.payload_envelope import EnvelopeLimits


def test_input_snapshot_and_consumers_never_accumulate_margin():
    measured = [.5, .3, .2]
    geometry = freeze_geometry(measured, [.08, -.03, -.1], [0, 0, 0, 1],
                              EnvelopeLimits(5, (.1, .1, .1), (0, 0), .005))
    digest = geometry.digest()
    measured[0] = 99
    for _ in range(10):
        p, q = geometry.conservative_pose_at_box((1, 2, 3), (0, 0, 0, 1))
        recovered, rotation = geometry.box_pose_at_conservative(p, q)
        assert recovered == pytest.approx((1, 2, 3))
        assert rotation == pytest.approx((0, 0, 0, 1))
        assert geometry.digest() == digest
    assert geometry.measured_dimensions_wdh == (.5, .3, .2)
    with pytest.raises(FrozenInstanceError):
        geometry.measured_dimensions_wdh = (1, 1, 1)


@pytest.mark.parametrize('tool_box_q,tool_box_z', [((0,0,0,1), -.1), ((1,0,0,0), .1)])
def test_capacity_target_maps_back_with_downward_offset(tool_box_q, tool_box_z):
    geometry = freeze_geometry((.5,.3,.2), (.08,-.03,tool_box_z), tool_box_q,
                              EnvelopeLimits(0, (.1,.1,.1), (0,0), .005))
    # Nominal box stays vertical in world regardless of the tool convention.
    box_position, box_rotation = (1.,2.,.1), (0.,0.,0.,1.)
    p, q = geometry.conservative_pose_at_box(box_position, box_rotation)
    assert p[2] == pytest.approx(.0875)
    assert geometry.conservative.dimensions_wdh == pytest.approx((.55,.33,.225))
    restored, _ = geometry.box_pose_at_conservative(p, q)
    assert restored == pytest.approx(box_position)


def test_eccentric_rotated_pose_roundtrip():
    geometry = freeze_geometry((.5,.3,.2), (.18,-.09,-.1),
                              (0,0,math.sin(.3),math.cos(.3)),
                              EnvelopeLimits(5, (.1,.1,.1), (0,0), .005))
    nominal = ((1,2,3), (0,0,math.sin(.7),math.cos(.7)))
    conservative = geometry.conservative_pose_at_box(*nominal)
    restored = geometry.box_pose_at_conservative(*conservative)
    assert restored[0] == pytest.approx(nominal[0])
    assert restored[1] == pytest.approx(nominal[1])


def test_tilted_measured_attachment_keeps_nominal_pose_and_roundtrip():
    from st09_contracts.attachment import transform_point

    tool_box = ((.18, -.09, .1), (math.cos(.02), 0., 0., math.sin(.02)))
    geometry = freeze_geometry((.5, .3, .2), *tool_box,
                              EnvelopeLimits(5, (.1, .1, .1), (.01, .02), .005))
    nominal = ((1., 2., 3.), (0., 0., math.sin(.7), math.cos(.7)))
    conservative = geometry.conservative_pose_at_box(*nominal)
    restored = geometry.box_pose_at_conservative(*conservative)
    assert restored[0] == pytest.approx(nominal[0])
    assert restored[1] == pytest.approx(nominal[1])
    assert geometry.t_tool_box_translation == tool_box[0]
    assert geometry.t_tool_box_quaternion_xyzw == tool_box[1]
    assert abs(conservative[1][0]) + abs(conservative[1][1]) > .001
    # Full envelope orientation is required in world: simply substituting
    # yaw-only nominal orientation would move the represented corner.
    corner = tuple(v / 2 for v in geometry.conservative.dimensions_wdh)
    correct = transform_point(*conservative, corner)
    flattened = transform_point(conservative[0], nominal[1], corner)
    assert math.dist(correct, flattened) > .001
