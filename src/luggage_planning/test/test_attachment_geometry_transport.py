"""Real ROS message roundtrip and stale/tampered geometry rejection."""
import math
import pytest
from luggage_msgs.msg import AttachmentGeometryV2
from st09_contracts._base import ContractViolation
from st09_contracts.attachment_geometry import freeze_geometry
from st09_contracts.payload_envelope import EnvelopeLimits
from luggage_planning.attachment_geometry_transport import geometry_from_message, geometry_to_message


def sample():
    return freeze_geometry((.5,.3,.2), (.08,-.03,-.1), (0,0,0,1),
                           EnvelopeLimits(5, (.1,.1,.1), (0,0), 0))


@pytest.mark.parametrize('tilt', [0., .02])
def test_real_wire_roundtrip_preserves_digest_without_expansion(tilt):
    from rclpy.serialization import serialize_message, deserialize_message
    original = freeze_geometry((.5,.3,.2), (.08,-.03,-.1),
        (math.sin(tilt),0.,0.,math.cos(tilt)),
        EnvelopeLimits(5,(.1,.1,.1),(0.,0.),.005))
    wire = geometry_to_message(original)
    restored = geometry_from_message(deserialize_message(serialize_message(wire), AttachmentGeometryV2))
    assert restored == original
    assert restored.digest() == original.digest()


def test_missing_geometry_refused():
    with pytest.raises(ContractViolation, match='GEOMETRY_MISSING'):
        geometry_from_message(AttachmentGeometryV2())


@pytest.mark.parametrize('field', ['conservative_dimensions_wdh', 'tool_box_translation'])
def test_changed_geometry_refused(field):
    wire = geometry_to_message(sample())
    value = list(getattr(wire, field))
    value[0] += .01
    setattr(wire, field, value)
    with pytest.raises(ContractViolation, match='MISMATCH'):
        geometry_from_message(wire)


def test_capacity_adapter_uses_conservative_dimensions_and_preserves_nominal_input():
    from luggage_msgs.srv import ComputePlacementV2
    from luggage_planning.typed_driver_adapter import TypedDriverAdapter
    geometry = freeze_geometry((.5,.3,.2), (.08,-.03,.1), (1,0,0,0),
                               EnvelopeLimits(0, (.1,.1,.1), (0,0), .005))
    adapter = object.__new__(TypedDriverAdapter)
    adapter._compute = object()
    response = ComputePlacementV2.Response()
    response.success = True
    response.slot.pose.orientation.w = 1.
    response.slot.pose.position.z = geometry.conservative.dimensions_wdh[2]/2
    response.candidates = [response.slot]
    calls = []
    adapter._call = lambda client, request, timeout: (calls.append(request) or response)
    result = adapter.compute_placement(request_id='test', box_wdh=(.5,.3,.2),
        height_measured=True, scene_revision='scene', conservative_geometry=geometry)
    assert list(calls[0].box_wdh) == pytest.approx((.55,.33,.225))
    # Solver envelope bottom at floor; nominal bottom has the known clearance.
    assert result.slot.pose.position.z == pytest.approx(.125)
    assert result.slot.pose.orientation.w == pytest.approx(1.)
    assert response.slot.pose.position.z == pytest.approx(.1125)
    assert geometry.measured_dimensions_wdh == (.5,.3,.2)
