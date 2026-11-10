"""Behavioural checks supplement the continuous extrema construction proof."""
from itertools import product
import math

import pytest
from st09_contracts._base import ContractViolation
from st09_contracts.attachment import transform_point
from st09_contracts.payload_envelope import EnvelopeLimits, build_envelope, sinusoid_bounds
from luggage_planning.attachment_geometry_policy import load_attachment_geometry_policy as load_envelope_limits


def limits(yaw=5, growth=(0., 0., 0.), position_xy=(0., 0.), away_error=0.):
    return EnvelopeLimits(yaw, growth, position_xy, away_error)


def inside(envelope, point):
    return all(abs(p-c) <= d/2+1e-11 for p,c,d in zip(
        point, envelope.t_tcp_box_translation, envelope.dimensions_wdh))


def test_stationary_extremum_not_at_zero_or_endpoints():
    theta = math.radians(2)
    lo, hi = sinusoid_bounds(math.cos(theta), math.sin(theta), -math.radians(5), math.radians(5))
    assert hi == pytest.approx(1, abs=1e-15)
    assert hi > max(math.cos(t-theta) for t in (-math.radians(5), 0, math.radians(5)))


@pytest.mark.parametrize('yaw', [0, 5, 100, 180])
@pytest.mark.parametrize('quaternion', [(0,0,0,1), (1,0,0,0), (0,0,math.sin(.7),math.cos(.7))])
def test_eccentric_continuous_family_crosscheck(yaw, quaternion):
    dimensions, translation = (.5,.3,.2), (.2,-.13,-.15)
    cfg = limits(yaw, (.1,.2,.2), (.1,.08), .06)
    env = build_envelope(dimensions, translation, quaternion, cfg)
    half = (dimensions[0]*.55, dimensions[1]*.6)
    # The translation puts the -box-Z face farther from the suction origin.
    z_values = (-dimensions[2]/2-dimensions[2]*.2, dimensions[2]/2)
    for sx,sy,z_local in product((-1,1),(-1,1),z_values):
        x,y,z = transform_point(translation, quaternion, (sx*half[0],sy*half[1],z_local))
        for step in range(101):
            a = math.radians(-yaw+2*yaw*step/100)
            p = (x*math.cos(a)-y*math.sin(a), x*math.sin(a)+y*math.cos(a),z)
            for ex,ey in product((-1,1),repeat=2):
                # quaternion identity and negative translation => away is tool -Z.
                assert inside(env, (p[0]+ex*.1,p[1]+ey*.08,p[2]-.06))
    assert env.dimensions_wdh[2] == pytest.approx(.30, abs=1e-10)


def test_off_center_pivot_moves_center_and_far_corner():
    env = build_envelope((.2,.2,.2), (.5,0,-.1), (0,0,0,1), limits(90))
    assert env.t_tcp_box_translation[0] > .2
    assert env.dimensions_wdh[1] == pytest.approx(2*math.hypot(.6,.1))


def test_tilt_refused():
    with pytest.raises(ContractViolation, match='UNSUPPORTED_TILT'):
        build_envelope((.5,.3,.2), (0,0,0), (math.sin(.01),0,0,math.cos(.01)), limits())


@pytest.mark.parametrize('quaternion,translation,expected_center,unchanged_face', [
    ((0,0,0,1), (0,0,-.1), -.1125, 0.),
    ((1,0,0,0), (0,0,.1), .1125, 0.),
])
def test_ten_percent_xy_and_one_sided_height_growth(
        quaternion, translation, expected_center, unchanged_face):
    env = build_envelope((.5,.3,.2), translation, quaternion,
                         limits(0,(.1,.1,.1),(0,0),.005))
    assert env.dimensions_wdh == pytest.approx((.55,.33,.225))
    # Near face remains at the suction origin; only the world-down/tool-away
    # face moves, so the box centre shifts by half of 20mm+5mm.
    assert env.t_tcp_box_translation[2] == pytest.approx(expected_center)
    near = (env.t_tcp_box_translation[2] + env.dimensions_wdh[2]/2
            if expected_center < 0 else
            env.t_tcp_box_translation[2] - env.dimensions_wdh[2]/2)
    assert near == pytest.approx(unchanged_face)


def test_ambiguous_contact_face_refused():
    with pytest.raises(ContractViolation, match='CONTACT_FACE_AMBIGUOUS'):
        build_envelope((.5,.3,.2), (0,0,0), (0,0,0,1), limits())
    with pytest.raises(ContractViolation, match='CONTACT_FACE_AMBIGUOUS'):
        build_envelope((.5,.3,.2), (0,0,.02), (0,0,0,1), limits())


@pytest.mark.parametrize('yaw', [-1, 181, float('nan'), True])
def test_invalid_angle(yaw):
    with pytest.raises(ContractViolation):
        limits(yaw)


def test_config_and_identity(tmp_path):
    path = tmp_path/'limits.yaml'
    path.write_text('yaw_half_range_deg: 5\nsize_growth_fraction_wdh: [0.1,0.1,0.1]\nposition_half_range_tool_xy_m: [0,0]\naway_from_contact_position_error_m: 0.005\n')
    cfg = load_envelope_limits(path)
    a = build_envelope((.5,.3,.2), (.02,.03,-.1), (1,0,0,0), cfg)
    b = build_envelope((.5,.3,.2), (.02,.03,-.1), (1,0,0,0),
        limits(6,cfg.size_growth_fraction_wdh,cfg.position_half_range_tool_xy_m,
               cfg.away_from_contact_position_error_m))
    assert a.digest() != b.digest()
    path.write_text('yaw_half_range_deg: 5\n')
    with pytest.raises(ContractViolation, match='CONFIG_INVALID'):
        load_envelope_limits(path)


def envelope_attachment(kind='eccentric'):
    from dataclasses import replace
    from st09_contracts.fixtures import make_attachment
    nominal = make_attachment(kind)
    env = build_envelope(nominal.dimensions_wdh, nominal.t_tcp_box_translation,
        nominal.t_tcp_box_quaternion_xyzw, limits(5,(.1,)*3,(0,0),.005))
    return nominal, replace(nominal, collision_envelope=env)


def test_attachment_binding_and_nominal_target_preserved():
    from dataclasses import replace
    from st09_contracts.attachment import compose_transform
    from luggage_planning.payload_envelope_geometry import geometry_at_box_pose
    nominal, attached = envelope_attachment()
    assert nominal.dimensions_wdh == attached.dimensions_wdh
    assert nominal.t_tcp_box_translation == attached.t_tcp_box_translation
    assert nominal.digest() != attached.digest()
    with pytest.raises(ContractViolation, match='BINDING_MISMATCH'):
        replace(attached, dimensions_wdh=(attached.dimensions_wdh[0] * 1.01,
                                         *attached.dimensions_wdh[1:]))
    placed = geometry_at_box_pose(attached, (1,2,3), (0,0,0,1))
    assert placed[2] == attached.collision_envelope.dimensions_wdh
    assert placed[0][2] == pytest.approx(3-(attached.dimensions_wdh[2]*.1+.005)/2)



def test_scenes_and_graph_identity_share_geometry():
    from types import SimpleNamespace
    from luggage_planning.route_producer import RouteProducer
    from luggage_planning.reachable_graph import GraphRoutePlanner
    _, attachment = envelope_attachment()
    env = attachment.collision_envelope
    producer = object.__new__(RouteProducer)
    candidate = SimpleNamespace(position_container=(0,0,1),quaternion_xyzw_container=(0,0,0,1))
    states = producer.edge_scene_states(None,
        [SimpleNamespace(edge_id='carry',load_state='carrying'),
         SimpleNamespace(edge_id='exit',load_state='unloaded')],
        None, attachment, candidate,
        {'container_transform':{'translation':(0,0,0),'quaternion_xyzw':(0,0,0,1)}})
    payload = states['carry'].payload
    assert payload.dimensions_wdh == env.dimensions_wdh
    assert payload.t_tcp_box_translation == env.t_tcp_box_translation
    assert states['exit'].released_box.size_whd == env.dimensions_wdh
    graph = object.__new__(GraphRoutePlanner)
    identity = graph._payload_identity(attachment)
    assert identity.dimensions_wdh == env.dimensions_wdh
    assert identity.t_tcp_box_translation == env.t_tcp_box_translation


@pytest.mark.parametrize('values', [(True,0), ('0.1',0), (-.1,0), (float('nan'),0), (0,)])
def test_invalid_xy_margin(values):
    with pytest.raises(ContractViolation):
        limits(position_xy=values)


@pytest.mark.parametrize('values', [(True,0,0), ('0.1',0,0), (-.1,0,0),
                                     (float('nan'),0,0), (1.1,0,0), (0,0)])
def test_invalid_growth(values):
    with pytest.raises(ContractViolation):
        limits(growth=values)


@pytest.mark.parametrize('value', [True, '0.1', -.1, float('nan')])
def test_invalid_away_error(value):
    with pytest.raises(ContractViolation):
        limits(away_error=value)


def test_missing_measured_relation_is_never_centered():
    with pytest.raises((ContractViolation, TypeError)):
        build_envelope((.5,.3,.2), None, (0,0,0,1), limits())


def test_certification_scene_uses_exact_offset_and_touch_links():
    from luggage_planning.moveit_state_validity import _attach_payload_scene
    _, attachment = envelope_attachment()
    env = attachment.collision_envelope
    scene = _attach_payload_scene(env)
    attached = scene.robot_state.attached_collision_objects[0]
    assert 'elfin_link2' not in attached.touch_links
    assert tuple(attached.object.primitives[0].dimensions) == env.dimensions_wdh
    pose = attached.object.primitive_poses[0]
    assert (pose.position.x,pose.position.y,pose.position.z) == env.t_tcp_box_translation
