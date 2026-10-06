"""Synthetic scan grasp uses the production tool-down attachment."""

from __future__ import division

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
for extra in (
        ROOT / "scripts" / "st09_i",
        ROOT / "src" / "luggage_planning",
        ROOT / "src" / "luggage_description",
        ROOT / "st09_contracts",
):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

from scan_grasp import synthetic_top_grasp  # noqa: E402
from st09_contracts.attachment import (  # noqa: E402
    invert_transform,
    quaternion_angle_between,
    rotate_vector,
    transform_point,
)

MEASURED_T = (0.0, 0.0, 0.1236271324575158)
MEASURED_Q = (1.0, 0.0, 0.0, 0.0)
SIZE = [0.55, 0.40, 0.25]
SLOT_POSITION = (0.005, 0.015, 0.6535717432182216)
SLOT_QUAT = (0.0, 0.0, 0.707106771713121, 0.707106790659974)


def _tcp_z(quaternion):
    return rotate_vector(quaternion, (0.0, 0.0, 1.0))


def _face_on_tcp(translation, quaternion, size):
    half = 0.5 * float(size[2])
    return transform_point(translation, quaternion, (0.0, 0.0, half))


def _container():
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_planning.typed_driver_adapter import container_transform_fields

    scene = ROOT / "src" / "luggage_description" / "config" / "scene_tf_deploy.yaml"
    transform = container_transform_fields(load_scene_tf_config(str(scene)))
    return tuple(transform[2]), tuple(transform[3])


def _synthesize(position, quaternion, grasp=None):
    import dataclasses

    from luggage_description.frame_transforms import rotate_vector as rotate_container
    from luggage_planning.attachment_provider import MeasuredAttachmentProvider
    from luggage_planning.place_targets import (
        box_target_from_candidate,
        expand_place_targets,
    )
    from luggage_planning.route_producer import (
        RELEASE_DROP_M,
        candidate_from_slot,
        register_attach_from_request,
    )

    translation, container_q = _container()
    grasp_t, grasp_q = grasp or synthetic_top_grasp(SIZE)
    request = {
        "request_id": "grasp-convention",
        "scene_revision": "grasp-convention",
        "box_wdh": list(SIZE),
        "box_id": "carryon",
        "generation": 1,
        "slots": [{
            "request_id": "slot",
            "position": [float(v) for v in position],
            "quaternion_xyzw": [float(v) for v in quaternion],
        }],
        "grasp": {
            "position": list(grasp_t),
            "quaternion_xyzw": list(grasp_q),
            "stamp_ns": 1_000_000_000,
        },
        "container_transform": {
            "translation": list(translation),
            "quaternion_xyzw": list(container_q),
        },
    }
    provider = MeasuredAttachmentProvider(
        "grasp-convention", "grasp-convention",
        tcp_frame="suction_contact_frame", box_frame="box")
    register_attach_from_request(
        provider, request, box_wdh=SIZE, box_id="carryon", generation=1)
    attachment = provider.latest_record(request["request_id"])
    candidate = candidate_from_slot(request["slots"][0], 0)
    box = box_target_from_candidate(
        candidate, attachment, stamp_ns=attachment.acquisition_stamp_ns,
        t_frame_container=translation, q_frame_container=container_q)
    delta = rotate_container(container_q, (0.0, 0.0, RELEASE_DROP_M))
    box = dataclasses.replace(
        box, position=tuple(box.position[i] + delta[i] for i in range(3)))
    tcp = expand_place_targets(box, attachment)[0]
    return attachment, tcp


def test_nominal_attachment_is_rx_pi_and_half_height():
    grasp = synthetic_top_grasp(SIZE)
    attachment_t, attachment_q = invert_transform(grasp[0], grasp[1])
    assert quaternion_angle_between(attachment_q, MEASURED_Q) < 1e-6
    assert abs(attachment_t[2] - 0.5 * SIZE[2]) < 1e-9
    assert abs(attachment_t[0]) < 1e-9 and abs(attachment_t[1]) < 1e-9


def test_both_placement_yaws_point_the_tool_down_and_hang_the_box():
    for yaw, quat in (
            (0.0, (0.0, 0.0, 0.0, 1.0)),
            (math.pi / 2.0, (0.0, 0.0, 0.7071067811865476,
                             0.7071067811865476))):
        attachment, tcp = _synthesize(SLOT_POSITION, quat)
        tool_z = _tcp_z(tcp.quaternion_xyzw)
        assert tool_z[2] < -0.99, yaw
        contact = _face_on_tcp(
            attachment.t_tcp_box_translation,
            attachment.t_tcp_box_quaternion_xyzw, SIZE)
        assert math.dist(contact, (0.0, 0.0, 0.0)) < 1e-6
        center_world = transform_point(
            tcp.position, tcp.quaternion_xyzw,
            attachment.t_tcp_box_translation)
        assert center_world[2] < tcp.position[2] - 0.05


def test_synthetic_matches_historical_attachment_orientation():
    measured_grasp = invert_transform(MEASURED_T, MEASURED_Q)
    synthetic, synthetic_tcp = _synthesize(SLOT_POSITION, SLOT_QUAT)
    measured, measured_tcp = _synthesize(
        SLOT_POSITION, SLOT_QUAT, measured_grasp)
    assert quaternion_angle_between(
        synthetic.t_tcp_box_quaternion_xyzw,
        measured.t_tcp_box_quaternion_xyzw) < math.radians(1.0)
    assert quaternion_angle_between(
        synthetic_tcp.quaternion_xyzw,
        measured_tcp.quaternion_xyzw) < math.radians(1.0)
    assert math.dist(synthetic_tcp.position, measured_tcp.position) < 0.005
    assert abs(synthetic.t_tcp_box_translation[2]
               - measured.t_tcp_box_translation[2]) < 0.005
