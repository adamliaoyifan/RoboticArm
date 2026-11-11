"""Task-directed surface queries over exact measurement-bound footprints."""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path

import numpy as np
import pytest

from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.observed_suction_domain import ObservedSuctionDomain
from luggage_perception.suction_height_map import HeightMap
from test_suction_patch_evaluator import make_model


def context_inputs():
    shape = (140, 90)
    domain = ObservedSuctionDomain('box', 'instance', 3, 12_000_000_000, 'world', 1,
        'synthetic-physical-model', (0., 0., .85), (0., 0., 0., 1.), (.7, .45, .3),
        (1., 0., 0.), (0., 1., 0.), (0., 0., 1.), -1., (-.35, -.225),
        .005, shape, bytes([1])*np.prod(shape))
    model = make_model(.12, footprint_size_xy_m=[.28, .12])
    model.identity_hash = domain.contact_model_hash
    hmap = HeightMap(.005, -.35, -.225, *shape,
        np.ones(shape, np.int32), np.zeros(shape), np.zeros(shape),
        np.ones(shape, np.uint8), np.ones(shape, bool),
        np.broadcast_to([0., 0., 1.], shape+(3,)).copy(),
        np.array([1., 0., 0.]), np.array([0., 1., 0.]), np.array([0., 0., 1.]), -1.,
        np.array([], int), np.array([0], int), np.array([], int),
        np.array([], float), np.empty((0, 2)))
    return domain, hmap, model


@pytest.fixture
def query(request):
    """Optional durable stage-boundary inputs/outputs, not internal dumps."""
    counter = 0

    def run(domain, hmap, model, uv=(0., 0.), yaw=0., **kwargs):
        nonlocal counter
        root = os.environ.get('JOINT_DIRECTED_QUALITY_EVIDENCE', '')
        output = None
        if root:
            output = Path(root)/request.node.name/str(counter)
            output.mkdir(parents=True, exist_ok=False)
            np.savez_compressed(output/'boundary_input.npz', support=domain.cells(),
                valid_count=hmap.valid_count, median_h=hmap.median_h, smooth_h=hmap.smooth_h,
                plane_label=hmap.plane_label, mask_member=hmap.mask_member,
                local_normal=hmap.local_normal)
            metadata = asdict(domain); metadata.pop('support')
            (output/'input.json').write_text(json.dumps(dict(domain=metadata,
                policy=model.as_dict(), uv=uv, yaw=yaw, fixture='synthetic measured plane',
                kwargs={key:value for key,value in kwargs.items() if key != 'clock'}), indent=2))
        counter += 1
        context = DirectedSuctionContext(domain, hmap, model)
        try:
            result = context.evaluate(uv, yaw, deadline=kwargs.pop('deadline', 10.),
                max_refinements=kwargs.pop('max_refinements', 3),
                clock=kwargs.pop('clock', lambda:0.), **kwargs)
        except Exception as error:
            if output:
                (output/'failure_output.json').write_text(json.dumps(dict(
                    exception=type(error).__name__, message=str(error)), indent=2))
            raise
        if output:
            (output/'output.json').write_text(json.dumps(asdict(result), indent=2))
            pose = np.array(result.world_from_contact_frame).reshape(4, 4) if result.accepted else None
            if pose is not None:
                window = context._window(pose)
                np.savez_compressed(output/'quality_footprint.npz', first_ij=window.first_ij,
                    membership=window.membership, world_from_contact_frame=pose)
        return result
    return run


@pytest.mark.parametrize('yaw', [0., np.pi/2, np.pi, -np.pi/2, np.pi/4, .17])
def test_planar_query_accepts_directed_angle_and_preserves_exact_subcell_uv(query, yaw):
    domain, hmap, model = context_inputs()
    uv = (.0013, -.0027)
    result = query(domain, hmap, model, uv=uv, yaw=yaw)
    assert result.accepted, result.reason
    pose = np.array(result.world_from_contact_frame).reshape(4, 4)
    assert pose[:2, 3] == pytest.approx(uv, abs=1e-15)
    assert pose[:3, 3] == pytest.approx([*uv, 1.], abs=1e-15)
    assert pose[:3, 0] == pytest.approx([np.cos(yaw), np.sin(yaw), 0.], abs=1e-14)
    assert pose[:3, 2] == pytest.approx([0., 0., -1.], abs=1e-14)
    assert np.linalg.det(pose[:3, :3]) == pytest.approx(1.)
    assert dict(result.metrics)['rms_residual'] < 1e-14
    assert dict(result.metrics)['valid_coverage'] == 1.
    assert result.evidence_sha256 and result.samples_sha256


def test_real_rotation_selects_quality_region_instead_of_reusing_old_score(query):
    domain, hmap, model = context_inputs()
    u = hmap.origin_u+(np.arange(hmap.nu)+.5)*hmap.cell_size_m
    hmap.smooth_h[u > .08, :] = .012
    long_axis_over_step = query(domain, hmap, model)
    rotated_clear = query(domain, hmap, model, yaw=np.pi/2)
    assert not long_axis_over_step.accepted
    assert rotated_clear.accepted, rotated_clear.reason


def test_rotated_quality_does_not_include_aabb_corners_outside_actual_rectangle(query):
    domain, hmap, model = context_inputs()
    u = hmap.origin_u+(np.arange(hmap.nu)+.5)*hmap.cell_size_m
    v = hmap.origin_v+(np.arange(hmap.nv)+.5)*hmap.cell_size_m
    defect = (np.abs(u[:, None]-.085) < .015) & (np.abs(v[None, :]+.085) < .015)
    hmap.smooth_h[defect] = .02
    clear = query(domain, hmap, model, yaw=np.pi/4)
    blocked = query(domain, hmap, model, yaw=-np.pi/4)
    assert clear.accepted, clear.reason
    assert not blocked.accepted


def test_unknown_single_cell_refuses_despite_passing_quality_plane(query):
    domain, hmap, model = context_inputs()
    support = domain.cells().copy(); support[70, 45] = False
    result = query(replace(domain, support=support.tobytes()), hmap, model)
    assert not result.accepted and result.reason == 'FOOTPRINT_NOT_FULLY_CONTAINED'
    assert not result.world_from_contact_frame


def test_effective_policy_overrides_change_identity_and_original_mutation_cannot_change_context():
    domain, hmap, model = context_inputs()
    context = DirectedSuctionContext(domain, hmap, model)
    original = context.evaluate((0., 0.), 0., deadline=1., max_refinements=3, clock=lambda:0.)
    model.max_normal_deviation_p95_deg = 15.
    overridden = DirectedSuctionContext(domain, hmap, model)
    assert overridden.policy_sha256 != context.policy_sha256
    assert overridden.domain.contact_model_hash == context.domain.contact_model_hash
    hmap.smooth_h[:] = .1
    unchanged = context.evaluate((0., 0.), 0., deadline=1., max_refinements=3, clock=lambda:0.)
    assert unchanged == original
    with pytest.raises(ValueError): context._height_map.smooth_h.setflags(write=True)


@pytest.mark.parametrize('damage', ['grid', 'basis', 'model', 'offset'])
def test_context_rejects_mismatched_source_grid_or_model(damage):
    domain, hmap, model = context_inputs()
    if damage == 'grid': hmap.origin_u += .001
    elif damage == 'basis': hmap.u_axis = -hmap.u_axis
    elif damage == 'model': model.identity_hash = 'other'
    else: hmap.plane_d = 0.
    with pytest.raises(ValueError): DirectedSuctionContext(domain, hmap, model)


def test_deadline_before_or_after_quality_computation_never_returns_permission(query):
    args = context_inputs()
    before = query(*args, deadline=0.)
    ticks = iter([0., 11.])
    after = query(*args, clock=lambda:next(ticks))
    assert before.reason == after.reason == 'QUALITY_DEADLINE'
    assert not before.accepted and not after.accepted
    assert not after.world_from_contact_frame


def test_bounded_refinement_does_not_misreport_incomplete_fit_as_physical_unreachable(query):
    domain, hmap, model = context_inputs()
    # A tilted plane changes the physical projected footprint after the first
    # fit. The one-pass compute cap must not mint a quality permission.
    u = hmap.origin_u+(np.arange(hmap.nu)+.5)*hmap.cell_size_m
    slope = np.tan(np.deg2rad(7.))
    hmap.smooth_h[:] = slope*u[:, None]
    normal = np.array([-slope, 0., 1.]); normal /= np.linalg.norm(normal)
    hmap.local_normal[:] = normal
    incomplete = query(domain, hmap, model, uv=(.001, .001), yaw=.19, max_refinements=1)
    complete = query(domain, hmap, model, uv=(.001, .001), yaw=.19, max_refinements=3)
    assert incomplete.reason == 'QUALITY_REFINEMENT_INCOMPLETE'
    assert not incomplete.accepted and not incomplete.world_from_contact_frame
    assert complete.accepted, complete.reason
    assert complete.refinement_count > 1


def test_no_sample_coverage_rejects_instead_of_ignoring_existing_coverage_gate(query):
    domain, hmap, model = context_inputs()
    hmap.valid_count[:] = 0
    result = query(domain, hmap, model)
    assert not result.accepted and result.reason == 'SUCTION_REJECT_VALID_CELLS'


def test_percentile_empty_rows_remain_undefined_without_indexing_outside_array():
    from luggage_perception.suction_patch_evaluator import _sorted_pct
    rows = np.array([[-np.inf, -np.inf, -np.inf], [-np.inf, 2., 4.]])
    values = _sorted_pct(rows, np.array([0, 2]), .5, offsets=np.array([3, 1]))
    assert np.isnan(values[0]) and values[1] == 3.


def test_evaluator_factory_binds_actual_depth_acquisition_and_rejects_stale_replacement(query):
    from types import SimpleNamespace
    from luggage_msgs.msg import DetectedLuggage
    from luggage_perception.dynamic_top_surface import estimate_dynamic_top_surface
    from luggage_perception.instance_depth_component import isolate_depth_component
    from luggage_perception.suction_patch_evaluator import SuctionPatchEvaluator
    from luggage_perception.eval.dynamic_suction_renderer import (
        NadirRenderer, DEFAULT_CAMERA, box_polygon, pixel_bbox,
    )
    renderer = NadirRenderer()
    polygon = box_polygon((.55, .40), (-1., 0.), 0.)
    z = DEFAULT_CAMERA['platform_z']+.25
    depth = renderer.render([(z, polygon)], noise_sigma_mm=0., missing_frac=0.,
        outlier_frac=0., seed=0)
    bbox = pixel_bbox(renderer, polygon, z, expand_frac=.05)
    component = isolate_depth_component(depth, bbox=bbox)
    assert component.ok
    region = np.zeros(depth.shape, bool)
    x0, y0, x1, y1 = bbox; region[y0:y1, x0:x1] = True
    intrinsics = SimpleNamespace(**{k:DEFAULT_CAMERA[k] for k in ('fx', 'fy', 'cx', 'cy')})
    transform = np.asarray(DEFAULT_CAMERA['optical_to_world'])
    top = estimate_dynamic_top_surface(depth, component.mask, intrinsics, transform,
        stamp=12., frame='world', instance_region=region)
    assert top.reason == 'ok'
    model = make_model(.18); model.identity_hash = 'synthetic-physical-model'
    evaluator = SuctionPatchEvaluator(model)
    evaluator.update(depth, intrinsics, transform, top, instance_region=region,
        stamp=12., frame_id='world', instance_id='instance', generation=3,
        capture_observed_domain=True)
    detection = DetectedLuggage()
    detection.id = 'box'; detection.header.frame_id = 'world'; detection.header.stamp.sec = 12
    detection.width, detection.depth, detection.height = .55, .40, .25
    detection.pose.position.x, detection.pose.position.z = -1., z-.125
    detection.pose.orientation.w = 1.
    detection.top_surface_valid = detection.height_valid = True
    detection.height_source = detection.HEIGHT_SOURCE_MEASURED_SUPPORT
    context = evaluator.directed_quality_context(detection)
    uv = evaluator.copy_output().accepted[0].center_uv
    capture = os.environ.get('JOINT_DIRECTED_QUALITY_EVIDENCE', '')
    if capture:
        output = Path(capture)/'factory_acquisition'
        output.mkdir(parents=True, exist_ok=False)
        np.savez_compressed(output/'raw_acquisition.npz', depth_mm=depth,
            instance_region=region, optical_to_world=transform, plane_labels=top.plane_labels,
            winning_patch_mask=top.winning_patch_mask)
        (output/'input.json').write_text(json.dumps(dict(
            intrinsics=intrinsics.__dict__, bbox=bbox, stamp=12., frame='world',
            instance_id='instance', generation=3, legacy_first_uv=uv), indent=2))
    result = context.evaluate(uv, .37, deadline=1., max_refinements=3, clock=lambda:0.)
    assert query(context.domain, context._height_map, model, uv=uv, yaw=.37) == result
    # Legacy rank/acceptance does not authorize an independently rotated pose.
    assert not result.accepted and result.reason == 'FOOTPRINT_OUTSIDE_TOP_GRID'
    uv = tuple(np.asarray(context.domain.origin_uv_top_m)
        + np.asarray(context.domain.shape)*context.domain.cell_size_m/2)
    result = context.evaluate(uv, .37, deadline=1., max_refinements=3, clock=lambda:0.)
    assert query(context.domain, context._height_map, model, uv=uv, yaw=.37) == result
    assert result.accepted, result.reason
    assert context.domain.stamp_ns == 12_000_000_000
    detection.header.stamp.sec = 13
    with pytest.raises(ValueError): evaluator.directed_quality_context(detection)
    evaluator.update(depth, intrinsics, transform, None, stamp=13., frame_id='world',
        instance_id='instance', generation=4, capture_observed_domain=True)
    with pytest.raises(ValueError): evaluator.directed_quality_context(detection)
    # A pinned context remains internally coherent; caller freshness admission
    # still decides whether it can be used for a new operation.
    assert context.evaluate(uv, .37, deadline=1., max_refinements=3, clock=lambda:0.) == result
