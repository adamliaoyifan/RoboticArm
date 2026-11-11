"""Image-to-plane support: whole cells, holes, frame identity and projection."""
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_perception.suction_top_domain import (
    build_top_domain, project_supported_cells,
)
from luggage_perception.suction_footprint_containment import check_footprint_containment


def geometry(**changes):
    values = dict(origin_uv=(-.205, -.205), shape=(41, 41), cell_size_m=.01,
                  plane_basis=np.eye(3), plane_offset=-1.,
                  intrinsics=SimpleNamespace(fx=100., fy=100., cx=20., cy=20.),
                  optical_to_world=np.eye(4))
    values.update(changes)
    return values


def test_complete_face_and_pixel_hole_propagate_to_footprint():
    image = np.ones((41, 41), dtype=bool)
    full = project_supported_cells(image, **geometry())
    assert full.all()
    image[20, 20] = False
    domain = project_supported_cells(image, **geometry())
    assert not domain[20, 20]
    assert domain[5, 5]
    args = dict(origin_uv=(-.205, -.205), cell_size_m=.01,
                center_uv=(0., 0.), yaw_rad=.3,
                footprint_size_xy_m=(.12, .08), boundary_margin_m=.015)
    assert check_footprint_containment(full, **args).contained
    assert not check_footprint_containment(domain, **args).contained


def test_one_invalid_pixel_rejects_cell_even_if_other_pixels_valid():
    image = np.ones((41, 41), dtype=bool)
    image[20, 20] = False
    domain = project_supported_cells(image, **geometry(
        origin_uv=(-.10, -.10), shape=(1, 1), cell_size_m=.20))
    assert not domain[0, 0]  # 440 / 441 pixels valid is insufficient


def test_out_of_image_not_clipped_to_valid_part():
    image = np.ones((41, 41), dtype=bool)
    domain = project_supported_cells(image, **geometry(
        origin_uv=(-.21, -.2), shape=(2, 1)))
    assert not domain[0, 0]
    assert domain[1, 0]


def test_behind_camera_has_no_support():
    domain = project_supported_cells(np.ones((41, 41), bool),
                                     **geometry(plane_offset=1.))
    assert not domain.any()


def test_rigid_world_transform_does_not_change_supported_cells():
    image = np.ones((41, 41), dtype=bool)
    image[10:15, 18:22] = False
    angle = .4
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    # Stay away from exact pixel boundaries in this numerical invariant test.
    args = geometry(origin_uv=(-.18, -.18), shape=(35, 35))
    reference = project_supported_cells(image, **args)
    args.update(plane_basis=rotation.T, optical_to_world=transform)
    assert np.array_equal(reference, project_supported_cells(image, **args))


@pytest.mark.parametrize("changes", [
    {"cell_size_m": 0.}, {"plane_offset": float("nan")},
    {"plane_basis": np.ones((3, 3))}, {"shape": (1.2, 2)},
    {"optical_to_world": np.zeros((4, 4))},
    {"intrinsics": SimpleNamespace(fx=0., fy=100., cx=20., cy=20.)},
])
def test_invalid_geometry_never_admitted(changes):
    with pytest.raises(ValueError):
        project_supported_cells(np.ones((41, 41), bool), **geometry(**changes))


def snapshot():
    image = np.ones((41, 41), dtype=bool)
    hmap = SimpleNamespace(origin_u=-.205, origin_v=-.205,
                           shape=(41, 41), cell_size_m=.01,
                           u_axis=np.eye(3)[0], v_axis=np.eye(3)[1],
                           normal=np.eye(3)[2], plane_d=-1.)
    top = SimpleNamespace(stamp=12., frame="world", reason="ok",
                          plane_labels=np.ones(image.shape, np.uint8),
                          winner_plane_id=1, winning_patch_mask=image.copy())
    return dict(hmap=hmap, top=top, depth_mm=np.full(image.shape, 1000.),
                instance_region=image, intrinsics=geometry()["intrinsics"],
                optical_to_world=np.eye(4), stamp=12., frame_id="world")


@pytest.mark.parametrize("source", ["instance", "patch", "label", "depth"])
def test_all_four_sources_must_admit_pixel(source):
    args = snapshot()
    if source == "instance":
        args["instance_region"][20, 20] = False
    elif source == "patch":
        args["top"].winning_patch_mask[20, 20] = False
    elif source == "label":
        args["top"].plane_labels[20, 20] = 2
    else:
        args["depth_mm"][20, 20] = 0
    domain = build_top_domain(**args)
    assert not domain.valid_cells[20, 20]
    assert domain.valid_cells[5, 5]
    assert not domain.valid_cells.flags.writeable
    assert domain.stamp == 12. and domain.frame_id == "world"


@pytest.mark.parametrize("changes", [
    {"stamp": 13.}, {"frame_id": "camera"}, {"instance_region": None},
])
def test_stale_or_missing_observation_rejected(changes):
    args = snapshot()
    args.update(changes)
    with pytest.raises(ValueError):
        build_top_domain(**args)


def test_connected_patch_required_no_rectangle_fallback():
    args = snapshot()
    args["top"].winning_patch_mask = None
    with pytest.raises(ValueError):
        build_top_domain(**args)


def test_observed_planar_box_produces_connected_domain_for_full_suction():
    from luggage_perception.dynamic_top_surface import estimate_dynamic_top_surface
    from luggage_perception.instance_depth_component import isolate_depth_component
    from luggage_perception.suction_height_map import build_height_map
    from luggage_perception.eval.dynamic_suction_renderer import (
        NadirRenderer, DEFAULT_CAMERA, box_polygon, pixel_bbox,
    )
    renderer = NadirRenderer()
    poly = box_polygon((.55, .40), (-1., 0.), 0.)
    z = DEFAULT_CAMERA["platform_z"] + .25
    depth = renderer.render([(z, poly)], noise_sigma_mm=0., missing_frac=0.,
                            outlier_frac=0., seed=0)
    bbox = pixel_bbox(renderer, poly, z, expand_frac=.05)
    component = isolate_depth_component(depth, bbox=bbox)
    assert component.ok
    region = np.zeros(depth.shape, bool)
    x0, y0, x1, y1 = bbox
    region[y0:y1, x0:x1] = True
    intrinsics = SimpleNamespace(**{k: DEFAULT_CAMERA[k]
                                   for k in ("fx", "fy", "cx", "cy")})
    transform = np.asarray(DEFAULT_CAMERA["optical_to_world"])
    top = estimate_dynamic_top_surface(depth, component.mask, intrinsics,
                                      transform, stamp=12., frame="world",
                                      instance_region=region)
    assert top.reason == "ok"
    hmap = build_height_map(depth, intrinsics, transform, top,
                           instance_region=region)
    domain = build_top_domain(hmap, top, depth, region, intrinsics, transform,
                              stamp=12., frame_id="world")
    assert domain.valid_cells.any()
    assert check_footprint_containment(
        domain.valid_cells, origin_uv=domain.origin_uv,
        cell_size_m=domain.cell_size_m, center_uv=(-1., 0.), yaw_rad=0.,
        footprint_size_xy_m=(.18, .18), boundary_margin_m=.015).contained
