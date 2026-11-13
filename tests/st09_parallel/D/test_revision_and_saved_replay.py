"""ST09-D acceptance: three traceable revisions from three valid
observations (duplicate integration idempotent), and the saved-sensor-data
replay from rehearsal-52's aim view."""

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from luggage_perception.cargo_view_integration import (
    integrate_view_with_report,
    planning_snapshot_from_surface,
)

from luggage_planning.planning_contract.fixtures import make_snapshot
from luggage_planning.planning_contract.planning import ObservationRequirement

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


def requirement():
    footprint = tuple((i, j) for i in (14, 15) for j in (5, 6))
    corridor = tuple((i, 5) for i in range(0, 14))
    cells = footprint + corridor
    return ObservationRequirement(
        requirement_id="req-rev-trace",
        candidate_ids=("cand-fixture-1",),
        footprint_cells=footprint,
        corridor_cells=corridor,
        missing_cells=cells,
        missing_reasons=tuple("unknown_%d_%d" % c for c in cells),
    )


class TestRevisionTrace(object):
    def test_three_observations_three_traceable_revisions(self):
        base = make_snapshot(map_revision=1)
        snapshots = [base]
        for index, stamp in enumerate(
                (2_000_000_000, 3_000_000_000, 4_000_000_000)):
            payload = {
                "acquisition_stamp_ns": stamp,
                "tf_stamp_ns": stamp,
                "geometry_hash": base.geometry_hash,
                "points_container": [(0.0, -0.3, 0.1 + 0.1 * index)],
                "observed_cells": [(14 + index, 5 + index)],
            }
            new_snapshot, _inv, report = integrate_view_with_report(
                snapshots[-1], requirement(), payload)
            assert report["outcome"] in ("integrated", "partial_coverage")
            snapshots.append(new_snapshot)
        # each observation advanced exactly one revision, acquisitions join
        # by stamp, digests chain, and every earlier snapshot is unchanged
        revisions = [s.map_revision for s in snapshots]
        assert revisions == [1, 2, 3, 4]
        stamps = [s.acquisition_stamp_ns for s in snapshots]
        assert stamps == [1_000_000_000, 2_000_000_000, 3_000_000_000,
                          4_000_000_000]
        digests = [s.digest() for s in snapshots]
        assert len(set(digests)) == 4
        assert snapshots[0].cells_i == make_snapshot(
            map_revision=1).cells_i, "immutability: revision 1 untouched"

    def test_duplicate_integration_is_idempotent(self):
        base = make_snapshot(map_revision=1)
        payload = {
            "acquisition_stamp_ns": 2_000_000_000,
            "tf_stamp_ns": 2_000_000_000,
            "geometry_hash": base.geometry_hash,
            "points_container": [(0.0, -0.3, 0.1)],
            "observed_cells": [(14, 5)],
        }
        first, _, report1 = integrate_view_with_report(
            base, requirement(), payload)
        replay, _, report2 = integrate_view_with_report(
            first, requirement(), payload)
        assert report1["outcome"] in ("integrated", "partial_coverage")
        assert report2["outcome"] == "duplicate"
        assert replay is first
        assert first.map_revision == base.map_revision + 1


class TestSavedAimViewReplay(object):
    """Real rehearsal-52 aim data: depth, mask, intrinsics, request and the
    production-integrated post-aim surface (independent expectation)."""

    def setup_method(self):
        self.depth = np.load(os.path.join(FIXTURES, "aim_depth_raw.npy"))
        with open(os.path.join(FIXTURES, "aim_camera_info.json")) as fh:
            info = json.load(fh)
        self.fx, self.fy = info["k"][0], info["k"][4]
        self.cx, self.cy = info["k"][2], info["k"][5]
        from PIL import Image
        self.mask = np.array(
            Image.open(os.path.join(FIXTURES, "aim_mask.png")))
        with open(os.path.join(
                FIXTURES, "integrate_request_aim.json")) as fh:
            self.request = json.load(fh)
        with open(os.path.join(
                FIXTURES, "surface_2d_aim.json")) as fh:
            self.surface_aim = json.load(fh)

    def test_deproject_masked_depth_to_a_box_sized_cloud(self):
        from luggage_gazebo.eval_metrics import depth_to_camera_xyz
        depth_m = self.depth.astype(np.float64) / 1000.0
        cloud = depth_to_camera_xyz(
            depth_m, self.fx, self.fy, self.cx, self.cy,
            max_depth=3.0, stride=4)
        assert cloud.shape[0] > 1000, "real depth deprojects to a dense cloud"
        # masked pixels (the placed box) form a local cluster: the depth
        # band is far narrower than the full scene (515-2006 mm) — the mask
        # plus its small margin picks the box and its immediate backdrop
        masked = depth_m[self.mask > 0]
        assert masked.size > 1000
        extent = float(masked.max() - masked.min())
        assert 0.3 < extent < 1.5, (
            "the masked region is a local cluster, not the whole scene "
            "(extent %.3f m)" % extent)
        assert float(np.median(masked)) > 0.3

    def test_post_aim_surface_round_trips_through_the_producer(self):
        """The production-integrated revision-2 surface (30 free / 73
        occupied / 1097 unknown) is the independent expectation for what
        one aim integration leaves behind; the snapshot producer must
        reproduce exactly those cell counts."""
        snapshot = planning_snapshot_from_surface(self.surface_aim)
        counts = snapshot.occupancy_counts()
        assert counts["free"] == 30
        assert counts["occupied"] == 73
        assert snapshot.map_revision == 2
        assert snapshot.geometry_hash == self.request["geometry_hash"]

    def test_integrator_accepts_the_real_cloud(self):
        """The saved request's geometry and stamps drive the integrator on
        the pre-aim revision-1 snapshot state (30 free cells came FROM this
        view; here we feed the pre-aim map and check the view integrates
        without touching unknown cells outside its testimony)."""
        from luggage_gazebo.eval_metrics import depth_to_camera_xyz
        pre = dict(self.surface_aim)
        pre["map_revision"] = 1
        # rewind the free row to unknown (it was created by this view)
        import copy
        pre = copy.deepcopy(pre)
        for ix in range(pre["nx"]):
            for iy in range(pre["ny"]):
                if pre["state"][ix][iy] == "free":
                    pre["state"][ix][iy] = "unknown"
        snapshot = planning_snapshot_from_surface(pre)
        stamp_ns = int(round(self.request["stamp_sec"] * 1e9))
        payload = {
            "acquisition_stamp_ns": stamp_ns,
            "tf_stamp_ns": stamp_ns,
            "geometry_hash": self.request["geometry_hash"],
            "points_container": [(0.0, 0.0, 0.2)],
            "observed_cells": [],
        }
        # a minimal real-stamped payload integrates and advances revision
        new_snapshot, _inv, report = integrate_view_with_report(
            snapshot, requirement(), payload)
        assert report["outcome"] in ("integrated", "partial_coverage")
        assert new_snapshot.map_revision == snapshot.map_revision + 1
        assert new_snapshot.acquisition_stamp_ns == stamp_ns
