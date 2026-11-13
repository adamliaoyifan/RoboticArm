"""Behavior contracts for the read-only upper-placement prior.

Each case is one independent behavioral run, never a reliability estimate.
Archive-backed comparisons skip honestly when the primary evidence index is
unavailable; geometry and fallback behaviors never depend on it.
"""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assets import load
from geometry import pose
from planning_types import UpperConfig, digest
from placement_request import PlacementRequest, target_actions
from scene import fixture
from ideal_snapshot import capture_fixture
from request_planner import request_from_record
from upper_prior import (UpperPlacementPrior, PlacementGoal, PriorQueryBudget,
                         apply_prior_to_actions, compose_seed_list)

REPO = Path(__file__).resolve().parents[3]
INDEX = Path(os.environ.get('ULD_UPPER_PRIOR_INDEX',
    '/home/adamliao/work/elfin_humble_ws/docs/status/evidence/uld_upper_layer_v1/posture_prior_g12_01'))
EVIDENCE = INDEX.parent
ARCHIVE = (INDEX / 'catalogue.json').is_file() and (INDEX / 'geometry.json').is_file()


class GeometryWindowContracts(unittest.TestCase):
    """Source-derived necessary geometry; recomputed every process."""

    @classmethod
    def setUpClass(cls):
        cls.assets = load(REPO)
        cls.prior = UpperPlacementPrior(cls.assets)

    @classmethod
    def tearDownClass(cls):
        pass

    @unittest.skipUnless(ARCHIVE, 'archive index unavailable')
    def test_orientation_window_matches_archive_within_0_1mm(self):
        geometry = json.loads((INDEX / 'geometry.json').read_text())
        windows = self.prior.orientation_window(tuple(geometry['box_dimensions']),
                                                geometry['measured_box_T_tool'],
                                                geometry['clearance_m'])
        self.assertEqual(len(windows), len(geometry['modes']))
        for window, reference in zip(windows, geometry['modes']):
            self.assertEqual(window['orientation_family'], reference['mode'])
            self.assertLessEqual(abs(window['required_gap_m'] - reference['required_gap_m']), 1e-4)
            self.assertLessEqual(abs(window['maximum_support_z_container_m']
                                     - reference['maximum_support_z_container_m']), 1e-4)

    def test_window_moves_with_box_height_grasp_and_tool(self):
        geometry = json.loads((INDEX / 'geometry.json').read_text()) if ARCHIVE else None
        attachment = (np.array(geometry['measured_box_T_tool']) if geometry
                      else pose([-.17, 0, .125], Rotation.from_euler('x', np.pi).as_matrix()))
        dims = tuple((.55, .4, .25)) if not ARCHIVE else tuple(geometry['box_dimensions'])
        clearance = .005
        base = {w['orientation_family']: w['required_gap_m']
                for w in self.prior.orientation_window(dims, attachment, clearance)}
        taller = {w['orientation_family']: w['required_gap_m']
                  for w in self.prior.orientation_window((.55, .4, .30), attachment, clearance)}
        self.assertAlmostEqual(taller['up'] - base['up'], .05, delta=1e-9)
        # The pad offset is measured from the box centre, so the down-mode
        # assembly top also rises when the box grows: 0.25 -> 0.275 centre+pad.
        self.assertAlmostEqual(taller['down'] - base['down'], .025, delta=1e-9)
        turned = attachment @ pose([0, 0, 0], Rotation.from_euler('x', np.pi).as_matrix())
        flipped = {w['orientation_family']: w['required_gap_m']
                   for w in self.prior.orientation_window(dims, turned, clearance)}
        self.assertAlmostEqual(flipped['down'], base['up'], delta=1e-9)
        self.assertGreater(flipped['up'], base['up'] + .05)
        grown = self.prior._vertices.copy()
        grown[:, 2] -= .05
        original = self.prior._vertices
        try:
            self.prior._vertices = grown
            lowered = {w['orientation_family']: w['required_gap_m']
                       for w in self.prior.orientation_window(dims, attachment, clearance)}
        finally:
            self.prior._vertices = original
        self.assertGreater(lowered['down'], base['down'] + .04)
        self.assertAlmostEqual(lowered['up'], base['up'], delta=1e-9)

    def test_necessary_rejection_only_inside_ceiling_footprint(self):
        assets = self.assets
        data = fixture(assets)
        data['pickup_payload'] = dict(name='upper_layer_payload', pose_world=[-1, 0, .986, 0, 0, 0])
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / 'fixture.json').write_text(json.dumps(data))
            config = UpperConfig.load()
            snapshot, box = capture_fixture(tmp, assets, config)
        top = max(p.surface_z_container for p in snapshot.surfaces)
        ids = tuple(p.source_id for p in snapshot.surfaces
                    if abs(p.surface_z_container - top) < 1e-7)
        tall = replace(box, dimensions=(.55, .4, .30))
        attachment = pose([-.17, 0, .125], Rotation.from_euler('x', np.pi).as_matrix())

        def request_at(x):
            target = pose([x, -.49, top + .15], Rotation.from_euler('x', np.pi).as_matrix())
            return PlacementRequest('prior-test', tall, snapshot, (0.,) * 6,
                                    (tuple(target.ravel()),), ids, start_attached=True,
                                    confirmed_box_T_tool=tuple(attachment.ravel()))

        deep = request_at(-.10)
        entrance = request_at(-.55)
        budget = PriorQueryBudget(query_budget_s=5.)
        hints = self.prior.query(deep, (PlacementGoal('g0', deep.final_box_poses[0]),), budget)
        self.assertEqual(hints.mode_hints[0].result, 'REJECT_NECESSARY_CEILING_COLLISION')
        self.assertEqual(hints.mode_hints[0].attachment_authority, 'confirmed')
        self.assertGreater(hints.mode_hints[0].required_gap_m,
                           hints.mode_hints[0].available_gap_m)
        entrance_hints = self.prior.query(entrance, (PlacementGoal('g1', entrance.final_box_poses[0]),), budget)
        self.assertEqual(entrance_hints.mode_hints[0].result, 'EXACT_MESH_CHECK_REQUIRED')
        self.assertFalse(entrance_hints.mode_hints[0].within_ceiling_footprint)


class _StubAction:
    def __init__(self, action_id, final_pose):
        from planning_types import PlacementCandidate, SupportMetrics
        self.action_id = action_id
        self.placement = PlacementCandidate(action_id, 'snapshot', 'config',
                                            final_pose, SupportMetrics(1., 1., '', '', 1., (), True, 'SUPPORTED'))


class ActionFilterContracts(unittest.TestCase):
    def test_apply_prior_rejects_orders_and_falls_back(self):
        from upper_prior import PriorHints
        good = tuple(pose([-.2, -.49, 1.876], Rotation.from_euler('x', np.pi).as_matrix()).ravel())
        ordered = tuple(pose([.2, -.49, 1.876], Rotation.from_euler('x', np.pi).as_matrix()).ravel())
        actions = [_StubAction('a', good), _StubAction('b', ordered), _StubAction('c', good)]

        from upper_prior import ModeHint, CandidateRank
        hints = PriorHints(
            identity=(), effective=(), goal_poses=(('g0', good), ('g1', ordered)),
            mode_hints=(ModeHint('g0', 'up', .25, .25, .255, .284, True, 'confirmed',
                                 'REJECT_NECESSARY_CEILING_COLLISION', (0., 0., 2.01, 1.751)),
                        ModeHint('g1', 'up', .25, .25, .255, .284, True, 'confirmed',
                                 'GEOMETRY_WINDOW_OK', (0., 0., 2.01, 1.751))),
            seed_sets=(), route_templates=(),
            candidate_order=(CandidateRank('g1', 0, ('mode_hint',)), CandidateRank('g0', 1, ('mode_hint',))),
            diagnostics=(('fallback_required', False),))
        kept, dropped = apply_prior_to_actions(actions, hints)
        self.assertEqual([a.action_id for a in kept], ['b'])
        self.assertEqual([row['action_id'] for row in dropped], ['a', 'c'])
        self.assertEqual({row['result'] for row in dropped},
                         {'REJECT_NECESSARY_CEILING_COLLISION'})

        approximate = PriorHints(
            identity=(), effective=(), goal_poses=(('g0', good),),
            mode_hints=(ModeHint('g0', 'up', .25, .25, .7, .284, True, 'archived_representative',
                                 'APPROXIMATE_ARCHIVED_ATTACHMENT', (0., 0., 2.01, 1.751)),),
            seed_sets=(), route_templates=(), candidate_order=(CandidateRank('g0', 0, ('mode_hint',)),),
            diagnostics=(('fallback_required', False),))
        kept, dropped = apply_prior_to_actions(actions[:1] + [_StubAction('z', ordered)], approximate)
        self.assertEqual([a.action_id for a in kept], ['a', 'z'])
        self.assertEqual(dropped, [])

        fallback = PriorHints(identity=(), effective=(), goal_poses=(('g0', good),),
                              mode_hints=(), seed_sets=(), route_templates=(),
                              candidate_order=(), diagnostics=(('fallback_required', True),))
        kept, dropped = apply_prior_to_actions(actions, fallback)
        self.assertEqual([a.action_id for a in kept], ['a', 'b', 'c'])
        self.assertEqual(dropped, [])

    def test_compose_seed_list_matches_legacy_when_disabled(self):
        config = UpperConfig.load()
        start = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)
        rng_seeds = [[float(i), 0., 0., 0., 0., 0.] for i in range(config.ik_seed_count - 1)]
        legacy = [list(start)] + rng_seeds[:config.ik_seed_count - 1]
        self.assertEqual(compose_seed_list(start, config, None, _StubAction('a', (0.,) * 16), rng_seeds),
                         legacy)

        from upper_prior import PriorHints, SeedSet
        hints = PriorHints(identity=(), effective=(), goal_poses=(('g0', tuple(range(16))),),
                           mode_hints=(), seed_sets=(
                               SeedSet('g0', 'final', tuple(range(16)),
                                       ((0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
                                        (1.5, 1.5, 1.5, 1.5, 1.5, 1.5),
                                        (2.5, 2.5, 2.5, 2.5, 2.5, 2.5)), 'atlas:up:0', 0.05, 3.),
                               SeedSet('g0', 'route', None, ((3.5,) * 6,), 'catalogue:x', 0., 0.)),
                           route_templates=(), candidate_order=(), diagnostics=())
        action = _StubAction('a', tuple(range(16)))
        seeds = compose_seed_list(start, config, hints, action, rng_seeds)
        self.assertEqual(seeds[0], list(start))
        self.assertEqual([round(v, 9) for v in seeds[1]], [1.5] * 6)
        self.assertEqual([round(v, 9) for v in seeds[2]], [2.5] * 6)
        self.assertEqual([round(v, 9) for v in seeds[3]], [3.5] * 6)
        self.assertEqual(len(seeds), config.ik_seed_count)
        self.assertLessEqual(max(abs(v) for v in seeds[-1]), float(len(rng_seeds)))


class QueryBehaviorContracts(unittest.TestCase):
    """Fallback, budget, read-only and attachment-mode behaviors."""

    @classmethod
    def setUpClass(cls):
        cls.assets = load(REPO)
        cls.prior = UpperPlacementPrior(cls.assets)
        cls.tmp = tempfile.TemporaryDirectory()
        data = fixture(cls.assets)
        data['pickup_payload'] = dict(name='upper_layer_payload', pose_world=[-1, 0, .986, 0, 0, 0])
        (Path(cls.tmp.name) / 'fixture.json').write_text(json.dumps(data))
        cls.config = UpperConfig.load()
        cls.snapshot, cls.box = capture_fixture(cls.tmp.name, cls.assets, cls.config)
        top = max(p.surface_z_container for p in cls.snapshot.surfaces)
        cls.ids = tuple(p.source_id for p in cls.snapshot.surfaces
                        if abs(p.surface_z_container - top) < 1e-7)
        cls.target = pose([-.10, -.49, top + cls.box.dimensions[2] / 2],
                          Rotation.from_euler('x', np.pi).as_matrix())

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def detached_request(self, x=-.10):
        target = self.target.copy()
        target[0, 3] = x
        return PlacementRequest('prior-test', self.box, self.snapshot, (0.,) * 6,
                                (tuple(target.ravel()),), self.ids)

    def attached_request(self):
        attachment = pose([-.17, 0, .125], Rotation.from_euler('x', np.pi).as_matrix())
        return replace(self.detached_request(), start_attached=True,
                       confirmed_box_T_tool=tuple(attachment.ravel()))

    def test_missing_index_falls_back_without_crash(self):
        request = self.attached_request()
        goal = PlacementGoal('g', request.final_box_poses[0])
        hints = self.prior.query(request, (goal,), PriorQueryBudget(query_budget_s=5.))
        self.assertEqual(dict(hints.diagnostics)['index_status'], 'MISSING')
        self.assertFalse(hints.seed_sets and hints.route_templates)
        # Normal search stays intact: nothing is dropped or reordered.
        kept, dropped = apply_prior_to_actions([_StubAction('a', request.final_box_poses[0])], hints)
        self.assertEqual(len(kept), 1)
        self.assertEqual(dropped, [])
        broken = UpperPlacementPrior(self.assets, index_path='/nonexistent/prior_index')
        hints = broken.query(request, (goal,), PriorQueryBudget(query_budget_s=5.))
        self.assertEqual(dict(hints.diagnostics)['index_status'], 'UNREADABLE')
        # With no attachment and no archive, nothing actionable remains.
        detached = self.detached_request()
        hints = self.prior.query(detached, (PlacementGoal('g', detached.final_box_poses[0]),),
                                 PriorQueryBudget(query_budget_s=5.))
        self.assertTrue(dict(hints.diagnostics)['fallback_required'])

    def test_unreachable_goal_records_misses(self):
        request = self.detached_request(x=3.2)
        hints = self.prior.query(request, (PlacementGoal('g', request.final_box_poses[0]),),
                                 PriorQueryBudget(query_budget_s=1.))
        self.assertFalse(any(s.goal_id == 'g' for s in hints.seed_sets))
        self.assertFalse(hints.route_templates)

    def test_tiny_budget_never_crashes_without_index(self):
        request = self.detached_request()
        goals = (PlacementGoal('g0', request.final_box_poses[0]),
                 PlacementGoal('g1', request.final_box_poses[0]))
        hints = self.prior.query(request, goals, PriorQueryBudget(query_budget_s=1e-9))
        rows = {row[0]: row[1] for row in dict(hints.diagnostics)['per_goal']}
        self.assertEqual(set(rows), {'g0', 'g1'})

    def test_multigoal_query_never_starves_later_goals(self):
        request = self.attached_request()
        far = self.target.copy()
        far[0, 3] = .3
        request = replace(request, final_box_poses=(request.final_box_poses[0], tuple(far.ravel())))
        goals = tuple(PlacementGoal(f'g{i}', p) for i, p in enumerate(request.final_box_poses))
        hints = self.prior.query(request, goals, PriorQueryBudget(query_budget_s=1.))
        rows = {row[0]: row[1] for row in dict(hints.diagnostics)['per_goal']}
        self.assertEqual(rows['g1'], 'queried')

    def test_attached_request_is_untouched_and_preserved_value_exactly(self):
        request = self.attached_request()
        before = digest(asdict(request))
        snapshot_before = digest(asdict(request.snapshot))
        hints = self.prior.query(request, (PlacementGoal('g', request.final_box_poses[0]),),
                                 PriorQueryBudget(query_budget_s=1.))
        self.assertEqual(digest(asdict(request)), before)
        self.assertEqual(digest(asdict(request.snapshot)), snapshot_before)
        self.assertEqual(dict(hints.identity)['confirmed_box_T_tool'],
                         tuple(request.confirmed_box_T_tool))
        self.assertTrue(all(s.requires_refinement for s in hints.seed_sets))
        with self.assertRaises(Exception):
            hints.identity = ()

    def test_detached_request_reports_attachment_gap_honestly(self):
        request = self.detached_request()
        hints = self.prior.query(request, (PlacementGoal('g', request.final_box_poses[0]),),
                                 PriorQueryBudget(query_budget_s=1.))
        hint = hints.mode_hints[0]
        self.assertEqual(hint.attachment_authority, 'none')
        self.assertEqual(hint.result, 'NOT_EVALUATED_NO_ATTACHMENT')
        self.assertIsNone(dict(hints.identity)['confirmed_box_T_tool'])
        self.assertEqual(hint.goal_id, 'g')

    def test_budget_reads_runtime_values_and_records_them(self):
        budget = PriorQueryBudget.from_config(self.config)
        self.assertEqual(budget.query_budget_s, self.config.prior_query_budget_s)
        self.assertEqual(budget.radius_m, self.config.prior_query_radius_m)
        self.assertEqual(budget.radius_deg, self.config.prior_query_radius_deg)
        self.assertEqual(budget.branches_per_stage, self.config.prior_branches_per_stage)
        request = self.attached_request()
        hints = self.prior.query(request, (PlacementGoal('g', request.final_box_poses[0]),), budget)
        effective = dict(hints.effective)
        for key in ('query_budget_s', 'radius_m', 'radius_deg', 'branches_per_stage'):
            self.assertIn(key, effective)
        json.dumps(hints.as_record(), allow_nan=False)


@unittest.skipUnless(ARCHIVE, 'archive index unavailable')
class ArchiveBackedContracts(unittest.TestCase):
    """Catalogue/atlas-backed gates against the archived physical request."""

    @classmethod
    def setUpClass(cls):
        cls.assets = load(REPO, REPO / 'src/luggage_description/config/scene_tf_deploy_update.yaml')
        cls.request = request_from_record(json.loads(
            (EVIDENCE / 'request_physical_g12_10/live_request.json').read_text()))
        cls.prior = UpperPlacementPrior(cls.assets, index_path=INDEX)
        cls.budget = PriorQueryBudget(query_budget_s=5., execution_budget_s=1200.)

    def query(self, request=None, budget=None):
        request = request or self.request
        goals = tuple(PlacementGoal(f'g{i}', p) for i, p in enumerate(request.final_box_poses))
        return self.prior.query(request, goals, budget or self.budget)

    def test_tiny_budget_exhausts_every_goal_without_crash(self):
        import upper_prior
        upper_prior._ATLAS_CACHE.clear()  # deterministic cold index loads
        cold = UpperPlacementPrior(self.assets, index_path=INDEX)
        request = replace(self.request, start_attached=False, confirmed_box_T_tool=None)
        goals = (PlacementGoal('g0', request.final_box_poses[0]),
                 PlacementGoal('g1', request.final_box_poses[0]))
        hints = cold.query(request, goals, PriorQueryBudget(query_budget_s=1e-9))
        rows = {row[0]: row[1] for row in dict(hints.diagnostics)['per_goal']}
        self.assertEqual(set(rows), {'g0', 'g1'})
        self.assertTrue(all(status == 'budget_exhausted' for status in rows.values()))
        self.assertTrue(dict(hints.diagnostics)['budget_exhausted'])

    def test_identity_change_invalidates_old_certificates(self):
        hints = self.query()
        matched = [t for t in hints.route_templates
                   if t.source_id == 'request_physical_g12_10']
        self.assertTrue(matched)
        self.assertTrue(dict(matched[0].identity)['snapshot_id'])
        changed = replace(self.request,
                          snapshot=replace(self.request.snapshot, snapshot_id='modified-scene'))
        hints = self.query(changed)
        for template in hints.route_templates:
            identity = dict(template.identity)
            self.assertFalse(identity['snapshot_id'])
            self.assertEqual(template.authority, 'seed_only')
        regrasped = replace(self.request,
                            confirmed_box_T_tool=tuple(
                                (np.array(self.request.confirmed_box_T_tool).reshape(4, 4)
                                 @ pose([0, 0, 0], Rotation.from_euler('z', np.pi / 8).as_matrix())).ravel().tolist()))
        hints = self.query(regrasped)
        for template in hints.route_templates:
            identity = dict(template.identity)
            self.assertFalse(identity['attachment'])
            self.assertEqual(template.authority, 'seed_only')
        for template in self.query().route_templates:
            self.assertEqual(template.authority, 'seed_only')

    def test_atlas_seeds_are_approximate_and_refine_to_exact_targets(self):
        hints = self.query()
        rows = [s for s in hints.seed_sets if s.stage == 'final']
        self.assertTrue(rows)
        row = rows[0]
        self.assertTrue(row.requires_refinement)
        urdf = self.assets['urdf']
        path = urdf.path_to('suction_contact_frame')
        target = self.prior._base_container @ np.array(row.target_container).reshape(4, 4)
        from luggage_planning.offline_ik import refine_to_pose
        refined_any = False
        for q in row.joints:
            actual = urdf.tcp(list(q), path)
            self.assertGreater(float(np.linalg.norm(actual[:3, 3] - target[:3, 3])), 1e-4)
            refined, info = refine_to_pose(urdf, path, target[:3, 3],
                                           Rotation.from_matrix(target[:3, :3]).as_quat(), list(q))
            if refined is not None:
                refined_any = True
                self.assertLessEqual(info['pos_err_m'], 5e-4)
        self.assertTrue(refined_any)
        limits = np.array(urdf.joint_limits())
        for seed in hints.seed_sets:
            for q in seed.joints:
                self.assertTrue(np.all(np.array(q) >= limits[:, 0] - 1e-12)
                                and np.all(np.array(q) <= limits[:, 1] + 1e-12))
        for seed in hints.seed_sets:
            if seed.source.startswith('atlas:'):
                self.assertLessEqual(seed.distance_m, self.budget.radius_m + 1e-9)
                self.assertLessEqual(seed.distance_deg, self.budget.radius_deg + 1e-9)

    def test_runtime_com_margin_changes_admission_without_compile(self):
        with tempfile.TemporaryDirectory() as tmp:
            wide = Path(tmp) / 'wide.yaml'
            wide.write_text(Path(REPO / 'research/uld_upper_layer/upper_config.yaml')
                            .read_text().replace('support_com_margin_m: 0.020',
                                                 'support_com_margin_m: 0.080'))
            narrow = UpperConfig.load()
            wide_config = UpperConfig.load(wide)
        self.assertEqual(narrow.support_com_margin_m, .020)
        self.assertEqual(wide_config.support_com_margin_m, .080)
        narrow_hints = self.query(budget=PriorQueryBudget.from_config(narrow))
        wide_hints = self.query(budget=PriorQueryBudget.from_config(wide_config))
        template = 'multi_scheme_g12_01/positions'

        def admission(hints):
            rows = [t for t in hints.route_templates if t.source_id == template]
            self.assertTrue(rows)
            return {row[0]: row for row in rows[0].admission}

        narrow_support = admission(narrow_hints)['handoff_support']
        wide_support = admission(wide_hints)['handoff_support']
        self.assertTrue(narrow_support[1])
        self.assertEqual(narrow_support[2], 'SUPPORTED')
        self.assertAlmostEqual(narrow_support[3], .065, delta=.001)
        self.assertFalse(wide_support[1])
        self.assertEqual(wide_support[2], 'COM_SUPPORT_MARGIN')

    def test_duration_gate_lifts_only_duration(self):
        catalogue = json.loads((INDEX / 'catalogue.json').read_text())
        slow = next(r for r in catalogue['records']
                    if r['id'] == 'depth_rows_g12_01/remove_front_column_lateral')
        self.assertGreater(slow['duration_sim_s'], 1200.)
        goal_pose = tuple(slow['poses_container_absolute']['final_box'])
        request = replace(self.request, final_box_poses=(goal_pose,))
        goals = (PlacementGoal('g0', goal_pose),)
        strict = self.prior.query(request, goals, self.budget)
        relaxed = self.prior.query(request, goals, replace(self.budget, execution_budget_s=1400.))

        def slow_template(hints):
            rows = [t for t in hints.route_templates if t.source_id == slow['id']]
            self.assertTrue(rows)
            return rows[0]

        strict_duration = {row[0]: row for row in slow_template(strict).admission}['execution_duration']
        relaxed_duration = {row[0]: row for row in slow_template(relaxed).admission}['execution_duration']
        self.assertFalse(strict_duration[1])
        self.assertEqual(strict_duration[2], 'EXECUTION_DURATION_EXCEEDS_BUDGET')
        self.assertTrue(relaxed_duration[1])
        self.assertEqual(relaxed_duration[2], 'EXECUTION_DURATION_WITHIN_BUDGET')
        strict_support = {row[0]: row for row in slow_template(strict).admission}['handoff_support']
        relaxed_support = {row[0]: row for row in slow_template(relaxed).admission}['handoff_support']
        self.assertEqual(strict_support, relaxed_support)
        self.assertEqual(dict(slow_template(strict).identity), dict(slow_template(relaxed).identity))

    def test_detached_request_uses_archive_attachments_without_rejection(self):
        request = replace(self.request, start_attached=False, confirmed_box_T_tool=None)
        hints = self.query(request)
        hint = hints.mode_hints[0]
        self.assertEqual(hint.attachment_authority, 'archived_representative')
        self.assertNotEqual(hint.result, 'REJECT_NECESSARY_CEILING_COLLISION')
        kept, dropped = apply_prior_to_actions(
            [_StubAction('a', request.final_box_poses[0])], hints)
        self.assertEqual(len(kept), 1)
        self.assertEqual(dropped, [])


if __name__ == '__main__':
    unittest.main()
