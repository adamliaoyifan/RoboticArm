"""Behavior contracts for the seven-size paired evaluation runner.

Fast structural tests only: matrix invariants, goal derivation from support
polygons, paired-config isolation, five-way classification, pairing gates and
audit linkage. No planner unit is executed here.
"""
import json
import os
from dataclasses import asdict
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate_prior as ep
from planning_types import UpperConfig, digest
from request_planner import request_from_record

REPO = Path(__file__).resolve().parents[3]
PRIMARY = Path(subprocess.check_output(
    ['git', '-C', str(REPO), 'rev-parse', '--path-format=absolute', '--git-common-dir'],
    text=True).strip()).parent / 'docs/status/evidence'
LIVE = PRIMARY / 'uld_upper_layer_v1/request_physical_g12_10/live_request.json'
ARCHIVE = LIVE.is_file()


def matrix(tmp, **changes):
    value = dict(
        scene_config=str(REPO / 'src/luggage_description/config/scene_tf_deploy_update.yaml'),
        live_request=str(LIVE), prior_index=str(PRIMARY / 'uld_upper_layer_v1/posture_prior_g12_01'),
        planning_profile=str(REPO / 'research/uld_upper_layer/profiles/nominal_physics_planning.yaml'),
        execution_profile=str(REPO / 'research/uld_upper_layer/profiles/nominal_physics_execution.yaml'),
        unit_search_budget_s=1200., watchdog_s=1320., total_budget_s=14 * 3600.,
        solution_count_target=3, minimum_executable_solutions=1, seeds=(0, 1, 2),
        sizes=[dict(size_id=name, dimensions=dims, mass_kg=8.0) for name, dims in ep.BASE_SIZES],
        rehearsal_budget_s=1800.,
        start_q_candidates=[[0.] * 6, [3.5702, -1.3263, -1.0965, 3.9564, 1.6234, 0.4522]])
    value.update(changes)
    path = Path(tmp) / 'matrix.json'
    path.write_text(json.dumps(value))
    return path


class MatrixContracts(unittest.TestCase):
    def test_fixed_seven_sizes_and_seeds(self):
        with tempfile.TemporaryDirectory() as tmp:
            loaded = ep.load_matrix(matrix(tmp))
        self.assertEqual([row['size_id'] for row in loaded['sizes']],
                         [name for name, _ in ep.BASE_SIZES])
        self.assertEqual(tuple(loaded['seeds']), (0, 1, 2))
        self.assertEqual(list(loaded['sizes'][0]['dimensions']), [.550, .400, .250])
        self.assertEqual(loaded['unit_search_budget_s'], 1200.)
        self.assertEqual(loaded['watchdog_s'], 1320.)

    def test_rejects_wrong_matrices(self):
        with tempfile.TemporaryDirectory() as tmp:
            good = json.loads(matrix(tmp).read_text())
            for mutate in (
                lambda m: m['sizes'].pop(),
                lambda m: m['sizes'][0].__setitem__('dimensions', [.4, .55, .25]),
                lambda m: m.__setitem__('seeds', (0, 1)),
                lambda m: m.__setitem__('start_q_candidates', [[0.] * 5])):
                broken = json.loads(json.dumps(good))
                mutate(broken)
                path = Path(tmp) / 'broken.json'
                path.write_text(json.dumps(broken))
                with self.assertRaises(ValueError):
                    ep.load_matrix(path)

    def test_evidence_must_be_primary_absolute(self):
        result = subprocess.run(
            [sys.executable, str(REPO / 'research/uld_upper_layer/evaluate_prior.py'),
             '--matrix', str(matrix(tempfile.mkdtemp())), '--evidence', '/tmp/not-primary',
             '--prepare-only'], capture_output=True, text=True, timeout=60)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('EVIDENCE_MUST_BE_PRIMARY_ABSOLUTE', result.stderr)


class GoalDerivationContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = UpperConfig.load(
            REPO / 'research/uld_upper_layer/profiles/nominal_physics_planning.yaml')
        cls.live = json.loads(LIVE.read_text())
        cls.request = request_from_record(cls.live)

    @unittest.skipUnless(ARCHIVE, 'archive live request unavailable')
    def test_base_size_reproduces_proven_archived_target(self):
        goal, derivation = ep.derive_goal(self.request.snapshot, self.request.support_source_ids,
                                          (.550, .400, .250), self.config)
        np.testing.assert_allclose(goal[:3, 3], [-0.398, -0.4905, 1.850999], atol=1e-9)

    @unittest.skipUnless(ARCHIVE, 'archive live request unavailable')
    def test_goal_tracks_dimensions_and_clearance(self):
        for dims, expected_z in (((.550, .400, .200), 1.825999), ((.550, .400, .300), 1.875999)):
            goal, _ = ep.derive_goal(self.request.snapshot, self.request.support_source_ids,
                                     dims, self.config)
            self.assertAlmostEqual(goal[2, 3], expected_z, delta=1e-9)
        for length in (.500, .600):
            goal, _ = ep.derive_goal(self.request.snapshot, self.request.support_source_ids,
                                     (length, .400, .250), self.config)
            self.assertAlmostEqual(goal[0, 3], -0.728 + length / 2 + 0.05 + 0.005, delta=1e-9)
        from dataclasses import replace
        wider = ep.derive_goal(self.request.snapshot, self.request.support_source_ids,
                               (.550, .400, .250),
                               replace(self.config, collision_margin_m=.010))
        base = ep.derive_goal(self.request.snapshot, self.request.support_source_ids,
                              (.550, .400, .250), self.config)
        self.assertAlmostEqual(wider[0][0, 3] - base[0][0, 3], .005, delta=1e-9)

    @unittest.skipUnless(ARCHIVE, 'archive live request unavailable')
    def test_requests_share_occ_and_carry_synthetic_marking(self):
        request, derivation, bottom = ep.build_request(self.live, (.550, .400, .250), 8.0,
                                                       self.config)
        self.assertFalse(request.start_attached)
        self.assertIsNone(request.confirmed_box_T_tool)
        self.assertEqual(request.snapshot.snapshot_id, self.request.snapshot.snapshot_id)
        self.assertEqual(request.box.mass_kg, 8.0)
        self.assertEqual(request.box.com_local, (0., 0., 0.))
        self.assertIn('eval-synthetic-fixture-unattached', request.request_id)
        self.assertAlmostEqual(request.box.container_T_box[11] - .250 / 2, bottom, delta=1e-9)
        tall, _, _ = ep.build_request(self.live, (.550, .400, .300), 8.0, self.config)
        # Fixed platform bottom: the centre rises by half the height delta.
        self.assertAlmostEqual(tall.box.container_T_box[11] - request.box.container_T_box[11],
                               .025, delta=1e-9)


class ConfigPairingContracts(unittest.TestCase):
    def test_modes_differ_only_in_prior_block(self):
        with tempfile.TemporaryDirectory() as tmp:
            configs = ep.derived_configs(ep.load_matrix(matrix(tmp)))
        baseline = asdict(configs['baseline'])
        prior = asdict(configs['prior'])
        diff = {key for key in baseline if baseline[key] != prior[key]}
        self.assertEqual(diff, {'prior_enabled', 'prior_index_path'})
        self.assertEqual(configs['baseline'].search_budget_s, 1200.)
        self.assertEqual(configs['baseline'].solution_count_target, 3)
        self.assertEqual(configs['baseline'].minimum_executable_solutions, 1)
        self.assertTrue(configs['baseline'].payload_effort_screen_enabled)
        self.assertFalse(configs['baseline'].prior_enabled)
        self.assertTrue(configs['prior'].prior_enabled)


def unit_dir(root, size_id, mode, seed):
    return Path(root) / 'units' / size_id / f'{mode}_seed{seed}'


def write_unit(root, size_id, mode, seed, outcome, *, valid=False, geometry=80,
               reason='NO_COMPLETE_PLAN_IN_ACTION_FAMILY', rejections=None,
               capture=True, summary=True, prior_file=None, elapsed=100.):
    directory = unit_dir(root, size_id, mode, seed)
    directory.mkdir(parents=True, exist_ok=True)
    if summary:
        directory.joinpath('summary.json').write_text(json.dumps(dict(
            valid=valid, reason='COMPLETE_SCORED_PLANS' if valid else reason,
            geometry_candidates=geometry, attempted_actions=1,
            rejection_counts=rejections or {}, elapsed_s=elapsed)))
    directory.joinpath('manifest.json').write_text(json.dumps(dict(
        capture_complete=capture, replay_possible=capture, config={}, revision='x')))
    for name in ('request.json', 'inputs.json', 'models.json', 'candidates.json',
                 'geometry_report.json'):
        directory.joinpath(name).write_text('{}')
    directory.joinpath('request_boundaries.jsonl').write_text('')
    if prior_file is not None:
        directory.joinpath('prior_query.json').write_text(json.dumps(prior_file))
    directory.joinpath('unit_summary.json').write_text(json.dumps(dict(
        size_id=size_id, mode=mode, seed=seed, outcome=outcome, detail={},
        elapsed_s=elapsed)))
    return directory


class ClassificationContracts(unittest.TestCase):
    def test_five_way_outcomes(self):
        with tempfile.TemporaryDirectory() as root:
            valid = write_unit(root, 'a', 'baseline', 0, 'NOMINAL_VALID', valid=True)
            self.assertEqual(ep.classify_unit(valid, False)[0], 'NOMINAL_VALID')
            geometry = write_unit(root, 'a', 'baseline', 1, 'GEOMETRY_REJECTED', geometry=0)
            self.assertEqual(ep.classify_unit(geometry, False)[0], 'GEOMETRY_REJECTED')
            endpoint = write_unit(root, 'a', 'baseline', 2, 'ENDPOINT_REJECTED',
                                  rejections={'NO_SAMPLED_HANDOFF_ENDPOINT_SOLUTION': 80})
            self.assertEqual(ep.classify_unit(endpoint, False)[0], 'ENDPOINT_REJECTED')
            initial = write_unit(root, 'b', 'baseline', 0, 'ENDPOINT_REJECTED',
                                 reason='INITIAL_STATE_INVALID')
            self.assertEqual(ep.classify_unit(initial, False)[0], 'ENDPOINT_REJECTED')
            mixed = write_unit(root, 'b', 'baseline', 1, 'NO_PLAN_WITHIN_BUDGET',
                               rejections={'NO_SAMPLED_PUSH_ENDPOINT_SOLUTION': 2,
                                           'SWEEP_REJECTED': 78})
            self.assertEqual(ep.classify_unit(mixed, False)[0], 'NO_PLAN_WITHIN_BUDGET')
            timeout = write_unit(root, 'b', 'baseline', 2, 'INFRA_FAILURE')
            self.assertEqual(ep.classify_unit(timeout, True)[0], 'INFRA_FAILURE')
            incomplete = write_unit(root, 'c', 'baseline', 0, 'INFRA_FAILURE', capture=False)
            self.assertEqual(ep.classify_unit(incomplete, False)[0], 'INFRA_FAILURE')
            crashed = unit_dir(root, 'c', 'baseline', 1)
            crashed.mkdir(parents=True, exist_ok=True)
            crashed.joinpath('error.txt').write_text('boom')
            outcome, detail = ep.classify_unit(crashed, False)
            self.assertEqual(outcome, 'INFRA_FAILURE')
            self.assertEqual(detail['detail'], 'NO_PLANNER_SUMMARY')
            prior_all = write_unit(root, 'd', 'prior', 0, 'GEOMETRY_REJECTED',
                                   prior_file=dict(diagnostics={}), geometry=5)
            prior_all.joinpath('summary.json').write_text(json.dumps(dict(
                valid=False, reason='NO_COMPLETE_PLAN_IN_ACTION_FAMILY',
                geometry_candidates=5, attempted_actions=0, rejection_counts={},
                elapsed_s=10.)))
            outcome, detail = ep.classify_unit(prior_all, False)
            self.assertEqual(outcome, 'GEOMETRY_REJECTED')
            self.assertEqual(detail['detail'], 'PRIOR_DROPPED_ALL_ACTIONS')

    def test_reports_and_pairing_gates(self):
        sizes = [name for name, _ in ep.BASE_SIZES]
        with tempfile.TemporaryDirectory() as root:
            frozen = Path(root) / 'frozen'
            for name in sizes:
                (frozen / name).mkdir(parents=True)
                (frozen / name / 'request.json').write_text('{"request_id":"x"}')
            (Path(root) / 'prepare_report.json').write_text(json.dumps(
                dict(baseline_revision='r1', prior_revision='r2')))
            # base size: three double successes; one other size has a regression.
            for seed in (0, 1, 2):
                write_unit(root, sizes[0], 'baseline', seed, 'NOMINAL_VALID', valid=True)
                write_unit(root, sizes[0], 'prior', seed, 'NOMINAL_VALID', valid=True,
                           prior_file={})
                write_unit(root, sizes[1], 'baseline', seed, 'NOMINAL_VALID', valid=True)
            write_unit(root, sizes[1], 'prior', 0, 'NO_PLAN_WITHIN_BUDGET')
            write_unit(root, sizes[1], 'prior', 1, 'NOMINAL_VALID', valid=True, prior_file={})
            write_unit(root, sizes[1], 'prior', 2, 'NOMINAL_VALID', valid=True, prior_file={})
            for name in sizes[2:]:
                for seed in (0, 1, 2):
                    write_unit(root, name, 'baseline', seed, 'NO_PLAN_WITHIN_BUDGET')
                    write_unit(root, name, 'prior', seed, 'NO_PLAN_WITHIN_BUDGET',
                               prior_file={})
            ep.write_reports(dict(sizes=[dict(size_id=n, dimensions=d) for n, d in ep.BASE_SIZES],
                                  seeds=(0, 1, 2)), Path(root))
            report = json.loads((Path(root) / 'paired_report.json').read_text())
            self.assertFalse(report['gates']['every_baseline_success_has_prior_success'])
            self.assertIn(f'{sizes[1]}/seed0', report['gates']['regressions'])
            self.assertEqual(report['gates']['base_size_paired_successes'], 3)
            self.assertEqual(report['speed']['double_success_pairs'], 5)
            self.assertIn('median', json.dumps(report['speed']))
            capability = {row['size_id']: row for row in
                          json.loads((Path(root) / 'capability_table.json').read_text())}
            self.assertEqual(capability[sizes[0]]['prior']['nominal_valid_count'], 3)
            rows = (Path(root) / 'results.csv').read_text().splitlines()
            self.assertEqual(len(rows), 1 + 42)

            # Audit flags the missing seed-0 prior plan as a pairing regression record.
            ep.audit(dict(sizes=[dict(size_id=n, dimensions=d) for n, d in ep.BASE_SIZES],
                          seeds=(0, 1, 2)), None, Path(root))
            audit = json.loads((Path(root) / 'audit.json').read_text())
            self.assertEqual(audit['verdict'], 'ISSUES')
            self.assertTrue(any('not_evaluated' in issue for issue in audit['issues'])
                            or audit['gates']['regressions'])

    def test_audit_detects_request_and_classification_drift(self):
        sizes = [ep.BASE_SIZES[0][0]]
        with tempfile.TemporaryDirectory() as root:
            frozen = Path(root) / 'frozen' / sizes[0]
            frozen.mkdir(parents=True)
            record = dict(request_id='frozen')
            frozen.joinpath('request.json').write_text(json.dumps(record))
            (Path(root) / 'prepare_report.json').write_text(json.dumps(
                dict(baseline_revision='r1', prior_revision='r2')))
            drifted = write_unit(root, sizes[0], 'baseline', 0, 'NOMINAL_VALID', valid=True)
            drifted.joinpath('request.json').write_text(json.dumps(dict(request_id='drifted')))
            misclassified = write_unit(root, sizes[0], 'prior', 0, 'NOMINAL_VALID',
                                       valid=False, prior_file={})
            ep.audit(dict(sizes=[dict(size_id=sizes[0], dimensions=(.55, .4, .25))],
                          seeds=(0, 1, 2)), None, Path(root))
            audit = json.loads((Path(root) / 'audit.json').read_text())
            self.assertTrue(any('request drift' in issue for issue in audit['issues']))
            self.assertTrue(any('classification drift' in issue for issue in audit['issues']))
            self.assertTrue(any('not_evaluated' in issue for issue in audit['issues']))


if __name__ == '__main__':
    unittest.main()
