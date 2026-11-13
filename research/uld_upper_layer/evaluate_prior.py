"""Seven-size paired baseline/prior nominal planning matrix runner.

Repeating entry point for the frozen size matrix: derives each size's targets
from the support polygons, box dimensions and runtime clearances, freezes one
request per size shared by both modes and all seeds, then runs planner units
sequentially with an outer watchdog. Every unit keeps an explicit outcome;
"no plan found" is never reported as "does not fit".

No simulator is launched; inputs are ideal sim fixtures and every artifact is
marked synthetic_fixture. Evidence only under the primary workspace root.
"""
import argparse
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from placement_request import PlacementRequest
from planning_types import UpperConfig, digest
from request_planner import UpperPlacementPlanner, request_from_record

BASE_SIZES = (
    ('base_550x400x250', (.550, .400, .250)),
    ('len_500x400x250', (.500, .400, .250)),
    ('len_600x400x250', (.600, .400, .250)),
    ('wid_550x350x250', (.550, .350, .250)),
    ('wid_550x450x250', (.550, .450, .250)),
    ('hgt_550x400x200', (.550, .400, .200)),
    ('hgt_550x400x300', (.550, .400, .300)),
)
OUTCOMES = ('GEOMETRY_REJECTED', 'ENDPOINT_REJECTED', 'NO_PLAN_WITHIN_BUDGET',
            'NOMINAL_VALID', 'INFRA_FAILURE')
ENDPOINT_REASONS = {'INITIAL_STATE_INVALID', 'NO_SAMPLED_PUSH_ENDPOINT_SOLUTION',
                    'NO_SAMPLED_HANDOFF_ENDPOINT_SOLUTION', 'CONFIRMED_SEAL_OUTSIDE_BOX',
                    'CONFIRMED_ATTACHMENT_POSE_MISMATCH'}
REQUIRED_UNIT_FILES = ('request.json', 'manifest.json', 'inputs.json', 'models.json',
                       'candidates.json', 'geometry_report.json', 'request_boundaries.jsonl')


def repo_root():
    return Path(__file__).resolve().parents[2]


def primary_evidence_root():
    common = Path(subprocess.check_output(['git', '-C', str(repo_root()), 'rev-parse',
        '--path-format=absolute', '--git-common-dir'], text=True).strip())
    return common.parent / 'docs/status/evidence'


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=1, allow_nan=False) + '\n')
    temporary.replace(path)


def default_matrix(prior_index, live_request, planning_profile, execution_profile):
    return dict(
        scene_config=str((repo_root() / 'src/luggage_description/config/scene_tf_deploy_update.yaml').resolve()),
        live_request=str(Path(live_request).resolve()), prior_index=str(Path(prior_index).resolve()),
        planning_profile=str(Path(planning_profile).resolve()),
        execution_profile=str(Path(execution_profile).resolve()),
        unit_search_budget_s=1200., watchdog_s=1320., total_budget_s=14 * 3600.,
        solution_count_target=3, minimum_executable_solutions=1, seeds=(0, 1, 2),
        sizes=[dict(size_id=name, dimensions=dims, mass_kg=8.0) for name, dims in BASE_SIZES],
        rehearsal_budget_s=1800.,
        # Deterministic unattached-start candidates, tried in order at prepare
        # time against the full scene; every trial (incl. failures) recorded.
        start_q_candidates=[[0., 0., 0., 0., 0., 0.],
                            [3.5702, -1.3263, -1.0965, 3.9564, 1.6234, 0.4522]])


def load_matrix(path):
    matrix = json.loads(Path(path).read_text())
    ids = [row['size_id'] for row in matrix['sizes']]
    dims = [tuple(row['dimensions']) for row in matrix['sizes']]
    if len(ids) != 7 or len(set(ids)) != 7:
        raise ValueError('MATRIX_MUST_HAVE_SEVEN_DISTINCT_SIZES')
    if dict(zip(ids, dims)).get('base_550x400x250') != (.550, .400, .250):
        raise ValueError('MATRIX_BASE_SIZE_MISMATCH')
    for dims_row in dims:
        if min(dims_row) <= 0 or dims_row[0] < dims_row[1]:
            raise ValueError('MATRIX_DIMENSIONS_MUST_KEEP_LONG_EDGE_ALONG_INSERTION')
    if tuple(matrix['seeds']) != (0, 1, 2):
        raise ValueError('MATRIX_SEEDS_MUST_BE_0_1_2')
    if not matrix.get('start_q_candidates') or not all(
            len(q) == 6 for q in matrix['start_q_candidates']):
        raise ValueError('MATRIX_START_CANDIDATES_MUST_BE_SIX_JOINT_VECTORS')
    return matrix


def derive_goal(snapshot, support_ids, dimensions, config):
    """Final centre from support polygons, box size and runtime clearances.

    Row choice is deterministic (most negative row centre); depth keeps the
    box leading edge one aperture margin plus collision margin inside the
    support union's entrance bound. For the base size this reproduces the
    proven archived target exactly; it is derived, never copied.
    """
    selected = [p for p in snapshot.surfaces if p.source_id in support_ids]
    if not selected:
        raise ValueError('NO_SUPPORT_PATCHES')
    rows = {}
    for patch in selected:
        key = round((patch.bounds_xy[1] + patch.bounds_xy[3]) / 2, 6)
        rows.setdefault(key, []).append(patch)
    row_key = min(rows)
    row = rows[row_key]
    ymin = min(p.bounds_xy[1] for p in row)
    ymax = max(p.bounds_xy[3] for p in row)
    xmin = min(p.bounds_xy[0] for p in row)
    support_z = row[0].surface_z_container
    goal_y = (ymin + ymax) / 2
    goal_x = xmin + dimensions[0] / 2 + config.outside_aperture_margin_m + config.collision_margin_m
    goal_z = support_z + dimensions[2] / 2
    rotation = Rotation.from_euler('z', 0., degrees=True).as_matrix() \
        @ Rotation.from_euler('x', np.pi).as_matrix()
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = (goal_x, goal_y, goal_z)
    return pose, dict(row_center_y=row_key, row_ymin=ymin, row_ymax=ymax, row_xmin=xmin,
                      support_z=support_z, goal=(goal_x, goal_y, goal_z))


def build_request(live_record, dimensions, mass_kg, config):
    """Unattached synthetic-fixture request: same OCC, per-size goal and box."""
    source = request_from_record(live_record)
    goal, derivation = derive_goal(source.snapshot, source.support_source_ids,
                                   dimensions, config)
    box = replace(source.box, box_id='eval_synthetic_fixture',
                  dimensions=tuple(dimensions), com_local=(0., 0., 0.),
                  mass_kg=mass_kg, com_source='geometric_center_assumption')
    live = np.array(source.box.container_T_box).reshape(4, 4)
    live_bottom = float(live[2, 3] - np.array(source.box.dimensions)[2] / 2)
    box_pose = live.copy()
    box_pose[2, 3] = live_bottom + dimensions[2] / 2
    box = replace(box, container_T_box=tuple(box_pose.ravel()))
    request = PlacementRequest(
        request_id='eval-synthetic-fixture-unattached-' + 'x'.join(f'{v*1000:.0f}' for v in dimensions),
        box=box, snapshot=source.snapshot, start_q=source.start_q,
        final_box_poses=(tuple(goal.ravel()),),
        support_source_ids=source.support_source_ids, start_attached=False,
        confirmed_box_T_tool=None, frame_id='container_link')
    return request, derivation, float(live_bottom)


def select_start(request, scene_path, candidates):
    """First collision-free unattached start; every trial is recorded."""
    from assets import load
    from full_arm import FullArmScene
    assets = load(repo_root(), scene_path)
    config = UpperConfig.load()
    scene = FullArmScene(assets, request.snapshot, config)
    scene.set_payload(request.box.dimensions, 'world',
                      scene.world_container @ np.array(request.box.container_T_box).reshape(4, 4),
                      allow_support=True)
    trials = []
    for index, q in enumerate(candidates):
        failure = scene.check_state(q, config.collision_margin_m)
        row = dict(candidate_index=index, q=[float(v) for v in q],
                   failure=failure, accepted=failure is None)
        trials.append(row)
        if failure is None:
            return tuple(float(v) for v in q), trials
    raise ValueError('NO_COLLISION_FREE_START_CANDIDATE')


def derived_configs(matrix):
    """Paired configs identical except the prior switch block."""
    base = UpperConfig.load(matrix['planning_profile'])
    overrides = dict(search_budget_s=float(matrix['unit_search_budget_s']),
                     solution_count_target=int(matrix['solution_count_target']),
                     minimum_executable_solutions=int(matrix['minimum_executable_solutions']))
    rows = {}
    for mode in ('baseline', 'prior'):
        prior = dict() if mode == 'baseline' else dict(
            prior_enabled=True, prior_index_path=str(matrix['prior_index']))
        config = replace(base, **overrides, **prior)
        rows[mode] = config
    return rows


def classify_unit(unit_dir, timed_out):
    """Explicit five-way outcome; incomplete captures are infra, not feasibility."""
    unit_dir = Path(unit_dir)
    summary_path = unit_dir / 'summary.json'
    manifest_path = unit_dir / 'manifest.json'
    error_path = unit_dir / 'error.txt'
    if timed_out:
        return 'INFRA_FAILURE', dict(detail='OUTER_WATCHDOG_TIMEOUT')
    if not summary_path.is_file():
        return 'INFRA_FAILURE', dict(detail='NO_PLANNER_SUMMARY',
                                     error=error_path.is_file())
    summary = json.loads(summary_path.read_text())
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    if not manifest.get('capture_complete') or not manifest.get('replay_possible'):
        return 'INFRA_FAILURE', dict(detail='INCOMPLETE_CAPTURE',
                                     reason=summary.get('reason'))
    if summary.get('valid'):
        return 'NOMINAL_VALID', dict(complete_plans=summary.get('complete_plans'))
    if summary.get('geometry_candidates') == 0:
        return 'GEOMETRY_REJECTED', dict(reason=summary.get('reason'))
    rejections = set(summary.get('rejection_counts') or {})
    prior_rejected_all = (summary.get('attempted_actions') == 0
                          and (unit_dir / 'prior_query.json').is_file())
    if prior_rejected_all:
        return 'GEOMETRY_REJECTED', dict(detail='PRIOR_DROPPED_ALL_ACTIONS')
    if summary.get('reason') == 'INITIAL_STATE_INVALID':
        return 'ENDPOINT_REJECTED', dict(reason=summary.get('reason'))
    if rejections and rejections <= ENDPOINT_REASONS:
        return 'ENDPOINT_REJECTED', dict(reasons=sorted(rejections))
    return 'NO_PLAN_WITHIN_BUDGET', dict(reason=summary.get('reason'),
                                         rejections=sorted(rejections))


def run_unit(matrix_path, evidence, size_id, mode, seed, extra_budget=None):
    """One planner unit in-process; the parent enforces the outer watchdog."""
    evidence = Path(evidence).resolve()
    matrix = load_matrix(matrix_path)
    frozen = json.loads((evidence / 'frozen' / size_id / 'request.json').read_text())
    request = request_from_record(frozen)
    unit_dir = evidence / 'units' / size_id / f'{mode}_seed{seed}'
    if unit_dir.exists():
        raise ValueError('UNIT_DIRECTORY_EXISTS: ' + str(unit_dir))
    started = time.monotonic()
    config = derived_configs(matrix)[mode]
    if extra_budget:
        config = replace(config, search_budget_s=float(extra_budget))
    import yaml
    staged = unit_dir.parent / ('.cfg_' + unit_dir.name + '.yaml')
    staged.parent.mkdir(parents=True, exist_ok=True)
    staged.write_text(yaml.safe_dump(asdict(config)))
    try:
        planner = UpperPlacementPlanner(repo_root(), matrix['scene_config'], config_path=staged)
        planner.plan(request, unit_dir, seed=seed)
    finally:
        staged.unlink(missing_ok=True)
        outcome, detail = classify_unit(unit_dir, timed_out=False)
        atomic_json(unit_dir / 'unit_summary.json', dict(
            size_id=size_id, mode=mode, seed=seed, outcome=outcome, detail=detail,
            started_wall=time.time(), elapsed_s=time.monotonic() - started,
            machine=platform.platform(), python=platform.python_version(),
            load_average=[float(v) for v in os.getloadavg()],
            config_revision=config.revision, frozen_request_revision=request.revision))
    return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--mode', choices=('baseline', 'prior', 'paired'))
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--audit', action='store_true')
    parser.add_argument('--unit', nargs=3, metavar=('SIZE_ID', 'MODE', 'SEED'))
    parser.add_argument('--watchdog-s', type=float)
    parser.add_argument('--budget-s', type=float)
    parser.add_argument('--sizes', nargs='*')
    parser.add_argument('--limit', type=int)
    args = parser.parse_args()

    evidence = args.evidence.resolve()
    root = primary_evidence_root()
    if not evidence.is_absolute() or not evidence.is_relative_to(root):
        raise SystemExit('EVIDENCE_MUST_BE_PRIMARY_ABSOLUTE: ' + str(root))
    matrix_path = args.matrix.resolve()
    matrix = load_matrix(matrix_path)

    if args.prepare_only:
        prepare(matrix, matrix_path, evidence)
        return 0
    if args.audit:
        audit(matrix, matrix_path, evidence)
        return 0
    if args.unit:
        size_id, mode, seed = args.unit[0], args.unit[1], int(args.unit[2])
        run_unit(matrix_path, evidence, size_id, mode, seed, args.budget_s)
        return 0
    if not args.mode:
        raise SystemExit('MODE_REQUIRED')
    run_matrix(matrix, matrix_path, evidence, args)
    return 0


def prepare(matrix, matrix_path, evidence):
    """Freeze one request per size and both derived configs; no planning."""
    evidence.mkdir(parents=True, exist_ok=True)
    live_record = json.loads(Path(matrix['live_request']).read_text())
    config = UpperConfig.load(matrix['planning_profile'])
    candidates = [tuple(live_record['start_q'])] + [tuple(q) for q in matrix['start_q_candidates']]
    sizes = matrix['sizes']
    provenance = []
    for row in sizes:
        request, derivation, platform_bottom = build_request(live_record, row['dimensions'],
                                                             row['mass_kg'], config)
        start_q, trials = select_start(request, matrix['scene_config'], candidates)
        request = replace(request, start_q=start_q)
        frozen_dir = evidence / 'frozen' / row['size_id']
        frozen_dir.mkdir(parents=True, exist_ok=False)
        atomic_json(frozen_dir / 'request.json', asdict(request))
        atomic_json(frozen_dir / 'provenance.json', dict(
            size_id=row['size_id'], dimensions=row['dimensions'],
            fixture_kind='synthetic_fixture_unattached_offline',
            goal_derivation=derivation, platform_bottom_z_container=platform_bottom,
            start_selection=trials, selected_start_q=list(start_q),
            snapshot_id=request.snapshot.snapshot_id,
            occupancy_box_count=len(request.snapshot.occupied),
            source_live_request=matrix['live_request'],
            scene_config=matrix['scene_config']))
        provenance.append(dict(size_id=row['size_id'], goal=derivation['goal'],
                               request_revision=request.revision))
    configs = derived_configs(matrix)
    import yaml
    for mode, config in configs.items():
        (evidence / f'config_{mode}.yaml').write_text(yaml.safe_dump(asdict(config)))
    diff = {field: (asdict(configs['baseline']).get(field), asdict(configs['prior']).get(field))
            for field in asdict(configs['baseline'])
            if asdict(configs['baseline']).get(field) != asdict(configs['prior']).get(field)}
    atomic_json(evidence / 'prepare_report.json', dict(
        matrix=str(matrix_path), matrix_digest=digest(json.loads(matrix_path.read_text())),
        sizes=provenance, config_field_diff=diff,
        baseline_revision=configs['baseline'].revision, prior_revision=configs['prior'].revision,
        command=[' '.join(sys.argv)], started_wall=time.time(),
        note='Unattached synthetic fixtures; goals derived from support polygons; '
             'requests frozen before pairing; no planning performed.'))
    print(json.dumps(dict(prepared=len(provenance), config_diff=diff), indent=1))


def unit_sizes_and_seeds(matrix, args):
    sizes = [row['size_id'] for row in matrix['sizes']]
    if getattr(args, 'sizes', None):
        order = {name: index for index, name in enumerate(sizes)}
        sizes = sorted((s for s in args.sizes if s in order), key=order.__getitem__)
    seeds = matrix['seeds']
    return sizes, seeds


def run_matrix(matrix, matrix_path, evidence, args):
    """Sequential units in matrix/seed/mode order under the total budget."""
    import signal
    started = time.monotonic()
    deadline = started + float(matrix['total_budget_s'])
    watchdog = float(args.watchdog_s or matrix['watchdog_s'])
    modes = ('baseline', 'prior') if args.mode == 'paired' else (args.mode,)
    sizes, seeds = unit_sizes_and_seeds(matrix, args)
    results = []
    ran = 0
    for size_id in sizes:
        for seed in seeds:
            for mode in modes:
                if time.monotonic() >= deadline:
                    atomic_json(evidence / 'results.json', dict(units=results, stopped='TOTAL_BUDGET'))
                    raise SystemExit('TOTAL_BUDGET_EXHAUSTED')
                unit_dir = evidence / 'units' / size_id / f'{mode}_seed{seed}'
                if (unit_dir / 'unit_summary.json').is_file():
                    results.append(json.loads((unit_dir / 'unit_summary.json').read_text()))
                    continue
                wall_started = time.monotonic()
                command = [sys.executable, str(Path(__file__).resolve()), '--matrix', str(matrix_path),
                           '--evidence', str(evidence), '--unit', size_id, mode, str(seed)]
                if args.budget_s:
                    command += ['--budget-s', str(args.budget_s)]
                process = subprocess.Popen(
                    command, cwd=str(Path(__file__).resolve().parent), stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, text=True, start_new_session=True)
                try:
                    process.communicate(timeout=watchdog)
                    timed_out = False
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.communicate()
                    timed_out = True
                summary_path = unit_dir / 'unit_summary.json'
                if not summary_path.is_file():
                    if timed_out:
                        detail = dict(detail='OUTER_WATCHDOG_TIMEOUT')
                    else:
                        detail = dict(detail='UNIT_PROCESS_FAILED', returncode=process.returncode)
                    atomic_json(summary_path, dict(
                        size_id=size_id, mode=mode, seed=seed, outcome='INFRA_FAILURE',
                        detail=detail, started_wall=time.time(),
                        elapsed_s=time.monotonic() - wall_started,
                        machine=platform.platform(), python=platform.python_version(),
                        load_average=[float(v) for v in os.getloadavg()],
                        config_revision=None, frozen_request_revision=None))
                row = json.loads(summary_path.read_text())
                results.append(row)
                atomic_json(evidence / 'results.json', dict(units=results, stopped=None))
                ran += 1
                if args.limit and ran >= args.limit:
                    write_reports(matrix, evidence)
                    return
    write_reports(matrix, evidence)


def collect_units(evidence, sizes, seeds, modes=('baseline', 'prior')):
    units = {}
    for size_id in sizes:
        for seed in seeds:
            for mode in modes:
                unit_dir = evidence / 'units' / size_id / f'{mode}_seed{seed}'
                path = unit_dir / 'unit_summary.json'
                units[(size_id, seed, mode)] = (json.loads(path.read_text())
                                                if path.is_file() else None)
    return units


def write_reports(matrix, evidence):
    sizes = [row['size_id'] for row in matrix['sizes']]
    dims = {row['size_id']: row['dimensions'] for row in matrix['sizes']}
    seeds = matrix['seeds']
    units = collect_units(evidence, sizes, seeds)
    rows = []
    for (size_id, seed, mode), row in sorted(units.items()):
        rows.append(dict(size_id=size_id, dimensions=dims[size_id], seed=seed, mode=mode,
                         outcome=row['outcome'] if row else 'not_evaluated',
                         elapsed_s=row['elapsed_s'] if row else None))
    atomic_json(evidence / 'results.json', dict(units=rows, stopped=None))
    with (evidence / 'results.csv').open('w') as stream:
        stream.write('size_id,dimensions,seed,mode,outcome,elapsed_s\n')
        for row in rows:
            stream.write(f"{row['size_id']},{'x'.join(str(v) for v in row['dimensions'])},"
                         f"{row['seed']},{row['mode']},{row['outcome']},{row['elapsed_s']}\n")
    capability = []
    for size_id in sizes:
        entry = dict(size_id=size_id, dimensions=dims[size_id])
        for mode in ('baseline', 'prior'):
            rows = [units.get((size_id, seed, mode)) for seed in seeds]
            outcomes = [row['outcome'] if row else 'not_evaluated' for row in rows]
            entry[mode] = dict(
                outcomes=outcomes,
                nominal_valid_count=outcomes.count('NOMINAL_VALID'),
                geometry_rejected=outcomes.count('GEOMETRY_REJECTED'),
                endpoint_rejected=outcomes.count('ENDPOINT_REJECTED'),
                no_plan=outcomes.count('NO_PLAN_WITHIN_BUDGET'),
                infra_failure=outcomes.count('INFRA_FAILURE'),
                not_evaluated=outcomes.count('not_evaluated'))
        capability.append(entry)
    atomic_json(evidence / 'capability_table.json', capability)
    with (evidence / 'capability_table.csv').open('w') as stream:
        stream.write('size_id,dimensions,'
                     'baseline_outcomes,prior_outcomes,'
                     'baseline_valid,prior_valid,pairs_both_valid,pairs_baseline_only\n')
        for entry in capability:
            pairs = [(entry['baseline']['outcomes'][i], entry['prior']['outcomes'][i])
                     for i in range(len(seeds))]
            stream.write(f"{entry['size_id']},{'x'.join(str(v) for v in entry['dimensions'])},"
                         f"\"{'|'.join(entry['baseline']['outcomes'])}\",\"{'|'.join(entry['prior']['outcomes'])}\","
                         f"{entry['baseline']['nominal_valid_count']},{entry['prior']['nominal_valid_count']},"
                         f"{sum(b == p == 'NOMINAL_VALID' for b, p in pairs)},"
                         f"{sum(b == 'NOMINAL_VALID' and p != 'NOMINAL_VALID' for b, p in pairs)}\n")
    pairs = []
    for size_id in sizes:
        for seed in seeds:
            baseline = units.get((size_id, seed, 'baseline'))
            prior = units.get((size_id, seed, 'prior'))
            b_out = baseline['outcome'] if baseline else 'not_evaluated'
            p_out = prior['outcome'] if prior else 'not_evaluated'
            pairs.append(dict(size_id=size_id, seed=seed, baseline=b_out, prior=p_out,
                              baseline_elapsed_s=baseline['elapsed_s'] if baseline else None,
                              prior_elapsed_s=prior['elapsed_s'] if prior else None,
                              regression=(b_out == 'NOMINAL_VALID' and p_out != 'NOMINAL_VALID'),
                              both_valid=(b_out == 'NOMINAL_VALID' and p_out == 'NOMINAL_VALID')))
    both = [p for p in pairs if p['both_valid']]
    both_times = sorted(p['baseline_elapsed_s'] for p in both) or []
    prior_times = sorted(p['prior_elapsed_s'] for p in both) or []
    median = lambda values: (values[len(values) // 2]
                             if len(values) % 2 else (values[len(values) // 2 - 1]
                                                      + values[len(values) // 2]) / 2) if values else None
    base_pairs = [p for p in pairs if p['size_id'] == 'base_550x400x250']
    speed = (dict(double_success_pairs=len(both),
                  baseline_median_s=median(both_times), prior_median_s=median(prior_times),
                  scope='wall time of double-success pairs only; medians, no significance test')
             if len(both) >= 3 else
             dict(double_success_pairs=len(both), comparison='not_evaluated',
                  note='fewer than three double-success pairs; raw timings retained'))
    report = dict(
        pairs=pairs,
        gates=dict(
            every_baseline_success_has_prior_success=not any(p['regression'] for p in pairs),
            regressions=[f"{p['size_id']}/seed{p['seed']}" for p in pairs if p['regression']],
            base_size_paired_successes=sum(p['both_valid'] for p in base_pairs),
            base_size_requires=3,
            all_double_success_pairs=sum(p['both_valid'] for p in pairs)),
        speed=speed,
        scope='Paired AND over this frozen matrix only; no production per-request '
              'success-rate or reliability claim. Timeouts/rejections are reported '
              'separately from double-success medians.')
    atomic_json(evidence / 'paired_report.json', report)
    print(json.dumps(dict(units=len(rows), double_success=len(both),
                          regressions=report['gates']['regressions']), indent=1))


def audit(matrix, matrix_path, evidence):
    """Completeness, hash linkage and gate recomputation; no text matching."""
    sizes = [row['size_id'] for row in matrix['sizes']]
    seeds = matrix['seeds']
    issues = []
    prepare_path = evidence / 'prepare_report.json'
    expected_configs = (json.loads(prepare_path.read_text()) if prepare_path.is_file() else {})
    expected_revisions = dict(baseline=expected_configs.get('baseline_revision'),
                              prior=expected_configs.get('prior_revision'))
    for size_id in sizes:
        frozen = evidence / 'frozen' / size_id / 'request.json'
        if not frozen.is_file():
            issues.append(f'missing frozen request: {size_id}')
            continue
        frozen_record = json.loads(frozen.read_text())
        for seed in seeds:
            for mode in ('baseline', 'prior'):
                unit_dir = evidence / 'units' / size_id / f'{mode}_seed{seed}'
                summary = unit_dir / 'unit_summary.json'
                if not summary.is_file():
                    issues.append(f'not_evaluated: {size_id}/{mode}/seed{seed}')
                    continue
                row = json.loads(summary.read_text())
                planner_request = unit_dir / 'request.json'
                if planner_request.is_file():
                    unit_record = json.loads(planner_request.read_text())
                    if digest(unit_record) != digest(frozen_record):
                        issues.append(f'request drift: {size_id}/{mode}/seed{seed}')
                manifest_path = unit_dir / 'manifest.json'
                manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
                if expected_revisions.get(mode) and manifest.get('config') \
                        and digest(manifest['config']) != expected_revisions[mode]:
                    issues.append(f'config drift: {size_id}/{mode}/seed{seed}')
                if row['outcome'] != 'INFRA_FAILURE':
                    for name in REQUIRED_UNIT_FILES:
                        if not (unit_dir / name).is_file():
                            issues.append(f'missing {name}: {size_id}/{mode}/seed{seed}')
                    if not manifest.get('capture_complete') or not manifest.get('replay_possible'):
                        issues.append(f'incomplete capture: {size_id}/{mode}/seed{seed}')
                outcome, detail = classify_unit(unit_dir, timed_out=False)
                if row['outcome'] != outcome:
                    issues.append(f'classification drift: {size_id}/{mode}/seed{seed}'
                                  f' recorded={row["outcome"]} recomputed={outcome}')
                if mode == 'prior' and not (unit_dir / 'prior_query.json').is_file():
                    issues.append(f'missing prior evidence: {size_id}/{mode}/seed{seed}')
    write_reports(matrix, evidence)
    report = json.loads((evidence / 'paired_report.json').read_text())
    atomic_json(evidence / 'audit.json', dict(
        matrix=str(matrix_path), units_checked=len(sizes) * len(seeds) * 2, issues=issues,
        gates=report['gates'], speed=report['speed'],
        capture_gate='non-infra units require full files and capture_complete/replay_possible',
        verdict='PASS' if not issues else 'ISSUES'))
    print(json.dumps(dict(issues=len(issues), verdict='PASS' if not issues else 'ISSUES'), indent=1))


if __name__ == '__main__':
    raise SystemExit(main())
