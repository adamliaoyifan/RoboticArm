"""Audit retained evidence only; never feed simulator references to production.

Functional execution and physical-envelope qualification are separate verdicts.
An execution success cannot override an observed containment violation.
"""
import collections
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


base = Path(__file__).resolve().parent
workspace = Path.cwd()
root = base / 'simulation_01'
read = lambda path: json.loads(path.read_text())
sha = lambda data: hashlib.sha256(data).hexdigest()
run = read(root / 'run_manifest.json')
driver = read(root / 'cycle/manifest.json')
review = read(root / 'three_box_review.json')
containment = read(root / 'payload_containment_eval_v5.json')
frozen = read(base / 'build_01/manifest.json')
launch = read(root / 'run/launch.json')
verified = collections.Counter()
errors = []


def verify_rows(folder, rows, label):
    for row in rows:
        if not row.get('file'):
            continue
        path = folder / row['file']
        stored = path.read_bytes()
        if row.get('stored_sha256') and sha(stored) != row['stored_sha256']:
            errors.append(str(path) + ':stored_sha256')
        raw = gzip.decompress(stored) if path.suffix == '.gz' else stored
        if row.get('sha256') and sha(raw) != row['sha256']:
            errors.append(str(path) + ':sha256')
        if 'bytes' in row and len(raw) != row['bytes']:
            errors.append(str(path) + ':bytes')
        verified[label] += 1


verify_rows(root / 'cycle', driver['records'], 'driver')
prepare = root / 'run/prepare'
boundaries = [json.loads(line) for line in (prepare / 'boundaries.jsonl').read_text().splitlines()]
verify_rows(prepare, boundaries, 'prepare')
promotions = []
for manifest in sorted((root / 'run/execution').glob('*/manifest.json')):
    execution = read(manifest)
    verify_rows(manifest.parent, execution['records'], 'execution')
    for row in execution['records']:
        if 'successful_path_promotion' in row['file']:
            promotions.append(json.loads(gzip.decompress((manifest.parent / row['file']).read_bytes())))
for filename in ('manifest.json', 'contacts_manifest.json'):
    verify_rows(root / 'reference', read(root / 'reference' / filename)['records'], 'reference')

source_changes = []
for name, digest in frozen['source_after'].items():
    path = workspace / name
    if not path.is_file() or sha(path.read_bytes()) != digest:
        source_changes.append(name)
install_changes = []
for row in frozen['matched']:
    path = workspace / row['install']
    if not path.is_file() or sha(path.read_bytes()) != frozen['source_after'][row['source']]:
        install_changes.append(row)

owned_digests = {box['plan_digest'] for box in driver['completed_boxes']}
cache_rows = []
cache_dir = root / 'successful_path_cache'
cache_dir.mkdir(exist_ok=True)
for path in (Path.home() / '.ros/luggage_successful_paths').glob('*.json.gz'):
    with gzip.open(path, 'rb') as stream:
        raw = stream.read(8*1024*1024 + 1)
    record = json.loads(raw)
    if record.get('source_plan_digest') not in owned_digests:
        continue
    assert set(record) == {'schema', 'key', 'source_plan_digest', 'source_geometry_digest',
                           'source_acquisition_stamp_ns', 'legs'}
    target = cache_dir / path.name
    shutil.copyfile(path, target)
    cache_rows.append(dict(file=str(target.relative_to(root)), stored_sha256=sha(target.read_bytes()),
                           source_plan_digest=record['source_plan_digest'],
                           source_geometry_digest=record['source_geometry_digest'],
                           source_acquisition_stamp_ns=record['source_acquisition_stamp_ns'],
                           legs=len(record['legs']), execution_authorization_stored=False))

contact = read(root / 'reference/contact_stream/summary.json')
objects = sorted({tuple(row['objects']) for row in contact['pairs']})
robot_contacts = [pair for pair in objects if any('elfin' in name for name in pair)]
owned_processes = []
for line in subprocess.check_output(['ps', '-eo', 'pid=,pgid=,args='], text=True).splitlines():
    pid, pgid, command = line.strip().split(None, 2)
    if int(pgid) == run['launch_pid']:
        owned_processes.append(dict(pid=int(pid), pgid=int(pgid), command=command))
envelopes = []
for box in containment['boxes']:
    checks = box['checks']
    maxima = [max(row['maximum_corner_excess_by_axis_m'][axis] for row in checks)
              for axis in range(3)] if checks else None
    envelopes.append(dict(instance=box['instance'], samples=len(checks),
        outside_samples=sum(not row['all_corners_inside'] for row in checks),
        maximum_corner_excess_by_axis_m=maxima,
        actual_collision_aabb_dimensions=box['actual_collision_aabb_dimensions'],
        measured_dimensions=box['measured_dimensions'], envelope_dimensions=box['envelope_dimensions']))

physical_pass = all(box['samples'] > 0 and box['outside_samples'] == 0 for box in envelopes)
functional_pass = (run['outcome'] == 'PASS' and driver['outcome'] == 'THREE_BOX_PASS'
    and len(driver['completed_boxes']) == 3 and all(box['success'] for box in driver['completed_boxes'])
    and [box['task_mode'] for box in driver['completed_boxes']] == ['floor', 'floor', 'stack'])
summary = dict(
    revision=run['revision'], dirty_inputs_preserved=True,
    functional_three_box='PASS' if functional_pass else 'FAIL',
    physical_payload_containment='PASS' if physical_pass else 'FAIL',
    physical_qualification='NOT_PASSED' if not physical_pass else 'WITNESS_ONLY',
    statistical_reliability='NOT_EVALUATED',
    source_files=len(frozen['source_after']), installed_matches=len(frozen['matched']),
    source_changes_after_freeze=source_changes, installed_changes_after_freeze=install_changes,
    source_change_scope='Two owned test updates; additional concurrent workspace cleanup after '
        'first audit. See integration_audit_first.json. Installed runtime artifacts unchanged.',
    verified_artifact_counts=dict(verified), integrity_errors=errors,
    model_sha256=launch['model_sha256'], scene_revision=launch['scene_revision'],
    atlas_status=launch['operation_atlas']['status'],
    mapping_backend=launch['map_owner']['backend'], resolution_m=launch['map_owner']['resolution_m'],
    cycle_stages=driver['stages'], completed_boxes=driver['completed_boxes'],
    promoted_paths=promotions, retained_cache_proposals=cache_rows,
    cache_hit_speedup='NOT_EVALUATED; actual promotions plus deterministic reuse/refusal/fallback tests only',
    envelope_audit=envelopes,
    contact_scope='Captured contact stream; not a proof of no unobserved contacts.',
    contact_messages=contact['messages'], captured_contact_pairs=objects,
    captured_robot_contact_pairs=robot_contacts, contact_error=contact['error'],
    teardown_returncode=run['teardown_returncode'],
    residual_owned_processes=owned_processes,
    lidar_source_qualification='NOT_EVALUATED; RGB-D/O integration and separate native LiDAR-label wire gate',
    raw_perception_failure_window='MISSING for boxes 2/3; cannot replay segmentation/fit from this run',
    containment_replay_possible=True,
    next_investigation='Capture synchronized raw sensor/mask/cargo-cloud and pre/post history geometry; '
        'replay whole-body footprint estimation separately from seal support. Do not tune against simulator truth.')
assert not errors, errors
assert not install_changes, install_changes
assert functional_pass, 'three-box functional witness missing'
assert len(cache_rows) == len(promotions) == 3
assert not owned_processes, owned_processes
(root / 'integration_audit.json').write_text(json.dumps(summary, indent=2) + '\n')
failure = dict(case='simulation_01 boxes 2/3', first_observed_failure='offline physical payload containment',
    revision=run['revision'], boundary='frozen attachment geometry versus sampled actual collision vertices',
    evidence=['payload_containment_eval_v5.json', 'three_box_review.json', 'run/execution/',
              'run/prepare/boundaries.jsonl', 'reference/manifest.json', 'effective_sources.zip'],
    capture_window='Loaded leg 2 through pre-release, phase/10-second reference samples',
    missing_items=['Synchronized raw RGB-D/segmentation/cargo-cloud for boxes 2/3'],
    capture_complete=False, boundary_capture_complete=True, replay_possible=True,
    perception_replay_possible=False,
    replay_scope='Containment boundary only; perception fit root-cause replay incomplete',
    inspection='source install-production/setup.bash; python3 ' + str(base.relative_to(workspace) /
        'review_payload_containment_v5.py') + ' ' + str(root.relative_to(workspace)))
(root / 'containment_failure_manifest.json').write_text(json.dumps(failure, indent=2) + '\n')
print(json.dumps({k: summary[k] for k in ('functional_three_box', 'physical_payload_containment',
    'verified_artifact_counts', 'source_changes_after_freeze', 'installed_changes_after_freeze',
    'captured_robot_contact_pairs', 'teardown_returncode')}))
