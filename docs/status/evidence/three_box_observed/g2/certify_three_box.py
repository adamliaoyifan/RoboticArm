"""Offline acceptance audit; all simulator references remain outside online inputs."""
import gzip
import hashlib
import json
import re
import runpy
import sys
from pathlib import Path

import numpy as np

root = Path(sys.argv[1])
helpers = Path(__file__).parent
audit = runpy.run_path(str(helpers / 'review_three_box.py'))
review = json.loads((root / 'three_box_review.json').read_text())
driver = json.loads((root / 'cycle/manifest.json').read_text())
run = json.loads((root / 'run_manifest.json').read_text())
containment = json.loads((root / 'payload_containment_eval_v4.json').read_text())
final_o = json.loads((root / 'final_o_export.json').read_text())
stream = json.loads((root / 'reference/contact_stream/summary.json').read_text())
failures = []

def require(condition, reason):
    if not condition:
        failures.append(reason)

require(driver.get('outcome') == 'THREE_BOX_PASS', 'DRIVER_NOT_THREE_BOX_PASS')
require(driver.get('capture_complete') and driver.get('replay_possible'), 'INCOMPLETE_CAPTURE')
require(run.get('outcome') == 'PASS' and run.get('teardown_returncode') == 0, 'RUN_OR_TEARDOWN_FAILED')
completed = driver.get('completed_boxes', [])
require([b['task_mode'] for b in completed] == ['floor', 'floor', 'stack'], 'WRONG_WORKLOAD')
require(len({b['map_epoch'] for b in completed}) == 1, 'MAP_EPOCH_CHANGED')
require(len(review['boxes']) == 3, 'MISSING_BOX_EVIDENCE')
checked_files = 0
for folder in [root / 'cycle', root / 'run/prepare', *sorted((root / 'run/execution').glob('*'))]:
    manifest = folder / 'manifest.json'
    if manifest.exists():
        rows = json.loads(manifest.read_text()).get('records', [])
    elif (folder / 'boundaries.jsonl').exists():
        rows = [json.loads(line) for line in (folder / 'boundaries.jsonl').read_text().splitlines()]
    else:
        continue
    for row in rows:
        if not row.get('file') or not row.get('sha256'):
            continue
        stored = (folder / row['file']).read_bytes()
        if row.get('stored_sha256'):
            require(hashlib.sha256(stored).hexdigest() == row['stored_sha256'], 'STORED_HASH:' + row['file'])
        raw = gzip.decompress(stored) if row['file'].endswith('.gz') else stored
        require(hashlib.sha256(raw).hexdigest() == row['sha256'], 'RAW_HASH:' + row['file'])
        checked_files += 1

per_box = []
for index, box in enumerate(review['boxes']):
    result = box.get('result', {})
    require(result.get('success') and result.get('motion_terminal') and not result.get('may_have_payload'), 'ACTION:' + box['instance_id'])
    require(box['release_dynamic'] is True, 'NOT_DYNAMICALLY_RELEASED:' + box['instance_id'])
    require(len(box['legs']) == 6 and all(leg['points'] > 0 for leg in box['legs']), 'MISSING_TRAJECTORY:' + box['instance_id'])
    require(any(v['coverage']['accepted'] for v in box['observed_views']), 'NO_OBSERVED_RELEASE:' + box['instance_id'])
    height = box['target_bottom_above_floor_m']
    require(abs(height) <= review['resolution_m'] if index < 2 else height > review['resolution_m'], 'SUPPORT_HEIGHT:' + box['instance_id'])
    physical = box.get('actual_physics')
    require(physical and physical['last_samples'] >= 1, 'NO_PHYSICAL_SAMPLE:' + box['instance_id'])
    envelope = next((b for b in containment['boxes'] if b['instance'] == box['instance_id']), None)
    require(envelope and envelope['checks'] and all(c['all_corners_inside'] for c in envelope['checks']), 'ENVELOPE:' + box['instance_id'])
    folder = root / 'run/execution' / box['execution']
    rows = json.loads((folder / 'manifest.json').read_text())['records']
    def json_rows(tag):
        return [json.loads(gzip.decompress((folder / r['file']).read_bytes())) for r in rows if tag in r['file']]
    outcomes = json_rows('strict_motion_outcome.json')
    for leg_index in range(6):
        require(any(v['index'] == leg_index and v['submitted'] and v['success'] and v['terminal'] for v in outcomes), 'LEG_RESULT:' + box['instance_id'] + ':' + str(leg_index))
    suffix = json_rows('native_suffix_verdict.json')
    require(any(v['accepted'] and v['native_validation_id'] for v in suffix), 'NO_NATIVE_CERTIFICATE:' + box['instance_id'])
    world_box = audit['mat'](physical['world_xyz'], physical['world_quaternion_xyzw'])
    actual_container = np.linalg.inv(audit['world_container']) @ world_box
    per_box.append(dict(instance=box['instance_id'], support_height_above_floor_m=height,
        actual_centre_container=actual_container[:3, 3].tolist(),
        requested_centre_container=box['target_xyz_container'],
        actual_minus_requested_centre_m=(actual_container[:3, 3] - box['target_xyz_container']).tolist(),
        settled_sample_span_m=physical['position_span_m'], settled_sample_interval_s=physical['sample_interval_s'],
        observed_ratios=[v['coverage']['observed_ratio'] for v in box['observed_views']],
        envelope_samples=len(envelope['checks']) if envelope else 0,
        native_suffix_checks=len(suffix), native_suffix_initial_refusals=[v for v in suffix if not v['accepted']],
        controller_leg_successes=6,
        source_refresh_before_submission=[v for v in outcomes if not v['submitted']],
        original_trajectory_csvs=[str(folder / (leg['name'] + '_trajectory.csv')) for leg in box['legs']]))

third = review['boxes'][2]['instance_id']
older = {b['instance_id'] for b in review['boxes'][:2]}
support_pairs = [p for p in stream['pairs'] if ':COMPLETE:' in p['phase']
    and any(third + '::' in obj for obj in p['objects'])
    and any(any(name + '::' in obj for name in older) for obj in p['objects'])]
require(support_pairs, 'NO_ACTUAL_OLDER_CARGO_SUPPORT_CONTACT')
require(not stream.get('error') and stream['messages'] > 0, 'CONTACT_CAPTURE_INCOMPLETE')
arm_load_pairs = [p for p in stream['pairs'] if any('pickup_box' in obj for obj in p['objects'])
    and any('elfin' in obj or 'suction' in obj for obj in p['objects'])]
require(not arm_load_pairs, 'RECORDED_ARM_LOAD_CONTACT')
require(final_o['source']['epoch'] == completed[-1]['map_epoch'], 'FINAL_O_EPOCH_MISMATCH')
require(final_o['source']['map_revision'] >= completed[-1]['map_revision'], 'STALE_FINAL_O')
atlas = json.loads((root / 'atlas_consumption_review.json').read_text())
require(len(atlas['boxes']) == 3 and all(b['qualified'] for b in atlas['boxes']), 'UNQUALIFIED_ATLAS')
support_z = []
for pair in support_pairs:
    text = gzip.decompress((root / 'reference/contact_stream' / pair['example']).read_bytes()).decode()
    for block in text.split('\ncontact {\n')[1:]:
        names = re.findall(r'name: "([^"]+)"', block)[:2]
        if set(names) == set(pair['objects']):
            for position in re.findall(r'\n  position \{(.*?)\n  \}', block, re.S):
                match = re.search(r'\bz: ([^\s]+)', position)
                if match:
                    support_z.append(float(match[1]))
result = dict(outcome='PASS' if not failures else 'FAIL', scope=__doc__,
    reliability_scope='One AND of three dependent functional units; no statistical reliability claim.',
    frozen_revision=run['revision'], archived_dirty_inputs=len(run['source_hashes']),
    verified_typed_files=checked_files, failures=failures, boxes=per_box,
    qualified_atlas_records='atlas_consumption_review.json',
    actual_cargo_support_pairs=support_pairs,
    support_contact_world_z_range_m=[min(support_z), max(support_z)] if support_z else None,
    contact_messages=stream['messages'], recorded_arm_load_pairs=arm_load_pairs,
    final_o_source=final_o['source'], timings=driver['stages'], teardown_returncode=run['teardown_returncode'])
(root / 'acceptance_audit.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(dict(outcome=result['outcome'], verified_typed_files=checked_files, failures=failures,
    physical_support_pair_count=len(support_pairs), support_contact_world_z_range_m=result['support_contact_world_z_range_m']), indent=2))
sys.exit(bool(failures))
