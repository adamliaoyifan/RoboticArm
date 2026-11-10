#!/usr/bin/env python3
"""Validate and summarize profiler raw records; never certify product O5 from T3.

Each input file is a separate window. Warmups remain in raw_samples.jsonl but do
not enter the statistics. Do not pool windows or sum percentile stage values.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import yaml

EXCLUSIVE = ('lock_wait', 'validation', 'fingerprint', 'dedup', 'depth_filter',
             'transform', 'ray_preparation', 'fusion', 'finalization')


def stats(values):
    if any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError("invalid metric sample")
    values = sorted(values)
    if not values:
        return {'n': 0}
    return {'n': len(values), 'p50': values[math.ceil(.50*len(values))-1],
            'p95': values[math.ceil(.95*len(values))-1],
            'p99': values[math.ceil(.99*len(values))-1], 'max': values[-1]}


def validate(row):
    if row['schema'] != 'observer_profile_v1' or row['status'] != 'OK':
        raise ValueError('invalid schema or failed sample')
    receipt = row['receipt']
    profile = receipt['profile']
    if profile['mode'] != row['mode']:
        raise ValueError('mode mismatch')
    if not profile['enabled']:
        return
    durations = profile['stage_ms']
    if any(not math.isfinite(value) or value < 0 for value in durations.values()):
        raise ValueError('invalid duration')
    if sum(durations[key] for key in EXCLUSIVE) > receipt['total_ms'] + .1:
        raise ValueError('exclusive spans exceed total')
    if durations['ray_generation']+durations['ray_filter_dedup'] > durations['ray_preparation']+.1:
        raise ValueError('ray child spans exceed parent')
    if durations['tree_update']+durations['block_maintenance'] > durations['fusion']+.1:
        raise ValueError('fusion child spans exceed parent')
    if receipt['status'] == 'INTEGRATED':
        if profile['ray_keys_generated'] != profile['active_ray_key_visits']+profile['inactive_ray_key_visits']:
            raise ValueError('ray key counts do not close')
        if profile['rays_attempted'] != profile['rays_with_active_keys']+profile['rays_without_active_keys']:
            raise ValueError('ray counts do not close')
        if profile['free_insert_attempts'] != profile['free_insert_duplicates']+profile['free_unique_before_hit_removal']:
            raise ValueError('dedup counts do not close')
    second = row.get('surface_second_call', {})
    if second.get('profile_enabled') and (not second['profile']['cache_hit'] or second['profile']['scanned_cells']):
        raise ValueError('second same-snapshot surface query was not a cache hit')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', nargs='+', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    output = Path(args.out)
    output.mkdir(parents=True, exist_ok=False)
    report = {'product_acceptance': 'not_evaluated', 'windows': [], 'errors': []}
    insufficient = False
    with (output/'raw_samples.jsonl').open('w') as raw:
        for name in args.samples:
            path = Path(name).resolve()
            try:
                with path.open() as source:
                    rows = []
                    indices = set()
                    for row in yaml.safe_load_all(source):
                        row['sample_file'] = str(path)
                        raw.write(json.dumps(row, allow_nan=False)+'\n')
                        validate(row)
                        if row['index'] in indices:
                            raise ValueError('duplicate sample index')
                        indices.add(row['index'])
                        rows.append(row)
                if len({(r['mode'],r['case_id'],r['manifest_sha256']) for r in rows}) != 1:
                    raise ValueError('window mixed modes/cases/manifests or empty window')
                scored = [r for r in rows if not r['warmup']]
                window = {'source': str(path), 'n': len(scored), 'mode': rows[0]['mode'],
                          'case_id': rows[0]['case_id'], 'metrics': {}, 'stage_fraction': {}}
                for key in ('integrate_external_ms','integrate_process_cpu_ms','acquire_ms','roi_ms',
                            'input_load_parse_ms','sample_compute_ms','checkpoint_digest_ms'):
                    window['metrics'][key] = stats([r[key] for r in scored if key in r])
                for key in ('surface_first_call','surface_second_call'):
                    window['metrics'][key+'_external_ms'] = stats([r[key]['external_ms'] for r in scored if key in r])
                for label, want_hit in (('actual_surface_cache_hit_ms', True), ('actual_surface_build_ms', False)):
                    window['metrics'][label] = stats([
                        r[key]['external_ms'] for r in scored for key in ('surface_first_call','surface_second_call')
                        if r.get(key,{}).get('profile_enabled') and r[key]['profile']['cache_hit']==want_hit])
                enabled = [r for r in scored if r['receipt']['profile']['enabled']]
                if enabled:
                    window['exterior_key_fraction'] = stats([
                        r['receipt']['profile']['inactive_ray_key_visits']/r['receipt']['profile']['ray_keys_generated']
                        for r in enabled if r['receipt']['profile']['ray_keys_generated']])
                    window['free_duplicate_fraction'] = stats([
                        r['receipt']['profile']['free_insert_duplicates']/r['receipt']['profile']['free_insert_attempts']
                        for r in enabled if r['receipt']['profile']['free_insert_attempts']])
                    for stage in enabled[0]['receipt']['profile']['stage_ms']:
                        window['metrics'][stage+'_ms'] = stats([r['receipt']['profile']['stage_ms'][stage] for r in enabled])
                        window['stage_fraction'][stage] = stats([
                            r['receipt']['profile']['stage_ms'][stage]/r['receipt']['total_ms']
                            for r in enabled if r['receipt']['total_ms']>0])
                window['sampling_status'] = 'sufficient' if len(scored)>=30 else 'not_evaluated'
                insufficient |= len(scored)<30
                report['windows'].append(window)
            except (KeyError, TypeError, ValueError, OSError, yaml.YAMLError) as error:
                report['errors'].append({'source': str(path), 'reason': str(error)})
    # Flatten already validated raw rows; a partial/failed input stays in JSONL
    # for diagnosis but must not silently enter a valid timing summary.
    counts = ('rays_attempted','ray_keys_generated','active_ray_key_visits',
              'inactive_ray_key_visits','rays_with_active_keys','rays_without_active_keys',
              'free_insert_attempts','free_insert_duplicates','free_unique_before_hit_removal',
              'copied_blocks','copied_cells')
    identity = ('sample_file','case_id','index','mode','warmup')
    with (output/'WORKLOAD_COUNTS.csv').open('w') as counts_file, (output/'STAGE_TIMINGS.csv').open('w') as times_file:
        cw = csv.DictWriter(counts_file, fieldnames=identity+('profile_enabled','raw_points','accepted_points','free_keys','hit_keys','changed_keys','rebuilt_blocks')+counts)
        tw = csv.DictWriter(times_file, fieldnames=identity+('stage','visited','ms','parent'))
        cw.writeheader(); tw.writeheader()
        for line in (output/'raw_samples.jsonl').read_text().splitlines():
            row = json.loads(line)
            try:
                validate(row)
            except (KeyError, TypeError, ValueError):
                continue
            receipt = row['receipt']; profile = receipt['profile']
            base = {key:row[key] for key in identity}
            cw.writerow(dict(base, profile_enabled=profile['enabled'],
                **{key:receipt[key] for key in ('raw_points','accepted_points','free_keys','hit_keys','changed_keys','rebuilt_blocks')},
                **{key:profile[key] for key in counts}))
            if profile['enabled']:
                for stage, value in profile['stage_ms'].items():
                    parent = 'total' if stage in EXCLUSIVE else 'ray_preparation' if stage.startswith('ray_') else 'fusion'
                    tw.writerow(dict(base, stage=stage, visited=profile['stage_visited'][stage], ms=value,parent=parent))
    (output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    return 3 if report['errors'] else 4 if insufficient else 0


if __name__ == '__main__':
    sys.exit(main())
