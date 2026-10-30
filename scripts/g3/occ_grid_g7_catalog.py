#!/usr/bin/env python3
"""Build the gen7 2/5cm CASE_CATALOG and RESOLUTION_SCOPE from the gen6 catalog.

Filters every list by NUMERIC resolution (0.02/0.05 kept, 0.01 excluded per
the user-confirmed 2cm representative target, plan 9b7d465b). The oracle
filter streams the 180000 per-row keys and counts exclusions row by row —
the 120000 result is proven against the journal, not by formula. Any
unparseable id, count drift, duplicate, or missing id aborts the build.
"""
import argparse
import json
import re
from pathlib import Path

KEPT = (0.02, 0.05)
EXCLUDED = (0.01,)
ID_RESOLUTION = re.compile(r'^r(\d+(?:\.\d+)?)_')
EXPECTED = dict(oracle_kept=120000, oracle_excluded=60000, capture_kept=132,
                capture_excluded=66, response_kept=408, response_excluded=204,
                writes_kept=48, writes_excluded=24, placement_kept=12,
                placement_excluded=6)


def resolution_of(item_id):
    match = ID_RESOLUTION.match(item_id)
    if not match:
        raise RuntimeError('unparseable resolution id: '+item_id)
    return float(match.group(1))


def split_ids(ids, what):
    kept, excluded = [], []
    for item in ids:
        resolution = resolution_of(item)
        if resolution in KEPT:
            kept.append(item)
        elif resolution in EXCLUDED:
            excluded.append(item)
        else:
            raise RuntimeError(f'{what}: resolution {resolution} outside contract: {item}')
    return kept, excluded


def check(what, kept, excluded, kept_key, excluded_key):
    if len(kept) != EXPECTED[kept_key] or len(excluded) != EXPECTED[excluded_key]:
        raise RuntimeError(f'{what}: kept {len(kept)} (want {EXPECTED[kept_key]}), '
                           f'excluded {len(excluded)} (want {EXPECTED[excluded_key]})')
    if len(set(kept)) != len(kept) or len(set(excluded)) != len(excluded):
        raise RuntimeError(what+': duplicate ids after filtering')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--g6-catalog', required=True)
    parser.add_argument('--oracle-keys-in', required=True)
    parser.add_argument('--out-catalog', required=True)
    parser.add_argument('--oracle-keys-out', required=True)
    parser.add_argument('--scope-out', required=True)
    args = parser.parse_args()

    g6 = json.loads(Path(args.g6_catalog).read_text())
    if set(g6) < {'capture_198', 'response_612', 'writes_72', 'placement_18'}:
        raise RuntimeError('gen6 catalog is missing sections')

    # Oracle: stream the 180000 row keys and count exclusions row by row.
    kept_rows, excluded_counts = [], {0.01: 0, 0.02: 0, 0.05: 0}
    with open(args.oracle_keys_in) as source, open(args.oracle_keys_out, 'w') as sink:
        for line in source:
            seed, transaction, resolution, boxes, shape = json.loads(line)
            resolution = float(resolution)
            excluded_counts.setdefault(resolution, 0)
            excluded_counts[resolution] += 1
            if resolution in KEPT:
                kept_rows.append((seed, transaction, resolution, boxes, shape))
                sink.write(json.dumps([seed, transaction, resolution, boxes, shape])+'\n')
    if len(kept_rows) != EXPECTED['oracle_kept']:
        raise RuntimeError(f'oracle kept {len(kept_rows)} rows, '
                           f'want {EXPECTED["oracle_kept"]}')
    if excluded_counts[0.01] != EXPECTED['oracle_excluded']:
        raise RuntimeError(f'oracle excluded 1cm rows = {excluded_counts[0.01]}, '
                           f'want {EXPECTED["oracle_excluded"]}')
    unexpected = {k: v for k, v in excluded_counts.items() if k not in (0.01, 0.02, 0.05)}
    if unexpected:
        raise RuntimeError(f'oracle rows at non-contract resolutions: {unexpected}')

    sections = {}
    scope = {'user_change': '2026-10-30 both occupancy lines drop 1cm; 2cm representative',
             'kept_resolutions_m': list(KEPT), 'excluded_resolution_m': list(EXCLUDED),
             'oracle_row_counts_by_resolution': {str(k): v for k, v in excluded_counts.items()},
             'sections': {}}
    for source_key, target_key, what in (('capture_198', 'capture_132', 'capture'),
                                         ('response_612', 'response_408', 'response'),
                                         ('writes_72', 'writes_48', 'writes'),
                                         ('placement_18', 'placement_12', 'placement')):
        kept, excluded = split_ids(g6[source_key]['ids'], what)
        check(what, kept, excluded, what+'_kept', what+'_excluded')
        sections[target_key] = dict(count=len(kept), ids=sorted(kept),
                                    source='gen6 '+source_key+' filtered by numeric '
                                           'resolution '+str(KEPT))
        scope['sections'][source_key] = dict(included=len(kept), excluded=len(excluded),
                                             excluded_ids=sorted(excluded),
                                             disposition='out_of_scope_by_user_resolution_change')
    # Independent set check: filtering must equal naive numeric membership.
    for target_key, source_key in (('capture_132', 'capture_198'), ('response_408', 'response_612'),
                                   ('writes_48', 'writes_72'), ('placement_12', 'placement_18')):
        naive = {i for i in g6[source_key]['ids'] if resolution_of(i) in KEPT}
        if naive != set(sections[target_key]['ids']):
            raise RuntimeError(target_key+': filtered set differs from naive membership')

    import hashlib
    catalog = dict(kind='occ_grid_acceptance_g7_2cm_case_catalog',
                   matrix_era='g7_2cm', kept_resolutions_m=list(KEPT),
                   sources=dict(g6_catalog=str(Path(args.g6_catalog).resolve()),
                                g6_catalog_sha256=hashlib.sha256(
                                    Path(args.g6_catalog).read_bytes()).hexdigest()),
                   oracle_120000=dict(row_keys_file=Path(args.oracle_keys_out).name,
                                      row_count=len(kept_rows),
                                      proof='row-by-row filter of the gen6 180000 journal'),
                   **sections,
                   regression_316=dict(disposition_file='regression_316_disposition.json',
                                       note='per-node reuse/out_of_scope/2cm-replacement map'))
    Path(args.out_catalog).write_text(json.dumps(catalog, indent=2, sort_keys=True)+'\n')
    Path(args.scope_out).write_text(json.dumps(scope, indent=2, sort_keys=True)+'\n')
    print(json.dumps({k: v['count'] for k, v in sections.items()}
                     | {'oracle_120000': len(kept_rows),
                        'oracle_excluded_1cm': excluded_counts[0.01]}))


if __name__ == '__main__':
    raise SystemExit(main())
