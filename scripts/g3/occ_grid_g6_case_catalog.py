#!/usr/bin/env python3
"""Rebuild the gen6 CASE_CATALOG from real historical sources only.

Every list is derived from a recorded historical manifest or journal
(review ruling of 2026-10-30_1539): query_matrix_02 instrumentation (198),
placement_matrix_03 response rows (612), oracle_matrix_10 input journals
(180000 per-row keys), the frozen write/placement generator dimensions
(72/18, cross-checked byte-for-byte against the gen5 catalog), and the gen5
regression node ids (316). Any count, uniqueness, or cross-check mismatch
aborts the build; no list is substituted or padded.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

EXPECTED_ORACLE_ROWS = 180000


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def capture_198(qm02):
    ids, applicable, excluded = [], set(), set()
    groups = qm02['groups']
    for group in groups:
        for case, entry in group['instrumentation'].items():
            if entry.get('outcome') == 'not_applicable':
                excluded.add(case)
                continue
            applicable.add(case)
            ids.append(f"r{group['resolution']}_b{group['boxes']}_{group['shape']}_{case}")
    if len(groups) != 18 or len(ids) != 198 or len(set(ids)) != 198 or len(applicable) != 11:
        raise RuntimeError(f'capture_198 mismatch: groups={len(groups)} ids={len(ids)} '
                           f'unique={len(set(ids))} classes={sorted(applicable)}')
    return sorted(ids), sorted(applicable), sorted(excluded)


def response_612(pm03):
    keys = []
    groups = pm03['groups']
    for group in groups:
        rows = group['response_comparison']['rows']
        if len(rows) != 34:
            raise RuntimeError(f"response rows != 34 in r{group['resolution']}_b{group['boxes']}_{group['path']}")
        for row in rows:
            keys.append(f"r{group['resolution']}_b{group['boxes']}_{group['path']}_{row['name']}")
    if len(groups) != 18 or len(keys) != 612 or len(set(keys)) != 612:
        raise RuntimeError(f'response_612 mismatch: groups={len(groups)} keys={len(keys)} '
                           f'unique={len(set(keys))}')
    return sorted(keys)


def oracle_180000(om10, baseline_journal, keys_out):
    inputs = om10['inputs']
    expected = (inputs['seeds']*inputs['transactions']*len(inputs['resolutions'])
                *len(inputs['boxes'])*len(inputs['shapes']))
    if om10.get('expected_rows') != expected or expected != EXPECTED_ORACLE_ROWS:
        raise RuntimeError(f"oracle expected_rows mismatch: {om10.get('expected_rows')} vs {expected}")
    keys, seen = [], set()
    with open(baseline_journal) as stream, open(keys_out, 'w') as sink:
        for line in stream:
            row = json.loads(line)
            key = [row['seed'], row['transaction'], row['resolution'],
                   row['initial_boxes'], row['shape']]
            tup = tuple(key)
            if tup in seen:
                raise RuntimeError(f'duplicate oracle row key: {tup}')
            seen.add(tup)
            keys.append(tup)
            sink.write(json.dumps(key)+'\n')
    if len(keys) != EXPECTED_ORACLE_ROWS:
        raise RuntimeError(f'oracle row keys: {len(keys)} != {EXPECTED_ORACLE_ROWS}')
    seeds = sorted({k[0] for k in keys}); transactions = sorted({k[1] for k in keys})
    resolutions = sorted({k[2] for k in keys}); boxes = sorted({k[3] for k in keys})
    shapes = sorted({k[4] for k in keys})
    for name, got, want in (('seeds', seeds, list(range(inputs['seeds']))),
                            ('transactions', transactions, list(range(inputs['transactions']))),
                            ('resolutions', resolutions, sorted(inputs['resolutions'])),
                            ('boxes', boxes, sorted(inputs['boxes'])),
                            ('shapes', shapes, sorted(inputs['shapes']))):
        if got != want:
            raise RuntimeError(f'oracle dimension drift in {name}: {got} != {want}')
    return dict(row_keys_file=keys_out.name, row_keys_sha256=sha256_file(keys_out),
                row_keys_count=len(keys), fixture_sha256=om10['fixture_sha256'],
                exact_match=om10['exact_match'],
                input_journals_equal=om10['input_journals_equal'],
                baseline_rows_sha256=om10['baseline_rows_sha256'],
                candidate_rows_sha256=om10['candidate_rows_sha256'],
                worker_inputs_sha256=[w.get('inputs_sha256') for w in om10['workers']])


def generator_ids(gen5):
    resolutions = [0.01, 0.02, 0.05]
    boxes = [0, 5, 20]
    writes = [f"r{r}_b{b}_p{p}_o{o}_c{c}"
              for r, b, p, o, c in itertools.product(resolutions, boxes,
                                                     (30000, 100000), (.5, .9), (0., .5))]
    placement = [f"r{r}_b{b}_{p}"
                 for r, b, p in itertools.product(resolutions, boxes, ('hit', 'fallback'))]
    old_writes, old_placement = gen5['writes_72']['ids'], gen5['placement_18']['ids']
    if len(writes) != 72 or len(set(writes)) != 72 or set(writes) != set(old_writes):
        raise RuntimeError('writes_72 cross-check against gen5 catalog failed')
    if len(placement) != 18 or len(set(placement)) != 18 or set(placement) != set(old_placement):
        raise RuntimeError('placement_18 cross-check against gen5 catalog failed')
    return writes, placement


def junit_node_ids(path):
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    return [f"{case.get('classname')}::{case.get('name')}"
            for case in root.iter('testcase')]


def regression_316(g5_root, result_path):
    result = load(result_path)
    anchor = Path(result['same_ids_as'])
    if not anchor.is_absolute():
        for parent in g5_root.parents:
            if (parent/anchor).exists():
                anchor = parent/anchor
                break
        else:
            anchor = None
    ids = junit_node_ids(result_path.parent/'junit.xml')
    if len(ids) != 316 or len(set(ids)) != 316 or result['unique_node_ids'] != 316:
        raise RuntimeError(f'regression node ids: {len(ids)} unique {len(set(ids))}')
    entry = dict(count=len(ids), ids=ids, source='gen5 regression_316/junit.xml '
                   '(source-level pass; ids identical to the G2 preflight registration)')
    if anchor and anchor.exists():
        anchor_ids = junit_node_ids(anchor)
        if set(anchor_ids) != set(ids):
            raise RuntimeError('regression ids differ from the G2 preflight registration')
        entry['anchor_path'] = str(anchor)
        entry['anchor_sha256'] = sha256_file(anchor)
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--g2-s5', required=True, help='occ_query_index_g2_20261025/s5 directory')
    parser.add_argument('--g5-root', required=True, help='occ_grid_acceptance_g5_20261030 directory')
    parser.add_argument('--out', required=True, help='output CASE_CATALOG.json path')
    parser.add_argument('--oracle-keys-out', required=True, help='output oracle row-key jsonl path')
    args = parser.parse_args()

    s5, g5 = Path(args.g2_s5), Path(args.g5_root)
    qm02_path = s5/'query_matrix_02/manifest.json'
    pm03_path = s5/'placement_matrix_03/manifest.json'
    om10_path = s5/'oracle_matrix_10/manifest.json'
    journal = s5/'oracle_matrix_10/baseline/inputs.jsonl'
    gen5_catalog_path = g5/'CASE_CATALOG.json'
    regression_path = g5/'regression_316/RESULT.json'

    capture_ids, classes, excluded = capture_198(load(qm02_path))
    response_ids = response_612(load(pm03_path))
    oracle = oracle_180000(load(om10_path), journal, Path(args.oracle_keys_out))
    gen5 = load(gen5_catalog_path)
    writes, placement = generator_ids(gen5)
    regression = regression_316(g5, regression_path)

    catalog = {
        'kind': 'occ_grid_acceptance_g6_case_catalog',
        'sources': {name: dict(path=str(path), sha256=sha256_file(path))
                    for name, path in (('query_matrix_02_manifest', qm02_path),
                                       ('placement_matrix_03_manifest', pm03_path),
                                       ('oracle_matrix_10_manifest', om10_path),
                                       ('oracle_baseline_inputs', journal),
                                       ('gen5_case_catalog', gen5_catalog_path),
                                       ('gen5_regression_result', regression_path))},
        'capture_198': dict(count=len(capture_ids), ids=capture_ids,
                            case_classes=classes, excluded_not_applicable=excluded),
        'response_612': dict(count=len(response_ids), ids=response_ids,
                             key_semantics='resolution/boxes/path/name; path=hit|fallback, '
                                          'name=cold(-01)+warmup(000-002)+scored(003-032)'),
        'oracle_180000': oracle,
        'writes_72': dict(count=len(writes), ids=writes,
                          source='frozen generator dimensions, cross-checked == gen5 catalog'),
        'placement_18': dict(count=len(placement), ids=placement,
                             source='frozen generator dimensions, cross-checked == gen5 catalog'),
        'regression_316': regression,
    }
    Path(args.out).write_text(json.dumps(catalog, indent=2, sort_keys=True)+'\n')
    print(json.dumps({k: catalog[k]['count'] for k in
                      ('capture_198', 'response_612', 'writes_72', 'placement_18', 'regression_316')}
                     | {'oracle_180000_rows': oracle['row_keys_count']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
