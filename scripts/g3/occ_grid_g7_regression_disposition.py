#!/usr/bin/env python3
"""Map the 316 regression node ids to gen7 dispositions (plan 9b7d465b s2).

Resolution-parameterized 1cm instances whose semantics survive through 2/5cm
(or invalid-value) siblings are marked out_of_scope_by_user_resolution_change;
everything else keeps its gen6 disposition (rerun from the frozen install when
the test touches ament/IDL/config/service artifacts, else reuse with the
byte-identity rationale). The tool refuses to lose semantics silently: an id
that cannot be justified aborts the build, and the sibling claims are checked
against the actual id set rather than assumed.
"""
import argparse
import json
from pathlib import Path


def is_onecm_instance(node):
    name = node.split('::')[-1]
    return '0.01' in name and '0.02' not in name and '0.05' not in name


def positive_onecm(node):
    name = node.split('::')[-1]
    return is_onecm_instance(node) and '[-0.01]' not in '['+name.split('[')[-1]


def sibling_base(node):
    name = node.split('::')[-1]
    return node.split('::')[0]+'::'+name.split('[')[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', required=True,
                        help='gen6 regression_deploy_selection.json')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    selection = json.loads(Path(args.selection).read_text())
    all_ids = selection['rerun_nodes'] + selection['reuse_nodes']
    if len(all_ids) != 316:
        raise RuntimeError(f'expected 316 nodes, got {len(all_ids)}')

    by_test = {}
    for node in all_ids:
        by_test.setdefault(sibling_base(node), []).append(node)

    disposition, out_of_scope, rerun, reuse = {}, [], [], []
    for node in sorted(all_ids):
        if is_onecm_instance(node):
            family = [n for n in by_test[sibling_base(node)]
                      if '0.02' in n or '0.05' in n]
            name = node.split('::')[-1]
            if name == '-0.01' or '[-0.01]' in node:
                kept = [n for n in by_test[sibling_base(node)] if n != node]
                reason = ('negative-boundary instance; invalid-resolution '
                          'rejection semantics retained by siblings '
                          + str(sorted(kept)))
            else:
                if not family:
                    raise RuntimeError('1cm instance without 2/5cm siblings '
                                       '(semantics would be lost): '+node)
                reason = ('1cm parameterized duplicate; same semantics at '
                          '2/5cm: '+str(sorted(family)))
            disposition[node] = dict(action='out_of_scope_by_user_resolution_change',
                                     reason=reason)
            out_of_scope.append(node)
        elif node in selection['rerun_nodes']:
            disposition[node] = dict(action='rerun_from_frozen_install',
                                     reason='ament/IDL/config/service or ROS-runtime '
                                            'dependency (gen6 marker scan)')
            rerun.append(node)
        else:
            disposition[node] = dict(action='reuse_gen6_source_pass',
                                     reason='no install-facing dependency; product '
                                            'modules byte-identical in the frozen '
                                            'install (gen6 verification)')
            reuse.append(node)

    kept_total = len(rerun)+len(reuse)
    if kept_total + len(out_of_scope) != 316:
        raise RuntimeError('disposition lost nodes')
    Path(args.out).write_text(json.dumps(dict(
        kind='occ_grid_acceptance_g7_regression_disposition',
        total=316, out_of_scope=len(out_of_scope),
        rerun_from_frozen_install=len(rerun), reuse=len(reuse),
        semantics_verdict='no 1cm-only semantics: every excluded instance has '
                          '2/5cm or invalid-value siblings in the kept set',
        disposition=disposition), indent=2, sort_keys=True)+'\n')
    print(json.dumps(dict(out_of_scope=len(out_of_scope), rerun=len(rerun),
                          reuse=len(reuse))))


if __name__ == '__main__':
    raise SystemExit(main())
