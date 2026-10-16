#!/usr/bin/env python3
"""Account captured prior/route costs; no launch, execution or benchmark verdict."""
import argparse
import hashlib
import json
from pathlib import Path

from luggage_planning.prior_phase_accounting import account_phases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-dir', type=Path, required=True)
    parser.add_argument('--route-dir', type=Path, required=True)
    parser.add_argument('--shared-prior-artifact', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    hashes = {}
    def read(path):
        data = path.read_bytes()
        hashes[str(path.resolve())] = hashlib.sha256(data).hexdigest()
        return json.loads(data)
    prepare = read(args.prepare_dir/'result.json')
    route = read(args.route_dir/'result.json')
    for report in (prepare, route):
        for name, digest in report['inputs'].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest:
                raise ValueError('captured input changed: '+name)
    resources = {name:read(args.prepare_dir/(name+'.resources.json')) for name in ('cold','cached')}
    resources['route'] = read(args.route_dir/'resources.json')
    result = account_phases(prepare,route,resources,str(args.shared_prior_artifact.resolve()))
    result['report_sha256'] = hashes
    with args.output.open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps({k:result[k] for k in ('verdict','benchmark_sample')}))


if __name__ == '__main__':
    main()
