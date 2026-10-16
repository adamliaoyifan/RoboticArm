#!/usr/bin/env python3
"""Validate frozen inputs and summarize paired samples; never launches motion.

Without --samples, writes the alternating 30-pair schedule only. Timing samples
must come from the instrumented planner/prior process trees, not this analyzer.
All artifact paths are relative to the manifest directory (or absolute).
"""
import argparse
import hashlib
import json
from pathlib import Path

from luggage_planning.prior_benchmark import analyze, schedule, validate_manifest


def verify_artifacts(artifacts, root):
    if not isinstance(artifacts, dict) or not artifacts:
        raise ValueError('nonempty path -> SHA256 artifact map required')
    for name, expected in artifacts.items():
        path = root / name
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        if digest.hexdigest() != expected:
            raise ValueError('artifact hash mismatch: ' + str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--samples', type=Path, help='JSONL raw arm samples, including every failure')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    cases = validate_manifest(manifest)
    root = args.manifest.resolve().parent
    for case in cases.values():
        verify_artifacts(case['artifacts'], root)
    samples = [json.loads(line) for line in args.samples.read_text().splitlines() if line.strip()] if args.samples else []
    for sample in samples:
        verify_artifacts(sample.get('artifacts'), root)
    result = analyze(manifest, samples)
    result['schedule'] = schedule(manifest)
    result['manifest_sha256'] = hashlib.sha256(args.manifest.read_bytes()).hexdigest()
    result['samples_sha256'] = hashlib.sha256(args.samples.read_bytes()).hexdigest() if args.samples else None
    result['scope'] = 'frozen six-workload paired planning comparison; not hardware or F3 acceptance'
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('verdict', 'expected_pairs', 'samples')}))


if __name__ == '__main__':
    main()
