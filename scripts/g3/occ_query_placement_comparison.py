"""Compare complete placement values independently of undefined CDR padding."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time


def compare_responses(baseline,candidate):
    from luggage_msgs.srv import ComputePlacement
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    baseline=Path(baseline);candidate=Path(candidate)
    left={p.name:p for p in baseline.glob('*.response.cdr')}
    right={p.name:p for p in candidate.glob('*.response.cdr')}
    rows=[]
    for name in sorted(left.keys()|right.keys()):
        row=dict(name=name,wire_equal=False,semantic_equal=False)
        if name not in left or name not in right:
            row['error']='missing response in one version'
        else:
            a,b=left[name].read_bytes(),right[name].read_bytes()
            row.update(baseline_sha256=hashlib.sha256(a).hexdigest(),
                candidate_sha256=hashlib.sha256(b).hexdigest(),wire_equal=a==b)
            try:
                x=message_to_ordereddict(deserialize_message(a,ComputePlacement.Response))
                y=message_to_ordereddict(deserialize_message(b,ComputePlacement.Response))
                row['semantic_equal']=x==y
                row['different_fields']=[key for key in x if x[key]!=y[key]]
            except Exception as error:
                row['error']=repr(error)
        rows.append(row)
    return dict(samples=len(rows),equal=bool(rows) and all(r['semantic_equal'] for r in rows),
        wire_equal=bool(rows) and all(r['wire_equal'] for r in rows),rows=rows)


def compare_in_environment(baseline, candidate, output, *, script, environment):
    """Decode in the selected candidate IDL environment, never the parent.

    A campaign may be launched from a shell without ROS. Its service children
    and this offline reader must resolve the same isolated generated messages.
    Inputs remain immutable; the child writes a separate comparison artifact.
    A semantic difference is a result, while a decoder/launch failure is an
    error with preserved stderr. Neither case retries a product request.
    """
    output = Path(output)
    control_path = output.with_suffix('.control.json')
    if output.exists() or control_path.exists():
        raise FileExistsError('comparison artifact already exists: '+str(output))
    command = [sys.executable, str(script), '--baseline', str(baseline),
               '--candidate', str(candidate), '--out', str(output)]
    control = dict(command=command, started_ns=time.time_ns())
    try:
        result = subprocess.run(command, env=environment, capture_output=True,
                                text=True, timeout=30.)
    except (OSError, subprocess.TimeoutExpired) as error:
        control.update(exit_code=None, error=repr(error), ended_ns=time.time_ns())
        control_path.write_text(json.dumps(control, indent=2)+'\n')
        raise RuntimeError('placement comparison child failed; see '+str(control_path)) from error
    control.update(exit_code=result.returncode, stdout=result.stdout,
                   stderr=result.stderr, ended_ns=time.time_ns())
    control_path.write_text(json.dumps(control, indent=2)+'\n')
    if result.returncode != 0:
        raise RuntimeError('placement comparison child failed; see '+
                           str(control_path))
    return json.loads(output.read_text())


def inspect_campaign(bundle,args,common):
    manifest_path=bundle/'manifest.json'
    manifest=json.loads(manifest_path.read_text())
    if manifest.get('kind')!='planning' or manifest.get('phase')!='complete':
        raise RuntimeError('requires completed frozen placement campaign')
    out=Path(args.out).resolve() if args.out else bundle/('semantic_inspection_'+str(time.time_ns()))
    out.mkdir(parents=True,exist_ok=False)
    groups=[]
    for group in manifest['groups']:
        directory=bundle/('r%s_b%d_%s'%(group['resolution'],group['boxes'],group['path']))
        comparison=compare_responses(directory/'baseline/client',directory/'candidate/client')
        comparison.update(resolution=group['resolution'],boxes=group['boxes'],path=group['path'],required=34)
        groups.append(comparison)
    report=dict(kind='placement_wire_semantic_inspection',reader_revision=common.git('rev-parse','HEAD'),
        producer_revision=manifest['revision'],baseline_revision=manifest['baseline_revision'],
        producer_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        inspection_only=True,reexecuted_product=False,groups=groups,overall_g2='not_evaluated',
        equal=all(g['equal'] and g['samples']==g['required'] for g in groups),
        note='All response fields compared exactly after deserialization. Raw wire differences remain preserved; undefined alignment bytes are not response fields.')
    common.write_json(out/'result.json',report)
    return int(not report['equal'])


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    output = Path(args.out)
    if output.exists():
        raise FileExistsError('comparison artifact already exists: '+str(output))
    comparison = compare_responses(args.baseline, args.candidate)
    with output.open('x') as stream:
        stream.write(json.dumps(comparison, indent=2)+'\n')


if __name__ == '__main__':
    main()
