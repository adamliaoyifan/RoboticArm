"""One-shot mechanical move; snapshot both index baseline and shared working bytes."""
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path.cwd()
EV=ROOT/'docs/status/evidence/planning_route_layout/20261112'
BASE=Path('src/luggage_planning/luggage_planning')

def mapping():
    result={}
    for p in (ROOT/BASE).glob('*.py'):
        n=p.stem
        if n.startswith('joint_operation_'): result[n]='joint_operation.'+n[len('joint_operation_'):]
        elif n in ('aperture_geometry','aperture_tool_geometry','preparation_profile','preparation_refresh'): result[n]='joint_operation.'+n
        elif n.startswith('placement_prior_'): result[n]='placement_prior.'+n[len('placement_prior_'):]
        elif n=='placement_prior':result[n]='placement_prior.core'
        elif n.startswith('prior_'):result[n]='placement_prior.'+n
    result.update(dict(atlas_runtime='atlas.runtime',atlas_frame_contracts='atlas.frame_contracts',atlas_prior='atlas.prior',atlas_proposals='atlas.proposals',reachability_atlas='atlas.model',reachability_wavefront='atlas.wavefront',reachable_graph='atlas.graph',reachable_task_grid='atlas.task_grid',layout_atlas='atlas.layout',seed_chain='atlas.seed_chain',route_producer='routing.producer',route_producer_factory='routing.factory',direct_first_route='routing.direct_first',lattice_route='routing.lattice',corridor_lattice='routing.corridor_lattice',corridor_height='routing.corridor_height',grid_path_search='routing.grid_search',placement_corridor_supergraph='routing.corridor_supergraph',staging_candidates='routing.staging_candidates',target_ladder='routing.target_ladder',retreat_continuation='routing.retreat_continuation',route_collision_diagnostics='routing.collision_diagnostics',planning_work_counters='routing.work_counters',moveit_edge_adapter='routing.moveit_edge_adapter'))
    return dict(sorted(result.items()))

def rewrite(raw, moves):
    text=raw.decode('utf-8')
    tree=ast.parse(text)
    lines=text.splitlines(keepends=True)
    offsets=[0]
    for line in lines:offsets.append(offsets[-1]+len(line))
    edits=[]
    for node in ast.walk(tree):
        if not isinstance(node,ast.ImportFrom) or node.module!='luggage_planning' or not any(a.name in moves for a in node.names):continue
        statements=[]
        for alias in node.names:
            if alias.name in moves:
                package,leaf=moves[alias.name].rsplit('.',1)
                local=alias.asname or alias.name
                tail=' as '+local if local!=leaf else ''
                statements.append('from luggage_planning.'+package+' import '+leaf+tail)
            else:
                tail=' as '+alias.asname if alias.asname else ''
                statements.append('from luggage_planning import '+alias.name+tail)
        edits.append((offsets[node.lineno-1]+node.col_offset,offsets[node.end_lineno-1]+node.end_col_offset,('\n'+' '*node.col_offset).join(statements)))
    for start,end,value in reversed(sorted(edits)):text=text[:start]+value+text[end:]
    pattern=re.compile(r'(?<![\w.])luggage_planning\.('+ '|'.join(re.escape(n) for n in sorted(moves,key=len,reverse=True))+r')(?!\w)')
    text=pattern.sub(lambda m:'luggage_planning.'+moves[m[1]],text)
    for old,new in moves.items():
        text=text.replace(str(BASE/(old+'.py')),str(BASE/Path(*new.split('.')))+'.py')
    # __file__ moved one directory deeper; preserve the original resource
    # anchor, including explicit flat-layout module_file injection.
    old = "def _module_dir(module_file=None):\n    return os.path.dirname(os.path.abspath(module_file or __file__))"
    replacement = '\n'.join([
        'def _module_dir(module_file=None):',
        '    """Resource anchor at the package module directory, before/after relocation.',
        '',
        '    Explicit module_file retains the flat-layout injection contract used by',
        '    offline callers. The real runtime now lives one level below that anchor.',
        '    """',
        '    if module_file is not None:',
        '        return os.path.dirname(os.path.abspath(module_file))',
        '    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))'])
    if old in text:
        text=text.replace(old,replacement)
        text=text.replace('os.path.dirname(os.path.abspath(__file__)), "..", "..",','_module_dir(), "..", "..",')
    return text.encode('utf-8')

def target(path,moves):
    p=Path(path)
    if p.parent==BASE and p.stem in moves:return str(BASE/Path(*moves[p.stem].split('.')))+'.py'
    return path

phase=sys.argv[1]
if phase=='snapshot':
    moves=mapping(); assert len(moves)==72,len(moves)
    paths=set(subprocess.check_output(['git','ls-files','-c','-o','--exclude-standard'],text=True).splitlines())
    paths={p for p in paths if p.startswith(('src/','scripts/','tests/','st09_contracts/','research/')) and p.endswith('.py') and Path(p).is_file()}
    paths.update(str(BASE/(n+'.py')) for n in moves)
    records=[]
    for name in sorted(paths):
        p=Path(name);raw=p.read_bytes()
        try:after=rewrite(raw,moves)
        except (SyntaxError,UnicodeDecodeError):continue
        dest=target(name,moves)
        if after==raw and dest==name:continue
        head=subprocess.run(['git','show','HEAD:'+name],capture_output=True)
        rec=dict(path=name,destination=dest,mode='100755' if p.stat().st_mode&0o111 else '100644',working_sha256=hashlib.sha256(raw).hexdigest(),tracked=head.returncode==0)
        for kind,data in [('working',raw)]+([('head',head.stdout)] if rec['tracked'] else []):
            out=EV/'snapshot'/kind/name;out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(data)
        if rec['tracked']:rec['preexisting_dirty']=raw!=head.stdout
        records.append(rec)
    test=Path('src/luggage_planning/test')
    tests=sorted(str(p) for p in test.glob('test_*.py') if ('_native' not in p.stem and (p.stem.startswith(('test_joint_operation','test_placement_prior')) or p.stem in ('test_atlas_proposals','test_atlas_runtime','test_route_producer_factory','test_direct_first_route','test_aperture_routing'))))
    manifest=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),dirty=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines(),mapping=moves,files=records,tests=tests)
    (EV/'baseline.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(moved=len(moves),touched=len(records),preexisting_dirty=sum(r.get('preexisting_dirty',False) for r in records),untracked=sum(not r['tracked'] for r in records),test_files=len(tests))))
else:
    data=json.loads((EV/'baseline.json').read_text()); moves=data['mapping']
    if phase=='migrate':
        for rec in data['files']:
            old=ROOT/rec['path']; dest=ROOT/rec['destination']
            assert old.read_bytes()==(EV/'snapshot/working'/rec['path']).read_bytes(),'Concurrent change: '+rec['path']
            assert old==dest or not dest.exists(),dest
        for rec in data['files']:
            old=ROOT/rec['path'];dest=ROOT/rec['destination'];raw=old.read_bytes()
            dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_bytes(rewrite(raw,moves));dest.chmod(int(rec['mode'],8)&0o777)
            if old!=dest:old.unlink()
        for group in ('joint_operation','placement_prior','routing','atlas'):
            (ROOT/BASE/group/'__init__.py').write_text('"""'+dict(joint_operation='Whole-operation planning and bound execution.',placement_prior='Read-only candidate screening and snapshot evidence.',routing='Segmented route proposals and current-scene connections.',atlas='Offline reachability sampling and qualified runtime hints.')[group]+'"""\n')
        print('Migration complete')
    elif phase=='check':
        checks=[]
        for rec in data['files']:
            actual=(ROOT/rec['destination']).read_bytes()
            expected=rewrite((EV/'snapshot/working'/rec['path']).read_bytes(),moves)
            checks.append(dict(path=rec['destination'],exact_expected_bytes=actual==expected,explicit_install_check_extension=rec['path']=='scripts/production/check_install.py',sha256=hashlib.sha256(actual).hexdigest()))
        (EV/'preservation.json').write_text(json.dumps(checks,indent=2)+'\n')
        assert all(r['exact_expected_bytes'] or r['explicit_install_check_extension'] for r in checks),'Unexpected non-import code change'
        print('Exact mechanical preservation:',len(checks),'files')
    elif phase=='stage':
        assert not subprocess.check_output(['git','diff','--cached','--name-only'],text=True).strip(),'Unexpected staged content'
        count=0
        for rec in data['files']:
            if not rec['tracked']:continue
            raw=(EV/'snapshot/head'/rec['path']).read_bytes();updated=rewrite(raw,moves)
            if raw==updated and rec['path']==rec['destination']:continue
            blob=subprocess.check_output(['git','hash-object','-w','--stdin'],input=updated).decode().strip()
            subprocess.run(['git','update-index','--add','--cacheinfo',rec['mode'],blob,rec['destination']],check=True)
            if rec['path']!=rec['destination']:subprocess.run(['git','update-index','--force-remove',rec['path']],check=True)
            count+=1
        print('Staged transformed HEAD only:',count,'tracked files; pre-existing working changes remain dirty')
    else:raise SystemExit('unknown phase')
