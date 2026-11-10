import ast,json,re,threading,time,subprocess
from pathlib import Path
root=Path('/tmp/elfin-master-pack-baseline-g3')
out=root/'docs/status/evidence/occ_owner_b1/release_pose_publication_20261023'
source=root/'src/luggage_gazebo/scripts/place_smoke_driver.py'
tree=ast.parse(source.read_text())
node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ReleasePoseStream')
namespace=dict(re=re,threading=threading,time=time,subprocess=subprocess,GZ_WORLD='airport_loading')
exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),namespace)
stream=namespace['ReleasePoseStream'](ign_bin='/bin/true')
raw=(out/'raw_pose.txt').read_text()
for line in raw.splitlines():stream._ingest_line(line)
models=['ground_plane','_preload_suitcase_loafbrr_small','pickup_box_0001_inch20']
result={'method':'AST-extracted unchanged ReleasePoseStream class; feed exact raw_pose.txt lines; no transport mocked into publication evidence','message_count':raw.count('header {'),'published_name_occurrences':{name:raw.count('name: "'+name+'"') for name in models},'parser_cache_present':{name:stream.latest(name) is not None for name in models},'cache_names':sorted(stream._latest),'scope':'Startup sample contains preloaded suitcases, not runtime pickup box. Target-box probe aborted during startup before creation.'}
(out/'parser_replay.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
