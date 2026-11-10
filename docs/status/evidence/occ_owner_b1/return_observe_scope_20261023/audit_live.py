"""Offline acceptance audit; reads durable trial/ledger, never changes inputs."""
import json
from pathlib import Path
import sys
root = Path(sys.argv[1])
ledger = [json.loads(line) for line in (root/'driver_out/ledger.jsonl').read_text().splitlines() if line]
rows = []
for entry in ledger:
    trial = json.loads((Path(entry['dump'])/'trial.json').read_text())
    extras = trial['extras']
    release = extras.get('release_kinematic', {})
    pose = release.get('presence_evidence', {})
    command = release.get('set_pose_response', {})
    commit = extras.get('cargo_map_commit_result', {})
    receipt = extras.get('cargo_map_commit_before_return', {})
    route = extras.get('return_route', {})
    frozen = json.loads(route.get('planner_detail') or '{}').get('frozen_map') or {}
    row = dict(seq=entry['seq'], committed=entry['committed'], fail_code=trial['fail_code'],
               verify=extras.get('verify',{}).get('verdict'),
               return_observe=extras.get('return_observe'),
               release_xy_m=release.get('declared_vs_world_xy_m'),
               release_stamp_ns=pose.get('stamp_ns'),command_sim_s=command.get('sim_s'),
               release_receive_monotonic_s=pose.get('received_monotonic_s'),
               command_receive_monotonic_s=command.get('monotonic_s'),
               commit_revision=commit.get('resulting_map_revision'),
               admission_revision=route.get('admission_map_revision'), frozen_map=frozen,
               segments_succeeded=trial.get('segments_succeeded'),
               readiness_before=extras.get('return_readiness_before',{}).get('scopes',{}))
    row['post_command_pose'] = bool(pose.get('stamp_ns',0)>round(command.get('sim_s',0)*1e9)
        and pose.get('received_monotonic_s',0)>command.get('monotonic_s',float('inf')))
    row['same_post_commit_map'] = bool(commit.get('committed') and receipt.get('committed')
        and commit.get('resulting_map_revision') == route.get('admission_map_revision')
        == frozen.get('map_revision') and frozen.get('snapshot_digest'))
    row['pass'] = bool(entry['committed'] and not trial['fail_code']
        and row['verify']=='ok' and row['return_observe'] and row['post_command_pose']
        and row['same_post_commit_map'] and row['release_xy_m']<=.005)
    rows.append(row)
result = dict(expected_boxes=3, observed_boxes=len(rows), passed_boxes=sum(r['pass'] for r in rows),
              functional_and3_pass=len(rows)==3 and all(r['pass'] for r in rows), rows=rows)
(root/'acceptance_audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
