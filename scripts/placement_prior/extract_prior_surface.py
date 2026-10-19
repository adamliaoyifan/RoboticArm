#!/usr/bin/env python3
"""Join exact published OCC to a captured Prepare response and owner stats."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
from extract_replay_surface import check_legacy


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--prepare-index',type=int,default=0)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from rclpy.serialization import deserialize_message as de,serialize_message
    from luggage_msgs.srv import PreparePlacementPriorV2,GetCargoMapStats
    from luggage_msgs.msg import SurfaceContractV2
    from std_msgs.msg import String
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_description.ros_frame_bindings.surfaces import surface_from_message
    from luggage_planning.occupancy_place_paths import OccupancySnapshot
    prior=args.capture/'prior_boundaries'
    rows=[json.loads(s) for s in (prior/'boundaries.jsonl').read_text().splitlines()]
    response_row=[r for r in rows if r['boundary']=='prepare_response'][args.prepare_index]
    response_path=prior/response_row['file']
    response=de(response_path.read_bytes(),PreparePlacementPriorV2.Response)
    surface=response.surface
    context=BoundaryContext('PRIOR_OCC','captured_owner','replay',surface.identity.request_id,
        surface.identity.scene_revision,'container_link','','scene',surface.geometry_hash,surface.map_revision)
    record=surface_from_message(surface,context)
    boundary=args.capture/'dumps/typed_service_boundaries'
    stats=[]
    for row in map(json.loads,(boundary/'boundaries.jsonl').read_text().splitlines()):
        if row['boundary']=='/cargo_map/get_stats:response':
            path=boundary/row['file'];value=de(path.read_bytes(),GetCargoMapStats.Response)
            if value.success and value.map_revision==surface.map_revision and value.geometry_hash==surface.geometry_hash:
                stats.append((path,value))
    if not stats:raise ValueError('no matching actual owner stats')
    typed=[];legacy={}
    for bag in sorted((args.capture/'bag').glob('*.db3')):
        with sqlite3.connect(bag.resolve().as_uri()+'?mode=ro',uri=True) as connection:
            for topic,stamp,data in connection.execute("SELECT topics.name,timestamp,data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name IN ('/luggage/cargo_map/surface_v2','/luggage/cargo_map/surface_2d') ORDER BY timestamp"):
                source={'bag':str(bag),'receive_ns':stamp,'sha256':hashlib.sha256(data).hexdigest()}
                if topic.endswith('surface_v2'):
                    if de(data,SurfaceContractV2)==surface:typed.append(source)
                    continue
                text=de(data,String).data;payload=json.loads(text)
                if payload.get('map_revision')!=surface.map_revision:continue
                OccupancySnapshot.from_surface_2d(record,expected_hash=surface.geometry_hash,
                    expected_revision=surface.map_revision,inner_size=payload['inner_size'],
                    geometry_descriptor=payload['geometry_descriptor'])
                matches=[]
                for path,value in stats:
                    try:check_legacy(payload,record,value,payload['inner_size'])
                    except (ValueError,KeyError,TypeError,IndexError):continue
                    matches.append(str(path))
                if matches:
                    digest=hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()
                    legacy[digest]=(bytes(data),text,source,matches)
    if not typed or len(legacy)!=1:raise ValueError(f'ambiguous/missing surface: typed={len(typed)} legacy={len(legacy)}')
    data,text,source,matches=next(iter(legacy.values()))
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'surface_v2.cdr').write_bytes(serialize_message(surface))
    (args.output/'surface_2d.cdr').write_bytes(data)
    (args.output/'surface_2d.json').write_text(text)
    manifest={'scope':'exact publication matching captured Prepare and actual owner stats; no geometry reconstruction',
        'prepare_response':str(response_path),'map_revision':surface.map_revision,'typed_sources':typed,
        'legacy_source':source,'stats_sources':matches}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('joined map revision',surface.map_revision)


if __name__=='__main__':main()
