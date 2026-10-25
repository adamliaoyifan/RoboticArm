"""Fixed-rate no-motion pressure client with original runtime heartbeats.

Baseline uses its existing stats/full-volume APIs for the same logical reads;
candidate uses the new bounded read services. Baseline volume refusal is
preserved, never replaced by a fabricated new service or a quiet workload.
"""
import argparse
from collections import deque
import hashlib
import importlib
import json
import os
from pathlib import Path
import time


RATES={'integrate':2.,'summary':10.,'roi':5.,'distance':2.,'surface':1.}
LIMITS_MS={'summary':(50.,200.),'roi':(100.,250.),'distance':(50.,200.),'surface':(100.,250.)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('out','owner','install'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--duration',type=float,default=120.)
    parser.add_argument('--baseline',action='store_true')
    args=parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    import numpy as np
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point
    from luggage_msgs import srv
    from luggage_description.ros_frame_bindings.runtime import NodeRuntime
    import luggage_perception.cargo_volume_mapper as artifact
    from occ_query_wire_journal import WireJournal
    out=Path(args.out).resolve();out.mkdir(parents=True,exist_ok=False)
    journal=WireJournal(out/'wire_journal')
    owner=Path(args.owner);ready=json.loads((owner/'ready.json').read_text())
    install=Path(args.install).resolve()
    if not Path(artifact.__file__).resolve().is_relative_to(install):raise RuntimeError('mixed load-client install')
    rclpy.init();node=rclpy.create_node('occ_g2_load')
    heartbeat=[]
    original_report=NodeRuntime._on_report
    def report(runtime,message):
        original_report(runtime,message)
        heartbeat.append(dict(runtime.receive_trace[-1]))
    NodeRuntime._on_report=report
    runtime=NodeRuntime(node,peer='occ_g2_load',identity=ready['identity'],resolved_config={'workload':RATES},
        peers=ready['peers'],modules=[(artifact,'module','application')],package_names=['luggage_perception'],
        transitions_path=str(out/'transitions.jsonl'))
    publish=node.create_publisher(PointCloud2,'/luggage/semantic/cargo_points_untracked',10)
    kinds={'integrate':srv.IntegrateCargoView,'stats':srv.GetCargoMapStats}
    if args.baseline:
        kinds.update(summary=srv.GetCargoMapStats,roi=srv.GetCargoVolumeV2,
            distance=srv.GetCargoVolumeV2,surface=srv.GetCargoVolumeV2)
    else:
        kinds.update(summary=srv.GetCargoMapSummaryV2,roi=srv.QueryCargoCellsV2,
            distance=srv.QueryCargoDistanceV2,surface=srv.GetCargoSurfacePatchV2)
    endpoints={'integrate':'integrate_cargo_view','stats':'get_stats','summary':'get_stats' if args.baseline else 'get_summary_v2',
        'roi':'get_volume_v2' if args.baseline else 'query_cells_v2',
        'distance':'get_volume_v2' if args.baseline else 'query_distance_v2',
        'surface':'get_volume_v2' if args.baseline else 'get_surface_patch_v2'}
    clients={key:node.create_client(kind,'/cargo_map/'+endpoints[key]) for key,kind in kinds.items()}
    rng=np.random.default_rng(20261025)
    cloud=rng.uniform([-.70,-.80,ready['floor_z']+.10],[.70,.30,ready['floor_z']+1.00],(30000,3)).astype('<f4')
    points=rng.uniform([-.70,-.80,ready['floor_z']+.40],[.70,.40,ready['ceiling_z']-.01],(256,3))
    np.save(out/'cloud.npy',cloud);np.save(out/'distance_points.npy',points)
    current=dict(revision=ready['map_revision'],epoch=ready['epoch'],content_digest=ready['content_digest'])
    pending=[];write_queue=deque();wire_ring=deque();sent_ring=deque();sent_trace=[]
    rows=[];counts={k:0 for k in RATES}
    failure=None;phase='startup';first_failure=None;scored_start=None;ended_scoring=None;backlog_max_s=0.
    def fail(code,**values):
        nonlocal failure,first_failure
        if failure is None:
            first_failure=time.monotonic();failure=dict(code=code,phase=phase,monotonic=first_failure,**values)
    def observe_backlog(age):
        nonlocal backlog_max_s
        backlog_max_s=max(backlog_max_s,age)
        if age>1.:fail('BACKLOG_OVER_1S',age_s=age)
    def send(case,request,scheduled,sequence,request_phase=None):
        request_phase=phase if request_phase is None else request_phase
        begin=time.monotonic()
        wire=serialize_message(request)
        journal.request(case,sequence,wire,phase=request_phase,scheduled=scheduled,begin=begin)
        sent_ring.append((begin,case,sequence,wire))
        sent_trace.append(dict(case=case,sequence=sequence,phase=request_phase,scheduled=scheduled,
            begin=begin,request_sha256=hashlib.sha256(wire).hexdigest()))
        cutoff=(first_failure if first_failure is not None else begin)-10.
        while sent_ring and sent_ring[0][0]<cutoff:sent_ring.popleft()
        future=clients[case].call_async(request)
        pending.append(dict(case=case,sequence=sequence,phase=request_phase,scheduled=scheduled,begin=begin,future=future,request=wire))
    def request_read(case,scheduled,sequence):
        request=kinds[case].Request()
        if hasattr(request,'query'):
            request.query.request_id=case+'-'+str(sequence)
            request.query.frame_id='container_link';request.query.use_current=True
            request.query.release_after_query=True;request.query.limit=4096
        if args.baseline and case in ('roi','distance','surface'):
            request.scene_revision=ready['identity']['scene_revision'];request.geometry_hash=ready['identity']['geometry_hash']
            request.expected_map_revision=current['revision'];request.expected_epoch=current['epoch']
            request.expected_content_digest=current['content_digest']
        elif case=='roi':
            request.query.has_region=True
            request.query.region_min=Point(x=-.15,y=-.15,z=ready['floor_z'])
            request.query.region_max=Point(x=.15,y=.15,z=ready['floor_z']+.3)
        elif case=='distance':
            request.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in points]
            request.max_radius_m=.30
        send(case,request,scheduled,sequence)
    def enqueue_write(scheduled,sequence):
        stamp=node.get_clock().now().to_msg()
        publish.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='container_link'),cloud))
        write_queue.append(dict(scheduled=scheduled,sequence=sequence,stamp=stamp,phase=phase))
    def pump():
        # Ack comes from a passive wrapper after the ORIGINAL cloud callback.
        # No timer assumption or sleep substitutes for measured ingestion.
        if write_queue and not any(p['case']=='integrate' for p in pending):
            entry=write_queue[0];ack=owner/'cloud_ack.json'
            if ack.exists() and [entry['stamp'].sec,entry['stamp'].nanosec] in json.loads(ack.read_text())['stamps']:
                write_queue.popleft();request=srv.IntegrateCargoView.Request()
                request.schema_version=1;request.settled=True;request.frame_count=1
                request.source_acquisition_stamp=entry['stamp']
                request.view_request_id=request.operation_id='pressure-'+str(entry['sequence'])
                request.expected_map_revision=current['revision'];request.expected_epoch=current['epoch']
                request.expected_content_digest=current['content_digest'];request.geometry_hash=ready['identity']['geometry_hash']
                send('integrate',request,entry['scheduled'],entry['sequence'],entry['phase'])
        for item in list(pending):
            future=item['future']
            if not future.done():continue
            pending.remove(item);case=item['case'];now=time.monotonic()
            if future.exception():fail('RPC_EXCEPTION',case=case,error=repr(future.exception()));continue
            response=future.result()
            status=(response.meta.status if hasattr(response,'meta') else
                'OK' if response.success else getattr(response,'reason_code',getattr(response,'reason',response.message if hasattr(response,'message') else 'REFUSED')))
            if case=='integrate' and response.success:
                current.update(revision=int(response.resulting_map_revision),epoch=response.epoch,content_digest=response.content_digest)
            elif case in ('summary','stats') and not hasattr(response,'meta') and response.success:
                if response.map_revision>=current['revision']:
                    current.update(revision=int(response.map_revision),epoch=response.epoch,content_digest=response.content_digest)
            # Old representation: compute the same logical read at the client.
            # Full transport and this conversion remain in its latency.
            if args.baseline and case in ('roi','distance','surface') and response.success:
                states=np.asarray(response.states,dtype='u1');active=np.asarray(response.active,dtype=bool)
                nx,ny,nz=map(int,response.dimensions_xyz);resolution=response.resolution_m
                origin=np.asarray(response.origin_container)
                ids=np.flatnonzero(active)
                coordinates=np.column_stack((ids%nx,ids//nx%ny,ids//(nx*ny)))
                lower=origin+coordinates*resolution
                if case=='roi':
                    mask=np.all(lower+resolution>[-.15,-.15,ready['floor_z']],axis=1)&np.all(lower<[.15,.15,ready['floor_z']+.3],axis=1)
                    selected=ids[mask][:4096]
                    _=states[selected].tobytes()
                elif case=='distance':
                    for state in (0,2):
                        obstacles=lower[states[ids]==state]
                        for point in points:
                            delta=np.maximum(np.maximum(obstacles-point,point-(obstacles+resolution)),0.)
                            _=float(np.min(np.einsum('ij,ij->i',delta,delta))) if len(delta) else None
                else:
                    observed=(active&(states!=0)).reshape((nz,ny,nx))
                    _=np.max(np.where(observed,np.arange(nz)[:,None,None]+1,0),axis=0).tobytes()
            elapsed=time.monotonic()-item['begin'];total=time.monotonic()-item['scheduled']
            pure=response.meta.elapsed_ns/1e6 if hasattr(response,'meta') and case=='distance' else elapsed*1000
            row=dict(case=case,sequence=item['sequence'],phase=item['phase'],completion_phase=phase,scheduled=item['scheduled'],
                begin=item['begin'],ended=time.monotonic(),elapsed_ms=elapsed*1000,total_ms=total*1000,
                gated_ms=pure,status=status)
            rows.append(row)
            if item['phase']=='scoring':observe_backlog(total)
            payload=serialize_message(response)
            journal.response(case,item['sequence'],payload,phase=item['phase'],completion_phase=phase,ended=time.monotonic(),status=status)
            wire_ring.append((time.monotonic(),case,item['sequence'],item['request'],payload))
            cutoff=(first_failure if first_failure is not None else time.monotonic())-10.
            while wire_ring and wire_ring[0][0]<cutoff:wire_ring.popleft()
            if status!='OK':fail('REQUEST_REFUSED',case=case,status=status)
            if item['phase']=='scoring' and case in LIMITS_MS and pure>LIMITS_MS[case][1]:
                fail('QUERY_MAX',case=case,latency_ms=pure,limit_ms=LIMITS_MS[case][1])
        queued=[p['scheduled'] for p in pending if p['phase']=='scoring']+[
            p['scheduled'] for p in write_queue if p['phase']=='scoring']
        if queued:observe_backlog(time.monotonic()-min(queued))
        accepted=[r for r in heartbeat if r['peer']=='cargo_volume_mapper' and r['status']=='accepted']
        if scored_start is not None and accepted and time.monotonic()-accepted[-1]['received_monotonic']>=1.:
            fail('HEARTBEAT_GAP',gap_s=time.monotonic()-accepted[-1]['received_monotonic'])
    result={}
    try:
        for key,client in clients.items():
            if not client.wait_for_service(timeout_sec=10.):raise RuntimeError('missing endpoint: '+key)
        startup=time.monotonic()
        while runtime.readiness()!='READY':
            if time.monotonic()-startup>30.:raise RuntimeError('runtime readiness: '+runtime.readiness())
            rclpy.spin_once(node,timeout_sec=.01)
        # One non-scored measured wiring attempt; failure remains evidence.
        phase='wiring';enqueue_write(time.monotonic(),-1)
        deadline=time.monotonic()+20.
        while (pending or write_queue) and failure is None:
            if time.monotonic()>deadline:fail('WIRING_DEADLINE');break
            rclpy.spin_once(node,timeout_sec=.001);pump()
        if failure is None:
            phase='pre_window';deadline=time.monotonic()+10.
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.01)
            phase='scoring';scored_start=time.monotonic()
            due={key:scored_start for key in RATES}
            while time.monotonic()-scored_start<args.duration and failure is None:
                now=time.monotonic()
                for case,rate in RATES.items():
                    while now>=due[case] and due[case]<scored_start+args.duration:
                        sequence=counts[case];counts[case]+=1
                        if case=='integrate':enqueue_write(due[case],sequence)
                        else:request_read(case,due[case],sequence)
                        due[case]+=1./rate
                rclpy.spin_once(node,timeout_sec=.001);pump()
            ended_scoring=time.monotonic()
        phase='post_window';deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.001);pump()
        if pending or write_queue:fail('INCOMPLETE_ACCEPTED_REQUESTS',pending=len(pending),queued=len(write_queue))
        times=[r['received_monotonic'] for r in heartbeat if r['peer']=='cargo_volume_mapper' and r['status']=='accepted']
        gaps=[b-a for a,b in zip(times,times[1:])]
        snapshot=runtime.snapshot()
        snapshot['observed_at_monotonic']=time.monotonic()
        observation_end=snapshot['observed_at_monotonic']
        expiry=sum(r.get('status')=='PEER_LEASE_EXPIRED' and (scored_start is None or
            scored_start-10.<=r['received_monotonic']<=observation_end) for r in heartbeat)
        groups={}
        for case,limits in LIMITS_MS.items():
            values=sorted(r['gated_ms'] for r in rows if r['case']==case and r['phase']=='scoring' and r['status']=='OK')
            import math
            groups[case]=dict(samples=len(values),p95_ms=values[math.ceil(.95*len(values))-1] if values else None,
                max_ms=values[-1] if values else None,limits_ms=limits)
        result=dict(outcome='fail' if failure else 'not_evaluated',first_failure=failure,
            phase='complete',counts_scheduled=counts,groups=groups,scored_start=scored_start,
            ended_scoring=ended_scoring,observation_end=observation_end,wall_duration_s=(ended_scoring-scored_start) if scored_start else 0.,
            heartbeat_max_gap_s=max(gaps,default=None),lease_expiry=expiry,backlog_max_s=backlog_max_s,
            semantic_oracle='not_evaluated',runtime=snapshot)
    except Exception as error:
        fail('FIXTURE_OR_STARTUP_ERROR',error=repr(error))
        result=dict(outcome='fail',first_failure=failure,capture_complete=False)
    finally:
        for name,values in [('heartbeat',heartbeat),('requests',rows),('requests_sent',sent_trace)]:
            with (out/(name+'.jsonl')).open('w') as stream:
                for row in values:stream.write(json.dumps(row)+'\n')
        if failure:
            capture=out/'failure_wire';capture.mkdir()
            for _,case,sequence,request in sent_ring:
                (capture/(case+'-'+str(sequence)+'.request.cdr')).write_bytes(request)
            for _,case,sequence,request,response in wire_ring:
                (capture/(case+'-'+str(sequence)+'.request.cdr')).write_bytes(request)
                (capture/(case+'-'+str(sequence)+'.response.cdr')).write_bytes(response)
        queued=[dict(scheduled=p['scheduled'],sequence=p['sequence'],
            stamp=[p['stamp'].sec,p['stamp'].nanosec]) for p in write_queue]
        (out/'unsent_writes.json').write_text(json.dumps(queued,indent=2)+'\n')
        capture_complete=all((out/name).is_file() for name in ('cloud.npy','distance_points.npy',
            'heartbeat.jsonl','requests.jsonl','requests_sent.jsonl','unsent_writes.json')) and (
                not journal.requests or (out/'wire_journal/index.jsonl').is_file())
        result.update(capture_complete=capture_complete,replay_possible=capture_complete,overall_g2='not_evaluated',
            wire_journal=journal.summary(),
            window='10s pre/5s post for scored failures; startup failures contain no scored unit',
            raw_points=30000,rates=RATES,baseline_adapter=args.baseline,
            input_sha256=hashlib.sha256(cloud.tobytes()).hexdigest())
        (out/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
        node.destroy_node();rclpy.shutdown()
    return int(result['outcome']!='pass')


if __name__=='__main__':raise SystemExit(main())
