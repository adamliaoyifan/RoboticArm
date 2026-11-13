"""Bounded durable successful-path proposals; never store an authorization.

Keys bind robot, ULD and spatial endpoints, not map revision or box dimensions.
A hit is assembled under the new identity and geometry and must pass the full
native certifier. Original timing and joint winding are retained. Only the
execution owner promotes terminal successful operations.
"""
from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zlib

from luggage_planning.joint_operation.contracts import (
    JointKnot, OperationLeg, JointOperationPlan, OperationBarrier,
    seal_operation_plan,
)


@dataclass(frozen=True)
class CachedPathProposal:
    plan: JointOperationPlan
    source_plan_digest: str
    source_geometry_digest: str
    source_acquisition_stamp_ns: int


class SuccessfulPathCache:
    def __init__(self, directory, *, capacity=16, max_entry_bytes=8*1024*1024):
        self.directory = Path(directory)
        self.capacity, self.max_entry_bytes = int(capacity), int(max_entry_bytes)
        if not directory or not 1 <= self.capacity <= 64 or not 1 <= self.max_entry_bytes <= 32*1024*1024:
            raise ValueError('explicit bounded path cache required')
        self.directory.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _key(identity, start, pick_box, pick_tool, release_box, release_tool, settled_box):
        data = [identity.model_sha256,identity.geometry_hash,identity.scene_revision,
                list(start),*[asdict(p) for p in (pick_tool,release_tool)]]
        return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

    def remember(self, plan, *, success, motion_terminal, may_have_payload):
        if success is not True or motion_terminal is not True or may_have_payload is not False:
            return False
        key = self._key(plan.identity,plan.start_joints,plan.world_from_pick_box,plan.world_from_pick_tool,
            plan.world_from_place_box,plan.world_from_place_tool,plan.world_from_settled_box)
        payload = dict(schema=1,key=key,source_plan_digest=plan.plan_digest,
                       source_geometry_digest=plan.predicted_attachment.digest(),
                       source_acquisition_stamp_ns=plan.identity.observation_stamp_ns,
                       legs=[asdict(leg) for leg in plan.legs])
        raw = json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        if len(raw) > self.max_entry_bytes: return False
        stored = gzip.compress(raw,compresslevel=1,mtime=0)
        fd, name = tempfile.mkstemp(prefix='.pending-',dir=self.directory)
        try:
            with os.fdopen(fd,'wb') as output:
                output.write(stored)
                output.flush()
                os.fsync(output.fileno())
            os.replace(name,self.directory/(key+'.json.gz'))
        finally:
            if os.path.exists(name): os.unlink(name)
        entries = sorted(self.directory.glob('*.json.gz'),key=lambda p:p.stat().st_mtime_ns,reverse=True)
        for path in entries[self.capacity:]: path.unlink(missing_ok=True)
        return True

    def proposal(self, identity, start, candidate):
        key = self._key(identity,start,candidate.world_from_pick_box,candidate.world_from_pick_tool,
            candidate.world_from_release_box,candidate.world_from_release_tool,candidate.world_from_settled_box)
        path = self.directory/(key+'.json.gz')
        try:
            if path.stat().st_size > self.max_entry_bytes: return None
            with gzip.open(path,'rb') as stream: raw = stream.read(self.max_entry_bytes+1)
            if len(raw) > self.max_entry_bytes: return None
            record = json.loads(raw)
            if record['schema'] != 1 or record['key'] != key: return None
            legs = tuple(OperationLeg(row['name'],row['phase'],identity.request_id,identity.scene_revision,
                identity.robot_state_stamp_ns,tuple(row['joint_names']),
                tuple(JointKnot(**knot) for knot in row['knots'])) for row in record['legs'])
            draft = JointOperationPlan(identity,candidate.candidate_id,'pending-native-certification',
                candidate.world_from_pick_box,candidate.world_from_pick_tool,candidate.world_from_release_box,
                candidate.world_from_release_tool,candidate.world_from_settled_box,candidate.predicted_attachment,
                start,legs,(OperationBarrier(2,'attachment_confirmed'),OperationBarrier(5,'release_confirmed')),'')
            return CachedPathProposal(seal_operation_plan(draft),record['source_plan_digest'],
                record['source_geometry_digest'],int(record['source_acquisition_stamp_ns']))
        except (OSError,EOFError,ValueError,TypeError,KeyError,OverflowError,zlib.error):
            return None
