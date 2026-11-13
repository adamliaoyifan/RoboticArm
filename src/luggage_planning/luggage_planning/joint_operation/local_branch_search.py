"""Model-derived whole-place-path winding proposals, never motion authority.

Uniform integer turns preserve revolute-joint geometry. Every path point must
remain within the actual model's bounds; final native whole-chain certification
still decides admission. Raw native branches remain available as fallbacks.
"""
from dataclasses import replace
import math

from luggage_planning.joint_operation.contracts import OPERATION_JOINTS


class LocalBranchSearch:
    def __init__(self, model):
        urdf = model.urdf
        moving = tuple(name for name in model.path if urdf.joints[name]['type'] != 'fixed')
        limits = tuple(urdf.joint_limits())
        if (moving != OPERATION_JOINTS or len(limits) != 6
                or any(urdf.joints[name]['type'] != 'revolute' for name in moving)
                or any(not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi for lo,hi in limits)):
            raise ValueError('canonical bounded revolute model required for branch proposals')
        self.limits = limits

    def placements(self, pick, branches, *, trace):
        """Keep place-family diversity before trying the original windings.

        Each original family contributes its nearest legal variant first. This
        is a joint-travel heuristic, not proof that its connection is easier.
        Offsets apply to the entire incoming/outgoing path, never individual
        samples, and are selected before any plan is issued or executed.
        """
        preferred, fallback = [], []
        reference = pick.outgoing[-1]
        for branch in branches:
            turns = []
            rows = branch.incoming+branch.outgoing
            for j, (lower,upper) in enumerate(self.limits):
                values = tuple(row[j] for row in rows)
                if min(values) < lower or max(values) > upper:
                    # Preserve the raw proposal for independent native refusal;
                    # never repair an invalid branch by clipping or wrapping.
                    turns = [0]*6
                    break
                first = math.ceil((lower-min(values))/math.tau)
                last = math.floor((upper-max(values))/math.tau)
                ideal = (reference[j]-branch.incoming[0][j])/math.tau
                choices = {max(first,min(last,k)) for k in (math.floor(ideal),math.ceil(ideal),0)}
                turns.append(min(choices,key=lambda k: (
                    abs(branch.incoming[0][j]+k*math.tau-reference[j]),abs(k),k)))
            if not any(turns):
                preferred.append(branch)
                continue
            def shifted(path):
                return tuple(tuple(q+k*math.tau for q,k in zip(row,turns)) for row in path)
            incoming,outgoing = shifted(branch.incoming),shifted(branch.outgoing)
            if not all(lo <= q <= hi for row in incoming+outgoing
                       for q,(lo,hi) in zip(row,self.limits)):
                preferred.append(branch)
                continue
            proposal = replace(branch,branch_id=branch.branch_id+':turns:'+','.join(map(str,turns)),
                               incoming=incoming,outgoing=outgoing)
            preferred.append(proposal)
            fallback.append(branch)
            trace.append(('equivalent_local_branch',pick.branch_id,branch.branch_id,
                          proposal.branch_id,tuple(turns)))
        return tuple(preferred+fallback)
