"""Bound the minimum noncontact clearance on an already planned joint locus.

Distance rows are conservative pointwise lower bounds; robot_minimum_m is an
exact minimum at that sample. The caller supplies source-geometry relative
motion bounds. No rejected search branch or global scene accumulator is used.
Contact-mode certification remains a separate prerequisite of the plan.
"""
from dataclasses import dataclass
import math
import time
import numpy as np


@dataclass(frozen=True)
class DistanceSample:
    distances: tuple  # (lower_bound_m, pair_tuple, 'robot' or 'payload')
    robot_minimum_m: float
    robot_witness: tuple
    failure: dict | None = None


def bound_joint_path(path,evaluate,motion_bound,margin_m,precision_m,deadline,emit=None):
    """Return a narrow interval containing the true robot clearance minimum.

    Cubic zero-velocity joint segments have the same locus as q-linear segments.
    Accepted interval distance bounds also prove the stated hard margin for all
    noncontact pairs. The finite deadline returns invalid, never a partial score.
    """
    path=np.asarray(path,float)
    if (path.ndim!=2 or len(path)<2 or not np.isfinite(path).all() or
        not math.isfinite(margin_m) or margin_m<=0 or
        not math.isfinite(precision_m) or precision_m<=0):
        raise ValueError('INVALID_CLEARANCE_PATH_OR_POLICY')
    robot_upper=float('inf');robot_lower=float('inf');payload_lower=float('inf')
    upper_witness=lower_witness=payload_witness=None
    checks=leaves=0
    def rejected(reason,**details):
        return dict(valid=False,reason=reason,checks=checks,certified_intervals=leaves,**details)
    for segment,(qa,qb) in enumerate(zip(path,path[1:])):
        pending=[(0.,1.)]
        while pending:
            if time.monotonic()>=deadline:return rejected('CLEARANCE_SCORE_BUDGET_EXHAUSTED')
            low,high=pending.pop();mid=(low+high)/2
            q=(1-mid)*qa+mid*qb
            sample=evaluate(segment,mid,q);checks+=1
            if sample.failure:return rejected('CLEARANCE_SCORE_STATE_REJECTED',diagnostic=sample.failure)
            if not sample.distances or not math.isfinite(sample.robot_minimum_m):
                return rejected('CLEARANCE_SCORE_MISSING_DISTANCE_EVIDENCE')
            if sample.robot_minimum_m<robot_upper:
                robot_upper=sample.robot_minimum_m
                upper_witness=dict(segment=segment,fraction=mid,q=q.tolist(),pair=sample.robot_witness)
            delta=np.abs(qb-qa)*(high-low)/2
            local_robot=float('inf');local_payload=float('inf');local_pair=None;local_payload_pair=None
            proven=True
            for distance,pair,kind in sample.distances:
                bound=float(motion_bound(pair,delta,segment,low,high))
                if not math.isfinite(distance) or not math.isfinite(bound) or bound<0:
                    return rejected('CLEARANCE_SCORE_INVALID_DISTANCE_BOUND',pair=pair)
                value=float(distance)-bound
                proven=proven and value>=margin_m-1e-10
                if kind=='robot' and value<local_robot:local_robot=value;local_pair=pair
                elif kind=='payload' and value<local_payload:local_payload=value;local_payload_pair=pair
                elif kind not in ('robot','payload'):return rejected('UNKNOWN_CLEARANCE_PAIR_KIND',kind=kind)
            if not math.isfinite(local_robot):return rejected('CLEARANCE_SCORE_NO_ROBOT_PAIRS')
            if proven and local_robot>=robot_upper-precision_m:
                leaves+=1
                if local_robot<robot_lower:
                    robot_lower=local_robot
                    lower_witness=dict(segment=segment,fraction_bounds=[low,high],pair=local_pair)
                if local_payload<payload_lower:
                    payload_lower=local_payload
                    payload_witness=dict(segment=segment,fraction_bounds=[low,high],pair=local_payload_pair)
                if emit:emit(dict(event='clearance_interval',segment=segment,fraction_bounds=[low,high],
                    robot_lower_m=local_robot,robot_sample_upper_m=sample.robot_minimum_m,
                    payload_lower_m=local_payload if math.isfinite(local_payload) else None))
            elif mid==low or mid==high:
                return rejected('UNCERTIFIABLE_CLEARANCE_SCORE_INTERVAL',segment=segment)
            else:pending.extend(((mid,high),(low,mid)))
    if robot_upper-robot_lower>precision_m+1e-10:
        return rejected('CLEARANCE_SCORE_PRECISION_NOT_PROVEN')
    return dict(valid=True,checks=checks,certified_intervals=leaves,
        robot_lower_m=robot_lower,robot_upper_m=robot_upper,precision_m=precision_m,
        robot_lower_witness=lower_witness,robot_upper_witness=upper_witness,
        payload_lower_m=payload_lower if math.isfinite(payload_lower) else None,
        payload_lower_witness=payload_witness,
        scope='source_distance_and_relative_motion_bounds_in_declared_nominal_contact_modes')
