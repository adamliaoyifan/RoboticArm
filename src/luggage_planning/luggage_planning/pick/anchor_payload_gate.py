"""Payload validity gate for carrying anchors, before any MoveGroup call.

Why it exists
-------------
Anchor joints (staging / portal / approach / upper) come from offline IK
(`RouteProducer._prepare_candidate`). IK knows kinematics only; it does not
know that the carried box may hit the arm at that joint state. Such a goal was
sent to MoveGroup and refused with -27 GOAL_STATE_INVALID, over and over,
for every slot, because the anchor does not depend on the slot (G1 capture:
`elfin_link1`/`b-payload` at all four goals).

Contract
--------
* Input: nodes of the carrying load state, a state checker with
  ``validate([(q, "carrying")]) -> {(q, "carrying"): bool}``, and the identity
  key of the scene state that checker judges (payload dims/pose, occupancy,
  ACM). The key is part of the cache key: the same q under a different scene
  is judged again, never reused.
* Output: ``GateResult`` with kept node ids, one provenance row per node and a
  named reason for each rejection. A checker failure is ``NOT_EVALUATED``: the
  node is kept (MoveGroup still decides) and the row says so; it is never
  reported as a collision.
* Rejection names ``ANCHOR_PAYLOAD_COLLISION``. It is not BIN_FULL and not a
  physical-unreachability claim.
* Request scope: the cache lives in one ``AnchorPayloadGate`` and the producer
  creates a new one per request.
"""

import dataclasses
from typing import Any, Dict, Iterable, List, Optional, Tuple

REASON_COLLISION = "ANCHOR_PAYLOAD_COLLISION"
REASON_NOT_EVALUATED = "ANCHOR_CHECK_NOT_EVALUATED"
CARRYING = "carrying"


@dataclasses.dataclass
class GateResult:
    kept_node_ids: set
    rows: List[Dict[str, Any]]
    submitted_to_checker: int
    cache_hits: int


def _octet(value):
    return value[0] if isinstance(value, bytes) else int(value)


def _pose_record(pose):
    return [round(float(v), 9) for v in (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)]


def _object_record(obj):
    """Every field of a CollisionObject that changes a collision verdict."""
    return [
        str(obj.id), str(obj.header.frame_id), _octet(obj.operation),
        _pose_record(obj.pose),
        [[_octet(p.type), [round(float(d), 9) for d in p.dimensions]]
         for p in obj.primitives],
        [_pose_record(pose) for pose in obj.primitive_poses],
        [[[[round(float(c), 9) for c in (v.x, v.y, v.z)]
           for v in m.vertices],
          [[int(i) for i in t.vertex_indices] for t in m.triangles]]
         for m in obj.meshes],
        [_pose_record(pose) for pose in obj.mesh_poses],
        [[round(float(v), 9) for v in (
            pl.coef[0], pl.coef[1], pl.coef[2], pl.coef[3])]
         for pl in obj.planes],
        [_pose_record(pose) for pose in obj.plane_poses],
    ]


def live_scene_digest(scene):
    """Semantic digest of a PlanningScene, no CDR bytes.

    Covers world geometry (mesh vertices and triangles included), the full
    allowed-collision matrix (pair entries and default entries) and attached
    objects (geometry, pose, link, touch links).
    """
    import hashlib
    import json

    acm = scene.allowed_collision_matrix
    attached = []
    for item in scene.robot_state.attached_collision_objects:
        attached.append([
            str(item.link_name), sorted(str(t) for t in item.touch_links),
            _object_record(item.object)])
    body = {
        "acm_names": [str(n) for n in acm.entry_names],
        "acm": [[bool(f) for f in row.enabled] for row in acm.entry_values],
        "acm_default_names": [str(n) for n in acm.default_entry_names],
        "acm_default": [bool(v) for v in acm.default_entry_values],
        "world": sorted(
            (_object_record(o) for o in scene.world.collision_objects),
            key=lambda row: row[0]),
        "attached": sorted(attached, key=lambda row: row[2][0]),
    }
    return hashlib.sha256(
        json.dumps(body, sort_keys=True).encode("utf-8")).hexdigest()


class AnchorPayloadGate(object):
    def __init__(self, checker_for_state):
        """``checker_for_state(state_key) -> checker or None``."""
        self._checker_for_state = checker_for_state
        self._verdicts: Dict[Tuple, Optional[bool]] = {}
        self._errors: Dict[Tuple, str] = {}
        self.submitted = 0
        self.cache_hits = 0

    def screen(self, nodes: Iterable, state_key: Any,
               slot_index: Optional[int] = None) -> GateResult:
        nodes = list(nodes)
        kept = set()
        rows = []
        pending = []
        for node in nodes:
            key = (tuple(float(v) for v in node.q), CARRYING, state_key)
            if key in self._verdicts:
                self.cache_hits += 1
            else:
                pending.append((node, key))
        checker = self._checker_for_state(state_key) if pending else None
        if pending:
            unique = list(dict.fromkeys(key for _node, key in pending))
            if checker is None:
                for key in unique:
                    self._verdicts[key] = None
                    self._errors[key] = "no_checker"
            else:
                try:
                    verdicts = checker.validate(
                        [(key[0], CARRYING) for key in unique])
                    self.submitted += len(unique)
                    for key in unique:
                        self._verdicts[key] = bool(
                            verdicts[(key[0], CARRYING)])
                except Exception as error:  # noqa: BLE001 - checker boundary
                    for key in unique:
                        self._verdicts[key] = None
                        self._errors[key] = repr(error)[:200]
        for node in nodes:
            key = (tuple(float(v) for v in node.q), CARRYING, state_key)
            verdict = self._verdicts.get(key)
            row = {
                "slot_index": slot_index,
                "node_id": node.node_id,
                "region": node.region,
                "load_state": str(node.load_state),
                "q": [float(v) for v in node.q],
                "state_key": str(state_key),
            }
            if verdict is False:
                row.update(verdict="rejected", reason=REASON_COLLISION)
            elif verdict is True:
                row.update(verdict="accepted", reason="")
                kept.add(node.node_id)
            else:
                row.update(verdict="not_evaluated", reason=REASON_NOT_EVALUATED,
                           detail=self._errors.get(key, ""))
                kept.add(node.node_id)
            rows.append(row)
        return GateResult(kept, rows, self.submitted, self.cache_hits)
