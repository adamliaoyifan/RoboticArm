"""Separate observed box task identity from actuator instance identity.

An observed suction candidate identifies the pickup instance/generation, not its
size or pose. The simulation actuator still verifies that instance against its
current registry; hardware backends do not consume the model name.
"""
from luggage_description.frame_contracts import ContractError


def actuator_instance_id(request):
    candidates = request.measured_box.suction_candidates
    if not candidates:
        # Legacy callers already supply the instance as box_id.
        return str(request.box_id)
    identities = {(str(candidate.instance_id), int(candidate.generation))
                  for candidate in candidates}
    if len(identities) != 1:
        raise ContractError('ATTACHMENT_INSTANCE_AMBIGUOUS', 'suction_candidates',
                            'one observed instance/generation', sorted(identities))
    instance, generation = next(iter(identities))
    if not instance or generation != int(request.generation):
        raise ContractError('ATTACHMENT_INSTANCE_MISMATCH', 'suction_candidates',
                            'nonempty instance in requested generation', (instance, generation))
    return instance
