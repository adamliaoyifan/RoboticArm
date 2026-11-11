"""Release initialization checks, independent of transport and execution effects.

These checks only establish an attached, supported and settled starting state.
They never authorize a robot trajectory or certify stability after detachment.
"""
from dataclasses import asdict
import numpy as np

from execution_monitor import pose_error
from observed_support import measure_observed_support
from support_contact import support_contact_failure


def inspect_release_start(sample, dimensions, com_local, config, policy, baseline=None):
    support = measure_observed_support(sample, dimensions, com_local,
        config.contact_tolerance_m, config.contact_penetration_m)
    failures = []
    if not sample.attached or not sample.constraint_present or sample.attachment_create_count != 1 or sample.attachment_detach_count != 0:
        failures.append(dict(reason='RELEASE_INITIAL_ATTACHMENT_INVALID'))
    box = np.asarray(sample.container_T_box).reshape(4, 4)
    tool = np.asarray(sample.container_T_tool).reshape(4, 4)
    translation, rotation = pose_error(tool @ np.array(sample.tool_T_box_at_attach).reshape(4, 4), box)
    if translation > config.contact_tolerance_m or rotation > policy.attachment_rotation_error_deg:
        failures.append(dict(reason='RELEASE_INITIAL_ATTACHMENT_RELATION_CHANGED'))
    bodies = {body.name + '::body::collision': body for body in sample.lower}
    point_count = 0
    for contact in sample.contacts:
        body = bodies.get(contact.other_collision)
        if body is None:
            # Tool/payload contacts need the separate CAD patch contract; this
            # readiness component cannot silently approve an unknown pair.
            failures.append(dict(reason='RELEASE_UNCLASSIFIED_PAYLOAD_CONTACT', other=contact.other_collision))
            continue
        point_count += 1
        half = np.asarray(body.dimensions) / 2
        failure = support_contact_failure(box, dimensions, body.container_T_body, [-half, half],
            contact.point_container, contact.normal_toward_payload_container,
            config.contact_tolerance_m, config.contact_penetration_m,
            min(config.contact_normal_alignment_deg, policy.contact_normal_alignment_deg),
            contact.depth_m, normal_toward_box=True)
        if failure:
            failures.append(dict(reason='RELEASE_INVALID_NATIVE_SUPPORT', body=body.name, diagnostic=failure))
    if not point_count:
        failures.append(dict(reason='RELEASE_NATIVE_SUPPORT_MISSING'))
    if support.near_contact_area_m2 <= 0:
        failures.append(dict(reason='RELEASE_GEOMETRIC_SUPPORT_MISSING'))
    if support.near_contact_area_ratio + 1e-10 < config.support_area_ratio_min:
        failures.append(dict(reason='RELEASE_SUPPORT_AREA_BELOW_POLICY'))
    if support.com_margin_m + 1e-10 < config.support_com_margin_m:
        failures.append(dict(reason='RELEASE_COM_MARGIN_BELOW_POLICY'))
    lower_translation = lower_rotation = box_translation = box_rotation = 0.
    if baseline is not None:
        old = {body.name: body for body in baseline.lower}
        if set(old) != {body.name for body in sample.lower}:
            raise ValueError('RELEASE_BASELINE_LOWER_SET_CHANGED')
        for body in sample.lower:
            if body.dimensions != old[body.name].dimensions:
                raise ValueError('RELEASE_BASELINE_LOWER_SIZE_CHANGED')
            distance, angle = pose_error(old[body.name].container_T_body, body.container_T_body)
            lower_translation = max(lower_translation, distance); lower_rotation = max(lower_rotation, angle)
        box_translation, box_rotation = pose_error(baseline.container_T_box, sample.container_T_box)
        if lower_translation > policy.lower_translation_m or lower_rotation > policy.lower_rotation_deg:
            failures.append(dict(reason='RELEASE_INITIAL_LOWER_NOT_SETTLED'))
        if box_translation > policy.stability_translation_m or box_rotation > policy.stability_rotation_deg:
            failures.append(dict(reason='RELEASE_INITIAL_BOX_NOT_SETTLED'))
    policy_reasons = {'RELEASE_SUPPORT_AREA_BELOW_POLICY', 'RELEASE_COM_MARGIN_BELOW_POLICY'}
    policy_failures = [failure for failure in failures if failure['reason'] in policy_reasons]
    physical_failures = [failure for failure in failures if failure['reason'] not in policy_reasons]
    return dict(accepted=not failures, failures=failures, support=asdict(support),
        physical_preconditions_satisfied=not physical_failures,
        execution_policy_satisfied=not policy_failures,
        execution_policy_failures=policy_failures,
        native_support_point_count=point_count, lower_translation_m=lower_translation,
        lower_rotation_deg=lower_rotation, box_translation_m=box_translation,
        box_rotation_deg=box_rotation, attachment_translation_m=translation,
        attachment_rotation_deg=rotation, execution_admitted=False,
        scope='attached_initialization_only_not_post_release_or_motion_admission')


class ReleaseReadyWindow:
    """Require uninterrupted valid observations for the configured stable dwell."""
    def __init__(self, dimensions, com_local, config, policy):
        self._dimensions = tuple(dimensions); self._com = tuple(com_local)
        self._config = config; self._policy = policy
        self._baseline = None; self._last_stamp = None

    @property
    def baseline(self):
        return self._baseline

    def observe(self, sample):
        return self._observe(sample, for_calibration=False)

    def _observe(self, sample, *, for_calibration):
        gap = 0. if self._last_stamp is None else sample.stamp_s - self._last_stamp
        if self._last_stamp is not None and gap <= 0:
            raise ValueError('RELEASE_READY_STAMP_NOT_INCREASING')
        self._last_stamp = sample.stamp_s
        if gap > self._policy.monitor_max_gap_s + 1e-9:
            self._baseline = None
            return dict(ready=False, accepted=False, failures=[dict(reason='RELEASE_READY_OBSERVATION_GAP')],
                        stable_duration_s=0., execution_admitted=False)
        result = inspect_release_start(sample, self._dimensions, self._com,
            self._config, self._policy, self._baseline)
        eligible = result['physical_preconditions_satisfied'] if for_calibration else result['accepted']
        if not eligible:
            self._baseline = None
        elif self._baseline is None:
            self._baseline = sample
        duration = 0. if self._baseline is None else sample.stamp_s - self._baseline.stamp_s
        result.update(ready=bool(eligible and duration + 1e-9 >= self._policy.stability_dwell_s),
            stable_duration_s=duration,
            baseline_s=self._baseline.stamp_s if self._baseline else None)
        return result


class CalibrationReadyWindow(ReleaseReadyWindow):
    """Independent below-policy experiments; never produce execution readiness.

    Policy failures remain explicit. Only area/COM thresholds are diagnostic;
    all native-contact, attachment, lower-motion and stability checks retain
    their original limits. Consumers must request calibration_ready explicitly.
    """
    def observe(self, sample):
        result = self._observe(sample, for_calibration=True)
        result['calibration_ready'] = result['ready']
        result['ready'] = False
        result['purpose'] = 'independent_release_calibration'
        return result
