"""Independent release workload and evidence-based outcome classification.

Geometry, physical instability and missing evidence are different outcomes.
The runner owns execution; this module reads frozen evidence and never moves
cargo or lowers the production area/COM thresholds.
"""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import yaml

from assets import sha256
from planning_types import digest
from release_geometry import release_matrix


@dataclass(frozen=True)
class CalibrationWorkload:
    support_ratios: tuple
    box_yaws_deg: tuple
    repetitions: int
    scheduling_real_time_factor: float
    max_retained_bytes: int

    def __post_init__(self):
        if (not self.support_ratios or any(isinstance(x, bool) or not math.isfinite(x) or not 0 < x <= 1 for x in self.support_ratios)
                or len(set(self.support_ratios)) != len(self.support_ratios)):
            raise ValueError('INVALID_CALIBRATION_SUPPORT_RATIOS')
        if (not self.box_yaws_deg or any(isinstance(x, bool) or not math.isfinite(x) or x % 90 != 0 for x in self.box_yaws_deg)
                or len(set(self.box_yaws_deg)) != len(self.box_yaws_deg)):
            raise ValueError('INVALID_CALIBRATION_YAWS')
        if type(self.repetitions) is not int or self.repetitions < 1:
            raise ValueError('INVALID_CALIBRATION_REPETITIONS')
        if (isinstance(self.scheduling_real_time_factor, bool) or not math.isfinite(self.scheduling_real_time_factor)
                or not 0 < self.scheduling_real_time_factor <= 1):
            raise ValueError('INVALID_CALIBRATION_SCHEDULING_FACTOR')
        if type(self.max_retained_bytes) is not int or self.max_retained_bytes < 1:
            raise ValueError('INVALID_CALIBRATION_RETENTION_BUDGET')

    @classmethod
    def load(cls, path=None):
        values = yaml.safe_load(Path(path or Path(__file__).with_name('release_calibration.yaml')).read_text())
        for key in ('support_ratios', 'box_yaws_deg'):
            values[key] = tuple(values[key])
        return cls(**values)

    @property
    def revision(self):
        return digest(asdict(self))


def calibration_cases(assets, snapshot, state, config, workload):
    return release_matrix(assets, snapshot, state, config, workload.support_ratios,
                          workload.box_yaws_deg, workload.repetitions)


@dataclass(frozen=True)
class CalibrationAssessment:
    case_id: str
    category: str
    stability_outcome: str
    unit_terminal: bool
    reason: str
    execution_admitted: bool = False


@dataclass(frozen=True)
class CalibrationEnrollment:
    campaign_id: str
    workload_revision: str
    case_id: str
    attempt_index: int
    scene_sha256: str

    def __post_init__(self):
        for name in ('campaign_id', 'workload_revision', 'case_id', 'scene_sha256'):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError('INVALID_CALIBRATION_ENROLLMENT:' + name)
        if type(self.attempt_index) is not int or self.attempt_index < 1:
            raise ValueError('INVALID_CALIBRATION_ATTEMPT_INDEX')


def assess_attempt(case, evidence=None, *, campaign_id=None, workload_revision=None):
    """A reason string alone cannot establish a physical calibration result.

    Stable needs a complete positive replay; unstable needs a detached,
    contact-free first instability witness replayed from native observations.
    Other failures remain unmeasured, even if the process exited cleanly.
    """
    def result(category, reason, outcome='not_evaluated', terminal=False):
        return CalibrationAssessment(case.case_id, category, outcome, terminal, reason)
    if not case.geometry_available:
        return result('geometrically_unavailable', case.reason, terminal=True)
    if evidence is None:
        return result('pending', 'NO_ATTEMPT')
    folder = Path(evidence)
    def read(name):
        path = folder/name
        return json.loads(path.read_text()) if path.is_file() else None
    manifest = read('manifest.json'); spec = read('release_scene.json')
    if not manifest or not spec:
        return result('not_evaluated', 'ATTEMPT_METADATA_MISSING')
    if digest(spec['case']) != digest(asdict(case)):
        return result('not_evaluated', 'ATTEMPT_CASE_MISMATCH')
    enrollment = manifest.get('calibration_enrollment') or {}
    if (not campaign_id or not workload_revision or enrollment.get('campaign_id') != campaign_id
            or enrollment.get('workload_revision') != workload_revision
            or enrollment.get('case_id') != case.case_id
            or enrollment.get('scene_sha256') != manifest.get('scene_sha256')
            or type(enrollment.get('attempt_index')) is not int or enrollment['attempt_index'] < 1):
        return result('not_evaluated', 'MATRIX_ENROLLMENT_MISSING_OR_MISMATCHED')
    hashes = manifest.get('artifact_hashes')
    required = ('release_scene.json', 'release.sdf', 'native.jsonl.gz', 'clock.jsonl.gz', 'calibration_enrollment.json')
    if not hashes or any(name not in hashes for name in required):
        return result('not_evaluated', 'ATTEMPT_HASH_INVENTORY_MISSING')
    for name, checksum in hashes.items():
        path = (folder/name).resolve()
        if not path.is_relative_to(folder.resolve()) or not path.is_file() or sha256(path) != checksum:
            return result('not_evaluated', 'ATTEMPT_ARTIFACT_HASH_MISMATCH')
    if (digest(read('calibration_enrollment.json')) != digest(enrollment)
            or manifest.get('scene_sha256') != spec.get('world_sha256')
            or sha256(folder/'release.sdf') != manifest.get('scene_sha256')):
        return result('not_evaluated', 'MATRIX_INPUT_IDENTITIES_MISMATCHED')
    if not manifest.get('capture_complete') or not manifest.get('replay_possible') or not manifest.get('teardown_complete'):
        return result('not_evaluated', 'ATTEMPT_CAPTURE_OR_TEARDOWN_INCOMPLETE')
    audit = read('audit.json'); success = read('result.json')
    if success and success.get('valid') is True and audit and audit.get('valid') is True:
        if any(success.get(name) is not True for name in ('native_detach_confirmed',
                'withdrawal_completed', 'independent_release_stability_observed')):
            return result('not_evaluated', 'PHYSICAL_COMPLETION_FLAGS_MISSING')
        checks = audit.get('checks', {})
        required_checks = ('all_captured_native_replayed', 'native_continuity', 'nominal_certificate',
            'native_detach_confirmed', 'both_motions_started', 'both_motions_completed',
            'no_monitor_failure', 'full_independent_stability', 'no_stop_event', 'live_result', 'capture', 'teardown')
        if (any(checks.get(key) is not True for key in required_checks)
                or 'audit.json' not in hashes or 'result.json' not in hashes):
            return result('not_evaluated', 'STABILITY_REPLAY_GATES_INCOMPLETE')
        return result('calibration_pass', 'INDEPENDENT_RELEASE_STABILITY_OBSERVED', 'stable', True)
    failure = read('failure.json'); witness = read('instability_audit.json')
    if failure and failure.get('error') == 'RELEASE_BOX_UNSTABLE' and witness and witness.get('valid') is True:
        checks = witness.get('checks', {})
        required_checks = ('all_captured_native_replayed', 'native_continuity', 'first_failure_reproduced',
            'detached_at_failure', 'tool_contact_free_through_failure', 'no_payload_commands',
            'valid_support_contacts_through_failure', 'baseline_valid', 'capture_complete', 'teardown')
        if (any(checks.get(key) is not True for key in required_checks)
                or any(name not in hashes for name in ('instability_audit.json', 'failure.json', 'failed_sample.json'))):
            return result('not_evaluated', 'INSTABILITY_REPLAY_GATES_INCOMPLETE')
        return result('physical_instability', 'DETACHED_UNASSISTED_BOX_EXCEEDED_STABILITY_LIMIT', 'unstable', True)
    return result('not_evaluated', failure.get('error', 'PHYSICAL_VERDICT_MISSING') if failure else 'PHYSICAL_VERDICT_MISSING')
