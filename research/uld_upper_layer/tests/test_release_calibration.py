"""Workload identity and evidence consumption; synthetic files are not physics."""
from dataclasses import asdict,replace
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from assets import sha256
from planning_types import SupportMetrics
from release_geometry import ReleaseGeometryCase
from release_calibration import CalibrationWorkload,CalibrationEnrollment,calibration_cases,assess_attempt
import test_release_geometry as geometry_fixtures


STABLE_CHECKS=('all_captured_native_replayed','native_continuity','nominal_certificate',
 'native_detach_confirmed','both_motions_started','both_motions_completed','no_monitor_failure',
 'full_independent_stability','no_stop_event','live_result','capture','teardown')
UNSTABLE_CHECKS=('all_captured_native_replayed','native_continuity','first_failure_reproduced',
 'detached_at_failure','tool_contact_free_through_failure','no_payload_commands',
 'valid_support_contacts_through_failure','baseline_valid','capture_complete','teardown')


class CalibrationContracts(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.folder=Path(self.temp.name)
        support=SupportMetrics(.1,.4,'POLYGON EMPTY','POLYGON EMPTY',-.02,('lower',),False,'below_policy')
        self.case=ReleaseGeometryCase('case-a',.4,0.,0,True,'geometry_only',tuple(np.eye(4).ravel()),
                                     tuple(np.eye(4).ravel()),support,'source-id')
        self.workload=CalibrationWorkload.load()
        self.expected=dict(campaign_id='matrix-test',workload_revision=self.workload.revision)

    def write(self,name,value):
        (self.folder/name).write_text(json.dumps(value))

    def freeze(self):
        manifest=json.loads((self.folder/'manifest.json').read_text())
        manifest['artifact_hashes']={p.name:sha256(p) for p in self.folder.iterdir() if p.name!='manifest.json'}
        self.write('manifest.json',manifest)

    def evidence(self,unstable=False):
        for name in ('release.sdf','native.jsonl.gz','clock.jsonl.gz'):
            (self.folder/name).write_bytes(b'opaque-unit-fixture-not-a-physical-run')
        world=sha256(self.folder/'release.sdf')
        enrollment=CalibrationEnrollment('matrix-test',self.workload.revision,self.case.case_id,1,world)
        self.write('calibration_enrollment.json',asdict(enrollment))
        self.write('release_scene.json',dict(case=asdict(self.case),world_sha256=world))
        self.write('manifest.json',dict(capture_complete=True,replay_possible=True,teardown_complete=True,scene_sha256=world,
                                      calibration_enrollment=asdict(enrollment)))
        if unstable:
            self.write('failure.json',dict(error='RELEASE_BOX_UNSTABLE'))
            self.write('failed_sample.json',dict(scope='opaque-test-witness'))
            self.write('instability_audit.json',dict(valid=True,checks=dict.fromkeys(UNSTABLE_CHECKS,True)))
        else:
            self.write('result.json',dict(valid=True,native_detach_confirmed=True,withdrawal_completed=True,independent_release_stability_observed=True))
            self.write('audit.json',dict(valid=True,checks=dict.fromkeys(STABLE_CHECKS,True)))
        self.freeze()

    def test_unavailable_pending_and_diagnostic_pilot_are_distinct(self):
        self.assertEqual(assess_attempt(replace(self.case,geometry_available=False)).category,'geometrically_unavailable')
        self.assertEqual(assess_attempt(self.case).category,'pending')
        self.evidence()
        m=json.loads((self.folder/'manifest.json').read_text());m['calibration_enrollment']=None
        self.write('manifest.json',m)
        self.assertEqual(assess_attempt(self.case,self.folder,**self.expected).category,'not_evaluated')

    def test_stable_requires_complete_motion_and_stability_replay(self):
        self.evidence();a=assess_attempt(self.case,self.folder,**self.expected)
        self.assertEqual(a.stability_outcome,'stable');self.assertTrue(a.unit_terminal);self.assertFalse(a.execution_admitted)
        for missing in ('both_motions_completed','full_independent_stability','native_detach_confirmed'):
            self.write('audit.json',dict(valid=True,checks={**dict.fromkeys(STABLE_CHECKS,True),missing:False}));self.freeze()
            self.assertEqual(assess_attempt(self.case,self.folder,**self.expected).category,'not_evaluated')

    def test_error_text_alone_or_assisted_instability_is_not_a_physical_verdict(self):
        self.evidence(unstable=True)
        a=assess_attempt(self.case,self.folder,**self.expected)
        self.assertEqual(a.stability_outcome,'unstable');self.assertTrue(a.unit_terminal)
        for missing in ('detached_at_failure','tool_contact_free_through_failure','first_failure_reproduced'):
            self.write('instability_audit.json',dict(valid=True,checks={**dict.fromkeys(UNSTABLE_CHECKS,True),missing:False}));self.freeze()
            self.assertEqual(assess_attempt(self.case,self.folder,**self.expected).category,'not_evaluated')
        (self.folder/'instability_audit.json').unlink();self.freeze()
        self.assertEqual(assess_attempt(self.case,self.folder,**self.expected).category,'not_evaluated')

    def test_case_workload_identity_hashes_and_teardown_enforced(self):
        self.evidence()
        self.assertEqual(assess_attempt(replace(self.case,repetition=1),self.folder,**self.expected).reason,'ATTEMPT_CASE_MISMATCH')
        self.assertEqual(assess_attempt(self.case,self.folder,campaign_id='another',workload_revision=self.workload.revision).category,'not_evaluated')
        (self.folder/'native.jsonl.gz').write_bytes(b'changed-after-freezing')
        self.assertEqual(assess_attempt(self.case,self.folder,**self.expected).reason,'ATTEMPT_ARTIFACT_HASH_MISMATCH')
        self.freeze();m=json.loads((self.folder/'manifest.json').read_text());m['teardown_complete']=False;self.write('manifest.json',m)
        self.assertEqual(assess_attempt(self.case,self.folder,**self.expected).reason,'ATTEMPT_CAPTURE_OR_TEARDOWN_INCOMPLETE')

    def test_workload_and_attempt_indices_are_validated(self):
        for kwargs in (dict(support_ratios=(.4,.4)),dict(support_ratios=(0.,)),dict(box_yaws_deg=(30.,)),
                       dict(repetitions=True),dict(scheduling_real_time_factor=2.),dict(max_retained_bytes=0)):
            with self.assertRaises(ValueError):replace(self.workload,**kwargs)
        with self.assertRaises(ValueError):CalibrationEnrollment('c','r','case',True,'hash')


class ActualCalibrationCatalog(unittest.TestCase):
    def test_default_catalog_has_unique_54_units_without_counting_geometry_as_physics(self):
        f=geometry_fixtures.ReleaseGeometryContracts
        f.setUpClass()
        try:
            cases=calibration_cases(f.assets,f.snapshot,f.state,f.config,CalibrationWorkload.load())
            self.assertEqual(len(cases),54);self.assertEqual(len({c.case_id for c in cases}),54)
            self.assertEqual(sum(c.geometry_available for c in cases),21)
            self.assertEqual(sum(not c.geometry_available for c in cases),33)
            self.assertTrue(all(assess_attempt(c).stability_outcome=='not_evaluated' for c in cases))
        finally:f.tearDownClass()


if __name__=='__main__':unittest.main()
