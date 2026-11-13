"""Exercise the actual CLI boundary before any simulator can be constructed."""
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from release_calibration import CalibrationEnrollment
from release_scene import write_release_scene
import release_trial
import test_release_scene as fixtures


class EnrollmentEntryContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=fixtures.ReleaseWorldContracts;cls.fixture.setUpClass()
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        f=cls.fixture
        cls.scene=cls.root/'scene'
        cls.spec=write_release_scene(f.assets,f.snapshot,f.state,f.config,f.case,f.policy,
            f.source,f.plugin,cls.scene)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup();cls.fixture.tearDownClass()

    def invoke(self,enrollment):
        path=self.root/'enrollment.json';path.write_text(json.dumps(asdict(enrollment)))
        argv=['release_trial.py','--scene-dir',str(self.scene),'--evidence',str(self.root/'docs/status/evidence/attempt'),
              '--joint-bridge',str(self.root/'unused-bridge'),'--calibration-enrollment',str(path)]
        with patch.object(release_trial,'COORD',self.root),patch('sys.argv',argv),\
             patch.object(release_trial,'assert_sim_slot_free',side_effect=RuntimeError('STOP_BEFORE_SIM')) as gate,\
             patch.object(release_trial,'ReleaseRuntime') as runtime:
            try:release_trial.main()
            except (ValueError,RuntimeError) as error:message=str(error)
            runtime.assert_not_called()
            return message,gate.call_count

    def test_wrong_case_or_world_rejected_before_sim_slot_or_output(self):
        for case,world in (('other',self.spec['world_sha256']),
                           (self.spec['case']['case_id'],'other')):
            message,calls=self.invoke(CalibrationEnrollment('campaign','workload',case,1,world))
            self.assertEqual(message,'CALIBRATION_ENROLLMENT_SCENE_MISMATCH');self.assertEqual(calls,0)
        self.assertFalse((self.root/'docs/status/evidence/attempt').exists())

    def test_matching_input_reaches_slot_gate_without_constructing_sim(self):
        message,calls=self.invoke(CalibrationEnrollment('campaign','workload',self.spec['case']['case_id'],1,self.spec['world_sha256']))
        self.assertEqual(message,'STOP_BEFORE_SIM');self.assertEqual(calls,1)
        self.assertFalse((self.root/'docs/status/evidence/attempt').exists())


if __name__=='__main__':unittest.main()
