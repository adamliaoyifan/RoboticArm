import unittest
from types import SimpleNamespace

from elfin_trajectory_executor.livox_codec import _PACK, _POINT_STEP, custom_points_to_cloud


class LivoxCodecTest(unittest.TestCase):
    def test_custom_points_pack_xyzrtl(self):
        header = SimpleNamespace(stamp=None, frame_id="livox_frame")
        pts = [
            SimpleNamespace(
                x=1.0, y=-2.0, z=0.5, reflectivity=12, tag=3, line=1, offset_time=1000
            ),
            SimpleNamespace(
                x=0.0, y=0.0, z=1.0, reflectivity=0, tag=0, line=2, offset_time=2000
            ),
        ]
        cloud = custom_points_to_cloud(header, pts)
        self.assertEqual(cloud.width, 2)
        self.assertEqual(cloud.point_step, _POINT_STEP)
        self.assertEqual(len(cloud.data), 2 * _POINT_STEP)
        x, y, z, inten, tag, line, ts = _PACK.unpack_from(cloud.data, 0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, -2.0)
        self.assertAlmostEqual(z, 0.5)
        self.assertAlmostEqual(inten, 12.0)
        self.assertEqual(tag, 3)
        self.assertEqual(line, 1)
        self.assertAlmostEqual(ts, 1000.0)
        self.assertEqual(cloud.fields[-1].name, "timestamp")
        self.assertEqual(cloud.fields[-1].offset, 18)
