"""B2: a placed box that hits no voxel must not commit."""

import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper


def _mapper():
    return CargoVolumeMapper(
        inner_size=[1.0, 1.0, 1.0],
        center_base=[0.0, 0.0, 0.5],
        yaw=0.0,
        resolution=0.1,
    )


class TestZeroVoxelCommit(unittest.TestCase):
    def test_in_grid_box_writes_voxels_and_advances_revision(self):
        mapper = _mapper()
        before = mapper.stats()
        self.assertTrue(mapper.mark_placed_box(
            [0.0, 0.0, 0.15], [0.2, 0.2, 0.2], yaw=0.0))
        after = mapper.stats()
        self.assertEqual(after["map_revision"], before["map_revision"] + 1)
        self.assertGreater(after["occupied_count"], 0)
        self.assertEqual(after["committed_box_count"], 1)
        self.assertGreater(max(max(row) for row in mapper.surface_map_2d()["height"]), 0.0)

    def test_out_of_grid_box_does_not_commit(self):
        mapper = _mapper()
        before = mapper.stats()
        with self.assertRaises(ValueError) as caught:
            mapper.mark_placed_box([50.0, 50.0, 0.15], [0.2, 0.2, 0.2])
        self.assertIn("zero voxels", str(caught.exception))
        after = mapper.stats()
        self.assertEqual(after["map_revision"], before["map_revision"])
        self.assertEqual(after["occupied_count"], 0)
        self.assertEqual(after["committed_box_count"], 0)
        self.assertEqual(mapper.commit_ledger(), [])
