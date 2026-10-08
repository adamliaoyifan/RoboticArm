"""The Gazebo spawn file can lose one panel collision without losing the visual."""
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET

from luggage_description.gazebo_spawn_collision import (
    BOX_CARRY_MASK, BOX_FULL_MASK, P04_MASK,
    masks_collide, panel_collision_absent, set_collision_bitmask_for_mesh,
    strip_link_collision_mesh)

_URDF = """<?xml version="1.0"?>
<robot name="S20">
  <link name="elfin_link6">
    <collision><geometry><mesh filename="package://x/link6.stl"/></geometry></collision>
  </link>
  <link name="suction_panel">
    <visual><geometry><mesh filename="package://x/p04.stl"/></geometry></visual>
    <collision><geometry><mesh filename="package://x/p01.stl"/></geometry></collision>
    <collision><geometry><mesh filename="package://x/p04.stl"/></geometry></collision>
    <collision><geometry><mesh filename="package://x/p05.stl"/></geometry></collision>
  </link>
</robot>
"""


class TestStripOnePanelCollision(unittest.TestCase):
    def test_only_the_named_collision_mesh_goes(self):
        xml, removed = strip_link_collision_mesh(_URDF, "suction_panel", "p04.stl")
        self.assertEqual(removed, 1)
        root = ET.fromstring(xml)
        panel = next(l for l in root.findall("link") if l.get("name") == "suction_panel")
        meshes = [c.find("geometry/mesh").get("filename") for c in panel.findall("collision")]
        self.assertEqual(meshes, ["package://x/p01.stl", "package://x/p05.stl"])
        self.assertIn("p04.stl", panel.find("visual/geometry/mesh").get("filename"))
        link6 = next(l for l in root.findall("link") if l.get("name") == "elfin_link6")
        self.assertEqual(len(link6.findall("collision")), 1)

    def test_absent_check_requires_the_visual_to_remain(self):
        xml, _ = strip_link_collision_mesh(_URDF, "suction_panel", "p04.stl")
        handle = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False)
        handle.write(xml)
        handle.close()
        try:
            ok, detail = panel_collision_absent(handle.name, "p04.stl")
        finally:
            os.unlink(handle.name)
        self.assertTrue(ok, detail)
        self.assertEqual(detail["suction_collisions_left"], 2)
        self.assertTrue(detail["visual_kept"])


_SDF = """<sdf version="1.9"><model name="S20">
  <link name="elfin_link6">
    <collision name="p01">
      <geometry><mesh><uri>model://luggage_description/meshes/suction_panel/p01.stl</uri></mesh></geometry>
      <surface><contact><ode/></contact></surface>
    </collision>
    <collision name="p04">
      <geometry><mesh><uri>model://luggage_description/meshes/suction_panel/p04.stl</uri></mesh></geometry>
      <surface><contact><ode/></contact></surface>
    </collision>
  </link>
</model></sdf>
"""


class TestP04PairMask(unittest.TestCase):
    def test_only_the_carried_box_misses_p04(self):
        self.assertFalse(masks_collide(P04_MASK, BOX_CARRY_MASK))
        self.assertTrue(masks_collide(P04_MASK, BOX_FULL_MASK))
        self.assertTrue(masks_collide(BOX_CARRY_MASK, BOX_FULL_MASK))

    def test_bitmask_is_written_only_on_the_p04_mesh(self):
        xml, changed = set_collision_bitmask_for_mesh(_SDF, "p04.stl", P04_MASK)
        self.assertEqual(changed, 1)
        root = ET.fromstring(xml)
        found = {}
        for collision in root.iter("collision"):
            uri = collision.find("geometry/mesh/uri").text
            bit = collision.find("surface/contact/collide_bitmask")
            found[uri] = None if bit is None else bit.text
        self.assertEqual(found["model://luggage_description/meshes/suction_panel/p04.stl"], "1")
        self.assertIsNone(found["model://luggage_description/meshes/suction_panel/p01.stl"])


if __name__ == "__main__":
    unittest.main()
