"""描画原本の共通化と、独立した物理設定の保持の検証。"""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location('rebuild_meshes', Path(__file__).resolve().parents[1]/'tools/rebuild_meshes.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class VisualSourceTest(unittest.TestCase):
    def test_sync_and_preserve_physics(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            source = Path(temporary_dir)/'source'
            target = Path(temporary_dir)/'app'
            for folder in (source, target):
                (folder/'meshes').mkdir(parents=True)
            (source/'meshes/cover.stl').write_bytes(b'new geometry')
            (target/'meshes/cover.stl').write_bytes(b'old geometry')
            visual = '<visual><geometry><mesh filename="meshes/cover.stl" scale=".001 .001 .001"/></geometry><material name="blue"/></visual>'
            (source/'robot.urdf').write_text('<robot name="r"><material name="blue"><color rgba="0 0 1 1"/></material><link name="world"/><link name="body">'+visual+'</link></robot>')
            collision = '<collision><geometry><box size="1 2 3"/></geometry></collision>'
            (target/'source.urdf').write_text('<robot name="r"><link name="world"/><link name="body">'+collision+'<inertial><mass value="10"/></inertial></link></robot>')
            module.sync_visual_source(target, 'long', source/'robot.urdf')
            root = ET.parse(target/'source.urdf').getroot()
            self.assertEqual(root.find('./link/visual/material/color').get('rgba'), '0 0 1 1')
            self.assertEqual(root.find('./link/collision/geometry/box').get('size'), '1 2 3')
            self.assertEqual(root.find('./link/inertial/mass').get('value'), '10')
            self.assertEqual((target/'meshes/cover.stl').read_bytes(), b'new geometry')
            previous = (target/'source.urdf').read_bytes()
            module.sync_visual_source(target, 'long', source/'robot.urdf')
            self.assertEqual((target/'source.urdf').read_bytes(), previous)

    def test_reject_different_robot(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            folder = Path(temporary_dir)
            (folder/'source.urdf').write_text('<robot><link name="a"/></robot>')
            (folder/'other.urdf').write_text('<robot><link name="b"/></robot>')
            with self.assertRaisesRegex(ValueError, 'リンク構成'):
                module.sync_visual_source(folder, 'long', folder/'other.urdf')


if __name__ == '__main__':
    unittest.main()
