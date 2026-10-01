"""外接球の包囲、自己干渉候補、左右接近時の退避余裕の検証。"""
from pathlib import Path
from itertools import product
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_avoidance_geometry import robot_geometry, mesh_vertices


class geometry_test(unittest.TestCase):
    def test_rotated_box_enclosure(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'box.urdf'
            path.write_text('''<robot name="box"><link name="base_footprint"/>
              <link name="camera_link"><collision><origin xyz="0.1 0.2 0.3" rpy="0 0 0.7"/>
                <geometry><box size="0.04 0.09 0.025"/></geometry></collision></link>
              <joint name="camera_fixed" type="fixed"><parent link="base_footprint"/>
                <child link="camera_link"/></joint></robot>''')
            geometry = robot_geometry(path)
            points = np.asarray(list(product(np.linspace(-1, 1, 11), repeat=3))) * [.02, .045, .0125]
            matrix = np.array([[np.cos(.7), -np.sin(.7), 0], [np.sin(.7), np.cos(.7), 0], [0, 0, 1]])
            points = points @ matrix.T + [.1, .2, .3]
            gaps = np.asarray([np.linalg.norm(points-point, axis=1)-radius
                               for _, point, radius in geometry.spheres])
            self.assertLessEqual(float(gaps.min(axis=0).max()), 1e-10)

    def test_invalid_box_dimensions_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'box.urdf'
            for size in ('0 1 1', '-1 1 1', 'nan 1 1', 'inf 1 1', '1 1', '1 1 1 1'):
                with self.subTest(size=size):
                    path.write_text('<robot name="box"><link name="base_footprint"><collision>'
                                    '<geometry><box size="'+size+'"/></geometry></collision></link></robot>')
                    with self.assertRaisesRegex(ValueError, '直方体寸法'):
                        robot_geometry(path)

    def test_mesh_enclosure_and_retreat(self):
        root = Path(__file__).resolve().parents[2]
        for model in ('topo_dual_arm_max', 'topo_dual_arm_max_long'):
            with self.subTest(model=model):
                path = root/'urdf'/model/'topo_dual_arm_max.urdf'
                geometry = robot_geometry(path)
                home = np.zeros(len(geometry.joint_names))
                self.assertEqual(len(geometry.joint_names), 19)
                self.assertTrue(geometry.has_internal_clearance(geometry.centers(home)))
                # ローカル座標の衝突メッシュを覆う球列の確認
                vertices = mesh_vertices(path.parent/'meshes/L_link4.stl')*.001
                spheres = [(point, radius) for name, point, radius in geometry.spheres if name == 'L_link4']
                gaps = np.array([np.linalg.norm(vertices-point, axis=1)-radius for point, radius in spheres])
                self.assertLessEqual(float(gaps.min(axis=0).max()), 1e-8)
                for sign in (1, -1):
                    positions = home.copy()
                    min_gap = float('inf')
                    for hand_x in np.linspace(.45, .03, 161):
                        hand = np.array([hand_x, sign*.36, .38])
                        elbow = hand+np.array([.35, 0, 0])
                        previous = positions.copy()
                        positions, has_candidate = geometry.choose_step(positions, home, hand, elbow, .045, .12, .035)
                        self.assertTrue(has_candidate)
                        self.assertLessEqual(float(np.max(np.abs(positions-previous))), .035+1e-10)
                        gap, centers, _, _ = geometry.clearance(positions, hand, elbow, .045)
                        self.assertTrue(geometry.has_internal_clearance(centers))
                        min_gap = min(min_gap, gap)
                    self.assertGreater(min_gap, .07)
                    self.assertLess(geometry.clearance(home, hand, elbow, .045)[0], 0)
                    self.assertGreater(float(np.max(np.abs(positions))), .2)


if __name__ == '__main__':
    unittest.main()
