"""実URDFに対するデモ姿勢の検査。"""
import copy
import importlib.util
from pathlib import Path
import unittest
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('dual_arm_gazebo_demo', WORKSPACE/'gng_vlut_system/scripts/dual_arm_gazebo_demo.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MotionConfigTest(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((WORKSPACE/'gng_vlut_system/config/dual_arm_gazebo_demo.yaml').read_text())['dual_arm_gazebo_demo']
        self.urdf = WORKSPACE/'urdf/topo_dual_arm_max/topo_dual_arm_max.urdf'

    def test_both_models(self):
        for model in ['topo_dual_arm_max','topo_dual_arm_max_long']:
            names, poses = module.load_motion(WORKSPACE/'urdf'/model/'topo_dual_arm_max.urdf', self.config)
            self.assertEqual(len(names), 19)
            self.assertNotIn('L_gripper_mimic', names)
            self.assertEqual(len(poses), 8)
            for _, values in poses:
                self.assertEqual(len(values), len(names))
                for name in ['waist_joint','neck_pan_joint','neck_tilt_joint']:
                    self.assertEqual(values[names.index(name)], 0.0)

    def test_reject_invalid_targets(self):
        for joints in [{'L_joint4':100.0}, {'unknown_joint':0.0}, {'L_joint1':float('nan')}]:
            config = copy.deepcopy(self.config)
            config['poses'] = [{'name':'invalid','joints':joints}]
            with self.assertRaises(ValueError):
                module.load_motion(self.urdf, config)

    def test_reject_empty_motion(self):
        self.config['poses'] = []
        with self.assertRaises(ValueError):
            module.load_motion(self.urdf, self.config)


if __name__ == '__main__':
    unittest.main()
