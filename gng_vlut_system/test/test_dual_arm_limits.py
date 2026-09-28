"""回避ノードの実測上限監視と停止判定の検証。"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from sensor_msgs.msg import JointState
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_avoidance_demo import avoidance_demo


class test_joint_limits(unittest.TestCase):
    def run_state(self, velocities, efforts):
        faults = []
        state = SimpleNamespace(geometry=SimpleNamespace(joint_names=['L_joint1']),
                                last_ros_time=-1, state='running', fail=faults.append,
                                joint_limits={'L_joint1': {'velocity': 3., 'effort': 6.},
                                              'L_gripper_mimic': {'velocity': 1., 'effort': 20.}})
        message = JointState(name=['L_joint1', 'L_gripper_mimic'], position=[0., 0.],
                             velocity=velocities, effort=efforts)
        message.header.stamp.sec = 1
        avoidance_demo.on_joints(state, message)
        return state, faults

    def test_within_limits_keeps_running(self):
        state, faults = self.run_state([2., .5], [5., 10.])
        self.assertFalse(state.has_joint_limit_violation)
        self.assertEqual(faults, [])

    def test_effort_and_mimic_velocity_fault(self):
        for velocities, efforts in (([2., .5], [6.2, 10.]), ([2., 1.1], [5., 10.])):
            with self.subTest(velocities=velocities, efforts=efforts):
                state, faults = self.run_state(velocities, efforts)
                self.assertTrue(state.has_joint_limit_violation)
                self.assertEqual(len(faults), 1)

    def test_missing_or_nonfinite_measurement_fault(self):
        for velocities, efforts in (([], []), ([float('nan'), .5], [5., 10.]), ([2., .5], [])):
            with self.subTest(velocities=velocities, efforts=efforts):
                state, faults = self.run_state(velocities, efforts)
                self.assertTrue(state.has_joint_limit_violation)
                self.assertEqual(len(faults), 1)


if __name__ == '__main__':
    unittest.main()
