"""回避ノードの実測上限監視と停止判定の検証。"""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
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


class test_software_stop(unittest.TestCase):
    def test_stop_latches_demo_without_motion(self):
        state = SimpleNamespace(has_safety_state=False, is_stop_latched=False,
                                enable_auto_start=True, state='running', phase='approaching', error='')
        avoidance_demo.on_safety_stop(state, Bool(data=True))
        self.assertTrue(state.has_safety_state)
        self.assertTrue(state.is_stop_latched)
        self.assertFalse(state.enable_auto_start)
        self.assertEqual(state.state, 'stopped')
        self.assertEqual(state.phase, 'software_stop')
        avoidance_demo.on_safety_stop(state, Bool(data=False))
        self.assertFalse(state.is_stop_latched)
        self.assertEqual(state.state, 'stopped')
        self.assertFalse(state.enable_auto_start)

    def test_start_rejected_until_safety_ready(self):
        for has_safety_state, is_stop_latched in ((False, False), (True, True)):
            state = SimpleNamespace(has_safety_state=has_safety_state, is_stop_latched=is_stop_latched)
            result = avoidance_demo.on_start(state, None, Trigger.Response())
            self.assertFalse(result.success)

    def test_stop_does_not_clear_latch(self):
        state = SimpleNamespace(is_stop_latched=True, state='stopped')
        result = avoidance_demo.on_stop(state, None, Trigger.Response())
        self.assertTrue(result.success)
        self.assertTrue(state.is_stop_latched)
        self.assertEqual(state.state, 'stopped')

    def test_latched_target_has_no_output(self):
        state = SimpleNamespace(is_stop_latched=True)
        avoidance_demo.publish_target(state, [1.0])

if __name__ == '__main__':
    unittest.main()
