"""回避ノードの実測上限監視と停止判定の検証。"""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import json
import unittest

import numpy as np

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
    def test_stop_geometry_persists_after_cloud_moves_away(self):
        centers = np.array([[.2, .1, .3]])
        closest = np.array([.21, .11, .31])
        state = SimpleNamespace(
            state='running', phase='avoiding', error='', positions=np.array([.2]), home=np.array([.1]),
            config={'enable_live_obstacles': True, 'min_clearance_th': .035, 'sides': ['left']},
            geometry=SimpleNamespace(spheres=[('L_finger_left', None, .02)], radii=[.02],
                                     root_link='base_link', joint_names=['L_joint1'], has_internal_clearance=lambda _, min_clearance_th=.005: True),
            min_observed_clearance=float('inf'), min_home_clearance=float('inf'), max_excursion=0.,
            stop_clearance=None, trails={}, last_visual=None, side_idx=0,
            run_generation=1, run_start_stamp_sec=1., is_stop_latched=False,
            joint_time=0., obstacle_time=0., is_fresh=lambda: True, update_obstacle=Mock(),
            observe_clearance=Mock(return_value=(.0336, centers, 0, closest)),
            publish_markers=Mock(), status=Mock(), hold=Mock(), get_logger=Mock())
        state.fail = lambda error: avoidance_demo.fail(state, error)
        avoidance_demo.tick(state)
        self.assertEqual(state.state, 'fault')
        self.assertIn('L_finger_left 33.6 mm', state.error)
        first = json.loads(state.status.publish.call_args.args[0].data)
        self.assertEqual(first['stop_clearance']['obstacle_point'], closest.tolist())
        self.assertEqual(first['stop_clearance']['joint_positions'], {'L_joint1': .2})
        state.observe_clearance.return_value = (.1, centers, 0, np.ones(3))
        avoidance_demo.tick(state)
        second = json.loads(state.status.publish.call_args.args[0].data)
        self.assertEqual(second['clearance_m'], .1)
        self.assertEqual(second['stop_clearance'], first['stop_clearance'])

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

    def test_start_distance_rejection_includes_measured_clearance(self):
        state = SimpleNamespace(has_safety_state=True, is_stop_latched=False,
            state='idle', has_joint_limit_violation=False, is_fresh=lambda: True,
            geometry=SimpleNamespace(centers=lambda _: [], has_internal_clearance=lambda _, min_clearance_th=.005: True,
                spheres=[('L_gripper_base', None, .04)], radii=[.04], root_link='base_link', joint_names=['L_joint1']),
            positions=np.array([-.785]),
            observe_clearance=lambda: (.0315, np.array([[.25, .08, .24]]), 0, np.array([.25, .03, .21])),
            config={'min_clearance_th': .035})
        result = avoidance_demo.on_start(state, None, Trigger.Response())
        self.assertFalse(result.success)
        self.assertIn('31.5 mm', result.message)
        self.assertIn('35.0 mm超', result.message)
        self.assertIn('部位=L_gripper_base', result.message)
        self.assertEqual(state.stop_clearance['event'], 'start_rejected')
        self.assertEqual(state.stop_clearance['obstacle_point'], [.25, .03, .21])
        self.assertEqual(state.stop_clearance['robot_radius_m'], .04)
        self.assertEqual(state.state, 'idle')

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
