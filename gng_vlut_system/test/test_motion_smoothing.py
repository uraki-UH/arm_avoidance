"""補間・速度制限と任意の関節運動状態観測の検証。"""
import math
import os
import time
import unittest
from unittest.mock import patch

from joint_motion_state import joint_motion_tracker

from motion_smoothing import make_rest_to_rest_trajectory, quintic_duration, quintic_max_step


class motion_smoothing_test(unittest.TestCase):
    def test_synchronized_joint_velocity_bounds(self):
        current, target = [0.1, -0.2, 0.3], [1.1, -0.7, 0.3]
        duration = quintic_duration(current, target, 0.1, 0.15)
        for idx in range(101):
            ratio = idx / 100
            for start, end in zip(current, target):
                velocity = (end-start) * 30 * ratio**2 * (1-ratio)**2 / duration
                self.assertLessEqual(abs(velocity), 0.15 + 1e-12)
        self.assertEqual(quintic_duration(current, current, 4.0, 0.15), 4.0)
        self.assertEqual(quintic_duration(current, target, 100.0, 0.15), 100.0)

    def test_avoidance_step_matches_interval(self):
        duration = 0.15
        step = quintic_max_step(0.45, duration)
        self.assertAlmostEqual(quintic_duration([0.0], [step], 0.001, 0.45), duration)

    def test_ros_message_contract(self):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        for duration in (0.15, 0.3, 4.0, 12.5):
            current, target = [0.0, -0.3], [0.2, 0.1]
            expected = JointTrajectory()
            expected.joint_names = ['left', 'right']
            first = JointTrajectoryPoint(positions=current, velocities=[0.0]*2, accelerations=[0.0]*2)
            final = JointTrajectoryPoint(positions=target, velocities=[0.0]*2, accelerations=[0.0]*2)
            final.time_from_start.sec = int(duration)
            final.time_from_start.nanosec = int((duration-int(duration))*1e9)
            expected.points = [first, final]
            actual = make_rest_to_rest_trajectory(expected.joint_names, current, target, duration)
            self.assertEqual(actual, expected)
            actual.points[0].positions[0] = 9.0
            self.assertEqual(current[0], 0.0)

    def test_reject_invalid_inputs(self):
        for current, target, duration, velocity in (([], [], 1.0, 1.0), ([0.0], [], 1.0, 1.0),
                ([0.0], [math.nan], 1.0, 1.0), ([0.0], [1.0], 0.0, 1.0), ([0.0], [1.0], 1.0, 0.0)):
            with self.assertRaises(ValueError):
                quintic_duration(current, target, duration, velocity)
        with self.assertRaises(ValueError):
            make_rest_to_rest_trajectory(['left'], [0.0], [1.0], 0.0)
        with self.assertRaises(ValueError):
            make_rest_to_rest_trajectory(['left'], [0.0], [], 1.0)


class joint_motion_state_test(unittest.TestCase):
    def test_cubic_nonuniform_samples_at_current_time(self):
        tracker = joint_motion_tracker()
        for stamp_sec in (0.0, 0.08, 0.21, 0.35):
            result = tracker.update(stamp_sec, ['joint'], [2 + 3 * stamp_sec + 4 * stamp_sec**2 + 5 * stamp_sec**3])
        state = result['joint']
        self.assertAlmostEqual(state.velocity, 3 + 8 * stamp_sec + 15 * stamp_sec**2)
        self.assertAlmostEqual(state.acceleration, 8 + 30 * stamp_sec)
        self.assertAlmostEqual(state.jerk, 30.0)
        self.assertEqual(state.estimated_fields, ('velocity', 'acceleration', 'jerk'))
        self.assertEqual(state.stamp_sec, stamp_sec)

    def test_observed_velocity_and_acceleration_take_precedence(self):
        tracker = joint_motion_tracker()
        for stamp_sec in (0.0, 0.1, 0.3):
            state = tracker.update(stamp_sec, ['joint'], [1000.0], [2 + 3 * stamp_sec + 4 * stamp_sec**2])['joint']
        self.assertAlmostEqual(state.velocity, 2 + 3 * stamp_sec + 4 * stamp_sec**2)
        self.assertAlmostEqual(state.acceleration, 3 + 8 * stamp_sec)
        self.assertAlmostEqual(state.jerk, 8.0)
        self.assertEqual(state.estimated_fields, ('acceleration', 'jerk'))
        state = tracker.update(0.4, ['joint'], [999.0], [0.0], [2.0], [7.0])['joint']
        self.assertEqual((state.acceleration, state.jerk, state.estimated_fields), (2.0, 7.0, ()))
        state = tracker.update(0.5, ['joint'], accelerations=[2.8])['joint']
        self.assertAlmostEqual(state.jerk, 8.0)
        self.assertIsNone(state.position)
        self.assertIsNone(state.velocity)

    def test_warmup_missing_is_distinct_from_zero(self):
        tracker = joint_motion_tracker()
        first = tracker.update(0.0, ['joint'], [0.0])['joint']
        self.assertEqual((first.velocity, first.acceleration, first.jerk), (None, None, None))
        second = tracker.update(0.1, ['joint'], [0.0])['joint']
        self.assertEqual(second.velocity, 0.0)
        self.assertIsNone(second.acceleration)
        third = tracker.update(0.2, ['joint'], [0.0])['joint']
        self.assertEqual(third.acceleration, 0.0)
        self.assertIsNone(third.jerk)
        fourth = tracker.update(0.3, ['joint'], [0.0])['joint']
        self.assertEqual(fourth.jerk, 0.0)

    def test_derivative_order_and_disabled_calculation(self):
        for order in range(4):
            tracker = joint_motion_tracker(max_derivative_order=order)
            for stamp_sec in (0.0, 0.1, 0.2, 0.3):
                state = tracker.update(stamp_sec, ['joint'], [stamp_sec**3])['joint']
            for derivative, name in enumerate(('velocity', 'acceleration', 'jerk'), 1):
                self.assertEqual(getattr(state, name) is not None, derivative <= order)
        tracker = joint_motion_tracker(max_derivative_order=0)
        with patch.object(tracker, '_differentiate', side_effect=AssertionError('微分の呼出しは禁止')):
            tracker.update(0.0, ['joint'], [0.0])
            state = tracker.update(0.1, ['joint'], [1.0], [2.0], [3.0], [4.0])['joint']
        self.assertEqual((state.velocity, state.acceleration, state.jerk), (2.0, 3.0, 4.0))
        self.assertEqual(tracker._history, {})

    def test_independent_partial_updates_and_joint_order(self):
        tracker = joint_motion_tracker()
        tracker.update(0.0, ['left', 'right'], [1.0, 10.0])
        left = tracker.update(0.1, ['left'], [1.2])['left']
        self.assertAlmostEqual(left.velocity, 2.0)
        states = tracker.update(0.2, ['right', 'left'], [12.0, 1.4])
        self.assertEqual(list(states), ['right', 'left'])
        self.assertAlmostEqual(states['right'].velocity, 10.0)
        self.assertAlmostEqual(states['left'].velocity, 2.0)

    def test_duplicate_short_interval_rewind_and_gap(self):
        tracker = joint_motion_tracker()
        tracker.update(0.0, ['joint'], [0.0])
        self.assertEqual(tracker.update(0.0, ['joint'], [100.0]), {})
        self.assertEqual(tracker.update(0.00001, ['joint'], [100.0]), {})
        state = tracker.update(0.1, ['joint'], [0.1])['joint']
        self.assertAlmostEqual(state.velocity, 1.0)
        for stamp_sec in (0.05, 1.0):
            state = tracker.update(stamp_sec, ['joint'], [100.0])['joint']
            self.assertIsNone(state.velocity)
        tracker.reset()
        self.assertIsNone(tracker.update(1.1, ['joint'], [100.1])['joint'].velocity)

    def test_nonfinite_samples_clear_only_affected_history(self):
        tracker = joint_motion_tracker()
        tracker.update(0.0, ['bad', 'good'], [0.0, 0.0])
        state = tracker.update(0.1, ['bad', 'good'], [math.nan, 0.1])
        self.assertIsNone(state['bad'].position)
        self.assertIsNone(state['bad'].velocity)
        self.assertAlmostEqual(state['good'].velocity, 1.0)
        state = tracker.update(0.2, ['bad'], [0.2], [math.inf])['bad']
        self.assertIsNone(state.velocity)
        state = tracker.update(0.3, ['bad'], [0.3])['bad']
        self.assertAlmostEqual(state.velocity, 1.0)

    def test_invalid_input_does_not_mutate_history(self):
        tracker = joint_motion_tracker()
        tracker.update(0.0, ['joint'], [0.0])
        for names, positions in ((['joint', 'joint'], [1.0, 2.0]), (['joint'], [1.0, 2.0]), ([''], [1.0])):
            with self.assertRaises(ValueError):
                tracker.update(0.1, names, positions)
        for stamp_sec in (-1.0, math.nan, math.inf):
            with self.assertRaises(ValueError):
                tracker.update(stamp_sec, ['joint'], [1.0])
        self.assertAlmostEqual(tracker.update(0.1, ['joint'], [0.1])['joint'].velocity, 1.0)
        for order in (-1, 4, True, 1.5):
            with self.assertRaises(ValueError):
                joint_motion_tracker(max_derivative_order=order)
        for min_period, max_gap in ((0.0, 1.0), (0.1, 0.01), (math.nan, 1.0), (0.1, math.inf)):
            with self.assertRaises(ValueError):
                joint_motion_tracker(min_sample_period_sec=min_period, max_sample_gap_sec=max_gap)

    def test_continuous_joint_unwrap_preserves_observed_position(self):
        tracker = joint_motion_tracker(continuous_joint_names=['continuous'])
        tracker.update(0.0, ['continuous', 'bounded'], [math.pi - 0.1] * 2)
        states = tracker.update(0.1, ['continuous', 'bounded'], [-math.pi + 0.1] * 2)
        self.assertAlmostEqual(states['continuous'].velocity, 2.0)
        self.assertEqual(states['continuous'].position, -math.pi + 0.1)
        self.assertLess(states['bounded'].velocity, -60.0)

    def test_epoch_time_and_velocity_source_change(self):
        tracker = joint_motion_tracker()
        for period_sec in (0.0, 0.125, 0.25, 0.375):
            state = tracker.update(1800000000.0 + period_sec, ['joint'], [period_sec**3])['joint']
        self.assertAlmostEqual(state.jerk, 6.0)
        state = tracker.update(1800000000.5, ['joint'], [0.5**3], [100.0])['joint']
        self.assertEqual(state.velocity, 100.0)
        self.assertIsNone(state.acceleration)
        self.assertIsNone(state.jerk)


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '89', '通信試験は隔離ROS_DOMAIN_ID=89専用')
class joint_motion_observer_test(unittest.TestCase):
    def test_standard_messages_and_no_command_publisher(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from control_msgs.msg import DynamicJointState
        from sensor_msgs.msg import JointState
        from joint_motion_observer import joint_motion_observer

        context = Context()
        rclpy.init(context=context)
        observer = peer = executor = None
        try:
            observer = joint_motion_observer(context=context, namespace='sim_joint_motion_check')
            peer = Node('joint_motion_check', context=context, namespace='sim_joint_motion_check')
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(observer)
            executor.add_node(peer)
            received = []
            publisher = peer.create_publisher(JointState, 'joint_states', qos_profile_sensor_data)
            subscription = peer.create_subscription(DynamicJointState, 'joint_motion_states', received.append, 10)

            def wait_for(predicate):
                deadline = time.monotonic() + 5.0
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.01)
                self.assertTrue(predicate())

            wait_for(lambda: publisher.get_subscription_count() == 1 and peer.count_publishers('joint_motion_states') == 1)
            for idx in range(4):
                message = JointState()
                message.header.stamp.nanosec = idx * 100000000
                message.name, message.position = ['joint'], [(idx * 0.1)**3]
                publisher.publish(message)
                wait_for(lambda: len(received) == idx + 1)
            first = dict(zip(received[0].interface_values[0].interface_names, received[0].interface_values[0].values))
            self.assertTrue(math.isnan(first['jerk']))
            last = dict(zip(received[-1].interface_values[0].interface_names, received[-1].interface_values[0].values))
            self.assertAlmostEqual(last['velocity'], 0.27)
            self.assertAlmostEqual(last['acceleration'], 1.8)
            self.assertAlmostEqual(last['jerk'], 6.0)
            self.assertEqual(last['is_velocity_estimated'], 1.0)
            self.assertEqual(received[-1].header, message.header)
            message.header.stamp.nanosec = 400000000
            message.velocity = [7.0]
            publisher.publish(message)
            wait_for(lambda: len(received) == 5)
            observed = dict(zip(received[-1].interface_values[0].interface_names, received[-1].interface_values[0].values))
            self.assertEqual(observed['velocity'], 7.0)
            self.assertEqual(observed['is_velocity_estimated'], 0.0)
            self.assertTrue(math.isnan(observed['acceleration']))
            message.header.frame_id = 'changed_frame'
            message.header.stamp.nanosec = 500000000
            message.velocity = []
            publisher.publish(message)
            wait_for(lambda: len(received) == 6)
            self.assertTrue(math.isnan(received[-1].interface_values[0].values[1]))
            topics = dict(peer.get_publisher_names_and_types_by_node('joint_motion_observer', '/sim_joint_motion_check'))
            self.assertEqual(set(topics) - {'/rosout', '/parameter_events'}, {'/sim_joint_motion_check/joint_motion_states'})
        finally:
            if executor is not None:
                executor.shutdown()
            if peer is not None:
                peer.destroy_node()
            if observer is not None:
                observer.destroy_node()
            context.shutdown()


if __name__ == '__main__':
    unittest.main()
