"""補間メッセージの互換性と関節速度制限の検証。"""
import math
import unittest

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


if __name__ == '__main__':
    unittest.main()
