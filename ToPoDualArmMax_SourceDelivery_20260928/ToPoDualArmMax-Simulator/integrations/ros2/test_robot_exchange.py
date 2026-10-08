"""状態ツリー・軌道形式の境界検証。ROS不要。"""
import copy
import unittest
from types import SimpleNamespace as record
from robot_exchange import validate_state, trajectory_payload


class RobotExchangeTest(unittest.TestCase):
    def setUp(self):
        self.state = dict(robot_model='standard', robot_pose={'joint': 0.}, transforms=[dict(parent='world', child='base_footprint', translation=[1., 2., 3.], rotation=[0., 0., 0., 1.])])
        self.command = record(header=record(stamp=record(sec=0, nanosec=0)), joint_names=['joint'], points=[record(positions=[.1], velocities=[], accelerations=[], effort=[], time_from_start=record(sec=1, nanosec=0))])

    def test_state(self):
        self.assertEqual(validate_state(self.state), self.state)

    def test_motion_fields(self):
        for field in ('joint_velocity', 'joint_effort'):
            state = dict(self.state, **{field: {'joint': .3}})
            self.assertEqual(validate_state(state)[field], {'joint': .3})
            for invalid in ({}, {'other': .3}, {'joint': float('nan')}, {'joint': True}):
                with self.subTest(field=field, invalid=invalid), self.assertRaises(ValueError):
                    validate_state(dict(self.state, **{field: invalid}))

    def test_bad_transform(self):
        for change in [dict(parent='base_footprint'), dict(rotation=[0., 0., 0., 0.]), dict(translation=[float('nan'), 0., 0.]), dict(child='world')]:
            state = copy.deepcopy(self.state)
            state['transforms'][0].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_state(state)

    def test_duplicate_and_disconnected(self):
        for parent, child in [('world', 'base_footprint'), ('missing', 'tip')]:
            state = copy.deepcopy(self.state)
            state['transforms'].append(dict(state['transforms'][0], parent=parent, child=child))
            with self.assertRaises(ValueError):
                validate_state(state)

    def test_trajectory(self):
        self.assertEqual(trajectory_payload(self.command)['points'][0]['time_sec'], 1.)

    def test_invalid_trajectory(self):
        for attribute, value in [('positions', [float('nan')]), ('positions', []), ('velocities', [1.]), ('time_from_start', record(sec=0, nanosec=0)), ('time_from_start', record(sec=121, nanosec=0))]:
            command = copy.deepcopy(self.command)
            setattr(command.points[0], attribute, value)
            with self.subTest(attribute=attribute), self.assertRaises(ValueError):
                trajectory_payload(command)

    def test_absolute_time(self):
        self.command.header.stamp.sec = 1
        with self.assertRaises(ValueError):
            trajectory_payload(self.command)


if __name__ == '__main__':
    unittest.main()
