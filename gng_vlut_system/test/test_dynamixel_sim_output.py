"""左腕7軸の部分欠測・再送・異常実測による停止判定の検証。"""
import json
import math
from pathlib import Path
import sys
import time
from unittest.mock import Mock, patch

import pytest
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamixel_sim_output import dynamixel_sim_output


@pytest.fixture
def output():
    node = dynamixel_sim_output.__new__(dynamixel_sim_output)
    node.names = ['L_joint'+str(value) for value in range(1, 8)]
    node.motor_names = {str(40+value): name for value, name in enumerate(node.names, 1)}
    node.mapping = SimpleNamespace(entries={name: (int(motor), -1., 0.) for motor, name in node.motor_names.items()})
    node.config = {'max_state_age_sec': .3, 'max_stop_velocity_th': .01, 'min_stop_duration_sec': .3}
    node.measured, node.velocity, node.stamps = {}, {}, {}
    node.low_since = None
    return node


def test_sim_target_uses_reference_without_deprecated_fields(output):
    message = JointTrajectoryControllerState()
    message.header.stamp.sec = 1
    message.joint_names = output.names
    message.reference.positions = [.1]*len(output.names)
    message.feedback.positions = [.9]*len(output.names)
    output.last_sim_stamp = -1
    output.sim_target, output.sim_sec = {}, -math.inf
    output.on_sim_target(message)
    assert output.sim_target == dict.fromkeys(output.names, .1)
    assert math.isfinite(output.sim_sec)
    assert not message.desired.positions and not message.actual.positions


def test_sim_target_without_reference_never_uses_feedback(output):
    message = JointTrajectoryControllerState()
    message.header.stamp.sec = 1
    message.joint_names = output.names
    message.feedback.positions = [.9]*len(output.names)
    output.last_sim_stamp = -1
    output.sim_target, output.sim_sec = {}, -math.inf
    output.on_sim_target(message)
    assert output.sim_target == {} and output.sim_sec == -math.inf


@pytest.mark.parametrize('allow_output', [False, True])
def test_torque_off_without_feedback_blocks_all_position_goals(output, allow_output):
    output.ids = list(range(41, 48))
    output.torque_pub = Mock() if allow_output else None
    output.goal_pub = Mock()
    output.is_torque_off_latched = False
    output.target = output.commanded = {'L_joint7': 1.}
    response = output.on_torque_off(None, SimpleNamespace())
    assert response.success is allow_output
    assert output.mode == 'torque_off' and output.is_stop_latched
    output.send_goal({'L_joint7': 2.})
    output.stop('通常保持停止')
    assert output.mode == 'torque_off' and not output.target
    output.goal_pub.publish.assert_not_called()
    if allow_output:
        message = output.torque_pub.publish.call_args.args[0]
        assert list(message.id_list) == list(range(41, 48))
        assert list(message.torque) == [False]*7
        assert not message.error and not message.mode and not message.ping


def test_reset_needs_torque_off_report_even_when_stationary(output):
    output.mode = 'torque_off'
    output.is_torque_off_latched = True
    output.is_stop_latched = True
    output.is_stationary = lambda: True
    output.has_torque_off_report = lambda: False
    response = output.on_reset(None, SimpleNamespace())
    assert not response.success and output.is_torque_off_latched


def test_enable_without_explicit_current_limit_has_no_output(output):
    output.config.update(allow_hardware_output=True, max_current_ma=0.)
    output.mode = 'off'
    output.is_stop_latched = False
    output.goal_pub, output.torque_pub = Mock(), Mock()
    response = output.on_enable(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and 'max_current_ma' in response.message
    output.goal_pub.publish.assert_not_called()
    output.torque_pub.publish.assert_not_called()


def sample(ids, stamp, velocity=0.):
    message = JointState(name=[str(value) for value in ids], position=[0.]*len(ids), velocity=[velocity]*len(ids))
    message.header.frame_id = 'dynamixel_motor'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    return message


def test_missing_one_left_arm_id_never_confirms_stop(output):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 47), now))
        assert not output.has_fresh_state()
        assert not output.is_stationary()
        output.on_measured(sample([47], now))
        assert output.has_fresh_state()
        assert output.low_since is not None


@pytest.mark.parametrize('kind', ['stale', 'future', 'replayed', 'wrong_frame', 'nan_velocity', 'duplicate_id'])
def test_invalid_feedback_does_not_refresh_left_arm(output, kind):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 48), now-100_000_000))
        stamps = dict(output.stamps)
        message = sample(range(41, 48), now)
        if kind in ('stale', 'future', 'replayed'):
            message = sample(range(41, 48), now+{'stale': -400_000_000, 'future': 1_000_000, 'replayed': -100_000_000}[kind])
        elif kind == 'wrong_frame':
            message.header.frame_id = 'base_link'
        elif kind == 'nan_velocity':
            message.velocity = [math.nan]*7
        else:
            message.name = ['47']*7
        output.on_measured(message)
        assert output.stamps == stamps
    with patch('dynamixel_sim_output.time.time_ns', return_value=now+300_000_000):
        assert not output.has_fresh_state()
        assert not output.is_stationary()


def test_one_moving_joint_cancels_stationary_duration(output):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 48), now-1_000))
        output.on_measured(sample([44], now, velocity=.1))
        assert output.low_since is None
        assert not output.is_stationary()


def test_zero_velocity_register_with_position_motion_is_not_stopped(output):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 48), now-20_000_000))
        message = sample([47], now)
        message.position = [math.radians(.08789)]
        output.on_measured(message)
        assert output.velocity['L_joint7'] == 0.
        assert not output.is_stationary()


@pytest.fixture
def running_output(output):
    """ROS初期化・実機通信を伴わない、出力周期の状態と送信先。"""
    output.names, output.ids = ['L_joint7'], [47]
    output.mapping = SimpleNamespace(entries={'L_joint7': (47, 1., 0.)},
                                     convert=Mock(return_value=([47], [0.])))
    output.model = SimpleNamespace(step=Mock(return_value={'L_joint7': .01}))
    output.config.update(allow_hardware_output=True, driver_namespace='/dynamixel',
        publish_hz=20., max_prepare_sec=3., max_start_dev_th=.02,
        max_status_age_sec=1.5, max_current_ma=10., max_velocity=.1,
        max_acceleration=1., max_follow_dev_th=.1)
    output.measured = output.target = {'L_joint7': 0.}
    output.commanded = {'L_joint7': 0.}
    output.velocity, output.stamps = {'L_joint7': 0.}, {'L_joint7': 20}
    output.stop_feedback_stamps = {'L_joint7': 10}
    output.mode, output.detail = 'hold', '保持'
    output.is_torque_off_latched = output.is_stop_latched = False
    output.has_sent_torque_on = output.has_pending_hold = False
    output.has_owned_output = True
    output.last_tick, output.prepare_sec = 9.98, 9.
    output.goal_sec = output.status_sec = 9.5
    output.goal_echo, output.motor_status = {47: (0., 1., 2., 8.)}, {47: (True, 0, True, 'cur_position')}
    output.has_fresh_state = Mock(return_value=True)
    output.is_stationary = Mock(return_value=False)
    output.has_torque_off_report = Mock(return_value=False)
    output.check_ready, output.check_target = Mock(), Mock()
    output.count_publishers = Mock(return_value=1)
    output.goal_pub, output.torque_pub, output.status_pub = Mock(), Mock(), Mock()
    output.send_goal = Mock()
    with patch('dynamixel_sim_output.time.monotonic', return_value=10.):
        yield output


@pytest.mark.parametrize('mode', ['off', 'preparing', 'hold', 'jog', 'follow', 'stopped'])
def test_torque_off_latch_has_priority_over_every_mode(running_output, mode):
    node = running_output
    node.mode, node.is_torque_off_latched = mode, True
    node.tick()
    message = node.torque_pub.publish.call_args.args[0]
    assert list(message.torque) == [False]
    node.send_goal.assert_not_called()
    node.check_ready.assert_not_called()
    node.model.step.assert_not_called()
    node.status_pub.publish.assert_called_once()


@pytest.mark.parametrize('goal_sec', [8.9, 9., 9.5])
def test_preparing_needs_new_goal_echo_before_torque_on(running_output, goal_sec):
    node = running_output
    node.mode, node.goal_sec = 'preparing', goal_sec
    events = Mock()
    events.attach_mock(node.torque_pub.publish, 'torque')
    events.attach_mock(node.send_goal, 'goal')
    node.tick()
    assert node.mode == 'preparing'
    assert node.has_sent_torque_on is (goal_sec > node.prepare_sec)
    assert [item[0] for item in events.mock_calls] == (
        ['torque', 'goal'] if goal_sec > node.prepare_sec else ['goal'])


@pytest.mark.parametrize('goal_echo', [None, {}, {48: (0., 1., 2., 8.)},
    {47: (.1, 1., 2., 8.)}, {47: (0., 0., 2., 8.)}, {47: (0., 6., 2., 8.)},
    {47: (0., 1., 0., 8.)}, {47: (0., 1., 58., 8.)},
    {47: (0., 1., 2., 0.)}, {47: (0., 1., 2., 10.01)}, {47: (0., 1., 2., math.nan)}])
def test_prepare_invalid_echo_never_enables_torque(running_output, goal_echo):
    node = running_output
    node.mode, node.goal_echo = 'preparing', goal_echo
    node.tick()
    assert node.mode == 'preparing' and not node.has_sent_torque_on
    node.torque_pub.publish.assert_not_called()
    node.send_goal.assert_called_once_with(node.target)


@pytest.mark.parametrize('status_sec, has_torque', [(8.9, True), (9., True), (9.5, False), (9.5, True)])
def test_preparing_needs_new_torque_status_after_torque_request(running_output, status_sec, has_torque):
    node = running_output
    node.mode, node.has_sent_torque_on = 'preparing', True
    node.status_sec, node.motor_status = status_sec, {47: (has_torque, 0, True, 'cur_position')}
    node.tick()
    assert node.mode == ('hold' if status_sec > node.prepare_sec and has_torque else 'preparing')
    node.torque_pub.publish.assert_not_called()
    node.send_goal.assert_called_once_with(node.target)


@pytest.mark.parametrize('fault', ['timeout', 'pose_change', 'publisher_conflict', 'not_ready'])
def test_prepare_failure_only_sends_measured_stop_hold(running_output, fault):
    node = running_output
    node.mode = 'preparing'
    node.target = {'L_joint7': .015}
    if fault == 'timeout':
        node.prepare_sec = 6.
    elif fault == 'pose_change':
        node.target = {'L_joint7': .03}
    elif fault == 'publisher_conflict':
        node.count_publishers.return_value = 2
    else:
        node.check_ready.side_effect = ValueError('実測失効')
    node.tick()
    assert node.is_stop_latched and node.mode == 'stopped'
    node.send_goal.assert_called_once_with(node.measured)
    node.torque_pub.publish.assert_not_called()
    node.model.step.assert_not_called()
    node.status_pub.publish.assert_called_once()


@pytest.mark.parametrize('fault', ['stale', 'missing', 'missing_motor', 'zero', 'over_limit'])
def test_current_echo_fault_latches_torque_off_without_position_goal(running_output, fault):
    node = running_output
    if fault == 'stale':
        node.goal_sec = 8.
    elif fault == 'missing':
        node.goal_echo = None
    elif fault == 'missing_motor':
        node.goal_echo = {48: (0., 1., 2., 8.)}
    else:
        node.goal_echo = {47: (0., 1., 2., 0. if fault == 'zero' else 10.01)}
    node.tick()
    assert node.is_torque_off_latched and node.mode == 'torque_off'
    assert node.target == {} and node.commanded == {}
    node.torque_pub.publish.assert_called_once()
    node.send_goal.assert_not_called()
    node.model.step.assert_not_called()


def test_follow_validates_before_step_and_send(running_output):
    node = running_output
    node.mode, node.sim_target = 'follow', {'L_joint7': .03}
    node.check_sim = Mock()
    node.last_tick = 8.
    events = Mock()
    for name, method in [('ready', node.check_ready), ('publishers', node.count_publishers),
                         ('sim', node.check_sim), ('target', node.check_target),
                         ('step', node.model.step), ('goal', node.send_goal),
                         ('status', node.status_pub.publish)]:
        events.attach_mock(method, name)
    node.tick()
    assert [item[0] for item in events.mock_calls] == ['ready', 'publishers', 'sim', 'target', 'step', 'goal', 'status']
    node.model.step.assert_called_once_with({'L_joint7': 0.}, {'L_joint7': .03}, .1, .1)
    node.send_goal.assert_called_once_with({'L_joint7': .01})


@pytest.mark.parametrize('fault', ['sim', 'target', 'deviation', 'torque_off'])
def test_follow_rejection_never_sends_old_motion_target(running_output, fault):
    node = running_output
    node.mode, node.sim_target = 'follow', {'L_joint7': .03}
    node.check_sim = Mock()
    if fault == 'sim':
        node.check_sim.side_effect = ValueError('Gazebo目標の失効')
    elif fault == 'target':
        node.check_target.side_effect = ValueError('可動域外')
    elif fault == 'deviation':
        node.commanded = {'L_joint7': .2}
    else:
        node.motor_status = {47: (False, 0, True, 'cur_position')}
    node.tick()
    assert node.is_stop_latched and node.mode == 'stopped'
    node.send_goal.assert_called_once_with(node.measured)
    node.model.step.assert_not_called()


@pytest.mark.parametrize('is_fresh, stamp, has_target, has_pending', [
    (False, 20, True, True), (True, 10, True, True),
    (True, 20, False, True), (True, 20, True, True), (True, 20, True, False)])
def test_stopped_output_requires_new_feedback_before_hold(running_output, is_fresh, stamp, has_target, has_pending):
    node = running_output
    node.mode, node.is_stop_latched = 'stopped', True
    node.target = {'L_joint7': .02} if has_target else {}
    node.has_pending_hold, node.stamps = has_pending, {'L_joint7': stamp}
    node.has_fresh_state.return_value = is_fresh
    node.tick()
    if is_fresh and stamp > 10 and has_target:
        node.send_goal.assert_called_once_with(node.measured if has_pending else {'L_joint7': .02})
        assert not node.has_pending_hold
    else:
        node.send_goal.assert_not_called()
        assert node.has_pending_hold is has_pending
    node.model.step.assert_not_called()


def test_jog_reports_hold_only_after_sent_target_and_measured_stop(running_output):
    node = running_output
    node.mode, node.target = 'jog', {'L_joint7': .001}
    node.is_stationary.return_value = True
    node.tick()
    assert node.mode == 'hold'
    node.send_goal.assert_called_once_with({'L_joint7': .01})
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status['mode'] == 'hold' and status['is_stationary']
