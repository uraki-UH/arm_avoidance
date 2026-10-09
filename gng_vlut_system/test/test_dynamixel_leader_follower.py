"""リーダーID換算・開始姿勢・欠測停止・指令先分離の検証。"""
import math
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import time

import pytest
import yaml

from test_dynamixel_sim_output import output, running_output, sample
from joint_command_model import joint_command_model, dynamixel_mapping, gripper_input_mapping


@pytest.fixture
def leader_output(running_output):
    node = running_output
    node.config.update(target_source='leader', allow_leader_follow=True, enable_relative_follow=True,
                       max_command_age_sec=.3)
    node.leader_mapping = SimpleNamespace(entries={'L_joint7': (17, -1., 0.)})
    node.leader_ids, node.leader_motor_names = [17], {'17': 'L_joint7'}
    node.leader_input_names = node.names
    node.leader_gripper_mapping = {}
    node.leader_measured, node.leader_velocity, node.leader_stamps = {}, {}, {}
    node.leader_anchor, node.follower_anchor = {}, {}
    node.leader_pub = Mock()
    node.is_stationary.return_value = True
    return node


def test_existing_id_tables_have_correct_pairs():
    root = Path(__file__).resolve().parents[2]
    model = joint_command_model(root/'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf')
    folder = root/'dynamixel_joint_state_bridge/config'
    leader, follower = [dynamixel_mapping(model, yaml.safe_load((folder/name).read_text())['/**']['ros__parameters'])
                        for name in ('dynamixel_joint_state_bridge.yaml', 'dynamixel_joint_state_bridge_ids_31_52.yaml')]
    for side, first, second in [('R', 1, 31), ('L', 11, 41)]:
        for idx in range(7):
            name = f'{side}_joint{idx+1}'
            assert leader.entries[name][0] == first+idx
            assert follower.entries[name][0] == second+idx
            # 機体ごとの符号・原点校正。校正値の同一性ではなく角度換算の整合
            for mapping in (leader, follower):
                motor_id, scale, offset = mapping.entries[name]
                assert abs(scale) == 1. and math.isfinite(offset)
                ids, motor_deg = mapping.convert({name: .13})
                assert ids == [motor_id]
                assert (math.radians(motor_deg[0]) + math.radians(offset)) * scale == pytest.approx(.13)
    config = yaml.safe_load((root/'gng_vlut_system/config/dynamixel_leader_follower.yaml').read_text())['/**']['ros__parameters']
    assert len(config['joint_names']) == 14
    assert not config['allow_hardware_output'] and config['max_current_ma'] == 0.
    assert not any('neck' in name or 'gripper' in name for name in config['joint_names'])


def test_only_selected_leader_ids_are_published(leader_output):
    node = leader_output
    now = 1_000_000_000
    message = sample([17, 47, 51], now)
    message.position = [.5, 2., 3.]
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        node.on_leader(message)
        assert node.has_fresh_leader()
    state = node.leader_pub.publish.call_args.args[0]
    assert list(state.name) == ['L_joint7'] and list(state.position) == [-.5]
    node.send_goal.assert_not_called()
    node.torque_pub.publish.assert_not_called()


@pytest.fixture
def gripper_output(leader_output):
    node = leader_output
    node.leader_input_names = node.names+['R_gripper_joint', 'L_gripper_joint']
    node.leader_motor_names.update({'8': 'R_gripper_joint', '18': 'L_gripper_joint'})
    node.leader_mapping.entries.update({'R_gripper_joint': (8, -.015, 0.),
                                       'L_gripper_joint': (18, -.015, 0.)})
    node.leader_gripper_mapping = {
        'R_gripper_joint': gripper_input_mapping(0., math.pi/4, 0., 180.),
        'L_gripper_joint': gripper_input_mapping(0., math.pi/4, 0., -180.)}
    return node


def test_gripper_input_directions_and_no_real_gripper_target(gripper_output):
    node = gripper_output
    now = 1_000_000_000
    message = sample([17, 8, 18, 38, 48], now)
    message.position = [.1, math.pi/2, -math.pi, 2., 2.]
    message.velocity = [0., 1., -1., 0., 0.]
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        node.on_leader(message)
        node.config['enable_relative_follow'] = False
        assert node.leader_target() == {'L_joint7': -.1}
    state = node.leader_pub.publish.call_args.args[0]
    assert list(state.name) == node.leader_input_names
    assert list(state.position) == pytest.approx([-.1, math.pi/8, 0.])
    assert list(state.velocity) == pytest.approx([0., -.25, 0.])
    assert node.ids == [47]
    node.send_goal.assert_not_called()
    node.torque_pub.publish.assert_not_called()


def test_missing_or_stale_gripper_never_refreshes_complete_input(gripper_output):
    node = gripper_output
    now = 1_000_000_000
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        node.on_leader(sample([17, 8], now))
        node.leader_pub.publish.assert_not_called()
        node.on_leader(sample([18], now))
        node.leader_pub.publish.assert_called_once()
    node.leader_pub.publish.reset_mock()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now+400_000_000):
        node.on_leader(sample([17, 8], now+400_000_000))
        node.leader_pub.publish.assert_not_called()
        assert node.has_fresh_leader()


def test_nonfinite_gripper_sample_invalidates_publication(gripper_output):
    node = gripper_output
    now = 1_000_000_000
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        node.on_leader(sample([17, 8, 18], now))
    node.leader_pub.publish.reset_mock()
    invalid = sample([17, 8, 18], now+10_000_000)
    invalid.position = [0., math.nan, 0.]
    with patch('dynamixel_sim_output.time.time_ns', return_value=now+10_000_000):
        node.on_leader(invalid)
    node.leader_pub.publish.assert_not_called()
    assert 'R_gripper_joint' not in node.leader_stamps


def test_relative_follow_has_no_start_jump_and_uses_calibrated_sign(leader_output):
    node = leader_output
    now = 1_000_000_000
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        message = sample([17], now)
        message.position = [1.]
        node.on_leader(message)
        response = node.on_follow(SimpleNamespace(data=True), SimpleNamespace())
        assert response.success and node.mode == 'follow'
        assert node.leader_target() == node.measured
    with patch('dynamixel_sim_output.time.time_ns', return_value=now+10_000_000):
        message = sample([17], now+10_000_000)
        message.position = [1.02]
        node.on_leader(message)
        node.tick()
    node.model.step.assert_called_once()
    assert node.model.step.call_args.args[1]['L_joint7'] == pytest.approx(-.02)


def test_absolute_follow_requires_aligned_start(leader_output):
    node = leader_output
    node.config['enable_relative_follow'] = False
    now = 1_000_000_000
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        message = sample([17], now)
        message.position = [1.]
        node.on_leader(message)
        response = node.on_follow(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and '開始姿勢差' in response.message
    assert node.mode == 'hold'
    node.send_goal.assert_not_called()


@pytest.mark.parametrize('kind', ['missing', 'expired', 'replayed', 'future', 'nan', 'duplicate', 'wrong_frame'])
def test_bad_leader_latches_stop_and_never_resumes(leader_output, kind):
    node = leader_output
    now = 1_000_000_000
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        node.on_leader(sample([17], now))
        assert node.on_follow(SimpleNamespace(data=True), SimpleNamespace()).success
    message = sample([] if kind == 'missing' else [17], now+100_000_000)
    if kind == 'replayed':
        message = sample([17], now)
    elif kind == 'future':
        message = sample([17], now+500_000_000)
    elif kind == 'nan':
        message.position = [math.nan]
    elif kind == 'duplicate':
        message = sample([17, 17], now+100_000_000)
    elif kind == 'wrong_frame':
        message.header.frame_id = 'world'
    with patch('dynamixel_sim_output.time.time_ns', return_value=now+400_000_000):
        node.on_leader(message)
        node.tick()
        assert node.mode == 'stopped' and node.is_stop_latched
        assert 'リーダー' in node.detail
        node.model.step.assert_not_called()
        node.send_goal.assert_called_once_with(node.measured)
        node.on_leader(sample([17], now+400_000_000))
        node.tick()
        assert node.mode == 'stopped'


def test_leader_start_with_one_missing_joint_is_rejected(leader_output):
    node = leader_output
    node.names.append('R_joint7')
    now = 1_000_000_000
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        node.on_leader(sample([17], now))
        response = node.on_follow(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and node.mode == 'hold'
    node.leader_pub.publish.assert_not_called()


@pytest.mark.parametrize('enable_off, has_owned, is_latched', [
    (True, True, False), (True, False, True), (True, False, False), (False, True, False)])
def test_main_exit_torque_off_only_for_owned_or_latched_output(monkeypatch, enable_off, has_owned, is_latched):
    import dynamixel_sim_output as module
    node = Mock(config={'enable_torque_off_on_exit': enable_off},
                has_owned_output=has_owned, is_torque_off_latched=is_latched)
    node.has_torque_off_report.return_value = True
    monkeypatch.setattr(module, 'dynamixel_sim_output', Mock(return_value=node))
    monkeypatch.setattr(module.signal, 'signal', Mock())
    monkeypatch.setattr(module.rclpy, 'init', Mock())
    monkeypatch.setattr(module.rclpy, 'ok', Mock(return_value=True))
    monkeypatch.setattr(module.rclpy, 'spin_once', Mock(side_effect=KeyboardInterrupt))
    monkeypatch.setattr(module.rclpy, 'shutdown', Mock())
    module.main()
    if enable_off and (has_owned or is_latched):
        node.on_torque_off.assert_called_once()
        node.stop.assert_not_called()
    else:
        node.on_torque_off.assert_not_called()
        node.stop.assert_called_once()
    node.destroy_node.assert_called_once()
    module.rclpy.shutdown.assert_called_once()


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '97' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
                    reason='実機と分離したROS domain97・localhost限定の試験')
@pytest.mark.parametrize('enable_gripper_input', [False, True])
def test_ros_mock_driver_follow_and_stop(tmp_path, enable_gripper_input):
    """実ROS配信と疑似モータによる保持準備・追従・入力途絶の通し検証。"""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool, Trigger
    from dynamixel_handler_msgs.msg import DynamixelGoal, DynamixelStatus, DynamixelExtra
    from dynamixel_sim_output import dynamixel_sim_output

    root = Path(__file__).resolve().parents[2]
    config = yaml.safe_load((root/'gng_vlut_system/config/dynamixel_leader_follower.yaml').read_text())['/**']['ros__parameters']
    config.update(urdf_path=str(root/'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf'),
        mapping_file=str(root/'dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_ids_31_52.yaml'),
        leader_mapping_file=str(root/'dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge.yaml'),
        driver_namespace='/fixture/dynamixel', leader_driver_namespace='/fixture/dynamixel',
        target_source='leader', allow_hardware_output=True, max_current_ma=50.,
        enable_leader_gripper_input=enable_gripper_input)
    path = tmp_path/'fixture.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
    rclpy.init(args=['--ros-args', '--params-file', str(path)])
    driver = rclpy.create_node('leader_follower_fixture')
    node = None
    executor = SingleThreadedExecutor()
    state = {'goal': None, 'has_torque': False, 'enable_leader': True, 'delta': 0., 'events': []}
    try:
        assert not [name for name in driver.get_node_names() if name != driver.get_name()], 'domain97の既存ノード'
        node = dynamixel_sim_output()
        executor.add_node(node)
        executor.add_node(driver)
        prefix = '/fixture/dynamixel'
        from rclpy.qos import qos_profile_sensor_data
        fresh = driver.create_publisher(JointState, prefix+'/fresh_joint_states', qos_profile_sensor_data)
        pubs = {key: driver.create_publisher(kind, prefix+'/state/'+key, 1) for key, kind in
                [('goal', DynamixelGoal), ('status', DynamixelStatus), ('extra', DynamixelExtra)]}
        def on_goal(message):
            assert set(message.id_list) == set(range(31, 38)) | set(range(41, 48))
            assert all(0 < value <= 50. for value in message.current_ma)
            state['goal'] = message
            state['events'].append('goal')
        def on_torque(message):
            assert set(message.id_list) == set(node.ids)
            if any(message.torque):
                assert state['goal'] is not None
            state['has_torque'] = all(message.torque)
            state['events'].append('torque_on' if state['has_torque'] else 'torque_off')
        driver.create_subscription(DynamixelGoal, prefix+'/command/goal', on_goal, 1)
        driver.create_subscription(DynamixelStatus, prefix+'/command/status', on_torque, 1)
        received = []
        driver.create_subscription(JointState, node.config['leader_topic'], received.append, qos_profile_sensor_data)
        initial_ids, initial_angles = node.mapping.convert(dict.fromkeys(node.names, 0.))
        leader_ids, leader_angles = node.leader_mapping.convert(dict.fromkeys(node.names, 0.))
        def publish():
            follower = state['goal'] if state['has_torque'] else None
            ids = list(follower.id_list) if follower is not None else list(initial_ids)
            angles = list(follower.position_deg) if follower is not None else list(initial_angles)
            if state['enable_leader']:
                ids += leader_ids
                angles += [value+math.degrees(state['delta']) if motor_id == 1 else value
                           for motor_id, value in zip(leader_ids, leader_angles)]
                if enable_gripper_input:
                    ids += [8, 18]
                    angles += [90., -180.]
            msg = JointState(name=list(map(str, ids)), position=[math.radians(value) for value in angles], velocity=[0.]*len(ids))
            msg.header.frame_id = 'dynamixel_motor'
            msg.header.stamp = driver.get_clock().now().to_msg()
            fresh.publish(msg)
            pubs['status'].publish(DynamixelStatus(id_list=node.ids, torque=[state['has_torque']]*len(node.ids),
                error=[False]*len(node.ids), ping=[True]*len(node.ids), mode=['cur_position']*len(node.ids)))
            extra = DynamixelExtra(id_list=node.ids, model=['X']*len(node.ids), model_number=[1020]*len(node.ids))
            extra.drive_mode.profile_configuration = ['velocity_based']*len(node.ids)
            extra.drive_mode.torque_on_by_goal_update = [False]*len(node.ids)
            pubs['extra'].publish(extra)
            if state['goal'] is not None:
                pubs['goal'].publish(state['goal'])
            node.heartbeat_sec = time.monotonic()
        driver.create_timer(.02, publish)
        def wait(predicate, max_sec=5.):
            deadline = time.monotonic()+max_sec
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
                if predicate():
                    return
            raise AssertionError(node.detail)
        wait(lambda: node.has_fresh_state() and node.has_fresh_leader() and node.is_stationary() and received)
        assert len(received[-1].name) == (16 if enable_gripper_input else 14)
        if enable_gripper_input:
            positions = dict(zip(received[-1].name, received[-1].position))
            assert positions['R_gripper_joint'] == pytest.approx(math.pi/8, abs=1e-6)
            assert positions['L_gripper_joint'] == pytest.approx(0., abs=1e-6)
        assert state['events'] == []
        assert node.on_enable(SetBool.Request(data=True), SetBool.Response()).success
        wait(lambda: node.mode == 'hold')
        assert state['events'].index('goal') < state['events'].index('torque_on')
        assert node.on_follow(SetBool.Request(data=True), SetBool.Response()).success
        state['delta'] = .015
        wait(lambda: abs(node.measured['R_joint1']+.015) < .003)
        state['enable_leader'] = False
        wait(lambda: node.mode == 'stopped')
        assert node.is_stop_latched and 'リーダー' in node.detail
        state['enable_leader'] = True
        wait(lambda: node.has_fresh_leader())
        assert node.mode == 'stopped'
        node.on_torque_off(None, Trigger.Response())
        wait(lambda: node.has_torque_off_report())
        assert not state['has_torque']
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
