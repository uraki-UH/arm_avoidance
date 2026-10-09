"""静的重力計算・両腕ID隔離・出力許可・電流上限の実機非接続試験。"""
from pathlib import Path
import math
import os
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import JointState
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dynamixel_hand_guiding as module
import dynamixel_neck_torque as core
from gravity_compensation import gravity_model

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf'
MAPPING = ROOT / 'dynamixel_joint_state_bridge/config/dynamixel_joint_state_bridge_ids_31_52.yaml'


@pytest.fixture
def gravity():
    return gravity_model(URDF, module.hand_guiding.names)


@pytest.mark.parametrize('seed', range(5))
def test_torque_equals_potential_gradient(gravity, seed):
    positions = dict(zip(gravity.names, np.random.default_rng(seed).uniform(-.4, .4, len(gravity.names))))
    torques, _ = gravity.evaluate(positions)
    for name in gravity.names:
        left, right = dict(positions), dict(positions)
        left[name] -= 1e-5
        right[name] += 1e-5
        difference = (gravity.evaluate(right)[1] - gravity.evaluate(left)[1]) / 2e-5
        assert torques[name] == pytest.approx(difference, abs=1e-7)


def test_other_arm_does_not_change_torque(gravity):
    baseline, _ = gravity.evaluate({})
    moved, _ = gravity.evaluate({f'L_joint{idx}': .3 for idx in range(1, 8)})
    for name in gravity.names[:7]:
        assert moved[name] == pytest.approx(baseline[name])
    assert any(abs(moved[name] - baseline[name]) > .01 for name in gravity.names[7:])


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_nonfinite_angle_rejected(gravity, bad):
    with pytest.raises(ValueError):
        gravity.evaluate({'R_joint1': bad})


@pytest.fixture
def rig(monkeypatch, gravity):
    clock = SimpleNamespace(now=100.)
    monkeypatch.setattr(core.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(core.time, 'time_ns', lambda: int(clock.now * 1e9))
    node = module.hand_guiding.__new__(module.hand_guiding)
    node.model = module.joint_command_model(URDF)
    node.mapping = module.dynamixel_mapping(node.model, yaml.safe_load(MAPPING.read_text())['/**']['ros__parameters'])
    node.gravity = gravity
    node.driver = '/dynamixel'
    node.allow_hardware_output = True
    node.max_current = [1000.] * 14
    node.torque_nm_per_ma = [.01] * 14
    node.gains = [.02] * 14
    node.gravity_ramp_sec = 2.
    node.max_velocity_th = .5
    node.max_start_velocity_th = .02
    node.enable_gravity_compensation = True
    node.has_verified_calibration = True
    node.report_sec = 100.
    node.state, node.stage_sec, node.has_owned_output = 'waiting', 100., False
    node.joints = {}
    for name, motor_id in zip(node.names, node.ids):
        _, scale, offset = node.mapping.entries[name]
        node.joints[motor_id] = (100_000_000_000, 0., -math.radians(offset))
    node.status = {motor_id: (False, False, True, 'current') for motor_id in node.ids}
    node.extra = {motor_id: (1020 if motor_id % 2 else 1120, False, False) for motor_id in node.ids}
    node.goals = {motor_id: (2000.,) for motor_id in node.ids}
    node.status_sec = node.extra_sec = node.goal_sec = 100.
    node.goal_pub, node.torque_pub, node.report_pub = Mock(), Mock(), Mock()
    node.goal_pub.get_subscription_count.return_value = node.torque_pub.get_subscription_count.return_value = 1
    node.get_topic_names_and_types = lambda: [(node.driver + '/command/goal', []), (node.driver + '/command/status', [])]
    node.count_publishers = lambda _: 1
    node.get_logger = Mock()
    return node, clock


def test_gravity_sign_motor_mapping_ramp_and_no_return_goal(rig):
    node, _ = rig
    torques, currents = node.support_currents()
    assert node.control_currents(100.) == [0.] * 14
    for idx, name in enumerate(node.names):
        _, scale, _ = node.mapping.entries[name]
        raw = scale * torques[name] / node.torque_nm_per_ma[idx]
        assert abs(currents[idx] - raw) < 2.69
        assert currents[idx] * raw >= 0
    assert node.control_currents(102.) == currents
    assert node.control_currents(105.) == currents
    node.joints[31] = (100_000_000_000, .1, node.joints[31][2])
    assert node.support_currents()[1][0] <= currents[0]


@pytest.mark.parametrize('fault', ['range', 'speed', 'limit', 'start_speed', 'mode', 'model', 'missing', 'stale', 'on', 'conflict'])
def test_unsafe_start_never_writes(rig, fault):
    node, clock = rig
    if fault == 'range':
        node.joints[32] = (100_000_000_000, 0., 100.)
    elif fault in ('speed', 'start_speed'):
        node.joints[31] = (100_000_000_000, 1. if fault == 'speed' else .03, node.joints[31][2])
    elif fault == 'limit':
        node.max_current = [2.69] * 14
    elif fault in ('mode', 'on'):
        node.status[31] = (fault == 'on', False, True, 'position' if fault == 'mode' else 'current')
    elif fault == 'model':
        node.extra[31] = (999, False, False)
    elif fault == 'missing':
        node.joints.pop(31)
        clock.now = 113.
    elif fault == 'stale':
        clock.now = 113.
    elif fault == 'conflict':
        node.count_publishers = lambda _: 2
    with pytest.raises(ValueError):
        node.step()
    assert not node.has_owned_output
    node.goal_pub.publish.assert_not_called()
    node.torque_pub.publish.assert_not_called()


def test_fourteen_only_zero_readback_then_on_and_no_position_commands(rig):
    node, clock = rig
    node.step()
    assert node.state == 'zero'
    node.torque_pub.publish.assert_not_called()
    clock.now = node.goal_sec = 100.05
    node.goals = {motor_id: (0.,) for motor_id in node.ids}
    node.step()
    assert node.state == 'enable'
    assert list(node.torque_pub.publish.call_args.args[0].torque) == [True] * 14
    clock.now = node.status_sec = 100.1
    node.status = {motor_id: (True, False, True, 'current') for motor_id in node.ids}
    node.step()
    assert node.state == 'running'
    node.step()
    for call in node.goal_pub.publish.call_args_list:
        message = call.args[0]
        assert tuple(message.id_list) == node.ids
        assert not message.position_deg and not message.velocity_deg_s and not message.pwm_percent
    assert not set(node.ids) & {38, 48, 51, 52}


@pytest.mark.parametrize('kind', ['nan', 'duplicate', 'frame'])
def test_invalid_measurement_invalidates_cache(rig, kind):
    node, _ = rig
    message = JointState(name=['31', '32'], position=[0., 0.], velocity=[0., 0.])
    message.header.frame_id = 'dynamixel_motor'
    message.header.stamp.sec = 100
    if kind == 'nan':
        message.position[0] = math.nan
    elif kind == 'duplicate':
        message.name = ['31', '31']
    else:
        message.header.frame_id = 'world'
    node.on_joints(message)
    assert not node.has_fresh_state()


def test_default_config_is_preview_and_uncalibrated():
    config = yaml.safe_load((ROOT / 'gng_vlut_system/config/dynamixel_hand_guiding.yaml').read_text())['/**']['ros__parameters']
    assert config['allow_hardware_output'] is False
    assert config['has_verified_calibration'] is False
    assert config['max_current_ma'] == config['torque_nm_per_ma'] == [0.] * 14


@pytest.mark.parametrize('allow_output', [False, True])
def test_constructor_preview_has_no_command_publishers_or_uncalibrated_refused(allow_output):
    """実通信から隔離したconstructor試験。実機指令publisherの不在。"""
    rclpy.init(args=['--ros-args', '-p', 'urdf_path:=' + str(URDF), '-p', 'mapping_file:=' + str(MAPPING),
                     '-p', 'allow_hardware_output:=' + str(allow_output).lower()])
    node = None
    try:
        if allow_output:
            with pytest.raises(ValueError, match='校正確認'):
                module.hand_guiding()
        else:
            node = module.hand_guiding()
            assert node.goal_pub is node.torque_pub is None
            assert not node.has_owned_output
            node.step()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_shutdown_all_selected_ids_without_touching_neck(rig, monkeypatch):
    node, clock = rig
    node.has_owned_output = True
    monkeypatch.setattr(core.rclpy, 'ok', lambda: True)
    def spin(*_args, **_kwargs):
        clock.now += .1
        node.status = {id_value: (False, False, True, 'current') for id_value in node.ids}
        node.status_sec = clock.now
        node.joints = {id_value: (int(clock.now * 1e9), 0., 0.) for id_value in node.ids}
    monkeypatch.setattr(core.rclpy, 'spin_once', spin)
    assert node.stop_output()
    for call in node.torque_pub.publish.call_args_list:
        assert tuple(call.args[0].id_list) == node.ids
        assert list(call.args[0].torque) == [False] * 14


def test_gravity_matches_mujoco_static_bias():
    """独立した物理ライブラリとの静的支持トルク照合。"""
    pytest.importorskip('mujoco')
    simulator = ROOT / 'ToPoDualArmMax_SourceDelivery_20260928/ToPoDualArmMax-Simulator'
    sys.path.insert(0, str(simulator / 'integrations/ros2'))
    from physics_scene import PhysicsScene
    positions = {name: .2 if idx % 2 else -.3 for idx, name in enumerate(module.hand_guiding.names)}
    scene = PhysicsScene([], dict(model='long', position=[0, 0, 0], quaternion=[0, 0, 0, 1], pose=positions))
    gravity = gravity_model(simulator / 'app/source.urdf', module.hand_guiding.names, scene.model.opt.gravity)
    torques, _ = gravity.evaluate(positions)
    for name in gravity.names:
        idx = scene.model.jnt_dofadr[scene.robot.ids[name]]
        assert torques[name] == pytest.approx(scene.data.qfrc_bias[idx], abs=1e-7)


def test_mock_driver_current_follow_and_fault_shutdown():
    """隔離domain96限定の疑似handler試験。実機通信・位置指令なし。"""
    if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('疑似指令試験には隔離domain96・localhost限定が必要')
    from dynamixel_handler_msgs.msg import DynamixelExtra, DynamixelGoal, DynamixelStatus
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    args = ['--ros-args', '-p', 'urdf_path:=' + str(URDF), '-p', 'mapping_file:=' + str(MAPPING),
            '-p', 'allow_hardware_output:=true', '-p', 'has_verified_calibration:=true',
            '-p', 'max_current_ma:=' + str([1000.] * 14), '-p', 'torque_nm_per_ma:=' + str([.01] * 14)]
    rclpy.init(args=args)
    driver = Node('hand_guiding_mock_driver')
    node, executor = None, SingleThreadedExecutor()
    ids = list(module.hand_guiding.ids)
    state = {'torque': [False] * 14, 'current': [2000.] * 14, 'enable_fresh': True}
    events = []
    fresh = driver.create_publisher(JointState, '/dynamixel/fresh_joint_states', qos_profile_sensor_data)
    status = driver.create_publisher(DynamixelStatus, '/dynamixel/state/status', 1)
    extra = driver.create_publisher(DynamixelExtra, '/dynamixel/state/extra', 1)
    goal = driver.create_publisher(DynamixelGoal, '/dynamixel/state/goal', 1)
    def on_goal(message):
        assert list(message.id_list) == ids
        assert not message.position_deg and not message.velocity_deg_s
        state['current'] = list(message.current_ma)
        events.append(('current', list(message.current_ma)))
    def on_torque(message):
        assert list(message.id_list) == ids
        assert not message.mode
        state['torque'] = list(message.torque)
        events.append(('torque', list(message.torque)))
    driver.create_subscription(DynamixelGoal, '/dynamixel/command/goal', on_goal, 1)
    driver.create_subscription(DynamixelStatus, '/dynamixel/command/status', on_torque, 1)
    try:
        node = module.hand_guiding()
        def publish():
            if state['enable_fresh']:
                q = [-math.radians(node.mapping.entries[name][2]) for name in node.names]
                message = JointState(name=list(map(str, ids)), position=q, velocity=[0.] * 14)
                message.header.frame_id = 'dynamixel_motor'
                message.header.stamp = driver.get_clock().now().to_msg()
                fresh.publish(message)
            status.publish(DynamixelStatus(id_list=ids, torque=state['torque'], error=[False] * 14,
                                           ping=[True] * 14, mode=['current'] * 14))
            message = DynamixelExtra(id_list=ids, model_number=[1020] * 14)
            message.drive_mode.torque_on_by_goal_update = [False] * 14
            message.drive_mode.reverse_mode = [False] * 14
            extra.publish(message)
            goal.publish(DynamixelGoal(id_list=ids, current_ma=state['current']))
        driver.create_timer(.01, publish)
        executor.add_node(driver)
        executor.add_node(node)
        assert not events
        end = time.monotonic() + 5.
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            node.step()
            if node.state == 'running' and any(abs(v) > 2.69 for v in state['current']):
                break
        assert node.state == 'running' and any(abs(v) > 2.69 for v in state['current'])
        first_on = next(idx for idx, event in enumerate(events) if event == ('torque', [True] * 14))
        assert any(event == ('current', [0.] * 14) for event in events[:first_on])
        state['enable_fresh'] = False
        end = time.monotonic() + .25
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        with pytest.raises(ValueError, match='失効'):
            node.step()
        # 終了確認中のcallback更新。実機ではなく疑似状態の配信のみ
        state['enable_fresh'] = True
        executor.remove_node(node)
        executor.remove_node(driver)
        monkey_spin = core.rclpy.spin_once
        try:
            def spin(_node, timeout_sec):
                executor.add_node(node)
                executor.add_node(driver)
                executor.spin_once(timeout_sec=timeout_sec)
                executor.remove_node(node)
                executor.remove_node(driver)
            core.rclpy.spin_once = spin
            assert node.stop_output()
        finally:
            core.rclpy.spin_once = monkey_spin
        assert state['torque'] == [False] * 14
        assert state['current'] == [0.] * 14
    finally:
        if node is not None:
            node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
