#!/usr/bin/env python3
"""隔離ROS・疑似サーボでの実キー操作、出力制限、欠測停止の有限検証。"""
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import JointTrajectoryControllerState
from dynamixel_handler_msgs.msg import DynamixelExtra, DynamixelGoal, DynamixelStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from check_dual_arm_control import pty_launch
from check_gazebo_software_stop import process_snapshot, save_json


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('隔離domain96・localhost限定が必要です')
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=False)
    baseline = process_snapshot()
    save_json(output/'baseline_processes.json', list(baseline.values()))
    launch = pty_launch(output, baseline)
    rclpy.init()
    node = rclpy.create_node('dynamixel_hardware_fixture')
    state = {'position': 0., 'velocity': 0., 'target': 0., 'torque': False, 'sim_target': 0.,
             'sim_stamp': 0, 'enable_clock': True, 'enable_feedback': True, 'status': {}, 'goals': [],
             'torque_commands': [], 'goal_echo': None, 'sim_stop_requests': 0,
             'enable_stamp': True, 'feedback_stamp': None, 'has_current_fault': False, 'driver_mode': 'cur_position'}
    driver, hw, sim = '/fixture/dynamixel', '/hw_ToPoDualArm', '/sim_ToPoDualArm'
    pubs = {name: node.create_publisher(kind, driver+'/state/'+name, 1) for name, kind in
            [('status', DynamixelStatus), ('extra', DynamixelExtra), ('goal', DynamixelGoal)]}
    measured = node.create_publisher(JointState, driver+'/fresh_joint_states', qos_profile_sensor_data)
    sim_target = node.create_publisher(JointTrajectoryControllerState, sim+'/dual_arm_controller/controller_state', qos_profile_sensor_data)
    sim_control = node.create_publisher(String, sim+'/control/status', 1)
    sim_safety = node.create_publisher(String, sim+'/safety/status', 1)
    node.create_subscription(String, hw+'/status', lambda msg: state.update(status=json.loads(msg.data)), 1)

    def on_goal(message):
        assert list(message.id_list) == [47], message.id_list
        assert message.profile_vel_deg_s and 0 < message.profile_vel_deg_s[0] <= 2.
        assert message.profile_acc_deg_ss and 0 < message.profile_acc_deg_ss[0] <= 30.
        assert message.current_ma and 0 < message.current_ma[0] <= 100.
        state['goals'].append(message.position_deg[0])
        state['target'] = math.radians(message.position_deg[0])
        state['goal_echo'] = message

    def on_torque(message):
        assert list(message.id_list) == [47] and len(message.torque) == 1
        if message.torque[0]:
            assert state['goal_echo'] is not None, '保持目標より前のtorque ON'
        state['torque_commands'].append(message)
        state['torque'] = message.torque[0]

    def on_sim_stop(_request, response):
        state['sim_stop_requests'] += 1
        response.success = True
        return response

    node.create_subscription(DynamixelGoal, driver+'/command/goal', on_goal, 1)
    node.create_subscription(DynamixelStatus, driver+'/command/status', on_torque, 1)
    node.create_service(Trigger, sim+'/control/stop', on_sim_stop)
    def publish():
        delta = max(-math.radians(1.374)*.02, min(math.radians(1.374)*.02, state['target']-state['position'])) if state['torque'] else 0.
        state['position'] += delta
        state['velocity'] = delta/.02
        if state['enable_feedback']:
            msg = JointState(name=['47'], position=[state['position']], velocity=[state['velocity']])
            if state['enable_stamp'] or state['feedback_stamp'] is None:
                state['feedback_stamp'] = node.get_clock().now().to_msg()
            msg.header.stamp = state['feedback_stamp']
            msg.header.frame_id = 'dynamixel_motor'
            measured.publish(msg)
        pubs['status'].publish(DynamixelStatus(id_list=[47], torque=[state['torque']], error=[False], ping=[True], mode=[state['driver_mode']]))
        extra = DynamixelExtra(id_list=[47], model=['X'], model_number=[1020])
        extra.drive_mode.profile_configuration = ['velocity_based']
        extra.drive_mode.torque_on_by_goal_update = [False]
        pubs['extra'].publish(extra)
        if state['goal_echo'] is not None:
            if state['has_current_fault']:
                state['goal_echo'].current_ma = [1000.]
            pubs['goal'].publish(state['goal_echo'])
        msg = JointTrajectoryControllerState(joint_names=['L_joint7'])
        if state['enable_clock']:
            state['sim_stamp'] += 20_000_000
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(state['sim_stamp'], 1_000_000_000)
        msg.reference.positions = [state['sim_target']]
        sim_target.publish(msg)
        sim_control.publish(String(data=json.dumps({'mode': 'hold'})))
        sim_safety.publish(String(data=json.dumps({'is_stop_latched': False})))
    node.create_timer(.02, publish)
    report = {'checks': {}, 'result': 'failed'}
    began = time.monotonic()
    def wait(predicate, max_sec=8.):
        deadline = time.monotonic()+max_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            launch.read_terminal()
            launch.track()
            if launch.process is not None and launch.process.poll() is not None:
                raise RuntimeError('launch途中終了')
            if predicate():
                return
            if time.monotonic()-began > 100:
                raise TimeoutError('試験全体期限')
        raise AssertionError(state['status'])
    def mode(value):
        return state['status'].get('mode') == value
    def key(value):
        launch.send_key(value)
    def reset_enable():
        key(b'r')
        wait(lambda: mode('off'))
        key(b'h')
        wait(lambda: mode('hold'))
    try:
        wait(lambda: time.monotonic()-began > .5)
        if any(name != node.get_name() for name in node.get_node_names()):
            raise RuntimeError('domain96に既存ノードあり')
        config = output/'hardware.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'driver_namespace': driver, 'max_current_ma': 100.}}}))
        command = ['ros2', 'launch', 'gng_vlut_system', 'dynamixel_sim_control.launch.py',
            'allow_hardware_output:=true', 'allow_sim_follow:=true', 'joint_names:=[L_joint7]', 'hardware_config:='+str(config)]
        launch.start(command)
        wait(lambda: mode('off') and state['status'].get('has_fresh_state'))
        assert not state['goals'] and not state['torque_commands']
        report['checks']['startup_no_output'] = True
        wait(lambda: state['status'].get('is_stationary'))
        key(b'h')
        wait(lambda: mode('hold'))
        assert len(state['torque_commands']) == 1
        report['checks']['goal_before_torque_enable'] = True
        key(b'j')
        wait(lambda: mode('jog'))
        wait(lambda: mode('hold'))
        assert abs(math.degrees(state['position'])+1.) < .2
        report['checks']['positive_jog_motor_deg'] = math.degrees(state['position'])
        key(b'k')
        wait(lambda: mode('jog'))
        key(b' ')
        wait(lambda: mode('stopped') and state['status'].get('is_stopped'))
        assert state['sim_stop_requests'] > 0
        key(b'j')
        wait(lambda: 'positive: 拒否' in launch.transcript)
        assert mode('stopped')
        report['checks']['space_stop_latched'] = dict(state['status'])
        reset_enable()
        state['sim_target'] = .2
        key(b'f')
        wait(lambda: '開始姿勢差' in launch.transcript)
        assert mode('hold')
        state['sim_target'] = -state['position']
        at = time.monotonic()
        wait(lambda: time.monotonic()-at > .2)
        key(b'f')
        wait(lambda: mode('follow'))
        state['sim_target'] += math.radians(.6)
        wait(lambda: abs(-state['position']-state['sim_target']) < math.radians(.15))
        report['checks']['gazebo_desired_follow'] = dict(state['status'])
        state['enable_clock'] = False
        wait(lambda: mode('stopped') and state['status'].get('is_stopped'))
        assert 'Gazebo' in state['status']['detail']
        report['checks']['paused_clock_stop'] = True
        state['enable_clock'] = True
        reset_enable()
        key(b'j')
        wait(lambda: mode('jog'))
        state['enable_feedback'] = False
        wait(lambda: mode('stopped'))
        assert not state['status']['is_stopped']
        state['enable_feedback'] = True
        wait(lambda: state['status']['is_stopped'])
        report['checks']['missing_feedback_unconfirmed_and_recovery'] = True
        reset_enable()
        keyboard = next(pid for pid, row in launch.track().items() if 'dynamixel_sim_keyboard.py --namespace' in row['command'])
        os.kill(keyboard, signal.SIGSTOP)
        try:
            wait(lambda: mode('stopped') and state['status'].get('is_stopped'))
            assert 'heartbeat' in state['status']['detail']
        finally:
            os.kill(keyboard, signal.SIGCONT)
        report['checks']['terminal_loss_stop'] = True
        wait(lambda: 'mode=stopped / 操作端末のheartbeat失効 / 実測停止=True' in launch.transcript)
        reset_enable()
        state['enable_stamp'] = False
        wait(lambda: mode('stopped'))
        assert not state['status']['is_stopped']
        state['enable_stamp'] = True
        wait(lambda: state['status']['is_stopped'])
        report['checks']['replayed_feedback_rejected'] = True
        reset_enable()
        anchor_deg = math.degrees(state['position'])
        for _ in range(4):
            key(b'j')
            wait(lambda: mode('jog'))
            wait(lambda: mode('hold'))
        # 現在姿勢からの追従開始と、有効化時姿勢を基準とした範囲超過目標の拒否
        state['sim_target'] = -state['position']
        at = time.monotonic()
        wait(lambda: time.monotonic()-at > .2)
        key(b'f')
        wait(lambda: mode('follow'))
        state['sim_target'] = math.radians(-anchor_deg+6.)
        wait(lambda: mode('stopped') and state['status']['is_stopped'])
        assert '試験範囲' in state['status']['detail']
        assert abs(math.degrees(state['position'])-anchor_deg) < 5.
        report['checks']['excursion_limit_stop'] = True
        reset_enable()
        key(b'j')
        wait(lambda: mode('jog'))
        state['enable_feedback'] = False
        key(b'e')
        wait(lambda: mode('torque_off') and state['status'].get('has_torque_off_report'))
        assert not state['torque']
        at = time.monotonic()
        wait(lambda: time.monotonic()-at > .4)
        assert not state['status']['is_stopped']
        num_goals = len(state['goals'])
        num_torque_on = sum(message.torque[0] for message in state['torque_commands'])
        key(b' ')
        at = time.monotonic()
        wait(lambda: time.monotonic()-at > .5)
        assert mode('torque_off') and len(state['goals']) == num_goals
        assert sum(message.torque[0] for message in state['torque_commands']) == num_torque_on
        state['enable_feedback'] = True
        wait(lambda: state['status']['is_stopped'])
        report['checks']['torque_off_missing_feedback_and_no_hold_resume'] = True
        key(b'r')
        wait(lambda: mode('off'))
        assert not state['torque']
        key(b'h')
        wait(lambda: mode('hold'))
        assert state['torque']
        report['checks']['torque_on_requires_reset_then_enable'] = True
        state['has_current_fault'] = True
        wait(lambda: mode('torque_off') and state['status'].get('has_torque_off_report'))
        assert not state['torque']
        assert '上限逸脱' in state['status']['detail']
        report['checks']['current_limit_readback_fault_torque_off'] = True
        state['has_current_fault'] = False
        wait(lambda: state['status']['is_stopped'])
        reset_enable()
        state['driver_mode'] = 'position'
        wait(lambda: mode('torque_off') and state['status'].get('has_torque_off_report'))
        assert not state['torque'] and '電流制御モード' in state['status']['detail']
        report['checks']['current_control_mode_loss_torque_off'] = True
        report['result'] = 'passed'
    except BaseException as error:
        report['error'] = repr(error)
    finally:
        report['cleanup'] = launch.cleanup()
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.get_node_names() == [node.get_name()]:
                break
        report['remaining_nodes'] = [name for name in node.get_node_names() if name != node.get_name()]
        report['cleanup']['is_success'] &= not report['remaining_nodes'] and launch.is_terminal_restored()
        report['last_status'] = state['status']
        save_json(output/'report.json', report)
        node.destroy_node()
        rclpy.shutdown()
        launch.close_terminal()
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return int(report['result'] != 'passed' or not report['cleanup']['is_success'])


if __name__ == '__main__':
    raise SystemExit(main())
