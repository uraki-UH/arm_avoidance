#!/usr/bin/env python3
"""実機出力OFFでのGazebo・操作端末一括起動と、両系統の停止要求の有限試験。"""
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from check_dual_arm_control import pty_launch
from check_gazebo_software_stop import port_is_listening, process_snapshot, save_json


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('隔離domain96・localhost限定が必要です')
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=False)
    baseline = process_snapshot()
    save_json(output/'baseline_processes.json', list(baseline.values()))
    launch = pty_launch(output, baseline)
    rclpy.init()
    node = rclpy.create_node('dynamixel_gazebo_start_check')
    state = {'hardware': {}, 'sim': {}, 'safety': {}, 'demo': {}, 'num_samples': 0}
    for name, topic in [('hardware', '/hw_ToPoDualArm/status'), ('sim', '/sim_ToPoDualArm/control/status'),
                        ('safety', '/sim_ToPoDualArm/safety/status'), ('demo', '/sim_ToPoDualArm/avoidance/status')]:
        node.create_subscription(String, topic, lambda msg, key=name: state.update({key: json.loads(msg.data)}), 1)
    def on_controller(message):
        if 'L_joint7' in message.joint_names and message.reference.positions:
            state.update(num_samples=state['num_samples']+1,
                positions=dict(zip(message.joint_names, message.feedback.positions)),
                max_velocity=max(map(abs, message.feedback.velocities)),
                stamp_sec=message.header.stamp.sec+message.header.stamp.nanosec*1e-9)
    node.create_subscription(JointTrajectoryControllerState, '/sim_ToPoDualArm/dual_arm_controller/controller_state',
        on_controller, qos_profile_sensor_data)
    joint_samples = []
    def on_joint(message):
        joint_samples.append({'stamp_sec': message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            'wall_sec': time.monotonic(), 'phase': state['sim'].get('phase'),
            'velocities': dict(zip(message.name, message.velocity))})
    node.create_subscription(JointState, '/sim_ToPoDualArm/joint_states', on_joint, qos_profile_sensor_data)
    def wait(predicate, max_sec=70.):
        deadline = time.monotonic()+max_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            launch.read_terminal()
            launch.track()
            if launch.process is not None and launch.process.poll() is not None:
                raise RuntimeError('launch途中終了')
            if predicate():
                return
        raise TimeoutError(state)
    report = {'result': 'failed'}
    try:
        began = time.monotonic()
        wait(lambda: time.monotonic()-began > .5)
        if port_is_listening() or node.get_node_names() != [node.get_name()]:
            raise RuntimeError('試験domainまたはGazeboポートが使用中')
        launch.start(['ros2', 'launch', 'gng_vlut_system', 'dynamixel_sim_control.launch.py',
            'enable_gazebo:=true', 'gui:=false', 'gazebo_master_uri:=http://127.0.0.1:11369'])
        wait(lambda: state['sim'].get('mode') == 'hold' and state['num_samples'] > 5 and state['hardware'].get('mode') == 'off')
        assert node.count_publishers('/dynamixel/command/goal') == 0
        assert node.count_publishers('/dynamixel/command/status') == 0
        launch.send_key(b'h')
        wait(lambda: '監視モード' in launch.transcript, 5.)
        wait(lambda: state['demo'].get('state') == 'idle' and '回避=開始待ち' in launch.transcript)
        # 前方斜め下45度への伸展直後の重力応答を含む初期整定の確認
        settle_stamp = state['stamp_sec']
        wait(lambda: state['stamp_sec']-settle_stamp > 1. and state['max_velocity'] < .01)
        initial = dict(state['positions'])
        assert abs(initial['L_joint1'] + math.pi/4) < .02, initial
        assert max(abs(value) for name, value in initial.items() if name.startswith('L_joint') and name != 'L_joint1') < .02, initial
        report['initial_positions'] = initial
        launch.send_key(b'a')
        wait(lambda: state['sim'].get('mode') == 'avoidance', 10.)
        wait(lambda: max(abs(state['positions'][name]-initial[name]) for name in initial if name.startswith('L_joint')) > .1)
        report['max_left_motion_rad'] = max(abs(state['positions'][name]-initial[name]) for name in initial if name.startswith('L_joint'))
        report['max_right_motion_rad'] = max(abs(state['positions'][name]-initial[name]) for name in initial if name.startswith('R_joint'))
        report['max_left_velocity_rad_sec'] = max(abs(value) for sample in joint_samples
            for name, value in sample['velocities'].items() if name.startswith('L_joint'))
        launch.send_key(b' ')
        wait(lambda: state['sim'].get('mode') == 'stopped' and state['hardware'].get('is_stop_latched'), 10.)
        wait(lambda: state['safety'].get('is_stopped') is True, 10.)
        at = time.monotonic()
        wait(lambda: time.monotonic()-at > .5, 2.)
        launch.send_key(b'b')
        wait(lambda: state['sim'].get('mode') == 'hold', 10.)
        report.update(result='passed', num_jtc_samples=state['num_samples'], has_hardware_publishers=False,
                      has_space_stop=True, has_gazebo_reset=True)
    except BaseException as error:
        report['error'] = repr(error)
    finally:
        report['cleanup'] = launch.cleanup()
        save_json(output/'joint_samples.json', joint_samples)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.get_node_names() == [node.get_name()]:
                break
        report['remaining_nodes'] = [name for name in node.get_node_names() if name != node.get_name()]
        report['cleanup']['is_success'] &= not report['remaining_nodes'] and not port_is_listening() and (launch.process is None or launch.is_terminal_restored())
        save_json(output/'report.json', report)
        node.destroy_node()
        rclpy.shutdown()
        launch.close_terminal()
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return int(report['result'] != 'passed' or not report['cleanup']['is_success'])


if __name__ == '__main__':
    raise SystemExit(main())
