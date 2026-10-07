#!/usr/bin/env python3
"""隔離Harmonicによる移動・中断・手動再開・保持・経由点・取消の有限試験。"""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

from check_gazebo_software_stop import owned_launch, process_snapshot, save_json


class task_trial(Node):
    def __init__(self, launch):
        super().__init__('task_program_check', namespace='sim_task_check')
        self.launch = launch
        self.status = {}
        self.positions, self.velocities = {}, {}
        self.transitions = []
        self.create_subscription(String, 'task_executor/status', self.on_status, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, qos_profile_sensor_data)
        self.command_clients = {name: self.create_client(Trigger, 'task_executor/' + name) for name in ('start', 'resume', 'cancel')}
        self.command_clients['obstacle'] = self.create_client(SetBool, 'task_executor/obstacle')

    def on_status(self, message):
        value = json.loads(message.data)
        marker = (value['state'], value['task_idx'], value['waypoint_idx'])
        if not self.transitions or marker != tuple(self.transitions[-1]['marker']):
            self.transitions.append({'marker': marker, 'status': value})
        self.status = value

    def on_joints(self, message):
        self.positions = dict(zip(message.name, message.position))
        self.velocities = dict(zip(message.name, message.velocity))

    def wait(self, predicate, label, duration=90.0):
        deadline = time.monotonic() + duration
        tracked_sec = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
            if self.status.get('state') == 'failed':
                raise AssertionError(self.status)
            if self.launch.process.poll() is not None:
                raise AssertionError('launch終了: ' + label)
            if time.monotonic() - tracked_sec > 1.0:
                self.launch.track()
                tracked_sec = time.monotonic()
        raise TimeoutError(label + ': ' + str(self.status))

    def call(self, name, data=None, allow_failure=False):
        client = self.command_clients[name]
        self.wait(client.service_is_ready, name + '準備')
        request = SetBool.Request(data=data) if name == 'obstacle' else Trigger.Request()
        future = client.call_async(request)
        self.wait(future.done, name + '応答', duration=5.0)
        response = future.result()
        if not allow_failure and not response.success:
            raise AssertionError(name + ': ' + response.message)
        return response

    def run(self):
        self.wait(lambda: self.status.get('state') == 'idle' and bool(self.positions), '初期化')
        self.wait(lambda: self.call('start', allow_failure=True).success, '停止姿勢での開始')
        self.wait(lambda: self.status.get('state') == 'running' and self.status.get('active_sec', 0) > 1.0
                  and abs(self.velocities.get('L_joint4', 0)) > 0.06, '軌道途中の実測運動')
        moving = dict(self.status)
        self.call('obstacle', True)
        self.wait(lambda: self.status.get('state') == 'paused', '障害物中断と実測停止')
        paused = dict(self.status)
        stopped = dict(self.velocities)
        if max(abs(value) for value in stopped.values()) > 0.03:
            raise AssertionError('停止判定と実測の不一致')
        assert not self.call('resume', allow_failure=True).success
        self.call('obstacle', False)
        self.wait(lambda: self.status.get('state') == 'paused' and not self.status.get('has_obstacle'), '自動再開なし')
        self.call('resume')
        self.wait(lambda: self.status.get('state') == 'succeeded', '全タスク完了', duration=150.0)
        home = {name: self.positions[name] for name in ('L_joint4', 'R_joint4')}
        assert max(abs(value) for value in home.values()) < 0.025, home
        self.call('start')
        self.wait(lambda: self.status.get('state') == 'running' and self.status.get('active_sec', 0) > 1.0
                  and abs(self.velocities.get('L_joint4', 0)) > 0.06, '取消試験の軌道途中')
        self.call('cancel')
        self.wait(lambda: self.status.get('state') == 'canceled', '取消と実測停止')
        assert not self.call('resume', allow_failure=True).success
        return dict(is_success=True, moving=moving, paused=paused, stopped_velocities=stopped,
                    home_positions=home, transitions=self.transitions)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if not os.environ.get('GZ_PARTITION', '').startswith('task_check_') or os.environ.get('ROS_DOMAIN_ID') != '96':
        parser.error('専用GZ_PARTITION=task_check_*とROS_DOMAIN_ID=96が必要です')
    args.output.mkdir(parents=True, exist_ok=False)
    launch = owned_launch(args.output, process_snapshot())
    report, node = {'is_success': False}, None
    rclpy.init()
    try:
        command = ['ros2', 'launch', 'gng_vlut_system', 'dual_arm_tasks.launch.py',
                   'namespace:=sim_task_check']
        if os.environ.get('ROS_DISTRO') != 'humble':
            command.append('output_dir:=' + str(args.output / 'generated'))
        launch.start(command)
        node = task_trial(launch)
        report.update(node.run())
    except Exception as error:
        report['error'] = str(error)
        if node is not None:
            report['transitions'] = node.transitions
            report['last_positions'] = node.positions
            report['last_velocities'] = node.velocities
        raise
    finally:
        report['cleanup'] = launch.cleanup()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        report['is_success'] = report['is_success'] and report['cleanup']['is_success']
        save_json(args.output / 'report.json', report)
        print(json.dumps(report, ensure_ascii=False))
    if not report['is_success']:
        raise RuntimeError('試験または片付けの失敗')


if __name__ == '__main__':
    main()
