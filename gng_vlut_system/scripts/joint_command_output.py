#!/usr/bin/env python3
"""統合済み関節指令のViewer・Gazebo・Dynamixel出力。"""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from joint_command_model import joint_command_model, dynamixel_mapping


class joint_command_output(Node):
    def __init__(self):
        super().__init__('joint_command_output')
        defaults = {'backend': 'viewer', 'urdf_path': '', 'target_topic': 'active_joint_commands',
                    'state_topic': 'joint_states', 'viewer_topic': 'viewer_joint_states',
                    'trajectory_topic': 'dual_arm_controller/joint_trajectory',
                    'dynamixel_topic': '/dynamixel/command/goal', 'mapping_file': '',
                    'publish_hz': 50.0, 'max_joint_velocity': 0.6,
                    'max_state_age_sec': 1.0, 'max_command_age_sec': 0.5,
                    'trajectory_duration_sec': 0.05, 'enable_direct_tracking': False}
        self.config = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        self.backend = self.config['backend']
        if self.backend not in ('viewer', 'gazebo', 'dynamixel'):
            raise ValueError('backendはviewer・gazebo・dynamixelのいずれかが必要です')
        for name in ('publish_hz', 'max_joint_velocity', 'max_state_age_sec',
                     'max_command_age_sec', 'trajectory_duration_sec'):
            if not math.isfinite(self.config[name]) or self.config[name] <= 0:
                raise ValueError('正の有限値が必要です: '+name)
        if (self.backend != 'viewer' and
                self.resolve_topic_name(self.config['state_topic']) == self.resolve_topic_name(self.config['viewer_topic'])):
            raise ValueError('実測入力とViewer出力には別のトピックが必要です')
        self.model = joint_command_model(self.config['urdf_path'])
        self.targets, self.measured, self.received = {}, {}, {}
        self.commanded = self.model.initial_positions() if self.backend == 'viewer' else {}
        self.controlled_names, self.previous_active_names = set(), set()
        self.last_command_sec = None
        self.last_tick_sec = None
        self.has_output = False
        self.last_status = None
        self.mapping = None
        self.goal_type = None
        self.command_pub = None
        if self.backend == 'gazebo':
            self.command_pub = self.create_publisher(JointTrajectory, self.config['trajectory_topic'], 1)
        elif self.backend == 'dynamixel':
            from dynamixel_handler_msgs.msg import DynamixelGoal
            self.goal_type = DynamixelGoal
            mapping_config = yaml.safe_load(Path(self.config['mapping_file']).read_text())['/**']['ros__parameters']
            self.mapping = dynamixel_mapping(self.model, mapping_config)
            self.command_pub = self.create_publisher(DynamixelGoal, self.config['dynamixel_topic'], 1)
        self.viewer_pub = self.create_publisher(JointState, self.config['viewer_topic'], 10)
        self.status_pub = self.create_publisher(String, 'joint_control/status', 10)
        self.create_subscription(JointState, self.config['target_topic'], self.on_command, 10)
        self.create_subscription(JointState, self.config['state_topic'], self.on_state, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0/self.config['publish_hz'], self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def report(self, state, detail=''):
        value = (state, detail)
        if value != self.last_status:
            self.get_logger().info(state+(': '+detail if detail else ''))
            self.last_status = value
        msg = String()
        msg.data = json.dumps({'backend': self.backend, 'state': state, 'detail': detail,
                               'active_joints': sorted(self.targets)}, ensure_ascii=False)
        self.status_pub.publish(msg)

    def on_command(self, msg):
        try:
            targets = self.model.canonical_positions(msg.name, msg.position)
            if self.mapping is not None and set(targets)-self.mapping.covered_names:
                raise ValueError('Dynamixel ID未割当の関節を含む指令です')
        except ValueError as exc:
            self.report('rejected', str(exc))
            return
        self.targets = targets
        self.last_command_sec = time.monotonic()

    def on_state(self, msg):
        try:
            values = self.model.feedback_positions(msg.name, msg.position)
        except ValueError as exc:
            self.report('invalid_state', str(exc))
            return
        received = time.monotonic()
        self.measured.update(values)
        self.received.update({name: received for name in values})
        if self.backend == 'viewer' and not self.has_output:
            self.commanded.update(values)
        if self.backend != 'viewer':
            # Viewerへの配信元はフォロワー側の実測値のみ
            self.publish_viewer(self.model.expand(values), msg.header.stamp)

    def publish_viewer(self, values, stamp=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg() if stamp is None else stamp
        msg.name = list(values)
        msg.position = list(values.values())
        self.viewer_pub.publish(msg)

    def tick(self):
        wall_sec = time.monotonic()
        clock_sec = self.get_clock().now().nanoseconds*1e-9
        duration = 0.0 if self.last_tick_sec is None else clock_sec-self.last_tick_sec
        self.last_tick_sec = clock_sec
        if duration <= 0:
            return
        duration = min(duration, 2.0/self.config['publish_hz'])
        if self.last_command_sec is None or wall_sec-self.last_command_sec > self.config['max_command_age_sec']:
            self.targets = {}
        targets = self.targets
        active_names = set(targets)
        required_names = (set(self.model.independent_names) if self.backend == 'gazebo'
                          else self.controlled_names | active_names)
        if self.backend != 'viewer':
            if not required_names or not (active_names or self.has_output):
                self.report('waiting_command')
                return
            missing = [name for name in required_names if name not in self.received or
                       wall_sec-self.received[name] > self.config['max_state_age_sec']]
            if missing:
                self.commanded.clear()
                self.previous_active_names.clear()
                self.report('waiting_state', ','.join(sorted(missing)))
                return
            for name in required_names:
                if name not in self.commanded:
                    self.commanded[name] = self.measured[name]
            # 入力失効時の目標は実測姿勢。古い到達目標への進行を停止
            for name in self.previous_active_names-active_names:
                self.commanded[name] = self.measured[name]
        self.commanded = self.model.step(self.commanded, targets, duration,
                                         self.config['max_joint_velocity'],
                                         self.backend == 'viewer' and self.config['enable_direct_tracking'])
        self.controlled_names.update(active_names)
        self.previous_active_names = active_names
        if self.backend == 'viewer':
            self.publish_viewer(self.model.expand(self.commanded))
        elif self.backend == 'gazebo':
            msg = JointTrajectory()
            msg.joint_names = self.model.independent_names
            point = JointTrajectoryPoint()
            point.positions = [self.commanded[name] for name in msg.joint_names]
            duration_ns = int(self.config['trajectory_duration_sec']*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(duration_ns, 10**9)
            msg.points = [point]
            self.command_pub.publish(msg)
        else:
            msg = self.goal_type()
            ids, positions = self.mapping.convert({name: self.commanded[name] for name in self.controlled_names})
            msg.id_list, msg.position_deg = ids, positions
            if ids:
                self.command_pub.publish(msg)
        self.has_output = True
        self.report('tracking' if active_names else 'holding')


def main():
    rclpy.init()
    node = None
    try:
        node = joint_command_output()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
